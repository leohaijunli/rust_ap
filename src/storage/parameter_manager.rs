//! Shared parameter API; only this layer owns FRAM and acknowledges persistence.

use core::cell::{Cell, RefCell};

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use v6x_rust_led::parameter_store::{
    self as store, Active, Commit, CommitError, DecodeError, OutputState, Slot,
};
use v6x_rust_led::parameters::{self, Id, Parameters};

use crate::fram::{self, Fram};

/// Ground stations write parameter batches one value at a time. Each commit
/// rewrites and verifies the whole record over SPI, so wait for the writes to
/// go quiet and persist the batch once instead of stalling per value.
const SAVE_POLL: Duration = Duration::from_millis(100);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Fram(fram::Error),
    OutputsActive,
    Unavailable,
    Readback,
    /// More values differ from their defaults than one record can key.
    Capacity,
}

impl From<fram::Error> for Error {
    fn from(error: fram::Error) -> Self {
        Self::Fram(error)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadState {
    Unloaded,
    Stored,
    Defaults,
    Unavailable,
}

#[derive(Clone, Copy)]
pub struct Status {
    pub load: LoadState,
    pub active: Option<Active>,
    pub slots: [Result<(), DecodeError>; 2],
    pub last_error: Option<Error>,
}

pub struct Saved {
    pub active: Active,
    pub written: bool,
}

pub struct ParameterManager<'d> {
    values: RefCell<Parameters>,
    fram: Mutex<NoopRawMutex, Fram<'d>>,
    status: Cell<Status>,
    save_due: Cell<parameters::DeferredSave>,
}

impl<'d> ParameterManager<'d> {
    pub fn new(fram: Fram<'d>) -> Self {
        v6x_rust_led::log_config::refresh_all();
        Self {
            values: RefCell::new(Parameters::default()),
            fram: Mutex::new(fram),
            status: Cell::new(Status {
                load: LoadState::Unloaded,
                active: None,
                slots: [Err(DecodeError::Uncommitted); 2],
                last_error: None,
            }),
            save_due: Cell::new(parameters::DeferredSave::default()),
        }
    }

    pub fn get(&self, id: Id) -> f32 {
        self.values.borrow().get(id)
    }

    /// Validate once, then let only the estimator's own parameters reprogram it.
    /// A refused configuration is rolled back so RAM never reports a value the
    /// estimator rejected.
    pub fn set(&self, id: Id, value: f32) -> Result<bool, parameters::Error> {
        let previous = self.get(id);
        let changed = self.values.borrow_mut().set(id, value)?;
        if !changed {
            return Ok(false);
        }
        if parameters::is_estimator_parameter(id) {
            // Drop the read borrow before a rejected configuration rolls RAM back.
            let snapshot = self.snapshot();
            if let Err(error) = crate::estimator::configure(snapshot) {
                let _ = self.values.borrow_mut().set(id, previous);
                return Err(error);
            }
        }
        v6x_rust_led::log_config::apply(id, value);
        crate::sd_logger::record_parameter(
            Instant::now().as_micros(),
            id as u16,
            id.definition().kind,
            value,
            self.is_dirty(),
        );
        Ok(true)
    }

    pub fn snapshot(&self) -> parameters::Snapshot {
        self.values.borrow().snapshot()
    }

    /// Push every LOG_* parameter's current RAM value into the logging policy.
    fn apply_log_config(&self) {
        for definition in parameters::REGISTRY {
            if definition.group == parameters::Group::SdLog {
                v6x_rust_led::log_config::apply(definition.id, self.get(definition.id));
            }
        }
    }

    pub fn is_dirty(&self) -> bool {
        self.values.borrow().is_dirty()
    }

    /// Defer the FRAM commit; a later write restarts the quiet window.
    pub fn defer_save(&self) {
        let mut pending = self.save_due.get();
        pending.defer(Instant::now().as_micros());
        self.save_due.set(pending);
    }

    pub fn save_pending(&self) -> bool {
        self.save_due.get().pending()
    }

    /// Commit the deferred snapshot once the batch has been quiet long enough.
    pub async fn flush_deferred(&self, outputs: OutputState) -> Option<Result<Saved, Error>> {
        if !self.save_due.get().ready(Instant::now().as_micros()) {
            return None;
        }
        Some(self.save(outputs).await)
    }

    pub fn status(&self) -> Status {
        self.status.get()
    }

    /// Read compatible storage or use defaults. Never writes on boot/load.
    pub async fn load(&self) -> Result<LoadState, Error> {
        // Loading replaces RAM, so any deferred commit is obsolete.
        self.save_due.set(parameters::DeferredSave::default());
        let result = self.load_inner().await;
        if let Err(error) = result {
            let mut status = self.status.get();
            status.load = LoadState::Unavailable;
            status.last_error = Some(error);
            self.status.set(status);
        }
        if result.is_ok() {
            self.log_all();
            self.apply_log_config();
            crate::estimator::configure(self.snapshot()).map_err(|_| Error::Unavailable)?;
        }
        result
    }

    async fn load_inner(&self) -> Result<LoadState, Error> {
        let mut fram = self.fram.lock().await;
        if !fram.identify().await?.supported {
            return Err(Error::Fram(fram::Error::Unidentified));
        }
        let mut records = [Err(DecodeError::Uncommitted); 2];
        let mut bytes = [0; store::SLOT_SIZE];
        for slot in Slot::ALL {
            read_bytes(&mut fram, slot.address(), &mut bytes).await?;
            records[slot as usize] = store::decode(&bytes);
        }
        let loaded = store::newest(&records);
        let (load, active) = if let Some((active, record)) = loaded {
            let mut values = self.values.borrow_mut();
            if record.migrated {
                // Values are usable, but the record on FRAM is in the old
                // positional format: keep it dirty so the next save rewrites it.
                values.adopt(record.snapshot);
            } else {
                values.restore(record.snapshot);
            }
            (LoadState::Stored, Some(active))
        } else {
            *self.values.borrow_mut() = Parameters::default();
            (LoadState::Defaults, None)
        };
        self.status.set(Status {
            load,
            active,
            slots: records.map(|record| record.map(|_| ())),
            last_error: None,
        });
        Ok(load)
    }

    /// Explicit, serialized save; the exact verified snapshot becomes `saved`.
    pub async fn save(&self, outputs: OutputState) -> Result<Saved, Error> {
        // An explicit save already persists everything pending.
        self.save_due.set(parameters::DeferredSave::default());
        let result = self.save_inner(outputs).await;
        let mut status = self.status.get();
        status.last_error = result.as_ref().err().copied();
        self.status.set(status);
        if result.is_ok() {
            self.log_all();
        }
        result
    }

    async fn save_inner(&self, outputs: OutputState) -> Result<Saved, Error> {
        let mut fram = self.fram.lock().await;
        let mut status = self.status.get();
        if matches!(status.load, LoadState::Unloaded | LoadState::Unavailable) {
            return Err(Error::Unavailable);
        }
        let snapshot = self.values.borrow().snapshot();
        if let Some(active) = status.active.filter(|_| !self.is_dirty()) {
            return Ok(Saved {
                active,
                written: false,
            });
        }
        let commit =
            Commit::new(status.active, snapshot, outputs).map_err(|error| match error {
                CommitError::OutputsActive => Error::OutputsActive,
                CommitError::Capacity => Error::Capacity,
            })?;
        let mut verified = [0; fram::CHUNK];
        for write in commit.writes() {
            for (index, chunk) in write.bytes.chunks(fram::CHUNK).enumerate() {
                let address = write.address + (index * fram::CHUNK) as u16;
                fram.write(address, chunk).await?;
                fram.read(address, &mut verified[..chunk.len()]).await?;
                if &verified[..chunk.len()] != chunk {
                    return Err(Error::Readback);
                }
            }
        }
        // Every byte just written was read back above, which already covers the
        // whole record; decoding the same bytes only proves the encoder built a
        // self-consistent frame and costs no extra I/O or stack.
        store::decode(commit.data()).map_err(|_| Error::Readback)?;
        self.values.borrow_mut().mark_saved(snapshot);
        status.load = LoadState::Stored;
        status.active = Some(commit.active);
        status.slots[commit.active.slot as usize] = Ok(());
        self.status.set(status);
        Ok(Saved {
            active: commit.active,
            written: true,
        })
    }

    pub async fn fram_info(&self) -> Result<fram::Info, fram::Error> {
        self.fram.lock().await.identify().await
    }

    pub async fn read_fram(&self, address: u16, bytes: &mut [u8]) -> Result<(), fram::Error> {
        self.fram.lock().await.read(address, bytes).await
    }

    fn log_all(&self) {
        let timestamp = embassy_time::Instant::now().as_micros();
        let dirty = self.is_dirty();
        for definition in parameters::REGISTRY {
            crate::sd_logger::record_parameter(
                timestamp,
                definition.id as u16,
                definition.kind,
                self.get(definition.id),
                dirty,
            );
        }
    }
}

/// Owns deferred commits, so no link handler blocks on FRAM mid-batch.
#[embassy_executor::task]
pub async fn save_task(parameters: &'static ParameterManager<'static>) {
    loop {
        Timer::after(SAVE_POLL).await;
        // Failures stay visible through `!params` last_error; the values remain
        // in RAM instead of being reported as persisted.
        let _ = parameters.flush_deferred(crate::OUTPUT_STATE).await;
    }
}

async fn read_bytes(
    fram: &mut Fram<'_>,
    address: u16,
    bytes: &mut [u8],
) -> Result<(), fram::Error> {
    for (index, chunk) in bytes.chunks_mut(fram::CHUNK).enumerate() {
        fram.read(address + (index * fram::CHUNK) as u16, chunk)
            .await?;
    }
    Ok(())
}
