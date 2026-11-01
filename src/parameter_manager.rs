//! Shared parameter API; only this layer owns FRAM and acknowledges persistence.

use core::cell::{Cell, RefCell};

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use v6x_rust_led::parameter_store::{
    self as store, Active, Commit, DecodeError, OutputState, Slot,
};
use v6x_rust_led::parameters::{self, Id, Parameters};

use crate::fram::{self, Fram};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Fram(fram::Error),
    OutputsActive,
    Unavailable,
    Readback,
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
}

impl<'d> ParameterManager<'d> {
    pub fn new(fram: Fram<'d>) -> Self {
        Self {
            values: RefCell::new(Parameters::default()),
            fram: Mutex::new(fram),
            status: Cell::new(Status {
                load: LoadState::Unloaded,
                active: None,
                slots: [Err(DecodeError::Uncommitted); 2],
                last_error: None,
            }),
        }
    }

    pub fn get(&self, id: Id) -> f32 {
        self.values.borrow().get(id)
    }

    pub fn set(&self, id: Id, value: f32) -> Result<bool, parameters::Error> {
        self.values.borrow_mut().set(id, value)
    }

    pub fn is_dirty(&self) -> bool {
        self.values.borrow().is_dirty()
    }

    pub fn status(&self) -> Status {
        self.status.get()
    }

    /// Read compatible storage or use defaults. Never writes on boot/load.
    pub async fn load(&self) -> Result<LoadState, Error> {
        let result = self.load_inner().await;
        if let Err(error) = result {
            let mut status = self.status.get();
            status.load = LoadState::Unavailable;
            status.last_error = Some(error);
            self.status.set(status);
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
        let (load, active) = if let Some((active, snapshot)) = loaded {
            self.values.borrow_mut().restore(snapshot);
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
        let result = self.save_inner(outputs).await;
        let mut status = self.status.get();
        status.last_error = result.as_ref().err().copied();
        self.status.set(status);
        result
    }

    async fn save_inner(&self, outputs: OutputState) -> Result<Saved, Error> {
        let mut fram = self.fram.lock().await;
        let mut status = self.status.get();
        if matches!(status.load, LoadState::Unloaded | LoadState::Unavailable) {
            return Err(Error::Unavailable);
        }
        let snapshot = self.values.borrow().snapshot();
        let commit =
            Commit::new(status.active, snapshot, outputs).map_err(|_| Error::OutputsActive)?;
        if let Some(active) = status.active.filter(|_| !self.is_dirty()) {
            return Ok(Saved {
                active,
                written: false,
            });
        }
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
        let mut bytes = [0; store::RECORD_SIZE];
        read_bytes(&mut fram, commit.active.slot.address(), &mut bytes).await?;
        if &bytes != commit.bytes() || store::decode(&bytes).is_err() {
            return Err(Error::Readback);
        }
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
