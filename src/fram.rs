//! FM25V02A on the board's dedicated SPI5 bus. See docs/fmuv6x-hardware.md.

use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::pac::SPI5;
use embassy_stm32::spi::{mode::Master, Spi};
use embassy_time::{with_timeout, Duration};

pub const CAPACITY: usize = 32 * 1024;
pub const CHUNK: usize = 256;
const DEVICE_ID: [u8; 9] = [0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0xc2, 0x22, 0x08];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Spi,
    Timeout,
    Unidentified,
    Range,
    Protected,
    WriteLatch,
}

pub struct Info {
    pub id: [u8; 9],
    pub status: u8,
    pub supported: bool,
    pub spi_hz: u32,
}

pub struct Fram<'d> {
    spi: Spi<'d, Async, Master>,
    cs: Output<'d>,
    identified: bool,
}

impl<'d> Fram<'d> {
    pub fn new(spi: Spi<'d, Async, Master>, cs: Output<'d>) -> Self {
        Self {
            spi,
            cs,
            identified: false,
        }
    }

    pub async fn identify(&mut self) -> Result<Info, Error> {
        self.identified = false;
        let mut bytes = [0; 10];
        bytes[0] = 0x9f;
        self.transaction(&mut bytes).await?;
        let id = bytes[1..].try_into().unwrap();
        let status = self.read_status().await?;
        self.identified = id == DEVICE_ID;
        Ok(Info {
            id,
            status,
            supported: self.identified,
            spi_hz: self.spi.get_current_config().frequency.0,
        })
    }

    pub async fn read(&mut self, address: u16, data: &mut [u8]) -> Result<(), Error> {
        self.check_range(address, data.len())?;
        let mut bytes = [0; CHUNK + 3];
        bytes[0] = 0x03;
        bytes[1..3].copy_from_slice(&address.to_be_bytes());
        self.transaction(&mut bytes[..data.len() + 3]).await?;
        data.copy_from_slice(&bytes[3..data.len() + 3]);
        Ok(())
    }

    pub async fn write(&mut self, address: u16, data: &[u8]) -> Result<(), Error> {
        self.check_range(address, data.len())?;
        // Any BP bit protects the tail used by our slots. Preserve the chip's
        // protection configuration; no WRSR command is provided.
        if self.read_status().await? & 0x0c != 0 {
            return Err(Error::Protected);
        }
        self.transaction(&mut [0x06]).await?;
        if self.read_status().await? & 0x02 == 0 {
            return Err(Error::WriteLatch);
        }
        let mut bytes = [0; CHUNK + 3];
        bytes[0] = 0x02;
        bytes[1..3].copy_from_slice(&address.to_be_bytes());
        bytes[3..data.len() + 3].copy_from_slice(data);
        self.transaction(&mut bytes[..data.len() + 3]).await?;
        if self.read_status().await? & 0x02 != 0 {
            return Err(Error::WriteLatch);
        }
        Ok(())
    }

    fn check_range(&self, address: u16, len: usize) -> Result<(), Error> {
        if !self.identified {
            return Err(Error::Unidentified);
        }
        if len == 0 || len > CHUNK || usize::from(address) + len > CAPACITY {
            return Err(Error::Range);
        }
        Ok(())
    }

    async fn read_status(&mut self) -> Result<u8, Error> {
        let mut bytes = [0x05, 0];
        self.transaction(&mut bytes).await?;
        Ok(bytes[1])
    }

    async fn transaction(&mut self, bytes: &mut [u8]) -> Result<(), Error> {
        // All DMA buffers live in AXI SRAM (task future or shared CPU stack),
        // with D-cache disabled by platform::init. DMA2 can access that region.
        self.cs.set_low();
        let _selected = Selected(&mut self.cs);
        with_timeout(Duration::from_millis(20), self.spi.transfer_in_place(bytes))
            .await
            .map_err(|_| Error::Timeout)?
            .map_err(|_| Error::Spi)
    }
}

struct Selected<'a, 'd>(&'a mut Output<'d>);

impl Drop for Selected<'_, '_> {
    fn drop(&mut self) {
        // A USB session reset can cancel a transfer. Its DMA futures drop first
        // and stop both streams; release CS and clear SPI's remaining enables.
        self.0.set_high();
        SPI5.cr1().modify(|w| w.set_spe(false));
        SPI5.cfg1().modify(|w| {
            w.set_txdmaen(false);
            w.set_rxdmaen(false);
        });
        SPI5.ifcr().write(|w| {
            w.set_ovrc(true);
            w.set_udrc(true);
            w.set_modfc(true);
            w.set_suspc(true);
        });
    }
}
