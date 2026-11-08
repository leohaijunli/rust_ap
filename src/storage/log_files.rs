//! Bounded log packets and root-only file removal, shared with in-memory tests.

use embedded_fatfs::{Error, FileSystem, OemCpConverter, ReadWriteSeek, TimeProvider};
use embedded_io_async::{Read, ReadExactError};

pub async fn read_packet<R: Read>(
    reader: &mut R,
    remaining: u32,
    packet: &mut [u8; 90],
) -> Result<u8, ReadExactError<R::Error>> {
    packet.fill(0);
    let count = remaining.min(90) as usize;
    // A FAT read can stop at a cluster boundary. Fill the packet's requested
    // length; only the request/file tail may be shorter than a 90-byte bin.
    reader.read_exact(&mut packet[..count]).await?;
    Ok(count as u8)
}

pub async fn erase_root_logs<IO, TP, OCC>(
    filesystem: &FileSystem<IO, TP, OCC>,
    accepts_name: fn(&[u8]) -> bool,
) -> Result<u32, Error<IO::Error>>
where
    IO: ReadWriteSeek,
    TP: TimeProvider,
    OCC: OemCpConverter,
{
    let root = filesystem.root_dir();
    let mut entries = root.iter();
    let mut deleted = 0u32;
    while let Some(entry) = entries.next().await {
        let entry = entry?;
        let name = entry.short_file_name_as_bytes();
        if entry.is_file() && accepts_name(name) {
            let name = core::str::from_utf8(name).map_err(|_| Error::InvalidInput)?;
            // No target File handle is open. FAT removal marks slots deleted;
            // it does not compact the directory or move the iterator's offsets.
            root.remove(name).await?;
            deleted = deleted.saturating_add(1);
        }
    }
    // The caller must unmount and flush the backing stream before confirming.
    Ok(deleted)
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::{
        cell::Cell,
        future::Future,
        task::{Context, Poll, Waker},
    };
    use embedded_fatfs::{format_volume, FormatVolumeOptions, FsOptions};
    use embedded_io_async::{ErrorKind, ErrorType, Seek, SeekFrom, Write};
    use std::{vec, vec::Vec};

    struct MemoryDisk<'a> {
        bytes: Vec<u8>,
        position: usize,
        fail_writes: &'a Cell<bool>,
    }

    impl ErrorType for MemoryDisk<'_> {
        type Error = ErrorKind;
    }

    impl Read for MemoryDisk<'_> {
        async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
            let count = buffer
                .len()
                .min(self.bytes.len().saturating_sub(self.position));
            if count != 0 {
                buffer[..count].copy_from_slice(&self.bytes[self.position..self.position + count]);
                self.position += count;
            }
            Ok(count)
        }
    }

    impl Write for MemoryDisk<'_> {
        async fn write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error> {
            if self.fail_writes.get() {
                return Err(ErrorKind::Other);
            }
            let count = bytes
                .len()
                .min(self.bytes.len().saturating_sub(self.position));
            if count != 0 {
                self.bytes[self.position..self.position + count].copy_from_slice(&bytes[..count]);
                self.position += count;
            }
            Ok(count)
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl Seek for MemoryDisk<'_> {
        async fn seek(&mut self, from: SeekFrom) -> Result<u64, Self::Error> {
            let position = match from {
                SeekFrom::Start(position) => {
                    i64::try_from(position).map_err(|_| ErrorKind::InvalidInput)?
                }
                SeekFrom::Current(offset) => self.position as i64 + offset,
                SeekFrom::End(offset) => self.bytes.len() as i64 + offset,
            };
            self.position = usize::try_from(position).map_err(|_| ErrorKind::InvalidInput)?;
            Ok(self.position as u64)
        }
    }

    fn run<F: Future>(future: F) -> F::Output {
        let mut future = core::pin::pin!(future);
        match future
            .as_mut()
            .poll(&mut Context::from_waker(Waker::noop()))
        {
            Poll::Ready(output) => output,
            Poll::Pending => panic!("memory-only I/O must not wait for external work"),
        }
    }

    async fn formatted_disk(fail_writes: &Cell<bool>) -> MemoryDisk<'_> {
        let mut disk = MemoryDisk {
            bytes: vec![0; 1024 * 1024],
            position: 0,
            fail_writes,
        };
        format_volume(&mut disk, FormatVolumeOptions::new())
            .await
            .unwrap();
        disk.seek(SeekFrom::Start(0)).await.unwrap();
        disk
    }

    #[test]
    fn packets_fill_across_real_fat_short_reads_and_zero_pad_the_tail() {
        run(async {
            let fail = Cell::new(false);
            let mut disk = formatted_disk(&fail).await;
            let fs = FileSystem::new(&mut disk, FsOptions::new()).await.unwrap();
            let cluster_size = fs.cluster_size() as usize;
            let data: Vec<_> = (0..cluster_size * 2 + 137)
                .map(|i| (i % 251) as u8)
                .collect();
            let root = fs.root_dir();
            let mut file = root.create_file("TEST.ULG").await.unwrap();
            file.write_all(&data).await.unwrap();
            file.flush().await.unwrap();
            let start = cluster_size / 90 * 90;
            file.seek(SeekFrom::Start(start as u64)).await.unwrap();
            let mut packet = [0; 90];
            assert!(file.read(&mut packet).await.unwrap() < 90);
            file.seek(SeekFrom::Start(start as u64)).await.unwrap();
            let mut offset = start;
            while offset < data.len() {
                assert_eq!(offset % 90, 0);
                let remaining = (data.len() - offset) as u32;
                let count = read_packet(&mut file, remaining, &mut packet)
                    .await
                    .unwrap() as usize;
                assert_eq!(count, remaining.min(90) as usize);
                assert_eq!(&packet[..count], &data[offset..offset + count]);
                assert!(packet[count..].iter().all(|byte| *byte == 0));
                offset += count;
            }
        });
    }

    #[test]
    fn unexpected_eof_is_an_error_not_a_successful_short_packet() {
        run(async {
            let fail = Cell::new(false);
            let mut disk = formatted_disk(&fail).await;
            let fs = FileSystem::new(&mut disk, FsOptions::new()).await.unwrap();
            let root = fs.root_dir();
            let mut file = root.create_file("SHORT.ULG").await.unwrap();
            file.write_all(b"short").await.unwrap();
            file.flush().await.unwrap();
            file.seek(SeekFrom::Start(0)).await.unwrap();
            assert!(matches!(
                read_packet(&mut file, 90, &mut [0; 90]).await,
                Err(ReadExactError::UnexpectedEof)
            ));
        });
    }

    #[test]
    fn erase_removes_only_root_logs_and_persists_a_fresh_empty_log_list() {
        run(async {
            let fail = Cell::new(false);
            let mut disk = formatted_disk(&fail).await;
            let fs = FileSystem::new(&mut disk, FsOptions::new()).await.unwrap();
            {
                let root = fs.root_dir();
                for name in ["ONE.ULG", "TWO.ulg", "NOTE.TXT"] {
                    let mut file = root.create_file(name).await.unwrap();
                    file.write_all(b"kept or removed by name").await.unwrap();
                    file.flush().await.unwrap();
                }
                let directory = root.create_dir("KEEP.ULG").await.unwrap();
                let mut file = directory.create_file("NESTED.ULG").await.unwrap();
                file.write_all(b"nested log stays").await.unwrap();
                file.flush().await.unwrap();
            }
            assert_eq!(
                erase_root_logs(&fs, crate::logger::is_ulog_name)
                    .await
                    .unwrap(),
                2
            );
            fs.unmount().await.unwrap();
            disk.flush().await.unwrap();
            disk.seek(SeekFrom::Start(0)).await.unwrap();
            let fs = FileSystem::new(&mut disk, FsOptions::new()).await.unwrap();
            let root = fs.root_dir();
            assert!(matches!(
                root.open_file("ONE.ULG").await,
                Err(Error::NotFound)
            ));
            assert!(matches!(
                root.open_file("TWO.ULG").await,
                Err(Error::NotFound)
            ));
            let mut note = root.open_file("NOTE.TXT").await.unwrap();
            let mut bytes = [0; 23];
            note.read_exact(&mut bytes).await.unwrap();
            assert_eq!(&bytes, b"kept or removed by name");
            assert!(root
                .open_dir("KEEP.ULG")
                .await
                .unwrap()
                .open_file("NESTED.ULG")
                .await
                .is_ok());
            let mut entries = root.iter();
            while let Some(entry) = entries.next().await {
                let entry = entry.unwrap();
                assert!(
                    !entry.is_file()
                        || !crate::logger::is_ulog_name(entry.short_file_name_as_bytes())
                );
            }
        });
    }

    #[test]
    fn erase_io_failure_cannot_return_a_successful_count() {
        run(async {
            let fail = Cell::new(false);
            let mut disk = formatted_disk(&fail).await;
            let fs = FileSystem::new(&mut disk, FsOptions::new()).await.unwrap();
            {
                let root = fs.root_dir();
                let mut file = root.create_file("FAIL.ULG").await.unwrap();
                file.write_all(b"log").await.unwrap();
                file.flush().await.unwrap();
            }
            fail.set(true);
            assert!(erase_root_logs(&fs, crate::logger::is_ulog_name)
                .await
                .is_err());
        });
    }
}
