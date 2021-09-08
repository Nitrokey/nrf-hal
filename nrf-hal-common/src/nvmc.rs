//! HAL interface to the Non-Volatile Memory Controller (NVMC) peripheral.

use core::ops::Deref;

#[cfg(not(feature = "9160"))]
use crate::pac::nvmc;
#[cfg(feature = "9160")]
use crate::pac::nvmc_ns as nvmc;
#[cfg(not(feature = "9160"))]
use crate::pac::NVMC;
#[cfg(feature = "9160")]
use crate::pac::NVMC_NS as NVMC;

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

/// Interface to an NVMC instance.
pub struct Nvmc<T: Instance> {
    nvmc: T,
    storage: &'static mut [u32],
}

impl<T> Nvmc<T>
where
    T: Instance,
{
    /// Takes ownership of the peripheral and storage area.
    pub fn new(nvmc: T, storage: &'static mut [u32]) -> Nvmc<T> {
        Self { nvmc, storage }
    }

    /// Consumes `self` and returns back the raw peripheral and associated storage.
    pub fn free(self) -> (T, &'static mut [u32]) {
        (self.nvmc, self.storage)
    }

    fn enable_erase(&self) {
        #[cfg(not(feature = "9160"))]
        self.nvmc.config.write(|w| w.wen().een());
        #[cfg(feature = "9160")]
        self.nvmc.configns.write(|w| w.wen().een());
    }

    fn enable_read(&self) {
        #[cfg(not(feature = "9160"))]
        self.nvmc.config.write(|w| w.wen().ren());
        #[cfg(feature = "9160")]
        self.nvmc.configns.write(|w| w.wen().ren());
    }

    fn enable_write(&self) {
        #[cfg(not(feature = "9160"))]
        self.nvmc.config.write(|w| w.wen().wen());
        #[cfg(feature = "9160")]
        self.nvmc.configns.write(|w| w.wen().wen());
    }

    #[inline]
    fn wait_ready(&self) {
        while !self.nvmc.ready.read().ready().bit_is_set() {}
    }

    #[cfg(feature = "9160")]
    #[inline]
    fn wait_write_ready(&self) {
        while !self.nvmc.readynext.read().readynext().bit_is_set() {}
    }

    #[cfg(not(feature = "9160"))]
    #[inline]
    fn erase_page(&mut self, offset: usize) {
        let bits = &mut (self.storage[offset >> 2]) as *mut _ as u32;
        self.nvmc.erasepage().write(|w| unsafe { w.bits(bits) });
        self.wait_ready();
    }

    #[cfg(feature = "9160")]
    #[inline]
    fn erase_page(&mut self, offset: usize) {
        self.storage[offset >> 2] = 0xffffffff;
        self.wait_ready();
    }

    #[inline]
    fn write_word(&mut self, offset: usize, word: u32) {
        #[cfg(not(feature = "9160"))]
        self.wait_ready();
        #[cfg(feature = "9160")]
        self.wait_write_ready();
        self.storage[offset >> 2] = word;
        cortex_m::asm::dmb();
    }

    pub fn try_read_nonmut(&self, offset: u32, bytes: &mut [u8]) -> Result<(), NvmcError> {
        let offset = offset as usize;
        let bytes_len = bytes.len();

        if offset % Self::READ_SIZE != 0 || bytes.len() % Self::READ_SIZE != 0 {
		return Err(NvmcError::Unaligned);
	}

	if offset + bytes_len > self.capacity() {
		return Err(NvmcError::OutOfBounds);
	}

	let read_count = bytes_len / Self::READ_SIZE;

	self.wait_ready();
	for i in 0..read_count {
		let wordbytes = self.storage[(offset >> 2) + i].to_ne_bytes();
		for j in 0..4 {
			bytes[(i << 2) + j] = wordbytes[j];
		}
	}

        Ok(())
    }

}

impl<T> ReadNorFlash for Nvmc<T>
where
    T: Instance,
{
    type Error = NvmcError;

    const READ_SIZE: usize = 4;

    fn try_read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
	self.try_read_nonmut(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.storage.len() << 2
    }
}

impl<T> NorFlash for Nvmc<T>
where
    T: Instance,
{
    const WRITE_SIZE: usize = 4;

    const ERASE_SIZE: usize = 4 * 1024;

    fn try_erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
	let from = from as usize;
	let to = to as usize;

        if from % Self::ERASE_SIZE != 0 || to % Self::ERASE_SIZE != 0 {
		return Err(NvmcError::Unaligned);
	}

        self.enable_erase();
        for offset in (from..to).step_by(Self::ERASE_SIZE) {
                self.erase_page(offset as usize >> 2);
        }
	self.enable_read();
        Ok(())
    }

    fn try_write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let offset = offset as usize;

        if offset % Self::WRITE_SIZE != 0 || bytes.len() % Self::WRITE_SIZE != 0 {
		return Err(NvmcError::Unaligned);
	}

	if offset + bytes.len() > self.capacity() {
		return Err(NvmcError::OutOfBounds);
	}

	let write_count = bytes.len() / Self::WRITE_SIZE;

        self.enable_write();
	for i in 0..write_count {
		use core::convert::TryInto;
                self.write_word(offset + (i << 2), u32::from_ne_bytes(bytes[(i<<2)..(i<<2)+4].try_into().unwrap()));
        }
        self.enable_read();
        Ok(())
    }
}

pub trait Instance: Deref<Target = nvmc::RegisterBlock> + sealed::Sealed {}

impl Instance for NVMC {}

mod sealed {
    use super::*;

    pub trait Sealed {}

    impl Sealed for NVMC {}
}

#[derive(Debug)]
pub enum NvmcError {
    /// An operation was attempted on an unaligned boundary
    Unaligned,
    OutOfBounds
}
