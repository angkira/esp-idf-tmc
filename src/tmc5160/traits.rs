// src/traits.rs
use std::error::Error;

pub trait DigitalOutputPin: Send + Sync {
    fn set_high(&mut self) -> Result<(), Box<dyn Error>>;
    fn set_low(&mut self) -> Result<(), Box<dyn Error>>;
}

pub trait SpiDevice {
    fn write(&mut self, buffer: &[u8; 5]) -> Result<(), Box<dyn Error>>;
    fn transfer(&mut self, buffer: &mut [u8; 5], cmd: &[u8; 5]) -> Result<(), Box<dyn Error>>;
}
