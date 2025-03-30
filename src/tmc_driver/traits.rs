use std::error::Error;

pub trait DigitalOutputPin: Send + Sync {
    fn set_high(&mut self) -> Result<(), Box<dyn Error>>;
    fn set_low(&mut self) -> Result<(), Box<dyn Error>>;
}

pub trait SpiDevice {
    /// Writes a value to a register, returning the status byte clocked out during the write.
    fn write_register(&mut self, address: u8, value: u32) -> Result<u8, Box<dyn Error>>;
    /// Reads a value from a register, returning the status byte and the 32-bit value.
    /// Handles the two-step read protocol internally.
    fn read_register(&mut self, address: u8) -> Result<(u8, u32), Box<dyn Error>>;
}