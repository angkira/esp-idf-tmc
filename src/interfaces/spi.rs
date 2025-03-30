// src/interfaces/spi.rs (Manual CS Control Version)

use crate::tmc_driver::traits::SpiDevice;
use esp_idf_hal::gpio::{AnyOutputPin, Output, PinDriver}; // Added PinDriver, Output, AnyOutputPin
use esp_idf_hal::spi::{config::Config, SpiBusDriver, SpiDriver};
use std::error::Error;
use esp_idf_hal::delay::FreeRtos;

// Define a simple error type for SPI communication (optional)
#[derive(Debug)]
struct SpiCommError(String);

impl std::fmt::Display for SpiCommError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "SPI Communication Error: {}", self.0)
    }
}

impl Error for SpiCommError {}


pub struct EspSpiDevice {
    driver: SpiBusDriver<'static, SpiDriver<'static>>,
    cs_pin: PinDriver<'static, AnyOutputPin, Output>, // Added CS pin driver
}

impl EspSpiDevice {
    // --- MODIFIED: `new` now takes the CS pin driver ---
    pub fn new(
        spi: SpiDriver<'static>,
        config: &Config,
        cs_pin_driver: PinDriver<'static, AnyOutputPin, Output>, // Added CS pin argument
    ) -> Self {
        let driver = SpiBusDriver::new(spi, config).expect("Error while the SPI bus driver init");
        let mut device = Self {
            driver,
            cs_pin: cs_pin_driver
        };
        // Ensure CS starts high (inactive)
        device.cs_pin.set_high().expect("Failed to set initial CS high");
        device
    }

    // Helper to wrap SPI transaction with CS control
    fn transaction<F, R>(&mut self, operation: F) -> Result<R, Box<dyn Error>>
    where
        F: FnOnce(&mut SpiBusDriver<'static, SpiDriver<'static>>) -> Result<R, Box<dyn Error>>,
    {
        self.cs_pin.set_low()?; // Assert CS
        // Short delay might be needed after CS assertion, depending on device spec
        // FreeRtos::delay_us(1);
        let result = operation(&mut self.driver);
        // FreeRtos::delay_us(1); // Optional delay before de-asserting
        self.cs_pin.set_high()?; // De-assert CS
        result
    }
}

impl SpiDevice for EspSpiDevice {
    // --- MODIFIED: write_register uses manual CS ---
    fn write_register(&mut self, address: u8, value: u32) -> Result<u8, Box<dyn Error>> {
        let write_cmd: [u8; 5] = [
            address | 0x80, // Set write bit
            (value >> 24) as u8,
            (value >> 16) as u8,
            (value >> 8) as u8,
            value as u8,
        ];
        let mut read_buffer: [u8; 5] = [0; 5];

        // Execute the transfer within a CS low/high block
        self.transaction(|driver| {
            driver.transfer(&mut read_buffer, &write_cmd).map_err(|e| e.into())
        })?;

        Ok(read_buffer[0]) // Return status byte
    }

    // --- MODIFIED: read_register uses manual CS for BOTH transactions ---
    fn read_register(&mut self, address: u8) -> Result<(u8, u32), Box<dyn Error>> {
        let read_addr_cmd: [u8; 5] = [
            address & 0x7F, // Clear write bit for read
            0, 0, 0, 0,     // Dummy data for read request
        ];
        let dummy_cmd: [u8; 5] = [0x00, 0, 0, 0, 0]; // Dummy command for second transaction
        let mut read_buffer: [u8; 5] = [0; 5];

        // 1. Send the read address command (CS low -> write -> CS high)
        self.transaction(|driver| {
            // Use write here, response buffer isn't useful yet
            driver.write(&read_addr_cmd).map_err(|e| e.into())
        })?; // CS goes high here

        // 2. Introduce a small delay between transactions
        FreeRtos::delay_ms(10); // Adjust if needed

        // 3. Send dummy command to clock out data (CS low -> transfer -> CS high)
        self.transaction(|driver| {
            driver.transfer(&mut read_buffer, &dummy_cmd).map_err(|e| e.into())
        })?; // CS goes high here

        // The requested data should now be in bytes 1-4 of read_buffer
        let value = u32::from_be_bytes([read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4]]);
        // The status byte from this second transaction is in read_buffer[0]
        let status = read_buffer[0];

        Ok((status, value))
    }
}