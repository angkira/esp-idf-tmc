use crate::tmc5160::traits::SpiDevice;
use esp_idf_hal::spi::{SpiBusDriver, SpiDriver, config::Config};
use std::error::Error;
use esp_idf_hal::delay::FreeRtos;

pub struct EspSpiDevice {
    driver: SpiBusDriver<'static, SpiDriver<'static>>,
}

impl EspSpiDevice {
    pub fn new(spi: SpiDriver<'static>, config: &Config) -> Self {
        let driver = SpiBusDriver::new(spi, config).expect("Error while the SPI bus driver init");

        Self { driver }
    }
}

impl SpiDevice for EspSpiDevice {
    fn write(&mut self, buffer: &[u8; 5]) -> Result<(), Box<dyn Error>> {
        self.driver.write(buffer).map_err(|e| e.into())
    }

    fn transfer(&mut self, data: &mut [u8; 5], cmd: &[u8; 5]) -> Result<(), Box<dyn Error>> {
        // First send the command (register address)
        // self.write(cmd).expect("Error writing command");

        // FreeRtos::delay_ms(1);

        self.driver.transfer(data, cmd).map_err(|e| e.into())
    }
}
