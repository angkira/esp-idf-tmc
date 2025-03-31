use crate::tmc_driver::traits::DigitalOutputPin;
use esp_idf_hal::gpio::{Output, PinDriver, AnyOutputPin};
use std::error::Error;

pub struct EspIdfDigitalOutputPin {
    pin_driver: PinDriver<'static, AnyOutputPin, Output>,
}

impl EspIdfDigitalOutputPin {
    pub fn new(pin: AnyOutputPin) -> Self {
        let pin_driver = PinDriver::output(pin).expect("Error while the pin_driver init");

        Self { pin_driver }
    }
}

unsafe impl Send for EspIdfDigitalOutputPin {}
unsafe impl Sync for EspIdfDigitalOutputPin {}
impl DigitalOutputPin for EspIdfDigitalOutputPin {
    fn set_high(&mut self) -> Result<(), Box<dyn Error>> {
        self.pin_driver.set_high()?;
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Box<dyn Error>> {
        self.pin_driver.set_low()?;
        Ok(())
    }

    fn get_num(&self) -> u32 {
        self.pin_driver.pin() as u32
    }
}