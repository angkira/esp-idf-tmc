use crate::tmc_driver::traits::DigitalOutputPin;

pub struct PinSettings {
    pub step_pin: Box<dyn DigitalOutputPin>,
    pub dir_pin: Box<dyn DigitalOutputPin>,
    pub enable_pin: Box<dyn DigitalOutputPin>,
}
