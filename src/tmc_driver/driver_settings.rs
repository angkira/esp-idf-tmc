use std::error::Error;
use crate::tmc_driver::pin::PinSettings;
use crate::tmc_driver::traits::SpiDevice;

#[derive(Debug)]
pub struct MotorParams {
    pub steps_per_revolution: u16,
}

#[derive(Debug)]
pub struct CurrentSettings {
    pub run_current: u8,    // 0-31 (Scales IRUN)
    pub hold_current: u8,   // 0-31 (Scales IHOLD)
    pub hold_delay: u8,   // 0-15 (Controls transition speed to IHOLD)
    pub global_scaler: u8, // 0-255 (Global current scale, 0=256/256)
}

#[derive(Debug)]
pub struct MicrostepSettings {
    pub microsteps: u16,
}

#[derive(Debug)]
pub struct StealthChopSettings {
    pub enable: bool,
    pub toff: u8,
}

#[derive(Debug)]
pub struct SpreadCycleSettings {
    pub enable: bool,
    pub toff: u8,
    pub hstrt: u8,
    pub hend: u8,
}

pub struct TMCBaseConfig {
    pub pins: PinSettings,
    pub current: CurrentSettings,
    pub microsteps: MicrostepSettings,
    pub motor_params: MotorParams,
    pub stealthchop: StealthChopSettings,
    pub spreadcycle: SpreadCycleSettings,
}

pub struct TMCConfig<FeatureConfig> {
    pub base: TMCBaseConfig,
    pub feature: FeatureConfig,
}

pub struct TMCDriver<FeatureConfig> {
    spi: Box<dyn SpiDevice>,
    config: TMCConfig<FeatureConfig>,
}

pub trait TMCDriverInterface<FeatureConfig> {
    fn new(spi: Box<dyn SpiDevice>, config: TMCConfig<FeatureConfig>) -> Self;

    fn init(&mut self) -> Result<(), Box<dyn Error>>;

    fn rotate_by_angle(&mut self, angle: f64, rpm: f64) -> Result<(), Box<dyn Error>>;

    fn configure_chopconf(&self) -> u32;

    fn calculate_mres(&self, microsteps: u16) -> u32;

    fn write_register(
        &mut self,
        name: &str,
        address: u8,
        value: u32,
    ) -> Result<(), Box<dyn Error>>;

    fn read_register(&mut self, name: &str, address: u8) -> Result<u32, Box<dyn Error>>;
}
