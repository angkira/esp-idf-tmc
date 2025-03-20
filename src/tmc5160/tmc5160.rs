use crate::tmc5160::pin::PinSettings;
use crate::tmc5160::traits::SpiDevice;
use std::error::Error;
use std::io::{self, Write};


use esp_idf_hal::delay::FreeRtos;
// Register definitions and constants remain unchanged
const REG_GCONF: u8 = 0x00;
const REG_IHOLD_IRUN: u8 = 0x10;
const REG_TPOWERDOWN: u8 = 0x11;
const REG_TPWMTHRS: u8 = 0x13;
const REG_CHOPCONF: u8 = 0x6C;
const REG_PWMCONF: u8 = 0x70;
pub const REG_DRV_STATUS: u8 = 0x6F;
const REG_SGTHRS: u8 = 0x40;
const REG_TCOOLTHRS: u8 = 0x14;
const REG_COOLCONF: u8 = 0x42;
const REG_IOIN: u8 = 0x04;
const REG_SW_MODE: u8 = 0x34;

const CHOPCONF_TBL: u32 = 1 << 15;
const CHOPCONF_MRES_SHIFT: u32 = 24;
const GCONF_EN_PWM_MODE: u32 = 1 << 0;
const GCONF_SHAFT: u32 = 1 << 2;

#[derive(Debug)]
pub struct MotorParams {
    pub steps_per_revolution: u16,
}

#[derive(Debug)]
pub struct CurrentSettings {
    pub run_current: u8,
    pub hold_current: u8,
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

#[derive(Debug)]
pub struct CoolStepConfig {
    pub enable: bool,
    pub semin: u8,
    pub semax: u8,
    pub seup: u8,
    pub sedn: u8,
    pub sgthrs: u8,
}

#[derive(Debug)]
pub struct DcStepConfig {
    pub enable: bool,
}

pub struct Tmc5160Config {
    pub pins: PinSettings,
    pub current: CurrentSettings,
    pub microsteps: MicrostepSettings,
    pub motor_params: MotorParams,
    pub stealthchop: StealthChopSettings,
    pub spreadcycle: SpreadCycleSettings,
    pub coolstep: CoolStepConfig,
    pub dcstep: DcStepConfig,
}

pub struct Tmc5160 {
    spi: Box<dyn SpiDevice>,
    config: Tmc5160Config,
}

impl Tmc5160 {
    pub fn new(spi: Box<dyn SpiDevice>, config: Tmc5160Config) -> Self {
        Tmc5160 { spi, config }
    }

    pub fn init(&mut self) -> Result<(), Box<dyn Error>> {
        log::info!("Waiting 100ms for power-up...");
        FreeRtos::delay_ms(100);

        let mut gconf = 0x00000000;
        if self.config.stealthchop.enable {
            gconf |= GCONF_EN_PWM_MODE;
        }
        gconf |= GCONF_SHAFT;
        self.write_register("GCONF", REG_GCONF, gconf)?;

        let ihold_irun = ((self.config.current.run_current as u32) << 8)
            | (self.config.current.hold_current as u32);
        self.write_register("IHOLD_IRUN", REG_IHOLD_IRUN, ihold_irun)?;

        self.write_register("SGTHRS", REG_SGTHRS, self.config.coolstep.sgthrs as u32)?;
        self.write_register("TCOOLTHRS", REG_TCOOLTHRS, 0x000000FF)?;
        self.write_register("CHOPCONF", REG_CHOPCONF, self.configure_chopconf())?;
        self.write_register("PWMCONF", REG_PWMCONF, 0x000401C8)?;

        if self.config.coolstep.enable {
            self.write_register("COOLCONF", REG_COOLCONF, self.configure_coolconf())?;
        } else {
            self.write_register("COOLCONF", REG_COOLCONF, 0x00000000)?;
        }

        let mut sw_mode = 0x00000000;
        if self.config.dcstep.enable {
            sw_mode |= 0x00000400;
        }
        self.write_register("SW_MODE", REG_SW_MODE, sw_mode)?;

        log::info!("Waiting 100ms after initialization...");
        FreeRtos::delay_ms(100);

        self.log_driver_status()?;

        let ioin_value = self.read_register("IOIN", REG_IOIN)?;
        log::info!("IOIN: 0x{:08X}", ioin_value);
        Ok(())
    }

    pub fn log_driver_status(&mut self) -> Result<(), Box<dyn Error>> {
        let drv_status = self.read_register("DRV_STATUS", REG_DRV_STATUS)?;
        log::info!("DRV_STATUS: 0x{:08X}", drv_status);
        Ok(())
    }

pub fn rotate_by_angle(&mut self, angle: f64, rpm: f64) -> Result<(), Box<dyn Error>> {
    let steps_per_revolution = self.config.motor_params.steps_per_revolution as f64;
    let microsteps = self.config.microsteps.microsteps as f64;
    let total_steps = (angle / 360.0 * steps_per_revolution * microsteps).round() as i32;
    let steps = total_steps.abs();

    self.log_driver_status()?;

    let period_micros = 60.0 * 1_000_000.0 / (rpm * steps_per_revolution * microsteps);
    let pulse_width_micros = 5;
    let delay_micros = if period_micros > pulse_width_micros as f64 {
        period_micros - pulse_width_micros as f64
    } else {
        0.0
    };

    if total_steps > 0 {
        self.config.pins.dir_pin.set_high().expect("Error setting dir pin high");
    } else {
        self.config.pins.dir_pin.set_low().expect("Error setting dir pin low");
    }

    for _ in 0..steps {
        self.config.pins.step_pin.set_high().expect("Error setting step pin high");
        FreeRtos::delay_ms(pulse_width_micros as u32);
        self.config.pins.step_pin.set_low().expect("Error setting step pin low");
        FreeRtos::delay_ms(delay_micros as u32);
    }

    Ok(())
}

    fn configure_chopconf(&self) -> u32 {
        let mut chopconf: u32 = 0;
        let toff = if self.config.stealthchop.enable {
            self.config.stealthchop.toff
        } else {
            self.config.spreadcycle.toff
        };
        chopconf |= (toff as u32) & 0x0F;
        let mres = self.calculate_mres(self.config.microsteps.microsteps);
        chopconf |= mres << CHOPCONF_MRES_SHIFT;

        if self.config.stealthchop.enable {
            chopconf |= CHOPCONF_TBL;
        } else if self.config.spreadcycle.enable {
            chopconf |= (self.config.spreadcycle.hstrt as u32) << 4;
            chopconf |= (self.config.spreadcycle.hend as u32) << 7;
        }
        chopconf
    }

    fn calculate_mres(&self, microsteps: u16) -> u32 {
        match microsteps {
            256 => 0,
            128 => 1,
            64 => 2,
            32 => 3,
            16 => 4,
            8 => 5,
            4 => 6,
            2 => 7,
            1 => 8,
            _ => 0,
        }
    }

    fn configure_coolconf(&self) -> u32 {
        let mut coolconf: u32 = 0;
        coolconf |= (self.config.coolstep.semin as u32) << 0;
        coolconf |= (self.config.coolstep.semax as u32) << 5;
        coolconf |= (self.config.coolstep.seup as u32) << 10;
        coolconf |= (self.config.coolstep.sedn as u32) << 12;
        coolconf
    }

    fn write_register(
        &mut self,
        name: &str,
        address: u8,
        value: u32,
    ) -> Result<(), Box<dyn Error>> {
        log::info!(
            "Writing {}: Register 0x{:02X}, Value: 0x{:08X}",
            name, address, value
        );
        let buffer = [
            address | 0x80,
            (value >> 24) as u8,
            (value >> 16) as u8,
            (value >> 8) as u8,
            value as u8,
        ];

        self.read_register(name, address).expect("Error reading register");

        self.spi.write(&buffer).expect("Error writing to SPI device");

        FreeRtos::delay_ms(10);

        self.read_register(name, address).expect("Error reading register");

        // if response_buffer != value {
        //     log::error!("Error writing {}: Register 0x{:02X}, Value: 0x{:08X}", name, address, value);
        //     log::error!("Expected: 0x{:08X}, Received: 0x{:08X}", value, response_buffer);
        // }

        Ok(())
    }

    fn read_register(&mut self, name: &str, address: u8) -> Result<u32, Box<dyn Error>> {
        let cmd = [address & 0x7F, 0, 0, 0, 0];
        let mut response = [0u8; 5];

        self.spi.transfer(&mut response, &cmd).expect("Error reading from SPI device");

        FreeRtos::delay_ms(10);

        let response_value = u32::from_be_bytes([
            response[1],
            response[2],
            response[3],
            response[4],
        ]);

        log::info!("Reading {}: Register 0x{:02X}, Value: 0x{:08X}", name, address, response_value);

        Ok(response_value)
    }

    pub fn tune_sgthrs(&mut self) -> Result<(), Box<dyn Error>> {
        let original_coolstep_enable = self.config.coolstep.enable;
        let original_dcstep_enable = self.config.dcstep.enable;
        self.config.coolstep.enable = false;
        self.config.dcstep.enable = false;
        self.write_register("COOLCONF", REG_COOLCONF, 0)?;
        self.write_register("SW_MODE", REG_SW_MODE, 0)?;

        let mut sgthrs = 10u8;
        self.write_register("SGTHRS", REG_SGTHRS, sgthrs as u32)?;

        log::info!("Starting stallGuard2 tuning.  Increase load gradually.");
        log::info!("Press Enter after each load increase...");

        loop {
            self.rotate_by_angle(360.0, 60.0)?;
            let drv_status = self.read_register("DRV_STATUS", REG_DRV_STATUS)?;
            let sg2 = (drv_status >> 10) & 0x3FF;
            log::info!("SGTHRS: {}, SG2: {}", sgthrs, sg2);

            print!("Increase load (or press \\q to quit, \\s to save): ");
            io::stdout().flush()?;
            let mut input = String::new();
            io::stdin().read_line(&mut input)?;

            match input.trim() {
                "q" => {
                    log::info!("Tuning aborted.");
                    break;
                }
                "s" => {
                    log::info!("Saving current SGTHRS value: {}", sgthrs);
                    self.config.coolstep.sgthrs = sgthrs;
                    break;
                }
                _ => {
                    if sg2 > 50 {
                        if sgthrs < 255 {
                            sgthrs += 5;
                            self.write_register("SGTHRS", REG_SGTHRS, sgthrs as u32)?;
                            log::info!("Increased SGTHRS to: {}", sgthrs);
                        } else {
                            log::info!("SGTHRS reached maximum value (255).");
                            break;
                        }
                    } else {
                        log::info!("SGTHRS value seems good. You can increase the load or press \\s to save");
                    }
                }
            }
        }

        self.config.coolstep.enable = original_coolstep_enable;
        self.config.dcstep.enable = original_dcstep_enable;
        if self.config.coolstep.enable {
            self.write_register("COOLCONF", REG_COOLCONF, self.configure_coolconf())?;
        }
        if self.config.dcstep.enable {
            let mut sw_mode = 0x00000000;
            sw_mode |= 0x00000400;
            self.write_register("SW_MODE", REG_SW_MODE, sw_mode)?;
        }
        Ok(())
    }
}
