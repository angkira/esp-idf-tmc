use crate::tmc_driver::driver_settings::{TMCConfig, TMCDriverInterface};
use crate::tmc_driver::traits::SpiDevice;
use esp_idf_hal::delay::Ets;
use std::error::Error;
use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::rmt;
use esp_idf_hal::rmt::config::TransmitConfig;
use esp_idf_hal::rmt::{PinState, Pulse, PulseTicks, TxRmtDriver, FixedLengthSignal};
use log::info;

fn delay_us(micros: u32) {
    Ets::delay_us(micros);
}

fn delay_ms(millis: u32) {
    Ets::delay_ms(millis);
}

#[derive(Debug)]
pub struct StallGuard4Config {
    pub enable: bool,
    pub angle_offset: bool,
    pub threshold: u8,
}

#[derive(Debug)]
pub struct AutoTuningConfig {
    pub enable: bool,
    pub pwm_reg: u8,
    pub pwm_ofs: u8,
    pub pwm_grad: u8,
}

// TMC2240 Register definitions
const REG_GCONF: u8 = 0x00;
const REG_GSTAT: u8 = 0x01;
const REG_GLOBAL_SCALER: u8 = 0x0B;
const REG_IHOLD_IRUN: u8 = 0x10;
const REG_TPOWERDOWN: u8 = 0x11;
const REG_TPWMTHRS: u8 = 0x13;
const REG_TCOOLTHRS: u8 = 0x14;
const REG_SGTHRS: u8 = 0x40;
const REG_SG4_THRS: u8 = 0x41;
const REG_COOLCONF: u8 = 0x42;
const REG_CHOPCONF: u8 = 0x6C;
const REG_PWMCONF: u8 = 0x70;
const REG_DRV_STATUS: u8 = 0x6F;
const REG_IOIN: u8 = 0x04;

// TMC2240 Specific register constants
const GCONF_EN_PWM_MODE: u32 = 1 << 0;
const GCONF_SHAFT: u32 = 1 << 2;
const GCONF_DIAG0_ERROR: u32 = 1 << 4;
const GCONF_DIAG1_STALL: u32 = 1 << 5;
const GCONF_MULTISTEP_FILT: u32 = 1 << 8;

const CHOPCONF_MRES_SHIFT: u32 = 24;
const CHOPCONF_INTPOL: u32 = 1 << 28;
const CHOPCONF_TBL_SHIFT: u32 = 15;

pub struct TMC2240Features {
    pub stallguard4: StallGuard4Config,
    pub autotuning: AutoTuningConfig,
}

pub struct Tmc2240<'d> {
    spi: Box<dyn SpiDevice>,
    config: TMCConfig<TMC2240Features>,
    tx_rmt_driver: Option<TxRmtDriver<'d>>,
}

impl<'a> TMCDriverInterface<'a, TMC2240Features> for Tmc2240<'a> {
    fn new(
        spi: Box<dyn SpiDevice>,
        config: TMCConfig<TMC2240Features>,
    ) -> Tmc2240<'static> {
        Tmc2240 {
            spi,
            config,
            tx_rmt_driver: None,
        }
    }

    fn init(&mut self, rmt_channel: impl Peripheral<P = impl rmt::RmtChannel> + 'a,
            step_pin: impl Peripheral<P = impl OutputPin> + 'a) -> Result<(), Box<dyn Error>> {
        self.enable()?;

        log::info!("Waiting 1000ms for power-up...");
        delay_ms(1000); // Power-up delay

        match self.init_rmt(rmt_channel, step_pin) {
            Ok(_) => log::info!("RMT is initialized"),
            Err(e) => {
                log::error!("Error initializing RMT: {}", e);
                return Err(e)
            },
        };

        // Clear any existing status flags
        match self.spi.read_register(REG_GSTAT) {
            Ok((status, value)) => {
                log::info!("Reading GSTAT: Status 0x{:02X}, Value: 0x{:08X}", status, value);
                // Write 1 to clear any existing flags
                self.spi.write_register(REG_GSTAT, value)?;
            },
            Err(e) => log::error!("Error reading GSTAT: {}", e),
        }

        // --- GCONF Configuration ---
        let mut gconf = 0x00000000;
        gconf |= GCONF_SHAFT; // Set direction bit
        if self.config.base.stealthchop.enable {
            gconf |= GCONF_EN_PWM_MODE; // Enable StealthChop2
        }
        // Enable DIAG0 for error output
        gconf |= GCONF_DIAG0_ERROR;
        
        log::info!(
            "Writing GCONF: Register 0x{:02X}, Value: 0x{:08X}",
            REG_GCONF,
            gconf
        );
        match self.spi.write_register(REG_GCONF, gconf) {
            Ok(status) => log::info!("GCONF write successful, SPI status: 0x{:02X}", status),
            Err(e) => log::error!("Error writing GCONF: {}", e),
        }

        // --- GLOBALSCALER ---
        let global_scaler_val = self.config.base.current.global_scaler as u32;
        log::info!(
            "Writing GLOBAL_SCALER: Register 0x{:02X}, Value: 0x{:08X}",
            REG_GLOBAL_SCALER,
            global_scaler_val
        );
        self.spi.write_register(REG_GLOBAL_SCALER, global_scaler_val)?;

        // --- IHOLD_IRUN ---
        let ihold_delay = self.config.base.current.hold_delay as u32;
        let ihold = self.config.base.current.hold_current as u32 & 0x1F;
        let irun = self.config.base.current.run_current as u32 & 0x1F;
        let ihold_irun = (ihold_delay << 16) | (irun << 8) | ihold;

        log::info!(
            "Writing IHOLD_IRUN: Register 0x{:02X}, Value: 0x{:08X}",
            REG_IHOLD_IRUN,
            ihold_irun
        );
        self.spi.write_register(REG_IHOLD_IRUN, ihold_irun)?;

        // --- StallGuard4 Configuration (TMC2240 specific) ---
        if self.config.feature.stallguard4.enable {
            log::info!(
                "Writing SG4_THRS: Register 0x{:02X}, Value: 0x{:08X}",
                REG_SG4_THRS,
                self.config.feature.stallguard4.threshold as u32
            );
            self.spi.write_register(REG_SG4_THRS, self.config.feature.stallguard4.threshold as u32)?;
        }

        // --- Velocity threshold for mode switching ---
        log::info!(
            "Writing TCOOLTHRS: Register 0x{:02X}, Value: 0x{:08X}",
            REG_TCOOLTHRS,
            0x000000FF
        );
        self.spi.write_register(REG_TCOOLTHRS, 0x000000FF)?;

        // --- CHOPCONF Configuration ---
        let chopconf_val = self.configure_chopconf();
        log::info!(
            "Writing CHOPCONF: Register 0x{:02X}, Value: 0x{:08X}",
            REG_CHOPCONF,
            chopconf_val
        );
        self.spi.write_register(REG_CHOPCONF, chopconf_val)?;

        // --- TPOWERDOWN ---
        let tpowerdown_val = 2u32;
        log::info!(
            "Writing TPOWERDOWN: Register 0x{:02X}, Value: 0x{:08X}",
            REG_TPOWERDOWN,
            tpowerdown_val
        );
        self.spi.write_register(REG_TPOWERDOWN, tpowerdown_val)?;

        // --- PWMCONF for StealthChop2 ---
        if self.config.base.stealthchop.enable {
            let mut pwmconf_val = 0xC1000024; // Default from datasheet
            
            // Configure automatic tuning if enabled
            if self.config.feature.autotuning.enable {
                pwmconf_val |= 1 << 18; // pwm_autoscale
                pwmconf_val |= 1 << 19; // pwm_autograd
                pwmconf_val |= (self.config.feature.autotuning.pwm_reg as u32) << 24; // PWM_REG
                pwmconf_val |= self.config.feature.autotuning.pwm_ofs as u32; // PWM_OFS
                pwmconf_val |= (self.config.feature.autotuning.pwm_grad as u32) << 8; // PWM_GRAD
            }
            
            log::info!(
                "Writing PWMCONF: Register 0x{:02X}, Value: 0x{:08X}",
                REG_PWMCONF,
                pwmconf_val
            );
            self.spi.write_register(REG_PWMCONF, pwmconf_val)?;
        }

        log::info!("TMC2240 initialization completed successfully");
        Ok(())
    }

    fn init_rmt(&mut self, rmt_channel: impl Peripheral<P = impl rmt::RmtChannel> + 'a,
                step_pin: impl Peripheral<P = impl OutputPin> + 'a) -> Result<(), Box<dyn Error>> {
        let rmt_config = TransmitConfig::new().clock_divider(self.config.base.rmt.rmt_clk_divider);
        let tx_rmt_driver = TxRmtDriver::new(rmt_channel, step_pin, &rmt_config)?;
        self.tx_rmt_driver = Some(tx_rmt_driver);
        Ok(())
    }

    fn rotate_by_angle(&mut self, angle: f64, rpm: f64) -> Result<(), Box<dyn Error>> {
        // Direction handling
        let direction = angle >= 0.0;
        let abs_angle = angle.abs();
        
        // Set direction
        if direction {
            self.config.base.pins.dir_pin.set_low()?;
        } else {
            self.config.base.pins.dir_pin.set_high()?;
        }

        // Calculate steps
        let steps_per_degree = self.config.base.motor_params.steps_per_revolution as f64 / 360.0;
        let total_steps = (abs_angle * steps_per_degree * self.config.base.microsteps.microsteps as f64) as u32;

        // Move steps
        self.move_steps_rmt(total_steps, rpm)?;

        Ok(())
    }

    fn configure_chopconf(&self) -> u32 {
        let mut chopconf = 0x10000053u32; // Default base value

        // Configure microstep resolution
        let mres = self.calculate_mres(self.config.base.microsteps.microsteps);
        chopconf = (chopconf & !(0xF << CHOPCONF_MRES_SHIFT)) | (mres << CHOPCONF_MRES_SHIFT);

        // Enable interpolation to 256 microsteps
        chopconf |= CHOPCONF_INTPOL;

        // Configure blanking time (TBL)
        chopconf = (chopconf & !(0x3 << CHOPCONF_TBL_SHIFT)) | (2u32 << CHOPCONF_TBL_SHIFT);

        // Set TOFF based on chopper mode
        let toff = if self.config.base.stealthchop.enable {
            self.config.base.stealthchop.toff as u32
        } else {
            self.config.base.spreadcycle.toff as u32
        };
        chopconf = (chopconf & !0xF) | (toff & 0xF);

        // Configure hysteresis for SpreadCycle
        if !self.config.base.stealthchop.enable {
            let hstrt = self.config.base.spreadcycle.hstrt as u32;
            let hend = self.config.base.spreadcycle.hend as u32;
            chopconf = (chopconf & !(0x7 << 4)) | ((hstrt & 0x7) << 4);
            chopconf = (chopconf & !(0xF << 7)) | ((hend & 0xF) << 7);
        }

        chopconf
    }

    fn calculate_mres(&self, microsteps: u16) -> u32 {
        match microsteps {
            256 => 0x0,
            128 => 0x1,
            64 => 0x2,
            32 => 0x3,
            16 => 0x4,
            8 => 0x5,
            4 => 0x6,
            2 => 0x7,
            1 => 0x8,
            _ => 0x4, // Default to 16 microsteps
        }
    }

    fn write_register(
        &mut self,
        name: &str,
        address: u8,
        value: u32,
    ) -> Result<(), Box<dyn Error>> {
        log::info!("Writing register {}: Address 0x{:02X}, Value: 0x{:08X}", name, address, value);
        
        match self.spi.write_register(address, value) {
            Ok(status) => {
                log::info!("Register {} write successful, SPI status: 0x{:02X}", name, status);
                Ok(())
            },
            Err(e) => {
                log::error!("Error writing register {}: {}", name, e);
                Err(e)
            }
        }
    }

    fn read_register(&mut self, name: &str, address: u8) -> Result<u32, Box<dyn Error>> {
        match self.spi.read_register(address) {
            Ok((status, value)) => {
                log::info!("Reading register {}: Status 0x{:02X}, Value: 0x{:08X}", name, status, value);
                Ok(value)
            },
            Err(e) => {
                log::error!("Error reading register {}: {}", name, e);
                Err(e)
            }
        }
    }

    fn enable(&mut self) -> Result<(), Box<dyn Error>> {
        self.config.base.pins.enable_pin.set_low()
    }

    fn disable(&mut self) -> Result<(), Box<dyn Error>> {
        self.config.base.pins.enable_pin.set_high()
    }

    fn move_steps_rmt(
        &mut self,
        steps: u32,
        rpm: f64,
    ) -> Result<(), Box<dyn Error>> {
        if let Some(ref mut driver) = self.tx_rmt_driver {
            let steps_per_rev = self.config.base.motor_params.steps_per_revolution as f64 
                * self.config.base.microsteps.microsteps as f64;
            let step_frequency = (rpm / 60.0) * steps_per_rev;
            let step_period_us = (1_000_000.0 / step_frequency) as u32;
            
            // Calculate pulse timing
            let pulse_width_ticks = PulseTicks::new((self.config.base.rmt.pulse_width_us as u16) * 10)?;
            let total_period_ticks = (step_period_us as u16) * 10;
            let low_period_ticks = total_period_ticks.saturating_sub((self.config.base.rmt.pulse_width_us as u16) * 10);
            let low_pulse_ticks = PulseTicks::new(low_period_ticks)?;
            
            // Create pulse pattern
            let high_pulse = Pulse::new(PinState::High, pulse_width_ticks);
            let low_pulse = Pulse::new(PinState::Low, low_pulse_ticks);
            
            // Generate steps using fixed length signals
            for _ in 0..steps {
                let mut signal = FixedLengthSignal::<1>::new();
                signal.set(0, &(high_pulse, low_pulse))?;
                driver.start_blocking(&signal)?;
            }
            
            log::info!("Successfully moved {} steps at {} RPM", steps, rpm);
            Ok(())
        } else {
            Err("RMT driver not initialized".into())
        }
    }
}

impl Tmc2240<'_> {
    fn log_driver_status(&mut self) -> Result<(), Box<dyn Error>> {
        let status = self.spi.read_register(REG_DRV_STATUS)?;
        
        log::info!("TMC2240 Driver Status: 0x{:08X}", status.1);
        
        // Check for errors
        if status.1 & (1 << 1) != 0 { // OT (overtemperature)
            log::warn!("Overtemperature detected!");
        }
        if status.1 & (1 << 0) != 0 { // OTPW (overtemperature pre-warning)
            log::warn!("Overtemperature pre-warning!");
        }
        if status.1 & (1 << 31) != 0 { // standstill
            log::info!("Motor is in standstill");
        }
        if status.1 & (1 << 30) != 0 { // StealthChop mode
            log::info!("Driver operating in StealthChop mode");
        } else {
            log::info!("Driver operating in SpreadCycle mode");
        }

        Ok(())
    }

    pub fn get_stallguard4_result(&mut self) -> Result<u16, Box<dyn Error>> {
        // TMC2240 has StallGuard4 result in DRV_STATUS register
        let status = self.spi.read_register(REG_DRV_STATUS)?;
        // Extract StallGuard4 result from bits [23:16] (example - adjust based on actual register layout)
        let sg4_result = ((status.1 >> 16) & 0xFF) as u16;
        Ok(sg4_result)
    }

    pub fn tune_stallguard4(&mut self) -> Result<(), Box<dyn Error>> {
        log::info!("Starting StallGuard4 tuning for TMC2240...");
        
        // Read current StallGuard4 result
        let sg4_result = self.get_stallguard4_result()?;
        log::info!("Current StallGuard4 result: {}", sg4_result);
        
        // Log current threshold
        let current_threshold = self.spi.read_register(REG_SG4_THRS)?;
        log::info!("Current StallGuard4 threshold: {}", current_threshold.1);
        
        Ok(())
    }

    pub fn enable_stallguard4(&mut self, threshold: u8) -> Result<(), Box<dyn Error>> {
        log::info!("Enabling StallGuard4 with threshold: {}", threshold);
        
        // Set StallGuard4 threshold
        self.spi.write_register(REG_SG4_THRS, threshold as u32)?;
        
        // Configure GCONF for StallGuard4 (if needed)
        let mut gconf = self.spi.read_register(REG_GCONF)?.1;
        gconf |= GCONF_DIAG1_STALL; // Enable stall output on DIAG1
        self.spi.write_register(REG_GCONF, gconf)?;
        
        log::info!("StallGuard4 enabled successfully");
        Ok(())
    }
} 