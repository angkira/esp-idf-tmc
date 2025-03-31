use crate::tmc_driver::driver_settings::{TMCConfig, TMCDriverInterface};
use crate::tmc_driver::traits::SpiDevice;
use esp_idf_hal::delay::Ets;
use std::error::Error;
use std::io::{self, Write};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::config::TransmitConfig;
use esp_idf_hal::rmt::{PinState, Pulse, PulseTicks, RmtChannel, TxRmtDriver, VariableLengthSignal}; // Import specific channel if known, or use generics
use log::info;

fn delay_us(micros: u32) {
    Ets::delay_us(micros);
}

fn delay_ms(millis: u32) {
    Ets::delay_ms(millis);
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

// Register definitions and constants remain unchanged
const REG_GCONF: u8 = 0x00;
const REG_GLOBAL_SCALER: u8 = 0x0B;
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

pub struct TMC5160Features {
    pub coolstep: CoolStepConfig,
    pub dcstep: DcStepConfig,
}

pub struct Tmc5160<'d> {
    spi: Box<dyn SpiDevice>,
    config: TMCConfig<'d, TMC5160Features>,
    tx_rmt_driver: Option<TxRmtDriver<'d>>,
}

impl TMCDriverInterface<TMC5160Features> for Tmc5160<'_> {
    fn new(
        spi: Box<dyn SpiDevice>,
        config: TMCConfig<TMC5160Features>,
    ) -> Tmc5160<'static> {
        Tmc5160 {
            spi,
            config,
            tx_rmt_driver: None,
        }
    }

    fn init(&mut self) -> Result<(), Box<dyn Error>> {
        self.enable()?;

        log::info!("Waiting 1000ms for power-up...");
        delay_ms(1000); // Power-up delay

        // --- GCONF ---
        let mut gconf = 0x00000000;
        // Set direction bit based on config (Example GCONF bit, adjust as needed)
        gconf |= GCONF_SHAFT; // Example: Set bit 2 for direction
        if self.config.base.stealthchop.enable {
            // Example: Enable stealthChop if configured
            // gconf |= GCONF_EN_PWM_MODE; // This bit doesn't exist directly, enable PWMCONF instead below
        }
        log::info!(
            "Writing GCONF: Register 0x{:02X}, Value: 0x{:08X}",
            REG_GCONF,
            gconf
        );
        match self.spi.write_register(REG_GCONF, gconf) {
            Ok(status) => log::info!("GCONF write successful, SPI status: 0x{:02X}", status),
            Err(e) => log::error!("Error writing GCONF: {}", e),
        }
        // Optional: Read back to verify
        match self.spi.read_register(REG_GCONF) {
            Ok((status, value)) => log::info!(
                "Reading GCONF: Status 0x{:02X}, Value: 0x{:08X}",
                status,
                value
            ),
            Err(e) => log::error!("Error reading GCONF: {}", e),
        }

        // --- GLOBALSCALER WRITE (Read from config) ---
        let global_scaler_val = self.config.base.current.global_scaler as u32; // Get from config
        log::info!(
            "Writing GLOBAL_SCALER: Register 0x{:02X}, Value: 0x{:08X}",
            REG_GLOBAL_SCALER,
            global_scaler_val
        );
        self.spi
            .write_register(REG_GLOBAL_SCALER, global_scaler_val)?;

        // --- IHOLD_IRUN ---
        let ihold_delay = 6u32; // Or use self.config.base.current.hold_delay if added to struct
        let ihold = self.config.base.current.hold_current as u32 & 0x1F; // Mask to 5 bits
        let irun = self.config.base.current.run_current as u32 & 0x1F; // Mask to 5 bits
        let ihold_irun = (ihold_delay << 16) | (irun << 8) | ihold;

        log::info!(
            "Writing IHOLD_IRUN: Register 0x{:02X}, Value: 0x{:08X}",
            REG_IHOLD_IRUN,
            ihold_irun
        );
        match self.spi.write_register(REG_IHOLD_IRUN, ihold_irun) {
            Ok(status) => log::info!("IHOLD_IRUN write successful, SPI status: 0x{:02X}", status),
            Err(e) => log::error!("Error writing IHOLD_IRUN: {}", e),
        }
        match self.spi.read_register(REG_IHOLD_IRUN) {
            Ok((status, value)) => log::info!(
                "Reading IHOLD_IRUN: Status 0x{:02X}, Value: 0x{:08X}",
                status,
                value
            ),
            Err(e) => log::error!("Error reading IHOLD_IRUN: {}", e),
        }

        // --- Other Registers (Using new write/read pattern) ---
        log::info!(
            "Writing SGTHRS: Register 0x{:02X}, Value: 0x{:08X}",
            REG_SGTHRS,
            self.config.feature.coolstep.sgthrs as u32
        );
        self.spi
            .write_register(REG_SGTHRS, self.config.feature.coolstep.sgthrs as u32)?;

        match self.spi.read_register(REG_SGTHRS) {
            Ok((status, value)) => log::info!(
                "Reading SGTHRS: Status 0x{:02X}, Value: 0x{:08X}",
                status,
                value
            ),
            Err(e) => log::error!("Error reading SGTHRS: {}", e),
        }

        log::info!(
            "Writing TCOOLTHRS: Register 0x{:02X}, Value: 0x{:08X}",
            REG_TCOOLTHRS,
            0x000000FF
        ); // Example value
        self.spi.write_register(REG_TCOOLTHRS, 0x000000FF)?;
        match self.spi.read_register(REG_TCOOLTHRS) {
            Ok((status, value)) => log::info!(
                "Reading TCOOLTHRS: Status 0x{:02X}, Value: 0x{:08X}",
                status,
                value
            ),
            Err(e) => log::error!("Error reading TCOOLTHRS: {}", e),
        }

        let chopconf_val = self.configure_chopconf();
        log::info!(
            "Writing CHOPCONF: Register 0x{:02X}, Value: 0x{:08X}",
            REG_CHOPCONF,
            chopconf_val
        );
        self.spi.write_register(REG_CHOPCONF, chopconf_val)?;
        match self.spi.read_register(REG_CHOPCONF) {
            Ok((status, value)) => log::info!(
                "Reading CHOPCONF: Status 0x{:02X}, Value: 0x{:08X}",
                status,
                value
            ),
            Err(e) => log::error!("Error reading CHOPCONF: {}", e),
        }

        let tpowerdown_val = 2u32; // Set delay to minimum (1 or 2 recommended)
        log::info!(
            "Writing TPOWERDOWN: Register 0x{:02X}, Value: 0x{:08X}",
            REG_TPOWERDOWN,
            tpowerdown_val
        );
        self.spi.write_register(REG_TPOWERDOWN, tpowerdown_val)?;

        // Configure PWMCONF based on stealthchop enable
        let mut pwmconf_val = 0x00000000; // Default if stealthchop disabled
        if self.config.base.stealthchop.enable {
            // Set appropriate PWMCONF bits for stealthchop based on datasheet/defaults
            // Example: Use the value from original log/code if known good for stealthchop
            pwmconf_val = 0x000401C8; // Default from datasheet PWMCONF reg description or your original code
                                      // Potentially configure bits like pwm_autoscale, pwm_autograd, pwm_freq etc.
                                      // pwmconf_val |= (1 << 18); // Example: enable pwm_autoscale
        }
        log::info!(
            "Writing PWMCONF: Register 0x{:02X}, Value: 0x{:08X}",
            REG_PWMCONF,
            pwmconf_val
        );
        self.spi.write_register(REG_PWMCONF, pwmconf_val)?;
        match self.spi.read_register(REG_PWMCONF) {
            Ok((status, value)) => log::info!(
                "Reading PWMCONF: Status 0x{:02X}, Value: 0x{:08X}",
                status,
                value
            ),
            Err(e) => log::error!("Error reading PWMCONF: {}", e),
        }

        let coolconf_val = if self.config.feature.coolstep.enable {
            self.configure_coolconf()
        } else {
            0x00000000
        };
        log::info!(
            "Writing COOLCONF: Register 0x{:02X}, Value: 0x{:08X}",
            REG_COOLCONF,
            coolconf_val
        );
        self.spi.write_register(REG_COOLCONF, coolconf_val)?;
        match self.spi.read_register(REG_COOLCONF) {
            Ok((status, value)) => log::info!(
                "Reading COOLCONF: Status 0x{:02X}, Value: 0x{:08X}",
                status,
                value
            ),
            Err(e) => log::error!("Error reading COOLCONF: {}", e),
        }

        // SW_MODE write removed as it wasn't being used in original code, uncomment if needed
        // let mut sw_mode = 0x00000000;
        // if self.config.base.dcstep.enable {
        //     // Set relevant bits for DCStep in SW_MODE if required
        //     // sw_mode |= (1 << ...); // Example bit
        // }
        // log::info!("Writing SW_MODE: Register 0x{:02X}, Value: 0x{:08X}", REG_SW_MODE, sw_mode);
        // self.spi.write_register(REG_SW_MODE, sw_mode)?;
        // match self.spi.read_register(REG_SW_MODE) {
        //      Ok((status, value)) => log::info!("Reading SW_MODE: Status 0x{:02X}, Value: 0x{:08X}", status, value),
        //      Err(e) => log::error!("Error reading SW_MODE: {}", e),
        // }

        log::info!("Waiting 100ms after initialization...");
        delay_ms(100); // Post-initialization delay

        // Final status checks
        self.log_driver_status()?;

        match self.spi.read_register(REG_IOIN) {
            Ok((status, value)) => log::info!(
                "Reading IOIN: Status 0x{:02X}, Value: 0x{:08X}",
                status,
                value
            ),
            Err(e) => log::error!("Error reading IOIN: {}", e),
        }

        Ok(())
    }

    fn init_rmt<'d>(&mut self) -> Result<(), Box<dyn Error>> {
        let peripherals = Peripherals::take()?;
        let rmt = peripherals.rmt;
        let rmt_config = &self.config.base.rmt;
        info!(
            "Initializing RMT Stepper on Channel {}...",
            rmt_config.step_rmt_channel
        );

        let tx_config = TransmitConfig::new().clock_divider(rmt_config.rmt_clk_divider);

        let channel = match &rmt_config.step_rmt_channel {
            0 => &rmt.channel0,
            1 => &rmt.channel1,
            2 => &rmt.channel2,
            3 => &rmt.channel3,
            4 => &rmt.channel4,
            5 => &rmt.channel5,
            6 => &rmt.channel6,
            7 => &rmt.channel7,
            _ => panic!("Invalid RMT channel")
        };

        // Create the RMT Transmit Driver
        let tx_driver = TxRmtDriver::new(
            &rmt,                         // RMT peripheral
            channel, // The selected RMT channel
            &tx_config,                   // RMT transmit configuration
        )?;

        let rmt_clk_hz = tx_driver.counter_clock()?;
        info!("RMT counter clock configured to: {} Hz", rmt_clk_hz.0);
    }

    fn rotate_by_angle(&mut self, angle: f64, rpm: f64) -> Result<(), Box<dyn Error>> {
        let steps_per_revolution = self.config.base.motor_params.steps_per_revolution as f64;
        let microsteps = self.config.base.microsteps.microsteps as f64;
        let total_steps = (angle / 360.0 * steps_per_revolution * microsteps).round() as i32;
        let steps = total_steps.abs();

        self.log_driver_status()?;

        let period_micros = 60.0 * 1_000_000.0 / (rpm * steps_per_revolution * microsteps);
        let pulse_width_micros = self.config.base.rmt.pulse_width_us as f64;
        let delay_micros = if period_micros > pulse_width_micros as f64 {
            period_micros - pulse_width_micros as f64
        } else {
            0.0
        };

        delay_us(100);

        let dir_pin = &mut self.config.base.pins.dir_pin;

        if total_steps > 0 {
            dir_pin.set_high().expect("Error setting dir pin high");
        } else {
            dir_pin.set_low().expect("Error setting dir pin low");
        }

        delay_us(100);

        self.move_steps_rmt(steps.abs() as u32, rpm)?;

        Ok(())
    }

    fn move_steps_rmt(
        &mut self,
        steps: u32,
        rpm: f64,
        // Direction is now handled separately/before calling
    ) -> Result<(), Box<dyn Error>> {
        let steps_per_revolution = self.config.base.motor_params.steps_per_revolution as f64;
        let microsteps_per_step = self.config.base.microsteps.microsteps as f64; // Assuming this is microsteps per FULL step

        // Calculate period in nanoseconds for higher precision with RMT ticks
        let period_ns = (60.0 * 1_000_000_000.0
            / (rpm * steps_per_revolution * microsteps_per_step))
            .max(1000.0); // Min period 1000ns = 1us

        // Define pulse width (e.g., 2-5 us) in nanoseconds
        let pulse_width_ns: u32 = 2_000; // 2 microseconds = 2000 nanoseconds

        // Calculate delay between pulses in nanoseconds
        let delay_ns = if period_ns > pulse_width_ns as f64 {
            (period_ns - pulse_width_ns as f64).max(2000.0) as u32 // Ensure minimum delay (e.g., 2us)
        } else {
            2_000 // Minimum delay if period is very short
        };

        let mut tx_driver = self.tx_rmt_driver.as_mut().unwrap();

        let ticks_hz = &tx_driver.counter_clock()?; // Get the actual RMT counter clock frequency
        let ns_per_tick = 1_000_000_000u64 / ticks_hz.0 as u64;

        let high_ticks = (pulse_width_ns as u64 / ns_per_tick) as u16;
        let low_ticks = (delay_ns as u64 / ns_per_tick) as u16;

        let high_ticks = PulseTicks::new(high_ticks)?;
        let low_ticks = PulseTicks::new(low_ticks)?;

        // Define the pulse sequence for a single step (HIGH then LOW)
        let pulses = [
            Pulse::new(PinState::High, high_ticks), // Step pulse HIGH
            Pulse::new(PinState::Low, low_ticks),   // Delay before next step LOW
        ];

        // Create a signal containing the pulse repeated 'steps' times
        // Note: RMT buffer size is limited. Very large 'steps' might require chunking.
        // The default buffer size (usually 64 items * number of memory blocks) limits the
        // number of *Pulse* elements (high + low = 2 elements per step) you can send at once.
        // For 1 memory block, this is ~32 steps per transmission.
        // We will send pulses iteratively for simplicity and robustness with large step counts.

        let mut signal = VariableLengthSignal::new();
        &signal.push(&pulses)?;
        // Send pulses iteratively
        for i in 0..steps {
            // Use start_blocking to send the two pulses for one step and wait for completion
            &tx_driver.start_blocking(&signal)?;
            if (i + 1) % 100 == 0 { // Optional: Log progress for long moves
                log::debug!("Sent {} steps...", i + 1);
            }
        }
        log::info!("RMT transmission finished.");

        Ok(())
    }

    // In src/tmc_driver/tmc_driver.rs
    fn configure_chopconf(&self) -> u32 {
        let mut chopconf: u32 = 0;
        let toff = if self.config.base.stealthchop.enable {
            self.config.base.stealthchop.toff.clamp(2, 15) // Ensure TOFF >= 2 if TBL=1
        } else {
            self.config.base.spreadcycle.toff.clamp(2, 15) // Ensure TOFF >= 2 if TBL=1
        };
        // --- CORRECTED LINE ---
        chopconf |= toff as u32; // Cast toff to u32 for bitwise OR
                                 // --- END CORRECTION ---

        // Microstep Resolution (MRES) bits 27..24
        let mres = self.calculate_mres(self.config.base.microsteps.microsteps);
        chopconf |= mres << 24; // Use correct shift amount

        // Blank time (TBL) bits 16..15 - Setting TBL=%01 (24 clocks)
        chopconf |= 1 << 15; // Set bit 15 for TBL = %01

        if self.config.base.spreadcycle.enable && !self.config.base.stealthchop.enable {
            // SpreadCycle specific bits
            chopconf |= (self.config.base.spreadcycle.hstrt as u32 & 0x07) << 4; // HSTRT bits 6..4
            chopconf |= (self.config.base.spreadcycle.hend as u32 & 0x0F) << 7; // HEND bits 10..7
                                                                                // CHM bit 14 = 0 for SpreadCycle (default)
        } else if self.config.base.stealthchop.enable {
            // StealthChop doesn't use HSTRT/HEND in CHOPCONF
            // CHM bit 14 = 0 (default)
        } else {
            // Default to SpreadCycle settings if neither specifically enabled
            chopconf |= (4u32 & 0x07) << 4; // Default HSTRT=4
            chopconf |= (1u32 & 0x0F) << 7; // Default HEND=1
        }

        // Interpolation (intpol) bit 28 - Enable for smoother steps
        chopconf |= 1 << 28;

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

    fn write_register(
        &mut self,
        name: &str,
        address: u8,
        value: u32,
    ) -> Result<(), Box<dyn Error>> {
        log::info!(
            "Writing {}: Register 0x{:02X}, Value: 0x{:08X}",
            name,
            address,
            value
        );

        // Call the underlying SpiDevice trait method
        match self.spi.write_register(address, value) {
            Ok(status) => {
                // Log the status byte received during the write
                log::debug!("Write {}: Success, SPI status: 0x{:02X}", name, status);
                // You could potentially check status bits here if needed
                Ok(())
            }
            Err(e) => {
                log::error!("SPI Error writing {}: {}", name, e);
                Err(e) // Propagate the error
            }
        }
    }

    /// Helper method to read a register via the SPI device.
    /// Logs the operation and handles potential errors.
    fn read_register(&mut self, name: &str, address: u8) -> Result<u32, Box<dyn Error>> {
        // Call the underlying SpiDevice trait method, which handles the two-step read
        match self.spi.read_register(address) {
            Ok((status, value)) => {
                log::info!(
                    "Reading {}: Register 0x{:02X}, SPI Status: 0x{:02X}, Value: 0x{:08X}",
                    name,
                    address,
                    status,
                    value
                );
                // You could potentially check status bits here if needed
                Ok(value) // Return the read value
            }
            Err(e) => {
                log::error!("SPI Error reading {}: {}", name, e);
                Err(e) // Propagate the error
            }
        }
    }

    fn enable(&mut self) -> Result<(), Box<dyn Error>> {
        self.config.base.pins.enable_pin.set_low()
    }

    fn disable(&mut self) -> Result<(), Box<dyn Error>> {
        self.config.base.pins.enable_pin.set_high()
    }
}

impl Tmc5160<'_> {
    fn log_driver_status(&mut self) -> Result<(), Box<dyn Error>> {
        match self.spi.read_register(REG_DRV_STATUS) {
            Ok((status, drv_status_val)) => {
                log::info!(
                    "Reading DRV_STATUS: Status 0x{:02X}, Value: 0x{:08X}",
                    status,
                    drv_status_val
                );
                // Optionally decode and log specific flags from drv_status_val
                if (drv_status_val >> 25) & 1 == 1 {
                    log::warn!("Overtemperature Shutdown!");
                }
                if (drv_status_val >> 26) & 1 == 1 {
                    log::warn!("Overtemperature Pre-Warning!");
                }
                if (drv_status_val >> 27) & 1 == 1 {
                    log::warn!("Short to GND A!");
                }
                if (drv_status_val >> 28) & 1 == 1 {
                    log::warn!("Short to GND B!");
                }
                if (drv_status_val >> 12) & 1 == 1 {
                    log::warn!("Short to VS A!");
                }
                if (drv_status_val >> 13) & 1 == 1 {
                    log::warn!("Short to VS B!");
                }
                if (drv_status_val >> 29) & 1 == 1 {
                    log::info!("Open Load A");
                }
                if (drv_status_val >> 30) & 1 == 1 {
                    log::info!("Open Load B");
                }
                if (drv_status_val >> 31) & 1 == 1 {
                    log::info!("Standstill detected");
                }
                Ok(())
            }
            Err(e) => {
                log::error!("Error reading DRV_STATUS: {}", e);
                Err(e)
            }
        }
    }

    fn configure_coolconf(&self) -> u32 {
        let mut coolconf: u32 = 0;
        coolconf |= (self.config.feature.coolstep.semin as u32) << 0;
        coolconf |= (self.config.feature.coolstep.semax as u32) << 5;
        coolconf |= (self.config.feature.coolstep.seup as u32) << 10;
        coolconf |= (self.config.feature.coolstep.sedn as u32) << 12;
        coolconf
    }

    pub fn tune_sgthrs(&mut self) -> Result<(), Box<dyn Error>> {
        let original_coolstep_enable = self.config.feature.coolstep.enable;
        let original_dcstep_enable = self.config.feature.dcstep.enable;
        self.config.feature.coolstep.enable = false;
        self.config.feature.dcstep.enable = false;
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
                    self.config.feature.coolstep.sgthrs = sgthrs;
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

        self.config.feature.coolstep.enable = original_coolstep_enable;
        self.config.feature.dcstep.enable = original_dcstep_enable;
        if self.config.feature.coolstep.enable {
            self.write_register("COOLCONF", REG_COOLCONF, self.configure_coolconf())?;
        }
        if self.config.feature.dcstep.enable {
            let mut sw_mode = 0x00000000;
            sw_mode |= 0x00000400;
            self.write_register("SW_MODE", REG_SW_MODE, sw_mode)?;
        }
        Ok(())
    }
}
