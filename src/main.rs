// src/main.rs (Complete File with Fixed Issues)

mod interfaces;
mod tmc_driver;

// Ensure necessary imports are present
use crate::interfaces::spi::EspSpiDevice;
use crate::tmc_driver::pin::PinSettings;
use crate::tmc_driver::tmc5160::{CoolStepConfig, DcStepConfig, TMC5160Features, Tmc5160};
use crate::interfaces::digital_pin::EspIdfDigitalOutputPin;

use esp_idf_hal::gpio::{AnyOutputPin, PinDriver};
use esp_idf_hal::spi::{config::Config, SpiDriver};
use tmc_driver::driver_settings::{CurrentSettings, MicrostepSettings, MotorParams, SpreadCycleSettings, StealthChopSettings};
use crate::tmc_driver::driver_settings::{RmtStepperConfig, TMCBaseConfig, TMCConfig, TMCDriverInterface};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::SpiDriverConfig;

use crate::tmc_driver::tmc2240::{Tmc2240, TMC2240Features, StallGuard4Config, AutoTuningConfig};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;

    log::info!("Starting TMC Driver Example - Supporting both TMC5160 and TMC2240");

    // Configure SPI
    let spi_driver = SpiDriver::new(
        peripherals.spi2,
        peripherals.pins.gpio18,
        peripherals.pins.gpio19,
        Some(peripherals.pins.gpio5),
        &SpiDriverConfig::new(),
    )?;

    // Example 1: TMC5160 Configuration
    log::info!("=== TMC5160 Example ===");
    {
        // Convert pins to AnyOutputPin for our custom pin types
        let cs_pin_any = AnyOutputPin::from(peripherals.pins.gpio21);
        let enable_pin_any = AnyOutputPin::from(peripherals.pins.gpio2);
        let dir_pin_any = AnyOutputPin::from(peripherals.pins.gpio4);
        let step_pin_any = AnyOutputPin::from(peripherals.pins.gpio16);

        // Create pin drivers
        let cs_pin_driver = PinDriver::output(cs_pin_any)?;
        let enable_pin_wrapper = EspIdfDigitalOutputPin::new(enable_pin_any);
        let dir_pin_wrapper = EspIdfDigitalOutputPin::new(dir_pin_any);

        // Create SPI device with manual CS control
        let spi_config = Config::new();
        let esp_spi = EspSpiDevice::new(spi_driver, &spi_config, cs_pin_driver);

        let pin_settings = PinSettings {
            enable_pin: Box::new(enable_pin_wrapper),
            dir_pin: Box::new(dir_pin_wrapper),
        };

        let base_config = TMCBaseConfig {
            rmt: RmtStepperConfig {
                rmt_clk_divider: 8,
                pulse_width_us: 2,
            },
            pins: pin_settings,
            current: CurrentSettings {
                run_current: 20,
                hold_current: 10,
                hold_delay: 6,
                global_scaler: 255,
            },
            microsteps: MicrostepSettings {
                microsteps: 16,
            },
            motor_params: MotorParams {
                steps_per_revolution: 200,
            },
            stealthchop: StealthChopSettings {
                enable: true,
                toff: 3,
            },
            spreadcycle: SpreadCycleSettings {
                enable: false,
                toff: 3,
                hstrt: 5,
                hend: 0,
            },
        };

        let tmc5160_features = TMC5160Features {
            coolstep: CoolStepConfig {
                enable: true,
                semin: 5,
                semax: 15,
                seup: 2,
                sedn: 1,
                sgthrs: 10,
            },
            dcstep: DcStepConfig {
                enable: false,
            },
        };

        let config = TMCConfig {
            base: base_config,
            feature: tmc5160_features,
        };

        let mut tmc5160 = Tmc5160::new(Box::new(esp_spi), config);
        
        // Initialize TMC5160 (use channel0 from rmt)
        tmc5160.init(peripherals.rmt.channel0, step_pin_any)?;
        
        // Demonstrate TMC5160 functionality
        log::info!("Moving TMC5160 motor 90 degrees at 60 RPM");
        tmc5160.rotate_by_angle(90.0, 60.0)?;
        
        log::info!("TMC5160 example completed successfully");
    }

    // Example 2: TMC2240 Configuration  
    log::info!("=== TMC2240 Example ===");
    {
        // Configure second SPI driver for TMC2240
        let spi_driver2 = SpiDriver::new(
            peripherals.spi3,
            peripherals.pins.gpio14,  // Different SPI pins
            peripherals.pins.gpio12,
            Some(peripherals.pins.gpio13),
            &SpiDriverConfig::new(),
        )?;

        // Convert pins to AnyOutputPin for TMC2240
        let cs_pin2_any = AnyOutputPin::from(peripherals.pins.gpio26);
        let enable_pin2_any = AnyOutputPin::from(peripherals.pins.gpio27);
        let dir_pin2_any = AnyOutputPin::from(peripherals.pins.gpio33);
        let step_pin2_any = AnyOutputPin::from(peripherals.pins.gpio32);
        
        // Create pin drivers
        let cs_pin2_driver = PinDriver::output(cs_pin2_any)?;
        let enable_pin2_wrapper = EspIdfDigitalOutputPin::new(enable_pin2_any);
        let dir_pin2_wrapper = EspIdfDigitalOutputPin::new(dir_pin2_any);
        
        // Create SPI device with manual CS control
        let spi_config2 = Config::new();
        let esp_spi2 = EspSpiDevice::new(spi_driver2, &spi_config2, cs_pin2_driver);

        let pin_settings2 = PinSettings {
            enable_pin: Box::new(enable_pin2_wrapper),
            dir_pin: Box::new(dir_pin2_wrapper),
        };

        let base_config2 = TMCBaseConfig {
            rmt: RmtStepperConfig {
                rmt_clk_divider: 8,
                pulse_width_us: 2,
            },
            pins: pin_settings2,
            current: CurrentSettings {
                run_current: 25,
                hold_current: 12,
                hold_delay: 8,
                global_scaler: 255,
            },
            microsteps: MicrostepSettings {
                microsteps: 32,
            },
            motor_params: MotorParams {
                steps_per_revolution: 200,
            },
            stealthchop: StealthChopSettings {
                enable: true,
                toff: 3,
            },
            spreadcycle: SpreadCycleSettings {
                enable: false,
                toff: 3,
                hstrt: 5,
                hend: 0,
            },
        };

        let tmc2240_features = TMC2240Features {
            stallguard4: StallGuard4Config {
                enable: true,
                angle_offset: false,
                threshold: 100,
            },
            autotuning: AutoTuningConfig {
                enable: true,
                pwm_reg: 4,
                pwm_ofs: 36,
                pwm_grad: 14,
            },
        };

        let config2 = TMCConfig {
            base: base_config2,
            feature: tmc2240_features,
        };

        let mut tmc2240 = Tmc2240::new(Box::new(esp_spi2), config2);
        
        // Initialize TMC2240 (use channel1 from rmt)
        tmc2240.init(peripherals.rmt.channel1, step_pin2_any)?;
        
        // Demonstrate TMC2240 functionality
        log::info!("Moving TMC2240 motor -45 degrees at 120 RPM");
        tmc2240.rotate_by_angle(-45.0, 120.0)?;
        
        // Demonstrate TMC2240-specific features
        tmc2240.enable_stallguard4(80)?;
        tmc2240.tune_stallguard4()?;
        
        log::info!("TMC2240 example completed successfully");
    }

    log::info!("Both TMC5160 and TMC2240 examples completed successfully!");
    log::info!("The architecture successfully supports multiple driver types with the same interface");

    Ok(())
}

// Helper function to demonstrate the common interface
fn demonstrate_common_interface<'a, T: TMCDriverInterface<'a, F>, F>(
    driver: &mut T,
    name: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    log::info!("=== Common Interface Demo for {} ===", name);
    
    // These methods work identically for both TMC5160 and TMC2240
    driver.enable()?;
    log::info!("{} enabled", name);
    
    driver.rotate_by_angle(180.0, 90.0)?;
    log::info!("{} rotated 180 degrees", name);
    
    driver.disable()?;
    log::info!("{} disabled", name);
    
    Ok(())
}