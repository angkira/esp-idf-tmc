// src/main.rs (Complete File with Manual CS Setup)

mod interfaces;
mod tmc_driver;

// Ensure necessary imports are present
use crate::interfaces::spi::EspSpiDevice;
use crate::tmc_driver::pin::PinSettings;
use crate::tmc_driver::tmc5160::{CoolStepConfig, DcStepConfig, TMC5160Features, Tmc5160, };
use crate::interfaces::digital_pin::EspIdfDigitalOutputPin;
use crate::tmc_driver::traits::DigitalOutputPin; // Import the trait itself

use esp_idf_hal::gpio::{AnyOutputPin, PinDriver}; // Added PinDriver, Output
use esp_idf_hal::prelude::*;
use esp_idf_hal::spi::{config::{Config, DriverConfig, MODE_3}, Dma, SpiDriver};
use esp_idf_hal::delay::FreeRtos;
use std::error::Error;
use tmc_driver::driver_settings::{CurrentSettings, MicrostepSettings, MotorParams, SpreadCycleSettings, StealthChopSettings};
use crate::tmc_driver::driver_settings::{RmtStepperConfig, TMCBaseConfig, TMCConfig, TMCDriverInterface};


fn init_driver() -> Result<Box<Tmc5160<'static>>, Box<dyn Error>> {
    // Retrieve ESP‑IDF peripherals & pins
    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    log::info!("Configuring GPIO pins...");
    // --- Pin Setup ---
    let step_pin = AnyOutputPin::from(pins.gpio5);
    let dir_pin = AnyOutputPin::from(pins.gpio6);
    let enable_pin = AnyOutputPin::from(pins.gpio7);

    // Box the STEP/DIR/ENABLE pins for the Tmc5160Config
    // let step_pin_dyn = Box::new(EspIdfDigitalOutputPin::new(step_pin)) as Box<dyn DigitalOutputPin>;
    let dir_pin_dyn = Box::new(EspIdfDigitalOutputPin::new(dir_pin)) as Box<dyn DigitalOutputPin>;
    let mut enable_pin_dyn = Box::new(EspIdfDigitalOutputPin::new(enable_pin)) as Box<dyn DigitalOutputPin>;

    // --- CS PIN SETUP ---
    log::info!("Configuring CS pin (GPIO 10)...");
    let cs_pin = AnyOutputPin::from(pins.gpio10); // Assuming GPIO 10 for CS
    let cs_driver = PinDriver::output(cs_pin)?; // Create PinDriver for CS

    log::info!("Configuring SPI...");
    // Define SPI configuration
    let spi_config = Config::new()
        .baudrate(Hertz(1_000_000)) // 1 MHz Baudrate
        .data_mode(MODE_3);        // Mode 3 for TMC5160

    let spi_driver_config = DriverConfig::new()
        .dma(Dma::Auto(4096)); // Use DMA

    // Initialize SPI Driver (SPI2: SCK=gpio12, MOSI=gpio11, MISO=gpio13)
    let spi = SpiDriver::new(
        peripherals.spi2,
        pins.gpio12,      // SCLK
        pins.gpio11,      // MOSI (SDI on TMC5160)
        Some(pins.gpio13), // MISO (SDO on TMC5160)
        &spi_driver_config,
    )?;

    // --- Create EspSpiDevice WITH CS Pin ---
    let spi_device = Box::new(EspSpiDevice::new(spi, &spi_config, cs_driver));

    log::info!("Building TMC5160 configuration...");
    // Build Tmc5160 configuration
    let config:TMCConfig<TMC5160Features> = {
        let motor_params = MotorParams {
            steps_per_revolution: 200,
        };

        let current_settings = CurrentSettings {
            run_current: 20, // Example value
            hold_current: 10, // Example value
            hold_delay: 5, // Example value
            global_scaler: 100, // Example value
        };

        let microstep_settings = MicrostepSettings {
            microsteps: 4, // Example value
        };

        let stealthchop = StealthChopSettings {
            enable: false,
            toff: 5,
        };

        let spreadcycle = SpreadCycleSettings {
            enable: true,
            toff: 5,
            hstrt: 1,
            hend: 1,
        };

        let coolstep = CoolStepConfig {
            enable: true,
            semin: 2,
            semax: 10,
            seup: 1,
            sedn: 1,
            sgthrs: 3,
        };

        let dcstep = DcStepConfig {
            enable: true,
        };

        TMCConfig {
            base: TMCBaseConfig {
                rmt: RmtStepperConfig {
                    rmt_clk_divider: 80, // 80MHz / 80 = 1MHz
                    pulse_width_us: 5,
                },
                pins: PinSettings {
                    dir_pin: dir_pin_dyn,
                    enable_pin: enable_pin_dyn,
                },
                current: current_settings,
                microsteps: microstep_settings,
                motor_params,
                stealthchop,
                spreadcycle,
            },
            feature: TMC5160Features {
                coolstep,
                dcstep,
            },

        }
    };

    log::info!("Creating TMC5160 instance...");
    let mut tmc5160 = Tmc5160::new(spi_device, config);

    log::info!("Initializing TMC5160...");

    let rmt_channel = peripherals.rmt.channel0;
    // Init should now work with manual CS control handling SPI reads correctly
    match tmc5160.init(rmt_channel, step_pin) {
        Ok(_) => log::info!("TMC5160 Initialized successfully."),
        Err(e) => {
            log::error!("Error initializing TMC5160: {}", e);
            return Err(e); // Stop if init fails
        }
    }

    log::info!("Attempting rotation test...");

    // Test Rotation
    match tmc5160.rotate_by_angle(3600.0, 600.0) { // 360 degrees at 60 RPM
        Ok(_) => log::info!("Rotation command finished."),
        Err(e) => log::error!("Error during rotation: {}", e),
    }

    FreeRtos::delay_ms(1000); // Pause

    log::info!("Attempting reverse rotation test...");
    match tmc5160.rotate_by_angle(-3600.0, 600.0) {
        Ok(_) => log::info!("Reverse rotation command finished."),
        Err(e) => log::error!("Error during reverse rotation: {}", e),
    }

    log::info!("init_driver completed.");

    let tmc5160 = Box::new(tmc5160); // Ensure tmc_driver is boxed for dynamic dispatch

    Ok(tmc5160)
}

fn main() {
    // Required initialization steps
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Starting TMC5160 Test Application...");

    let mut driver: Box<Tmc5160> = init_driver().expect("Error initializing TMC5160 driver");

    FreeRtos::delay_ms(500);

    driver
        .rotate_by_angle(72000.0, 600.0)
        .expect("Error during rotation");

    log::info!("Application finished setup. Looping indefinitely.");
    // Loop indefinitely
    loop {
        FreeRtos::delay_ms(5000);
        log::info!("Main loop heartbeat...");
    }
}