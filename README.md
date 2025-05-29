# ESP Stepper Motor Driver

A Rust library for controlling stepper motors on ESP32 microcontrollers using Trinamic TMC driver chips. This library provides a unified interface for multiple TMC driver variants while maintaining driver-specific features.

## Supported Drivers

### TMC5160
- **Voltage Range**: 8-60V
- **Current Rating**: Up to 4.4A RMS per coil
- **Features**:
  - StealthChop2 for silent operation
  - SpreadCycle for high-performance operation
  - CoolStep for energy-efficient current control
  - StallGuard2 for sensorless load detection
  - Integrated motion controller

### TMC2240  
- **Voltage Range**: 4.5-36V
- **Current Rating**: Up to 2.1A RMS per coil (3A peak)
- **Features**:
  - StealthChop2 with automatic tuning
  - SpreadCycle operation
  - StallGuard4 for enhanced stall detection
  - Automatic current scaling
  - Integrated current sensing (no external sense resistors needed)

## Architecture

The library uses a trait-based architecture that allows:

1. **Common Interface**: Both drivers implement `TMCDriverInterface<FeatureConfig>`
2. **Shared Base Configuration**: Common settings like current, microsteps, and motor parameters
3. **Driver-Specific Features**: Each driver has its own feature configuration:
   - TMC5160: `CoolStepConfig` and `DcStepConfig`
   - TMC2240: `StallGuard4Config` and `AutoTuningConfig`

## Usage

### Basic TMC5160 Example

```rust
use crate::tmc_driver::tmc5160::{Tmc5160, TMC5160Features, CoolStepConfig, DcStepConfig};

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

let mut tmc5160 = Tmc5160::new(Box::new(spi_device), config);
tmc5160.init(rmt_channel, step_pin)?;
tmc5160.rotate_by_angle(90.0, 60.0)?; // 90 degrees at 60 RPM
```

### Basic TMC2240 Example

```rust
use crate::tmc_driver::tmc2240::{Tmc2240, TMC2240Features, StallGuard4Config, AutoTuningConfig};

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

let config = TMCConfig {
    base: base_config,
    feature: tmc2240_features,
};

let mut tmc2240 = Tmc2240::new(Box::new(spi_device), config);
tmc2240.init(rmt_channel, step_pin)?;
tmc2240.rotate_by_angle(-45.0, 120.0)?; // -45 degrees at 120 RPM
```

## Key Features

### StealthChop2
- Ultra-quiet motor operation
- Automatic parameter tuning (TMC2240)
- Suitable for noise-sensitive applications

### SpreadCycle
- High-performance chopper for dynamic movements
- Excellent for high-speed applications
- Superior resonance dampening

### SPI Communication
- Hardware SPI interface
- Status monitoring and diagnostics
- Register read/write access

### ESP32 Integration
- Uses ESP-IDF HAL
- RMT peripheral for precise step generation
- GPIO abstraction for control pins

## Hardware Connections

### Common Connections (Both Drivers)
- **VCC_IO**: 3.3V or 5V logic supply
- **GND**: Ground connection
- **SPI Interface**:
  - SCK: SPI Clock
  - MOSI: SPI Data Out (SDI on TMC)
  - MISO: SPI Data In (SDO on TMC)  
  - CS: Chip Select (active low)
- **Motor Control**:
  - STEP: Step input
  - DIR: Direction input
  - ENN: Enable (active low)

### TMC5160 Specific
- **VS**: Motor supply voltage (8-60V)
- **Sense Resistors**: External current sensing resistors required

### TMC2240 Specific  
- **VS**: Motor supply voltage (4.5-36V)
- **Internal Sensing**: No external sense resistors needed (when configured)

## Getting Started

1. **Hardware Setup**: Connect your TMC driver according to the pinout
2. **Dependencies**: Add this crate to your `Cargo.toml`
3. **Configuration**: Create appropriate base and feature configurations
4. **Initialization**: Initialize the driver with SPI and pin setup
5. **Motor Control**: Use the common interface methods for motor control

## Contributing

Contributions are welcome! Please:
1. Follow Rust coding conventions
2. Add documentation for new features
3. Include examples for new driver support
4. Test with actual hardware when possible

## License

This project is licensed under the MIT License - see the LICENSE file for details. 