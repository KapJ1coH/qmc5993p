# qmc5883p

An asynchronous driver for the QMC5883P 3-Axis Magnetic Sensor using I2C. This
driver is designed for `no_std` environments and uses `embedded-hal-async`
for non-blocking I2C communication.

## Features

* **Asynchronous Interface**: Built for `async/await` using `embedded-hal-async` trait.
* **Platform-Agnostic**: Compatible with any microcontroller that implements the `embedded-hal-async` I2C traits.
* **no_std**: No need for the standard library.
* **Configurable Settings**: Full control over operating modes, output data
rates, measurement ranges, and oversampling ratios using a builder pattern.
* **Self-Test Support**: Includes a built-in self-test procedure to verify hardware functionality.
* **Magnitude Calculation**: Helper methods to read raw X, Y, Z data or calculate the total magnetic field magnitude.
* **Optional Logging**: Integrated with `defmt` for efficient logging in embedded environments.

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
qmc5883p = "0.1.0"

```

## Usage

### Basic Configuration

```rust
use qmc5883p::{Range, Mode, Qmc5883PConfig, OutputDataRate, OverSampleRate, OverSampleRatio1};

// Create a custom configuration
let config = Qmc5883PConfig::default()
    .with_mode(Mode::Continuous)
    .with_odr(OutputDataRate::Hz100)
    .with_range(Range::Gauss8)
    .with_osr1(OverSampleRatio1::Ratio4)
    .with_osr2(OverSampleRate::Rate4);

```

### Driver Initialization

```rust
// Initialize the sensor with your I2C peripheral
let mut sensor = Qmc5883p::new(i2c);
sensor.init(config).await?;

// Read magnetic field data
let [x, y, z] = sensor.read_x_y_z().await?;
let magnitude = sensor.read_magnitude().await?;

```
Look into tests for more usage examples.

## Supported Hardware

This driver is specifically for the **QMC5883P** magnetic sensor. It verifies the chip ID (expected `0x80`) during initialization.

## License

This project is licensed under either of:
* **Apache License, Version 2.0** ([LICENSE-APACHE](https://www.google.com/search?q=LICENSE-APACHE) or [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0))
* **MIT license** ([LICENSE-MIT](https://www.google.com/search?q=LICENSE-MIT) or [http://opensource.org/licenses/MIT](http://opensource.org/licenses/MIT))
at your option.
