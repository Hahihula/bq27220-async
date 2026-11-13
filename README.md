# bq27220-async

[![Crates.io](https://img.shields.io/crates/v/bq27220-async?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/bq27220-async)
[![docs.rs](https://img.shields.io/docsrs/bq27220-async?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/bq27220-async)
![Crates.io](https://img.shields.io/crates/l/bq27220-async?labelColor=1C2C2E&style=flat-square)

`no_std` async driver for the Texas Instruments BQ27220 battery fuel gauge, built on
[`embedded-hal-async`](https://crates.io/crates/embedded-hal-async).

- Voltage, current (signed: positive while charging), average current and power
- State of charge (relative to the full charge capacity), state of health, remaining,
  full-charge and design capacity
- Temperature in 0.1 °C, time to empty and time to full (`None` when not applicable)
- Cycle count and all the `BatteryStatus()` flags
- Everything in one 56-byte I2C read, `status()`: one bus access, one consistent snapshot
- Device number and firmware version through `Control()`; the driver waits for the answer
  and checks the echoed subcommand, the length and the checksum
- Any other standard command with `read_word` and the `command` codes
- Default I2C address `0x55`
- Optional `defmt` feature: `defmt::Format` for `Status`, `StatusFlags` and `Error`

Not supported: configuring the gauge (battery capacity, chemistry, calibration), the data
memory, and the sealing and reset commands. The gauge keeps its configuration in its own
flash; this driver only reads.

`Bq27220::new` takes any `embedded-hal-async` I2C bus by value; pass `&mut bus` to keep using
the bus afterwards, or a bus-sharing device (e.g. `embassy-embedded-hal`'s `I2cDevice`) to
share it. The `Control()` calls poll for their answer and take any `embedded-hal-async`
`DelayNs`, for example `embassy_time::Delay`.

Tested on the ESP-VoCat v1.2, with a 700 mAh cell and a 10 mΩ sense resistor.

## Example

```rust
use bq27220_async::{Bq27220, DEFAULT_ADDRESS, DEVICE_NUMBER, Error};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

async fn read<I: I2c, D: DelayNs>(i2c: I, delay: &mut D) -> Result<(), Error<I::Error>> {
    let mut gauge = Bq27220::new(i2c, DEFAULT_ADDRESS);
    assert_eq!(gauge.device_number(delay).await?, DEVICE_NUMBER);

    let status = gauge.status().await?;
    // status.current_ma is negative while the battery discharges.
    let _ = (
        status.voltage_mv,
        status.current_ma,
        status.state_of_charge_pct,
        status.time_to_empty_min,
    );
    Ok(())
}
```

The TRM asks hosts not to read the standard commands more than twice a second, so poll
`status()` every second or two at most.

## [Documentation]

[documentation]: https://docs.rs/bq27220-async/

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
