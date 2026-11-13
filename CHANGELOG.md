# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

- Initial driver: voltage, current, state of charge and health, capacities, temperature,
  time to empty and full, cycle count and the battery status flags
- `status()` reads everything with one 56-byte I2C read (a single snapshot, one bus access)
- `device_number` and `firmware_version` through `Control()`: they poll for the answer and
  check the echoed subcommand, `MACDataLen()` and `MACDataSum()`; `Error::Timeout` if the
  gauge doesn't answer
- `read_word` and the `command` codes for the standard commands without a method
- All fifteen `BatteryStatus()` flags
- `Error<E>` (`I2c`, `Timeout`) with `Display` and `core::error::Error`
- `defmt` feature: `defmt::Format` for `Status`, `StatusFlags` and `Error`
