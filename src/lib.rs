//! `no_std` async driver for the Texas Instruments BQ27220 battery fuel gauge.
//!
//! The BQ27220 measures the voltage, current and temperature of a single-cell Li-ion
//! battery and runs TI's CEDV algorithm for the state of charge, the state of health
//! and the time estimates. This driver reads the standard commands, which are 16-bit
//! little-endian registers, over any `embedded-hal-async` I2C bus. It does not
//! configure the gauge (battery capacity, chemistry, calibration): the gauge keeps
//! its settings in its own flash.
//!
//! ```no_run
//! use bq27220_async::{Bq27220, DEFAULT_ADDRESS, DEVICE_NUMBER, Error};
//! use embedded_hal_async::{delay::DelayNs, i2c::I2c};
//!
//! async fn read<I: I2c, D: DelayNs>(i2c: I, delay: &mut D) -> Result<(), Error<I::Error>> {
//!     let mut gauge = Bq27220::new(i2c, DEFAULT_ADDRESS);
//!     assert_eq!(gauge.device_number(delay).await?, DEVICE_NUMBER);
//!     let status = gauge.status().await?;
//!     let _ = (status.voltage_mv, status.current_ma, status.state_of_charge_pct);
//!     Ok(())
//! }
//! ```
//!
//! # Units and signs
//!
//! Current is positive while charging and negative while discharging. Power follows the
//! same sign. Temperature is in 0.1 °C. Times are in minutes, or `None` when the gauge
//! reports 0xFFFF (not discharging, or not charging).
//!
//! # One read for everything
//!
//! [`Bq27220::status`] reads all values with a single 56-byte I2C read, so the values come
//! from one snapshot and the bus is taken once, which matters on a shared bus. The gauge
//! supports incremental reads (TRM, SLUUBD4A section 5.1).
//!
//! It is one `write_read`, not several write/read pairs in one transaction: the gauge needs
//! a bus-free time between packets (section 5.3) and answered a repeated start after a read
//! with a NACK when tested on an ESP-VoCat. Also keep in mind that the TRM asks hosts not
//! to read the standard commands more than twice a second.
//!
//! # `Control()` commands
//!
//! [`Bq27220::device_number`] and [`Bq27220::firmware_version`] write a subcommand and read
//! the answer from `MACData()`. The driver waits until the gauge has put the answer there:
//! it re-reads the block from `ManufacturerAccessControl()` to `MACDataSum()` until the
//! echoed subcommand, the length and the checksum all match, and gives up with
//! [`Error::Timeout`] after about 20 ms. This follows the TRM's advice to verify the
//! active subcommand.
//!
//! # Sharing the bus
//!
//! Pass the bus by `&mut` (or an `embedded-hal-bus` device) to share it with other
//! drivers.
//!
//! # Not covered
//!
//! The data memory, calibration, sealing and every `Control()` subcommand that changes
//! state. Use [`Bq27220::read_word`] and the [`command`] codes for standard commands the
//! driver has no method for, such as `AtRate()` or `InternalTemperature()`.

#![no_std]
#![warn(missing_docs)]

use core::fmt;

use embedded_hal_async::{delay::DelayNs, i2c::I2c};

/// The BQ27220's 7-bit I2C address.
pub const DEFAULT_ADDRESS: u8 = 0x55;

/// The answer of a BQ27220 to the `DEVICE_NUMBER()` subcommand, see
/// [`Bq27220::device_number`].
pub const DEVICE_NUMBER: u16 = 0x0220;

/// Standard command codes (register addresses), for [`Bq27220::read_word`]. All are
/// 16-bit little-endian values at this address and the next.
pub mod command {
    /// `Control()`.
    pub const CONTROL: u8 = 0x00;
    /// `AtRate()`, mA (read-write).
    pub const AT_RATE: u8 = 0x02;
    /// `AtRateTimeToEmpty()`, minutes.
    pub const AT_RATE_TIME_TO_EMPTY: u8 = 0x04;
    /// `Temperature()`, 0.1 K.
    pub const TEMPERATURE: u8 = 0x06;
    /// `Voltage()`, mV.
    pub const VOLTAGE: u8 = 0x08;
    /// `BatteryStatus()`, see [`StatusFlags`](crate::StatusFlags).
    pub const BATTERY_STATUS: u8 = 0x0A;
    /// `Current()`, mA, signed.
    pub const CURRENT: u8 = 0x0C;
    /// `RemainingCapacity()`, mAh.
    pub const REMAINING_CAPACITY: u8 = 0x10;
    /// `FullChargeCapacity()`, mAh.
    pub const FULL_CHARGE_CAPACITY: u8 = 0x12;
    /// `AverageCurrent()`, mA, signed.
    pub const AVERAGE_CURRENT: u8 = 0x14;
    /// `TimeToEmpty()`, minutes; 0xFFFF while not discharging.
    pub const TIME_TO_EMPTY: u8 = 0x16;
    /// `TimeToFull()`, minutes; 0xFFFF while not charging.
    pub const TIME_TO_FULL: u8 = 0x18;
    /// `StandbyCurrent()`, mA, signed.
    pub const STANDBY_CURRENT: u8 = 0x1A;
    /// `MaxLoadCurrent()`, mA, signed.
    pub const MAX_LOAD_CURRENT: u8 = 0x1E;
    /// `AveragePower()`, mW, signed.
    pub const AVERAGE_POWER: u8 = 0x24;
    /// `InternalTemperature()`, 0.1 K.
    pub const INTERNAL_TEMPERATURE: u8 = 0x28;
    /// `CycleCount()`.
    pub const CYCLE_COUNT: u8 = 0x2A;
    /// `RelativeStateOfCharge()`, %.
    pub const RELATIVE_STATE_OF_CHARGE: u8 = 0x2C;
    /// `StateOfHealth()`, %.
    pub const STATE_OF_HEALTH: u8 = 0x2E;
    /// `ChargeVoltage()`, mV.
    pub const CHARGE_VOLTAGE: u8 = 0x30;
    /// `ChargeCurrent()`, mA.
    pub const CHARGE_CURRENT: u8 = 0x32;
    /// `OperationStatus()`.
    pub const OPERATION_STATUS: u8 = 0x3A;
    /// `DesignCapacity()`, mAh.
    pub const DESIGN_CAPACITY: u8 = 0x3C;
}

/// `ManufacturerAccessControl()` and the `MACData()` block up to `MACDataSum()` and
/// `MACDataLen()`.
mod mac {
    /// First register of the block: the 2-byte subcommand that is active.
    pub const CONTROL: u8 = 0x3E;
    /// Number of registers from `CONTROL` to `MACDataLen()`.
    pub const BLOCK_LEN: usize = 0x62 - 0x3E;
    /// Offset of `MACDataSum()` in the block.
    pub const SUM: usize = 0x60 - 0x3E;
    /// Offset of `MACDataLen()` in the block.
    pub const LEN: usize = 0x61 - 0x3E;
    /// Offset of the first data byte, after the 2-byte echo of the subcommand.
    pub const DATA: usize = 2;
}

/// `Control()` subcommands that read something.
mod sub {
    pub const GET_DEVICE_NUMBER: u16 = 0x0001;
    pub const GET_FW_VERSION: u16 = 0x0002;
}

/// Pause between asking for a `Control()` answer and (re-)reading it, in ms.
const CONTROL_POLL_MS: u32 = 2;
/// How often the answer is read before giving up: about 20 ms in all.
const CONTROL_POLLS: usize = 10;

/// The registers [`Bq27220::status`] reads in one go: from `Temperature()` (0x06) to the
/// end of `DesignCapacity()` (0x3D).
const STATUS_FIRST: u8 = command::TEMPERATURE;
const STATUS_BLOCK_LEN: usize = (command::DESIGN_CAPACITY as usize + 2) - STATUS_FIRST as usize;

/// What the gauge reports for the temperature, in 0.1 K, at 0 °C.
const ZERO_CELSIUS_DK: i32 = 2731;

/// Driver errors, generic over the I2C error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error<E> {
    /// An I2C transfer failed.
    I2c(E),
    /// The gauge did not give the answer to a `Control()` subcommand in time: the
    /// `MACData()` block never echoed the subcommand with a valid length and checksum.
    Timeout,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Self::I2c(e)
    }
}

impl<E: fmt::Debug> fmt::Display for Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::I2c(error) => write!(f, "I2C error: {error:?}"),
            Self::Timeout => f.write_str("no valid answer to the Control() subcommand"),
        }
    }
}

impl<E: fmt::Debug> core::error::Error for Error<E> {}

/// `BatteryStatus()` flags (SLUUBD4A, table 2-6).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StatusFlags(pub u16);

impl StatusFlags {
    fn bit(self, n: u32) -> bool {
        self.0 & (1 << n) != 0
    }

    /// `DSG`: discharging (clear while charging or relaxing).
    #[must_use]
    pub fn discharging(self) -> bool {
        self.bit(0)
    }
    /// `SYSDWN`: the system should shut down.
    #[must_use]
    pub fn system_down(self) -> bool {
        self.bit(1)
    }
    /// `TDA`: terminate-discharge alarm.
    #[must_use]
    pub fn terminate_discharge_alarm(self) -> bool {
        self.bit(2)
    }
    /// `BATTPRES`: battery detected.
    #[must_use]
    pub fn battery_present(self) -> bool {
        self.bit(3)
    }
    /// `AUTH_GD`: an inserted battery was detected.
    #[must_use]
    pub fn battery_inserted(self) -> bool {
        self.bit(4)
    }
    /// `OCVGD`: a good open-circuit-voltage measurement was taken.
    #[must_use]
    pub fn ocv_good(self) -> bool {
        self.bit(5)
    }
    /// `TCA`: terminate-charge alarm.
    #[must_use]
    pub fn terminate_charge_alarm(self) -> bool {
        self.bit(6)
    }
    /// `CHGINH`: charging inhibited, the temperature is outside the charge range.
    #[must_use]
    pub fn charge_inhibit(self) -> bool {
        self.bit(8)
    }
    /// `FC`: full charge detected.
    #[must_use]
    pub fn full_charge(self) -> bool {
        self.bit(9)
    }
    /// `OTD`: over-temperature in discharge.
    #[must_use]
    pub fn over_temp_discharge(self) -> bool {
        self.bit(10)
    }
    /// `OTC`: over-temperature in charge.
    #[must_use]
    pub fn over_temp_charge(self) -> bool {
        self.bit(11)
    }
    /// `SLEEP`: the gauge is in SLEEP mode.
    #[must_use]
    pub fn sleeping(self) -> bool {
        self.bit(12)
    }
    /// `OCVFAIL`: an open-circuit-voltage reading failed because of current.
    #[must_use]
    pub fn ocv_failed(self) -> bool {
        self.bit(13)
    }
    /// `OCVCOMP`: an open-circuit-voltage update is complete.
    #[must_use]
    pub fn ocv_complete(self) -> bool {
        self.bit(14)
    }
    /// `FD`: fully discharged.
    #[must_use]
    pub fn fully_discharged(self) -> bool {
        self.bit(15)
    }
}

/// A full set of gauge readings, from [`Bq27220::status`].
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    /// Battery voltage in mV.
    pub voltage_mv: u16,
    /// Current in mA: positive while charging, negative while discharging.
    pub current_ma: i16,
    /// Current averaged by the gauge over about a minute, in mA.
    pub average_current_ma: i16,
    /// Power averaged by the gauge, in mW.
    pub average_power_mw: i16,
    /// `RelativeStateOfCharge()` in %: the remaining capacity as a share of the full
    /// charge capacity (so it reads 100 % on an aged battery that is full).
    pub state_of_charge_pct: u16,
    /// `StateOfHealth()` in %: the full charge capacity as a share of the design capacity.
    pub state_of_health_pct: u16,
    /// Remaining capacity in mAh.
    pub remaining_mah: u16,
    /// Capacity when fully charged, in mAh.
    pub full_charge_mah: u16,
    /// Design capacity, in mAh.
    pub design_mah: u16,
    /// Temperature in 0.1 °C.
    pub temperature_dc: i16,
    /// Minutes until empty at the average current; `None` while not discharging.
    pub time_to_empty_min: Option<u16>,
    /// Minutes until full at the average current; `None` while not charging.
    pub time_to_full_min: Option<u16>,
    /// Number of charge cycles.
    pub cycle_count: u16,
    /// The `BatteryStatus()` flags.
    pub flags: StatusFlags,
}

impl Status {
    /// Decodes the registers 0x06 to 0x3D, read in one go.
    fn from_block(block: &[u8; STATUS_BLOCK_LEN]) -> Self {
        let word = |command: u8| {
            let i = usize::from(command - STATUS_FIRST);
            u16::from_le_bytes([block[i], block[i + 1]])
        };
        let signed = |command: u8| word(command).cast_signed();
        let minutes = |v: u16| (v != 0xFFFF).then_some(v);
        Self {
            voltage_mv: word(command::VOLTAGE),
            current_ma: signed(command::CURRENT),
            average_current_ma: signed(command::AVERAGE_CURRENT),
            average_power_mw: signed(command::AVERAGE_POWER),
            state_of_charge_pct: word(command::RELATIVE_STATE_OF_CHARGE),
            state_of_health_pct: word(command::STATE_OF_HEALTH),
            remaining_mah: word(command::REMAINING_CAPACITY),
            full_charge_mah: word(command::FULL_CHARGE_CAPACITY),
            design_mah: word(command::DESIGN_CAPACITY),
            temperature_dc: temperature_dc(word(command::TEMPERATURE)),
            time_to_empty_min: minutes(word(command::TIME_TO_EMPTY)),
            time_to_full_min: minutes(word(command::TIME_TO_FULL)),
            cycle_count: word(command::CYCLE_COUNT),
            flags: StatusFlags(word(command::BATTERY_STATUS)),
        }
    }
}

/// BQ27220 driver over any async I2C bus.
pub struct Bq27220<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C: I2c> Bq27220<I2C> {
    /// Wraps the bus; talks to the gauge at `address`, normally [`DEFAULT_ADDRESS`].
    /// Nothing is sent yet.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Gives the bus back.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Reads a standard command, a 16-bit little-endian value at `command` and the next
    /// address. See the [`command`] codes; the TRM (SLUUBD4A, table 2-1) lists them all.
    ///
    /// # Errors
    ///
    /// If the I2C transfer fails.
    pub async fn read_word(&mut self, command: u8) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(self.address, &[command], &mut buf)
            .await?;
        Ok(u16::from_le_bytes(buf))
    }

    /// Writes a `Control()` subcommand and waits for its answer in `MACData()`, returning
    /// the first data word.
    async fn control(
        &mut self,
        subcommand: u16,
        delay: &mut impl DelayNs,
    ) -> Result<u16, Error<I2C::Error>> {
        let [lo, hi] = subcommand.to_le_bytes();
        self.i2c
            .write(self.address, &[command::CONTROL, lo, hi])
            .await?;
        for _ in 0..CONTROL_POLLS {
            delay.delay_ms(CONTROL_POLL_MS).await;
            let mut block = [0u8; mac::BLOCK_LEN];
            self.i2c
                .write_read(self.address, &[mac::CONTROL], &mut block)
                .await?;
            if let Some(word) = mac_answer(&block, subcommand) {
                return Ok(word);
            }
        }
        Err(Error::Timeout)
    }

    /// Reads the device number; [`DEVICE_NUMBER`] for a BQ27220.
    ///
    /// # Errors
    ///
    /// If an I2C transfer fails, or [`Error::Timeout`] if the gauge doesn't answer.
    pub async fn device_number(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<u16, Error<I2C::Error>> {
        self.control(sub::GET_DEVICE_NUMBER, delay).await
    }

    /// Reads the firmware version.
    ///
    /// # Errors
    ///
    /// If an I2C transfer fails, or [`Error::Timeout`] if the gauge doesn't answer.
    pub async fn firmware_version(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<u16, Error<I2C::Error>> {
        self.control(sub::GET_FW_VERSION, delay).await
    }

    /// Reads all values with one 56-byte I2C read, a snapshot of the gauge.
    ///
    /// The TRM asks hosts not to read the standard commands more than twice a second.
    ///
    /// # Errors
    ///
    /// If the I2C transfer fails.
    pub async fn status(&mut self) -> Result<Status, Error<I2C::Error>> {
        let mut block = [0u8; STATUS_BLOCK_LEN];
        self.i2c
            .write_read(self.address, &[STATUS_FIRST], &mut block)
            .await?;
        Ok(Status::from_block(&block))
    }
}

/// The first data word of the `MACData()` answer to `subcommand`, if `block` (from
/// `ManufacturerAccessControl()` to `MACDataLen()`) holds a complete one: the echoed
/// subcommand, a plausible length, and the matching checksum.
fn mac_answer(block: &[u8; mac::BLOCK_LEN], subcommand: u16) -> Option<u16> {
    if u16::from_le_bytes([block[0], block[1]]) != subcommand {
        return None;
    }
    // MACDataLen() counts the subcommand, the data, the checksum and itself.
    let len = usize::from(block[mac::LEN]);
    if !(mac::DATA + 2 + 2..=mac::BLOCK_LEN).contains(&len) {
        return None;
    }
    // MACDataSum() is the complement of the sum of the subcommand and the data bytes.
    let sum = !block[..len - 2]
        .iter()
        .fold(0u8, |sum, b| sum.wrapping_add(*b));
    if block[mac::SUM] != sum {
        return None;
    }
    Some(u16::from_le_bytes([block[mac::DATA], block[mac::DATA + 1]]))
}

/// The gauge reports the temperature in 0.1 K; this is 0.1 °C.
fn temperature_dc(raw_dk: u16) -> i16 {
    // The difference is at least -2731, so only the top can overflow: readings above
    // 3549.8 K, which no real battery produces.
    (i32::from(raw_dk) - ZERO_CELSIUS_DK).min(i32::from(i16::MAX)) as i16
}

#[cfg(test)]
mod tests {
    use core::{
        pin::pin,
        task::{Context, Poll, Waker},
    };

    use embedded_hal_async::i2c::{ErrorType, Operation};

    use super::*;

    /// The I2C error of the fake bus.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct Nack;

    impl embedded_hal_async::i2c::Error for Nack {
        fn kind(&self) -> embedded_hal_async::i2c::ErrorKind {
            embedded_hal_async::i2c::ErrorKind::NoAcknowledge(
                embedded_hal_async::i2c::NoAcknowledgeSource::Data,
            )
        }
    }

    /// How the fake gauge answers a `Control()` subcommand.
    #[derive(Clone, Copy, PartialEq, Eq)]
    enum Answer {
        /// Immediately, correctly.
        Ready,
        /// At this read of the block (the gauge is busy before).
        After(usize),
        /// Never: the block keeps its old content.
        Never,
        /// With a wrong checksum.
        BadChecksum,
    }

    /// A register file in place of the gauge.
    struct Registers {
        values: [u8; 256],
        answer: Answer,
        /// Block reads still to be served stale for `Answer::After`.
        stale_reads: usize,
        /// Transfers served so far, and the one that fails (if any).
        transfers: usize,
        fail_at: Option<usize>,
        /// Every read, as (register, length).
        reads: [(u8, usize); 64],
        read_count: usize,
    }

    impl Registers {
        fn new() -> Self {
            Self {
                values: [0; 256],
                answer: Answer::Ready,
                stale_reads: 0,
                transfers: 0,
                fail_at: None,
                reads: [(0, 0); 64],
                read_count: 0,
            }
        }

        fn set(&mut self, command: u8, value: u16) {
            let [lo, hi] = value.to_le_bytes();
            self.values[usize::from(command)] = lo;
            self.values[usize::from(command) + 1] = hi;
        }

        /// Puts the answer to `subcommand` into the MACData block, as the gauge does:
        /// the echo, the data, MACDataSum() and MACDataLen().
        fn put_answer(&mut self, subcommand: u16, data: &[u8]) {
            let base = usize::from(mac::CONTROL);
            self.values[base..base + 2].copy_from_slice(&subcommand.to_le_bytes());
            self.values[base + 2..base + 2 + data.len()].copy_from_slice(data);
            let len = 2 + data.len() + 2;
            let sum = !self.values[base..base + len - 2]
                .iter()
                .fold(0u8, |s, b| s.wrapping_add(*b));
            self.values[base + mac::SUM] = if self.answer == Answer::BadChecksum {
                sum ^ 0xFF
            } else {
                sum
            };
            self.values[base + mac::LEN] = len as u8;
        }
    }

    impl ErrorType for Registers {
        type Error = Nack;
    }

    impl I2c for Registers {
        async fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Nack> {
            assert_eq!(address, DEFAULT_ADDRESS);
            self.transfers += 1;
            if self.fail_at == Some(self.transfers) {
                return Err(Nack);
            }
            let mut register = 0;
            for operation in operations {
                match operation {
                    Operation::Write([selected]) => register = usize::from(*selected),
                    Operation::Write([command::CONTROL, lo, hi]) => {
                        let subcommand = u16::from_le_bytes([*lo, *hi]);
                        let data: &[u8] = match subcommand {
                            sub::GET_DEVICE_NUMBER => &[0x20, 0x02],
                            // A longer answer: version 0x2002 and more bytes.
                            sub::GET_FW_VERSION => &[0x02, 0x20, 0x01, 0x02, 0x03, 0x04],
                            other => panic!("unexpected subcommand {other:#06x}"),
                        };
                        match self.answer {
                            Answer::Never => {}
                            Answer::After(reads) => {
                                self.stale_reads = reads;
                                self.put_answer(subcommand, data);
                                // Stale until the reads are used up: a wrong echo.
                                self.values[usize::from(mac::CONTROL)] ^= 0xFF;
                            }
                            _ => self.put_answer(subcommand, data),
                        }
                    }
                    Operation::Read(buffer) => {
                        if register == usize::from(mac::CONTROL)
                            && self.stale_reads > 0
                            && let Answer::After(_) = self.answer
                        {
                            self.stale_reads -= 1;
                            if self.stale_reads == 0 {
                                self.values[usize::from(mac::CONTROL)] ^= 0xFF;
                            }
                        }
                        self.reads[self.read_count] = (register as u8, buffer.len());
                        self.read_count += 1;
                        buffer.copy_from_slice(&self.values[register..][..buffer.len()]);
                    }
                    Operation::Write(bytes) => panic!("unexpected write {bytes:02x?}"),
                }
            }
            Ok(())
        }
    }

    /// A delay that returns at once and counts the time asked for.
    #[derive(Default)]
    struct CountingDelay {
        ns: u64,
    }

    impl DelayNs for CountingDelay {
        async fn delay_ns(&mut self, ns: u32) {
            self.ns += u64::from(ns);
        }
    }

    fn block_on<F: Future>(future: F) -> F::Output {
        match pin!(future).poll(&mut Context::from_waker(Waker::noop())) {
            Poll::Ready(output) => output,
            Poll::Pending => panic!("the register file never blocks"),
        }
    }

    fn gauge(registers: Registers) -> Bq27220<Registers> {
        Bq27220::new(registers, DEFAULT_ADDRESS)
    }

    #[test]
    fn identifies_the_chip() {
        let mut gauge = gauge(Registers::new());
        let mut delay = CountingDelay::default();
        assert_eq!(block_on(gauge.device_number(&mut delay)), Ok(DEVICE_NUMBER));
        assert_eq!(block_on(gauge.firmware_version(&mut delay)), Ok(0x2002));
    }

    #[test]
    fn waits_for_a_busy_gauge() {
        let mut registers = Registers::new();
        registers.answer = Answer::After(3);
        let mut gauge = gauge(registers);
        let mut delay = CountingDelay::default();
        assert_eq!(block_on(gauge.device_number(&mut delay)), Ok(DEVICE_NUMBER));
        // Two stale reads, then the answer: three polls of 2 ms.
        assert_eq!(delay.ns, 3 * 2_000_000);
    }

    #[test]
    fn gives_up_when_the_gauge_never_answers() {
        for answer in [Answer::Never, Answer::BadChecksum] {
            let mut registers = Registers::new();
            registers.answer = answer;
            let mut gauge = gauge(registers);
            let mut delay = CountingDelay::default();
            assert_eq!(
                block_on(gauge.device_number(&mut delay)),
                Err(Error::Timeout)
            );
            assert_eq!(delay.ns, 10 * 2_000_000);
        }
    }

    #[test]
    fn rejects_an_answer_to_another_subcommand() {
        // A stale answer to FW_VERSION must not pass for the device number.
        let mut registers = Registers::new();
        registers.put_answer(sub::GET_FW_VERSION, &[0x02, 0x20]);
        registers.answer = Answer::Never;
        let mut gauge = gauge(registers);
        assert_eq!(
            block_on(gauge.device_number(&mut CountingDelay::default())),
            Err(Error::Timeout)
        );
    }

    #[test]
    fn checks_the_mac_answer() {
        let mut registers = Registers::new();
        registers.put_answer(0x0001, &[0x20, 0x02]);
        let block: [u8; mac::BLOCK_LEN] = registers.values
            [usize::from(mac::CONTROL)..usize::from(mac::CONTROL) + mac::BLOCK_LEN]
            .try_into()
            .unwrap();
        // The checksum the real gauge showed for this answer: 0xDC, length 6.
        assert_eq!((block[mac::SUM], block[mac::LEN]), (0xDC, 6));
        assert_eq!(mac_answer(&block, 0x0001), Some(0x0220));
        assert_eq!(mac_answer(&block, 0x0002), None);
        let mut bad = block;
        bad[mac::LEN] = 3;
        assert_eq!(mac_answer(&bad, 0x0001), None);
        bad[mac::LEN] = 200;
        assert_eq!(mac_answer(&bad, 0x0001), None);
        let mut bad = block;
        bad[mac::DATA] ^= 1;
        assert_eq!(mac_answer(&bad, 0x0001), None);
    }

    #[test]
    fn reads_a_word() {
        let mut registers = Registers::new();
        registers.set(command::INTERNAL_TEMPERATURE, 3000);
        registers.set(command::AT_RATE, (-5i16).cast_unsigned());
        let mut gauge = gauge(registers);
        assert_eq!(
            block_on(gauge.read_word(command::INTERNAL_TEMPERATURE)),
            Ok(3000)
        );
        assert_eq!(
            block_on(gauge.read_word(command::AT_RATE)),
            Ok((-5i16).cast_unsigned())
        );
    }

    fn discharging_registers() -> Registers {
        let mut registers = Registers::new();
        registers.set(command::VOLTAGE, 4010);
        registers.set(command::CURRENT, (-95i16).cast_unsigned());
        registers.set(command::AVERAGE_CURRENT, (-90i16).cast_unsigned());
        registers.set(command::AVERAGE_POWER, (-360i16).cast_unsigned());
        registers.set(command::RELATIVE_STATE_OF_CHARGE, 87);
        registers.set(command::STATE_OF_HEALTH, 96);
        registers.set(command::REMAINING_CAPACITY, 537);
        registers.set(command::FULL_CHARGE_CAPACITY, 623);
        registers.set(command::DESIGN_CAPACITY, 650);
        registers.set(command::TEMPERATURE, 2731 + 187); // 18.7 °C
        registers.set(command::TIME_TO_EMPTY, 354);
        registers.set(command::TIME_TO_FULL, 0xFFFF);
        registers.set(command::CYCLE_COUNT, 12);
        registers.set(command::BATTERY_STATUS, 0b1001);
        // Registers the driver ignores must not leak into the fields.
        registers.set(command::STANDBY_CURRENT, 0x1234);
        registers.set(command::INTERNAL_TEMPERATURE, 0x4321);
        registers
    }

    #[test]
    fn discharging_status() {
        let mut gauge = gauge(discharging_registers());
        let status = block_on(gauge.status()).unwrap();
        assert_eq!(
            status,
            Status {
                voltage_mv: 4010,
                current_ma: -95,
                average_current_ma: -90,
                average_power_mw: -360,
                state_of_charge_pct: 87,
                state_of_health_pct: 96,
                remaining_mah: 537,
                full_charge_mah: 623,
                design_mah: 650,
                temperature_dc: 187,
                time_to_empty_min: Some(354),
                time_to_full_min: None,
                cycle_count: 12,
                flags: StatusFlags(0b1001),
            }
        );
        assert!(status.flags.discharging());
        assert!(status.flags.battery_present());
        assert!(!status.flags.full_charge());
    }

    #[test]
    fn status_is_one_read() {
        let mut gauge = gauge(discharging_registers());
        block_on(gauge.status()).unwrap();
        let registers = gauge.release();
        assert_eq!(registers.transfers, 1);
        assert_eq!(registers.read_count, 1);
        assert_eq!(registers.reads[0], (0x06, 56));
    }

    #[test]
    fn charging_has_time_to_full_only() {
        let mut registers = Registers::new();
        registers.set(command::TIME_TO_EMPTY, 0xFFFF);
        registers.set(command::TIME_TO_FULL, 45);
        let status = block_on(gauge(registers).status()).unwrap();
        assert_eq!(status.time_to_empty_min, None);
        assert_eq!(status.time_to_full_min, Some(45));
    }

    #[test]
    fn an_i2c_error_is_passed_on() {
        let mut registers = discharging_registers();
        registers.fail_at = Some(1);
        let mut gauge = gauge(registers);
        assert_eq!(block_on(gauge.status()), Err(Error::I2c(Nack)));
        assert_eq!(block_on(gauge.read_word(command::VOLTAGE)), Ok(4010));

        // An error while waiting for a Control() answer, after the write.
        let mut registers = Registers::new();
        registers.fail_at = Some(2);
        let mut gauge = self::gauge(registers);
        assert_eq!(
            block_on(gauge.device_number(&mut CountingDelay::default())),
            Err(Error::I2c(Nack))
        );
    }

    #[test]
    fn temperature_conversion() {
        assert_eq!(temperature_dc(2731), 0);
        assert_eq!(temperature_dc(2731 - 50), -50);
        assert_eq!(temperature_dc(0), -2731);
        assert_eq!(temperature_dc(2731 + 32767), i16::MAX);
        assert_eq!(temperature_dc(u16::MAX), i16::MAX);
    }

    #[test]
    fn status_flags() {
        let all = StatusFlags(0xFFFF);
        assert!(all.discharging());
        assert!(all.system_down());
        assert!(all.terminate_discharge_alarm());
        assert!(all.battery_present());
        assert!(all.battery_inserted());
        assert!(all.ocv_good());
        assert!(all.terminate_charge_alarm());
        assert!(all.charge_inhibit());
        assert!(all.full_charge());
        assert!(all.over_temp_discharge());
        assert!(all.over_temp_charge());
        assert!(all.sleeping());
        assert!(all.ocv_failed());
        assert!(all.ocv_complete());
        assert!(all.fully_discharged());

        // One flag at a time, at the bit the TRM gives it.
        let one = |bit: u32| StatusFlags(1 << bit);
        assert!(one(0).discharging() && !one(1).discharging());
        assert!(one(1).system_down() && !one(0).system_down());
        assert!(one(2).terminate_discharge_alarm());
        assert!(one(3).battery_present());
        assert!(one(4).battery_inserted());
        assert!(one(5).ocv_good());
        assert!(one(6).terminate_charge_alarm());
        assert!(one(8).charge_inhibit());
        assert!(one(9).full_charge());
        assert!(one(10).over_temp_discharge());
        assert!(one(11).over_temp_charge());
        assert!(one(12).sleeping());
        assert!(one(13).ocv_failed());
        assert!(one(14).ocv_complete());
        assert!(one(15).fully_discharged());
        assert!(!StatusFlags::default().battery_present());
    }

    #[test]
    fn defaults_are_empty() {
        let status = Status::default();
        assert_eq!(status.voltage_mv, 0);
        assert_eq!(status.time_to_empty_min, None);
        assert_eq!(status.flags, StatusFlags(0));
    }
}
