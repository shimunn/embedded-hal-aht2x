#![cfg_attr(not(test), no_std)]
#![feature(int_roundings)]
use core::fmt::{Debug, Display};
use core::marker::PhantomData;
#[cfg(feature = "crc")]
use crc::{Crc, CRC_8_NRSC_5};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

pub const I2C_ADDRESS: u8 = 0x38;

pub const I2C_TRIGGER_MEASURE: u8 = 0xAC;

pub const MEASURE_TIME_MS: u32 = 80;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HumidityData(u32);

impl HumidityData {
    pub fn from_bytes(bytes: [u8; 3]) -> Self {
        // | x2 | x1 | half byte x0 |
        let mut me = Self(0);
        me.0 = bytes[0] as u32;
        me.0 <<= 8;
        me.0 |= bytes[1] as u32;
        me.0 <<= 8;
        me.0 |= (bytes[2] & 0b11110000u8) as u32;
        me.0 >>= 4;
        me
    }
    pub fn relative_mille(&self) -> u32 {
        ((self.0 as u64 * 1000) / (2 << 20)) as _
    }
    pub fn relative(&self) -> (u16, u16) {
        let mille = self.relative_mille();
        let div = 10;
        let percent = mille.div_floor(div);
        let frac = mille % div;
        (percent as _, frac as _)
    }
    #[cfg(feature = "float")]
    pub fn relative_f32(&self) -> f32 {
        self.relative_mille() as f32 * 1000
    }
}

impl Display for HumidityData {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let (percent, frac) = self.relative();
        write!(f, "{}.{}%", percent, frac)
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TemperatureData(u32);
impl TemperatureData {
    pub fn from_bytes(bytes: [u8; 3]) -> Self {
        // | half byte x2  | x1 | x0 |
        let mut me = Self(0);
        me.0 = (bytes[0] & 0b00001111u8) as u32;
        me.0 <<= 8;
        me.0 |= bytes[1] as u32;
        me.0 <<= 8;
        me.0 |= bytes[2] as u32;
        me
    }
    pub fn milli_celsius(&self) -> i32 {
        ((self.0 as i64 * 200 * 1000) / ((1u64 << 20) as i64) - 50 * 1000) as _
    }
    pub fn celsius(&self) -> (i16, u16) {
        let milli = self.milli_celsius();
        let int = milli.div_floor(1000);
        let frac = milli % 1000;
        (int as _, frac as _)
    }
    #[cfg(feature = "float")]
    pub fn celsius_f32(&self) -> f32 {
        self.milli_celsius() as f32 * 1000
    }
}

impl Display for TemperatureData {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let (int, frac) = self.celsius();
        write!(f, "{}.{}°C", int, frac)
    }
}

#[repr(u8)]
enum StatusBits {
    Busy = 0x01,
    Calibrated = 0x10,
    CalibrationOn = 0x08,
    FifoOn = 0x04,
    FifoFull = 0x02,
}

#[derive(Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status(u8);

impl Status {
    pub fn is_busy(&self) -> bool {
        self.0 & (StatusBits::Busy as u8) > 0
    }
    pub fn is_calibrated(&self) -> bool {
        self.0 & (StatusBits::Calibrated as u8) > 0
    }
    pub fn is_calibration_on(&self) -> bool {
        self.0 & (StatusBits::CalibrationOn as u8) > 0
    }
    pub fn is_fifo_on(&self) -> bool {
        self.0 & (StatusBits::FifoOn as u8) > 0
    }
    pub fn is_fido_full(&self) -> bool {
        self.0 & (StatusBits::FifoFull as u8) > 0
    }
}

impl Debug for Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Status")
            .field("busy", &self.is_busy())
            .field("calibrated", &self.is_calibrated())
            .field("calibration on", &self.is_calibration_on())
            .field("fifo on", &self.is_fifo_on())
            .field("fifo full", &self.is_fido_full())
            .finish()
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(u8)]
pub enum Aht2xCommand {
    Init = 0xBE,
    Status = 0x71,
    Measure = 0xAC,
    Reset = 0xBA,
}

pub struct Aht2X<T> {
    _marker: PhantomData<T>,
}

impl<T: I2c> Aht2X<T> {
    async fn write(
        &mut self,
        interface: &mut T,
        cmd: Aht2xCommand,
        bytes: &[u8],
    ) -> Result<(), T::Error> {
        let mut payload = [0u8; 12];
        payload[0] = cmd as _;
        payload[1..(bytes.len() + 1)].copy_from_slice(bytes);
        #[cfg(feature = "defmt")]
        defmt::debug!("cmd: {:?} {:?}", cmd, bytes);
        let res = interface
            .write(I2C_ADDRESS, &payload[..(bytes.len() + 1)])
            .await;
        res
    }

    async fn read(&mut self, interface: &mut T, bytes: &mut [u8]) -> Result<(), T::Error> {
        let res = interface.read(I2C_ADDRESS, bytes).await;
        #[cfg(feature = "defmt")]
        defmt::debug!("read {:?}", bytes);
        res
    }

    pub async fn setup<'a>(interface: &mut T, delay: &mut impl DelayNs) -> Result<Self, T::Error> {
        let mut me = Self {
            _marker: Default::default(),
        };
        delay.delay_ms(100).await;
        me.write(interface, Aht2xCommand::Reset, &[]).await?;
        delay.delay_ms(100).await;
        me.write(interface, Aht2xCommand::Init, &[0b10101100, 0x00])
            .await?;
        while me.status(interface).await?.is_busy() {
            delay.delay_ms(100).await;
        }
        let status = me.status(interface).await?;
        #[cfg(feature = "defmt")]
        defmt::debug!("setup status: {:?}", status);
        if !status.is_calibrated() {
            #[cfg(feature = "defmt")]
            defmt::warn!("calibration failed!");
        }
        Ok(me)
    }

    pub async fn status<'a>(&mut self, interface: &mut T) -> Result<Status, T::Error> {
        self.write(interface, Aht2xCommand::Status, &[]).await?;
        let mut status = Status(0);
        self.read(interface, core::slice::from_mut(&mut status.0))
            .await?;
        Ok(status)
    }
    pub async fn measure<'a>(
        &mut self,
        interface: &mut T,
        delay: &mut impl DelayNs,
    ) -> Result<(HumidityData, TemperatureData), T::Error> {
        delay.delay_ms(100).await;
        let mut resp = [0u8; if cfg!(feature = "crc") { 7 } else { 6 }];
        self.write(interface, Aht2xCommand::Measure, &[0x33, 0x00])
            .await?;
        loop {
            delay.delay_ms(MEASURE_TIME_MS).await;
            self.read(interface, &mut resp).await?;
            let full_resp = &resp[..];
            let resp = full_resp;
            let state = Status(resp[0]);
            let resp = &resp[1..];
            #[cfg(feature = "defmt")]
            defmt::debug!("{:?} {}", state, state.is_busy());
            if state.is_busy() {
                continue;
            }
            let humidity = HumidityData::from_bytes(resp[..3].try_into().unwrap());
            let resp = &resp[2..];
            let temperature = TemperatureData::from_bytes(resp[..3].try_into().unwrap());
            #[allow(unused)]
            let resp = &resp[3..];
            #[cfg(feature = "crc")]
            let _ = {
                let digest = calc_crc(&full_resp[..6]);
                match digest {
                    digest if digest == resp[0] => {}
                    #[allow(unused)]
                    digest => {
                        #[cfg(feature = "defmt")]
                        defmt::warn!("crc mismatch, expected {}, got {}", resp[0], digest);
                        continue;
                    }
                }
                &resp[1..]
            };
            #[cfg(feature = "defmt")]
            defmt::debug!("temperature: {:?}, humidity: {:?}", temperature, humidity);
            break Ok((humidity, temperature));
        }
    }
}

#[cfg(feature = "crc")]
fn calc_crc(resp: &[u8]) -> u8 {
    let crc = Crc::<u8>::new(&CRC_8_NRSC_5);
    let mut digest = crc.digest_with_initial(0xFF);
    digest.update(resp);
    digest.finalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status() {
        assert!(Status(159).is_busy())
    }

    #[test]
    fn parse_measurement() {
        let resp: [u8; 5] = [111, 65, 133, 170, 130];
        let hum = HumidityData::from_bytes(resp[..3].try_into().unwrap());
        assert_eq!(hum.relative(), (21, 7));
        let temp = TemperatureData::from_bytes(resp[2..].try_into().unwrap());
        assert_eq!(temp.celsius(), (20, 825));
    }
}
