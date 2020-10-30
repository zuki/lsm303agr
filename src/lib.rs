//! A platform agnostic driver to interface with the LSM303AGR (accelerometer + compass)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2
//!
//! # Examples
//!
//! You should find at least one example in the [f3] crate.
//!
//! [f3]: https://docs.rs/f3/~0.6

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

extern crate cast;
extern crate embedded_hal as hal;
extern crate generic_array;

use core::mem;

use cast::u16;
use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};
use hal::blocking::i2c::{Write, WriteRead};

mod accel;
mod mag;

/// LSM303AGR driver
pub struct Lsm303agr<I2C> {
    i2c: I2C,
}

impl<I2C, E> Lsm303agr<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Creates a new driver from a I2C peripheral
    pub fn new(i2c: I2C) -> Result<Self, E> {
        let mut lsm303agr = Lsm303agr { i2c };

        // TODO reset all the registers / the device

        // configure the accelerometer to operate at 400 Hz
        lsm303agr.write_accel_register(accel::Register::CTRL_REG1_A, 0b0111_0_111)?;

        // configure the magnetometer to operate in continuous mode
        // and enable temperature compensation
        lsm303agr.write_mag_register(mag::Register::CFG_REG_A_M, 0x80)?;
        // enable offset cancellation
        lsm303agr.write_mag_register(mag::Register::CFG_REG_B_M, 0x02)?;

        // enable the temperature sensor: TEMP_EN = '11', BDU = '1' (4.5, page 34)
        lsm303agr.write_accel_register(accel::Register::TEMP_CFG_REG_A, 0xC0)?;
        // set high resolution mode
        lsm303agr.write_accel_register(accel::Register::CTRL_REG4_A, 0x88)?;

        Ok(lsm303agr)
    }

    /// Accelerometer measurements
    pub fn accel(&mut self) -> Result<I16x3, E> {
        let buffer: GenericArray<u8, U6> = self.read_accel_registers(accel::Register::OUT_X_L_A)?;

        Ok(I16x3 {
            x: (u16(buffer[0]) + (u16(buffer[1]) << 8)) as i16,
            y: (u16(buffer[2]) + (u16(buffer[3]) << 8)) as i16,
            z: (u16(buffer[4]) + (u16(buffer[5]) << 8)) as i16,
        })
    }

    /// Sets the accelerometer output data rate
    pub fn accel_odr(&mut self, odr: AccelOdr) -> Result<(), E> {
        self.modify_accel_register(accel::Register::CTRL_REG1_A, |r| {
            r & !(0b1111 << 4) | ((odr as u8) << 4)
        })
    }

    /// Magnetometer measurements
    pub fn mag(&mut self) -> Result<I16x3, E> {
        let buffer: GenericArray<u8, U6> = self.read_mag_registers(mag::Register::OUTX_L_REG_M)?;

        Ok(I16x3 {
            x: (u16(buffer[0]) + (u16(buffer[1]) << 8)) as i16,
            y: (u16(buffer[2]) + (u16(buffer[3]) << 8)) as i16,
            z: (u16(buffer[4]) + (u16(buffer[5]) << 8)) as i16,
        })
    }

    /// Sets the magnetometer output data rate
    pub fn mag_odr(&mut self, odr: MagOdr) -> Result<(), E> {
        self.modify_mag_register(mag::Register::CFG_REG_A_M, |r| {
            r & !(0b11 << 2) | ((odr as u8) << 2)
        })
    }

    /// Temperature sensor measurement
    ///
    /// - Resolution: 8-bit
    /// - Range: [-40, +85]
    pub fn temp(&mut self) -> Result<i16, E> {
    
        loop {
            if let Ok(r) = self.read_accel_register(accel::Register::STATUS_REG_AUX_A) {
                if (r & 0x04) == 0x04 {
                    break;
                }
            }
        }
    
        let buffer: GenericArray<u8, U2> = self.read_accel_registers(accel::Register::OUT_TEMP_L_A)?;

        Ok((u16(buffer[0]) + (u16(buffer[1]) << 8)) as i16)
    }

    /// Changes the `sensitivity` of the accelerometer
    pub fn set_accel_sensitivity(&mut self, sensitivity: Sensitivity) -> Result<(), E> {
        self.modify_accel_register(accel::Register::CTRL_REG4_A, |r| {
            r & !(0b11 << 4) | (sensitivity.value() << 4)
        })
    }

    fn modify_accel_register<F>(&mut self, reg: accel::Register, f: F) -> Result<(), E>
    where
        F: FnOnce(u8) -> u8,
    {
        let r = self.read_accel_register(reg)?;
        self.write_accel_register(reg, f(r))?;
        Ok(())
    }

    fn modify_mag_register<F>(&mut self, reg: mag::Register, f: F) -> Result<(), E>
    where
        F: FnOnce(u8) -> u8,
    {
        let r = self.read_mag_register(reg)?;
        self.write_mag_register(reg, f(r))?;
        Ok(())
    }

    fn read_accel_registers<N>(&mut self, reg: accel::Register) -> Result<GenericArray<u8, N>, E>
    where
        N: ArrayLength<u8>,
    {
        let mut buffer: GenericArray<u8, N> = unsafe { mem::MaybeUninit::<GenericArray<u8, N>>::uninit().assume_init() };

        {
            let buffer: &mut [u8] = &mut buffer;

            const MULTI: u8 = 1 << 7;
            self.i2c
                .write_read(accel::ADDRESS, &[reg.addr() | MULTI], buffer)?;
        }

        Ok(buffer)
    }

    fn read_accel_register(&mut self, reg: accel::Register) -> Result<u8, E> {
        self.read_accel_registers::<U1>(reg).map(|b| b[0])
    }

    fn read_mag_register(&mut self, reg: mag::Register) -> Result<u8, E> {
        self.read_mag_registers::<U1>(reg).map(|b| b[0])
    }

    fn read_mag_registers<N>(&mut self, reg: mag::Register) -> Result<GenericArray<u8, N>, E>
    where
        N: ArrayLength<u8>,
    {
        let mut buffer: GenericArray<u8, N> = unsafe { mem::MaybeUninit::<GenericArray<u8, N>>::uninit().assume_init() };

        {
            let buffer: &mut [u8] = &mut buffer;
            const MULTI: u8 = 1 << 7;
            self.i2c.write_read(mag::ADDRESS, &[reg.addr() | MULTI], buffer)?;
        }

        Ok(buffer)
    }

    fn write_accel_register(&mut self, reg: accel::Register, byte: u8) -> Result<(), E> {
        self.i2c.write(accel::ADDRESS, &[reg.addr(), byte])
    }

    fn write_mag_register(&mut self, reg: mag::Register, byte: u8) -> Result<(), E> {
        self.i2c.write(mag::ADDRESS, &[reg.addr(), byte])
    }
}

/// XYZ triple
#[derive(Debug)]
pub struct I16x3 {
    /// X component
    pub x: i16,
    /// Y component
    pub y: i16,
    /// Z component
    pub z: i16,
}

/// Accelerometer Output Data Rate
pub enum AccelOdr {
    /// 1 Hz
    Hz1 = 0b0001,
    /// 10 Hz
    Hz10 = 0b0010,
    /// 25 Hz
    Hz25 = 0b0011,
    /// 50 Hz
    Hz50 = 0b0100,
    /// 100 Hz
    Hz100 = 0b0101,
    /// 200 Hz
    Hz200 = 0b0110,
    /// 400 Hz
    Hz400 = 0b0111,
}

/// Magnetometer Output Data Rate
pub enum MagOdr {
    /// 10 Hz
    Hz10 = 0b00,
    /// 20 Hz
    Hz20 = 0b01,
    /// 50 Hz
    Hz50 = 0b10,
    /// 100 Hz
    Hz100 = 0b11,
}

/// Acceleration sensitivity
#[derive(Clone, Copy)]
pub enum Sensitivity {
    /// Range: [-2g, +2g]. Sensitivity ~ 1 g / (1 << 14) LSB
    G1,
    /// Range: [-4g, +4g]. Sensitivity ~ 2 g / (1 << 14) LSB
    G2,
    /// Range: [-8g, +8g]. Sensitivity ~ 4 g / (1 << 14) LSB
    G4,
    /// Range: [-16g, +16g]. Sensitivity ~ 12 g / (1 << 14) LSB
    G12,
}

impl Sensitivity {
    fn value(&self) -> u8 {
        *self as u8
    }
}
