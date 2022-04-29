#![no_std]

//! I2C interface for pimoroni trackball.


use hal::blocking::delay::DelayUs;
use hal::blocking::i2c::{Operation, Transactional, Write, WriteRead};
use hal::digital::v2::InputPin;
use hal::timer::CountDown;
use rp_hal::hal;

/// I2C communication interface
pub struct I2CInterface<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I> I2CInterface<I>
where
    I: Write,
{
    /// Create new I2C interface with a default I2C address of 0x0A.
    pub fn new(i2c: I) -> Self {
        Self::new_custom_address(i2c, 0x0A)
    }

    /// Create a new I2C interface with the alternate address 0x0B as specified in the datasheet.
    pub fn new_alternate_address(i2c: I) -> Self {
        Self::new_custom_address(i2c, 0x0B)
    }

    /// Create a new I2C interface with a custom address.
    pub fn new_custom_address(i2c: I, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Consume the trackball interface and return
    /// the underlying peripherial driver
    pub fn release(self) -> I {
        self.i2c
    }
}

const SWITCH_STATE_MASK: u8 = 0b1000_0000;

impl<I2C> I2CInterface<I2C>
where
    I2C: Write<Error = <I2C as WriteRead>::Error>
        + WriteRead<Error = <I2C as Transactional>::Error>
        + Transactional,
{
    fn chip_id(&mut self) -> Result<u16, <I2C as Write>::Error> {
        let mut bytes = [0u8; 2];
        self.i2c.write_read(self.addr, &[CHIP_ID_L], &mut bytes)?;
        let chip_id: u16 = u16::from_ne_bytes(bytes);
        // let mut chip_id = (*unsafe { byte.get_unchecked(0) } as u16) << 8;
        // self.i2c.write_read(self.addr, &[CHIP_ID_L], &mut byte)?;
        // chip_id |= (*unsafe { byte.get_unchecked(0) }) as u16;
        Ok(chip_id)
    }

    fn set_red(&mut self, val: u8) -> Result<(), <I2C as Write>::Error> {
        self.i2c.write(self.addr, &[RED_REGISTER, val])
    }

    fn set_green(&mut self, val: u8) -> Result<(), <I2C as Write>::Error> {
        self.i2c.write(self.addr, &[GREEN_REGISTER, val])
    }

    fn set_blue(&mut self, val: u8) -> Result<(), <I2C as Write>::Error> {
        self.i2c.write(self.addr, &[BLUE_REGISTER, val])
    }

    fn set_white(&mut self, val: u8) -> Result<(), <I2C as Write>::Error> {
        self.i2c.write(self.addr, &[WHITE_REGISTER, val])
    }

    fn get_interrupt(&mut self) -> Result<bool, <I2C as Write>::Error> {
        let mut val = 0;
        self.i2c.write_read(
            self.addr,
            &[WHITE_REGISTER],
            core::slice::from_mut(&mut val),
        )?;
        Ok((val & INTERRUPT_TRIGGERED_MASK) > 0)
    }

    fn read(&mut self) -> Result<Data, <I2C as Write>::Error> {
        let mut left = 0;
        let mut right = 0;
        let mut up = 0;
        let mut down = 0;
        let mut switch_state = 0;
        self.i2c.exec(
            self.addr,
            &mut [
                Operation::Write(&[LEFT]),
                Operation::Read(core::slice::from_mut(&mut left)),
                Operation::Write(&[RIGHT]),
                Operation::Read(core::slice::from_mut(&mut right)),
                Operation::Write(&[UP]),
                Operation::Read(core::slice::from_mut(&mut up)),
                Operation::Write(&[DOWN]),
                Operation::Read(core::slice::from_mut(&mut down)),
                Operation::Write(&[SWITCH]),
                Operation::Read(core::slice::from_mut(&mut switch_state)),
            ],
        )?;
        let switch_changed = (switch_state & !SWITCH_STATE_MASK) > 0;
        let switch_pressed = (switch_state & SWITCH_STATE_MASK) > 0;

        Ok(Data {
            left,
            right,
            up,
            down,
            switch_changed,
            switch_pressed,
        })
    }
}

const CHIP_ID: u16 = 0xBA11;
#[allow(dead_code)]
const VERSION: u8 = 1;
const DEFAULT_TIMEOUT: u32 = 5;

pub struct Data {
    pub left: u8,
    pub right: u8,
    pub up: u8,
    pub down: u8,
    pub switch_changed: bool,
    pub switch_pressed: bool,
}

const RED_REGISTER: u8 = 0x00;
const GREEN_REGISTER: u8 = 0x01;
const BLUE_REGISTER: u8 = 0x02;
const WHITE_REGISTER: u8 = 0x03;

const LEFT: u8 = 0x04;
const RIGHT: u8 = 0x05;
const UP: u8 = 0x06;
const DOWN: u8 = 0x07;
const SWITCH: u8 = 0x08;

#[allow(dead_code)]
const USER_FLASH: u8 = 0xD0;
#[allow(dead_code)]
const FLASH_PAGE: u8 = 0xF0;
const INTERRUPT: u8 = 0xF9;

const CHIP_ID_L: u8 = 0xFA;
#[allow(dead_code)]
const CHIP_ID_H: u8 = 0xFB;
#[allow(dead_code)]
const VERSION_REG: u8 = 0xFC;
const I2C_ADDR: u8 = 0xFD;
#[allow(dead_code)]
const CTRL: u8 = 0xFE;

const INTERRUPT_OUT_ENABLE_MASK: u8 = 0b0000_0010;
const INTERRUPT_TRIGGERED_MASK: u8 = 0b0000_0001;

pub enum TrackballError<I, P>
where
    I: Write,
    P: InputPin,
{
    I2C(I::Error),
    Pin(P::Error),
    ChipId,
    Timeout,
}

pub struct TrackballBuilder<I, P> {
    i2c: I2CInterface<I>,
    interrupt_pin: Option<P>,
    timeout: u32,
}

impl<I, P> TrackballBuilder<I, P> {
    pub fn new(i2c: I2CInterface<I>) -> Self {
        Self {
            i2c,
            interrupt_pin: None,
            timeout: DEFAULT_TIMEOUT,
        }
    }

    pub fn interrupt_pin(mut self, pin: P) -> Self {
        self.interrupt_pin = Some(pin);
        self
    }

    pub fn timeout(mut self, timeout: u32) -> Self {
        self.timeout = timeout;
        self
    }

    pub fn build(self) -> Trackball<I, P> {
        Trackball {
            i2c: self.i2c,
            interrupt_pin: self.interrupt_pin,
            timeout: self.timeout,
        }
    }
}

pub struct Trackball<I, P> {
    i2c: I2CInterface<I>,
    interrupt_pin: Option<P>,
    timeout: u32,
}

impl<I, P> Trackball<I, P>
where
    P: InputPin,
    I: Write<Error = <I as WriteRead>::Error>
        + WriteRead<Error = <I as Transactional>::Error>
        + Transactional,
{
    pub fn init(&mut self) -> Result<(), TrackballError<I, P>> {
        let chip_id = self.i2c.chip_id().map_err(TrackballError::I2C)?;
        if chip_id == CHIP_ID {
            self.enable_interrupt()?;
            Ok(())
        } else {
            Err(TrackballError::ChipId)
        }
    }

    pub fn enable_interrupt(&mut self) -> Result<(), TrackballError<I, P>> {
        let mut byte = [0u8; 1];
        self.i2c
            .i2c
            .write_read(self.i2c.addr, &[INTERRUPT], &mut byte)
            .map_err(TrackballError::I2C)?;
        let mut value = *unsafe { byte.get_unchecked(0) };
        if self.interrupt_pin.is_some() {
            value |= INTERRUPT_OUT_ENABLE_MASK;
        } else {
            value &= !INTERRUPT_OUT_ENABLE_MASK;
        }

        self.i2c
            .i2c
            .write(self.i2c.addr, &[INTERRUPT, value])
            .map_err(TrackballError::I2C)
    }

    pub fn set_rgbw(&mut self, r: u8, g: u8, b: u8, w: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_red(r)?;
        self.i2c.set_green(g)?;
        self.i2c.set_blue(b)?;
        self.i2c.set_white(w)?;
        Ok(())
    }

    #[inline]
    pub fn set_red(&mut self, val: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_red(val)
    }

    #[inline]
    pub fn set_green(&mut self, val: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_green(val)
    }

    #[inline]
    pub fn set_blue(&mut self, val: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_blue(val)
    }

    #[inline]
    pub fn set_white(&mut self, val: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_white(val)
    }

    #[inline]
    pub fn read(&mut self) -> Result<Data, <I as Write>::Error> {
        self.i2c.read()
    }

    pub fn change_address<C, D>(
        &mut self,
        new_address: u8,
        countdown: &mut C,
        delay: &mut D,
    ) -> Result<(), TrackballError<I, P>>
    where
        C: CountDown,
        D: DelayUs<u8>,
        <C as CountDown>::Time: From<u32>,
    {
        self.i2c
            .i2c
            .write(self.i2c.addr, &[I2C_ADDR, new_address])
            .map_err(TrackballError::I2C)?;
        self.wait_for_flash(countdown, delay)?;
        Ok(())
    }

    fn wait_for_flash<C, D>(
        &mut self,
        countdown: &mut C,
        delay: &mut D,
    ) -> Result<(), TrackballError<I, P>>
    where
        C: CountDown,
        D: DelayUs<u8>,
        <C as CountDown>::Time: From<u32>,
    {
        countdown.start(self.timeout);
        while self.get_interrupt()? {
            if let Ok(()) = countdown.wait() {
                return Err(TrackballError::Timeout);
            }
            delay.delay_us(1u8);
        }

        countdown.start(self.timeout);
        while !self.get_interrupt()? {
            if let Ok(()) = countdown.wait() {
                return Err(TrackballError::Timeout);
            }
            delay.delay_us(1u8);
        }
        Ok(())
    }

    pub fn get_interrupt(&mut self) -> Result<bool, TrackballError<I, P>> {
        if let Some(pin) = &self.interrupt_pin {
            pin.is_low().map_err(TrackballError::Pin)
        } else {
            self.i2c.get_interrupt().map_err(TrackballError::I2C)
        }
    }
}
