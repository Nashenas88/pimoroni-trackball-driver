#![no_std]

//! I2C interface for pimoroni trackball.

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use embedded_hal::digital::v2::InputPin;
use embedded_time::duration::Microseconds;
use embedded_time::{Clock, TimeError};

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
    I2C: Write<Error = <I2C as WriteRead>::Error> + WriteRead,
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
}

impl<I2C> I2CInterface<I2C>
where
    I2C: WriteRead,
{
    fn read(&mut self) -> Result<Data, I2C::Error> {
        let mut left = 0;
        let mut right = 0;
        let mut up = 0;
        let mut down = 0;
        let mut switch_state = 0;
        self.i2c
            .write_read(self.addr, &[LEFT], core::slice::from_mut(&mut left))?;
        self.i2c
            .write_read(self.addr, &[RIGHT], core::slice::from_mut(&mut right))?;
        self.i2c
            .write_read(self.addr, &[UP], core::slice::from_mut(&mut up))?;
        self.i2c
            .write_read(self.addr, &[DOWN], core::slice::from_mut(&mut down))?;
        self.i2c.write_read(
            self.addr,
            &[SWITCH],
            core::slice::from_mut(&mut switch_state),
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
const DEFAULT_TIMEOUT_US: u32 = 5;

/// Data read from the trackball.
pub struct Data {
    /// How many units the trackball was moved to the left.
    pub left: u8,
    /// How many units the trackball was moved to the right.
    pub right: u8,
    /// How many units the trackball was moved up.
    pub up: u8,
    /// How many units the trackball was moved down.
    pub down: u8,
    /// Whether or not the switch state has changed from the last update.
    pub switch_changed: bool,
    /// The state of the switch being pressed.
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

/// Possible errors from trackball functions.
pub enum TrackballError<I, P> {
    /// Error in I2C communication.
    I2C(I),
    /// Error reading interrupt pin.
    Pin(P),
    /// Error running timers.
    TimeError(TimeError),
    /// Mismatching chip id.
    ChipId,
    /// Timeout occurred after setting new I2C address.
    Timeout,
}

impl<I, P> From<TimeError> for TrackballError<I, P> {
    fn from(t: TimeError) -> Self {
        Self::TimeError(t)
    }
}

pub type TrackballResult<T, I, P> =
    Result<T, TrackballError<<I as Write>::Error, <P as InputPin>::Error>>;

/// Builder struct to simplify constructing a [Trackball] instance.
pub struct TrackballBuilder<I, P> {
    i2c: I2CInterface<I>,
    interrupt_pin: Option<P>,
    timeout: u32,
}

impl<I, P> TrackballBuilder<I, P> {
    /// Construct a new builder with the required [I2CInterface].
    pub fn new(i2c: I2CInterface<I>) -> Self {
        Self {
            i2c,
            interrupt_pin: None,
            timeout: DEFAULT_TIMEOUT_US,
        }
    }

    /// Sets the interrupt pin. When this is called, you must supply an 
    pub fn interrupt_pin(mut self, pin: P) -> Self {
        self.interrupt_pin = Some(pin);
        self
    }

    /// Override the default timeout of 5 microseconds which is used when
    /// changing the address of the trackball.
    pub fn timeout(mut self, timeout: u32) -> Self {
        self.timeout = timeout;
        self
    }

    /// Construct an instance of [Trackball].
    pub fn build(self) -> Trackball<I, P> {
        Trackball {
            i2c: self.i2c,
            interrupt_pin: self.interrupt_pin,
            timeout: self.timeout,
        }
    }
}

/// The `Trackball` struct manages communication with a Pimoroni trackball.
pub struct Trackball<I, P> {
    i2c: I2CInterface<I>,
    interrupt_pin: Option<P>,
    timeout: u32,
}

impl<I, P> Trackball<I, P>
where
    P: InputPin,
    I: Write<Error = <I as WriteRead>::Error> + WriteRead,
{
    /// Initialize the trackball.
    ///
    /// The `interrupt_enable_func` is only called if an interrupt pin
    /// was configured on the [TrackballBuilder].
    pub fn init(
        &mut self,
        interrupt_enable_func: impl FnOnce(&mut P),
    ) -> TrackballResult<(), I, P> {
        let chip_id = self.i2c.chip_id().map_err(TrackballError::I2C)?;
        if chip_id == CHIP_ID {
            self.enable_interrupt(interrupt_enable_func)?;
            Ok(())
        } else {
            Err(TrackballError::ChipId)
        }
    }

    fn enable_interrupt(
        &mut self,
        interrupt_enable_func: impl FnOnce(&mut P),
    ) -> TrackballResult<(), I, P> {
        let mut value = 0u8;
        self.i2c
            .i2c
            .write_read(
                self.i2c.addr,
                &[INTERRUPT],
                core::slice::from_mut(&mut value),
            )
            .map_err(TrackballError::I2C)?;
        if self.interrupt_pin.is_some() {
            value |= INTERRUPT_OUT_ENABLE_MASK;
        } else {
            value &= !INTERRUPT_OUT_ENABLE_MASK;
        }

        let res = self
            .i2c
            .i2c
            .write(self.i2c.addr, &[INTERRUPT, value])
            .map_err(TrackballError::I2C);
        if let Some(pin) = &mut self.interrupt_pin {
            interrupt_enable_func(pin);
        }
        res
    }

    /// Get the interrupt pin used by this instance.
    pub fn interrupt(&mut self) -> Option<&mut P> {
        self.interrupt_pin.as_mut()
    }

    /// The the rgbw colors on the trackball's LED.
    pub fn set_rgbw(&mut self, r: u8, g: u8, b: u8, w: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_red(r)?;
        self.i2c.set_green(g)?;
        self.i2c.set_blue(b)?;
        self.i2c.set_white(w)?;
        Ok(())
    }

    /// The the red color on the trackball's LED.
    #[inline]
    pub fn set_red(&mut self, val: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_red(val)
    }

    /// The the green color on the trackball's LED.
    #[inline]
    pub fn set_green(&mut self, val: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_green(val)
    }

    /// The the blue color on the trackball's LED.
    #[inline]
    pub fn set_blue(&mut self, val: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_blue(val)
    }

    /// The the white color on the trackball's LED.
    #[inline]
    pub fn set_white(&mut self, val: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_white(val)
    }

    /// Read the current state of the trackball.
    #[inline]
    pub fn read(&mut self) -> Result<Data, <I as Write>::Error> {
        self.i2c.read()
    }

    /// Change the I2C address of the trackball.
    ///
    /// This will save the new value in the trackball's flash, and so needs time to update the flash
    /// storage. It will delay by the timeout configured in [TrackballBuilder] or the default of
    /// 5us.
    pub fn change_address<C, De>(
        &mut self,
        new_address: u8,
        clock: &mut C,
        delay: &mut De,
    ) -> TrackballResult<(), I, P>
    where
        C: Clock,
        De: DelayUs<u8>,
        <C as Clock>::T: From<u32>,
    {
        self.i2c
            .i2c
            .write(self.i2c.addr, &[I2C_ADDR, new_address])
            .map_err(TrackballError::I2C)?;
        self.wait_for_flash(clock, delay)?;
        Ok(())
    }

    fn wait_for_flash<C, De>(&mut self, clock: &mut C, delay: &mut De) -> TrackballResult<(), I, P>
    where
        C: Clock,
        De: DelayUs<u8>,
        <C as Clock>::T: From<u32>,
    {
        let timer = clock.new_timer(Microseconds(self.timeout));
        let timer = timer.start()?;
        while self.get_interrupt()? {
            if timer.is_expired()? {
                return Err(TrackballError::Timeout);
            }
            delay.delay_us(1u8);
        }

        let timer = clock.new_timer(Microseconds(self.timeout));
        let timer = timer.start()?;
        while !self.get_interrupt()? {
            if timer.is_expired()? {
                return Err(TrackballError::Timeout);
            }
            delay.delay_us(1u8);
        }
        Ok(())
    }

    /// Return whether the interrupt is currently triggering.
    pub fn get_interrupt(&mut self) -> TrackballResult<bool, I, P> {
        if let Some(pin) = &self.interrupt_pin {
            pin.is_low().map_err(TrackballError::Pin)
        } else {
            self.i2c.get_interrupt().map_err(TrackballError::I2C)
        }
    }
}
