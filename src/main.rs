#![no_std]
#![no_main]

use core::{cell::RefCell, mem::MaybeUninit};

use esp32_hal::{
    clock::ClockControl,
    gpio::{Gpio32, Gpio33},
    gpio_types::{Event, OpenDrain, Output, Pin},
    interrupt,
    pac::{self, Peripherals},
    prelude::*,
    Cpu, RtcCntl, Timer, IO,
};
use esp_backtrace as _;
use esp_println::println;
use pc_keyboard::{layouts, HandleControl, ScancodeSet2};
use xtensa_lx::mutex::{Mutex, SpinLockMutex};
use xtensa_lx_rt::entry;

static mut CLK: SpinLockMutex<RefCell<Option<Gpio33<Output<OpenDrain>>>>> =
    SpinLockMutex::new(RefCell::new(None));
static mut DATA: SpinLockMutex<RefCell<Option<Gpio32<Output<OpenDrain>>>>> =
    SpinLockMutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.DPORT.split();
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    timer0.disable();
    rtc_cntl.set_wdt_global_enable(false);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut data_pin = io.pins.gpio32.into_open_drain_output();
    let mut clk_pin = io.pins.gpio33.into_open_drain_output();

    clk_pin.listen(Event::FallingEdge);

    data_pin.set_low().unwrap();
    clk_pin.set_low().unwrap();

    data_pin.set_high().unwrap();
    clk_pin.set_high().unwrap();

    unsafe {
        (&CLK).lock(|data| (*data).replace(Some(clk_pin)));
        (&DATA).lock(|data| (*data).replace(Some(data_pin)));
    }

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::GPIO,
        interrupt::CpuInterrupt::Interrupt1LevelPriority1,
    );

    unsafe {
        xtensa_lx::interrupt::enable_mask(1 << 1);
    }

    let mut kb = pc_keyboard::Keyboard::new(
        layouts::Us104Key,
        ScancodeSet2,
        HandleControl::MapLettersToUnicode,
    );
    loop {
        if let Some(byte) = get_byte() {
            match kb.add_byte(byte) {
                Ok(Some(event)) => {
                    println!("Event {:?}", event);
                }
                Ok(None) => (),
                Err(e) => {
                    println!("Error decoding: {:?}", e);
                }
            }
        }
    }
}

static mut QUEUE: Option<SimpleQueue<u8, 5>> = None;

fn send_byte(byte: u8) {
    // this is not multi-core safe - would need a spin lock in that case
    // this example is single-core anyway
    xtensa_lx::interrupt::free(|_| unsafe {
        if QUEUE.is_none() {
            QUEUE = Some(SimpleQueue::new());
        }
        match QUEUE {
            Some(ref mut queue) => {
                queue.enqueue(byte);
            }
            None => (),
        }
    });
}

fn get_byte() -> Option<u8> {
    // this is not multi-core safe - would need a spin lock in that case
    // this example is single-core anyway
    xtensa_lx::interrupt::free(|_| unsafe {
        match QUEUE {
            Some(ref mut queue) => queue.dequeue(),
            None => None,
        }
    })
}

#[no_mangle]
pub fn level1_interrupt() {
    static mut BIT_COUNT: usize = 0;
    static mut CURRENT: u8 = 0;

    unsafe {
        let state = (&DATA).lock(|data| {
            let data = data.borrow_mut();
            let data = data.as_ref().unwrap();
            data.is_high().unwrap()
        });

        let bit = if state { 1 } else { 0 };

        interrupt::clear(
            Cpu::ProCpu,
            interrupt::CpuInterrupt::Interrupt1LevelPriority1,
        );

        (&CLK).lock(|data| {
            let mut clk = data.borrow_mut();
            let clk = clk.as_mut().unwrap();
            clk.clear_interrupt();
        });

        if BIT_COUNT > 0 && BIT_COUNT < 9 {
            CURRENT = CURRENT.overflowing_shr(1).0;
            CURRENT |= bit << 7;
        }
        BIT_COUNT += 1;

        if BIT_COUNT == 11 {
            send_byte(CURRENT);

            BIT_COUNT = 0;
            CURRENT = 0;
        }
    };
}

pub struct SimpleQueue<T, const N: usize> {
    data: [Option<T>; N],
    read_index: usize,
    write_index: usize,
}

impl<T, const N: usize> SimpleQueue<T, N> {
    pub fn new() -> SimpleQueue<T, N> {
        let mut queue = unsafe {
            SimpleQueue {
                data: MaybeUninit::uninit().assume_init(),
                read_index: 0,
                write_index: 0,
            }
        };

        for i in 0..N {
            queue.data[i] = None;
        }

        queue
    }

    pub fn enqueue(&mut self, e: T) -> bool {
        self.data[self.write_index] = Some(e);

        self.write_index += 1;
        self.write_index %= N;

        if self.write_index == self.read_index {
            return false;
        }

        true
    }

    pub fn dequeue(&mut self) -> Option<T> {
        if self.write_index == self.read_index {
            None
        } else {
            let result = self.data[self.read_index].take();
            self.read_index += 1;
            self.read_index %= N;
            result
        }
    }

    pub fn is_empty(&self) -> bool {
        self.read_index == self.write_index
    }

    pub fn is_full(&self) -> bool {
        let mut next_write = self.read_index + 1;
        next_write %= N;

        next_write == self.read_index
    }
}
