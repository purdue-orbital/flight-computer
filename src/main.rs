#![no_std]
#![no_main]

use core::ptr::addr_of_mut;
use core::u8;
use embedded_alloc::LlffHeap as Heap;
use flight_builder::prelude::Scheduler;

use embassy_rp::gpio::{Level, Output};
use flight_builder::prelude::*;
use {defmt_rtt as _, panic_probe as _};

extern crate alloc;

#[global_allocator]
static HEAP: Heap = Heap::empty();
const TIMER_DELAY: f32 = 30.0;

struct State{
    time_elapsed: f32,
}

struct Pins<'a>{
    led_pin: Output<'a>,
    output_pin: Output<'a>,
}

pub fn increment_timer(mut pins: ResMut<Pins>, mut state: ResMut<State>) {
    if state.time_elapsed < TIMER_DELAY {
        if pins.led_pin.is_set_high(){
            pins.led_pin.set_low();
        } else {
            pins.led_pin.set_high();
        }
        state.time_elapsed = state.time_elapsed + 1.0;
    } else {
        pins.led_pin.set_high();
        pins.output_pin.set_high();
    } 
}

#[cortex_m_rt::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1280;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_25, Level::Low);
    let output_pin = Output::new(p.PIN_16, Level::Low);
    //led.set_high();

    let pins = Pins{
        led_pin: led,
        output_pin: output_pin,
    };

    let state = State {
        time_elapsed: 0.0,
    };
    //led.set_low();
    let mut s = Scheduler::default();

    s.add_resource(pins);
    s.add_resource(state);

    s.add_task(Startup, increment_timer);
    s.add_task(Update(1.0), increment_timer);

    s.build_with_clock::<133_000_000>().run();
}
