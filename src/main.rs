use flight_builder::prelude::*;
use flight_computer::poll_gps;
use std::{ptr::read, time::{Duration, Instant}};
use flexi_logger;
use log::*;

#[derive(Copy, Clone)]
enum FlightStates{
    Init, // Initialization. Make sure that the radio's connected and that we're ready to start flying. 
    Grounded, // Still on the ground. Continually check for sustained positive rate.
    Ascending, // We're rising now. Await either radio signals from the ground or certain barometric
    // pressure readings - when we get them, activate whichever system they tell us to (rocket ignition, balloon
    // cut/pop, parachute deploy, et cetera.)
    PostCut, // We've cut the balloon and deployed the chute, but we still haven't reached a terminal velocity
    // (terminal w/ chute, of course!). Check until we do.
    Descending, // We're descending at a constant rate. Just keep checking that we haven't hit the ground yet
    // (Sink rate > 0).
    Landed, // We're on the ground! Yahoo.
}

enum Commands{
    LaunchRocket,
    LetDown,
    ActivateGNC,
    CutDown1,
    CutDown2,
}

const LOGGED_ALTS: usize = 20;

#[derive(Copy, Clone)]
struct State {
    state: FlightStates, // The current flight state.
    barometric_alts: [f32; LOGGED_ALTS], // The last twenty read barometric altitudes.
    barometric_timestamps: [Instant; LOGGED_ALTS], // The timestamps these altitudes were read at (should be uniform, but you never know!)
    gnc_activation_timestamp: Instant,
    sink_rates: [f32; LOGGED_ALTS], // The calculated vertical velocity from the previous two entries.
    location: (f32, f32, f32), // The location last polled from the GPS.
    acceleration: (f32, f32, f32), // The acceleration last polled from the IMUs.
}

struct Signals {
    gnc_activation: bool,
    launch_rocket: bool,
    let_down: bool,
    cut_down_1: bool,
    cut_down_2: bool,
}

fn main() {
    flexi_logger::Logger::try_with_env_or_str("info")?.start()?;
    let mut s = Scheduler::new(); //Create a scheduling object.

    let state = State{ // Initialize a state object.
        state: FlightStates::Init, 
        barometric_alts: [0.0; LOGGED_ALTS],
        barometric_timestamps: [Instant::now(); LOGGED_ALTS],
        gnc_activation_timestamp: Instant::now(),
        sink_rates: [0.0; LOGGED_ALTS],
        location: (0.0, 0.0, 0.0),
        acceleration: (0.0, 0.0, 0.0),
    };

    let has_done = Signals{
        gnc_activation: false,
        launch_rocket: false,
        let_down: false,
        cut_down_1: false,
        cut_down_2: false,
    };

    let (mpu, baro, gps) = init_sensors();
    
    s.add_resource(mpu); // Add the IMU as a resource.
    s.add_resource(baro); // Add the barometer as a resource.
    s.add_resource(gps); // Add the GPS as a resource.
    s.add_resource(state); // Add the state as a resource.

    s.add_task(Schedule::Update(POLL_FREQ as f32 / 1000000.0), states); // Define our tasks - most of 'em don't have to run all that often!
    s.add_task(Schedule::Update(0.01), barometer);
    s.add_task(Schedule::Update(0.01), find_sink_rate);
    s.add_task(Schedule::Update(0.01), gps);
    s.add_task(Schedule::Update(0.01), imu);
}

fn states(mut state: ResMut<State>){ // Run the functia corresponding to each state.
    state.state = match state.state {
        FlightStates::Init => init_tasks(&state),
        FlightStates::Grounded => grounded_tasks(&state),
        FlightStates::Ascending => ascending_tasks(&state, &has_done),
        FlightStates::PostCut => post_cut_tasks(&state),
        FlightStates::Descending => descending_tasks(&state),
        FlightStates::Landed => landed_tasks(&state),
    };
}

fn find_sink_rate(mut state: ResMut<State>){ // Use the ends of the barometric altitude / timestamp array to calculate a sink rate (v. velocity)
    for i in 1..LOGGED_ALTS - 1(){
        state.sink_rates[i - 1] = state.sink_rates[i];
    }
    state.sink_rates[LOGGED_ALTS - 1] = ((state.barometric_alts[LOGGED_ALTS - 1] - state.barometric_alts[0]) /
        state.barometric_timestamps[LOGGED_ALTS - 1].duration_since(state.barometric_timestamps[0]).as_secs_f32()); 
        //Average this w/ the previous value.
        // We don't want a sudden spike causing something to get kippered up. That can still happen, but it's less likely this way.
        // EDIT 02-APR-2025: This should actually be even less likely now, since there's a new filter implemented on the initial input
        // of barometry data (upon which everything else is based). Still, I'm keeping this code here - redundancy is good, right?
    new_rate = state.sink_rates[LOGGED_ALTS - 1];
    info!("Vertical Velocity: {new_rate}.\n");
}

fn barometer(mut state: ResMut<State>, mut baro: ResMut<BaroData>){ // Read the barometer and update the state.
    let read_value = poll_barometer(&mut baro);
    // Shift everything in this array right by one (except for element 0, which is overwritten,
    // and then add the new value at the end.)
    for i in 1..state.barometric_alts.len(){
        state.barometric_alts[i - 1] = state.barometric_alts[i];
    }
    let prev_avg = 
    (state.barometric_alts[state.barometric_alts.len() - 2] +
    state.barometric_alts[state.barometric_alts.len() - 3] +
    state.barometric_alts[state.barometric_alts.len() - 4] ) / 3.0;
    // Take the average of the previous three values, so that we can filter out sudden spikes.

    let filter_const = 1.1.powf(-(read_value - prev_avg)); //This constant controls how much the current value should be "trusted".
    // If it's very high compared to the average of the prior values, then this constant will evaluate to a low value.
    // Turn the constant (1.1 here) higher in order to be more strict about it, and lower to be more lax.

    let new_rate = prev_avg + filter_const * (read_value - prev_avg);
    state.barometric_alts[state.barometric_alts.len() - 1] = new_rate;

    info!("Barometric Pressure: {new_rate}.\n");

    // Do it again, but for timestamps. 
    for i in 1..state.barometric_timestamps.len(){
        state.barometric_timestamps[i - 1] = state.barometric_timestamps[i];
    }

    state.barometric_timestamps[state.barometric_timestamps.len()] = Instant::now();
}

fn imu(mut state: ResMut<State>, mut mpu: ResMut<MpuData>){
    let read_values = poll_imu(&mut mpu);
    let read_x = read_values.0;
    let read_y = read_values.1;
    let read_z = read_values.2;
    let stored_x = (read_x + state.acceleration.0) / 2.0;
    let stored_y = (read_y + state.acceleration.1) / 2.0;
    let stored_z = (read_z + state.acceleration.2) / 2.0; //More con-pre-value averaging!
    state.acceleration = read_values;
}

fn gps(mut state: ResMut<State>, mut gps: ResMut<Receiver<(f32, f32, f32)>>){
    let read_values = poll_gps(&mut gps);
    let read_x = read_values.0;
    let read_y = read_values.1;
    let read_z = read_values.2; // We don't do that here, though - gps data is reliable and non-critical enough that it seems a bit silly.
    // We're gathering it mostly to log it, anyway.
    state.acceleration = read_values;
}

const POLL_FREQ: u128 = 10000; //Frequency of polling in microseconds (10000 = 1/100 of a second, 1000 = 1/1000, et cetera.)

// If these tasks don't end up causing a move to the next state in the sequence,
// they'll return their own state. Otherwise, they'll return the state they cause
// a move to.

// Ideally, these tasks should be scheduled so that they only ever need data from the state. If they need to mutate the state somehow, we're
// doing something wrong.

const LIFTOFF_DETECTION_RATE: i32 = 10;

fn init_tasks(mut state: ResMut<State>) -> FlightStates{ //...What do we need to do here?
    return FlightStates::Grounded;
}

fn grounded_tasks(mut state: ResMut<State>) -> FlightStates{
    if (state.acceleration <= LIFTOFF_DETECTION_RATE || state.sink_rates[LOGGED_ALTS - 1] <= LIFTOFF_DETECTION_RATE){
        return state.state;
    } else {
        info!("Launch detected! Transitioning to ascending state.");
        return FlightStates::Ascending;
    }
}

const GNC_ALTITUDE: f32 = (5000.0 / 3.2808);
const CUT_DOWN_ALTITUDE: f32 = (5000.0 / 3.2808);
const LET_DOWN_ALTITUDE: f32 = (5000.0 / 3.2808);

fn ascending_tasks(mut state: ResMut<State>, mut has_done: ResMut<Signals>) -> FlightStates{
    /*let signals = Signals{
        launch_rocket: false,
        let_down: false,
        cut_down_1: false,
        cut_down_2: false,
    }; //In futuro, this will come from some other piece of code, but I'm defining it manually here (all-false) as a temporary measure.
    // Just remember that none of the code oughta mutate it!*/
    
    if (state.barometric_alts[LOGGED_ALTS - 1] >= GNC_ALTITUDE && !has_done.gnc_activation){
        send_command_to_board(Commands::ActivateGNC, 0);
        state.gnc_activation_timestamp = Instant::now();
        info!("GNC Activation altitude reached!");
        has_done.gnc_activation = true;
    }

    if (has_done.gnc_activation && !has_done.launch_rocket){
        if (Instant::now().duration_since(state.gnc_activation_timestamp).as_secs() > 15) {
            info!("Rocket launched!");
            send_command_to_board(Commands::LaunchRocket, 0);
            has_done.launch_rocket = true;
        }
    }

    if (has_done.launch_rocket){
        if (state.barometric_alts[LOGGED_ALTS - 1] >= LET_DOWN_ALTITUDE && !has_done.let_down){
            info!("Letting down...");
            send_command_to_board(Commands::LetDown, 0);
            has_done.let_down = true;
        }
        if (has_done.cut_down_1){
            if (state.sink_rates[LOGGED_ALTS - 1] >= 0 && state.barometric_alts[LOGGED_ALTS - 1] >= CUT_DOWN_ALTITUDE){
                info!("Still rising after first cut down, doing second...");
                send_command_to_board(Commands::CutDown2, 0);
                has_done.cut_down_2 = true;
            }
        } else {
            if (state.barometric_alts[LOGGED_ALTS - 1] >= CUT_DOWN_ALTITUDE){
                info!("Cutting down...");
                send_command_to_board(Commands::CutDown1, 0);
                has_done.cut_down_1 = true;
            }
        }
    }
    return state.state;
}

fn post_cut_tasks(mut state: ResMut<State>) -> FlightStates{
    if (state.sink_rates[LOGGED_ALTS - 1] - state.sink_rates[LOGGED_ALTS - 2]) 
    + (state.sink_rates[LOGGED_ALTS - 2] - state.sink_rates[LOGGED_ALTS - 3]).abs() <= 0.1{
        info!("Constant vertical descent rate reached, transitioning to descent phase...");
        return FlightStates::Descending;
    }
    return state.state;
}

fn descending_tasks(mut state: ResMut<State>) -> FlightStates{
    if (state.sink_rates[LOGGED_ALTS - 1] + state.sink_rates[LOGGED_ALTS - 2]) 
    + state.sink_rates[LOGGED_ALTS - 3] <= 0.1{
        info!("Landing detected!");
        return FlightStates::Landed;
    }
    return state.state;
}

fn landed_tasks(mut state: ResMut<State>) -> FlightStates{
    return state.state;
}

const BAUD_RATE: u32 = 19200;
fn send_command_to_board(command: Commands, tries: u8){
    let uart = Uart::new(BAUD_RATE, Parity::None, 8, 1)?;
    let sent_packet: [u8] = [Commands as u8];
    let write_result = uart.write(&sent_packet);
    match write_result{
        Err(error) => {
            if (tries <= 10){
                warn!("Error communicating with board!! {error}");
                send_command_to_board(command, tries + 1);
            } else {
                warn!("Ten consecutive failed communications with board! Aborting communication attempt!");
                return;
            }
        },
        _ => return,
    }
}

// A, pehdaun - e a makina, e'a kohasaun
// A ki faj a hicmu, a hima, i a tuda kada pahci du kansaun.