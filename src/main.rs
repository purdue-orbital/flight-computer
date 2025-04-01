use flight_builder::prelude::*;
use flight_computer::poll_gps;
use std::{ptr::read, time::{Duration, Instant}};
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

#[derive(Copy, Clone)]
struct State {
    state: FlightStates, // The current flight state.
    barometric_alts: [f32; 20], // The last twenty read barometric altitudes.
    barometric_timestamps: [Instant; 20], // The timestamps these altitudes were read at (should be uniform, but you never know!)
    sink_rate: f32, // The calculated vertical velocity from the previous two entries.
    location: (f32, f32, f32), // The location last polled from the GPS.
    acceleration: (f32, f32, f32), // The acceleration last polled from the IMUs.
}

fn main() {
    let mut s = Scheduler::new(); //Create a scheduling object.

    let state = State{ // Initialize a state object.
        state: FlightStates::Init, 
        barometric_alts: [0.0; 20],
        barometric_timestamps: [Instant::now(); 20],
        sink_rate: 0.0,
        location: (0.0, 0.0, 0.0),
        acceleration: (0.0, 0.0, 0.0),
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
        FlightStates::Ascending => ascending_tasks(&state),
        FlightStates::PostCut => post_cut_tasks(&state),
        FlightStates::Descending => descending_tasks(&state),
        FlightStates::Landed => landed_tasks(&state),
    };
}

fn find_sink_rate(mut state: ResMut<State>){ // Use the ends of the barometric altitude / timestamp array to calculate a sink rate (v. velocity)
    state.sink_rate = (state.sink_rate + ((state.barometric_alts[19] - state.barometric_alts[0]) /
        state.barometric_timestamps[19].duration_since(state.barometric_timestamps[0]).as_secs_f32())) / 2.0; 
        //Average this w/ the previous value.
        // We don't want a sudden spike causing something to get kippered up. That can still happen, but it's less likely this way.
    new_rate = state.sink_rate;
    info!("Vertical Velocity: {new_rate}.\n");
}

fn barometer(mut state: ResMut<State>, mut baro: ResMut<BaroData>){ // Read the barometer and update the state.
    let read_value = poll_barometer(&mut baro);
    // Shift everything in this array right by one (except for element 0, which is overwritten,
    // and then add the new value at the end.)
    for i in 1..state.barometric_alts.len(){
        state.barometric_alts[i - 1] = state.barometric_alts[i];
    }
    state.barometric_alts[state.barometric_alts.len()] = 
    (read_value + 
    state.barometric_alts[state.barometric_alts.len() - 1] +
    state.barometric_alts[state.barometric_alts.len() - 2] +
    state.barometric_alts[state.barometric_alts.len() - 3] ) / 2.0;
    // Averaging again, as an anti-kippering mechanism. This time in quadruplicate.

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

fn init_tasks(state: &State) -> FlightStates{ //...What do we need to do here?
    return FlightStates::Grounded;
}

fn grounded_tasks(state: &State) -> FlightStates{
    if (state.acceleration <= LIFTOFF_DETECTION_RATE || state.sink_rate <= LIFTOFF_DETECTION_RATE){
        return state.state;
    } else {
        info!("Launch detected! Transitioning to ascending state.");
        return FlightStates::Ascending;
    }
}

fn ascending_tasks(state: &State) -> FlightStates{
    
    return state.state;
}

fn post_cut_tasks(state: &State) -> FlightStates{
    return state.state;
}

fn descending_tasks(state: &State) -> FlightStates{
    return state.state;
}

fn landed_tasks(state: &State) -> FlightStates{
    return state.state;
}

// Savaz aguaz di masu feixandu feran, ia promesa di fida no teu corasan.