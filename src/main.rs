use flight_builder::prelude::*;
use std::{ptr::read, time::{Duration, Instant}};
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

    s.add_resource(state); // Add the state as a resource.

    s.add_task(Schedule::Update(POLL_FREQ as f32 / 1000000.0), states); // Define our tasks - most of 'em don't have to run all that often!
    s.add_task(Schedule::Update(0.1), barometer);
    s.add_task(Schedule::Update(0.1), find_sink_rate);
    s.add_task(Schedule::Update(1.0), gps);
    s.add_task(Schedule::Update(0.1), imu);
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
    state.sink_rate = (state.barometric_alts[19] - state.barometric_alts[0]) /
        state.barometric_timestamps[19].duration_since(state.barometric_timestamps[0]).as_secs_f32();
}

fn barometer(mut state: ResMut<State>){
    let read_value = poll_barometer(); // TODO: implement this!
    // Shift everything in this array right by one (except for element 0, which is overwritten,
    // and then add the new value at the end.)
    for i in 1..state.barometric_alts.len(){
        state.barometric_alts[i - 1] = state.barometric_alts[i];
    }
    state.barometric_alts[state.barometric_alts.len()] = read_value;

    // Do it again, but for timestamps. 
    for i in 1..state.barometric_timestamps.len(){
        state.barometric_timestamps[i - 1] = state.barometric_timestamps[i];
    }
    state.barometric_timestamps[state.barometric_timestamps.len()] = Instant::now();
}

fn imu(mut state: ResMut<State>){
    
}

fn gps(mut state: ResMut<State>){
}

const POLL_FREQ: u128 = 10000; //Frequency of polling in microseconds (10000 = 1/100 of a second, 1000 = 1/1000, et cetera.)

// If these tasks don't end up causing a move to the next state in the sequence,
// they'll return their own state. Otherwise, they'll return the state they cause
// a move to.

// Ideally, these tasks should be scheduled so that they only ever need data from the state. If they need to mutate the state somehow, we're
// doing something wrong.

fn init_tasks(state: &State) -> FlightStates{
    return state.state;
}

fn grounded_tasks(state: &State) -> FlightStates{
    
    return state.state;
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