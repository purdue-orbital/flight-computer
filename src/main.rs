use flight_builder::prelude::*;
use std::time::{Duration, Instant};
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
    state: FlightStates,
    barometric_alts: (f32, f32),
    timestamps: (Instant, Instant),
    location: (f32, f32, f32),
    acceleration: (f32, f32, f32),
}

fn main() {
    let mut s = Scheduler::new();

    let state = State{
        state: FlightStates::Init,
        barometric_alts: (0.0, 0.0),
        timestamps: (Instant::now(), Instant::now()),
        location: (0.0, 0.0, 0.0),
        acceleration: (0.0, 0.0, 0.0),
    };

    s.add_resource(state);

    s.add_task(Schedule::Update(POLL_FREQ as f32 / 1000000.0), states);

    /*
    } */
}

fn states(mut state: ResMut<State>){
    state.state = match state.state {
        FlightStates::Init => init_tasks(&state),
        FlightStates::Grounded => grounded_tasks(&state),
        FlightStates::Ascending => ascending_tasks(&state),
        FlightStates::PostCut => post_cut_tasks(&state),
        FlightStates::Descending => descending_tasks(&state),
        FlightStates::Landed => landed_tasks(&state),
    };
    let poll_results = global_tasks(&state);
    if poll_results.0 {
        state.timestamps.0 = state.timestamps.1;
        state.timestamps.1 = Instant::now();

        state.barometric_alts.0 = state.barometric_alts.1;
        state.barometric_alts.1 = poll_results.1;

        state.acceleration = poll_results.2;
        state.location = poll_results.3;
    }
}

fn poll_barometer() -> f32{
    return 0.0;
}

fn poll_imu() -> (f32, f32, f32){
    return (0.0, 0.0, 0.0);
}

fn poll_gps() -> (f32, f32, f32) {
    return (0.0, 0.0, 0.0);
}

// These tasks should be run *in perpetuo* - e.g. polling the barometres, GPS, IMUs, logging the data we get from 'em,
// et cetera.

const POLL_FREQ: u128 = 10000; //Frequency of polling in microseconds (10000 = 1/100 of a second, 1000 = 1/1000, et cetera.)

fn global_tasks(state: &State) -> (bool, f32, (f32, f32, f32), (f32, f32, f32)){
    let mut updated = false;
    let mut barometer_result = 0.0;
    let mut imu_result = (0.0, 0.0, 0.0);
    let mut gps_result = (0.0, 0.0, 0.0);
    if state.timestamps.1.duration_since(state.timestamps.0).as_micros() > POLL_FREQ{
        updated = true;
        barometer_result = poll_barometer();
        imu_result = poll_imu();
        gps_result = poll_gps();
    }
    return (updated, barometer_result, imu_result, gps_result);
}

// If these tasks don't end up causing a move to the next state in the sequence,
// they'll return their own state. Otherwise, they'll return the state they cause
// a move to.

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