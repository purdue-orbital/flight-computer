use log::*;
use std::iter::Rev;
use std::sync::mpsc::{
    self,
    Receiver,
    TryRecvError,
    Sender,
};

use std::fs::File;
use std::io;
use std::io::prelude::*;

/// returns sensor handles: (mpu, barometer, gps)
pub fn init_sensors() -> (MpuData, BaroData, Receiver<(f32, f32, f32)>) {
    let mpu_file = File::open("mpu_data.txt").unwrap();
    let baro_file = File::open("baro_data.txt").unwrap();
    let gps_file = File::open("gps_data.txt").unwrap();

    let (gps_tx, gps_rx) = mpsc::channel();

    std::thread::spawn(move || {
        loop {
            let mut contents = String::new();
            gps_file.read_to_string(&mut contents).unwrap();
            for line in contents.split('\n') {
                let line = line.unwrap();
                let parts: Vec<&str> = line.split(',').collect();
                if parts.len() >= 5 {
                    let lat: f32 = parts[0].parse().unwrap();
                    let lon: f32 = parts[1].parse().unwrap();
                    let alt: f32 = parts[2].parse().unwrap();
                    let speed: f32 = parts[3].parse().unwrap();
                    let heading: f32 = parts[4].parse().unwrap();
                    let time: f32 = parts[5].parse().unwrap();

                    info!("TELEMETRY: Time: {:.5} Lat {:.5} Long {:5} Alt {:.2} m Spd {:.2} m/s Head {:.2} deg", time, lat, lon, alt, speed, heading);

                    gps_tx.send((lat, lon, alt)).unwrap();
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(500));
        }
    })

    (MpuData::new(mpu_file), BaroData::new(baro_file), gps_rx)
}

pub struct BaroData {
    file_contents: Vec<(f32, f32, f32)>,
    line: usize,
}

impl BaroData {
    pub fn new(file: File) -> BaroData {
        let mut file_contents_string = String::new();
        file.read_to_string(&mut file_contents_string).unwrap();

        let mut contents = Vec::new();
        for line in contents.split('\n') {
            let line = line.unwrap();
            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() >= 3 {
                let temperature: f32 = parts[0].parse().unwrap();
                let pressure: f32 = parts[1].parse().unwrap();
                let altitude: f32 = parts[2].parse().unwrap();
                contents.push((temperature, pressure, altitude));
            }
        }

        BaroData {
            file_contents: contents,
            line: 0,
        }
    }

    pub fn log_next_reading(&mut self) -> (f32, f32, f32) {
        self.line += 1;
        if self.line < self.file_contents.len() {
            self.file_contents[self.line]
        } else {
            self.line = 0;
            self.file_contents[self.line]
        }
    }
}

pub struct MpuData {
    file_contents: Vec<(f32, (f32, f32, f32), (f32, f32, f32))>,
    line: usize,
}

impl MpuData {
    pub fn new(file: File) -> MpuData {
        let mut file_contents_string = String::new();
        file.read_to_string(&mut file_contents_string).unwrap();

        let mut contents = Vec::new();
        for line in contents.split('\n') {
            let line = line.unwrap();
            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() >= 3 {
                let mut accel: (f32, f32, f32);
                let mut gyro: (f32, f32, f32);

                let temperature: f32 = parts[0].parse().unwrap();

                accel.0 = parts[1].parse().unwrap();
                accel.1 = parts[2].parse().unwrap();
                accel.2 = parts[3].parse().unwrap();

                gyro.0 = parts[4].parse().unwrap();
                gyro.1 = parts[5].parse().unwrap();
                gyro.2 = parts[6].parse().unwrap();

                contents.push((temperature, accel, gyro));
            }
        }

        MpuData {
            file_contents: contents,
            line: 0,
        }
    }

    pub fn log_next_reading(&mut self) -> (f32, (f32, f32, f32), (f32, f32, f32)) {
        self.line += 1;
        if self.line < self.file_contents.len() {
            self.file_contents[self.line]
        } else {
            self.line = 0;
            self.file_contents[self.line]
        }
    }
}

pub fn poll_mpu(mpu_data: &mut MpuData) -> (f32, f32, f32) {
    (_, accel, _) = mpu_data.log_next_reading();
    accel
}
pub fn poll_barometer(baro_data: &mut BaroData) -> (f32, f32) {
    (_, _, altitude) = baro_data.log_next_reading();
    altitude
}

pub fn poll_gps(gps_rx: &mut Receiver<(f32, f32, f32)>) -> Option<(f32, f32, f32)> {
    match gps_rx.try_recv() {
        Ok(data) => {
            Some(data)
        }
        Err(TryRecvError::Empty) => {
            None
        }
        Err(TryRecvError::Disconnected) => {
            error!("GPS thread disconnected");
            None
        }
    }
}