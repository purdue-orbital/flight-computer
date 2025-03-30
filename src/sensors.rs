use ublox::*;
use ublox::{
    CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode, UartPortId,
};
use mpu9250::{Mpu9250, Marg};
use bmp280::{Bmp280, Bmp280Builder};
use embedded_hal::spi::SpiDevice;
use rppal::spi::{Bus, SlaveSelect, Spi};
use log::*;
use rppal::i2c::I2c;
use std::sync::mpsc::{
    self,
    Receiver,
    RecvError,
    Sender,
};

pub type Mpu9250Device = Mpu9250<Spi, Marg>;

pub fn init_sensors() -> (Bmp280, Mpu9250Device, GPSDevice) {
    let baro = Bmp280Builder::new()
        .path("/dev/i2c-1")
        .address(0x76)
        .build()
        .expect("BMP280 init");

    let mut spi = rppal::spi::Spi::new(rppal::spi::Bus::Spi1, rppal::spi::SlaveSelect::Ss0, 8_000_000, rppal::spi::Mode::Mode0).expect("SPI init");
    let cs = rppal::gpio::Gpio::new().expect("gpio init").get(27).expect("chip select pin").into_output(); // TODO: which pin is chip select pin for spi1
    let delay = Delay::new();
    let mpu = Mpu9250::marg_default(spi, cs, &mut delay).expect("MPU9250 init");

    let gps = initialize_gps().expect("GPS init failed");

    let (gps_tx, gps_rx) = mpsc::channel();

    spawn_gps_thread(gps, gps_tx);

    (baro, mpu, gps_rx)
}

fn spawn_gps_thread(gps: GPSDevice, gps_tx: Sender<(f32, f32, f32)>) {
    std::thread::spawn(move || {
        let mut last_packet = Instant::now();
        loop {
            gps
                .update(|packet| match packet {
                    PacketRef::MonVer(packet) => {
                        debug!(
                            "SW version: {} HW version: {}; Extensions: {:?}",
                            packet.software_version(),
                            packet.hardware_version(),
                            packet.extension().collect::<Vec<&str>>()
                        );
                        debug!("{:?}", packet);
                    },
                    PacketRef::NavPvt(sol) => {
                        let has_time = sol.fix_type() == GpsFix::Fix3D
                            || sol.fix_type() == GpsFix::GPSPlusDeadReckoning
                            || sol.fix_type() == GpsFix::TimeOnlyFix;
                        let has_posvel = sol.fix_type() == GpsFix::Fix3D
                            || sol.fix_type() == GpsFix::GPSPlusDeadReckoning;
    
                        if has_posvel {
                            let pos: Position = (&sol).into();
                            let vel: Velocity = (&sol).into();
                            // println!(
                            //     "Latitude: {:.5} Longitude: {:.5} Altitude: {:.2}m",
                            //     pos.lat, pos.lon, pos.alt
                            // );
                            // println!(
                            //     "Speed: {:.2} m/s Heading: {:.2} degrees",
                            //     vel.speed, vel.heading
                            // );
                            match gps_tx.send((pos.lat, pos.lon, pos.alt)) {
                                Ok(_) => {}
                                Err(mpsc::SendError(_)) => {
                                    error!("GPS data failed to send from GPS thread");
                                }
                            }
                            info!("TELEMETRY: Lat {:.5} Long {:5} Alt {:.2} m Spd {:.2} m/s Head {:.2} deg", pos.lat, pos.lon, pos.alt, vel.speed, vel.heading);
                            // println!("Sol: {:?}", sol);
                            last_packet = Instant::now();
                        }
    
                        if has_time {
                            let time: DateTime<Utc> = (&sol)
                                .try_into()
                                .expect("Could not parse NAV-PVT time field to UTC");

                            let time_result: Result<DateTime<Utc>, _> = time.try_into();
                            match time_result {
                                Ok(time) => {
                                    info!("GPS TIME (UTC): {:?}", time);
                                }
                                Err(_) => {
                                    error!("Failed to parse GPS time");
                                }
                            }
                        }
                    },
                    _ => {
                        println!("{:?}", packet);
                    },
                })
                .unwrap();

            if Instant::now() > last_packet + Duration::from_millis(600 * 1000) && !state.read().expect("shared state read").cut_done
            {
                info!("No GPS for over 10 minutes! Cutting down.");
                gpio_actions.send(GpioAction::Drop(None)).expect("gpio action channel to be open");
            }
        }

        info!("GPS Shutdown");
    });
}

fn get_all_mpu(mpu: &mut Mpu9250Device) -> Result<(f32, (f32, f32, f32), (f32, f32, f32)), rppal::spi::Error> {
    let readings = mpu.all()?;
    Ok( (readings.temp, readings.gyro, readings.accel) )
}

fn get_baro_readings(baro: &mut Bmp280) -> Result<(f32, f32, f32), rppal::i2c::Error> {
    let temp = baro.temperature_celsius()?;
    let pressure = baro.pressure_kpa()?;
    let altitude = baro.altitude_m()?;
    Ok( (temp, pressure, altitude) )
}

fn get_gps_readings(gps_rx: Receiver<(f32, f32, f32)>) -> Option<(f32, f32, f32)> {
    let mut lat = 0.0;
    let mut lon = 0.0;
    let mut alt = 0.0;

    result = gps_rx.try_recv();
    match Result {
        Ok((lat, lon, alt)) => {
            info!("GPS: Lat {:.5} Long {:5} Alt {:.2} m", lat, lon, alt);
        }
        Err(mpsc::TryRecvError::Empty) => {
            // No data available
            return None;
        }
        Err(mpsc::TryRecvError::Disconnected) => {
            // Channel disconnected
            return None;
        }        
    }

    Ok( (lat, lon, alt) )
}

#[derive(Debug)]
enum GpsError {
    SerialError(serialport::Error),
}

fn initialize_gps() -> Result<GPSDevice, GpsError> {
    let builder = serialport::new("/dev/ttyS0", 9600)
        .flow_control(serialport::FlowControl::None)
        .data_bits(serialport::DataBits::Eight)
        .stop_bits(serialport::StopBits::One)
        .open()
        .map_err(|e| GpsError::SerialError(e))?;

    let mut device = GPSDevice::new(builder);

    // Configure the device to talk UBX
    debug!("Configuring UART1 port ...");
    device
        .write_with_ack::<CfgPrtUart>(
            &CfgPrtUartBuilder {
                portid: UartPortId::Uart1,
                reserved0: 0,
                tx_ready: 0,
                mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
                baud_rate: 9600,
                in_proto_mask: InProtoMask::UBLOX,
                out_proto_mask: OutProtoMask::union(OutProtoMask::NMEA, OutProtoMask::UBLOX),
                flags: 0,
                reserved5: 0,
            }
            .into_packet_bytes(),
        )
        .expect("Could not configure UBX-CFG-PRT-UART");
    // TODO: Make retries automatic
    // device
    //     .wait_for_ack::<CfgPrtUart>()
    //     .expect("Could not acknowledge UBX-CFG-PRT-UART msg");

    // Enable the NavPvt packet
    device
        .write_with_ack::<CfgMsgAllPorts>(
            &CfgMsgAllPortsBuilder::set_rate_for::<NavPvt>([0, 1, 0, 0, 0, 0]).into_packet_bytes(),
        )
        .expect("Could not configure ports for UBX-NAV-PVT");
    // device
    //     .wait_for_ack::<CfgMsgAllPorts>()
    //     .expect("Could not acknowledge UBX-CFG-PRT-UART msg");

    // Send a packet request for the MonVer packet
    device
        .write_all(&UbxPacketRequest::request_for::<MonVer>().into_packet_bytes())
        .expect("Unable to write request/poll for UBX-MON-VER message");

    // Start reading data
    debug!("Opened uBlox device, waiting for messages...");

    Ok(device)
}

// Shamelessly copied https://github.com/ublox-rs/ublox/blob/master/examples/basic_cli/src/main.rs
pub struct GPSDevice {
    port: Box<dyn serialport::SerialPort>,
    parser: Parser<Vec<u8>>,
}

const UART_TIMEOUT: Duration = Duration::from_millis(1000);
const UART_RETRIES: usize = 10;

impl GPSDevice {
    pub fn new(port: Box<dyn serialport::SerialPort>) -> GPSDevice {
        let parser = Parser::default();
        GPSDevice { port, parser }
    }

    pub fn write_with_ack<T: UbxPacketMeta>(&mut self, data: &[u8]) -> std::io::Result<()> {
        // First write the packet
        self.write_all(data)?;

        // Start waiting for ACK, allowing retries until max is hit
        let mut attempts = 0;
        while attempts < UART_RETRIES {
            match self.wait_for_ack::<T>() {
                Ok(()) => return Ok(()),
                Err(e) => match e.kind() {
                    ErrorKind::TimedOut => {
                        // Didn't receive packet, try again
                        self.write_all(data)?;
                        attempts += 1;
                    }
                    _ => return Err(e),
                },
            }
        }

        Err(std::io::Error::from(ErrorKind::TimedOut))
    }

    pub fn write_all(&mut self, data: &[u8]) -> std::io::Result<()> {
        self.port.write_all(data)
    }

    pub fn update<T: FnMut(PacketRef)>(&mut self, mut cb: T) -> std::io::Result<()> {
        loop {
            const MAX_PAYLOAD_LEN: usize = 1240;
            let mut local_buf = [0; MAX_PAYLOAD_LEN];
            let nbytes = self.read_port(&mut local_buf)?;
            if nbytes == 0 {
                break;
            }

            // parser.consume adds the buffer to its internal buffer, and
            // returns an iterator-like object we can use to process the packets
            let mut it = self.parser.consume(&local_buf[..nbytes]);
            loop {
                match it.next() {
                    Some(Ok(packet)) => {
                        cb(packet);
                    }
                    Some(Err(_)) => {
                        // Received a malformed packet, ignore it
                    }
                    None => {
                        // We've eaten all the packets we have
                        break;
                    }
                }
            }
        }
        Ok(())
    }

    pub fn wait_for_ack<T: UbxPacketMeta>(&mut self) -> std::io::Result<()> {
        let mut found_packet = false;
        let start = Instant::now();
        while !found_packet {
            self.update(|packet| {
                if let PacketRef::AckAck(ack) = packet {
                    if ack.class() == T::CLASS && ack.msg_id() == T::ID {
                        found_packet = true;
                    }
                }
            })?;

            if start.elapsed() > UART_TIMEOUT && !found_packet {
                return Err(std::io::Error::from(ErrorKind::TimedOut));
            }
        }
        Ok(())
    }

    /// Reads the serial port, converting timeouts into "no data received"
    fn read_port(&mut self, output: &mut [u8]) -> std::io::Result<usize> {
        match self.port.read(output) {
            Ok(b) => Ok(b),
            Err(e) => {
                if e.kind() == std::io::ErrorKind::TimedOut {
                    Ok(0)
                } else {
                    Err(e)
                }
            }
        }
    }
}
