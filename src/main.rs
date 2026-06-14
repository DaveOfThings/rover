// use lx16a_servo::{ServoBus, Servo, ServoID};

mod lx_16a;

use std::time::Duration;

use serialport::SerialPort;
use lx_16a::{SerialBus, Servo};

const SERIAL_PORT: &str = "/dev/ttyUSB0";
const BAUD: u32 = 115200;

const RIGHT_FRONT_STEER_ID: u8 = 1;
const RIGHT_BACK_STEER_ID: u8 = 4;
const LEFT_BACK_STEER_ID: u8 = 6;
const LEFT_FRONT_STEER_ID: u8 = 9;

const RIGHT_FRONT_STRAIGHT: u32 = 397;
const RIGHT_BACK_STRAIGHT: u32 = 509;
const LEFT_BACK_STRAIGHT: u32 = 447;
const LEFT_FRONT_STRAIGHT: u32 = 564;

fn main() -> anyhow::Result<()> {
    println!("Hello, world!");

    let port = serialport::new(SERIAL_PORT, BAUD)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");
    let serial_bus = SerialBus::new(port);

    let right_front = Servo::new(RIGHT_FRONT_STEER_ID, &serial_bus);
    let right_back  = Servo::new(RIGHT_BACK_STEER_ID,  &serial_bus);
    let left_back   = Servo::new(LEFT_BACK_STEER_ID,   &serial_bus);
    let left_front  = Servo::new(LEFT_FRONT_STEER_ID,  &serial_bus);
    
    for servo in [&right_front, &right_back, &left_back, &left_front] {
    	let read_id = servo.read_servo_id();
        println!("Servo {} reports id {read_id}", servo.get_id());
    }

    Ok(())
}
