mod lx_16a;

use std::time::Duration;
use lx_16a::Lx16aBus;

const SERIAL_PORT: &str = "/dev/ttyUSB0";
const BAUD: u32 = 115200;

const RIGHT_FRONT_STEER_ID: u8 = 1;
const RIGHT_BACK_STEER_ID: u8 = 4;
const LEFT_BACK_STEER_ID: u8 = 6;
const LEFT_FRONT_STEER_ID: u8 = 9;

// const RIGHT_FRONT_STRAIGHT: u32 = 397;
// const RIGHT_BACK_STRAIGHT: u32 = 509;
// const LEFT_BACK_STRAIGHT: u32 = 447;
// const LEFT_FRONT_STRAIGHT: u32 = 564;

fn main() -> anyhow::Result<()> {
    println!("Hello, world!");

    let port = serialport::new(SERIAL_PORT, BAUD)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");
    let lx16a_bus = Lx16aBus::new(port);

    let right_front = lx16a_bus.servo(RIGHT_FRONT_STEER_ID);
    let right_back  = lx16a_bus.servo(RIGHT_BACK_STEER_ID);
    let left_back   = lx16a_bus.servo(LEFT_BACK_STEER_ID);
    let left_front  = lx16a_bus.servo(LEFT_FRONT_STEER_ID);
    
    for servo in [&right_front, &right_back, &left_back, &left_front] {
    	let read_id = servo.read_servo_id().unwrap();
        let temp = servo.read_temp_c().unwrap();
        let pos = servo.read_pos().unwrap();
        let vin_mv = servo.read_vin_mv().unwrap();
        println!("Servo {}:", servo.get_id());
        println!("    id :{read_id}, temp: {temp}, position: {pos}, vin [mv]: {vin_mv}");

        // Move to position 500 over 1 sec.
        servo.move_wait(500, 1000)?;
    }

    // start the move
    lx16a_bus.broadcast().move_start()?;

    Ok(())
}
