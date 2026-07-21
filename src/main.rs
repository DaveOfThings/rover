mod rover;
mod drive;
mod control_link;

use crate::control_link::ControlLink;
use tokio::main;
use std::time::Duration;
use lx_16a::Lx16aBus;
use std::thread;
use crate::drive::DriveTrain;

use rover::Rover;



const RIGHT_FRONT_STEER_ID: u8 = 1;
const RIGHT_BACK_STEER_ID: u8 = 4;
const LEFT_BACK_STEER_ID: u8 = 6;
const LEFT_FRONT_STEER_ID: u8 = 9;


/*
fn main() -> anyhow::Result<()> {

    let port = serialport::new(SERIAL_PORT, BAUD)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");

    let lx16a_bus = Lx16aBus::new(port);

    let right_front = lx16a_bus.servo(RIGHT_FRONT_STEER_ID);
    let right_back  = lx16a_bus.servo(RIGHT_BACK_STEER_ID);
    let left_back   = lx16a_bus.servo(LEFT_BACK_STEER_ID);
    let left_front  = lx16a_bus.servo(LEFT_FRONT_STEER_ID);
    
    println!("Hello LX-16A world.");
    for servo in [&right_front, &right_back, &left_back, &left_front] {
    	let read_id = servo.read_servo_id().unwrap();
        println!("id : {read_id}");
    }

    for servo in [&right_front, &right_back, &left_back, &left_front] {
    	// let read_id = servo.read_servo_id().unwrap();
        // let temp = servo.read_temp_c().unwrap();
        // let pos = servo.read_pos().unwrap();
        // let vin_mv = servo.read_vin_mv().unwrap();
        // println!("Servo {}:", servo.get_id());
        // println!("    id :{read_id}, temp: {temp}, position: {pos}, vin [mv]: {vin_mv}");

        // Move to position 500 over 1 sec.
        servo.move_wait(500, 1000)?;
    }

    // start the move
    println!("Powering on.");
    lx16a_bus.broadcast().set_powered(true)?;
    lx16a_bus.broadcast().move_start()?;
    println!("Moving.");

    // thread::sleep(Duration::from_millis(1100));

    let mut rover = Rover::new(&lx16a_bus);
    rover.wiggle();
    rover.drive_turn(0.2, -std::f64::consts::PI/10.0)?;  // PI/2 radians (1/4 turn) in 5 seconds.

    thread::sleep(Duration::from_millis(5000));
    rover.drive(0.0)?;

    lx16a_bus.broadcast().set_powered(false)?;
    println!("Powered off.");

    
    thread::sleep(Duration::from_millis(100));

    Ok(())
}
    */


#[tokio::main]
async fn main() {
    // Create RobotLink
    let control_link = ControlLink::new();             // task to manage MQTT link
    let drive = DriveTrain::new();                          // drive subsystem

    let rover = Rover::new(&robot_link, &drive);     // task to direct robot actions
    
    let (quit_tx, mut quit_rx) = mpsc::channel(1);     // signal to shut down. 

    // Run all the tasks.  If one quits, the app ends.
    select! {
        _ = quit_rx.recv() => {
            println!("Quit signalled.");
        },
        _ = control_link.run() => { 
            println!("mqtt link quit.");
        },
        _ = rover.run() => {
            println!("rover quit.");
        }
        _ = drive.run() => {
            println!("Drive task quit.");
        }
    };

    println!("All done.");
}