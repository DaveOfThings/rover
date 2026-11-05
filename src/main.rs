mod rover;
mod drive;
mod control_link;

use crate::control_link::ControlLink;
use tokio::{select, sync::mpsc};
use crate::drive::DriveTrain;

use rover::Rover;

#[tokio::main]
async fn main() {    
    // Signal the app to exit by writing to quit_tx
    // (Gamepad menu button triggers this.)
    let (quit_tx, mut quit_rx) = mpsc::channel::<()>(1);

    // Create RobotLink and DriveTrain
    let control_link = ControlLink::new(quit_tx);    // task to manage MQTT link
    let drive = DriveTrain::new();                    // drive subsystem

    // Create Rover to convert MQTT inputs into drive train outputs
    let rover = Rover::new(&control_link, &drive);     // task to direct robot actions


    // Run all the tasks.  If any one quits, the app ends.
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

    // TODO : Better Error handling
    let _ = drive.set_speed(0.0, 0.0).await;
    let _ = drive.set_powered(false).await;

    println!("All done.");
}