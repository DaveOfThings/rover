mod rover;
mod drive;
mod control_link;

use crate::control_link::ControlLink;
use tokio::{select, sync::mpsc};
use crate::drive::DriveTrain;

use rover::Rover;

#[tokio::main]
async fn main() {    
    // TODO-DW : Arrange for something to send to quit_tx to trigger exit.
    let (quit_tx, mut quit_rx) = mpsc::channel::<()>(1);     // signal to shut down. 

    // Create RobotLink
    let control_link = ControlLink::new(quit_tx);             // task to manage MQTT link
    let drive = DriveTrain::new();                          // drive subsystem

    let rover = Rover::new(&control_link, &drive);     // task to direct robot actions


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

    // TODO : Error handling
    let _ = drive.set_speed(0.0, 0.0);
    let _ = drive.set_powered(false);

    println!("All done.");
}