use std::time::Duration;

use crate::drive::DriveTrain;
use crate::control_link::ControlLink;
use serde::{Serialize, Deserialize};

#[derive(Clone, Copy, Default, Debug, Serialize, Deserialize)]
pub struct DriveSpeed {
    lin_mps: f32,
    curvature: f32,
}

#[derive(Clone, Copy, Default, Debug, Serialize, Deserialize)]
pub struct SpinRate {
    spin_rps: f32,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum RobotVel {
    Drive(DriveSpeed),
    Spin(SpinRate),
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Default)]
pub enum CommandState {
    #[default]
    Disabled,
    Teleop(RobotVel),
}


pub struct Rover<'a> {
    link: &'a ControlLink,
    drive: &'a DriveTrain,
}

impl<'a> Rover<'a> {
    pub fn new(link: &'a ControlLink, drive: &'a DriveTrain) -> Rover<'a> {
        Rover { link, drive }
    }

    pub async fn run(&self) {
        // Periodically check with the link
        let mut interval = tokio::time::interval(Duration::from_millis(20));
        let mut powered = false;
        loop {
            interval.tick().await;

            // Get command state from link
            match self.link.get_command_state().await {
                CommandState::Disabled => {
                    // println!("Got disabled state from link");
                    let _ = self.drive.set_powered(false).await;    // TODO : Convert servo controls to async
                    powered = false;
                }
                CommandState::Teleop(cmd_vel) => {
                    // println!("Got teleop state from link");
                    if !powered {
                        let _ = self.drive.set_powered(true).await;
                        powered = true;
                    }
                    match cmd_vel {
                        RobotVel::Drive(ds) => {
                            let _ = self.drive.set_speed(ds.lin_mps as f64, ds.curvature as f64).await;
                        }
                        RobotVel::Spin(spin) => {
                            let _ = self.drive.set_spin(spin.spin_rps as f64).await;
                        }
                    }
                    
                    // println!("Told Drive to go.");
                }
            }
        }
    }
}