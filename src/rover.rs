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

    // TODO: Fix problem of calling blocking read/write from async domain.

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
                    powered = false;
                    if let Err(_e) = self.drive.set_powered(powered) {
                        // Bus error, TODO: log this.
                    }
                }
                CommandState::Teleop(cmd_vel) => {
                    // println!("Got teleop state from link");
                    if !powered {
                        // Power up, if we were previously powered down
                        powered = true;
                        if let Err(_e) = self.drive.set_powered(powered) {
                            // Bus error, TODO: log this.
                        }
                    }
                    match cmd_vel {
                        RobotVel::Drive(ds) => {
                            if let Err(_e) = self.drive.set_speed(ds.lin_mps as f64, ds.curvature as f64) {
                                // Bus error, TODO: log this
                            }
                        }
                        RobotVel::Spin(spin) => {
                            if let Err(_e) = self.drive.set_spin(spin.spin_rps as f64) {
                                // Bus error, TODO: log this
                            }
                        }
                    }
                }
            }
        }
    }
}