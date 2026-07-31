use std::{io::{Error, Read, Write}, time::Duration};

use lx_16a::Lx16aBus;
use crate::drive::DriveTrain;
use crate::control_link::ControlLink;
use serde::{Serialize, Deserialize};


#[derive(Clone, Copy, Default, Debug, Serialize, Deserialize)]
pub struct RobotVel {
    lin_mps: f32,
    ang_rps: f32,
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
        // TODO : Periodically check with the link
        let mut interval = tokio::time::interval(Duration::from_millis(20));
        loop {
            interval.tick().await;

            // Get command state from link
            match self.link.get_command_state().await {
                CommandState::Disabled => {
                    // println!("Got disabled state from link"); // TODO : Create enum to represent enabled/disabled + speed.
                    let _ = self.drive.set_powered(false);    // TODO : Convert servo controls to async
                }
                CommandState::Teleop(cmd_vel) => {
                    // println!("Got teleop state from link");
                    let _ = self.drive.set_powered(true);
                    let _ = self.drive.set_speed(cmd_vel.lin_mps as f64, cmd_vel.ang_rps as f64);
                    // println!("Told Drive to go.");
                }
            }
        }
    }

    /*
    pub fn wiggle(&self) {
        println!("They see me roving.");
        let seven = self.bus.servo(7);
        let return_id = seven.read_servo_id().unwrap();
        println!("Seven reports it is {return_id}");
        println!("  temp: {}", seven.read_temp_c().unwrap());
        println!("  voltage: {}", 0.001 * seven.read_vin_mv().unwrap() as f64);
    }

    pub fn drive(&mut self, speed_mps: f64) -> Result<(), Error>  {
        self.drive_turn(speed_mps, 0.0)?;

        Ok(())
    }
    */

    pub fn cmd_vel(&mut self, linear_speed_mps: f64, rotation_speed_rps: f64) -> Result<(), Error> {
        self.drive.set_speed(linear_speed_mps, rotation_speed_rps)?;

        Ok(())
    }
}