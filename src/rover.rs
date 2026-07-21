use std::{io::{Read, Write, Error}};

use lx_16a::Lx16aBus;
use crate::drive::DriveTrain;
use crate::control_link::ControlLink;
use serde::Serialize;


#[derive(Clone, Copy, Default, Debug, Serialize)]
pub struct RobotVel {
    lin_mps: f32,
    ang_rps: f32,
}

#[derive(Clone, Copy, Debug, Serialize, Default)]
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