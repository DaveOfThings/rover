use std::{io::{Read, Write, Error}};

use lx_16a::Lx16aBus;
use crate::drive::Drive;

pub struct Rover<'a, T: Read+Write> {
    bus: &'a Lx16aBus<T>,
    drive: Drive<'a, T>,
}

impl<'a, T: Read+Write> Rover<'a, T> {
    pub fn new(servo_bus: &'a Lx16aBus<T>) -> Rover<'a, T> {
        let drive = Drive::new(&servo_bus);
        Rover { bus: servo_bus, drive: drive }
    }

    pub fn wiggle(&self) {
        println!("They see me roving.");
        let seven = self.bus.servo(7);
        let return_id = seven.read_servo_id().unwrap();
        println!("Seven reports it is {return_id}");
        println!("  temp: {}", seven.read_temp_c().unwrap());
        println!("  voltage: {}", seven.read_vin_mv().unwrap());
    }

    pub fn drive(&mut self, speed_mps: f32) -> Result<(), Error>  {
        self.drive_turn(speed_mps, crate::drive::GO_STRAIGHT)?;

        Ok(())
    }

    pub fn drive_turn(&mut self, speed_mps: f32, turn_radius_m: f32) -> Result<(), Error> {
        self.drive.set_speed(speed_mps, turn_radius_m)?;

        Ok(())
    }
}