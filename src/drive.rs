use std::io::{Read, Write, Error};
use lx_16a::{Lx16aBus, Lx16a, Lx16aMode};
use vector2::Vector2;

// Note: The drive module has adopted an ENU reference frame:
//   +X is to starboard (right side of robot)
//   +Y is forward
//   +Z is up.

pub const GO_STRAIGHT: f32 = 10000.0;

const SERVO_ID_RIGHT_FRONT_STEER:  u8 =  1;
const SERVO_ID_RIGHT_FRONT_DRIVE:  u8 =  2;
const SERVO_ID_RIGHT_CENTER_DRIVE: u8 =  3;
const SERVO_ID_RIGHT_REAR_STEER:   u8 =  4;
const SERVO_ID_RIGHT_REAR_DRIVE:   u8 =  5;
const SERVO_ID_LEFT_REAR_STEER:    u8 =  6;
const SERVO_ID_LEFT_REAR_DRIVE:    u8 =  7;
const SERVO_ID_LEFT_CENTER_DRIVE:  u8 =  8;
const SERVO_ID_LEFT_FRONT_STEER:   u8 =  9;
const SERVO_ID_LEFT_FRONT_DRIVE:   u8 = 10;

const HALF_BASELINE_M: f64 = 0.26;
const CORNER_WHEEL_X_M: f64 = 0.225;
const FRONT_CORNER_WHEEL_Y_M: f64 = 0.25;
const BACK_CORNER_WHEEL_Y_M: f64 = -0.28;

const TICK_MS: u16 = 20;
const RAD_TO_DEG: f32 = 180.0/std::f32::consts::PI;
const DEG_TO_RAW: f32 = 1000.0 / 240.0;
const RAD_TO_RAW: f32 = RAD_TO_DEG * DEG_TO_RAW;
const MAX_MPS: f32 = 0.265;  // measured max free speed.
const MPS_TO_RAW: f32 = 1000.0 / MAX_MPS;  // Speed 1000 corresponds to 1 meter per sec.

const RIGHT_FRONT_OFFSET: u16 = 397;
const RIGHT_BACK_OFFSET: u16 = 509;
const LEFT_BACK_OFFSET: u16 = 447;
const LEFT_FRONT_OFFSET: u16 = 564;

struct WheelModule<'a, T: Read+Write> {
    drive_servo: Lx16a<'a, T>,
    steer_servo: Option<Lx16a<'a, T>>,
    steer_offset: u16,  // TODO : Implement offset
    location: Vector2,
    reverse: bool,
}

impl<'a, T: Read+Write> WheelModule<'a, T> {
    fn new(drive_servo: Lx16a<'a, T>, 
               steer_servo: Option<Lx16a<'a, T>>, 
               steer_offset: u16,
               location: Vector2,
               reverse: bool) -> WheelModule<'a, T> {
        WheelModule { drive_servo, steer_servo, steer_offset, location, reverse }

        // TODO: Set initial mode, position, speed of servos
    }

    fn to_raw_speed(&self, speed_mps: f32) -> i16 {
        let mut raw_speed = match self.reverse {
            false => (speed_mps * MPS_TO_RAW) as i16,
            true => -(speed_mps * MPS_TO_RAW) as i16,
        };
        println!("Setting speed: {speed_mps}, raw: {raw_speed}");

        if raw_speed > 1000 { raw_speed = 1000; }
        if raw_speed < -1000 { raw_speed = -1000; }

        raw_speed
    }

    fn to_raw_angle(&self, angle_rad: f32) -> u16 {
        let degrees = angle_rad * RAD_TO_DEG;
        let raw = degrees * DEG_TO_RAW;
        let position = (self.steer_offset as isize) + (raw as isize);

        println!("radians:{angle_rad}, degrees:{degrees}, plus offset:{} -> position:{position}", self.steer_offset);
        position as u16
    }

    // Convert robot speed and rotation into drive speed and steering angle
    // for this wheel module.
    fn set_speed(&self, speed_mps: f32, turn_radius_m: f32) -> Result<(), Error> {
        // radius from center of turn to outside wheel
        let ref_radius = match turn_radius_m > 0.0 {
            true => turn_radius_m + HALF_BASELINE_M as f32,   
            false => turn_radius_m - HALF_BASELINE_M as f32, 
        };

        let wheel_x = turn_radius_m - self.location.x as f32;
        let wheel_y = self.location.y as f32;
        let wheel_r = (wheel_x*wheel_x + wheel_y*wheel_y).sqrt();    // Distance from wheel to center of turn
        let wheel_speed = speed_mps * wheel_r / ref_radius;
        let wheel_angle_rad = (wheel_y/wheel_r).asin();

        let speed_raw = self.to_raw_speed(wheel_speed);
        let angle_raw = self.to_raw_angle(wheel_angle_rad);

        self.drive_servo.set_mode(Lx16aMode::Speed(speed_raw))?;  // TODO-DW
        if let Some(steer_servo) = &self.steer_servo {
            steer_servo.move_time(angle_raw, TICK_MS)?;    // Move to desired angle in one tick
        }

        Ok(())
    }
}

pub struct Drive<'a, T: Write+Read> {
    // bus: &'a Lx16aBus<T>,
    wheels: [WheelModule<'a, T>; 6], // 0 is front right, numbers increase clockwise
    speed_mps: f32,
    turn_radius_m: f32,
}

impl<'a, T: Read+Write> Drive<'a, T> {
    pub fn new(bus: &'a Lx16aBus<T>) -> Drive<'a, T> {
        // Create Lx16a servos and organize them into units.
        let wheels = [
            WheelModule::new(
                bus.servo(SERVO_ID_RIGHT_FRONT_DRIVE),
                Some(bus.servo(SERVO_ID_RIGHT_FRONT_STEER)),
                RIGHT_FRONT_OFFSET,
                Vector2::new(CORNER_WHEEL_X_M, FRONT_CORNER_WHEEL_Y_M),
                true),
            WheelModule::new(
                bus.servo(SERVO_ID_RIGHT_CENTER_DRIVE),
                None,
                0_u16,
                Vector2::new(HALF_BASELINE_M, 0.0),
                true),
            WheelModule::new(
                bus.servo(SERVO_ID_RIGHT_REAR_DRIVE),
                Some(bus.servo(SERVO_ID_RIGHT_REAR_STEER)),
                RIGHT_BACK_OFFSET,
                Vector2::new(CORNER_WHEEL_X_M, BACK_CORNER_WHEEL_Y_M),
                true),
            WheelModule::new(
                bus.servo(SERVO_ID_LEFT_REAR_DRIVE),
                Some(bus.servo(SERVO_ID_LEFT_REAR_STEER)),
                LEFT_BACK_OFFSET,
                Vector2::new(-CORNER_WHEEL_X_M, BACK_CORNER_WHEEL_Y_M),
                false),
            WheelModule::new(
                bus.servo(SERVO_ID_LEFT_CENTER_DRIVE),
                None,
                0_u16,
                Vector2::new(-HALF_BASELINE_M, 0.0),
                false),
            WheelModule::new(
                bus.servo(SERVO_ID_LEFT_FRONT_DRIVE),
                Some(bus.servo(SERVO_ID_LEFT_FRONT_STEER)),
                LEFT_FRONT_OFFSET,
                Vector2::new(-CORNER_WHEEL_X_M, FRONT_CORNER_WHEEL_Y_M),
                false),
            ];

        Drive { wheels, speed_mps: 0.0, turn_radius_m: GO_STRAIGHT}
    }

    // Set speed and turning radius.
    // speed_mps is in meters per second.  
    //   To stop set speed to 0.0.
    //   Non-zero values will result in the outside center wheel going the specified speed.
    //   Other wheels will go at related speeds, based on turning radius and robot geometry.
    //
    // turn_radius_m is the turning radius in meters.  
    //   The point about which the robot rotates is at (X=0, Y=turn_radius_m)
    //   Positive turns left, Negative right.  Zero pivots about the robots center
    //   To drive straight, set turn_radius_m to GO_STRAIGHT.
    pub fn set_speed(&mut self, speed_mps: f32, turn_radius_m: f32) -> Result<(), Error> {
        self.speed_mps = speed_mps;
        self.turn_radius_m = turn_radius_m;

        let mut retval = Ok(());

        self.wheels.iter().for_each(|wheel| {
            match wheel.set_speed(self.speed_mps, self.turn_radius_m) {
                Err(e) => retval = Err(e),
                _ => ()
            };
        });

        retval
    }

    // TODO-DW : Implement some drive functionality
}