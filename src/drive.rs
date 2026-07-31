use std::{io::{Error, Read, Write}, sync::Arc, time::Duration};
use lx_16a::{Lx16aBus, Lx16a, Lx16aMode};
use serialport::SerialPort;
use vector2::Vector2;

const SERIAL_PORT: &str = "/dev/ttyUSB0";
const BAUD: u32 = 115200;

// Note: The drive module has adopted an NED reference frame:
//   +X is forward
//   +Y is starboard
//   +Z is down

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

const FRONT_CORNER_WHEEL_X_M: f64 = 0.25;        // X coord of two front corner wheels
const MID_WHEEL_X_M: f64 = 0.0;                  // X coord of two middle wheels.
const BACK_CORNER_WHEEL_X_M: f64 = -0.28;        // X coord of two rear corner wheels
const MID_WHEEL_Y_M: f64 = 0.26;                 // Y coord (+/-) of two middle wheels
const CORNER_WHEEL_Y_M: f64 = 0.225;             // Y coord (+/-) of four corner wheels

const TICK_MS: u16 = 20;
const RAD_TO_DEG: f64 = 180.0/std::f64::consts::PI;
const DEG_TO_RAW: f64 = 1000.0 / 240.0;
const RAD_TO_RAW: f64 = RAD_TO_DEG * DEG_TO_RAW;
const MAX_MPS: f64 = 0.265;                      // measured max free speed with 7.5V regulator.
const MPS_TO_RAW: f64 = 1000.0 / MAX_MPS;        // Speed 1000 corresponds to 1 meter per sec.

const RIGHT_FRONT_OFFSET: u16 = 397;             // Raw angle of right front, when straight.
const RIGHT_BACK_OFFSET: u16 = 509;              // raw angle of right back, when straight.
const LEFT_BACK_OFFSET: u16 = 447;               // raw angle of left back, when straight.
const LEFT_FRONT_OFFSET: u16 = 564;              // raw angle of left front, when straight.

type Bus = Arc<Lx16aBus<Box<dyn SerialPort>>>;
type Servo = Lx16a<Box<dyn SerialPort>>;

struct WheelModule {
    drive_servo: Servo,
    steer: Option<(Servo, u16)>,   // servo and offset
    // location: Vector2,
    radius: f64,
    rot_dir: Vector2,
    max_steer: f64,
    min_steer: f64,
    reverse: bool,
}

impl<'a> WheelModule {
    fn new(drive_servo: Servo, 
               steer: Option<(Servo, u16)>, // Steering servo and offset
               location: Vector2,
               reverse: bool) -> WheelModule {
        let radius = location.magnitude();                               
        let mut rot_dir = Vector2::new(-location.y, location.x);
        rot_dir.normalize();

        // Compute min, max steering angles
        let mut max_steer = rot_dir.y.atan2(rot_dir.x);
        if max_steer < 0.0 {
            max_steer += std::f64::consts::PI;  // TODO: Clarify all these angles, this is feeling like a hack
        }
        let min_steer = max_steer - std::f64::consts::PI;

        let module = WheelModule { drive_servo, steer, radius, rot_dir, max_steer, min_steer, reverse };

        // Set initial mode, position, speed of servos
        module.init_servos();
        
        module
    }

    fn init_servos(&self) {
        // TODO
    }

    fn to_raw_speed(&self, speed_mps: f64) -> i16 {
        let mut raw_speed = match self.reverse {
            false => (speed_mps * MPS_TO_RAW) as i16,
            true => -(speed_mps * MPS_TO_RAW) as i16,
        };
        // println!("Setting speed: {speed_mps}, raw: {raw_speed}");

        if raw_speed > 1000 { raw_speed = 1000; }
        if raw_speed < -1000 { raw_speed = -1000; }

        raw_speed
    }

    fn to_raw_angle(&self, angle_rad: f64) -> u16 {
        let raw = angle_rad * RAD_TO_RAW;

        let mut position = raw as isize;
        if let Some((_, offset)) = self.steer {
            position += offset as isize;
        }

        position as u16
    }

    fn compute_speed_angle(&self, linear_mps: f64, rotation_rps: f64) -> (f64, f64) {
        // Get X, Y components of linear speed
        let lin_x_mps = linear_mps;
        let lin_y_mps = 0.0;

        // Get X, Y components of rotational speed
        let rot_mag_mps = rotation_rps * self.radius;  // meters per sec
        let rot_x_mps = rot_mag_mps * self.rot_dir.x;
        let rot_y_mps = rot_mag_mps * self.rot_dir.y;

        // Combine linear and rotational components
        let x_mps = lin_x_mps + rot_x_mps;
        let y_mps = lin_y_mps + rot_y_mps;

        // Get speed and angle for this wheel module
        let mut speed_mps = (x_mps*x_mps + y_mps*y_mps).sqrt();
        let mut ang_rad = y_mps.atan2(x_mps);

        // Map angles to the valid range
        if ang_rad > self.max_steer {
            ang_rad = ang_rad - std::f64::consts::PI;
            speed_mps = -speed_mps
        }
        if ang_rad < self.min_steer {
            ang_rad = ang_rad + std::f64::consts::PI;
            speed_mps = -speed_mps
        }

        // println!("lin_mps: ({lin_x_mps}, {lin_y_mps})");
        // println!("rot_mps: ({rot_x_mps}, {rot_y_mps})");
        // println!("speed_mps: {speed_mps}");
        // println!("ang_rad: {ang_rad}");

        (speed_mps, ang_rad)
    }

    fn write_servos(&self, speed_mps: f64, angle_rad: f64) -> Result<(), Error> {
        let speed_raw = self.to_raw_speed(speed_mps);
        let angle_raw = self.to_raw_angle(angle_rad);

        self.drive_servo.set_mode(Lx16aMode::Speed(speed_raw))?;
        if let Some((steer_servo, _offset)) = &self.steer {
            steer_servo.move_time(angle_raw, TICK_MS)?;
        }

        Ok(())
    }

    // Convert robot speed and rotation into drive speed and steering angle
    // for this wheel module.
    fn set_speed(&self, linear_mps: f64, rotation_rps: f64) -> Result<(), Error> {
        let (speed_mps, ang_rad) = self.compute_speed_angle(linear_mps, rotation_rps);

        println!("lin: {linear_mps}, rot: {rotation_rps} -> speed: {speed_mps}, angle: {ang_rad}");

        // Write the results to the servo
        self.write_servos(speed_mps, ang_rad)?;

        Ok(())
    }

    fn set_powered(&self, powered: bool) -> Result<(), Error> {
        self.drive_servo.set_powered(powered)?;
        if let Some((steer_servo, _offset)) = &self.steer {
            steer_servo.set_powered(powered)?;
        }

        Ok(())
    }

}

pub struct DriveTrain {
    bus: Bus,
    wheels: [WheelModule; 6], // 0 is front right, numbers increase clockwise
    // linear_speed_mps: f64,
    // rotation_speed_rps: f64,
}

impl DriveTrain {
    pub fn new() -> DriveTrain {

        let port = serialport::new(SERIAL_PORT, BAUD)
            .timeout(Duration::from_millis(10))
            .open()
            .expect("Failed to open port");

        let bus: Bus = Arc::new(Lx16aBus::new(port));

        // Create Lx16a servos and organize them into units.
        // Wheels are ordered clockwise from front right.
        let wheels = [
            WheelModule::new(
                bus.servo(SERVO_ID_RIGHT_FRONT_DRIVE),
                Some((bus.servo(SERVO_ID_RIGHT_FRONT_STEER), RIGHT_FRONT_OFFSET)),
                Vector2::new(FRONT_CORNER_WHEEL_X_M, CORNER_WHEEL_Y_M),
                true),
            WheelModule::new(
                bus.servo(SERVO_ID_RIGHT_CENTER_DRIVE),
                None,
                Vector2::new(MID_WHEEL_X_M, MID_WHEEL_Y_M),
                true),
            WheelModule::new(
                bus.servo(SERVO_ID_RIGHT_REAR_DRIVE),
                Some((bus.servo(SERVO_ID_RIGHT_REAR_STEER), RIGHT_BACK_OFFSET)),
                Vector2::new(BACK_CORNER_WHEEL_X_M, CORNER_WHEEL_Y_M),
                true),
            WheelModule::new(
                bus.servo(SERVO_ID_LEFT_REAR_DRIVE),
                Some((bus.servo(SERVO_ID_LEFT_REAR_STEER), LEFT_BACK_OFFSET)),
                Vector2::new(BACK_CORNER_WHEEL_X_M, -CORNER_WHEEL_Y_M),
                false),
            WheelModule::new(
                bus.servo(SERVO_ID_LEFT_CENTER_DRIVE),
                None,
                Vector2::new(MID_WHEEL_X_M, -MID_WHEEL_Y_M, ),
                false),
            WheelModule::new(
                bus.servo(SERVO_ID_LEFT_FRONT_DRIVE),
                Some((bus.servo(SERVO_ID_LEFT_FRONT_STEER), LEFT_FRONT_OFFSET)),
                Vector2::new(FRONT_CORNER_WHEEL_X_M, -CORNER_WHEEL_Y_M),
                false),
            ];

        DriveTrain { bus, wheels }
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
    pub fn set_speed(&self, linear_mps: f64, rotation_rps: f64) -> Result<(), Error> {
        // self.linear_speed_mps = linear_mps;
        // self.rotation_speed_rps = rotation_rps;

        let mut retval = Ok(());

        self.wheels.iter().for_each(|wheel| {
            match wheel.set_speed(linear_mps, rotation_rps) {
                Err(e) => retval = Err(e),
                _ => ()
            };
        });

        retval
    }

    pub fn set_powered(&self, powered: bool) -> Result<(), Error> {
        let mut retval = Ok(());

        self.wheels.iter().for_each(|wheel| {
            match wheel.set_powered(false) {
                Err(e) => retval = Err(e),
                _ => ()
            };
        });

        retval        
    }

    
    pub async fn run(&self) {
        loop {
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
        // TODO
    }

    // TODO-DW : Implement some drive functionality
}