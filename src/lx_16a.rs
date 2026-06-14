use std::{io::Error, sync::Mutex};
use serialport::SerialPort;

// TODO-DW : Look into making T embeddedio::Read and embeddedio::Write instead of SerialPort

// LX-16A Command bytes
const SERVO_ID_READ: u8 = 14;

pub struct SerialBus {
    port: Mutex<Box<dyn SerialPort>>,
}

impl SerialBus {
    pub fn new(port: Box<dyn SerialPort>) -> SerialBus {
        SerialBus { port: Mutex::new(port) }
    }

    // TODO: Create status return value
    // Write data
    pub fn write(&self, out_data: &[u8]) {
        // TODO
    }

    // TODO: Create status return value
    // Write data then read expected response.
    pub fn write_read(&self, out_data: &[u8], rx_data: &mut[u8]) -> Result<usize, Error> {
        // Get access to the bus
        let mut port = self.port.lock().unwrap();

        // write the request
        print!("Write:");
        out_data.iter().for_each(|byte| {print!(" {byte:02x}")});
        println!("");

        port.write(out_data)?;
        port.flush()?;

        // read the response
        port.read_exact(rx_data)?;

        print!("Read:");
        rx_data.iter().for_each(|byte| {print!(" {byte:02x}")});
        println!("");

        Ok(rx_data.len())
    }
}

// Implement interface to one LX-16a servo on a bus.
pub struct Servo<'a> {
    id: u8, 
    bus: &'a SerialBus,
}

impl<'a> Servo<'a> {
    pub fn new(id: u8, bus: &'a SerialBus) -> Servo<'a> {
        Servo { id, bus }
    }

    pub fn get_id(&self) -> u8 {
        self.id
    }

    pub fn set_checksum(msg: &mut [u8]) {
        // Add the payload bytes together, mod 256, and negate.
        let len = msg.len();
        let sum = msg[2..len-1].iter().sum::<u8>() ^ 0xFF;

        // Stuff the result into the last byte of the message.
        msg[len-1] = sum;
    }

    pub fn read_servo_id(&self) -> u8 {
        // Construct the query
        let mut query = [0x55, 0x55, self.id, 3, SERVO_ID_READ, 0x00];
        Self::set_checksum(&mut query);

        let mut response = [0x00; 7];

        self.bus.write_read(&query, &mut response);

        // TODO : Validate response (header, id, command, len, checksum)

        // TODO : Extract ID from response

        // TODO : Return id
        response[5]
    }
}