use std::{io::{Error, ErrorKind::InvalidData, Read, Write}, sync::Mutex};
use byteorder::{ByteOrder, LittleEndian};

// TODO-DW : Look into making T embeddedio::Read and embeddedio::Write instead of SerialPort

// TODO-DW : Add support for direction control?
pub struct Lx16aBus<T: Read+Write> {
    port: Mutex<T>,
}

impl<'a, T: Read+Write> Lx16aBus<T> {
    // Create an Lx16a bus from anything that implements Read+Write
    pub fn new(port: T) -> Lx16aBus<T> {
        Lx16aBus { port: Mutex::new(port) }
    }

    // Get the "broadcast" servo
    pub fn broadcast(&'a self) -> Servo<'a, T> {
        const BROADCAST_ID: u8 = 254;

        Servo::new(BROADCAST_ID, self)
    }

    // Get an individual servo
    pub fn servo(&'a self, id: u8) -> Servo<'a, T> {
        Servo::new(id, self)
    }

    // TODO: Create status return value
    // Write data
    pub fn write(&self, out_data: &[u8])  -> Result<(), Error> {
        // Get exclusive access to the bus
        let mut port = self.port.lock().unwrap();

        // write the request
        port.write(out_data)?;
        port.flush()?;

        Ok(())
    }

    // Write data then read expected response.
    pub fn write_read(&self, out_data: &[u8], rx_data: &mut[u8]) -> Result<usize, Error> {
        // Get exclusive access to the bus
        let mut port = self.port.lock().unwrap();

        // write the request
        port.write(out_data)?;
        port.flush()?;

        // read the response
        port.read_exact(rx_data)?;

        Ok(rx_data.len())
    }
}

// Implement interface to one LX-16a servo on a bus.
pub struct Servo<'a, T: Read+Write> {
    id: u8, 
    bus: &'a Lx16aBus<T>,
}

impl<'a, T: Read+Write> Servo<'a, T> {
    // Private new method, used by Lx16aBus.
    // To create a servo, first create an Lx16aBus, then use servo() factory method.
    fn new(id: u8, bus: &'a Lx16aBus<T>) -> Servo<'a, T> {
        Servo { id, bus }
    }

    // Get the ID associated with this servo
    pub fn get_id(&self) -> u8 {
        self.id
    }

    // --- Public operations corresponding to servo commands

    // TODO : Move LX16a stuff to its own library crate
    #[allow(unused)] 
    pub fn move_time(&self, pos: i16, time_ms: u16) -> Result<(), Error> {
        const SERVO_MOVE_TIME_WRITE: u8 = 1;

        let mut params = [0; 4];
        LittleEndian::write_i16(&mut params[0..2], pos);
        LittleEndian::write_u16(&mut params[2..4], time_ms);
        self.write(self.id, SERVO_MOVE_TIME_WRITE, &params)?;

        Ok(())
    }
    
    pub fn move_wait(&self, pos: i16, time_ms: u16) -> Result<(), Error> {
        const SERVO_MOVE_TIME_WAIT_WRITE: u8 = 7;

        let mut params = [0; 4];
        LittleEndian::write_i16(&mut params[0..2], pos);
        LittleEndian::write_u16(&mut params[2..4], time_ms);
        self.write(self.id, SERVO_MOVE_TIME_WAIT_WRITE, &params)?;

        Ok(())
    }

    pub fn move_start(&self) -> Result<(), Error> {
        const SERVO_MOVE_START: u8 = 11;

        let params = [];
        self.write(self.id, SERVO_MOVE_START, &params)?;

        Ok(())
    }

    pub fn read_servo_id(&self) -> Result<u8, Error> {
        const SERVO_ID_READ: u8 = 14;

        let mut rx_buf = [0; 32];
        let response = self.read(self.id, SERVO_ID_READ, &[], &mut rx_buf, 1)?;

        Ok(response[0])
    }

    pub fn read_temp_c(&self) -> Result<i8, Error> {
        const SERVO_TEMP_READ: u8 = 26;

        let mut rx_buf = [0; 32];
        let response = self.read(self.id, SERVO_TEMP_READ, &[], &mut rx_buf, 1)?;

        Ok(response[0] as i8)
    }

    pub fn read_vin_mv(&self) -> Result<i16, Error> {
        const SERVO_VIN_READ: u8 = 27;

        let mut rx_buf = [0; 32];
        let response = self.read(self.id, SERVO_VIN_READ, &[], &mut rx_buf, 2)?;

        Ok(LittleEndian::read_i16(&response[0..2]))
    }

    pub fn read_pos(&self) -> Result<i16, Error> {
        const SERVO_POS_READ: u8 = 28;

        let mut rx_buf = [0; 32];
        let response = self.read(self.id, SERVO_POS_READ, &[], &mut rx_buf, 2)?;

        Ok(LittleEndian::read_i16(&response[0..2]))
    }

    // --- Utility methods --------------------------------------------
    
    fn checksum(msg: &[u8]) -> u8 {
        // Add the payload bytes together, mod 256, and negate.
        let sum = msg.iter()
            .fold(0_u8, |acc, &x| { 
                acc.wrapping_add(x)
            }) ^ 0xFF;

        // Stuff the result into the last byte of the message.
        sum
    }

    fn form_packet(tx_data: &mut[u8], id: u8, cmd: u8, params: &[u8]) -> usize {
        let mut len;
        let len_field = 3_u8 + params.len() as u8;

        // Format contents of packet in tx_data
        tx_data[0] = 0x55;
        tx_data[1] = 0x55;
        tx_data[2] = id;
        tx_data[3] = len_field;
        tx_data[4] = cmd;
        len = 5;
        params.iter()
            .for_each(|b| { 
                tx_data[len] = *b; 
                len += 1; 
            });
        tx_data[len] = Self::checksum(&tx_data[2..len]);
        len += 1;

        len
    }

    fn write(&self, id: u8, cmd: u8, params: &[u8]) -> Result<(), Error> {
        let mut tx_data: [u8; 32] = [0; 32];
        let tx_data_len = Self::form_packet(&mut tx_data, id, cmd, params);

        self.bus.write(&tx_data[0..tx_data_len])?;

        Ok(())
    }

    fn read(&self, id: u8, cmd: u8, params: &[u8], rx_buf: &'a mut[u8], params_len: usize) -> Result<&'a [u8], Error> {
        let mut tx_data: [u8; 32] = [0; 32];
        let tx_data_len = Self::form_packet(&mut tx_data, id, cmd, params);
        let resp_len = params_len + 6;

        self.bus.write_read(&tx_data[0..tx_data_len], &mut rx_buf[0..resp_len])?;

        // Validate response: header, id, length, cmd, checksum
        if Self::checksum(&rx_buf[2..resp_len]) != 0_u8 {
            Err(Error::new(InvalidData, "Checksum error"))
        }
        else if (rx_buf[0] != 0x55) ||
           (rx_buf[1] != 0x55) ||
           (rx_buf[2] != id) ||
           (rx_buf[3] != 3_u8 + params_len as u8) ||
           (rx_buf[4] != cmd) {
            Err(Error::new(InvalidData, "Bad response"))
        }
        else {
            Ok(&rx_buf[5..5+params_len])
        }
    }
}