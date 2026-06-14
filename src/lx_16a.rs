use std::{io::{Error, ErrorKind::InvalidData, Read, Write}, sync::Mutex};
use byteorder::{ByteOrder, LittleEndian};

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

    #[allow(unused)] 
    pub fn read_move_time(&self) -> Result<(i16, u16), Error> {
        const SERVO_MOVE_TIME_READ: u8 = 2;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_MOVE_TIME_READ, &params, &mut rx_buf, 4)?;

        let pos = LittleEndian::read_i16(&response[0..2]);
        let time_ms = LittleEndian::read_u16(&response[2..4]);
        Ok((pos, time_ms))
    }
    
    pub fn move_wait(&self, pos: i16, time_ms: u16) -> Result<(), Error> {
        const SERVO_MOVE_TIME_WAIT_WRITE: u8 = 7;

        let mut params = [0; 4];
        LittleEndian::write_i16(&mut params[0..2], pos);
        LittleEndian::write_u16(&mut params[2..4], time_ms);
        self.write(self.id, SERVO_MOVE_TIME_WAIT_WRITE, &params)?;

        Ok(())
    }

    // read_move_wait
    #[allow(unused)] 
    pub fn read_move_wait(&self) -> Result<(i16, u16), Error> {
        const SERVO_MOVE_TIME_WAIT_READ: u8 = 8;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_MOVE_TIME_WAIT_READ, &params, &mut rx_buf, 4)?;

        let pos = LittleEndian::read_i16(&response[0..2]);
        let time_ms = LittleEndian::read_u16(&response[2..4]);
        Ok((pos, time_ms))
    }

    pub fn move_start(&self) -> Result<(), Error> {
        const SERVO_MOVE_START: u8 = 11;

        let params = [];
        self.write(self.id, SERVO_MOVE_START, &params)?;

        Ok(())
    }

    #[allow(unused)] 
    pub fn move_stop(&self) -> Result<(), Error> {
        const SERVO_MOVE_STOP: u8 = 12;

        let params = [];
        self.write(self.id, SERVO_MOVE_STOP, &params)?;

        Ok(())
    }

    #[allow(unused)]
    pub fn set_servo_id(&self, id: u8) -> Result<(), Error> {
        const SERVO_ID_WRITE: u8 = 13;

        let params = [id];
        self.write(self.id, SERVO_ID_WRITE, &params)?;

        Ok(())
    }

    pub fn read_servo_id(&self) -> Result<u8, Error> {
        const SERVO_ID_READ: u8 = 14;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_ID_READ, &params, &mut rx_buf, 1)?;

        Ok(response[0])
    }

    // TODO-DW : set_angle_offset / SERVO_ANGLE_OFFSET_ADJUST, 17
    // TODO-DW : save_angle_offset / SERVO_ANGLE_OFFSET_WRITE, 18
    // TODO-DW : read_angle_offset / SERVO_ANGLE_OFFSET_READ, 19
    // TODO-DW : set_angle_limit / SERVO_ANGLE_LIMIT_WRITE, 20
    // TODO-DW : read_angle_limit / SERVO_ANGLE_LIMIT_READ, 21
    // TODO-DW : set_vin_limit_mv / SERVO_VIN_LIMIT_WRITE, 22
    // TODO-DW : read_vin_limit_mv / SERVO_VIN_LIMIT_READ, 23
    // TODO-DW : set_temp_limit_c / SERVO_TEMP_MAX_LIMIT_WRITE, 24
    // TODO-DW : read_temp_limit_c / SERVO_TEMP_MAX_LIMIT_READ, 25

    pub fn read_temp_c(&self) -> Result<i8, Error> {
        const SERVO_TEMP_READ: u8 = 26;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_TEMP_READ, &params, &mut rx_buf, 1)?;

        Ok(response[0] as i8)
    }

    pub fn read_vin_mv(&self) -> Result<i16, Error> {
        const SERVO_VIN_READ: u8 = 27;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_VIN_READ, &params, &mut rx_buf, 2)?;

        Ok(LittleEndian::read_i16(&response[0..2]))
    }

    pub fn read_pos(&self) -> Result<i16, Error> {
        const SERVO_POS_READ: u8 = 28;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_POS_READ, &params, &mut rx_buf, 2)?;

        Ok(LittleEndian::read_i16(&response[0..2]))
    }

    // TODO-DW : set_mode / SERVO_OR_MOTOR_MODE_WRITE, 29  (Create enum of SERVO, SPEED(speed))
    // TODO-DW : get_mode / SERVO_OR_MOTOR_MODE_READ, 30
    // TODO-DW : set_powered / SERVO_LOAD_OR_UNLOAD_WRITE, 31
    // TODO-DW : read_powered / SERVO_LOAD_OR_UNLOAD_READ, 32
    // TODO-DW : set_led / SERVO_LED_CTRL_WRITE, 33
    // TODO-DW : read_led / SERVO_LED_CTRL_READ, 34
    // TODO-DW : set_led_err / SERVO_LED_ERROR_WRITE, 35
    // TODO-DW : read_led_err / SERVO_LED_ERROR_READ, 36

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