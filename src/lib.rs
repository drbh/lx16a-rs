use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio_serial::SerialPort;
use tokio_serial::SerialPortBuilderExt;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

#[derive(Debug)]
pub enum ServoError {
    Timeout(String, Option<u8>),
    Checksum(String, Option<u8>),
    Argument(String, Option<u8>),
    Logical(String, Option<u8>),
}

impl std::fmt::Display for ServoError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            ServoError::Timeout(msg, id) => write!(f, "Timeout Error: {} ID: {:?}", msg, id),
            ServoError::Checksum(msg, id) => write!(f, "Checksum Error: {} ID: {:?}", msg, id),
            ServoError::Argument(msg, id) => write!(f, "Argument Error: {} ID: {:?}", msg, id),
            ServoError::Logical(msg, id) => write!(f, "Logical Error: {} ID: {:?}", msg, id),
        }
    }
}

impl std::error::Error for ServoError {}

pub struct BSpline {
    knots: Vec<f32>,
    control_points: Vec<(f32, f32)>,
    degree: i32,
    samples: Vec<(f32, f32)>,
}

impl BSpline {
    fn new(
        knots: Vec<f32>,
        control_points: Vec<(f32, f32)>,
        degree: i32,
        num_samples: i32,
    ) -> Self {
        let mut bspline = BSpline {
            knots,
            control_points,
            degree,
            samples: vec![],
        };

        for i in 0..=num_samples {
            let u = i as f32 / num_samples as f32;
            bspline.samples.push(bspline.sample(u));
        }

        bspline
    }

    fn weight(&self, i: usize, u: f32) -> f32 {
        if self.degree == 0 {
            if self.knots[i] <= u && u < self.knots[i + 1] {
                return 1.0;
            } else {
                return 0.0;
            }
        }

        let term1 = if self.knots[i] == self.knots[i + self.degree as usize] {
            0.0
        } else {
            (u - self.knots[i]) / (self.knots[i + self.degree as usize] - self.knots[i])
                * self.weight_recursive(i, u, self.degree - 1)
        };

        let term2 = if self.knots[i + 1] == self.knots[i + self.degree as usize + 1] {
            0.0
        } else {
            (self.knots[i + self.degree as usize + 1] - u)
                / (self.knots[i + self.degree as usize + 1] - self.knots[i + 1])
                * self.weight_recursive(i + 1, u, self.degree - 1)
        };

        term1 + term2
    }

    // Fixed recursive weight calculation
    fn weight_recursive(&self, i: usize, u: f32, degree: i32) -> f32 {
        if degree == 0 {
            if self.knots[i] <= u && u < self.knots[i + 1] {
                return 1.0;
            } else {
                return 0.0;
            }
        }

        let term1 = if self.knots[i] == self.knots[i + degree as usize] {
            0.0
        } else {
            (u - self.knots[i]) / (self.knots[i + degree as usize] - self.knots[i])
                * self.weight_recursive(i, u, degree - 1)
        };

        let term2 = if self.knots[i + 1] == self.knots[i + degree as usize + 1] {
            0.0
        } else {
            (self.knots[i + degree as usize + 1] - u)
                / (self.knots[i + degree as usize + 1] - self.knots[i + 1])
                * self.weight_recursive(i + 1, u, degree - 1)
        };

        term1 + term2
    }

    fn sample(&self, u: f32) -> (f32, f32) {
        let mut sx = 0.0;
        let mut sy = 0.0;

        for (i, control_point) in self.control_points.iter().enumerate() {
            let w = self.weight(i, u);
            sx += w * control_point.0;
            sy += w * control_point.1;
        }

        (sx, sy)
    }

    fn sample_x(&self, x: f32) -> f32 {
        for i in 0..self.samples.len() - 1 {
            if self.samples[i].0 >= x {
                return self.samples[i].1;
            }
        }

        // Return the last y value if x is beyond the last sample
        self.samples.last().unwrap().1
    }
}

pub struct LX16A {
    id: u8,
    commanded_angle: i32,
    waiting_angle: i32,
    waiting_for_move: bool,
    angle_offset: i32,
    angle_limits: (i32, i32),
    vin_limits: (i32, i32),
    temp_limit: i32,
    motor_mode: bool,
    motor_speed: Option<i32>,
    torque_enabled: bool,
    led_powered: bool,
    led_error_triggers: (bool, bool, bool),
    bspline: Option<BSpline>,
    controller: Arc<Mutex<Box<dyn SerialPort>>>,
}

impl LX16A {
    // Static controller initialization
    pub fn initialize(port: &str, timeout: Duration) -> Result<Arc<Mutex<Box<dyn SerialPort>>>> {
        // Use the newer API style for tokio_serial
        let serial_port = tokio_serial::new(port, 115200)
            .timeout(timeout)
            .open_native_async()?;

        Ok(Arc::new(Mutex::new(Box::new(serial_port))))
    }

    // Create a new LX16A servo instance
    pub fn new(
        id: u8,
        controller: Arc<Mutex<Box<dyn SerialPort>>>,
        disable_torque: bool,
    ) -> Result<Self> {
        if id > 253 {
            return Err(Box::new(ServoError::Argument(
                format!("Servo ID must be between 0 and 253 inclusive"),
                Some(id),
            )));
        }

        let mut servo = LX16A {
            id,
            commanded_angle: 0,
            waiting_angle: 0,
            waiting_for_move: false,
            angle_offset: 0,
            angle_limits: (0, 1000),   // Placeholder values
            vin_limits: (4500, 12000), // Default values
            temp_limit: 85,            // Default value
            motor_mode: false,
            motor_speed: None,
            torque_enabled: true,
            led_powered: true,
            led_error_triggers: (true, true, true), // Default values
            bspline: None,
            controller,
        };

        // Get physical angle and set as commanded angle
        let physical_angle = servo.get_physical_angle()?;
        servo.commanded_angle = LX16A::to_servo_range(physical_angle);
        servo.waiting_angle = servo.commanded_angle;

        // Get angle offset from hardware
        let angle_offset = servo.get_angle_offset(true)?;
        servo.angle_offset = LX16A::to_servo_range(angle_offset);

        // Get angle limits from hardware
        let angle_limits = servo.get_angle_limits(true)?;
        servo.angle_limits = (
            LX16A::to_servo_range(angle_limits.0),
            LX16A::to_servo_range(angle_limits.1),
        );

        // Get other settings from hardware
        servo.vin_limits = servo.get_vin_limits(true)?;
        servo.temp_limit = servo.get_temp_limit(true)?;
        servo.motor_mode = servo.is_motor_mode(true)?;

        if servo.motor_mode {
            servo.motor_speed = Some(servo.get_motor_speed(true)?);
        }

        servo.torque_enabled = servo.is_torque_enabled(true)?;
        servo.led_powered = servo.is_led_power_on(true)?;
        servo.led_error_triggers = servo.get_led_error_triggers(true)?;

        if disable_torque {
            servo.disable_torque()?;
        } else {
            servo.enable_torque()?;
        }

        Ok(servo)
    }

    // Utility Functions
    fn checksum(packet: &[u8]) -> u8 {
        (!packet[2..].iter().fold(0u8, |acc, &x| acc.wrapping_add(x))) & 0xFF
    }

    fn to_bytes(n: i32) -> (u8, u8) {
        ((n % 256) as u8, (n / 256) as u8)
    }

    fn check_packet(packet: &[u8], servo_id: u8) -> Result<()> {
        if packet.iter().all(|&x| x == 0) {
            return Err(Box::new(ServoError::Timeout(
                format!("Servo {}: not responding", servo_id),
                Some(servo_id),
            )));
        }

        let calculated_checksum = LX16A::checksum(&packet[..packet.len() - 1]);
        if calculated_checksum != packet[packet.len() - 1] {
            return Err(Box::new(ServoError::Checksum(
                format!("Servo {}: bad checksum", servo_id),
                Some(servo_id),
            )));
        }

        Ok(())
    }

    fn send_packet(&self, packet: &[u8]) -> Result<()> {
        let mut full_packet = vec![0x55, 0x55];
        full_packet.extend_from_slice(packet);
        let checksum = LX16A::checksum(&full_packet);
        full_packet.push(checksum);

        println!("Sending packet: {:?}", full_packet);

        let mut controller = self.controller.lock().unwrap();
        controller.write_all(&full_packet)?;
        controller.flush()?;

        Ok(())
    }

    fn read_packet(&self, num_bytes: usize) -> Result<Vec<u8>> {
        let mut controller = self.controller.lock().unwrap();
        let mut buffer = vec![0u8; num_bytes + 6];

        // Add a small delay to allow the device to respond
        std::thread::sleep(Duration::from_millis(50));

        // Read with timeout handling
        let mut bytes_read = 0;
        let start_time = std::time::Instant::now();
        while bytes_read < buffer.len() {
            match controller.read(&mut buffer[bytes_read..]) {
                Ok(n) if n > 0 => {
                    bytes_read += n;
                }
                Ok(_) => {
                    // No data, wait a bit and try again
                    std::thread::sleep(Duration::from_millis(10));
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // WouldBlock error, wait a bit and try again
                    std::thread::sleep(Duration::from_millis(10));
                }
                Err(e) => return Err(Box::new(e)),
            }

            // Check for timeout
            if start_time.elapsed() > Duration::from_millis(500) {
                return Err(Box::new(ServoError::Timeout(
                    format!("Servo {}: read timeout", self.id),
                    Some(self.id),
                )));
            }
        }

        LX16A::check_packet(&buffer, self.id)?;
        println!("Received packet: {:?}", buffer);

        // Return only the data portion of the packet (excluding headers, length, ID, and checksum)
        Ok(buffer[5..buffer.len() - 1].to_vec())
    }

    fn to_servo_range(angle: f32) -> i32 {
        (angle * 25.0 / 6.0).round() as i32
    }

    fn from_servo_range(angle: i32) -> f32 {
        angle as f32 * 6.0 / 25.0
    }

    fn check_within_limits<T: PartialOrd + std::fmt::Display>(
        value: T,
        lower_limit: T,
        upper_limit: T,
        variable_name: &str,
        servo_id: u8,
    ) -> Result<()> {
        if value < lower_limit || value > upper_limit {
            return Err(Box::new(ServoError::Argument(
                format!(
                    "Servo {}: {} must be between {} and {} (received {})",
                    servo_id, variable_name, lower_limit, upper_limit, value
                ),
                Some(servo_id),
            )));
        }

        Ok(())
    }

    // Write Commands
    pub fn move_servo(&mut self, angle: f32, time: i32, relative: bool, wait: bool) -> Result<()> {
        if !self.torque_enabled {
            return Err(Box::new(ServoError::Logical(
                format!("Servo {}: torque must be enabled to move", self.id),
                Some(self.id),
            )));
        }

        if self.motor_mode {
            return Err(Box::new(ServoError::Logical(
                format!(
                    "Servo {}: motor mode must be disabled to control movement",
                    self.id
                ),
                Some(self.id),
            )));
        }

        LX16A::check_within_limits(angle, 0.0, 240.0, "angle", self.id)?;
        LX16A::check_within_limits(
            angle,
            LX16A::from_servo_range(self.angle_limits.0),
            LX16A::from_servo_range(self.angle_limits.1),
            "angle",
            self.id,
        )?;

        let mut angle_servo = LX16A::to_servo_range(angle);

        if relative {
            angle_servo += self.commanded_angle;
        }

        let (angle_low, angle_high) = LX16A::to_bytes(angle_servo);
        let (time_low, time_high) = LX16A::to_bytes(time);

        let packet = if wait {
            vec![self.id, 7, 7, angle_low, angle_high, time_low, time_high]
        } else {
            vec![self.id, 7, 1, angle_low, angle_high, time_low, time_high]
        };

        self.send_packet(&packet)?;

        if wait {
            self.waiting_angle = angle_servo;
            self.waiting_for_move = true;
        } else {
            self.commanded_angle = angle_servo;
        }

        Ok(())
    }

    pub fn move_bspline(&mut self, x: f32, time: i32, wait: bool) -> Result<()> {
        if let Some(bspline) = &self.bspline {
            let y = bspline.sample_x(x);
            self.move_servo(y, time, false, wait)
        } else {
            Err(Box::new(ServoError::Logical(
                format!("Servo {}: no B-Spline defined", self.id),
                Some(self.id),
            )))
        }
    }

    pub fn move_start(&mut self) -> Result<()> {
        if !self.waiting_for_move {
            return Err(Box::new(ServoError::Logical(
                format!("Servo {}: not waiting for move", self.id),
                Some(self.id),
            )));
        }

        if !self.torque_enabled {
            return Err(Box::new(ServoError::Logical(
                format!("Servo {}: torque must be enabled to move", self.id),
                Some(self.id),
            )));
        }

        if self.motor_mode {
            return Err(Box::new(ServoError::Logical(
                format!(
                    "Servo {}: motor mode must be disabled to control movement",
                    self.id
                ),
                Some(self.id),
            )));
        }

        let packet = vec![self.id, 3, 11];
        self.send_packet(&packet)?;

        self.commanded_angle = self.waiting_angle;
        self.waiting_for_move = false;

        Ok(())
    }

    pub fn move_stop(&mut self) -> Result<()> {
        if self.motor_mode {
            return Err(Box::new(ServoError::Logical(
                format!(
                    "Servo {}: motor mode must be disabled to control movement",
                    self.id
                ),
                Some(self.id),
            )));
        }

        let packet = vec![self.id, 3, 12];
        self.send_packet(&packet)?;

        let angle = self.get_physical_angle()?;
        self.commanded_angle = LX16A::to_servo_range(angle);

        Ok(())
    }

    pub fn set_id(&mut self, id: u8) -> Result<()> {
        LX16A::check_within_limits(id, 0, 253, "servo ID", self.id)?;

        let packet = vec![self.id, 4, 13, id];
        self.send_packet(&packet)?;
        self.id = id;

        Ok(())
    }

    pub fn set_angle_offset(&mut self, offset: f32, permanent: bool) -> Result<()> {
        LX16A::check_within_limits(offset, -30.0, 30.0, "angle offset", self.id)?;

        let mut offset_servo = LX16A::to_servo_range(offset);
        if offset_servo < 0 {
            offset_servo = 256 + offset_servo;
        }

        let packet = vec![self.id, 4, 17, offset_servo as u8];
        self.send_packet(&packet)?;
        self.angle_offset = offset_servo;

        if permanent {
            let packet = vec![self.id, 3, 18];
            self.send_packet(&packet)?;
        }

        Ok(())
    }

    pub fn set_angle_limits(&mut self, lower_limit: f32, upper_limit: f32) -> Result<()> {
        LX16A::check_within_limits(lower_limit, 0.0, 240.0, "lower limit", self.id)?;
        LX16A::check_within_limits(upper_limit, 0.0, 240.0, "upper limit", self.id)?;

        if upper_limit < lower_limit {
            return Err(Box::new(ServoError::Argument(
                format!(
                    "Servo {}: lower limit (received {}) must be less than upper limit (received {})",
                    self.id, lower_limit, upper_limit
                ),
                Some(self.id),
            )));
        }

        let lower_limit_servo = LX16A::to_servo_range(lower_limit);
        let upper_limit_servo = LX16A::to_servo_range(upper_limit);

        let (ll_low, ll_high) = LX16A::to_bytes(lower_limit_servo);
        let (ul_low, ul_high) = LX16A::to_bytes(upper_limit_servo);

        let packet = vec![self.id, 7, 20, ll_low, ll_high, ul_low, ul_high];
        self.send_packet(&packet)?;

        self.angle_limits = (lower_limit_servo, upper_limit_servo);

        Ok(())
    }

    pub fn set_vin_limits(&mut self, lower_limit: i32, upper_limit: i32) -> Result<()> {
        LX16A::check_within_limits(lower_limit, 4500, 12000, "lower limit", self.id)?;
        LX16A::check_within_limits(upper_limit, 4500, 12000, "upper limit", self.id)?;

        if upper_limit < lower_limit {
            return Err(Box::new(ServoError::Argument(
                format!(
                    "Servo {}: lower limit (received {}) must be less than upper limit (received {})",
                    self.id, lower_limit, upper_limit
                ),
                Some(self.id),
            )));
        }

        let (ll_low, ll_high) = LX16A::to_bytes(lower_limit);
        let (ul_low, ul_high) = LX16A::to_bytes(upper_limit);

        let packet = vec![self.id, 7, 22, ll_low, ll_high, ul_low, ul_high];
        self.send_packet(&packet)?;

        self.vin_limits = (lower_limit, upper_limit);

        Ok(())
    }

    pub fn set_temp_limit(&mut self, upper_limit: i32) -> Result<()> {
        LX16A::check_within_limits(upper_limit, 50, 100, "temperature limit", self.id)?;

        let packet = vec![self.id, 4, 24, upper_limit as u8];
        self.send_packet(&packet)?;

        self.temp_limit = upper_limit;

        Ok(())
    }

    pub fn motor_mode(&mut self, speed: i32) -> Result<()> {
        if !self.torque_enabled {
            return Err(Box::new(ServoError::Logical(
                format!(
                    "Servo {}: torque must be enabled to control movement",
                    self.id
                ),
                Some(self.id),
            )));
        }

        LX16A::check_within_limits(speed, -1000, 1000, "motor speed", self.id)?;

        let adjusted_speed = if speed < 0 { speed + 65536 } else { speed };
        let (speed_low, speed_high) = LX16A::to_bytes(adjusted_speed);

        let packet = vec![self.id, 7, 29, 1, 0, speed_low, speed_high];
        self.send_packet(&packet)?;

        self.motor_mode = true;
        self.motor_speed = Some(speed);

        Ok(())
    }

    pub fn servo_mode(&mut self) -> Result<()> {
        let packet = vec![self.id, 7, 29, 0, 0, 0, 0];
        self.send_packet(&packet)?;

        self.motor_mode = false;
        self.motor_speed = None;

        Ok(())
    }

    pub fn enable_torque(&mut self) -> Result<()> {
        let packet = vec![self.id, 4, 31, 1];
        self.send_packet(&packet)?;

        self.torque_enabled = true;

        Ok(())
    }

    pub fn disable_torque(&mut self) -> Result<()> {
        let packet = vec![self.id, 4, 31, 0];
        self.send_packet(&packet)?;

        self.torque_enabled = false;

        Ok(())
    }

    pub fn led_power_off(&mut self) -> Result<()> {
        let packet = vec![self.id, 4, 33, 1];
        self.send_packet(&packet)?;

        self.led_powered = false;

        Ok(())
    }

    pub fn led_power_on(&mut self) -> Result<()> {
        let packet = vec![self.id, 4, 33, 0];
        self.send_packet(&packet)?;

        self.led_powered = true;

        Ok(())
    }

    pub fn set_led_error_triggers(
        &mut self,
        over_temperature: bool,
        over_voltage: bool,
        rotor_locked: bool,
    ) -> Result<()> {
        let combined =
            (rotor_locked as u8) * 4 + (over_voltage as u8) * 2 + (over_temperature as u8);

        let packet = vec![self.id, 4, 35, combined];
        self.send_packet(&packet)?;

        self.led_error_triggers = (over_temperature, over_voltage, rotor_locked);

        Ok(())
    }

    pub fn set_bspline(
        &mut self,
        knots: Vec<f32>,
        control_points: Vec<(f32, f32)>,
        degree: i32,
        num_samples: i32,
    ) -> Result<()> {
        if knots.len() != control_points.len() - degree as usize + 1 {
            return Err(Box::new(ServoError::Argument(
                format!(
                    "Servo {}: len(knots) != len(control_points) - degree + 1",
                    self.id
                ),
                Some(self.id),
            )));
        }

        self.bspline = Some(BSpline::new(knots, control_points, degree, num_samples));

        Ok(())
    }

    // Read Commands
    pub fn get_last_instant_move_hw(&self) -> Result<(f32, i32)> {
        let packet = vec![self.id, 3, 2];
        self.send_packet(&packet)?;

        let received = self.read_packet(4)?;
        let angle = LX16A::from_servo_range((received[0] as i32) + (received[1] as i32) * 256);
        let time = (received[2] as i32) + (received[3] as i32) * 256;

        Ok((angle, time))
    }

    pub fn get_last_delayed_move_hw(&self) -> Result<(f32, i32)> {
        let packet = vec![self.id, 3, 8];
        self.send_packet(&packet)?;

        let received = self.read_packet(4)?;
        let angle = LX16A::from_servo_range((received[0] as i32) + (received[1] as i32) * 256);
        let time = (received[2] as i32) + (received[3] as i32) * 256;

        Ok((angle, time))
    }

    pub fn get_id(&self, poll_hardware: bool) -> Result<u8> {
        if !poll_hardware {
            return Ok(self.id);
        }

        let packet = vec![self.id, 3, 14];
        self.send_packet(&packet)?;

        let received = self.read_packet(1)?;

        Ok(received[0])
    }

    pub fn get_angle_offset(&self, poll_hardware: bool) -> Result<f32> {
        if !poll_hardware {
            return Ok(LX16A::from_servo_range(self.angle_offset));
        }

        let packet = vec![self.id, 3, 19];
        self.send_packet(&packet)?;

        let received = self.read_packet(1)?;
        let offset = if received[0] > 125 {
            LX16A::from_servo_range((received[0] as i32) - 256)
        } else {
            LX16A::from_servo_range(received[0] as i32)
        };

        Ok(offset)
    }

    pub fn get_angle_limits(&self, poll_hardware: bool) -> Result<(f32, f32)> {
        if !poll_hardware {
            return Ok((
                LX16A::from_servo_range(self.angle_limits.0),
                LX16A::from_servo_range(self.angle_limits.1),
            ));
        }

        let packet = vec![self.id, 3, 21];
        self.send_packet(&packet)?;

        let received = self.read_packet(4)?;
        let lower_limit =
            LX16A::from_servo_range((received[0] as i32) + (received[1] as i32) * 256);
        let upper_limit =
            LX16A::from_servo_range((received[2] as i32) + (received[3] as i32) * 256);

        Ok((lower_limit, upper_limit))
    }

    pub fn get_vin_limits(&self, poll_hardware: bool) -> Result<(i32, i32)> {
        if !poll_hardware {
            return Ok(self.vin_limits);
        }

        let packet = vec![self.id, 3, 23];
        self.send_packet(&packet)?;

        let received = self.read_packet(4)?;
        let lower_limit = (received[0] as i32) + (received[1] as i32) * 256;
        let upper_limit = (received[2] as i32) + (received[3] as i32) * 256;

        Ok((lower_limit, upper_limit))
    }

    pub fn get_temp_limit(&self, poll_hardware: bool) -> Result<i32> {
        if !poll_hardware {
            return Ok(self.temp_limit);
        }

        let packet = vec![self.id, 3, 25];
        self.send_packet(&packet)?;

        let received = self.read_packet(1)?;

        Ok(received[0] as i32)
    }

    pub fn get_temp(&self) -> Result<i32> {
        let packet = vec![self.id, 3, 26];
        self.send_packet(&packet)?;

        let received = self.read_packet(1)?;

        Ok(received[0] as i32)
    }

    pub fn get_vin(&self) -> Result<i32> {
        let packet = vec![self.id, 3, 27];
        self.send_packet(&packet)?;

        let received = self.read_packet(2)?;
        let voltage = (received[0] as i32) + (received[1] as i32) * 256;

        Ok(voltage)
    }

    pub fn get_physical_angle(&self) -> Result<f32> {
        let packet = vec![self.id, 3, 28];
        self.send_packet(&packet)?;

        let received = self.read_packet(2)?;
        let angle_servo = (received[0] as i32) + (received[1] as i32) * 256;

        Ok(LX16A::from_servo_range(angle_servo))
    }

    pub fn get_commanded_angle(&self) -> Result<f32> {
        Ok(LX16A::from_servo_range(self.commanded_angle))
    }

    pub fn is_motor_mode(&self, poll_hardware: bool) -> Result<bool> {
        if !poll_hardware {
            return Ok(self.motor_mode);
        }

        let packet = vec![self.id, 3, 30];
        self.send_packet(&packet)?;

        let received = self.read_packet(4)?;

        Ok(received[0] == 1)
    }

    pub fn get_motor_speed(&self, poll_hardware: bool) -> Result<i32> {
        if !poll_hardware {
            if let Some(speed) = self.motor_speed {
                return Ok(speed);
            } else {
                return Err(Box::new(ServoError::Logical(
                    format!("Servo {}: not in motor mode", self.id),
                    Some(self.id),
                )));
            }
        }

        let packet = vec![self.id, 3, 30];
        self.send_packet(&packet)?;

        let received = self.read_packet(4)?;

        if received[0] != 1 {
            return Err(Box::new(ServoError::Logical(
                format!("Servo {}: not in motor mode", self.id),
                Some(self.id),
            )));
        }

        let mut speed = (received[2] as i32) + (received[3] as i32) * 256;
        if speed > 32767 {
            speed -= 65536;
        }

        Ok(speed)
    }

    pub fn is_torque_enabled(&self, poll_hardware: bool) -> Result<bool> {
        if !poll_hardware {
            return Ok(self.torque_enabled);
        }

        let packet = vec![self.id, 3, 32];
        self.send_packet(&packet)?;

        let received = self.read_packet(1)?;

        Ok(received[0] == 1)
    }

    pub fn is_led_power_on(&self, poll_hardware: bool) -> Result<bool> {
        if !poll_hardware {
            return Ok(self.led_powered);
        }

        let packet = vec![self.id, 3, 34];
        self.send_packet(&packet)?;

        let received = self.read_packet(1)?;

        Ok(received[0] == 0)
    }

    pub fn get_led_error_triggers(&self, poll_hardware: bool) -> Result<(bool, bool, bool)> {
        if !poll_hardware {
            return Ok(self.led_error_triggers);
        }

        let packet = vec![self.id, 3, 36];
        self.send_packet(&packet)?;

        let received = self.read_packet(1)?;
        let combined = received[0];

        let over_temperature = (combined & 1) != 0;
        let over_voltage = (combined & 2) != 0;
        let rotor_locked = (combined & 4) != 0;

        Ok((over_temperature, over_voltage, rotor_locked))
    }
}
