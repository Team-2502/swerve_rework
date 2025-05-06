use frcrs::{alliance_station, deadzone};
use crate::constants::robotmap;
use frcrs::ctre::{ControlMode, Talon,};
use nalgebra::{Rotation2, Vector2};
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::Angle;
use crate::{constants, Ferris, Joysticks};
use crate::constants::joystick_map::SLOW_MODE;
use frcrs::redux::CanAndGyro;

pub struct Drivetrain {
    gyro: CanAndGyro,
    pub offset: Angle,
}

impl Default for Drivetrain {
    fn default() -> Self {
        Self::new()
    }
}

impl Drivetrain {
    pub fn new() -> Self {
        Self {
            gyro: CanAndGyro::new(constants::robotmap::drivetrain::GYRO_ID),
            offset: Angle::new::<degree>(0.),
        }
    }
    pub fn control_drivetrain(&mut self, joysticks: &mut Joysticks) {
        if joysticks.left_drive.get(constants::joystick_map::ZERO_HEADING) {
            if alliance_station().blue() {
                self.set_heading(Angle::new::<degree>(0.));
            }
            else {
                self.set_heading(Angle::new::<degree>(180.));
            }
        }
        
        let mut x = -joysticks.left_drive.get_y(); 
        let mut y = -joysticks.left_drive.get_x();
        let mut rotation = -joysticks.right_drive.get_z();

        if alliance_station().red() {
            x *= -1.0;
            y *= -1.0;
        }

        let joystick_range = constants::drivetrain::DEADZONE..1.;
        let power_translate = if joysticks.left_drive.get(SLOW_MODE) {
            0.0..constants::drivetrain::SLOW_MODE_SPEED
        } else {
            0.0..1.
        };
        let power_rotate = if joysticks.left_drive.get(SLOW_MODE) {
            0.0..constants::drivetrain::SLOW_MODE_ROTATION_SPEED
        } else {
            0.0..1.
        };
        
        let mut deadzone_x = deadzone(x, &joystick_range, &power_translate);
        let mut deadzone_y = deadzone(y, &joystick_range, &power_translate);
        let deadzone_rotation = deadzone(rotation, &joystick_range, &power_rotate);
        self.set_speeds(deadzone_x, deadzone_y, deadzone_rotation);
    }
    
    pub fn set_speeds(&mut self, x: f64, y: f64, rotation: f64) {
        let mut transform = Vector2::new(x, y);
        transform = Rotation2::new(-self.get_heading_wrapped().get::<radian>()) * transform;
        print!("heading: {} ", self.get_heading_wrapped().get::<degree>());
        print!("x: {} ", transform.x);
        println!("y: {}", transform.y);
        
    }
    
    pub fn get_heading_wrapped(&mut self) -> Angle {
        let mut difference = (self.get_raw_heading() - self.offset).get::<degree>();

        difference = (difference + 180.) % 360. - 180.;
        if difference < -180. {
            difference += 360.
        };

        Angle::new::<degree>(difference)
    }
    
    pub fn get_heading(&self) -> Angle {
        let mut difference = (self.get_raw_heading() - self.offset).get::<degree>();

        Angle::new::<degree>(difference)
    }

    pub fn get_raw_heading(&self) -> Angle {
        Angle::new::<revolution>(self.gyro.get_angle())
    }
    
    pub fn set_heading(&mut self, heading: Angle) {
        self.offset = self.get_raw_heading() - heading;
    }
}
