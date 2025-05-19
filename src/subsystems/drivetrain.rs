use std::ops::Index;
use frcrs::{alliance_station, deadzone};
use crate::constants::robotmap;
use frcrs::ctre::{CanCoder, ControlMode, Talon};
use nalgebra::{Rotation2, Vector2};
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::{Angle, Length};
use crate::{constants, Ferris, Joysticks};
use crate::constants::joystick_map::SLOW_MODE;
use frcrs::redux::CanAndGyro;
use frcrs::telemetry::Telemetry;
use uom::si::length::inch;
use crate::constants::drivetrain::{SWERVE_TURN_RATIO, WHEELBASE_LENGTH_INCHES, WHEELBASE_WIDTH_INCHES};
use crate::constants::robotmap::drivetrain::{BL_DRIVE_ID, BL_ENCODER_ID, BL_TURN_ID, BR_DRIVE_ID, BR_ENCODER_ID, BR_TURN_ID, FL_DRIVE_ID, FL_ENCODER_ID, FL_TURN_ID, FR_DRIVE_ID, FR_ENCODER_ID, FR_TURN_ID};
use crate::swerve::Kinematics;

pub struct Drivetrain {
    gyro: CanAndGyro,
    pub offset: Angle,
    kinematics: Kinematics,

    fl_drive: Talon,
    fl_turn: Talon,
    fl_encoder: CanCoder,
    bl_drive: Talon,
    bl_turn: Talon,
    bl_encoder: CanCoder,
    br_drive: Talon,
    br_turn: Talon,
    br_encoder: CanCoder,
    fr_drive: Talon,
    fr_turn: Talon,
    fr_encoder: CanCoder,
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
            kinematics: Kinematics::new_rectangle(Length::new::<inch>(WHEELBASE_LENGTH_INCHES), Length::new::<inch>(WHEELBASE_WIDTH_INCHES)),
            
            fl_drive: Talon::new(FL_DRIVE_ID, Some("can0".to_string())),
            fl_turn: Talon::new(FL_TURN_ID, Some("can0".to_string())),
            fl_encoder: CanCoder::new(FL_ENCODER_ID, Some("can0".to_string())),
            
            bl_drive: Talon::new(BL_DRIVE_ID, Some("can0".to_string())),
            bl_turn: Talon::new(BL_TURN_ID, Some("can0".to_string())),
            bl_encoder: CanCoder::new(BL_ENCODER_ID, Some("can0".to_string())),
            
            br_drive: Talon::new(BR_DRIVE_ID, Some("can0".to_string())),
            br_turn: Talon::new(BR_TURN_ID, Some("can0".to_string())),
            br_encoder: CanCoder::new(BR_ENCODER_ID, Some("can0".to_string())),

            fr_drive: Talon::new(FR_DRIVE_ID, Some("can0".to_string())),
            fr_turn: Talon::new(FR_TURN_ID, Some("can0".to_string())),
            fr_encoder: CanCoder::new(FR_ENCODER_ID, Some("can0".to_string())),
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
        let speeds = self.kinematics.calculate(transform.x, transform.y, rotation);
        
        self.fl_drive.set(ControlMode::Percent, speeds[0].0);
        self.fl_turn.set(ControlMode::Position, speeds[0].1.get::<revolution>() * SWERVE_TURN_RATIO);
        
        self.bl_drive.set(ControlMode::Percent, speeds[1].0);
        self.bl_turn.set(ControlMode::Position, speeds[1].1.get::<revolution>() * SWERVE_TURN_RATIO);
        
        self.br_drive.set(ControlMode::Percent, speeds[2].0);
        self.br_turn.set(ControlMode::Position, speeds[2].1.get::<revolution>() * SWERVE_TURN_RATIO);
        
        self.fr_drive.set(ControlMode::Percent, speeds[3].0);
        self.fr_turn.set(ControlMode::Position, speeds[3].1.get::<revolution>() * SWERVE_TURN_RATIO);
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
