pub mod constants;
pub mod subsystems;

use crate::constants::joystick_map::{LEFT_JOYSTICK_ID, RIGHT_JOYSTICK_ID};
use crate::subsystems::Drivetrain;
use frcrs::input::Joystick;

pub struct Ferris {
    drivetrain: Drivetrain,
}
impl Ferris {
    pub fn new() -> Ferris {
        Ferris {
            drivetrain: Drivetrain::new(),
        }
    }
}
pub struct Joysticks {
    left_drive: Joystick,
    right_drive: Joystick,
}

impl Joysticks {
    pub fn new() -> Joysticks {
        Joysticks {
            left_drive: Joystick::new(LEFT_JOYSTICK_ID),
            right_drive: Joystick::new(RIGHT_JOYSTICK_ID),
        }
    }
}

pub fn teleop(ferris: &mut Ferris, joysticks: &mut Joysticks) {
    ferris.drivetrain.control_drivetrain(joysticks);
}
