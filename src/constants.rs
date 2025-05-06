pub mod robotmap {
    pub mod drivetrain {
        pub const MOTOR_ID: i32 = 2;
        pub const GYRO_ID: i32 = 9;
    }
}

pub mod joystick_map {
    pub const LEFT_JOYSTICK_ID: i32 = 1;
    pub const RIGHT_JOYSTICK_ID: i32 = 0;
    //left drive
    pub const SLOW_MODE: usize = 1;
    pub const ZERO_HEADING: usize = 2;
    
}

pub mod drivetrain {
    pub const DEADZONE: f64 = 0.04;
    pub const SLOW_MODE_SPEED: f64 = 0.3;
    pub const SLOW_MODE_ROTATION_SPEED: f64 = 0.2;
    
    
}
