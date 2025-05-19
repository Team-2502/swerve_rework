pub mod robotmap {
    pub mod drivetrain {
        pub const GYRO_ID: i32 = 9;
        pub const FL_DRIVE_ID: i32 = 4;
        pub const FL_TURN_ID: i32 = 3;
        pub const FL_ENCODER_ID: i32 = 14;
        pub const BL_DRIVE_ID: i32 = 5;
        pub const BL_TURN_ID: i32 = 6;
        pub const BL_ENCODER_ID: i32 = 15;
        pub const BR_DRIVE_ID: i32 = 7;
        pub const BR_TURN_ID: i32 = 8;
        pub const BR_ENCODER_ID: i32 = 16;
        pub const FR_DRIVE_ID: i32 = 1;
        pub const FR_TURN_ID: i32 = 2;
        pub const FR_ENCODER_ID: i32 = 13;
        


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

    pub const WHEELBASE_WIDTH_INCHES: f64 = 750.;
    pub const WHEELBASE_LENGTH_INCHES: f64 = 750.;
    pub const SWERVE_TURN_RATIO: f64 = 12.8; 
    
    
}
