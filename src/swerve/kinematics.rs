use nalgebra::{Rotation2, Vector2};
use uom::num_traits::FloatConst;
use uom::si::angle::radian;
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;

pub struct Kinematics {
    module_positions: Vec<Vector2<f64>>,
}
impl Kinematics {
    pub fn new(module_positions: Vec<Vector2<f64>>) -> Kinematics {
        Self { module_positions }
    }

    pub fn new_rectangle(length: Length, width: Length) -> Kinematics {
        let mut module_positions: Vec<Vector2<f64>> = Vec::new();
        let half_length = length.get::<meter>() / 2.;
        let half_width = width.get::<meter>() / 2.;
        module_positions.push(Vector2::new(half_length, half_width)); //fl, quadrant 1
        module_positions.push(Vector2::new(-half_length, half_width)); //bl, quadrant 2
        module_positions.push(Vector2::new(-half_length, -half_width)); //br, quadrant 3
        module_positions.push(Vector2::new(half_length, -half_width)); //fr, quadrant 4
        Self { module_positions }
    }
    pub fn calculate(&self, x: f64, y: f64, rotation: f64) -> Vec<(f64, Angle)> {
        let transformation = Vector2::new(x, y);
        let ninety_degrees = Rotation2::new(f64::PI() / 2.);
        let mut return_speeds = Vec::new();

        for module_position in self.module_positions.clone() {
            let mut rotation_direction = ninety_degrees * module_position.normalize();
            let mut rotation_vector = rotation_direction * rotation;
            let mut final_vector = rotation_vector + transformation;
            let final_angle = Angle::new::<radian>(f64::atan2(final_vector.y, final_vector.x));
            let final_tuple = (final_vector.magnitude(), final_angle);
            // print!("x: {} ", rotation_vector.x);
            // print!("y: {} ", rotation_vector.y);
            return_speeds.push(final_tuple);
        }
        // println!(" ");
        let mut max = 0.;
        for module_state in return_speeds.clone() {
            if module_state.0 > max {
                max = module_state.0;
            }
        }

        let return_speeds = return_speeds
            .clone()
            .iter()
            .map(|(speed, direction)| {
                if max > 1. {
                    let new_speed = speed / max;
                    (new_speed, direction.clone())
                } else {
                    let new_speed = speed.clone();
                    (new_speed, direction.clone())
                }
            })
            .collect();
        return_speeds
    }
}
//hi :D
