mod r#struct;
mod physics;
mod vectormath;
mod flatrgb;
mod collisions;
mod flattransfom;
mod workspace;
mod flataabb;
mod flatmanifold;

use std::f64::consts::PI;
use crate::flatrgb::initializer_r;
use crate::r#struct::{RigidBody, Vector2D,  ShapeType, FlatTransform, WorkSpace};
use crate::vectormath::vec_zero;

fn main() {
    // Create two RigidBody objects with initial values
    let po = vec_zero();
    let mut body1 = initializer_r(po,1.0,1.0,1.0,1.0,true,1.0,1.0,1.0,ShapeType::Box);
    // Define the delta time (dt) for the simulation
    let dt = 10.0;
    // Update elapsed time


    // Assert expected behavior after simulation (replace with your assertions)
    //assert!(body1.position.y < 10.0); // Body1 should fall due to gravity
    //assert!(body2.position.x != 5.0 || body2.position.y != 5.0); // Body2 position should change

    // You can add more assertions based on your `apply_gravity` and `update_pos` implementations
}
