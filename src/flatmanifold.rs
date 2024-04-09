use crate::r#struct::{ShapeType, Vector2D,WorkSpace,FlatTransform,ManiFold};
use crate::r#struct::RigidBody;
use std::f64::consts::PI;
use crate::collisions::{check_intersection_circles, intersect_circles, intersect_polygon2, intersect_polygon_circles2, min};
use crate::flatrgb::{clamp,clamp_for_int, initializer_r, which_shape};
use crate::flattransfom::init_tf_zero;
use crate::r#struct::ShapeType::Circle;
use crate::vectormath::{dot_s, sm, mn, div_s, c_vect, vec_zero, transform_v};

pub fn init_manifold<'a>(body_1:&'a mut RigidBody,body_2:&'a mut RigidBody,norm:&'a Vector2D,
                     depths:&'a f64,contact_a:Vector2D,
                     contact_b:Vector2D,nb_contact:i32)->ManiFold<'a>{
    ManiFold{
        body_a:body_1,
        body_b:body_2,
        normal:norm,
        depth:depths,
        contact1:contact_a,
        contact2:contact_b,
        contact_count:nb_contact,
    }
}