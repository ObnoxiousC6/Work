//use std::thread;
//use std::time::Duration;
//use std::f64::consts::PI;
//use std::cmp::Ordering;

#[derive(Debug, Clone, Copy)]
pub struct Vector2D {
    pub x : f64,
    pub y : f64,
}

#[derive(Debug, Clone, Copy)]
pub struct AABB {
    pub min: Vector2D,
    pub max: Vector2D,
}

#[derive(Debug, Clone, Copy)]
pub struct Particle2D{
    position : Vector2D,
    velocity : Vector2D,
    mass : f64,
}

#[derive(Debug, Clone, Copy)]
pub enum  ShapeType{
    Circle = 0,
    Box = 1
}

#[derive(Debug, Clone)]
pub struct RigidBody{
    pub position : Vector2D,
    pub linear_velocity : Vector2D,
    pub angle : f64,
    pub angular_velocity : f64,
    pub force : Vector2D,
    pub mass : f64,
    pub inv_mass:f64,
    pub density : f64,
    pub area : f64,
    pub restitution : f64,
    pub is_static : bool,
    pub radius : f64,
    pub width : f64,
    pub height:f64,
    pub shape:ShapeType,
    pub triangles : Vec<i32>,
    pub vertices: Vec<Vector2D>,
    pub transformed_vertices: Vec<Vector2D>,
    pub aabb : AABB,
    pub tfv_required:bool,
    pub aabb_update:bool,
    pub index: i32,

}

#[derive(Debug, Clone, Copy)]
pub struct FlatTransform{
    pub pos_x:f64,
    pub pos_y:f64,
    pub sin:f64,
    pub cos:f64,
}

#[derive(Debug)]
pub struct WorkSpace<'a>{
    pub mn_bs : f64,
    pub mx_bs : f64,
    pub mn_d : f64,
    pub mx_d : f64,
    pub min_iter:i32,
    pub max_iter:i32,
    pub body_list:Vec<RigidBody>,
    pub gravity: Vector2D,
    pub body_count : usize,
    pub contact_list : Vec<ManiFold<'a>>
}

#[derive(Debug)]
pub struct ManiFold<'a>{
    pub body_a : &'a mut RigidBody,
    pub body_b : &'a mut RigidBody,
    pub normal : &'a Vector2D,
    pub depth  : &'a f64,
    pub contact1 : Vector2D,
    pub contact2 :  Vector2D,
    pub contact_count : i32,
}
