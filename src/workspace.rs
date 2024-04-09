use crate::r#struct::{ShapeType, Vector2D,WorkSpace,ManiFold};
use crate::r#struct::RigidBody;
use std::f64::consts::PI;
use crate::collisions::{check_intersection_circles, intersect_circles, intersect_polygon2, intersect_polygon_circles2, min};
use crate::flatmanifold::init_manifold;
use crate::flatrgb::{clamp,clamp_for_int, initializer_r, which_shape};
use crate::flattransfom::init_tf_zero;
use crate::r#struct::ShapeType::Circle;
use crate::vectormath::{dot_s, sm, mn, div_s, c_vect, vec_zero, transform_v};

static mut BODIES : Option<Vec<RigidBody>> = None;

pub fn init_workspace()->WorkSpace<'static>{
    WorkSpace{
        mn_bs : 0.01*0.01,
        mx_bs : 64.0*64.0,
        mn_d :0.5,
        mx_d:21.4,
        min_iter:1,
        max_iter:128,
        body_list : Vec::new(),
        gravity : c_vect(0.0,9.81),
        body_count: 0,
        contact_list:Vec::new(),
    }
}

impl WorkSpace<'_>{
    pub fn add_body(&mut self, body: &mut RigidBody) {
        self.body_list.push(body.clone());
        body.index=self.body_count as i32;
        self.body_count=self.body_list.len();
    }
    pub fn remove_body(&mut self, body:&mut RigidBody) -> bool {
        if body.index == -1{
            return false
        }
        self.body_list.remove(body.index as usize);
        body.index=-1;
        true
    }
    pub fn get_body(&mut self, id:usize)->(bool,&RigidBody){
        if id < self.body_list.len()  {
            return (true,&self.body_list[id]);
        }
        (false,&self.body_list[0])
    }

    pub fn step(&mut self,time:f64,mut iterations:i32){

        iterations = clamp_for_int(iterations,self.min_iter,self.max_iter).unwrap();
        for it in 0..iterations {
            //Movement step
            for i in 0..self.body_list.len() {
                self.body_list[i].step_body(time,iterations);
            }
            //Collision step
            for i in 0..self.body_count {
                let mut body_a =  self.body_list[i].clone();

                for j in i + 1..self.body_count {
                    let mut body_b = self.body_list[j].clone();

                    if body_a.is_static && body_b.is_static { continue; }
                    let (res, mut norm, mut depth) = collide(&mut body_a.clone(), &mut body_b.clone());
                    if res {
                        if body_a.is_static {
                            self.body_list[j].moves(dot_s(norm, depth));
                        } else if body_b.is_static {
                            self.body_list[i].moves(dot_s(dot_s(norm, -1.0), depth));
                        } else {
                            self.body_list[i].moves(dot_s(norm, depth / 2.0));
                            self.body_list[j].moves(dot_s(norm, depth / 2.0));
                        }
                        //let mut contact = init_manifold(&mut body_a,&mut body_b,&norm,&depth,);

                        resolve_collision(&mut self.body_list,i,j , norm, depth);

                    }
                }
            }
        }
    }
}

pub fn resolve_collision(body_s:& mut Vec<RigidBody>,idx_a : usize,idx_b:usize, normal:Vector2D,depth:f64){
    let e = min(body_s[idx_a].restitution,body_s[idx_b].restitution);
    let rv :Vector2D = mn(body_s[idx_b].linear_velocity,body_s[idx_a].linear_velocity);
    if rv.dot(normal)>0.0 { return; }
    let mut j:f64 = -(1.0+e)*rv.dot(normal);
    let impulse= dot_s(normal,j);
    j = j/(body_s[idx_a].inv_mass + body_s[idx_b].inv_mass);
    body_s[idx_a].linear_velocity= mn(body_s[idx_a].linear_velocity,dot_s(impulse,body_s[idx_a].inv_mass));
    body_s[idx_b].linear_velocity= sm(body_s[idx_b].linear_velocity,dot_s(impulse,body_s[idx_b].inv_mass));
}

pub fn collide(body_a:&mut RigidBody, body_b:&mut RigidBody)->(bool,Vector2D,f64){
    let mut normal : Vector2D = vec_zero();
    let mut depth :f64 =f64::MAX;
    let shape_a:ShapeType = body_a.shape;
    let shape_b:ShapeType = body_b.shape;
    if which_shape(shape_a)==1 {
        if which_shape(shape_b)==1 {
            return intersect_polygon2(body_a.position,body_a.get_tfv(),body_b.position,body_b.get_tfv());
        }
        else if which_shape(shape_b)==0 {
            let (res,mut normal,depth)= intersect_polygon_circles2(body_b.position,body_b.radius,body_a.position,body_a.get_tfv());
            normal = dot_s(normal,-1.0);
            return (res,normal,depth);
        }
    }
    else if which_shape(shape_a)==0 {

        if which_shape(shape_b)==1 {
            return intersect_polygon_circles2(body_a.position,body_a.radius,body_b.position,body_b.get_tfv())
        }
        else if which_shape(shape_b)==0 {
            return intersect_circles(body_a.position,body_a.radius,body_b.position,body_b.radius);
        }
    }
    (false,normal,depth)
}
