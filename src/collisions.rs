use crate::r#struct::{ShapeType, Vector2D, WorkSpace};
use crate::r#struct::RigidBody;
use std::f64::consts::PI;
use crate::vectormath::{dot_s, sm, mn, div_s, vec_zero,c_vect};


pub fn intersect_polygon_circles2(center_c:Vector2D,c_rad:f64,polygon_center:Vector2D,vertices:&mut Vec<Vector2D>)->(bool,Vector2D,f64){
    let mut normal : Vector2D = vec_zero();
    let mut depth :f64 =f64::MAX;
    for i in 0..vertices.len(){
        let va = vertices[i];
        let vb = vertices[(i+1) % vertices.len()];
        let edge:Vector2D = mn(vb,va);
        let mut axis:Vector2D =c_vect(-edge.y,edge.x);
        axis =axis.normalize();
        let (min_a, max_a) = project_vertices(vertices,axis);
        let (min_b,max_b) = project_circle(center_c,c_rad,axis);
        if min_a >= max_b || min_b >= max_a {
            return (false,normal,depth);
        }
        let axis_depth = min(max_b-min_a,max_a-min_b);
        if axis_depth<depth {
            depth = axis_depth;
            normal = axis;
        }
    }
    let cp_index = closest_point(center_c,vertices);
    let cp = vertices[cp_index as usize];
    let axis = mn(cp,center_c);

    let (min_a, max_a) = project_vertices(vertices,axis);
    let (min_b,max_b) = project_circle(center_c,c_rad,axis);
    if min_a >= max_b || min_b >= max_a {
        return (false,normal,depth);
    }
    let axis_depth = min(max_b-min_a,max_a-min_b);
    if axis_depth<depth {
        depth = axis_depth;
        normal = axis;
    }
    let direction : Vector2D = mn(polygon_center,center_c);
    if direction.dot(normal)< 0.0 {
        normal = dot_s(normal,-1.0);
    }
    (true,normal,depth)
}

//Only polygon v2

pub fn intersect_polygon2(center_a:Vector2D,vec_a: &mut Vec<Vector2D>,center_b:Vector2D,vec_b:&mut Vec<Vector2D>)->(bool,Vector2D,f64){
    let mut normal : Vector2D = vec_zero();
    let mut depth :f64 =f64::MAX;
    for i in 0..vec_a.len(){
        let va = vec_a[i];
        let vb = vec_a[(i+1) % vec_a.len()];
        let edge:Vector2D = mn(vb,va);
        let mut axis:Vector2D =c_vect(-edge.y,edge.x);
        axis =axis.normalize();
        let (min_a, max_a) = project_vertices(vec_a,axis);
        let (min_b, max_b) = project_vertices(vec_b,axis);
        if min_a >= max_b || min_b >= max_a {
            return (false,normal,depth);
        }
        let axis_depth = min(max_b-min_a,max_a-min_b);
        if axis_depth<depth {
            depth = axis_depth;
            normal = axis;
        }
    }
    for j in 0..vec_b.len(){
        let va = vec_b[j];
        let vb = vec_b[(j+1) % vec_b.len()];
        let edge:Vector2D = mn(vb,va);
        let mut  axis:Vector2D =c_vect(-edge.y,edge.x);
        axis =axis.normalize();
        let (min_a, max_a) = project_vertices(vec_a,axis);
        let (min_b, max_b) = project_vertices(vec_b,axis);
        if min_a >= max_b || min_b >= max_a {
            return (false,normal,depth);
        }
        let axis_depth = min(max_b-min_a,max_a-min_b);
        if axis_depth<depth {
            depth = axis_depth;
            normal = axis;
        }
    }

    let direction : Vector2D = mn(center_b,center_a);
    if direction.dot(normal)< 0.0 {
        normal = dot_s(normal,-1.0);
    }
    (true,normal,depth)
}


//Intersect polygon circle 1
pub fn intersect_polygon_circles(center_c:Vector2D,c_rad:f64,vertices:&mut Vec<Vector2D>)->(bool,Vector2D,f64){
    let mut normal : Vector2D = vec_zero();
    let mut depth :f64 =f64::MAX;
    for i in 0..vertices.len(){
        let va = vertices[i];
        let vb = vertices[(i+1) % vertices.len()];
        let edge:Vector2D = mn(vb,va);
        let mut axis:Vector2D =c_vect(-edge.y,edge.x);
        axis =axis.normalize();
        let (min_a, max_a) = project_vertices(vertices,axis);
        let (min_b,max_b) = project_circle(center_c,c_rad,axis);
        if min_a >= max_b || min_b >= max_a {
            return (false,normal,depth);
        }
        let axis_depth = min(max_b-min_a,max_a-min_b);
        if axis_depth<depth {
            depth = axis_depth;
            normal = axis;
        }
    }
    let cp_index = closest_point(center_c,vertices);
    let cp = vertices[cp_index as usize];
    let axis = mn(cp,center_c);

    let (min_a, max_a) = project_vertices(vertices,axis);
    let (min_b,max_b) = project_circle(center_c,c_rad,axis);
    if min_a >= max_b || min_b >= max_a {
        return (false,normal,depth);
    }
    let axis_depth = min(max_b-min_a,max_a-min_b);
    if axis_depth<depth {
        depth = axis_depth;
        normal = axis;
    }

    let polygon_center = find_arithmetic_mean(&vertices);
    let direction : Vector2D = mn(polygon_center,center_c);
    if direction.dot(normal)< 0.0 {
        normal = dot_s(normal,-1.0);
    }
    (true,normal,depth)
}
//Closest point
pub fn closest_point(center_c:Vector2D,vertices:& Vec<Vector2D >)->i32{
    let mut result = 1;
    let mut min_dist:f64 = f64::MAX;
    for i in 0..vertices.len() as i32{
        let mut v = vertices[i as usize];
        let mut dist = v.dist(center_c);
        if dist<min_dist{
            min_dist=dist ;
            result = i ;
        }
    }
    result
}
//Only polygon 1
pub fn intersect_polygon(vec_a: &mut Vec<Vector2D>,vec_b:&mut Vec<Vector2D>)->(bool,Vector2D,f64){
    let mut normal : Vector2D = vec_zero();
    let mut depth :f64 =f64::MAX;
    for i in 0..vec_a.len(){
        let va = vec_a[i];
        let vb = vec_a[(i+1) % vec_a.len()];
        let edge:Vector2D = mn(vb,va);
        let mut axis:Vector2D =c_vect(-edge.y,edge.x);
        axis =axis.normalize();
        let (min_a, max_a) = project_vertices(vec_a,axis);
        let (min_b, max_b) = project_vertices(vec_b,axis);
        if min_a >= max_b || min_b >= max_a {
            return (false,normal,depth);
        }
        let axis_depth = min(max_b-min_a,max_a-min_b);
        if axis_depth<depth {
            depth = axis_depth;
            normal = axis;
        }
    }
    for j in 0..vec_b.len(){
        let va = vec_b[j];
        let vb = vec_b[(j+1) % vec_b.len()];
        let edge:Vector2D = mn(vb,va);
        let mut  axis:Vector2D =c_vect(-edge.y,edge.x);
        axis =axis.normalize();
        let (min_a, max_a) = project_vertices(vec_a,axis);
        let (min_b, max_b) = project_vertices(vec_b,axis);
        if min_a >= max_b || min_b >= max_a {
            return (false,normal,depth);
        }
        let axis_depth = min(max_b-min_a,max_a-min_b);
        if axis_depth<depth {
            depth = axis_depth;
            normal = axis;
        }
    }

    let center_a = find_arithmetic_mean(&vec_a);
    let center_b = find_arithmetic_mean(&vec_b);

    let direction : Vector2D = mn(center_b,center_a);
    if direction.dot(normal)< 0.0 {
        normal = dot_s(normal,-1.0);
    }
    (true,normal,depth)
}

pub fn find_arithmetic_mean(vertices:&Vec<Vector2D>) ->Vector2D{
    let mut sum_x= 0.0;
    let mut sum_y= 0.0;

    for i in 0..vertices.len(){
        let v = vertices[i];
        sum_x = sum_x +v.x;
        sum_y = sum_y +v.y;
    }
    c_vect(sum_x/vertices.len() as f64,sum_x/vertices.len() as f64)
}

pub fn min (a:f64,b:f64)->f64{
    if a < b {
        return a;
    }
    b
}

pub fn project_vertices(vertices:&mut Vec<Vector2D>,axis:Vector2D)->(f64,f64){
    let mut min : f64 = f64::MAX;
    let mut max : f64 = f64::MIN;
    for i in 0..vertices.len(){
        let v =  vertices[i];
        let proj = v.dot(axis);
        if proj < min {min = proj;}
        if proj > max {max = proj;}
    }
    (min,max)
}

pub fn project_circle (center:Vector2D,rad:f64,axis:Vector2D)->(f64,f64){
    let mut min : f64 = f64::MAX;
    let mut max : f64 = f64::MIN;
    let direction = axis.normalize();
    let direction_and_radius= div_s(direction,rad);
    let point1= sm(center,direction_and_radius);
    let point2 = mn(center,direction_and_radius);
    min = point1.dot(axis);
    max = point2.dot(axis);
    if (min>max){let tp = min;min = max; max = tp;}
    (min,max)



}

pub fn intersect_circles(center_a:Vector2D,radia_a:f64,center_b:Vector2D,radia_b:f64)->(bool,Vector2D,f64){
    let mut normal = vec_zero();
    let mut distance = center_a.dist(center_b);
    let mut radii = radia_a+radia_b;
    if(distance >= radii){
        return (false,normal,0.0);
    }
    normal = mn(center_b,center_a).normalize();
    let depth = radii - distance;
    return (true,normal,depth);
}

pub fn check_intersection_circles(bodies :&mut Vec<RigidBody>){
    for i in 0..bodies.len()-1{
        let mut rigid_a=&mut bodies[i].clone();
        for j in i+1..bodies.len(){
            let mut rigid_b=& mut bodies[j].clone();
            let (is_colliding, norm, depth) = intersect_circles(rigid_a.position, rigid_a.radius, rigid_b.position, rigid_b.radius);
            if is_colliding {
                rigid_a.moves(dot_s(norm,depth /2.0));
                rigid_b.moves_to(dot_s(norm,depth / 2.0));
            }
        }
    }
}