use crate::r#struct::Vector2D;
use  crate::r#struct::FlatTransform;

pub fn c_vect(xx:f64,yy:f64)->Vector2D{
    Vector2D{x:xx,y:yy}
}
pub fn sm(v1: Vector2D, v2: Vector2D)->Vector2D{
    Vector2D{x:v1.x+v2.x,y:v1.y+v2.y,}
}

pub fn mn(v1: Vector2D, v2: Vector2D)->Vector2D{
    Vector2D{x:v1.x-v2.x,y:v1.y-v2.y,}
}

pub fn dot_s(v1: Vector2D, s : f64)->Vector2D{
    Vector2D{x:v1.x*s,y:v1.y*s}
}

pub fn div_s(v1: Vector2D, s : f64)->Vector2D{
    Vector2D{x:v1.x/s,y:v1.y*s}
}

pub fn vec_zero()->Vector2D{
    Vector2D{x:0.0,y:0.0,}
}

pub fn transform_v(v: Vector2D,tf:FlatTransform)->Vector2D{
    let rx = tf.cos * v.x - tf.sin*v.y;
    let ry = tf.sin *v.x + tf.cos *v.y;

    let tx = rx + tf.pos_x;
    let ty = ry + tf.pos_y;

    return Vector2D{x:tx,y:ty,}
}

impl Vector2D {
    pub fn len(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    pub fn dist(&self, v: Vector2D) -> f64 {
        let dx = self.x - v.x;
        let dy = self.y - v.y;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn normalize(&self) -> Vector2D {
        let l = self.len();
        if l > 0.0 {
            Vector2D { x: self.x / l, y: self.y / l }
        } else {
            // Return zero vector if the original vector has zero magnitude
            Vector2D { x: 0.0, y: 0.0 }
        }
    }

    pub fn dot(&self, v: Vector2D) -> f64 {
        self.x * v.x + self.y * v.y
    }

    pub fn cross(&self, v: Vector2D) -> f64 {
        self.x * v.y - self.y * v.x
    }


}