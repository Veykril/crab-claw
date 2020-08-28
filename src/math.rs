#[inline]
pub fn sub_v2([lhs_x, lhs_y]: [f32; 2], [rhs_x, rhs_y]: [f32; 2]) -> [f32; 2] {
    [lhs_x - rhs_x, lhs_y - rhs_y]
}

#[inline]
pub fn div_v2([lhs_x, lhs_y]: [f32; 2], [rhs_x, rhs_y]: [f32; 2]) -> [f32; 2] {
    [lhs_x / rhs_x, lhs_y / rhs_y]
}

#[inline]
pub fn sub_v3([lhs_x, lhs_y, lhs_z]: [f32; 3], [rhs_x, rhs_y, rhs_z]: [f32; 3]) -> [f32; 3] {
    [lhs_x - rhs_x, lhs_y - rhs_y, lhs_z - rhs_z]
}

#[inline]
pub fn dot_v3([lhs_x, lhs_y, lhs_z]: [f32; 3], [rhs_x, rhs_y, rhs_z]: [f32; 3]) -> f32 {
    lhs_x * rhs_x + lhs_y * rhs_y + lhs_z * rhs_z
}

#[inline]
pub fn cross([lhs_x, lhs_y, lhs_z]: [f32; 3], [rhs_x, rhs_y, rhs_z]: [f32; 3]) -> [f32; 3] {
    [
        lhs_y.mul_add(rhs_z, -lhs_z * rhs_y),
        lhs_z.mul_add(rhs_x, -lhs_x * rhs_z),
        lhs_x.mul_add(rhs_y, -lhs_y * rhs_x),
    ]
}

#[inline]
pub fn magnitude_squared([x, y, z]: [f32; 3]) -> f32 {
    x.mul_add(x, y.mul_add(y, z * z))
}

#[inline]
pub fn magnitude(v: [f32; 3]) -> f32 {
    magnitude_squared(v).sqrt()
}

#[inline]
pub fn normalized([x, y, z]: [f32; 3]) -> [f32; 3] {
    let mag = magnitude([x, y, z]);
    [x / mag, y / mag, z / mag]
}
