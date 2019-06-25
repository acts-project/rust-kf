use super::super::config::*;

/// Finding the attributes of a generic sensor's plane
pub trait Plane {
    // needs to be & mut since it will call plane() which expects 
    // mutable access
    /// Checks that a given point is located on a plane
    fn on_plane(&self, input_point: &P3) -> bool;

    fn plane_normal_vec(&self) -> &Vec3;
}

/// Transformations between global and local reference frames. Additionally, It can be used to check if a 
/// given local or global point is within the bounds of the sensor.
pub trait Transform{
    /// Converts a point in the global reference frame to a point in the local reference frame of the sensor.
    fn to_global(&self, input_point: P3) -> P3;

    /// Converts a point in the local refernce frame of the sensor to the global reference frame.
    fn to_local(&self, input_point: P3) -> P2;

    /// Checks if a global point is contained within the localized bounds of a sensor.
    fn inside_global(&self, input_point: P3) -> bool {
        let local_point = Self::to_local(&self, input_point);
        Self::inside(&self,&local_point)
    }

    /// Checks if a local point is contained within the bounds of a sensor.
    fn inside(&self, input: &P2) -> bool;
}
