use super::super::config::*;

/// Finding the attributes of a generic sensor's plane
pub trait Plane {
     
    /// Checks that a given point is located on a plane
    fn on_plane(&self, input_point: &P3) -> bool;

    /// Returns the normal vector of the plane the sensor lies on
    fn plane_normal_vec(&self) -> &Vec3;

    // In the equation of a plane Ax + By + Cz = D
    // this function will return D. used in the prediction of the hit
    // on the next sensor
    fn plane_constant(&self) -> Real;

    /// Returns the center of the sensor in global coordinates
    fn global_center(&self) -> &P3;
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

    /// Fetches the rotation matrix from local -> global of the sensor.Default
    /// This is done so that KF calculations can be generic over sensor types
    fn rotation_to_global(&self) -> &Mat4;

    
    /// Fetches the rotation matrix from global -> local of the sensor.Default
    /// This is done so that KF calculations can be generic over sensor types
    fn rotation_to_local(&self) -> &Mat4;

}
