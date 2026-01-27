use crate::TransformStamped;
use crate::transform::Transformable;
use crate::{Tf2Error, buffer::BufferCore};
use crate::ffi::ffi;
use crate::time::LookupTime;
use super::{header_from_ffi, header_to_ffi};
use crate::ffi_utils::call_out;

// pub fn transform_point_stamped_builtin(
//     buffer: &BufferCore,
//     input: &geometry_msgs::msg::PointStamped,
//     target_frame: &str,
//     when: LookupTime,
// ) -> Result<geometry_msgs::msg::PointStamped, Tf2Error> {
//     // Typically you use the message stamp (unless you explicitly choose Latest).
//     let tf = buffer.lookup_transform(target_frame, &input.header.frame_id, when)?;

//     let ffi_in = ffi::Tf2PointStamped {
//         header: header_to_ffi(&input.header),
//         x: input.point.x,
//         y: input.point.y,
//         z: input.point.z,
//     };

//     let ffi_out = ffi::do_transform_point_stamped(&ffi_in, &tf.to_ffi()).map_err(map_cxx_err)?;

//     let mut out = geometry_msgs::msg::PointStamped::default();
//     out.header = header_from_ffi(ffi_out.header);
//     out.point.x = ffi_out.x;
//     out.point.y = ffi_out.y;
//     out.point.z = ffi_out.z;
//     Ok(out)
// }

crate::impl_has_header_for_ros2_msg!(sensor_msgs::msg::PointCloud2);

impl Transformable for sensor_msgs::msg::PointCloud2 {
    fn apply_transform(&self, tf: &TransformStamped) -> Result<Self, Tf2Error> {
        let ffi_in = pc2_to_ffi(self);
        let ffi_out = call_out(|out| {
            ffi::do_transform_pointcloud2(&ffi_in, &tf.to_ffi(), out)
        })?;
        Ok(pc2_from_ffi(ffi_out))
    }
}

fn pc2_to_ffi(pc: &sensor_msgs::msg::PointCloud2) -> ffi::Tf2PointCloud2 {
    ffi::Tf2PointCloud2 {
        header: ffi::Tf2Header {
            stamp: ffi::Tf2Time { sec: pc.header.stamp.sec, nanosec: pc.header.stamp.nanosec },
            frame_id: pc.header.frame_id.clone(),
        },
        height: pc.height,
        width: pc.width,
        fields: pc.fields.iter().map(|f| ffi::Tf2PointField {
            name: f.name.clone(),
            offset: f.offset,
            datatype: f.datatype,
            count: f.count,
        }).collect(),
        is_bigendian: pc.is_bigendian,
        point_step: pc.point_step,
        row_step: pc.row_step,
        data: pc.data.clone(),
        is_dense: pc.is_dense,
    }
}

fn pc2_from_ffi(pc: ffi::Tf2PointCloud2) -> sensor_msgs::msg::PointCloud2 {
    let mut out = sensor_msgs::msg::PointCloud2::default();
    out.header.stamp.sec = pc.header.stamp.sec;
    out.header.stamp.nanosec = pc.header.stamp.nanosec;
    out.header.frame_id = pc.header.frame_id;

    out.height = pc.height;
    out.width = pc.width;
    out.fields = pc.fields.into_iter().map(|f| {
        let mut pf = sensor_msgs::msg::PointField::default();
        pf.name = f.name;
        pf.offset = f.offset;
        pf.datatype = f.datatype;
        pf.count = f.count;
        pf
    }).collect();

    out.is_bigendian = pc.is_bigendian;
    out.point_step = pc.point_step;
    out.row_step = pc.row_step;
    out.data = pc.data;
    out.is_dense = pc.is_dense;
    out
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::ffi::ffi;

    // ROS PointField datatype enum: FLOAT32 == 7.
    const PF_FLOAT32: u8 = 7;

    fn make_tf(
        parent_frame: &str, // target frame (tf header.frame_id)
        child_frame: &str,  // source frame (tf child_frame_id)
        translation: [f64; 3],
        rotation_xyzw: [f64; 4],
        stamp: (i32, u32),
    ) -> crate::TransformStamped {
        let raw = ffi::Tf2TransformStamped {
            stamp: ffi::Tf2Time {
                sec: stamp.0,
                nanosec: stamp.1,
            },
            parent_frame: parent_frame.to_string(),
            child_frame: child_frame.to_string(),

            translation_x: translation[0],
            translation_y: translation[1],
            translation_z: translation[2],

            rotation_x: rotation_xyzw[0],
            rotation_y: rotation_xyzw[1],
            rotation_z: rotation_xyzw[2],
            rotation_w: rotation_xyzw[3],
        };

        crate::TransformStamped::from_ffi(raw)
    }

    fn make_xyz_cloud(frame: &str, points: &[[f32; 3]]) -> sensor_msgs::msg::PointCloud2 {
        let mut cloud = sensor_msgs::msg::PointCloud2::default();

        cloud.header.frame_id = frame.to_string();
        cloud.header.stamp.sec = 0;
        cloud.header.stamp.nanosec = 0;

        cloud.height = 1;
        cloud.width = points.len() as u32;
        cloud.is_bigendian = false;
        cloud.is_dense = true;

        let mut x = sensor_msgs::msg::PointField::default();
        x.name = "x".to_string();
        x.offset = 0;
        x.datatype = PF_FLOAT32;
        x.count = 1;

        let mut y = sensor_msgs::msg::PointField::default();
        y.name = "y".to_string();
        y.offset = 4;
        y.datatype = PF_FLOAT32;
        y.count = 1;

        let mut z = sensor_msgs::msg::PointField::default();
        z.name = "z".to_string();
        z.offset = 8;
        z.datatype = PF_FLOAT32;
        z.count = 1;

        cloud.fields = vec![x, y, z];

        cloud.point_step = 12;
        cloud.row_step = cloud.point_step * cloud.width;

        cloud.data = Vec::with_capacity(cloud.row_step as usize);
        for p in points {
            cloud.data.extend_from_slice(&p[0].to_le_bytes());
            cloud.data.extend_from_slice(&p[1].to_le_bytes());
            cloud.data.extend_from_slice(&p[2].to_le_bytes());
        }

        cloud
    }

    fn read_xyz_cloud(cloud: &sensor_msgs::msg::PointCloud2) -> Vec<[f32; 3]> {
        assert_eq!(cloud.point_step, 12, "test helper assumes point_step=12");
        cloud
            .data
            .chunks_exact(12)
            .map(|c| {
                let x = f32::from_le_bytes(c[0..4].try_into().unwrap());
                let y = f32::from_le_bytes(c[4..8].try_into().unwrap());
                let z = f32::from_le_bytes(c[8..12].try_into().unwrap());
                [x, y, z]
            })
            .collect()
    }

    fn assert_near(a: f32, b: f32, eps: f32, label: &str) {
        if (a - b).abs() > eps {
            panic!("{label}: {a} vs {b} (eps={eps})");
        }
    }

    #[test]
    fn transform_pointcloud2_translation_only() {
        // Transform from "lidar" (source) into "map" (target)
        let tf = make_tf(
            "map",
            "lidar",
            [10.0, -2.0, 0.5],
            [0.0, 0.0, 0.0, 1.0], // identity quaternion
            (123, 456),
        );

        let cloud_in = make_xyz_cloud("lidar", &[[1.0, 2.0, 3.0]]);
        let cloud_out = cloud_in.apply_transform(&tf).expect("transform should succeed");

        // tf2_sensor_msgs does: p_out = p_in; p_out.header = t_in.header;
        // So output header should match the transform header. :contentReference[oaicite:2]{index=2}
        assert_eq!(cloud_out.header.frame_id, "map");
        assert_eq!(cloud_out.header.stamp.sec, 123);
        assert_eq!(cloud_out.header.stamp.nanosec, 456);

        let pts = read_xyz_cloud(&cloud_out);
        assert_eq!(pts.len(), 1);
        assert_near(pts[0][0], 11.0, 1e-5, "x");
        assert_near(pts[0][1], 0.0, 1e-5, "y");
        assert_near(pts[0][2], 3.5, 1e-5, "z");
    }

    #[test]
    fn transform_pointcloud2_rotate_z_90deg() {
        // 90 deg yaw about +Z: q = [0,0,sin(pi/4),cos(pi/4)]
        let s = (0.5f64).sqrt();
        let tf = make_tf("map", "lidar", [0.0, 0.0, 0.0], [0.0, 0.0, s, s], (0, 0));

        let cloud_in = make_xyz_cloud("lidar", &[[1.0, 0.0, 0.0]]);
        let cloud_out = cloud_in.apply_transform(&tf).expect("transform should succeed");

        let pts = read_xyz_cloud(&cloud_out);
        assert_eq!(pts.len(), 1);

        // (1,0,0) rotated +90deg about Z -> (0,1,0)
        assert_near(pts[0][0], 0.0, 1e-5, "x");
        assert_near(pts[0][1], 1.0, 1e-5, "y");
        assert_near(pts[0][2], 0.0, 1e-5, "z");
    }

    #[test]
    fn transform_pointcloud2_missing_xyz_fields_returns_err() {
        let tf = make_tf("map", "lidar", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], (0, 0));

        // Malformed cloud: no fields at all, but has 12 bytes of data.
        let mut cloud = sensor_msgs::msg::PointCloud2::default();
        cloud.header.frame_id = "lidar".to_string();
        cloud.height = 1;
        cloud.width = 1;
        cloud.point_step = 12;
        cloud.row_step = 12;
        cloud.is_bigendian = false;
        cloud.is_dense = true;
        cloud.data = vec![0u8; 12];

        let res = cloud.apply_transform(&tf);
        assert!(
            matches!(res, Err(crate::Tf2Error::InvalidArgument(_)) | Err(crate::Tf2Error::Other(_))),
            "expected an error for malformed PointCloud2, got: {res:?}"
        );
    }
}
