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

