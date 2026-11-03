//! A subscriber that joins after the static broadcaster has sent several times
//! must still receive every static transform, through `/tf_static`'s
//! transient-local durability.

use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use rclrs::{Context, CreateBasicExecutor, InitOptions, IntoPrimitiveOptions, SpinOptions};
use ros_env::tf2_msgs::msg::TFMessage;
use tf2_rs::{StaticTransformBroadcaster, TransformStamped};

fn tf(child: &str, x: f64) -> TransformStamped {
    TransformStamped {
        stamp_sec: 0,
        stamp_nanosec: 0,
        parent_frame: "map".into(),
        child_frame: child.into(),
        translation: [x, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
    }
}

#[test]
fn late_subscriber_receives_every_static_transform() {
    // A domain of its own, so other nodes and test runs on this machine don't
    // publish into the test.
    let domain_id = 100 + std::process::id() as usize % 100;
    let context = Context::new(
        std::iter::empty(),
        InitOptions::new().with_domain_id(Some(domain_id)),
    )
    .unwrap();
    let mut executor = context.create_basic_executor();
    let node = executor
        .create_node("tf2_rs_static_broadcaster_test")
        .unwrap();

    let broadcaster = StaticTransformBroadcaster::new(&node).unwrap();
    let clone = broadcaster.clone();
    broadcaster.send_transform(tf("a", 1.0)).unwrap();
    clone.send_transform(tf("b", 2.0)).unwrap();
    broadcaster.send_transforms([tf("a", 3.0)]).unwrap();

    let last = Arc::new(Mutex::new(None::<Vec<(String, f64)>>));
    let last_cb = Arc::clone(&last);
    let _sub = node
        .create_subscription::<TFMessage, _>(
            "/tf_static".keep_last(1).reliable().transient_local(),
            move |msg: TFMessage| {
                let frames = msg
                    .transforms
                    .into_iter()
                    .map(|t| (t.child_frame_id, t.transform.translation.x))
                    .collect();
                *last_cb.lock().unwrap() = Some(frames);
            },
        )
        .unwrap();

    let deadline = Instant::now() + Duration::from_secs(10);
    while last.lock().unwrap().is_none() && Instant::now() < deadline {
        executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(100)));
    }

    // One message holds the whole set: "a" replaced in place, "b" from the clone.
    let received = last
        .lock()
        .unwrap()
        .take()
        .expect("no message on /tf_static");
    assert_eq!(received, [("a".to_string(), 3.0), ("b".to_string(), 2.0)]);
}
