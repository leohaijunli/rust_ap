use std::env;
use std::fs;
use std::mem::size_of;

use v6x_rust_led::ulog::{self, DataEncoder, TopicId};

fn main() {
    let path = env::args()
        .nth(1)
        .expect("usage: ulog_fixture <output.ulg>");
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&ulog::file_header(1_000_000));
    bytes.extend_from_slice(ulog::flag_bits().as_bytes());
    for topic in ulog::TOPICS {
        bytes.extend_from_slice(ulog::format(topic.id).unwrap().as_bytes());
    }
    for record in [
        ulog::information("char[7] sys_name", b"rust_ap").unwrap(),
        ulog::parameter_i32("MAV_SYS_ID", 1).unwrap(),
        ulog::parameter_f32("LINK_LED_HZ", 2.0).unwrap(),
    ] {
        bytes.extend_from_slice(record.as_bytes());
    }
    for topic in ulog::TOPICS {
        bytes.extend_from_slice(ulog::subscription(topic.id).unwrap().as_bytes());
    }
    bytes.extend_from_slice(ulog::padding(5).unwrap().as_bytes());
    let mut data = DataEncoder::new();
    for topic in ulog::TOPICS {
        let mut fields = vec![0; topic.payload_len - size_of::<u64>()];
        if topic.id == TopicId::ArmingStatus {
            fields[..4].copy_from_slice(&1_u32.to_le_bytes());
        }
        bytes.extend_from_slice(
            data.encode(topic.id, 1_000_100 + topic.id as u64, &fields)
                .unwrap()
                .as_bytes(),
        );
    }
    bytes.extend_from_slice(ulog::dropout(25).as_bytes());
    fs::write(path, bytes).expect("write ULog fixture");
}
