// Auto-generated. Do not edit!

// (in-package uuv_sensor_ros_plugins_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class AcousticRangeTWTT {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.from_ns = null;
      this.to_ns = null;
      this.t0 = null;
      this.t1 = null;
      this.t2 = null;
      this.r1 = null;
      this.peer_pose_t1 = null;
      this.success = null;
      this.variance = null;
    }
    else {
      if (initObj.hasOwnProperty('from_ns')) {
        this.from_ns = initObj.from_ns
      }
      else {
        this.from_ns = '';
      }
      if (initObj.hasOwnProperty('to_ns')) {
        this.to_ns = initObj.to_ns
      }
      else {
        this.to_ns = '';
      }
      if (initObj.hasOwnProperty('t0')) {
        this.t0 = initObj.t0
      }
      else {
        this.t0 = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('t1')) {
        this.t1 = initObj.t1
      }
      else {
        this.t1 = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('t2')) {
        this.t2 = initObj.t2
      }
      else {
        this.t2 = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('r1')) {
        this.r1 = initObj.r1
      }
      else {
        this.r1 = 0.0;
      }
      if (initObj.hasOwnProperty('peer_pose_t1')) {
        this.peer_pose_t1 = initObj.peer_pose_t1
      }
      else {
        this.peer_pose_t1 = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('variance')) {
        this.variance = initObj.variance
      }
      else {
        this.variance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AcousticRangeTWTT
    // Serialize message field [from_ns]
    bufferOffset = _serializer.string(obj.from_ns, buffer, bufferOffset);
    // Serialize message field [to_ns]
    bufferOffset = _serializer.string(obj.to_ns, buffer, bufferOffset);
    // Serialize message field [t0]
    bufferOffset = _serializer.time(obj.t0, buffer, bufferOffset);
    // Serialize message field [t1]
    bufferOffset = _serializer.time(obj.t1, buffer, bufferOffset);
    // Serialize message field [t2]
    bufferOffset = _serializer.time(obj.t2, buffer, bufferOffset);
    // Serialize message field [r1]
    bufferOffset = _serializer.float64(obj.r1, buffer, bufferOffset);
    // Serialize message field [peer_pose_t1]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.peer_pose_t1, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [variance]
    bufferOffset = _serializer.float64(obj.variance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AcousticRangeTWTT
    let len;
    let data = new AcousticRangeTWTT(null);
    // Deserialize message field [from_ns]
    data.from_ns = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [to_ns]
    data.to_ns = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [t0]
    data.t0 = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [t1]
    data.t1 = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [t2]
    data.t2 = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [r1]
    data.r1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [peer_pose_t1]
    data.peer_pose_t1 = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [variance]
    data.variance = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.from_ns);
    length += _getByteLength(object.to_ns);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.peer_pose_t1);
    return length + 49;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_sensor_ros_plugins_msgs/AcousticRangeTWTT';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '998a5403ceadaee45594f77aee3f1e98';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # From initiator namespace to responder namespace
    string from_ns
    string to_ns
    # Time stamps in ROS time (sim time)
    time t0
    # arrival time at responder (unknown to initiator; for logging)
    time t1
    # arrival time of reply at initiator
    time t2
    # Measured r1 = || p1(t2) - p2(t1) || (delta assumed 0 in minimal version)
    float64 r1
    # Peer pose at t1 (as provided by responder)
    geometry_msgs/PoseStamped peer_pose_t1
    # Success flag and variance of r1
    bool success
    float64 variance
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AcousticRangeTWTT(null);
    if (msg.from_ns !== undefined) {
      resolved.from_ns = msg.from_ns;
    }
    else {
      resolved.from_ns = ''
    }

    if (msg.to_ns !== undefined) {
      resolved.to_ns = msg.to_ns;
    }
    else {
      resolved.to_ns = ''
    }

    if (msg.t0 !== undefined) {
      resolved.t0 = msg.t0;
    }
    else {
      resolved.t0 = {secs: 0, nsecs: 0}
    }

    if (msg.t1 !== undefined) {
      resolved.t1 = msg.t1;
    }
    else {
      resolved.t1 = {secs: 0, nsecs: 0}
    }

    if (msg.t2 !== undefined) {
      resolved.t2 = msg.t2;
    }
    else {
      resolved.t2 = {secs: 0, nsecs: 0}
    }

    if (msg.r1 !== undefined) {
      resolved.r1 = msg.r1;
    }
    else {
      resolved.r1 = 0.0
    }

    if (msg.peer_pose_t1 !== undefined) {
      resolved.peer_pose_t1 = geometry_msgs.msg.PoseStamped.Resolve(msg.peer_pose_t1)
    }
    else {
      resolved.peer_pose_t1 = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.variance !== undefined) {
      resolved.variance = msg.variance;
    }
    else {
      resolved.variance = 0.0
    }

    return resolved;
    }
};

module.exports = AcousticRangeTWTT;
