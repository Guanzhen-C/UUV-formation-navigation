// Auto-generated. Do not edit!

// (in-package uuv_sensor_ros_plugins_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PositionWithCovarianceStamped = require('./PositionWithCovarianceStamped.js');

//-----------------------------------------------------------

class AcousticRangeOWTT {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.from_ns = null;
      this.to_ns = null;
      this.t_tx = null;
      this.t_rx = null;
      this.range = null;
      this.variance = null;
      this.tx_pos = null;
      this.tx_cross_cov_x_p = null;
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
      if (initObj.hasOwnProperty('t_tx')) {
        this.t_tx = initObj.t_tx
      }
      else {
        this.t_tx = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('t_rx')) {
        this.t_rx = initObj.t_rx
      }
      else {
        this.t_rx = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('range')) {
        this.range = initObj.range
      }
      else {
        this.range = 0.0;
      }
      if (initObj.hasOwnProperty('variance')) {
        this.variance = initObj.variance
      }
      else {
        this.variance = 0.0;
      }
      if (initObj.hasOwnProperty('tx_pos')) {
        this.tx_pos = initObj.tx_pos
      }
      else {
        this.tx_pos = new PositionWithCovarianceStamped();
      }
      if (initObj.hasOwnProperty('tx_cross_cov_x_p')) {
        this.tx_cross_cov_x_p = initObj.tx_cross_cov_x_p
      }
      else {
        this.tx_cross_cov_x_p = new Array(45).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AcousticRangeOWTT
    // Serialize message field [from_ns]
    bufferOffset = _serializer.string(obj.from_ns, buffer, bufferOffset);
    // Serialize message field [to_ns]
    bufferOffset = _serializer.string(obj.to_ns, buffer, bufferOffset);
    // Serialize message field [t_tx]
    bufferOffset = _serializer.time(obj.t_tx, buffer, bufferOffset);
    // Serialize message field [t_rx]
    bufferOffset = _serializer.time(obj.t_rx, buffer, bufferOffset);
    // Serialize message field [range]
    bufferOffset = _serializer.float64(obj.range, buffer, bufferOffset);
    // Serialize message field [variance]
    bufferOffset = _serializer.float64(obj.variance, buffer, bufferOffset);
    // Serialize message field [tx_pos]
    bufferOffset = PositionWithCovarianceStamped.serialize(obj.tx_pos, buffer, bufferOffset);
    // Check that the constant length array field [tx_cross_cov_x_p] has the right length
    if (obj.tx_cross_cov_x_p.length !== 45) {
      throw new Error('Unable to serialize array field tx_cross_cov_x_p - length must be 45')
    }
    // Serialize message field [tx_cross_cov_x_p]
    bufferOffset = _arraySerializer.float64(obj.tx_cross_cov_x_p, buffer, bufferOffset, 45);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AcousticRangeOWTT
    let len;
    let data = new AcousticRangeOWTT(null);
    // Deserialize message field [from_ns]
    data.from_ns = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [to_ns]
    data.to_ns = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [t_tx]
    data.t_tx = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [t_rx]
    data.t_rx = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [range]
    data.range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [variance]
    data.variance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tx_pos]
    data.tx_pos = PositionWithCovarianceStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [tx_cross_cov_x_p]
    data.tx_cross_cov_x_p = _arrayDeserializer.float64(buffer, bufferOffset, 45)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.from_ns);
    length += _getByteLength(object.to_ns);
    length += PositionWithCovarianceStamped.getMessageSize(object.tx_pos);
    return length + 400;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_sensor_ros_plugins_msgs/AcousticRangeOWTT';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8215537df9cb58042c7c2090621e64e6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string from_ns
    string to_ns
    # Transmit time at sender and receive time at receiver (ROS sim time)
    time t_tx
    time t_rx
    
    # One-way measured range (meters) and its variance
    float64 range
    float64 variance
    
    # Transmitter's position estimate and covariance at t_tx
    uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped tx_pos
    
    # Cross-covariance between sender 15-dim error state and its position (15x3, row-major)
    float64[45] tx_cross_cov_x_p
    
    
    
    ================================================================================
    MSG: uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped
    # Copyright (c) 2016 The UUV Simulator Authors.
    # All rights reserved.
    #
    # Licensed under the Apache License, Version 2.0 (the "License");
    # you may not use this file except in compliance with the License.
    # You may obtain a copy of the License at
    #
    #     http://www.apache.org/licenses/LICENSE-2.0
    #
    # Unless required by applicable law or agreed to in writing, software
    # distributed under the License is distributed on an "AS IS" BASIS,
    # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    # See the License for the specific language governing permissions and
    # limitations under the License.
    
    # This expresses an estimated position with a reference coordinate frame and
    # timestamp
    
    Header header
    PositionWithCovariance pos
    
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
    MSG: uuv_sensor_ros_plugins_msgs/PositionWithCovariance
    # Copyright (c) 2016 The UUV Simulator Authors.
    # All rights reserved.
    #
    # Licensed under the Apache License, Version 2.0 (the "License");
    # you may not use this file except in compliance with the License.
    # You may obtain a copy of the License at
    #
    #     http://www.apache.org/licenses/LICENSE-2.0
    #
    # Unless required by applicable law or agreed to in writing, software
    # distributed under the License is distributed on an "AS IS" BASIS,
    # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    # See the License for the specific language governing permissions and
    # limitations under the License.
    
    # This represents a position in free space with uncertainty.
    
    geometry_msgs/Point pos
    
    # Row-major representation of the 3x3 covariance matrix
    float64[9] covariance
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AcousticRangeOWTT(null);
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

    if (msg.t_tx !== undefined) {
      resolved.t_tx = msg.t_tx;
    }
    else {
      resolved.t_tx = {secs: 0, nsecs: 0}
    }

    if (msg.t_rx !== undefined) {
      resolved.t_rx = msg.t_rx;
    }
    else {
      resolved.t_rx = {secs: 0, nsecs: 0}
    }

    if (msg.range !== undefined) {
      resolved.range = msg.range;
    }
    else {
      resolved.range = 0.0
    }

    if (msg.variance !== undefined) {
      resolved.variance = msg.variance;
    }
    else {
      resolved.variance = 0.0
    }

    if (msg.tx_pos !== undefined) {
      resolved.tx_pos = PositionWithCovarianceStamped.Resolve(msg.tx_pos)
    }
    else {
      resolved.tx_pos = new PositionWithCovarianceStamped()
    }

    if (msg.tx_cross_cov_x_p !== undefined) {
      resolved.tx_cross_cov_x_p = msg.tx_cross_cov_x_p;
    }
    else {
      resolved.tx_cross_cov_x_p = new Array(45).fill(0)
    }

    return resolved;
    }
};

module.exports = AcousticRangeOWTT;
