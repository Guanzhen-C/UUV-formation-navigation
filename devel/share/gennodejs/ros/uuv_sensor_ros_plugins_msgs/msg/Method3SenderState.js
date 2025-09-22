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
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Method3SenderState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ns = null;
      this.pos = null;
      this.cross_cov_x_p = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ns')) {
        this.ns = initObj.ns
      }
      else {
        this.ns = '';
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = new PositionWithCovarianceStamped();
      }
      if (initObj.hasOwnProperty('cross_cov_x_p')) {
        this.cross_cov_x_p = initObj.cross_cov_x_p
      }
      else {
        this.cross_cov_x_p = new Array(45).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Method3SenderState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ns]
    bufferOffset = _serializer.string(obj.ns, buffer, bufferOffset);
    // Serialize message field [pos]
    bufferOffset = PositionWithCovarianceStamped.serialize(obj.pos, buffer, bufferOffset);
    // Check that the constant length array field [cross_cov_x_p] has the right length
    if (obj.cross_cov_x_p.length !== 45) {
      throw new Error('Unable to serialize array field cross_cov_x_p - length must be 45')
    }
    // Serialize message field [cross_cov_x_p]
    bufferOffset = _arraySerializer.float64(obj.cross_cov_x_p, buffer, bufferOffset, 45);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Method3SenderState
    let len;
    let data = new Method3SenderState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ns]
    data.ns = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pos]
    data.pos = PositionWithCovarianceStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [cross_cov_x_p]
    data.cross_cov_x_p = _arrayDeserializer.float64(buffer, bufferOffset, 45)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.ns);
    length += PositionWithCovarianceStamped.getMessageSize(object.pos);
    return length + 364;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_sensor_ros_plugins_msgs/Method3SenderState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76191afa343ce0995d28537435cdb145';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string ns
    uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped pos
    float64[45] cross_cov_x_p
    
    
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
    const resolved = new Method3SenderState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ns !== undefined) {
      resolved.ns = msg.ns;
    }
    else {
      resolved.ns = ''
    }

    if (msg.pos !== undefined) {
      resolved.pos = PositionWithCovarianceStamped.Resolve(msg.pos)
    }
    else {
      resolved.pos = new PositionWithCovarianceStamped()
    }

    if (msg.cross_cov_x_p !== undefined) {
      resolved.cross_cov_x_p = msg.cross_cov_x_p;
    }
    else {
      resolved.cross_cov_x_p = new Array(45).fill(0)
    }

    return resolved;
    }
};

module.exports = Method3SenderState;
