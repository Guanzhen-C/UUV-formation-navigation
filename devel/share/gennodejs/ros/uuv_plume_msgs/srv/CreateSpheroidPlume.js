// Auto-generated. Do not edit!

// (in-package uuv_plume_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class CreateSpheroidPlumeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.source = null;
      this.orientation = null;
      this.n_points = null;
      this.a = null;
      this.c = null;
      this.x_min = null;
      this.x_max = null;
      this.y_min = null;
      this.y_max = null;
      this.z_min = null;
      this.z_max = null;
    }
    else {
      if (initObj.hasOwnProperty('source')) {
        this.source = initObj.source
      }
      else {
        this.source = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('n_points')) {
        this.n_points = initObj.n_points
      }
      else {
        this.n_points = 0;
      }
      if (initObj.hasOwnProperty('a')) {
        this.a = initObj.a
      }
      else {
        this.a = 0.0;
      }
      if (initObj.hasOwnProperty('c')) {
        this.c = initObj.c
      }
      else {
        this.c = 0.0;
      }
      if (initObj.hasOwnProperty('x_min')) {
        this.x_min = initObj.x_min
      }
      else {
        this.x_min = 0.0;
      }
      if (initObj.hasOwnProperty('x_max')) {
        this.x_max = initObj.x_max
      }
      else {
        this.x_max = 0.0;
      }
      if (initObj.hasOwnProperty('y_min')) {
        this.y_min = initObj.y_min
      }
      else {
        this.y_min = 0.0;
      }
      if (initObj.hasOwnProperty('y_max')) {
        this.y_max = initObj.y_max
      }
      else {
        this.y_max = 0.0;
      }
      if (initObj.hasOwnProperty('z_min')) {
        this.z_min = initObj.z_min
      }
      else {
        this.z_min = 0.0;
      }
      if (initObj.hasOwnProperty('z_max')) {
        this.z_max = initObj.z_max
      }
      else {
        this.z_max = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CreateSpheroidPlumeRequest
    // Serialize message field [source]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.source, buffer, bufferOffset);
    // Serialize message field [orientation]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.orientation, buffer, bufferOffset);
    // Serialize message field [n_points]
    bufferOffset = _serializer.int32(obj.n_points, buffer, bufferOffset);
    // Serialize message field [a]
    bufferOffset = _serializer.float32(obj.a, buffer, bufferOffset);
    // Serialize message field [c]
    bufferOffset = _serializer.float32(obj.c, buffer, bufferOffset);
    // Serialize message field [x_min]
    bufferOffset = _serializer.float64(obj.x_min, buffer, bufferOffset);
    // Serialize message field [x_max]
    bufferOffset = _serializer.float64(obj.x_max, buffer, bufferOffset);
    // Serialize message field [y_min]
    bufferOffset = _serializer.float64(obj.y_min, buffer, bufferOffset);
    // Serialize message field [y_max]
    bufferOffset = _serializer.float64(obj.y_max, buffer, bufferOffset);
    // Serialize message field [z_min]
    bufferOffset = _serializer.float64(obj.z_min, buffer, bufferOffset);
    // Serialize message field [z_max]
    bufferOffset = _serializer.float64(obj.z_max, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CreateSpheroidPlumeRequest
    let len;
    let data = new CreateSpheroidPlumeRequest(null);
    // Deserialize message field [source]
    data.source = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [orientation]
    data.orientation = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [n_points]
    data.n_points = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [a]
    data.a = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [c]
    data.c = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x_min]
    data.x_min = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x_max]
    data.x_max = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_min]
    data.y_min = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_max]
    data.y_max = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_min]
    data.z_min = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_max]
    data.z_max = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 116;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_plume_msgs/CreateSpheroidPlumeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'db32e79c6acbda79b2d8be3ff5b7d146';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    
    geometry_msgs/Point source
    geometry_msgs/Quaternion orientation
    int32 n_points
    float32 a
    float32 c
    float64 x_min
    float64 x_max
    float64 y_min
    float64 y_max
    float64 z_min
    float64 z_max
    
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
    const resolved = new CreateSpheroidPlumeRequest(null);
    if (msg.source !== undefined) {
      resolved.source = geometry_msgs.msg.Point.Resolve(msg.source)
    }
    else {
      resolved.source = new geometry_msgs.msg.Point()
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = geometry_msgs.msg.Quaternion.Resolve(msg.orientation)
    }
    else {
      resolved.orientation = new geometry_msgs.msg.Quaternion()
    }

    if (msg.n_points !== undefined) {
      resolved.n_points = msg.n_points;
    }
    else {
      resolved.n_points = 0
    }

    if (msg.a !== undefined) {
      resolved.a = msg.a;
    }
    else {
      resolved.a = 0.0
    }

    if (msg.c !== undefined) {
      resolved.c = msg.c;
    }
    else {
      resolved.c = 0.0
    }

    if (msg.x_min !== undefined) {
      resolved.x_min = msg.x_min;
    }
    else {
      resolved.x_min = 0.0
    }

    if (msg.x_max !== undefined) {
      resolved.x_max = msg.x_max;
    }
    else {
      resolved.x_max = 0.0
    }

    if (msg.y_min !== undefined) {
      resolved.y_min = msg.y_min;
    }
    else {
      resolved.y_min = 0.0
    }

    if (msg.y_max !== undefined) {
      resolved.y_max = msg.y_max;
    }
    else {
      resolved.y_max = 0.0
    }

    if (msg.z_min !== undefined) {
      resolved.z_min = msg.z_min;
    }
    else {
      resolved.z_min = 0.0
    }

    if (msg.z_max !== undefined) {
      resolved.z_max = msg.z_max;
    }
    else {
      resolved.z_max = 0.0
    }

    return resolved;
    }
};

class CreateSpheroidPlumeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CreateSpheroidPlumeResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CreateSpheroidPlumeResponse
    let len;
    let data = new CreateSpheroidPlumeResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_plume_msgs/CreateSpheroidPlumeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CreateSpheroidPlumeResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: CreateSpheroidPlumeRequest,
  Response: CreateSpheroidPlumeResponse,
  md5sum() { return '38f854ae91d626d8d19fa91958cef8f9'; },
  datatype() { return 'uuv_plume_msgs/CreateSpheroidPlume'; }
};
