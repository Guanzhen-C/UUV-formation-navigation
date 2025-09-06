// Auto-generated. Do not edit!

// (in-package uuv_plume_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GetPlumeSourcePositionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPlumeSourcePositionRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPlumeSourcePositionRequest
    let len;
    let data = new GetPlumeSourcePositionRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_plume_msgs/GetPlumeSourcePositionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
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
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPlumeSourcePositionRequest(null);
    return resolved;
    }
};

class GetPlumeSourcePositionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.source = null;
    }
    else {
      if (initObj.hasOwnProperty('source')) {
        this.source = initObj.source
      }
      else {
        this.source = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPlumeSourcePositionResponse
    // Serialize message field [source]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.source, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPlumeSourcePositionResponse
    let len;
    let data = new GetPlumeSourcePositionResponse(null);
    // Deserialize message field [source]
    data.source = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_plume_msgs/GetPlumeSourcePositionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '74e696b4bb2fd623e3aef7e08928baee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point source
    
    
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
    const resolved = new GetPlumeSourcePositionResponse(null);
    if (msg.source !== undefined) {
      resolved.source = geometry_msgs.msg.Point.Resolve(msg.source)
    }
    else {
      resolved.source = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPlumeSourcePositionRequest,
  Response: GetPlumeSourcePositionResponse,
  md5sum() { return '74e696b4bb2fd623e3aef7e08928baee'; },
  datatype() { return 'uuv_plume_msgs/GetPlumeSourcePosition'; }
};
