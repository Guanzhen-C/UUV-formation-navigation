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


//-----------------------------------------------------------

class SetPlumeConfigurationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.n_points = null;
      this.max_particles_per_iter = null;
    }
    else {
      if (initObj.hasOwnProperty('n_points')) {
        this.n_points = initObj.n_points
      }
      else {
        this.n_points = 0;
      }
      if (initObj.hasOwnProperty('max_particles_per_iter')) {
        this.max_particles_per_iter = initObj.max_particles_per_iter
      }
      else {
        this.max_particles_per_iter = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetPlumeConfigurationRequest
    // Serialize message field [n_points]
    bufferOffset = _serializer.int32(obj.n_points, buffer, bufferOffset);
    // Serialize message field [max_particles_per_iter]
    bufferOffset = _serializer.int32(obj.max_particles_per_iter, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetPlumeConfigurationRequest
    let len;
    let data = new SetPlumeConfigurationRequest(null);
    // Deserialize message field [n_points]
    data.n_points = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [max_particles_per_iter]
    data.max_particles_per_iter = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_plume_msgs/SetPlumeConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a7f8d06a81519e9cf6e191dda3bb0fa5';
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
    
    int32 n_points
    int32 max_particles_per_iter
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetPlumeConfigurationRequest(null);
    if (msg.n_points !== undefined) {
      resolved.n_points = msg.n_points;
    }
    else {
      resolved.n_points = 0
    }

    if (msg.max_particles_per_iter !== undefined) {
      resolved.max_particles_per_iter = msg.max_particles_per_iter;
    }
    else {
      resolved.max_particles_per_iter = 0
    }

    return resolved;
    }
};

class SetPlumeConfigurationResponse {
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
    // Serializes a message object of type SetPlumeConfigurationResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetPlumeConfigurationResponse
    let len;
    let data = new SetPlumeConfigurationResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_plume_msgs/SetPlumeConfigurationResponse';
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
    const resolved = new SetPlumeConfigurationResponse(null);
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
  Request: SetPlumeConfigurationRequest,
  Response: SetPlumeConfigurationResponse,
  md5sum() { return '55045be54c5c1ddb03ae4cfc1750c6aa'; },
  datatype() { return 'uuv_plume_msgs/SetPlumeConfiguration'; }
};
