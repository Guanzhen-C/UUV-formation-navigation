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

class GetNumParticlesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetNumParticlesRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetNumParticlesRequest
    let len;
    let data = new GetNumParticlesRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_plume_msgs/GetNumParticlesRequest';
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
    const resolved = new GetNumParticlesRequest(null);
    return resolved;
    }
};

class GetNumParticlesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num_particles = null;
    }
    else {
      if (initObj.hasOwnProperty('num_particles')) {
        this.num_particles = initObj.num_particles
      }
      else {
        this.num_particles = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetNumParticlesResponse
    // Serialize message field [num_particles]
    bufferOffset = _serializer.int32(obj.num_particles, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetNumParticlesResponse
    let len;
    let data = new GetNumParticlesResponse(null);
    // Deserialize message field [num_particles]
    data.num_particles = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_plume_msgs/GetNumParticlesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b3e7a05396001d2e02f04b13547a3da2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 num_particles
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetNumParticlesResponse(null);
    if (msg.num_particles !== undefined) {
      resolved.num_particles = msg.num_particles;
    }
    else {
      resolved.num_particles = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetNumParticlesRequest,
  Response: GetNumParticlesResponse,
  md5sum() { return 'b3e7a05396001d2e02f04b13547a3da2'; },
  datatype() { return 'uuv_plume_msgs/GetNumParticles'; }
};
