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

class StorePlumeStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output_dir = null;
      this.filename = null;
    }
    else {
      if (initObj.hasOwnProperty('output_dir')) {
        this.output_dir = initObj.output_dir
      }
      else {
        this.output_dir = '';
      }
      if (initObj.hasOwnProperty('filename')) {
        this.filename = initObj.filename
      }
      else {
        this.filename = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StorePlumeStateRequest
    // Serialize message field [output_dir]
    bufferOffset = _serializer.string(obj.output_dir, buffer, bufferOffset);
    // Serialize message field [filename]
    bufferOffset = _serializer.string(obj.filename, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StorePlumeStateRequest
    let len;
    let data = new StorePlumeStateRequest(null);
    // Deserialize message field [output_dir]
    data.output_dir = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [filename]
    data.filename = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.output_dir);
    length += _getByteLength(object.filename);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_plume_msgs/StorePlumeStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '808e933d49d59744bb06ce71ae0bb8f3';
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
    
    string output_dir
    string filename
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StorePlumeStateRequest(null);
    if (msg.output_dir !== undefined) {
      resolved.output_dir = msg.output_dir;
    }
    else {
      resolved.output_dir = ''
    }

    if (msg.filename !== undefined) {
      resolved.filename = msg.filename;
    }
    else {
      resolved.filename = ''
    }

    return resolved;
    }
};

class StorePlumeStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.plume_file = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('plume_file')) {
        this.plume_file = initObj.plume_file
      }
      else {
        this.plume_file = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StorePlumeStateResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [plume_file]
    bufferOffset = _serializer.string(obj.plume_file, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StorePlumeStateResponse
    let len;
    let data = new StorePlumeStateResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [plume_file]
    data.plume_file = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.plume_file);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uuv_plume_msgs/StorePlumeStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bc8ae3d15235b49c122723db29550c50';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string plume_file
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StorePlumeStateResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.plume_file !== undefined) {
      resolved.plume_file = msg.plume_file;
    }
    else {
      resolved.plume_file = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: StorePlumeStateRequest,
  Response: StorePlumeStateResponse,
  md5sum() { return '4fbc4c340238072a838f039245e40b09'; },
  datatype() { return 'uuv_plume_msgs/StorePlumeState'; }
};
