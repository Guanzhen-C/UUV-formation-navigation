// Auto-generated. Do not edit!

// (in-package uuv_sensor_ros_plugins_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class AcousticTxRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_ns = null;
      this.request_id = null;
    }
    else {
      if (initObj.hasOwnProperty('target_ns')) {
        this.target_ns = initObj.target_ns
      }
      else {
        this.target_ns = '';
      }
      if (initObj.hasOwnProperty('request_id')) {
        this.request_id = initObj.request_id
      }
      else {
        this.request_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AcousticTxRequest
    // Serialize message field [target_ns]
    bufferOffset = _serializer.string(obj.target_ns, buffer, bufferOffset);
    // Serialize message field [request_id]
    bufferOffset = _serializer.string(obj.request_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AcousticTxRequest
    let len;
    let data = new AcousticTxRequest(null);
    // Deserialize message field [target_ns]
    data.target_ns = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [request_id]
    data.request_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.target_ns);
    length += _getByteLength(object.request_id);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uuv_sensor_ros_plugins_msgs/AcousticTxRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c1b001fcc36be9a459785df486239119';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Target AUV namespace to contact (e.g., eca_a9_2)
    string target_ns
    # Optional request identifier for bookkeeping
    string request_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AcousticTxRequest(null);
    if (msg.target_ns !== undefined) {
      resolved.target_ns = msg.target_ns;
    }
    else {
      resolved.target_ns = ''
    }

    if (msg.request_id !== undefined) {
      resolved.request_id = msg.request_id;
    }
    else {
      resolved.request_id = ''
    }

    return resolved;
    }
};

module.exports = AcousticTxRequest;
