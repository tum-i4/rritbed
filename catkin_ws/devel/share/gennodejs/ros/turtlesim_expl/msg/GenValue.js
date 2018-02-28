// Auto-generated. Do not edit!

// (in-package turtlesim_expl.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class GenValue {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.value = null;
      this.intrusion = null;
    }
    else {
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
      }
      if (initObj.hasOwnProperty('intrusion')) {
        this.intrusion = initObj.intrusion
      }
      else {
        this.intrusion = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GenValue
    // Serialize message field [value]
    bufferOffset = _serializer.uint8(obj.value, buffer, bufferOffset);
    // Serialize message field [intrusion]
    bufferOffset = _serializer.string(obj.intrusion, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GenValue
    let len;
    let data = new GenValue(null);
    // Deserialize message field [value]
    data.value = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [intrusion]
    data.intrusion = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.intrusion.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'turtlesim_expl/GenValue';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fff21b1ce4d892d68e646aa7a0d9dac4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 value
    string intrusion
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GenValue(null);
    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
    }

    if (msg.intrusion !== undefined) {
      resolved.intrusion = msg.intrusion;
    }
    else {
      resolved.intrusion = ''
    }

    return resolved;
    }
};

module.exports = GenValue;
