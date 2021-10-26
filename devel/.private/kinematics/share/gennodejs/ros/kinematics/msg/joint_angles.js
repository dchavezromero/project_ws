// Auto-generated. Do not edit!

// (in-package kinematics.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class joint_angles {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.theta1 = null;
      this.theta2 = null;
      this.theta3 = null;
      this.theta4 = null;
      this.theta5 = null;
      this.theta6 = null;
      this.theta7 = null;
    }
    else {
      if (initObj.hasOwnProperty('theta1')) {
        this.theta1 = initObj.theta1
      }
      else {
        this.theta1 = 0.0;
      }
      if (initObj.hasOwnProperty('theta2')) {
        this.theta2 = initObj.theta2
      }
      else {
        this.theta2 = 0.0;
      }
      if (initObj.hasOwnProperty('theta3')) {
        this.theta3 = initObj.theta3
      }
      else {
        this.theta3 = 0.0;
      }
      if (initObj.hasOwnProperty('theta4')) {
        this.theta4 = initObj.theta4
      }
      else {
        this.theta4 = 0.0;
      }
      if (initObj.hasOwnProperty('theta5')) {
        this.theta5 = initObj.theta5
      }
      else {
        this.theta5 = 0.0;
      }
      if (initObj.hasOwnProperty('theta6')) {
        this.theta6 = initObj.theta6
      }
      else {
        this.theta6 = 0.0;
      }
      if (initObj.hasOwnProperty('theta7')) {
        this.theta7 = initObj.theta7
      }
      else {
        this.theta7 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type joint_angles
    // Serialize message field [theta1]
    bufferOffset = _serializer.float64(obj.theta1, buffer, bufferOffset);
    // Serialize message field [theta2]
    bufferOffset = _serializer.float64(obj.theta2, buffer, bufferOffset);
    // Serialize message field [theta3]
    bufferOffset = _serializer.float64(obj.theta3, buffer, bufferOffset);
    // Serialize message field [theta4]
    bufferOffset = _serializer.float64(obj.theta4, buffer, bufferOffset);
    // Serialize message field [theta5]
    bufferOffset = _serializer.float64(obj.theta5, buffer, bufferOffset);
    // Serialize message field [theta6]
    bufferOffset = _serializer.float64(obj.theta6, buffer, bufferOffset);
    // Serialize message field [theta7]
    bufferOffset = _serializer.float64(obj.theta7, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type joint_angles
    let len;
    let data = new joint_angles(null);
    // Deserialize message field [theta1]
    data.theta1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta2]
    data.theta2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta3]
    data.theta3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta4]
    data.theta4 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta5]
    data.theta5 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta6]
    data.theta6 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta7]
    data.theta7 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kinematics/joint_angles';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b8d6cfc93d9c18a43f67f9435ffa4b9a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 theta1
    float64 theta2
    float64 theta3
    float64 theta4
    float64 theta5
    float64 theta6
    float64 theta7
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new joint_angles(null);
    if (msg.theta1 !== undefined) {
      resolved.theta1 = msg.theta1;
    }
    else {
      resolved.theta1 = 0.0
    }

    if (msg.theta2 !== undefined) {
      resolved.theta2 = msg.theta2;
    }
    else {
      resolved.theta2 = 0.0
    }

    if (msg.theta3 !== undefined) {
      resolved.theta3 = msg.theta3;
    }
    else {
      resolved.theta3 = 0.0
    }

    if (msg.theta4 !== undefined) {
      resolved.theta4 = msg.theta4;
    }
    else {
      resolved.theta4 = 0.0
    }

    if (msg.theta5 !== undefined) {
      resolved.theta5 = msg.theta5;
    }
    else {
      resolved.theta5 = 0.0
    }

    if (msg.theta6 !== undefined) {
      resolved.theta6 = msg.theta6;
    }
    else {
      resolved.theta6 = 0.0
    }

    if (msg.theta7 !== undefined) {
      resolved.theta7 = msg.theta7;
    }
    else {
      resolved.theta7 = 0.0
    }

    return resolved;
    }
};

module.exports = joint_angles;
