// Auto-generated. Do not edit!

// (in-package kinematics.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let joint_angles = require('../msg/joint_angles.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class MoveJointRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_set_points = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_set_points')) {
        this.joint_set_points = initObj.joint_set_points
      }
      else {
        this.joint_set_points = new joint_angles();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveJointRequest
    // Serialize message field [joint_set_points]
    bufferOffset = joint_angles.serialize(obj.joint_set_points, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveJointRequest
    let len;
    let data = new MoveJointRequest(null);
    // Deserialize message field [joint_set_points]
    data.joint_set_points = joint_angles.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kinematics/MoveJointRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd79d8a7ed95cb38190b2abaabcce5da1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    joint_angles joint_set_points
    
    ================================================================================
    MSG: kinematics/joint_angles
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
    const resolved = new MoveJointRequest(null);
    if (msg.joint_set_points !== undefined) {
      resolved.joint_set_points = joint_angles.Resolve(msg.joint_set_points)
    }
    else {
      resolved.joint_set_points = new joint_angles()
    }

    return resolved;
    }
};

class MoveJointResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.valid_position = null;
    }
    else {
      if (initObj.hasOwnProperty('valid_position')) {
        this.valid_position = initObj.valid_position
      }
      else {
        this.valid_position = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveJointResponse
    // Serialize message field [valid_position]
    bufferOffset = _serializer.bool(obj.valid_position, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveJointResponse
    let len;
    let data = new MoveJointResponse(null);
    // Deserialize message field [valid_position]
    data.valid_position = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kinematics/MoveJointResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d7edd7f640319e72564c9ef71c5afb3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool valid_position
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveJointResponse(null);
    if (msg.valid_position !== undefined) {
      resolved.valid_position = msg.valid_position;
    }
    else {
      resolved.valid_position = false
    }

    return resolved;
    }
};

module.exports = {
  Request: MoveJointRequest,
  Response: MoveJointResponse,
  md5sum() { return 'bc0ca7f92d556a08737d72803818c721'; },
  datatype() { return 'kinematics/MoveJoint'; }
};
