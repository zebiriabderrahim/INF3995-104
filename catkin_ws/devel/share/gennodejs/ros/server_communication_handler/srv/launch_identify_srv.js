// Auto-generated. Do not edit!

// (in-package server_communication_handler.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class launch_identify_srvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.in = null;
    }
    else {
      if (initObj.hasOwnProperty('in')) {
        this.in = initObj.in
      }
      else {
        this.in = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type launch_identify_srvRequest
    // Serialize message field [in]
    bufferOffset = _serializer.string(obj.in, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type launch_identify_srvRequest
    let len;
    let data = new launch_identify_srvRequest(null);
    // Deserialize message field [in]
    data.in = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.in);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'server_communication_handler/launch_identify_srvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2718218ecd3037e7050a0e8416c50c33';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string in
     
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new launch_identify_srvRequest(null);
    if (msg.in !== undefined) {
      resolved.in = msg.in;
    }
    else {
      resolved.in = ''
    }

    return resolved;
    }
};

class launch_identify_srvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type launch_identify_srvResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type launch_identify_srvResponse
    let len;
    let data = new launch_identify_srvResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'server_communication_handler/launch_identify_srvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new launch_identify_srvResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: launch_identify_srvRequest,
  Response: launch_identify_srvResponse,
  md5sum() { return '2718218ecd3037e7050a0e8416c50c33'; },
  datatype() { return 'server_communication_handler/launch_identify_srv'; }
};
