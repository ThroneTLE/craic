// Auto-generated. Do not edit!

// (in-package TTS_audio.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class TTSRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.text = null;
    }
    else {
      if (initObj.hasOwnProperty('text')) {
        this.text = initObj.text
      }
      else {
        this.text = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TTSRequest
    // Serialize message field [text]
    bufferOffset = _serializer.string(obj.text, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TTSRequest
    let len;
    let data = new TTSRequest(null);
    // Deserialize message field [text]
    data.text = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.text.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'TTS_audio/TTSRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '74697ed3d931f6eede8bf3a8dfeca160';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string text
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TTSRequest(null);
    if (msg.text !== undefined) {
      resolved.text = msg.text;
    }
    else {
      resolved.text = ''
    }

    return resolved;
    }
};

class TTSResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.audio_file_path = null;
    }
    else {
      if (initObj.hasOwnProperty('audio_file_path')) {
        this.audio_file_path = initObj.audio_file_path
      }
      else {
        this.audio_file_path = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TTSResponse
    // Serialize message field [audio_file_path]
    bufferOffset = _serializer.string(obj.audio_file_path, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TTSResponse
    let len;
    let data = new TTSResponse(null);
    // Deserialize message field [audio_file_path]
    data.audio_file_path = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.audio_file_path.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'TTS_audio/TTSResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9d71d2815b94c8285786e11e368ad5ac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string audio_file_path
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TTSResponse(null);
    if (msg.audio_file_path !== undefined) {
      resolved.audio_file_path = msg.audio_file_path;
    }
    else {
      resolved.audio_file_path = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: TTSRequest,
  Response: TTSResponse,
  md5sum() { return '4027c3e5e88791dcb539699a421751c8'; },
  datatype() { return 'TTS_audio/TTS'; }
};
