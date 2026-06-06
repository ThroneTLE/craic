// Auto-generated. Do not edit!

// (in-package robot_slam.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class NavCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_nav = null;
      this.target_x = null;
      this.target_y = null;
      this.target_yaw = null;
      this.robot_id = null;
    }
    else {
      if (initObj.hasOwnProperty('start_nav')) {
        this.start_nav = initObj.start_nav
      }
      else {
        this.start_nav = false;
      }
      if (initObj.hasOwnProperty('target_x')) {
        this.target_x = initObj.target_x
      }
      else {
        this.target_x = 0.0;
      }
      if (initObj.hasOwnProperty('target_y')) {
        this.target_y = initObj.target_y
      }
      else {
        this.target_y = 0.0;
      }
      if (initObj.hasOwnProperty('target_yaw')) {
        this.target_yaw = initObj.target_yaw
      }
      else {
        this.target_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('robot_id')) {
        this.robot_id = initObj.robot_id
      }
      else {
        this.robot_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NavCmd
    // Serialize message field [start_nav]
    bufferOffset = _serializer.bool(obj.start_nav, buffer, bufferOffset);
    // Serialize message field [target_x]
    bufferOffset = _serializer.float64(obj.target_x, buffer, bufferOffset);
    // Serialize message field [target_y]
    bufferOffset = _serializer.float64(obj.target_y, buffer, bufferOffset);
    // Serialize message field [target_yaw]
    bufferOffset = _serializer.float64(obj.target_yaw, buffer, bufferOffset);
    // Serialize message field [robot_id]
    bufferOffset = _serializer.string(obj.robot_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NavCmd
    let len;
    let data = new NavCmd(null);
    // Deserialize message field [start_nav]
    data.start_nav = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [target_x]
    data.target_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [target_y]
    data.target_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [target_yaw]
    data.target_yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [robot_id]
    data.robot_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.robot_id.length;
    return length + 29;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_slam/NavCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '49a73690f439fc687c4e207d9765b972';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # NavCmd.msg - 导航指令消息
    bool start_nav       # 是否启动导航（true=启动，false=停止）
    float64 target_x     # 目标点 x 坐标（map 坐标系）
    float64 target_y     # 目标点 y 坐标（map 坐标系）
    float64 target_yaw   # 目标点偏航角（弧度）
    string robot_id      # 目标机器人 ID（区分发给 A/B，可选）
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NavCmd(null);
    if (msg.start_nav !== undefined) {
      resolved.start_nav = msg.start_nav;
    }
    else {
      resolved.start_nav = false
    }

    if (msg.target_x !== undefined) {
      resolved.target_x = msg.target_x;
    }
    else {
      resolved.target_x = 0.0
    }

    if (msg.target_y !== undefined) {
      resolved.target_y = msg.target_y;
    }
    else {
      resolved.target_y = 0.0
    }

    if (msg.target_yaw !== undefined) {
      resolved.target_yaw = msg.target_yaw;
    }
    else {
      resolved.target_yaw = 0.0
    }

    if (msg.robot_id !== undefined) {
      resolved.robot_id = msg.robot_id;
    }
    else {
      resolved.robot_id = ''
    }

    return resolved;
    }
};

module.exports = NavCmd;
