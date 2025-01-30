#include "MKS_SERVO42.h"

void MKS_SERVO42::initialize(HardwareSerial *serialPort, long const &baudRate) {
  port_ = serialPort;
  port_->begin(baudRate);
}

byte MKS_SERVO42::calculateChecksum(const byte *message, int length) {
  byte checksum = 0;
  for (int i = 0; i < length; i++) checksum += message[i];
  return checksum & 0xFF;
}

bool MKS_SERVO42::sendMessage(byte stepperId, byte const &commandId) {
  // Flush
  while (port_->read() != -1);
  stepperId += 0xe0;  // e0 is the 0th stepper on this uart port
  uint8_t length = 3;
  byte message[length];
  message[0] = stepperId;
  message[1] = commandId;
  message[2] = calculateChecksum(message, length-1);
  int bytes_written = port_->write(message, length);
  if (bytes_written == length) {
    Serial.println("Failed to send");
    return false;
  }
  return true;
}

bool MKS_SERVO42::sendMessageUint8(byte stepperId, byte const &commandId, uint8_t value) {
  // Flush
  while (port_->read() != -1);
  stepperId += 0xe0;
  uint8_t length = 4;
  byte message[length];
  message[0] = stepperId;
  message[1] = commandId;
  message[2] = value;
  message[3] = calculateChecksum(message, length-1);
  int bytes_written = port_->write(message, length);
  if (bytes_written == length) {
    Serial.println("Failed to send");
    return false;
  }
  return true;
}

bool MKS_SERVO42::sendMessageUint16(byte stepperId, byte const &commandId, uint16_t value) {
  // Flush
  while (port_->read() != -1);
  stepperId += 0xe0;
  uint8_t length = 5;
  byte message[length];
  message[0] = stepperId;
  message[1] = commandId;
  message[2] = (value >> 8) & 0xFF;;
  message[3] = value & 0xFF;
  message[4] = calculateChecksum(message, length-1);
  int bytes_written = port_->write(message, length);
  if (bytes_written == length) {
    Serial.println("Failed to send");
    return false;
  }
  return true;
}

uint8_t MKS_SERVO42::receiveUint8() {
  int messageSize = 2 + sizeof(uint8_t);
  byte bytes[messageSize];
  if (port_->readBytes(bytes, messageSize) != messageSize) {
    return 0; // Or some error value
  }
  return (uint8_t)bytes[1];
}

uint16_t MKS_SERVO42::receiveUint16() {
  int messageSize = 2 + sizeof(uint16_t);
  byte bytes[messageSize];
  if (port_->readBytes(bytes, messageSize) != messageSize) {
    return 0; // Or some error value
  }
  return (uint16_t)(bytes[1] << 8) | bytes[2];
}

uint32_t MKS_SERVO42::receiveUint32() {
  int messageSize = 2 + sizeof(uint16_t);
  byte bytes[messageSize];
  if (port_->readBytes(bytes, messageSize) != messageSize) {
    return 0; // Or some error value
  }
  return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 | (uint32_t)bytes[2] << 8 | bytes[3];
}

// generic handler for any command that sends a number, and receives a status
bool MKS_SERVO42::sendCommandAndCheckStatus(byte stepperId, byte commandId, uint16_t value) {
  if (value == 0) {
    if (!sendMessage(stepperId, commandId)) {
      return false; // Failed to send message
    }
  } else if (value <= 255) {
    if (!sendMessageUint8(stepperId, commandId, (uint8_t)value)) {
      return false; // Failed to send message
    }
  } else {
    if (!sendMessageUint16(stepperId, commandId, value)) {
      return false; // Failed to send message
    }
  }

  return receiveUint8() == 1;
}

bool MKS_SERVO42::ping(byte const &stepperId) {
  // even though ping can return three possible statuses, we only care if it equals 1,
  // so we can use the generic method.
  return sendCommandAndCheckStatus(stepperId, instruction::STEPPER_PING);
}

// long MKS_SERVO42::getCurrentPosition(byte const &stepperId) {
//   if (!sendMessage(stepperId, instruction::GET_ENCODER_POS)) {
//     return -1;
//   }
//   return recieveEncoderPosition(stepperId);
// }

// long MKS_SERVO42::recieveEncoderPosition(byte const &stepperId) {
//   byte receivedBytes[8] = {0xe0, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20};
//   size_t bytesRead = port_->readBytes(receivedBytes, 8);
//   if (bytesRead == 8 && receivedBytes[0] == (stepperId + 0xE0)) {
//     int32_t carry = (int32_t)receivedBytes[1] << 24 |
//                     (int32_t)receivedBytes[2] << 16 |
//                     (int32_t)receivedBytes[3] << 8 | (int32_t)receivedBytes[4];
//     uint16_t value =
//         (uint16_t)receivedBytes[5] << 8 | (uint16_t)receivedBytes[6];
//     return (carry * 0xffff) + value;
//   } else {
//     Serial.println("Invalid response from motor controller");
//     return false;
//   }
// }

uint16_t MKS_SERVO42::getEncoderPosition(byte stepperId) {
  if (!sendMessage(stepperId, instruction::GET_ENCODER_POS)) return 0;
  return receiveUint16();
}

uint32_t MKS_SERVO42::getPulsesReceived(byte stepperId) {
  if (!sendMessage(stepperId, instruction::READ_PULSES)) return 0;
  return receiveUint32();
}

uint32_t MKS_SERVO42::getMotorShaftAngle(byte stepperId) {
  if (!sendMessage(stepperId, instruction::READ_ANGLE)) return 0;
  return receiveUint32();
}

int16_t MKS_SERVO42::getMotorShaftAngleError(byte stepperId) {
  if (!sendMessage(stepperId, instruction::READ_ANGLE_ERROR)) return 0;
  return (int16_t)receiveUint16(); // Cast to int16_t
}

bool MKS_SERVO42::runMotorConstantSpeed(byte stepperId, int8_t speed) {
  // The int8_t here will have the sign bit int he right spot for the command
  // arduino won't let me reinterpret_cast it go figure
  while (port_->read() != -1);
  stepperId += 0xe0;
  uint8_t length = 4;
  byte message[length];
  message[0] = stepperId;
  message[1] = instruction::RUN_MOTOR_CONSTANT_SPEED;
  message[2] = speed;
  message[3] = calculateChecksum(message, length-1);
  int bytes_written = port_->write(message, length);
  if (bytes_written == length) {
    Serial.println("Failed to send");
    return false;
  }
  return true;
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setTargetPosition(byte stepperId, int8_t speed, uint32_t pulses) {
  if (abs(speed) > 2000) {
    Serial.println("Speed out of range or invalid directionection");
    return false;
  }
  stepperId += 0xE0;
  byte message[8];
  message[0] = stepperId;
  message[1] = instruction::MOVE_SPEED_PULSES;
  message[2] = speed;  // signed int gets turned into a byte. MSB indicates direction 
  message[3] = (pulses >> 24) & 0xFF;
  message[4] = (pulses >> 16) & 0xFF;
  message[5] = (pulses >> 8) & 0xFF;
  message[6] = pulses & 0xFF;
  message[7] = calculateChecksum(message, 7);
  port_->write(message, sizeof(message));

  return receiveUint8() != 0;
}

// Implementations for commands that take a number and return a status
bool MKS_SERVO42::setMotorType(byte stepperId, uint8_t motorType) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_MOTOR_TYPE, motorType);
}

bool MKS_SERVO42::setWorkMode(byte stepperId, uint8_t workMode) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_WORK_MODE, workMode);
}

bool MKS_SERVO42::setCurrentGear(byte stepperId, uint8_t currentGear) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_CURRENT_GEAR, currentGear);
}

bool MKS_SERVO42::setSubdivision(byte stepperId, uint16_t subdivision) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_SUBDIVISION, subdivision);
}

bool MKS_SERVO42::setEnPinActive(byte stepperId, uint8_t enPinActive) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_EN_PIN_ACTIVE, enPinActive);
}

bool MKS_SERVO42::setRotationDirection(byte stepperId, uint8_t rotationDirection) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_ROTATION_DIRECTION, rotationDirection);
}

bool MKS_SERVO42::setAutoSDD(byte stepperId, uint8_t autoSDD) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_AUTO_SDD, autoSDD);
}

bool MKS_SERVO42::setStallProtection(byte stepperId, uint8_t stallProtection) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_STALL_PROTECTION, stallProtection);
}

bool MKS_SERVO42::setSubdivisionInterpolation(byte stepperId, uint8_t subdivisionInterpolation) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_SUBDIVISION_INTERPOLATION, subdivisionInterpolation);
}

bool MKS_SERVO42::setBaudRate(byte stepperId, uint8_t baudRate) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_BAUD_RATE, baudRate);
}

bool MKS_SERVO42::setUartAddress(byte stepperId, uint8_t uartAddress) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_UART_ADDRESS, uartAddress);
}

bool MKS_SERVO42::setZeroMode(byte stepperId, uint8_t zeroMode) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_ZERO_MODE, zeroMode);
}

bool MKS_SERVO42::setZeroPosition(byte stepperId) { // No argument
  return sendCommandAndCheckStatus(stepperId, instruction::SET_ZERO_POSITION);
}

bool MKS_SERVO42::setZeroSpeed(byte stepperId, uint8_t zeroSpeed) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_ZERO_SPEED, zeroSpeed);
}

bool MKS_SERVO42::setZeroDirection(byte stepperId, uint8_t zeroDirection) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_ZERO_DIRECTION, zeroDirection);
}

bool MKS_SERVO42::returnToZero(byte stepperId) { // No argument
  return sendCommandAndCheckStatus(stepperId, instruction::RETURN_TO_ZERO);
}

bool MKS_SERVO42::setPostionKp(byte stepperId, uint16_t kp) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_POSITION_KP, kp);
}

bool MKS_SERVO42::setPositionKi(byte stepperId, uint16_t ki) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_POSITION_KI, ki);
}

bool MKS_SERVO42::setPositionKd(byte stepperId, uint16_t kd) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_POSITION_KD, kd);
}

bool MKS_SERVO42::setAcceleration(byte stepperId, uint16_t acceleration) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_ACCELERATION, acceleration);
}

bool MKS_SERVO42::setMaxTorque(byte stepperId, uint16_t maxTorque) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_MAX_TORQUE, maxTorque);
}

bool MKS_SERVO42::restoreDefaultParameters(byte stepperId) {
  return sendCommandAndCheckStatus(stepperId, instruction::RESTORE_DEFAULT_PARAMETERS);
}

bool MKS_SERVO42::setEnPinCrUart(byte stepperId, uint8_t enPinStatus) {
  return sendCommandAndCheckStatus(stepperId, instruction::SET_EN_PIN_CR_UART, enPinStatus);
}

bool MKS_SERVO42::stopMotor(byte stepperId) {
  return sendCommandAndCheckStatus(stepperId, instruction::STOP_MOTOR);
}

bool MKS_SERVO42::saveClearStatus(byte stepperId, uint8_t status) {
  return sendCommandAndCheckStatus(stepperId, instruction::SAVE_CLEAR_STATUS, status);
}