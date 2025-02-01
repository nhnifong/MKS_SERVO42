#include "MKS_SERVO42.h"

void MKS_SERVO42::initialize(HardwareSerial *serialPort, long const &baudRate) {
  port_ = serialPort;
  port_->begin(baudRate);
}

byte MKS_SERVO42::calculateChecksum(const byte *message, int length) {
  byte checksum = 0;
  for (int i = 0; i < length; i++) {
    checksum += message[i];
  }
  return checksum & 0xFF;
}

bool MKS_SERVO42::sendMessage(byte stepperId, byte const &commandId) {
  while (port_->available() > 0) {
    port_->read();  // Read and discard available bytes
  }
  stepperId += 0xe0;  // e0 is the 0th stepper on this uart port
  uint8_t length = 3;
  byte message[length];
  message[0] = stepperId;
  message[1] = commandId;
  message[2] = calculateChecksum(message, length-1);
  int bytes_written = port_->write(message, length);
  delay(1);
  if (bytes_written != length) {
    Serial.println("Failed to send basic");
    for (size_t i = 0; i < length; i++) {
      Serial.print(" ");
      Serial.print(message[i], HEX);
    }
    Serial.println();
    return false;
  }
  return true;
}

bool MKS_SERVO42::sendMessageUint8(byte stepperId, byte const &commandId, uint8_t value) {
  while (port_->available() > 0) {
    port_->read();  // Read and discard available bytes
  }
  stepperId += 0xe0;
  uint8_t length = 4;
  byte message[length];
  message[0] = stepperId;
  message[1] = commandId;
  message[2] = value;
  message[3] = calculateChecksum(message, length-1);
  int bytes_written = port_->write(message, length);
  delay(1);
  if (bytes_written != length) {
    Serial.println("Failed to send uint8");
    for (size_t i = 0; i < length; i++) {
      Serial.print(" ");
      Serial.print(message[i], HEX);
    }
    Serial.println();
    return false;
  }
  return true;
}

bool MKS_SERVO42::sendMessageUint16(byte stepperId, byte const &commandId, uint16_t value) {
  while (port_->available() > 0) {
    port_->read();  // Read and discard available bytes
  }
  stepperId += 0xe0;
  uint8_t length = 5;
  byte message[length];
  message[0] = stepperId;
  message[1] = commandId;
  message[2] = (value >> 8) & 0xFF;
  message[3] = value & 0xFF;
  message[4] = calculateChecksum(message, length-1); // E0 A4 0 40 C4
  int bytes_written = port_->write(message, length);
  delay(1);
  if (bytes_written != length) {
    Serial.print("Failed to send uint16");
    for (size_t i = 0; i < length; i++) {
      Serial.print(" ");
      Serial.print(message[i], HEX);
    }
    Serial.println();
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
  // The number starts at the 1st byte, not the zeroth
  return (uint16_t)(bytes[1] << 8) | bytes[2];
}

uint32_t MKS_SERVO42::receiveUint32() {
  int messageSize = 2 + sizeof(uint32_t);
  byte bytes[messageSize];
  if (port_->readBytes(bytes, messageSize) != messageSize) {
    return 0; // Or some error value
  }
  // The number starts at the 1st byte, not the zeroth
  return (uint32_t)bytes[1] << 24 | (uint32_t)bytes[2] << 16 | (uint32_t)bytes[3] << 8 | bytes[4];
}

bool MKS_SERVO42::ping(byte const &stepperId) {
  // even though ping can return three possible statuses, we only care if it equals 1,
  // so we can use the generic method.
  return sendMessage(stepperId, instruction::STEPPER_PING);
  return receiveUint8() == 1;
}

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

bool MKS_SERVO42::runMotorConstantSpeed(byte stepperId, byte direction, uint8_t speed) {
  while (port_->available() > 0) {
    port_->read();  // Read and discard available bytes
  }
  stepperId += 0xe0;
  uint8_t length = 4;
  byte message[length];
  message[0] = stepperId;
  message[1] = instruction::RUN_MOTOR_CONSTANT_SPEED;
  message[2] = (direction << 7) | (speed & 0x7F);
  message[3] = calculateChecksum(message, length-1);
  int bytes_written = port_->write(message, length);
  delay(1);
  if (bytes_written != length) {
    Serial.println("Failed to send.");
    return false;
  }
  return true;
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setTargetPosition(byte stepperId, byte direction, uint8_t speed, uint32_t pulses) {
  if (abs(speed) > 2000) {
    Serial.println("Speed out of range or invalid direction");
    return false;
  }
  Serial.print("setTargetPosition ");
  Serial.print(direction);
  Serial.print(" ");
  Serial.print(speed);
  Serial.print(" ");
  Serial.println(pulses);
  stepperId += 0xE0;
  byte message[8];
  message[0] = stepperId;
  message[1] = instruction::MOVE_SPEED_PULSES;
  message[2] = (direction << 7) | (speed & 0x7F); // VAL byte, with directionection and speed
  message[3] = (pulses >> 24) & 0xFF;
  message[4] = (pulses >> 16) & 0xFF;
  message[5] = (pulses >> 8) & 0xFF;
  message[6] = pulses & 0xFF;
  message[7] = calculateChecksum(message, 7);
  port_->write(message, sizeof(message));
  delay(1);
  for (size_t i = 0; i < 7; i++) {
    Serial.print(" ");
    Serial.print(message[i], HEX);
  }
  Serial.println();

  return receiveUint8() != 0;
}

// Implementations for commands that take a number and return a status
bool MKS_SERVO42::setMotorType(byte stepperId, uint8_t motorType) {
  sendMessageUint8(stepperId, instruction::SET_MOTOR_TYPE, motorType);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setWorkMode(byte stepperId, uint8_t workMode) {
  sendMessageUint8(stepperId, instruction::SET_WORK_MODE, workMode);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setCurrentGear(byte stepperId, uint8_t currentGear) {
  sendMessageUint8(stepperId, instruction::SET_CURRENT_GEAR, currentGear);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setSubdivision(byte stepperId, uint16_t subdivision) {
  sendMessageUint16(stepperId, instruction::SET_SUBDIVISION, subdivision);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setEnPinActive(byte stepperId, uint8_t enPinActive) {
  sendMessageUint8(stepperId, instruction::SET_EN_PIN_ACTIVE, enPinActive);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setRotationDirection(byte stepperId, uint8_t rotationDirection) {
  sendMessageUint8(stepperId, instruction::SET_ROTATION_DIRECTION, rotationDirection);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setAutoSDD(byte stepperId, uint8_t autoSDD) {
  sendMessageUint8(stepperId, instruction::SET_AUTO_SDD, autoSDD);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setStallProtection(byte stepperId, uint8_t stallProtection) {
  sendMessageUint8(stepperId, instruction::SET_STALL_PROTECTION, stallProtection);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setSubdivisionInterpolation(byte stepperId, uint8_t subdivisionInterpolation) {
  sendMessageUint8(stepperId, instruction::SET_SUBDIVISION_INTERPOLATION, subdivisionInterpolation);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setBaudRate(byte stepperId, uint8_t baudRate) {
  sendMessageUint8(stepperId, instruction::SET_BAUD_RATE, baudRate);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setUartAddress(byte stepperId, uint8_t uartAddress) {
  sendMessageUint8(stepperId, instruction::SET_UART_ADDRESS, uartAddress);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setZeroMode(byte stepperId, uint8_t zeroMode) {
  sendMessageUint8(stepperId, instruction::SET_ZERO_MODE, zeroMode);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setZeroPosition(byte stepperId) { // No argument
  sendMessage(stepperId, instruction::SET_ZERO_POSITION);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setZeroSpeed(byte stepperId, uint8_t zeroSpeed) {
  sendMessageUint8(stepperId, instruction::SET_ZERO_SPEED, zeroSpeed);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setZeroDirection(byte stepperId, uint8_t zeroDirection) {
  sendMessageUint8(stepperId, instruction::SET_ZERO_DIRECTION, zeroDirection);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::returnToZero(byte stepperId) { // No argument
  sendMessage(stepperId, instruction::RETURN_TO_ZERO);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setPostionKp(byte stepperId, uint16_t kp) {
  sendMessageUint16(stepperId, instruction::SET_POSITION_KP, kp);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setPositionKi(byte stepperId, uint16_t ki) {
  sendMessageUint16(stepperId, instruction::SET_POSITION_KI, ki);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setPositionKd(byte stepperId, uint16_t kd) {
  sendMessageUint16(stepperId, instruction::SET_POSITION_KD, kd);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setAcceleration(byte stepperId, uint16_t acceleration) {
  sendMessageUint16(stepperId, instruction::SET_ACCELERATION, acceleration);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setMaxTorque(byte stepperId, uint16_t maxTorque) {
  sendMessageUint16(stepperId, instruction::SET_MAX_TORQUE, maxTorque);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::restoreDefaultParameters(byte stepperId) {
  sendMessage(stepperId, instruction::RESTORE_DEFAULT_PARAMETERS);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::setEnPinCrUart(byte stepperId, uint8_t enPinStatus) {
  sendMessageUint8(stepperId, instruction::SET_EN_PIN_CR_UART, enPinStatus);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::stopMotor(byte stepperId) {
  sendMessage(stepperId, instruction::STOP_MOTOR);
  return receiveUint8() == 1;
}

bool MKS_SERVO42::saveClearStatus(byte stepperId, uint8_t status) {
  sendMessageUint8(stepperId, instruction::SAVE_CLEAR_STATUS, status);
  return receiveUint8() == 1;
}