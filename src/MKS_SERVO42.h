/// \file MKS_SERVO42.h
/// \brief Provides control over MKS SERVO42 stepper motors via serial communication.
///
/// \details This class is designed to interface with MKS SERVO42 stepper motors, allowing for
///      initialization and control of the motors through a series of commands sent over a
///      serial connection. It provides methods to ping motors to check their status, retrieve
///      current position, and set target positions with specified speed and direction.
///      It encapsulates the communication protocol needed to interact with the motors, providing
///      a higher-level interface for ease of use in applications.
#ifndef MKS_SERVO42_h
#define MKS_SERVO42_h

#include <Arduino.h>

namespace instruction {
  // read parameter commands
  byte const GET_ENCODER_POS = 0x30;      // Read the encoder value
  byte const READ_PULSES = 0x33;          // Read the number of pulses received
  byte const READ_ANGLE = 0x36;           // Read the angle of the motor shaft
  byte const READ_ANGLE_ERROR = 0x39;      // Read the error of the motor shaft angle
  byte const STEPPER_PING = 0x3A;        // Read the En pins status, also knows as ping command
  byte const READ_MOTOR_STATUS = 0x3E;     // Read the motor shaft status

  // set system parameter commands
  byte const CALIBRATE_ENCODER = 0x80;    // Calibrate the encoder
  byte const SET_MOTOR_TYPE = 0x81;        // Set the motor type
  byte const SET_WORK_MODE = 0x82;        // Set the work mode
  byte const SET_CURRENT_GEAR = 0x83;      // Set the current gear
  byte const SET_SUBDIVISION = 0x84;       // Set subdivision
  byte const SET_EN_PIN_ACTIVE = 0x85;     // Set the active of the En pin
  byte const SET_ROTATION_DIRECTION = 0x86; // Set the direction of motor rotation
  byte const SET_AUTO_SDD = 0x87;          // Set automatic turn off the screen
  byte const SET_STALL_PROTECTION = 0x88;   // Set the stall protection
  byte const SET_SUBDIVISION_INTERPOLATION = 0x89; // Set the subdivision interpolation function
  byte const SET_BAUD_RATE = 0x8A;         // Set the baud rate
  byte const SET_UART_ADDRESS = 0x8B;      // Set the uart address

  // set zeroMode commands
  byte const SET_ZERO_MODE = 0x90;         // Set the mode of zeroMode
  byte const SET_ZERO_POSITION = 0x91;     // Set the zero of zeroMode
  byte const SET_ZERO_SPEED = 0x92;        // Set the speed of zeroMode
  byte const SET_ZERO_DIRECTION = 0x93;    // Set the dir of zeroMode
  byte const RETURN_TO_ZERO = 0x94;        // Return to zero

  // Se PID, Acceleration, and Torque commands
  byte const SET_POSITION_KP = 0xA1;        // Set the position Kp parameter
  byte const SET_POSITION_KI = 0xA2;        // Set the position Ki parameter
  byte const SET_POSITION_KD = 0xA3;        // Set the position Kd parameter
  byte const SET_ACCELERATION = 0xA4;       // Set the acceleration (ACC) parameter
  byte const SET_MAX_TORQUE = 0xA5;         // Set the maximum torque (MaxT) parameter

  // Restore defaults
  byte const RESTORE_DEFAULT_PARAMETERS = 0x3F; // Restore default parameter command

  // Serial control
  byte const SET_EN_PIN_CR_UART = 0xF3;     // Set the En pin status in CR_UART mode
  byte const RUN_MOTOR_CONSTANT_SPEED = 0xF6; // Run the motor forward / reverse in a Constant speed
  byte const STOP_MOTOR = 0xF7;            // Stop the motor
  byte const SAVE_CLEAR_STATUS = 0xFF;      // Save/Clear the status set by the RUN_MOTOR_CONSTANT_SPEED command
  byte const MOVE_SPEED_PULSES = 0xFD;     // Run the motor to an angle at a speed. This can be more than one rotation away
};

/// @brief Driver for MKS SERVO42 stepper motors via serial communication
/// original doumentation of serial protocol https://github.com/makerbase-mks/MKS-SERVO42C/wiki/Serial-communication-description
class MKS_SERVO42 {
 public:
  /// @brief Initializes the MKS_SERVO42 object with a serial port and a baud rate.
  /// @param serialPort Pointer to the HardwareSerial object that corresponds to the serial port.
  /// @param baudRate The baud rate for serial communication. If not specified, defaults to 38400.
  void initialize(HardwareSerial *serialPort = nullptr, long const &baudRate = 38400);

  /// @brief Sends a ping command to the stepper motor to check its presence and connectivity.
  /// @param stepperId The ID of the stepper motor to ping.
  /// @return Returns true if the stepper motor is successfully pinged, false otherwise.
  bool ping(byte const &stepperId);

  uint16_t getEncoderPosition(byte stepperId);
  uint32_t getPulsesReceived(byte stepperId);
  uint32_t getMotorShaftAngle(byte stepperId);
  int16_t getMotorShaftAngleError(byte stepperId);

  /// @brief Sets the target position for the stepper motor by specifying the direction, speed, and number of pulses.
  ///        The speed parameter should not exceed 2000 RPM and the direction should be 0 for one way or 1 for the opposite.
  /// @param stepperId The ID of the stepper motor to which the command will be sent.
  /// @param speed The speed at which the motor should run, up to a maximum of 2000 RPM. negative numbers spin the other direction
  /// @param pulses The number of pulses to move, which translates to the target position.
  /// @return Returns true if the command is successfully sent and the motor starts running, false if there is an error.
  bool setTargetPosition(byte stepperId, byte direction, uint8_t speed, uint32_t pulses);

  bool runMotorConstantSpeed(byte stepperId, byte direction, uint8_t speed);

  bool setMotorType(byte stepperId, uint8_t motorType);
  bool setWorkMode(byte stepperId, uint8_t workMode);
  bool setCurrentGear(byte stepperId, uint8_t currentGear);
  bool setSubdivision(byte stepperId, uint16_t subdivision);
  bool setEnPinActive(byte stepperId, uint8_t enPinActive);
  bool setRotationDirection(byte stepperId, uint8_t rotationDirection);
  bool setAutoSDD(byte stepperId, uint8_t autoSDD);
  bool setStallProtection(byte stepperId, uint8_t stallProtection);
  bool setSubdivisionInterpolation(byte stepperId, uint8_t subdivisionInterpolation);
  bool setBaudRate(byte stepperId, uint8_t baudRate);
  bool setUartAddress(byte stepperId, uint8_t uartAddress);
  bool setZeroMode(byte stepperId, uint8_t zeroMode);
  bool setZeroPosition(byte stepperId);
  bool setZeroSpeed(byte stepperId, uint8_t zeroSpeed);
  bool setZeroDirection(byte stepperId, uint8_t zeroDirection);
  bool returnToZero(byte stepperId);
  bool setPostionKp(byte stepperId, uint16_t kp);
  bool setPositionKi(byte stepperId, uint16_t ki);
  bool setPositionKd(byte stepperId, uint16_t kd);
  bool setAcceleration(byte stepperId, uint16_t acceleration); // if you set this too high you will let the magic smoke out
  bool setMaxTorque(byte stepperId, uint16_t maxTorque);
  bool restoreDefaultParameters(byte stepperId);
  bool setEnPinCrUart(byte stepperId, uint8_t enPinStatus);
  bool stopMotor(byte stepperId);
  bool saveClearStatus(byte stepperId, uint8_t status); // resets the runMotorConstantSpeed command

 private:
  HardwareSerial *port_;

  bool sendMessage(byte stepperId, byte const &commandID);
  bool sendMessageUint8(byte stepperId, byte const &commandId, uint8_t value);
  bool sendMessageUint16(byte stepperId, byte const &commandId, uint16_t value);
  uint8_t receiveUint8();
  uint16_t receiveUint16();
  uint32_t receiveUint32();
  long recieveEncoderPosition(byte const &stepperId);
  byte calculateChecksum(const byte *message, int length);
};

#endif
