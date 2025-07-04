#ifndef HEXAMOTION_MOCK_INTERFACES_H
#define HEXAMOTION_MOCK_INTERFACES_H

#include <Arduino.h>
#include <locomotion_system.h>

/**
 * @brief Example IMU interface returning mock data.
 */
class ExampleIMU : public IIMUInterface {
  private:
    IMUMode current_mode_;
    bool has_absolute_positioning_;

  public:
    ExampleIMU() : current_mode_(IMU_MODE_RAW_DATA), has_absolute_positioning_(false) {}

    bool initialize() override { return true; }

    IMUData readIMU() override {
        IMUData data{};
        // Basic IMU data (always available)
        data.roll = 0.0f;
        data.pitch = 0.0f;
        data.yaw = 0.0f;
        data.accel_x = 0.0f;
        data.accel_y = 0.0f;
        data.accel_z = 9.8f;
        data.gyro_x = 0.0f;
        data.gyro_y = 0.0f;
        data.gyro_z = 0.0f;
        data.is_valid = true;
        data.mode = current_mode_;
        data.has_absolute_capability = has_absolute_positioning_;

        // Absolute data (if supported)
        if (has_absolute_positioning_) {
            data.absolute_data.absolute_roll = 0.0f;
            data.absolute_data.absolute_pitch = 0.0f;
            data.absolute_data.absolute_yaw = 0.0f;
            data.absolute_data.linear_accel_x = 0.0f;
            data.absolute_data.linear_accel_y = 0.0f;
            data.absolute_data.linear_accel_z = 0.0f;
            data.absolute_data.quaternion_w = 1.0f;
            data.absolute_data.quaternion_x = 0.0f;
            data.absolute_data.quaternion_y = 0.0f;
            data.absolute_data.quaternion_z = 0.0f;
            data.absolute_data.absolute_orientation_valid = true;
            data.absolute_data.linear_acceleration_valid = true;
            data.absolute_data.quaternion_valid = true;
            data.absolute_data.calibration_status = 3;  // Fully calibrated
            data.absolute_data.system_status = 5;       // System running normally
            data.absolute_data.self_test_result = 0x0F; // All tests passed
        }

        return data;
    }

    bool calibrate() override { return true; }
    bool isConnected() override { return true; }

    bool setIMUMode(IMUMode mode) override {
        current_mode_ = mode;
        return true;
    }

    IMUMode getIMUMode() const override {
        return current_mode_;
    }

    bool hasAbsolutePositioning() const override {
        return has_absolute_positioning_;
    }

    bool getCalibrationStatus(uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag) override {
        if (system)
            *system = 3;
        if (gyro)
            *gyro = 3;
        if (accel)
            *accel = 3;
        if (mag)
            *mag = 3;
        return true;
    }

    bool runSelfTest() override { return true; }
    bool resetOrientation() override { return true; }

    bool update() override {
        // Example implementation: Update IMU readings for parallel operation
        // In real implementation, this would trigger non-blocking sensor reads
        // and update internal data registers for synchronized operation with FSR
        return true;
    }

    // Helper method to simulate an advanced IMU like BNO055
    void enableAbsolutePositioning(bool enable) {
        has_absolute_positioning_ = enable;
        if (enable && current_mode_ == IMU_MODE_RAW_DATA) {
            current_mode_ = IMU_MODE_ABSOLUTE_POS;
        }
    }
};

/**
 * @brief Example FSR interface providing constant readings.
 */
class ExampleFSR : public IFSRInterface {
  public:
    bool initialize() override { return true; }
    FSRData readFSR(int /*leg_index*/) override {
        FSRData data{};
        data.pressure = 0.0f;
        data.in_contact = false;
        data.contact_time = 0.0f;
        return data;
    }
    bool calibrateFSR(int /*leg_index*/) override { return true; }
    double getRawReading(int /*leg_index*/) override { return 0.0f; }
    bool update() override {
        // Example implementation: Update FSR readings using AdvancedAnalog DMA
        // In real implementation, this would trigger simultaneous ADC reads
        return true;
    }
};

/**
 * @brief Example servo interface storing commanded angles in RAM.
 */
class ExampleServo : public IServoInterface {
  public:
    ExampleServo() {
        for (int i = 0; i < NUM_LEGS; ++i) {
            for (int j = 0; j < DOF_PER_LEG; ++j) {
                angles[i][j] = 0.0f;
            }
        }
    }

    bool initialize() override { return true; }

    bool hasBlockingStatusFlags(int /*leg_index*/, int /*joint_index*/, uint8_t *active_flags = nullptr) override {
        // Mock implementation - no servos are blocked
        if (active_flags) {
            *active_flags = 0; // No flags active
        }
        return false; // No blocking flags
    }

    bool setJointAngleAndSpeed(int leg_index, int joint_index, double angle, double speed) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS)
            return false;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG)
            return false;
        angles[leg_index][joint_index] = angle;
        // Speed parameter is acknowledged but not used in this mock implementation
        (void)speed;
        return true;
    }

    double getJointAngle(int leg_index, int joint_index) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS)
            return 0.0f;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG)
            return 0.0f;
        return angles[leg_index][joint_index];
    }

    bool isJointMoving(int /*leg_index*/, int /*joint_index*/) override { return false; }
    bool enableTorque(int /*leg_index*/, int /*joint_index*/, bool /*enable*/) override { return true; }

  private:
    double angles[NUM_LEGS][DOF_PER_LEG];
};

#endif // HEXAMOTION_MOCK_INTERFACES_H
