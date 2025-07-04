#ifndef TEST_STUBS_H
#define TEST_STUBS_H

#include "../src/HexaModel.h"
#include <cmath>
#include <iostream>
#include <random>

struct DummyIMU : IIMUInterface {
    double test_roll = 0.0f, test_pitch = 0.0f, test_yaw = 0.0f;
    IMUMode current_mode_ = IMU_MODE_RAW_DATA;
    bool has_absolute_ = false;

    bool initialize() override { return true; }
    IMUData readIMU() override {
        IMUData data{};
        data.roll = test_roll;
        data.pitch = test_pitch;
        data.yaw = test_yaw;
        data.accel_x = 0.0f;
        data.accel_y = 0.0f;
        data.accel_z = 9.81f;
        data.gyro_x = 0.0f;
        data.gyro_y = 0.0f;
        data.gyro_z = 0.0f;
        data.is_valid = true;
        data.mode = current_mode_;
        data.has_absolute_capability = has_absolute_;

        // Populate absolute data if enabled
        if (has_absolute_) {
            data.absolute_data.absolute_roll = test_roll;
            data.absolute_data.absolute_pitch = test_pitch;
            data.absolute_data.absolute_yaw = test_yaw;
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
            data.absolute_data.calibration_status = 3;
            data.absolute_data.system_status = 5;
            data.absolute_data.self_test_result = 0x0F;
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
        return has_absolute_;
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
        // PARALLEL SENSOR READING: For parallel implementation, this would:
        // 1. Trigger non-blocking IMU data acquisition
        // 2. Update internal data registers with latest readings
        // 3. Synchronize with FSR sensor updates for coherent timing
        // 4. Handle sensor-specific optimizations (e.g., BNO055 burst reads)
        // For simulation, just return success
        return true;
    }

    // Test helper
    void enableAbsoluteMode(bool enable) { has_absolute_ = enable; }
    void enableAbsolutePositioning(bool enable) { has_absolute_ = enable; }

    // Test helper method
    void setRPY(double roll, double pitch, double yaw) {
        test_roll = roll;
        test_pitch = pitch;
        test_yaw = yaw;
    }

    void setTestOrientation(double roll, double pitch, double yaw) {
        test_roll = roll;
        test_pitch = pitch;
        test_yaw = yaw;
    }

    void setCalibrationStatus(uint8_t system, uint8_t gyro, uint8_t accel, uint8_t mag) {
        // Update internal calibration status for testing
    }

    // NOTE: Real IMU implementation needs:
    // 1. Convention detection (physics vs direct)
    // 2. Bias calibration
    // 3. Noise filtering
    // 4. Temperature compensation
    // 5. Error handling and validation
};

struct DummyFSR : IFSRInterface {
    FSRData test_data[NUM_LEGS];

    DummyFSR() {
        for (int i = 0; i < NUM_LEGS; i++) {
            // Pre-set contact data for testing
            // Real FSR starts with no contact and requires calibration
            test_data[i] = FSRData{5.0f, true, 0.0f};
        }
    }

    bool initialize() override { return true; }
    FSRData readFSR(int leg) override {
        if (leg >= 0 && leg < NUM_LEGS) {
            return test_data[leg];
        }
        // Real implementation needs error handling for invalid leg indices
        return FSRData{0.0f, false, 0.0f};
    }
    bool calibrateFSR(int) override { return true; }
    double getRawReading(int leg) override {
        if (leg >= 0 && leg < NUM_LEGS) {
            // Returns processed pressure value
            // Real implementation returns raw ADC reading requiring conversion
            return test_data[leg].pressure;
        }
        return 0.0f;
    }

    bool update() override {
        // In real implementation, this would:
        // 1. Trigger AdvancedAnalog DMA to read all FSR channels simultaneously
        // 2. Update internal ADC value registers for all legs
        // 3. Apply calibration curves and noise filtering
        // 4. Update contact detection states
        // For simulation, just return success
        return true;
    }

    // Test helper method
    void setFSRData(int leg, double pressure, bool contact) {
        if (leg >= 0 && leg < NUM_LEGS) {
            test_data[leg].pressure = pressure;
            test_data[leg].in_contact = contact;
        }
    }

    // NOTE: Production FSR implementation needs:
    // 1. Non-linear ADC to force conversion with calibration curve
    // 2. Adaptive threshold for contact detection based on noise level
    // 3. Baseline drift compensation for temperature/aging effects
    // 4. Multi-sample filtering and debouncing for stable readings
    // 5. Cross-talk prevention between adjacent leg sensors
    // 6. Hysteresis compensation for loading/unloading differences
    // 7. Periodic recalibration and health monitoring
    // See docs/HARDWARE_INTEGRATION_CONSIDERATIONS.md for details
};

struct DummyServo : IServoInterface {
    bool initialize() override { return true; }
    bool hasBlockingStatusFlags(int, int) override {
        return false;
    }
    bool setJointAngleAndSpeed(int, int, double, double) override { return true; }
    double getJointAngle(int, int) override { return 0.0f; }
    bool isJointMoving(int, int) override { return false; }
    bool enableTorque(int, int, bool) override { return true; }
};

// Progressive servo simulator that starts with random positions and gradually moves
struct ProgressiveServo : IServoInterface {
    double current_angles[NUM_LEGS][3];
    double target_angles[NUM_LEGS][3];
    std::mt19937 rng;
    std::uniform_real_distribution<double> noise_dist;
    std::uniform_real_distribution<double> init_dist;
    bool initialized = false;
    int update_count = 0;

    ProgressiveServo() : rng(std::random_device{}()),
                         noise_dist(-0.2f, 0.2f),
                         init_dist(-15.0f, 15.0f) {
        // Initialize with random but realistic starting positions
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            // Start with varied angles that aren't the default standing pose
            current_angles[leg][0] = init_dist(rng);          // Coxa: -15 to +15°
            current_angles[leg][1] = 25.0f + init_dist(rng);  // Femur: 10 to 40°
            current_angles[leg][2] = -55.0f + init_dist(rng); // Tibia: -70 to -40°

            // Targets start the same as current
            target_angles[leg][0] = current_angles[leg][0];
            target_angles[leg][1] = current_angles[leg][1];
            target_angles[leg][2] = current_angles[leg][2];
        }
    }

    bool initialize() override {
        initialized = true;
        return true;
    }

    bool hasBlockingStatusFlags(int leg, int joint) override {
        // Mock implementation - no servos are blocked
        return false; // No blocking flags
    }

    bool setJointAngleAndSpeed(int leg, int joint, double angle, double speed) override {
        if (leg >= 0 && leg < NUM_LEGS && joint >= 0 && joint < 3) {
            // Set new target angle
            target_angles[leg][joint] = angle;

            // Use speed to adjust movement rate (simulate servo speed control)
            double diff = target_angles[leg][joint] - current_angles[leg][joint];
            double movement_rate = 0.2 * speed;                             // Base rate scaled by speed parameter
            movement_rate = std::max(0.05, std::min(1.0, movement_rate)); // Clamp to reasonable range

            // Move toward target with speed-adjusted rate, plus some noise
            current_angles[leg][joint] += diff * movement_rate + noise_dist(rng);

            return true;
        }
        return false;
    }

    double getJointAngle(int leg, int joint) override {
        if (leg >= 0 && leg < NUM_LEGS && joint >= 0 && joint < 3) {
            // Add small noise to simulate real servo feedback
            return current_angles[leg][joint] + noise_dist(rng) * 0.05f;
        }
        return 0.0f;
    }

    // Simulate servo progression each update cycle
    void updateServoPositions() {
        update_count++;
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            for (int joint = 0; joint < 3; joint++) {
                // Continue moving toward targets
                double diff = target_angles[leg][joint] - current_angles[leg][joint];
                if (std::abs(diff) > 0.1f) {
                    current_angles[leg][joint] += diff * 0.15f + noise_dist(rng) * 0.1f;
                }
            }
        }
    }

    bool isJointMoving(int, int) override { return false; }
    bool enableTorque(int, int, bool) override { return true; }
};

// Alternative names for compatibility
typedef DummyIMU MockIMU;
typedef DummyFSR MockFSR;
typedef DummyServo MockServo;
typedef ProgressiveServo RealisticServo; // Alias for backward compatibility

// Additional aliases for terrain adaptation tests
typedef DummyIMU MockIMUInterface;
typedef DummyFSR MockFSRInterface;

inline Parameters createDefaultParameters() {
    Parameters params;
    params.hexagon_radius = 200.0f;
    params.coxa_length = 50.0f;
    params.femur_length = 101.0f;
    params.tibia_length = 208.0f;
    params.robot_height = 120.0f;
    params.robot_weight = 2.0f;
    params.center_of_mass = Eigen::Vector3d(0, 0, 0);
    params.coxa_angle_limits[0] = -65.0f;
    params.coxa_angle_limits[1] = 65.0f;
    params.femur_angle_limits[0] = -75.0f;
    params.femur_angle_limits[1] = 75.0f;
    params.tibia_angle_limits[0] = -45.0f;
    params.tibia_angle_limits[1] = 45.0f;
    params.max_velocity = 100.0f;
    params.max_angular_velocity = 45.0f;
    params.stability_margin = 0.02f;
    params.control_frequency = 50.0f;
    params.fsr_touchdown_threshold = 0.1f;
    params.fsr_liftoff_threshold = 0.05f;
    params.fsr_max_pressure = 10.0f;
    // Disable smooth trajectory features for unit tests
    params.smooth_trajectory.use_current_servo_positions = false;
    params.smooth_trajectory.enable_pose_interpolation = false;
    return params;
}

#endif // TEST_STUBS_H
