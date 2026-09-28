#include <unity.h>
#include "LineFollowerController.h"

void setUp(void) {
}

void tearDown(void) {
}

static void assert_standby_outputs(const LineFollowerController& controller) {
    TEST_ASSERT_TRUE(controller.getState() == RobotState::STANDBY);
    ControllerOutputs outputs = controller.getOutputs();
    TEST_ASSERT_TRUE(outputs.isStopped);
    TEST_ASSERT_EQUAL_UINT8(0, outputs.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(0, outputs.motors.rightPwm);
    TEST_ASSERT_FALSE(outputs.indicatorActive);
}

void test_initial_state_is_standby(void) {
    LineFollowerController controller;
    assert_standby_outputs(controller);
}

void test_unpressed_button_on_boot_keeps_standby(void) {
    LineFollowerController controller;
    SensorInputs sensors = {false, false};

    // Driver has not pressed button yet
    for (uint32_t t = 0; t < 1000; t += 100) {
        controller.update(false, sensors, t);
        assert_standby_outputs(controller);
    }
}

void test_standby_ignores_sensor_readings_both_unpressed_and_held(void) {
    LineFollowerController controller;

    // Line detected while button is unpressed
    SensorInputs sensorsBoth = {true, true};
    controller.update(false, sensorsBoth, 100);
    assert_standby_outputs(controller);

    // Line detected while driver holds button down on starting grid
    controller.update(true, sensorsBoth, 200);
    assert_standby_outputs(controller);

    // Single sensor line detections while driver holds button down
    SensorInputs sensorsLeft = {true, false};
    controller.update(true, sensorsLeft, 300);
    assert_standby_outputs(controller);

    SensorInputs sensorsRight = {false, true};
    controller.update(true, sensorsRight, 400);
    assert_standby_outputs(controller);
}

void test_button_pressed_and_held_maintains_standby(void) {
    LineFollowerController controller;
    SensorInputs sensors = {false, false};

    // Driver depresses and holds button in grid
    for (uint32_t t = 100; t <= 3000; t += 500) {
        controller.update(true, sensors, t);
        assert_standby_outputs(controller);
    }
}

void test_button_released_transitions_immediately_to_racing_with_indicator(void) {
    LineFollowerController controller;
    SensorInputs sensors = {true, true};

    // Driver holds button on starting grid
    controller.update(true, sensors, 1000);
    assert_standby_outputs(controller);

    // Referee gives start signal -> Driver releases button
    controller.update(false, sensors, 1050);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RACING);

    ControllerOutputs outputs = controller.getOutputs();
    TEST_ASSERT_TRUE(outputs.indicatorActive);
    TEST_ASSERT_FALSE(outputs.isStopped);
}

void test_racing_state_persists_after_button_release(void) {
    LineFollowerController controller;
    SensorInputs sensors = {true, true};

    // Hold and release
    controller.update(true, sensors, 100);
    controller.update(false, sensors, 200);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RACING);

    // Subsequent cycles during race keep racing state and indicator active
    for (uint32_t t = 300; t <= 1000; t += 100) {
        controller.update(false, sensors, t);
        TEST_ASSERT_TRUE(controller.getState() == RobotState::RACING);
        ControllerOutputs outputs = controller.getOutputs();
        TEST_ASSERT_TRUE(outputs.indicatorActive);
        TEST_ASSERT_FALSE(outputs.isStopped);
    }
}

static void start_racing(LineFollowerController& controller) {
    SensorInputs sensors = {true, true};
    controller.update(true, sensors, 100);
    controller.update(false, sensors, 200);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RACING);
}

void test_straight_cruising_applies_base_speed_with_trim(void) {
    LineFollowerController controller;
    start_racing(controller);

    SensorInputs centered = {true, true};
    controller.update(false, centered, 300);

    ControllerOutputs outputs = controller.getOutputs();
    TEST_ASSERT_FALSE(outputs.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, outputs.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, outputs.motors.rightPwm);
    TEST_ASSERT_TRUE(outputs.motors.leftForward);
    TEST_ASSERT_TRUE(outputs.motors.rightForward);
}

void test_left_drift_steers_right_smoothly_forward(void) {
    LineFollowerController controller;
    start_racing(controller);

    // Left sensor drifted off the line -> Right sensor remains active -> steer right
    SensorInputs leftDrift = {false, true};
    controller.update(false, leftDrift, 300);

    ControllerOutputs outputs = controller.getOutputs();
    TEST_ASSERT_FALSE(outputs.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, outputs.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, outputs.motors.rightPwm);
    TEST_ASSERT_TRUE(outputs.motors.leftForward);
    TEST_ASSERT_TRUE(outputs.motors.rightForward);
}

void test_right_drift_steers_left_smoothly_forward(void) {
    LineFollowerController controller;
    start_racing(controller);

    // Right sensor drifted off the line -> Left sensor remains active -> steer left
    SensorInputs rightDrift = {true, false};
    controller.update(false, rightDrift, 300);

    ControllerOutputs outputs = controller.getOutputs();
    TEST_ASSERT_FALSE(outputs.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, outputs.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, outputs.motors.rightPwm);
    TEST_ASSERT_TRUE(outputs.motors.leftForward);
    TEST_ASSERT_TRUE(outputs.motors.rightForward);
}

void test_counter_rotation_strictly_excluded_in_active_cruising(void) {
    LineFollowerController controller;
    start_racing(controller);

    SensorInputs scenarios[] = {
        {true, true},   // Centered
        {false, true},  // Left drift
        {true, false}   // Right drift
    };

    for (size_t i = 0; i < sizeof(scenarios)/sizeof(scenarios[0]); ++i) {
        controller.update(false, scenarios[i], 300 + (uint32_t)i * 100);
        ControllerOutputs outputs = controller.getOutputs();
        TEST_ASSERT_FALSE(outputs.isStopped);
        TEST_ASSERT_TRUE_MESSAGE(outputs.motors.leftForward, "Left motor counter-rotation detected!");
        TEST_ASSERT_TRUE_MESSAGE(outputs.motors.rightForward, "Right motor counter-rotation detected!");
    }
}

void test_unified_track_polarity_normalization(void) {
    // WHITE_LINE: 0 (LOW) is line detected, 1 (HIGH) is background
    SensorInputs wCentered = LineFollowerController::normalizeSensors(0, 0, TrackPolarity::WHITE_LINE);
    TEST_ASSERT_TRUE(wCentered.leftDetected);
    TEST_ASSERT_TRUE(wCentered.rightDetected);

    SensorInputs wLeftDrift = LineFollowerController::normalizeSensors(1, 0, TrackPolarity::WHITE_LINE);
    TEST_ASSERT_FALSE(wLeftDrift.leftDetected);
    TEST_ASSERT_TRUE(wLeftDrift.rightDetected);

    SensorInputs wRightDrift = LineFollowerController::normalizeSensors(0, 1, TrackPolarity::WHITE_LINE);
    TEST_ASSERT_TRUE(wRightDrift.leftDetected);
    TEST_ASSERT_FALSE(wRightDrift.rightDetected);

    SensorInputs wBothLost = LineFollowerController::normalizeSensors(1, 1, TrackPolarity::WHITE_LINE);
    TEST_ASSERT_FALSE(wBothLost.leftDetected);
    TEST_ASSERT_FALSE(wBothLost.rightDetected);

    // BLACK_LINE: 1 (HIGH) is line detected, 0 (LOW) is background
    SensorInputs bCentered = LineFollowerController::normalizeSensors(1, 1, TrackPolarity::BLACK_LINE);
    TEST_ASSERT_TRUE(bCentered.leftDetected);
    TEST_ASSERT_TRUE(bCentered.rightDetected);

    SensorInputs bLeftDrift = LineFollowerController::normalizeSensors(0, 1, TrackPolarity::BLACK_LINE);
    TEST_ASSERT_FALSE(bLeftDrift.leftDetected);
    TEST_ASSERT_TRUE(bLeftDrift.rightDetected);

    SensorInputs bRightDrift = LineFollowerController::normalizeSensors(1, 0, TrackPolarity::BLACK_LINE);
    TEST_ASSERT_TRUE(bRightDrift.leftDetected);
    TEST_ASSERT_FALSE(bRightDrift.rightDetected);

    SensorInputs bBothLost = LineFollowerController::normalizeSensors(0, 0, TrackPolarity::BLACK_LINE);
    TEST_ASSERT_FALSE(bBothLost.leftDetected);
    TEST_ASSERT_FALSE(bBothLost.rightDetected);
}

void test_navigation_commands_across_both_track_polarities(void) {
    // 1. Controller configured for WHITE_LINE
    LineFollowerController whiteController(TrackPolarity::WHITE_LINE);
    start_racing(whiteController);

    // White line centered (both sensors read 0 / LOW)
    whiteController.updateRaw(false, 0, 0, 300);
    ControllerOutputs out = whiteController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);

    // White line left drift (left reads 1 / HIGH, right reads 0 / LOW) -> steer right
    whiteController.updateRaw(false, 1, 0, 350);
    out = whiteController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, out.motors.rightPwm);

    // White line right drift (left reads 0 / LOW, right reads 1 / HIGH) -> steer left
    whiteController.updateRaw(false, 0, 1, 400);
    out = whiteController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);

    // 2. Controller configured for BLACK_LINE
    LineFollowerController blackController(TrackPolarity::BLACK_LINE);
    start_racing(blackController);

    // Black line centered (both sensors read 1 / HIGH)
    blackController.updateRaw(false, 1, 1, 300);
    out = blackController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);

    // Black line left drift (left reads 0 / LOW, right reads 1 / HIGH) -> steer right
    blackController.updateRaw(false, 0, 1, 350);
    out = blackController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, out.motors.rightPwm);

    // Black line right drift (left reads 1 / HIGH, right reads 0 / LOW) -> steer left
    blackController.updateRaw(false, 1, 0, 400);
    out = blackController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_is_standby);
    RUN_TEST(test_unpressed_button_on_boot_keeps_standby);
    RUN_TEST(test_standby_ignores_sensor_readings_both_unpressed_and_held);
    RUN_TEST(test_button_pressed_and_held_maintains_standby);
    RUN_TEST(test_button_released_transitions_immediately_to_racing_with_indicator);
    RUN_TEST(test_racing_state_persists_after_button_release);
    RUN_TEST(test_straight_cruising_applies_base_speed_with_trim);
    RUN_TEST(test_left_drift_steers_right_smoothly_forward);
    RUN_TEST(test_right_drift_steers_left_smoothly_forward);
    RUN_TEST(test_counter_rotation_strictly_excluded_in_active_cruising);
    RUN_TEST(test_unified_track_polarity_normalization);
    RUN_TEST(test_navigation_commands_across_both_track_polarities);
    return UNITY_END();
}
