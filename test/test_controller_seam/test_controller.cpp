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

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_is_standby);
    RUN_TEST(test_unpressed_button_on_boot_keeps_standby);
    RUN_TEST(test_standby_ignores_sensor_readings_both_unpressed_and_held);
    RUN_TEST(test_button_pressed_and_held_maintains_standby);
    RUN_TEST(test_button_released_transitions_immediately_to_racing_with_indicator);
    RUN_TEST(test_racing_state_persists_after_button_release);
    return UNITY_END();
}
