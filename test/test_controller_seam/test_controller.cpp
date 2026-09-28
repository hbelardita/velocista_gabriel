#include <unity.h>
#include "LineFollowerController.h"

void setUp(void) {
}

void tearDown(void) {
}

void test_initial_state_is_standby(void) {
    LineFollowerController controller;
    TEST_ASSERT_TRUE(controller.getState() == RobotState::STANDBY);
    ControllerOutputs outputs = controller.getOutputs();
    TEST_ASSERT_TRUE(outputs.isStopped);
    TEST_ASSERT_EQUAL_UINT8(0, outputs.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(0, outputs.motors.rightPwm);
    TEST_ASSERT_FALSE(outputs.indicatorActive);
}

void test_button_pressed_keeps_standby(void) {
    LineFollowerController controller;
    SensorInputs sensors = {false, false};
    controller.update(true, sensors, 100);

    TEST_ASSERT_TRUE(controller.getState() == RobotState::STANDBY);
    ControllerOutputs outputs = controller.getOutputs();
    TEST_ASSERT_TRUE(outputs.isStopped);
    TEST_ASSERT_EQUAL_UINT8(0, outputs.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(0, outputs.motors.rightPwm);
    TEST_ASSERT_FALSE(outputs.indicatorActive);
}

void test_button_released_transitions_to_racing(void) {
    LineFollowerController controller;
    SensorInputs sensors = {true, true};

    // Keep button pressed
    controller.update(true, sensors, 100);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::STANDBY);

    // Release button
    controller.update(false, sensors, 200);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RACING);
    ControllerOutputs outputs = controller.getOutputs();
    TEST_ASSERT_TRUE(outputs.indicatorActive);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_is_standby);
    RUN_TEST(test_button_pressed_keeps_standby);
    RUN_TEST(test_button_released_transitions_to_racing);
    return UNITY_END();
}
