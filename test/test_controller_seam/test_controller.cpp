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
    SensorInputs sensors = {false, false};
    controller.update(true, sensors, 100);
    controller.update(false, sensors, 200);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RACING);
}

void test_straight_cruising_applies_base_speed_with_trim(void) {
    LineFollowerController controller;
    start_racing(controller);

    // Topología a horcajadas: ambos sensores en fondo negro (línea blanca en el medio)
    SensorInputs centered = {false, false};
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

    // Curva a la derecha / desvío a la izquierda: sensor derecho toca la línea blanca
    SensorInputs curveRight = {false, true};

    // Stage 1 (t = 300 ms start, check at 320 ms, elapsed 20 ms < 70 ms):
    // frenado suave inicial (PWM 0) y ambos motores hacia adelante (anti-zigzag)
    controller.update(false, curveRight, 300);
    controller.update(false, curveRight, 320);
    ControllerOutputs outStage1 = controller.getOutputs();
    TEST_ASSERT_FALSE(outStage1.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, outStage1.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, outStage1.motors.rightPwm);
    TEST_ASSERT_TRUE(outStage1.motors.leftForward);
    TEST_ASSERT_TRUE(outStage1.motors.rightForward);

    // Stage 2 (t = 380 ms, elapsed 80 ms >= 70 ms):
    // contramarcha en reversa en rueda interna derecha para giro cerrado
    controller.update(false, curveRight, 380);
    ControllerOutputs outStage2 = controller.getOutputs();
    TEST_ASSERT_FALSE(outStage2.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, outStage2.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, outStage2.motors.rightPwm);
    TEST_ASSERT_TRUE(outStage2.motors.leftForward);
    TEST_ASSERT_FALSE(outStage2.motors.rightForward);
}

void test_right_drift_steers_left_smoothly_forward(void) {
    LineFollowerController controller;
    start_racing(controller);

    // Curva a la izquierda / desvío a la derecha: sensor izquierdo toca la línea blanca
    SensorInputs curveLeft = {true, false};

    // Stage 1 (t = 300 ms start, check at 320 ms, elapsed 20 ms < 70 ms):
    // frenado suave inicial (PWM 0) y ambos motores hacia adelante (anti-zigzag)
    controller.update(false, curveLeft, 300);
    controller.update(false, curveLeft, 320);
    ControllerOutputs outStage1 = controller.getOutputs();
    TEST_ASSERT_FALSE(outStage1.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, outStage1.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, outStage1.motors.rightPwm);
    TEST_ASSERT_TRUE(outStage1.motors.leftForward);
    TEST_ASSERT_TRUE(outStage1.motors.rightForward);

    // Stage 2 (t = 380 ms, elapsed 80 ms >= 70 ms):
    // contramarcha en reversa en rueda interna izquierda para giro cerrado
    controller.update(false, curveLeft, 380);
    ControllerOutputs outStage2 = controller.getOutputs();
    TEST_ASSERT_FALSE(outStage2.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, outStage2.motors.leftPwm);
    TEST_ASSERT_FALSE(outStage2.motors.leftForward);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, outStage2.motors.rightPwm);
    TEST_ASSERT_TRUE(outStage2.motors.rightForward);
}

void test_transverse_mark_continues_straight_ahead(void) {
    LineFollowerController controller;
    start_racing(controller);

    // Marca transversal de largada/meta: ambos sensores tocan la línea blanca
    SensorInputs transverse = {true, true};
    controller.update(false, transverse, 300);

    ControllerOutputs outputs = controller.getOutputs();
    TEST_ASSERT_FALSE(outputs.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, outputs.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, outputs.motors.rightPwm);
    TEST_ASSERT_TRUE(outputs.motors.leftForward);
    TEST_ASSERT_TRUE(outputs.motors.rightForward);
}

void test_counter_rotation_strictly_excluded_in_active_cruising(void) {
    LineFollowerController controller;
    start_racing(controller);

    // En avance recto y marcas transversales ambas ruedas deben avanzar siempre hacia adelante,
    // incluso a través de múltiples ciclos de tiempo superiores a REVERSE_ENGAGEMENT_DELAY_MS
    SensorInputs forwardScenarios[] = {
        {false, false}, // Centered (line in middle)
        {true, true}    // Transverse mark
    };

    for (size_t i = 0; i < sizeof(forwardScenarios)/sizeof(forwardScenarios[0]); ++i) {
        for (uint32_t t = 300; t <= 500; t += 50) {
            controller.update(false, forwardScenarios[i], t + (uint32_t)i * 1000);
            ControllerOutputs outputs = controller.getOutputs();
            TEST_ASSERT_FALSE(outputs.isStopped);
            TEST_ASSERT_TRUE_MESSAGE(outputs.motors.leftForward, "Left motor counter-rotation detected in straight cruise!");
            TEST_ASSERT_TRUE_MESSAGE(outputs.motors.rightForward, "Right motor counter-rotation detected in straight cruise!");
        }
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
    // 1. Controller configured for WHITE_LINE (0 = white line, 1 = dark background)
    LineFollowerController whiteController(TrackPolarity::WHITE_LINE);
    start_racing(whiteController);

    // White line in middle (both sensors read 1 / HIGH on dark background) -> straight cruise
    whiteController.updateRaw(false, 1, 1, 300);
    ControllerOutputs out = whiteController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_TRUE(out.motors.rightForward);

    // Curve right Stage 1 (t = 350 ms, elapsed 0 ms < 70 ms): mild brake forward
    whiteController.updateRaw(false, 1, 0, 350);
    out = whiteController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_TRUE(out.motors.rightForward);

    // Curve right Stage 2 (t = 430 ms, elapsed 80 ms >= 70 ms): reverse counter-rotation
    whiteController.updateRaw(false, 1, 0, 430);
    out = whiteController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_FALSE(out.motors.rightForward);

    // Curve left Stage 1 (t = 500 ms, elapsed 0 ms < 70 ms): mild brake forward
    whiteController.updateRaw(false, 0, 1, 500);
    out = whiteController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_TRUE(out.motors.rightForward);

    // Curve left Stage 2 (t = 580 ms, elapsed 80 ms >= 70 ms): reverse counter-rotation
    whiteController.updateRaw(false, 0, 1, 580);
    out = whiteController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, out.motors.leftPwm);
    TEST_ASSERT_FALSE(out.motors.leftForward);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.rightForward);

    // Transverse line (both sensors read 0 / LOW) -> continues straight ahead
    whiteController.updateRaw(false, 0, 0, 650);
    out = whiteController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_TRUE(out.motors.rightForward);

    // 2. Controller configured for BLACK_LINE (1 = black line, 0 = light background)
    LineFollowerController blackController(TrackPolarity::BLACK_LINE);
    start_racing(blackController);

    // Black line in middle (both sensors read 0 / LOW on light background) -> straight cruise
    blackController.updateRaw(false, 0, 0, 300);
    out = blackController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_TRUE(out.motors.rightForward);

    // Curve right Stage 1 (t = 350 ms, elapsed 0 ms < 70 ms): mild brake forward
    blackController.updateRaw(false, 0, 1, 350);
    out = blackController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_TRUE(out.motors.rightForward);

    // Curve right Stage 2 (t = 430 ms, elapsed 80 ms >= 70 ms): reverse counter-rotation
    blackController.updateRaw(false, 0, 1, 430);
    out = blackController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_FALSE(out.motors.rightForward);

    // Curve left Stage 1 (t = 500 ms, elapsed 0 ms < 70 ms): mild brake forward
    blackController.updateRaw(false, 1, 0, 500);
    out = blackController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_TRUE(out.motors.rightForward);

    // Curve left Stage 2 (t = 580 ms, elapsed 80 ms >= 70 ms): reverse counter-rotation
    blackController.updateRaw(false, 1, 0, 580);
    out = blackController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, out.motors.leftPwm);
    TEST_ASSERT_FALSE(out.motors.leftForward);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.rightForward);

    // Transverse line (both sensors read 1 / HIGH) -> continues straight ahead
    blackController.updateRaw(false, 1, 1, 650);
    out = blackController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_TRUE(out.motors.rightForward);
}

void test_turn_timer_resets_when_re_centering(void) {
    LineFollowerController controller;
    start_racing(controller);

    SensorInputs curveRight = {false, true};
    SensorInputs centered = {false, false};

    // 1. Initial veer right at t = 300 ms -> reaches stage 2 at t = 380 ms (80 ms >= 70 ms)
    controller.update(false, curveRight, 300);
    controller.update(false, curveRight, 380);
    ControllerOutputs out = controller.getOutputs();
    TEST_ASSERT_FALSE(out.motors.rightForward); // In stage 2 reverse

    // 2. Robot successfully re-centers onto line at t = 400 ms
    controller.update(false, centered, 400);
    out = controller.getOutputs();
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_TRUE(out.motors.rightForward);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);

    // 3. Robot veers right again at t = 450 ms -> should enter Stage 1, NOT Stage 2!
    controller.update(false, curveRight, 450);
    controller.update(false, curveRight, 470); // elapsed 20 ms < 70 ms
    out = controller.getOutputs();
    TEST_ASSERT_TRUE_MESSAGE(out.motors.rightForward, "Turn timer was not reset upon re-centering; premature reverse engaged!");
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, out.motors.rightPwm);

    // 4. Staying on curve until t = 530 ms (80 ms >= 70 ms) engages Stage 2 cleanly
    controller.update(false, curveRight, 530);
    out = controller.getOutputs();
    TEST_ASSERT_FALSE(out.motors.rightForward);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, out.motors.rightPwm);
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
    RUN_TEST(test_transverse_mark_continues_straight_ahead);
    RUN_TEST(test_counter_rotation_strictly_excluded_in_active_cruising);
    RUN_TEST(test_unified_track_polarity_normalization);
    RUN_TEST(test_navigation_commands_across_both_track_polarities);
    RUN_TEST(test_turn_timer_resets_when_re_centering);
    return UNITY_END();
}
