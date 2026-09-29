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

static void start_racing(LineFollowerController& controller) {
    SensorInputs centered = {false, true, true, false};
    controller.update(true, centered, 100);
    controller.update(false, centered, 200);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RACING);
}

void test_initial_state_is_standby(void) {
    LineFollowerController controller;
    assert_standby_outputs(controller);
}

void test_unpressed_button_on_boot_keeps_standby(void) {
    LineFollowerController controller;
    SensorInputs sensors = {false, false, false, false};

    for (uint32_t t = 0; t < 1000; t += 100) {
        controller.update(false, sensors, t);
        assert_standby_outputs(controller);
    }
}

void test_standby_ignores_sensor_readings_both_unpressed_and_held(void) {
    LineFollowerController controller;

    // Line detected across all sensors while button is unpressed
    SensorInputs sensorsAll = {true, true, true, true};
    controller.update(false, sensorsAll, 100);
    assert_standby_outputs(controller);

    // Line detected while driver holds button down on starting grid
    controller.update(true, sensorsAll, 200);
    assert_standby_outputs(controller);

    // Single sensor detections while driver holds button down
    SensorInputs sensorsS1 = {true, false, false, false};
    controller.update(true, sensorsS1, 300);
    assert_standby_outputs(controller);

    SensorInputs sensorsS4 = {false, false, false, true};
    controller.update(true, sensorsS4, 400);
    assert_standby_outputs(controller);
}

void test_button_pressed_and_held_maintains_standby(void) {
    LineFollowerController controller;
    SensorInputs sensors = {false, false, false, false};

    for (uint32_t t = 100; t <= 3000; t += 500) {
        controller.update(true, sensors, t);
        assert_standby_outputs(controller);
    }
}

void test_button_released_transitions_immediately_to_racing_with_indicator(void) {
    LineFollowerController controller;
    SensorInputs centered = {false, true, true, false};

    // Driver holds button on starting grid
    controller.update(true, centered, 1000);
    assert_standby_outputs(controller);

    // Referee gives start signal -> Driver releases button
    controller.update(false, centered, 1050);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RACING);

    ControllerOutputs outputs = controller.getOutputs();
    TEST_ASSERT_TRUE(outputs.indicatorActive);
    TEST_ASSERT_FALSE(outputs.isStopped);
}

void test_racing_state_persists_after_button_release(void) {
    LineFollowerController controller;
    SensorInputs centered = {false, true, true, false};

    controller.update(true, centered, 100);
    controller.update(false, centered, 200);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RACING);

    for (uint32_t t = 300; t <= 1000; t += 100) {
        controller.update(false, centered, t);
        TEST_ASSERT_TRUE(controller.getState() == RobotState::RACING);
        ControllerOutputs outputs = controller.getOutputs();
        TEST_ASSERT_TRUE(outputs.indicatorActive);
        TEST_ASSERT_FALSE(outputs.isStopped);
    }
}

void test_centered_cruising_applies_base_speed_with_trim(void) {
    LineFollowerController controller;
    start_racing(controller);

    // [0, 1, 1, 0]: Ambos sensores internos sobre la línea
    SensorInputs centered = {false, true, true, false};
    controller.update(false, centered, 300);

    ControllerOutputs outputs = controller.getOutputs();
    TEST_ASSERT_FALSE(outputs.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, outputs.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, outputs.motors.rightPwm);
    TEST_ASSERT_TRUE(outputs.motors.leftForward);
    TEST_ASSERT_TRUE(outputs.motors.rightForward);
}

void test_straightaway_progressive_acceleration(void) {
    LineFollowerController controller;
    SensorInputs centered = {false, true, true, false};

    controller.update(true, centered, 200);
    controller.update(false, centered, 300);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RACING);

    // t = 350 ms (elapsed 50 ms <= STRAIGHT_ACCEL_DELAY_MS 80 ms): velocidad base
    controller.update(false, centered, 350);
    ControllerOutputs out350 = controller.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out350.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out350.motors.rightPwm);
    TEST_ASSERT_TRUE(out350.motors.leftForward);
    TEST_ASSERT_TRUE(out350.motors.rightForward);

    // t = 420 ms (elapsed 120 ms -> accelTime = 40 ms): +10 PWM -> 140
    controller.update(false, centered, 420);
    ControllerOutputs out420 = controller.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(140, out420.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(140 - LineFollowerController::TRIM_RIGHT, out420.motors.rightPwm);
    TEST_ASSERT_TRUE(out420.motors.leftForward);
    TEST_ASSERT_TRUE(out420.motors.rightForward);

    // t = 460 ms (elapsed 160 ms -> accelTime = 80 ms): +20 PWM -> 150
    controller.update(false, centered, 460);
    ControllerOutputs out460 = controller.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(150, out460.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(150 - LineFollowerController::TRIM_RIGHT, out460.motors.rightPwm);

    // t = 500 ms (elapsed 200 ms -> accelTime = 120 ms): +30 PWM -> 160
    controller.update(false, centered, 500);
    ControllerOutputs out500 = controller.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(160, out500.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(160 - LineFollowerController::TRIM_RIGHT, out500.motors.rightPwm);

    // t = 700 ms (elapsed 400 ms -> accelTime = 320 ms): topado en MAX_STRAIGHT_SPEED 180
    controller.update(false, centered, 700);
    ControllerOutputs out700 = controller.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::MAX_STRAIGHT_SPEED, out700.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::MAX_STRAIGHT_SPEED - LineFollowerController::TRIM_RIGHT, out700.motors.rightPwm);
}

void test_etapa1_soft_differential_steering_drift_right(void) {
    LineFollowerController controller;
    start_racing(controller);

    // [0, 1, 0, 0]: Desvío leve a la derecha -> rueda interna izq forward a CURVE_SPEED, externa der forward a BASE_SPEED
    SensorInputs driftRight = {false, true, false, false};
    controller.update(false, driftRight, 300);

    ControllerOutputs out = controller.getOutputs();
    TEST_ASSERT_FALSE(out.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_TRUE(out.motors.rightForward);
    TEST_ASSERT_TRUE(controller.getActiveTurn() == LineFollowerController::ActiveTurn::LEFT);
}

void test_etapa1_soft_differential_steering_drift_left(void) {
    LineFollowerController controller;
    start_racing(controller);

    // [0, 0, 1, 0]: Desvío leve a la izquierda -> externa izq forward a BASE_SPEED, interna der forward a CURVE_SPEED
    SensorInputs driftLeft = {false, false, true, false};
    controller.update(false, driftLeft, 300);

    ControllerOutputs out = controller.getOutputs();
    TEST_ASSERT_FALSE(out.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, out.motors.rightPwm);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_TRUE(out.motors.rightForward);
    TEST_ASSERT_TRUE(controller.getActiveTurn() == LineFollowerController::ActiveTurn::RIGHT);
}

void test_etapa2_sharp_steering_left_immediate_counter_rotation(void) {
    LineFollowerController controller;
    start_racing(controller);

    // [1, 0, 0, 0]: Curva cerrada izquierda (S1 exterior izquierdo activo)
    SensorInputs sharpLeft1 = {true, false, false, false};
    controller.update(false, sharpLeft1, 300);

    ControllerOutputs out1 = controller.getOutputs();
    TEST_ASSERT_FALSE(out1.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, out1.motors.leftPwm);
    TEST_ASSERT_FALSE(out1.motors.leftForward); // Contramarcha rueda interna
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out1.motors.rightPwm);
    TEST_ASSERT_TRUE(out1.motors.rightForward); // Rueda externa hacia adelante
    TEST_ASSERT_TRUE(controller.getActiveTurn() == LineFollowerController::ActiveTurn::LEFT);

    // [1, 1, 0, 0]: Curva cerrada izquierda con S1 y S2 activos
    SensorInputs sharpLeft2 = {true, true, false, false};
    controller.update(false, sharpLeft2, 310);

    ControllerOutputs out2 = controller.getOutputs();
    TEST_ASSERT_FALSE(out2.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, out2.motors.leftPwm);
    TEST_ASSERT_FALSE(out2.motors.leftForward);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out2.motors.rightPwm);
    TEST_ASSERT_TRUE(out2.motors.rightForward);
    TEST_ASSERT_TRUE(controller.getActiveTurn() == LineFollowerController::ActiveTurn::LEFT);
}

void test_etapa2_sharp_steering_right_immediate_counter_rotation(void) {
    LineFollowerController controller;
    start_racing(controller);

    // [0, 0, 0, 1]: Curva cerrada derecha (S4 exterior derecho activo)
    SensorInputs sharpRight1 = {false, false, false, true};
    controller.update(false, sharpRight1, 300);

    ControllerOutputs out1 = controller.getOutputs();
    TEST_ASSERT_FALSE(out1.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out1.motors.leftPwm);
    TEST_ASSERT_TRUE(out1.motors.leftForward); // Rueda externa hacia adelante
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, out1.motors.rightPwm);
    TEST_ASSERT_FALSE(out1.motors.rightForward); // Contramarcha rueda interna
    TEST_ASSERT_TRUE(controller.getActiveTurn() == LineFollowerController::ActiveTurn::RIGHT);

    // [0, 0, 1, 1]: Curva cerrada derecha con S3 y S4 activos
    SensorInputs sharpRight2 = {false, false, true, true};
    controller.update(false, sharpRight2, 310);

    ControllerOutputs out2 = controller.getOutputs();
    TEST_ASSERT_FALSE(out2.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out2.motors.leftPwm);
    TEST_ASSERT_TRUE(out2.motors.leftForward);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, out2.motors.rightPwm);
    TEST_ASSERT_FALSE(out2.motors.rightForward);
    TEST_ASSERT_TRUE(controller.getActiveTurn() == LineFollowerController::ActiveTurn::RIGHT);
}

void test_curve_entry_instantly_cancels_straightaway_speed_boost(void) {
    LineFollowerController controller;
    SensorInputs centered = {false, true, true, false};

    controller.update(true, centered, 200);
    controller.update(false, centered, 300);

    // Acelerar en recta hasta velocidad máxima (180)
    controller.update(false, centered, 1000);
    ControllerOutputs outCruise = controller.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::MAX_STRAIGHT_SPEED, outCruise.motors.leftPwm);

    // Entrada a curva suave a la derecha [0, 0, 1, 0]
    SensorInputs driftLeft = {false, false, true, false};
    controller.update(false, driftLeft, 1010);
    ControllerOutputs outCurve = controller.getOutputs();

    // Velocidad se resetea de inmediato a BASE_SPEED (130) en externa, CURVE_SPEED en interna
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, outCurve.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::CURVE_SPEED, outCurve.motors.rightPwm);
}

void test_transverse_mark_continues_straight_ahead(void) {
    LineFollowerController controller;
    start_racing(controller);

    // [1, 1, 1, 1]: Marca transversal / largada
    SensorInputs transverse = {true, true, true, true};
    controller.update(false, transverse, 300);

    ControllerOutputs outputs = controller.getOutputs();
    TEST_ASSERT_FALSE(outputs.isStopped);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, outputs.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, outputs.motors.rightPwm);
    TEST_ASSERT_TRUE(outputs.motors.leftForward);
    TEST_ASSERT_TRUE(outputs.motors.rightForward);
}

void test_rescuing_state_entry_and_maintains_last_detected_direction(void) {
    // 1. Rescate tras giro a la derecha
    {
        LineFollowerController controller;
        start_racing(controller);

        SensorInputs curveRight = {false, false, true, false};
        controller.update(false, curveRight, 300);
        TEST_ASSERT_TRUE(controller.getActiveTurn() == LineFollowerController::ActiveTurn::RIGHT);

        // Pérdida total de pista [0, 0, 0, 0] a t = 350 ms
        SensorInputs lost = {false, false, false, false};
        controller.update(false, lost, 350);

        TEST_ASSERT_TRUE(controller.getState() == RobotState::RESCUING);
        ControllerOutputs out = controller.getOutputs();
        TEST_ASSERT_FALSE(out.isStopped);
        TEST_ASSERT_TRUE(out.indicatorActive);
        // Gira hacia la derecha (rueda izq fwd a BASE_SPEED, rueda der rev a TURN_REVERSE_PWM)
        TEST_ASSERT_TRUE(out.motors.leftForward);
        TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
        TEST_ASSERT_FALSE(out.motors.rightForward);
        TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, out.motors.rightPwm);
    }

    // 2. Rescate tras giro a la izquierda
    {
        LineFollowerController controller;
        start_racing(controller);

        SensorInputs curveLeft = {false, true, false, false};
        controller.update(false, curveLeft, 300);
        TEST_ASSERT_TRUE(controller.getActiveTurn() == LineFollowerController::ActiveTurn::LEFT);

        // Pérdida total de pista [0, 0, 0, 0] a t = 350 ms
        SensorInputs lost = {false, false, false, false};
        controller.update(false, lost, 350);

        TEST_ASSERT_TRUE(controller.getState() == RobotState::RESCUING);
        ControllerOutputs out = controller.getOutputs();
        TEST_ASSERT_FALSE(out.isStopped);
        TEST_ASSERT_TRUE(out.indicatorActive);
        // Gira hacia la izquierda (rueda izq rev a TURN_REVERSE_PWM, rueda der fwd a BASE_SPEED - TRIM_RIGHT)
        TEST_ASSERT_FALSE(out.motors.leftForward);
        TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, out.motors.leftPwm);
        TEST_ASSERT_TRUE(out.motors.rightForward);
        TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);
    }
}

void test_rescuing_line_recovery_resumes_racing_immediately(void) {
    LineFollowerController controller;
    start_racing(controller);

    SensorInputs curveRight = {false, false, false, true};
    controller.update(false, curveRight, 300);

    // Despiste [0, 0, 0, 0] a t = 350 ms
    SensorInputs lost = {false, false, false, false};
    controller.update(false, lost, 350);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RESCUING);

    // Recupera línea a t = 450 ms (100 ms transcurridos < 200 ms) con centrado [0, 1, 1, 0]
    SensorInputs recovered = {false, true, true, false};
    controller.update(false, recovered, 450);

    TEST_ASSERT_TRUE(controller.getState() == RobotState::RACING);
    ControllerOutputs out = controller.getOutputs();
    TEST_ASSERT_FALSE(out.isStopped);
    TEST_ASSERT_TRUE(out.indicatorActive);
    TEST_ASSERT_TRUE(out.motors.leftForward);
    TEST_ASSERT_TRUE(out.motors.rightForward);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, out.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, out.motors.rightPwm);
}

void test_emergency_stop_after_rescue_timeout_exceeded(void) {
    LineFollowerController controller;
    start_racing(controller);

    SensorInputs curveLeft = {true, false, false, false};
    controller.update(false, curveLeft, 300);

    // Despiste [0, 0, 0, 0] a t = 350 ms -> rescueStartTimeMs = 350
    SensorInputs lost = {false, false, false, false};
    controller.update(false, lost, 350);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RESCUING);

    // A t = 540 ms (190 ms <= 200 ms): continúa en RESCUING
    controller.update(false, lost, 540);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RESCUING);
    TEST_ASSERT_FALSE(controller.getOutputs().isStopped);
    TEST_ASSERT_TRUE(controller.getOutputs().indicatorActive);

    // A t = 550 ms (200 ms <= 200 ms): continúa en RESCUING
    controller.update(false, lost, 550);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::RESCUING);

    // A t = 551 ms (201 ms > 200 ms): TRANSICIÓN A EMERGENCY_STOP
    controller.update(false, lost, 551);
    TEST_ASSERT_TRUE(controller.getState() == RobotState::EMERGENCY_STOP);
    ControllerOutputs outStop = controller.getOutputs();
    TEST_ASSERT_TRUE(outStop.isStopped);
    TEST_ASSERT_FALSE(outStop.indicatorActive);
    TEST_ASSERT_EQUAL_UINT8(0, outStop.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(0, outStop.motors.rightPwm);
}

void test_unified_track_polarity_normalization(void) {
    // WHITE_LINE: 0 (LOW) es línea detectada, 1 (HIGH) es fondo oscuro
    SensorInputs wCentered = LineFollowerController::normalizeSensors(1, 0, 0, 1, TrackPolarity::WHITE_LINE);
    TEST_ASSERT_FALSE(wCentered.s1_outerLeft);
    TEST_ASSERT_TRUE(wCentered.s2_innerLeft);
    TEST_ASSERT_TRUE(wCentered.s3_innerRight);
    TEST_ASSERT_FALSE(wCentered.s4_outerRight);

    SensorInputs wDriftRight = LineFollowerController::normalizeSensors(1, 0, 1, 1, TrackPolarity::WHITE_LINE);
    TEST_ASSERT_FALSE(wDriftRight.s1_outerLeft);
    TEST_ASSERT_TRUE(wDriftRight.s2_innerLeft);
    TEST_ASSERT_FALSE(wDriftRight.s3_innerRight);
    TEST_ASSERT_FALSE(wDriftRight.s4_outerRight);

    SensorInputs wSharpLeft = LineFollowerController::normalizeSensors(0, 1, 1, 1, TrackPolarity::WHITE_LINE);
    TEST_ASSERT_TRUE(wSharpLeft.s1_outerLeft);
    TEST_ASSERT_FALSE(wSharpLeft.s2_innerLeft);
    TEST_ASSERT_FALSE(wSharpLeft.s3_innerRight);
    TEST_ASSERT_FALSE(wSharpLeft.s4_outerRight);

    SensorInputs wTransverse = LineFollowerController::normalizeSensors(0, 0, 0, 0, TrackPolarity::WHITE_LINE);
    TEST_ASSERT_TRUE(wTransverse.s1_outerLeft);
    TEST_ASSERT_TRUE(wTransverse.s2_innerLeft);
    TEST_ASSERT_TRUE(wTransverse.s3_innerRight);
    TEST_ASSERT_TRUE(wTransverse.s4_outerRight);

    SensorInputs wLost = LineFollowerController::normalizeSensors(1, 1, 1, 1, TrackPolarity::WHITE_LINE);
    TEST_ASSERT_FALSE(wLost.s1_outerLeft);
    TEST_ASSERT_FALSE(wLost.s2_innerLeft);
    TEST_ASSERT_FALSE(wLost.s3_innerRight);
    TEST_ASSERT_FALSE(wLost.s4_outerRight);

    // BLACK_LINE: 1 (HIGH) es línea detectada, 0 (LOW) es fondo claro
    SensorInputs bCentered = LineFollowerController::normalizeSensors(0, 1, 1, 0, TrackPolarity::BLACK_LINE);
    TEST_ASSERT_FALSE(bCentered.s1_outerLeft);
    TEST_ASSERT_TRUE(bCentered.s2_innerLeft);
    TEST_ASSERT_TRUE(bCentered.s3_innerRight);
    TEST_ASSERT_FALSE(bCentered.s4_outerRight);

    SensorInputs bDriftRight = LineFollowerController::normalizeSensors(0, 1, 0, 0, TrackPolarity::BLACK_LINE);
    TEST_ASSERT_FALSE(bDriftRight.s1_outerLeft);
    TEST_ASSERT_TRUE(bDriftRight.s2_innerLeft);
    TEST_ASSERT_FALSE(bDriftRight.s3_innerRight);
    TEST_ASSERT_FALSE(bDriftRight.s4_outerRight);

    SensorInputs bSharpRight = LineFollowerController::normalizeSensors(0, 0, 0, 1, TrackPolarity::BLACK_LINE);
    TEST_ASSERT_FALSE(bSharpRight.s1_outerLeft);
    TEST_ASSERT_FALSE(bSharpRight.s2_innerLeft);
    TEST_ASSERT_FALSE(bSharpRight.s3_innerRight);
    TEST_ASSERT_TRUE(bSharpRight.s4_outerRight);

    SensorInputs bLost = LineFollowerController::normalizeSensors(0, 0, 0, 0, TrackPolarity::BLACK_LINE);
    TEST_ASSERT_FALSE(bLost.s1_outerLeft);
    TEST_ASSERT_FALSE(bLost.s2_innerLeft);
    TEST_ASSERT_FALSE(bLost.s3_innerRight);
    TEST_ASSERT_FALSE(bLost.s4_outerRight);
}

void test_navigation_commands_across_both_track_polarities(void) {
    // 1. WHITE_LINE (0 = activo, 1 = inactivo)
    LineFollowerController whiteController(TrackPolarity::WHITE_LINE);
    start_racing(whiteController);

    // Centrado: S1=1, S2=0, S3=0, S4=1
    whiteController.updateRaw(false, 1, 0, 0, 1, 300);
    ControllerOutputs outW = whiteController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, outW.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, outW.motors.rightPwm);
    TEST_ASSERT_TRUE(outW.motors.leftForward);
    TEST_ASSERT_TRUE(outW.motors.rightForward);

    // Curva cerrada derecha S4 activo: S1=1, S2=1, S3=1, S4=0
    whiteController.updateRaw(false, 1, 1, 1, 0, 350);
    outW = whiteController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, outW.motors.leftPwm);
    TEST_ASSERT_TRUE(outW.motors.leftForward);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, outW.motors.rightPwm);
    TEST_ASSERT_FALSE(outW.motors.rightForward);

    // 2. BLACK_LINE (1 = activo, 0 = inactivo)
    LineFollowerController blackController(TrackPolarity::BLACK_LINE);
    start_racing(blackController);

    // Centrado: S1=0, S2=1, S3=1, S4=0
    blackController.updateRaw(false, 0, 1, 1, 0, 300);
    ControllerOutputs outB = blackController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED, outB.motors.leftPwm);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, outB.motors.rightPwm);
    TEST_ASSERT_TRUE(outB.motors.leftForward);
    TEST_ASSERT_TRUE(outB.motors.rightForward);

    // Curva cerrada izquierda S1 activo: S1=1, S2=0, S3=0, S4=0
    blackController.updateRaw(false, 1, 0, 0, 0, 350);
    outB = blackController.getOutputs();
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::TURN_REVERSE_PWM, outB.motors.leftPwm);
    TEST_ASSERT_FALSE(outB.motors.leftForward);
    TEST_ASSERT_EQUAL_UINT8(LineFollowerController::BASE_SPEED - LineFollowerController::TRIM_RIGHT, outB.motors.rightPwm);
    TEST_ASSERT_TRUE(outB.motors.rightForward);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_is_standby);
    RUN_TEST(test_unpressed_button_on_boot_keeps_standby);
    RUN_TEST(test_standby_ignores_sensor_readings_both_unpressed_and_held);
    RUN_TEST(test_button_pressed_and_held_maintains_standby);
    RUN_TEST(test_button_released_transitions_immediately_to_racing_with_indicator);
    RUN_TEST(test_racing_state_persists_after_button_release);
    RUN_TEST(test_centered_cruising_applies_base_speed_with_trim);
    RUN_TEST(test_straightaway_progressive_acceleration);
    RUN_TEST(test_etapa1_soft_differential_steering_drift_right);
    RUN_TEST(test_etapa1_soft_differential_steering_drift_left);
    RUN_TEST(test_etapa2_sharp_steering_left_immediate_counter_rotation);
    RUN_TEST(test_etapa2_sharp_steering_right_immediate_counter_rotation);
    RUN_TEST(test_curve_entry_instantly_cancels_straightaway_speed_boost);
    RUN_TEST(test_transverse_mark_continues_straight_ahead);
    RUN_TEST(test_rescuing_state_entry_and_maintains_last_detected_direction);
    RUN_TEST(test_rescuing_line_recovery_resumes_racing_immediately);
    RUN_TEST(test_emergency_stop_after_rescue_timeout_exceeded);
    RUN_TEST(test_unified_track_polarity_normalization);
    RUN_TEST(test_navigation_commands_across_both_track_polarities);
    return UNITY_END();
}
