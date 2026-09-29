#ifndef LINE_FOLLOWER_CONTROLLER_H
#define LINE_FOLLOWER_CONTROLLER_H

#include <stdint.h>

struct SensorInputs {
    bool s1_outerLeft;
    bool s2_innerLeft;
    bool s3_innerRight;
    bool s4_outerRight;
};

struct MotorOutputs {
    uint8_t leftPwm;
    uint8_t rightPwm;
    bool leftForward;
    bool rightForward;
};

struct ControllerOutputs {
    MotorOutputs motors;
    bool indicatorActive;
    bool isStopped;
};

enum class RobotState {
    STANDBY,
    RACING,
    RESCUING,
    EMERGENCY_STOP
};

enum class TrackPolarity {
    WHITE_LINE, // Línea blanca sobre fondo oscuro (Sensor activo = LOW / 0)
    BLACK_LINE  // Línea negra sobre fondo blanco (Sensor activo = HIGH / 1)
};

class LineFollowerController {
public:
    enum class ActiveTurn {
        NONE,
        LEFT,
        RIGHT
    };

    static const uint8_t BASE_SPEED = 130;
    static const uint8_t TRIM_RIGHT = 0;
    static const uint8_t TURN_REVERSE_PWM = 105; // Contramarcha en rueda interna para giro cerrado sobre su eje
    static const uint8_t CURVE_SPEED = 20;
    static const uint8_t MAX_STRAIGHT_SPEED = 180;
    static const uint32_t STRAIGHT_ACCEL_DELAY_MS = 80;
    static const uint32_t ACCEL_STEP_INTERVAL_MS = 40;
    static const uint8_t ACCEL_STEP_PWM = 10;
    static const uint32_t RESCUE_TIMEOUT_MS = 200;

    explicit LineFollowerController(TrackPolarity polarity = TrackPolarity::WHITE_LINE)
        : state_(RobotState::STANDBY),
          wasButtonPressed_(false),
          polarity_(polarity),
          activeTurn_(ActiveTurn::NONE),
          inStraightCruise_(false),
          straightStartTimeMs_(0),
          rescueStartTimeMs_(0) {
        outputs_.motors.leftPwm = 0;
        outputs_.motors.rightPwm = 0;
        outputs_.motors.leftForward = true;
        outputs_.motors.rightForward = true;
        outputs_.indicatorActive = false;
        outputs_.isStopped = true;
    }

    void setTrackPolarity(TrackPolarity polarity) {
        polarity_ = polarity;
    }

    TrackPolarity getTrackPolarity() const {
        return polarity_;
    }

    ActiveTurn getActiveTurn() const {
        return activeTurn_;
    }

    // Normalización de sensores según la polaridad de la pista para arreglo discreto de 4 sensores
    static SensorInputs normalizeSensors(int rawS1, int rawS2, int rawS3, int rawS4, TrackPolarity polarity) {
        int activeLevel = (polarity == TrackPolarity::WHITE_LINE) ? 0 : 1;
        return SensorInputs{
            rawS1 == activeLevel,
            rawS2 == activeLevel,
            rawS3 == activeLevel,
            rawS4 == activeLevel
        };
    }

    // Actualización del ciclo de control
    // Cumple con el Reglamento de Carreras:
    // - Art 2.2.3 & 4.1.1: El robot permanece inmóvil en STANDBY mientras se mantiene presionado el pulsador de largada.
    // - Art 2.2.7 & 4.1.2: Al soltarse el pulsador, pasa inmediatamente a RACING y enciende el indicador luminoso.
    // - Arreglo discreto de 4 sensores (S1..S4) con control de giro en dos etapas por discriminación espacial:
    //   * [0, 1, 1, 0]: Centrado. Crucero recto con aceleración progresiva (BASE_SPEED a MAX_STRAIGHT_SPEED).
    //   * [0, 1, 0, 0]: Desvío leve a la derecha -> Etapa 1 giro suave izquierda (interna fwd CURVE_SPEED, externa fwd BASE_SPEED).
    //   * [0, 0, 1, 0]: Desvío leve a la izquierda -> Etapa 1 giro suave derecha (externa fwd BASE_SPEED, interna fwd CURVE_SPEED).
    //   * [1, 0, 0, 0] o [1, 1, 0, 0]: Curva cerrada izquierda / 30cm radio -> Etapa 2 contramarcha inmediata.
    //   * [0, 0, 0, 1] o [0, 0, 1, 1]: Curva cerrada derecha / 30cm radio -> Etapa 2 contramarcha inmediata.
    //   * [1, 1, 1, 1]: Marca transversal / cruce -> crucero recto a BASE_SPEED.
    //   * [0, 0, 0, 0]: Pérdida de pista -> transición a RESCUING con contramarcha en última dirección (<= 200 ms).
    //     - Si recupera línea en <= 200 ms: reanuda RACING inmediatamente.
    //     - Si transcurren > 200 ms: transición a EMERGENCY_STOP (detención total, indicador apagado).
    void update(bool buttonPressed, const SensorInputs& sensors, uint32_t currentTimeMs) {
        if (state_ == RobotState::STANDBY) {
            inStraightCruise_ = false;
            straightStartTimeMs_ = 0;
            rescueStartTimeMs_ = 0;
            if (buttonPressed) {
                wasButtonPressed_ = true;
                return;
            } else if (wasButtonPressed_) {
                state_ = RobotState::RACING;
                activeTurn_ = ActiveTurn::NONE;
                inStraightCruise_ = false;
                straightStartTimeMs_ = 0;
                outputs_.indicatorActive = true;
                outputs_.isStopped = false;
                // Continúa de inmediato a evaluar RACING en este mismo ciclo
            } else {
                return;
            }
        }

        if (state_ == RobotState::EMERGENCY_STOP) {
            outputs_.isStopped = true;
            outputs_.indicatorActive = false;
            outputs_.motors.leftPwm = 0;
            outputs_.motors.rightPwm = 0;
            if (buttonPressed) {
                wasButtonPressed_ = true;
                return;
            } else if (wasButtonPressed_) {
                state_ = RobotState::RACING;
                activeTurn_ = ActiveTurn::NONE;
                inStraightCruise_ = false;
                straightStartTimeMs_ = 0;
                outputs_.indicatorActive = true;
                outputs_.isStopped = false;
            } else {
                return;
            }
        }

        if (state_ == RobotState::RESCUING) {
            bool lineDetected = sensors.s1_outerLeft || sensors.s2_innerLeft ||
                                sensors.s3_innerRight || sensors.s4_outerRight;
            if (lineDetected) {
                state_ = RobotState::RACING;
                // Continúa inmediatamente a la evaluación de RACING más abajo
            } else {
                if (currentTimeMs - rescueStartTimeMs_ > RESCUE_TIMEOUT_MS) {
                    state_ = RobotState::EMERGENCY_STOP;
                    outputs_.isStopped = true;
                    outputs_.indicatorActive = false;
                    outputs_.motors.leftPwm = 0;
                    outputs_.motors.rightPwm = 0;
                    return;
                } else {
                    // Mantener contramarcha en la última dirección detectada
                    outputs_.indicatorActive = true;
                    outputs_.isStopped = false;
                    if (activeTurn_ == ActiveTurn::LEFT) {
                        outputs_.motors.leftForward = false;
                        outputs_.motors.rightForward = true;
                        outputs_.motors.leftPwm = TURN_REVERSE_PWM;
                        outputs_.motors.rightPwm = BASE_SPEED - TRIM_RIGHT;
                    } else { // RIGHT o NONE por defecto
                        outputs_.motors.leftForward = true;
                        outputs_.motors.rightForward = false;
                        outputs_.motors.leftPwm = BASE_SPEED;
                        outputs_.motors.rightPwm = TURN_REVERSE_PWM;
                    }
                    return;
                }
            }
        }

        if (state_ == RobotState::RACING) {
            outputs_.indicatorActive = true;
            outputs_.isStopped = false;

            // 1. Pérdida total de pista [0, 0, 0, 0] -> Transición a RESCUING
            if (!sensors.s1_outerLeft && !sensors.s2_innerLeft &&
                !sensors.s3_innerRight && !sensors.s4_outerRight) {
                state_ = RobotState::RESCUING;
                rescueStartTimeMs_ = currentTimeMs;
                inStraightCruise_ = false;
                straightStartTimeMs_ = 0;

                // Aplicar contramarcha de rescate inmediatamente
                if (activeTurn_ == ActiveTurn::LEFT) {
                    outputs_.motors.leftForward = false;
                    outputs_.motors.rightForward = true;
                    outputs_.motors.leftPwm = TURN_REVERSE_PWM;
                    outputs_.motors.rightPwm = BASE_SPEED - TRIM_RIGHT;
                } else { // RIGHT o NONE por defecto
                    outputs_.motors.leftForward = true;
                    outputs_.motors.rightForward = false;
                    outputs_.motors.leftPwm = BASE_SPEED;
                    outputs_.motors.rightPwm = TURN_REVERSE_PWM;
                }
                return;
            }

            // 2. Marca transversal / Cruce [1, 1, 1, 1] o ambos extremos activos
            if (sensors.s1_outerLeft && sensors.s4_outerRight) {
                activeTurn_ = ActiveTurn::NONE;
                inStraightCruise_ = false;
                straightStartTimeMs_ = 0;
                outputs_.motors.leftForward = true;
                outputs_.motors.rightForward = true;
                outputs_.motors.leftPwm = BASE_SPEED;
                outputs_.motors.rightPwm = BASE_SPEED - TRIM_RIGHT;
                return;
            }

            // 3. Etapa 2 - Curva cerrada a la izquierda (S1 exterior izquierdo activo)
            // Cubre [1, 0, 0, 0], [1, 1, 0, 0], etc.
            if (sensors.s1_outerLeft) {
                activeTurn_ = ActiveTurn::LEFT;
                inStraightCruise_ = false;
                straightStartTimeMs_ = 0;
                outputs_.motors.leftForward = false;
                outputs_.motors.rightForward = true;
                outputs_.motors.leftPwm = TURN_REVERSE_PWM;
                outputs_.motors.rightPwm = BASE_SPEED - TRIM_RIGHT;
                return;
            }

            // 4. Etapa 2 - Curva cerrada a la derecha (S4 exterior derecho activo)
            // Cubre [0, 0, 0, 1], [0, 0, 1, 1], etc.
            if (sensors.s4_outerRight) {
                activeTurn_ = ActiveTurn::RIGHT;
                inStraightCruise_ = false;
                straightStartTimeMs_ = 0;
                outputs_.motors.leftForward = true;
                outputs_.motors.rightForward = false;
                outputs_.motors.leftPwm = BASE_SPEED;
                outputs_.motors.rightPwm = TURN_REVERSE_PWM;
                return;
            }

            // 5. Etapa 1 - Desvío leve a la derecha [0, 1, 0, 0] (giro suave izquierda)
            if (sensors.s2_innerLeft && !sensors.s3_innerRight) {
                activeTurn_ = ActiveTurn::LEFT;
                inStraightCruise_ = false;
                straightStartTimeMs_ = 0;
                outputs_.motors.leftForward = true;
                outputs_.motors.rightForward = true;
                outputs_.motors.leftPwm = CURVE_SPEED;
                outputs_.motors.rightPwm = BASE_SPEED - TRIM_RIGHT;
                return;
            }

            // 6. Etapa 1 - Desvío leve a la izquierda [0, 0, 1, 0] (giro suave derecha)
            if (!sensors.s2_innerLeft && sensors.s3_innerRight) {
                activeTurn_ = ActiveTurn::RIGHT;
                inStraightCruise_ = false;
                straightStartTimeMs_ = 0;
                outputs_.motors.leftForward = true;
                outputs_.motors.rightForward = true;
                outputs_.motors.leftPwm = BASE_SPEED;
                outputs_.motors.rightPwm = CURVE_SPEED;
                return;
            }

            // 7. Centrado [0, 1, 1, 0]: ambos sensores internos sobre la línea
            if (sensors.s2_innerLeft && sensors.s3_innerRight) {
                activeTurn_ = ActiveTurn::NONE;
                if (!inStraightCruise_) {
                    inStraightCruise_ = true;
                    straightStartTimeMs_ = currentTimeMs;
                }
                uint32_t straightElapsed = currentTimeMs - straightStartTimeMs_;
                uint8_t currentSpeed = BASE_SPEED;
                if (straightElapsed > STRAIGHT_ACCEL_DELAY_MS) {
                    uint32_t accelTime = straightElapsed - STRAIGHT_ACCEL_DELAY_MS;
                    uint32_t speedBoost = (accelTime / ACCEL_STEP_INTERVAL_MS) * ACCEL_STEP_PWM;
                    if (speedBoost > (uint32_t)(MAX_STRAIGHT_SPEED - BASE_SPEED)) {
                        currentSpeed = MAX_STRAIGHT_SPEED;
                    } else {
                        currentSpeed = (uint8_t)(BASE_SPEED + speedBoost);
                    }
                }
                outputs_.motors.leftForward = true;
                outputs_.motors.rightForward = true;
                outputs_.motors.leftPwm = currentSpeed;
                outputs_.motors.rightPwm = currentSpeed - TRIM_RIGHT;
                return;
            }
        }
    }

    void updateRaw(bool buttonPressed, int rawS1, int rawS2, int rawS3, int rawS4, uint32_t currentTimeMs) {
        update(buttonPressed, normalizeSensors(rawS1, rawS2, rawS3, rawS4, polarity_), currentTimeMs);
    }

    ControllerOutputs getOutputs() const {
        return outputs_;
    }

    RobotState getState() const {
        return state_;
    }

private:
    RobotState state_;
    bool wasButtonPressed_;
    TrackPolarity polarity_;
    ControllerOutputs outputs_;
    ActiveTurn activeTurn_;
    bool inStraightCruise_;
    uint32_t straightStartTimeMs_;
    uint32_t rescueStartTimeMs_;
};

#endif // LINE_FOLLOWER_CONTROLLER_H
