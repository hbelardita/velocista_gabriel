#ifndef LINE_FOLLOWER_CONTROLLER_H
#define LINE_FOLLOWER_CONTROLLER_H

#include <stdint.h>

struct SensorInputs {
    bool leftDetected;
    bool rightDetected;
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
    static const uint8_t BASE_SPEED = 160;
    static const uint8_t TRIM_RIGHT = 20;
    static const uint8_t CURVE_SPEED = 100; // ~39.2% PWM (~35-40% PWM)

    explicit LineFollowerController(TrackPolarity polarity = TrackPolarity::WHITE_LINE)
        : state_(RobotState::STANDBY),
          wasButtonPressed_(false),
          polarity_(polarity) {
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

    // Normalización de sensores según la polaridad de la pista sin duplicar lógica
    static SensorInputs normalizeSensors(int rawLeft, int rawRight, TrackPolarity polarity) {
        int activeLevel = (polarity == TrackPolarity::WHITE_LINE) ? 0 : 1;
        return SensorInputs{
            rawLeft == activeLevel,
            rawRight == activeLevel
        };
    }

    // Actualización del ciclo de control
    // Cumple con el Reglamento de Carreras y ADRs 0001, 0002, 0003:
    // - Art 2.2.3 & 4.1.1: El robot permanece inmóvil en STANDBY mientras se mantiene presionado el pulsador de largada.
    // - Art 2.2.7 & 4.1.2: Al soltarse el pulsador, pasa inmediatamente a RACING y enciende el indicador luminoso.
    // - Tracción diferencial hacia adelante sin contramarcha ni freno motor agresivo (ADR 0002).
    void update(bool buttonPressed, const SensorInputs& sensors, uint32_t currentTimeMs) {
        (void)currentTimeMs;

        if (state_ == RobotState::STANDBY) {
            if (buttonPressed) {
                wasButtonPressed_ = true;
            } else if (wasButtonPressed_) {
                // El pulsador fue presionado y ahora es liberado -> inicio inmediato de Rutina de Carrera
                state_ = RobotState::RACING;
                outputs_.indicatorActive = true;
                outputs_.isStopped = false;
            }
        }

        if (state_ == RobotState::RACING) {
            outputs_.indicatorActive = true;
            outputs_.isStopped = false;
            outputs_.motors.leftForward = true;
            outputs_.motors.rightForward = true;

            if (sensors.leftDetected && sensors.rightDetected) {
                // Centrado: avance recto a velocidad base con compensación de trim derecho
                outputs_.motors.leftPwm = BASE_SPEED;
                outputs_.motors.rightPwm = BASE_SPEED - TRIM_RIGHT;
            } else if (!sensors.leftDetected && sensors.rightDetected) {
                // Desvío a la izquierda: rueda interna (derecha) frena a CURVE_SPEED, rueda externa a velocidad base
                outputs_.motors.leftPwm = BASE_SPEED;
                outputs_.motors.rightPwm = CURVE_SPEED;
            } else if (sensors.leftDetected && !sensors.rightDetected) {
                // Desvío a la derecha: rueda interna (izquierda) frena a CURVE_SPEED, rueda externa a velocidad base con trim
                outputs_.motors.leftPwm = CURVE_SPEED;
                outputs_.motors.rightPwm = BASE_SPEED - TRIM_RIGHT;
            }
            // Nota: Pérdida total de ambos sensores será gestionada por el módulo de Despiste y Rescate (Issue 04)
        }
    }

    void updateRaw(bool buttonPressed, int rawLeft, int rawRight, uint32_t currentTimeMs) {
        update(buttonPressed, normalizeSensors(rawLeft, rawRight, polarity_), currentTimeMs);
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
};

#endif // LINE_FOLLOWER_CONTROLLER_H
