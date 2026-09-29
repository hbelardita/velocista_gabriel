#include <Arduino.h>
#include "LineFollowerController.h"

// Pines de control del Motor Izquierdo (Driver A físicamente conectado al motor izquierdo)
const int ENA = 5;  // Pin PWM para velocidad izquierda
const int IN1 = 4;  // Dirección izquierda
const int IN2 = 8;  // Dirección izquierda

// Pines de control del Motor Derecho (Driver B físicamente conectado al motor derecho)
const int ENB = 6;  // Pin PWM para velocidad derecha
const int IN3 = 10; // Dirección derecha
const int IN4 = 9;  // Dirección derecha

// Pines de los sensores TCRT5000 (Arreglo discreto de 4 sensores)
const int SENSOR_1_EXT_IZQ = A0;
const int SENSOR_2_INT_IZQ = 2;
const int SENSOR_3_INT_DER = 3;
const int SENSOR_4_EXT_DER = A1;

// Pin del pulsador de largada (término canónico según CONTEXT.md)
const int PIN_PULSADOR_LARGADA = 7;

// Pin del indicador luminoso de carrera (Reglamento Art 2.2.7 y 4.1.2)
const int PIN_INDICADOR = LED_BUILTIN;

// Configuración unificada de polaridad de pista (WHITE_LINE o BLACK_LINE)
const TrackPolarity POLARIDAD_PISTA = TrackPolarity::WHITE_LINE;

// Instancia del controlador de dominio puro
LineFollowerController controller(POLARIDAD_PISTA);

void setMotor(int pinIn1, int pinIn2, int pinEn, bool forward, uint8_t pwm) {
  if (pwm == 0) {
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
    analogWrite(pinEn, 0);
  } else {
    digitalWrite(pinIn1, forward ? HIGH : LOW);
    digitalWrite(pinIn2, forward ? LOW : HIGH);
    analogWrite(pinEn, pwm);
  }
}

void applyOutputs(const ControllerOutputs& outputs) {
  // Indicador luminoso de estado de carrera
  digitalWrite(PIN_INDICADOR, outputs.indicatorActive ? HIGH : LOW);

  // Control de motores
  if (outputs.isStopped) {
    setMotor(IN1, IN2, ENA, true, 0);
    setMotor(IN3, IN4, ENB, true, 0);
  } else {
    setMotor(IN1, IN2, ENA, outputs.motors.leftForward, outputs.motors.leftPwm);
    setMotor(IN3, IN4, ENB, outputs.motors.rightForward, outputs.motors.rightPwm);
  }
}

#ifdef DEBUG_MODE
const uint8_t DEBUG_MOTOR_PWM = 150;
bool telemetryActive = false;
unsigned long lastTelemetryMs = 0;

void printDebugMenu() {
  Serial.println(F("\n=========================================="));
  Serial.println(F("   VELOCISTA GABRIEL - MODO DEBUG (4 SENSORES)"));
  Serial.println(F("=========================================="));
  Serial.println(F("[1] Motor Izquierdo adelante (PWM 150)"));
  Serial.println(F("[2] Motor Derecho adelante (PWM 150)"));
  Serial.println(F("[3] Ambos Motores adelante (PWM 150)"));
  Serial.println(F("[s] Parar todos los motores"));
  Serial.println(F("[4] Monitoreo continuo 4 Sensores y Pulsador"));
  Serial.println(F("[5] Test recta con trim (simula centrado [0,1,1,0])"));
  Serial.println(F("[m] Mostrar este menu"));
  Serial.println(F("=========================================="));
  Serial.print(F("Seleccione opcion: "));
}

void setupDebug() {
  Serial.begin(115200);
  printDebugMenu();
}

void loopDebug() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      return;
    }

    if (telemetryActive) {
      telemetryActive = false;
      Serial.println(F("\n[INFO] Monitoreo detenido."));
      printDebugMenu();
      return;
    }

    switch (c) {
      case '1':
        setMotor(IN1, IN2, ENA, true, DEBUG_MOTOR_PWM);
        setMotor(IN3, IN4, ENB, true, 0);
        Serial.println(F("\n[MOTOR] Izquierdo: ON (PWM 150) | Derecho: OFF. Presione 's' para parar."));
        break;

      case '2':
        setMotor(IN1, IN2, ENA, true, 0);
        setMotor(IN3, IN4, ENB, true, DEBUG_MOTOR_PWM);
        Serial.println(F("\n[MOTOR] Izquierdo: OFF | Derecho: ON (PWM 150). Presione 's' para parar."));
        break;

      case '3':
        setMotor(IN1, IN2, ENA, true, DEBUG_MOTOR_PWM);
        setMotor(IN3, IN4, ENB, true, DEBUG_MOTOR_PWM);
        Serial.println(F("\n[MOTOR] Ambos Motores: ON (PWM 150). Presione 's' para parar."));
        break;

      case 's':
      case 'S':
        setMotor(IN1, IN2, ENA, true, 0);
        setMotor(IN3, IN4, ENB, true, 0);
        Serial.println(F("\n[MOTOR] Motores DETENIDOS."));
        break;

      case '4':
        telemetryActive = true;
        lastTelemetryMs = 0;
        Serial.println(F("\n[TELEMETRIA] Iniciando lectura cada 200 ms (presione cualquier tecla para salir):"));
        break;

      case '5':
        // Simular estado RACING con sensores centrados [0, 1, 1, 0] para ver PWM con trim
        // WHITE_LINE: 0 = línea, 1 = fondo -> raw: S1=1, S2=0, S3=0, S4=1
        controller.updateRaw(false, 1, 0, 0, 1, millis());
        {
          ControllerOutputs out = controller.getOutputs();
          Serial.print(F("\n[TRIM TEST] Estado: "));
          switch (controller.getState()) {
            case RobotState::STANDBY: Serial.print(F("STANDBY")); break;
            case RobotState::RACING: Serial.print(F("RACING")); break;
            case RobotState::RESCUING: Serial.print(F("RESCUING")); break;
            case RobotState::EMERGENCY_STOP: Serial.print(F("EMERGENCY_STOP")); break;
          }
          Serial.print(F(" | PWM Izq: "));
          Serial.print(out.motors.leftPwm);
          Serial.print(out.motors.leftForward ? F(" FWD") : F(" REV"));
          Serial.print(F(" | PWM Der: "));
          Serial.print(out.motors.rightPwm);
          Serial.print(out.motors.rightForward ? F(" FWD") : F(" REV"));
          Serial.print(F(" | Trim aplicado: "));
          Serial.print(LineFollowerController::BASE_SPEED - out.motors.rightPwm);
          Serial.println();
        }
        break;

      case 'm':
      case 'M':
      case 'h':
      case 'H':
        printDebugMenu();
        break;

      default:
        Serial.print(F("\n[AVISO] Comando no reconocido: '"));
        Serial.print(c);
        Serial.println(F("'. Presione 'm' para ver el menu."));
        break;
    }
  }

  if (telemetryActive) {
    unsigned long now = millis();
    if (now - lastTelemetryMs >= 200) {
      lastTelemetryMs = now;

      bool pulsadorPresionado = (digitalRead(PIN_PULSADOR_LARGADA) == LOW);
      int rawS1 = digitalRead(SENSOR_1_EXT_IZQ);
      int rawS2 = digitalRead(SENSOR_2_INT_IZQ);
      int rawS3 = digitalRead(SENSOR_3_INT_DER);
      int rawS4 = digitalRead(SENSOR_4_EXT_DER);
      SensorInputs norm = LineFollowerController::normalizeSensors(rawS1, rawS2, rawS3, rawS4, POLARIDAD_PISTA);

      ControllerOutputs out = controller.getOutputs();

      Serial.print(F("[SENSORES] S1:"));
      Serial.print(rawS1);
      Serial.print(norm.s1_outerLeft ? F("(L)") : F("(F)"));
      Serial.print(F(" S2:"));
      Serial.print(rawS2);
      Serial.print(norm.s2_innerLeft ? F("(L)") : F("(F)"));
      Serial.print(F(" S3:"));
      Serial.print(rawS3);
      Serial.print(norm.s3_innerRight ? F("(L)") : F("(F)"));
      Serial.print(F(" S4:"));
      Serial.print(rawS4);
      Serial.print(norm.s4_outerRight ? F("(L)") : F("(F)"));
      Serial.print(F(" | Pulsador: "));
      Serial.print(pulsadorPresionado ? F("PRESIONADO") : F("LIBRE"));
      Serial.print(F(" | Estado: "));
      switch (controller.getState()) {
        case RobotState::STANDBY: Serial.print(F("STANDBY")); break;
        case RobotState::RACING: Serial.print(F("RACING")); break;
        case RobotState::RESCUING: Serial.print(F("RESCUING")); break;
        case RobotState::EMERGENCY_STOP: Serial.print(F("EMERGENCY_STOP")); break;
      }
      Serial.print(F(" | PWM: "));
      Serial.print(out.motors.leftPwm);
      Serial.print(out.motors.leftForward ? F("F") : F("R"));
      Serial.print(F("/"));
      Serial.print(out.motors.rightPwm);
      Serial.print(out.motors.rightForward ? F("F") : F("R"));
      Serial.println();
    }
  }
}
#endif

void setup() {
  // Configuración de pines de motores
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configuración de sensores (arreglo discreto de 4 sensores)
  pinMode(SENSOR_1_EXT_IZQ, INPUT);
  pinMode(SENSOR_2_INT_IZQ, INPUT);
  pinMode(SENSOR_3_INT_DER, INPUT);
  pinMode(SENSOR_4_EXT_DER, INPUT);

  // Configuración de pulsador de largada con pullup interno
  pinMode(PIN_PULSADOR_LARGADA, INPUT_PULLUP);

  // Configuración de indicador luminoso
  pinMode(PIN_INDICADOR, OUTPUT);

  // Aplicar estado inicial seguro (motores apagados, indicador apagado)
  applyOutputs(controller.getOutputs());

#ifdef DEBUG_MODE
  setupDebug();
#endif
}

void loop() {
#ifdef DEBUG_MODE
  loopDebug();
#else
  // Lectura del pulsador de largada (activo en bajo por INPUT_PULLUP)
  bool pulsadorPresionado = (digitalRead(PIN_PULSADOR_LARGADA) == LOW);

  // Lectura física de sensores ópticos TCRT5000 (S1..S4)
  int rawS1 = digitalRead(SENSOR_1_EXT_IZQ);
  int rawS2 = digitalRead(SENSOR_2_INT_IZQ);
  int rawS3 = digitalRead(SENSOR_3_INT_DER);
  int rawS4 = digitalRead(SENSOR_4_EXT_DER);

  // Actualización del controlador de dominio puro con normalización unificada
  controller.updateRaw(pulsadorPresionado, rawS1, rawS2, rawS3, rawS4, millis());

  // Aplicar salidas calculadas al hardware
  applyOutputs(controller.getOutputs());
#endif
}
