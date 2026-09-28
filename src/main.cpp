#include <Arduino.h>
#include "LineFollowerController.h"

// Pines de control del Motor Izquierdo (Driver B)
const int ENA = 6;  // Pin PWM para velocidad izquierda
const int IN1 = 10; // Dirección izquierda (invertido para coincidir con el avance físico hacia adelante)
const int IN2 = 9;  // Dirección izquierda

// Pines de control del Motor Derecho (Driver A)
const int ENB = 5;  // Pin PWM para velocidad derecha
const int IN3 = 4;  // Dirección derecha
const int IN4 = 8;  // Dirección derecha

// Pines de los sensores TCRT5000 (Digitales)
const int SENSOR_IZQ = 2;
const int SENSOR_DER = 3;

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
  Serial.println(F("   VELOCISTA GABRIEL - MODO DEBUG"));
  Serial.println(F("=========================================="));
  Serial.println(F("[1] Motor Izquierdo adelante (PWM 150)"));
  Serial.println(F("[2] Motor Derecho adelante (PWM 150)"));
  Serial.println(F("[3] Ambos Motores adelante (PWM 150)"));
  Serial.println(F("[s] Parar todos los motores"));
  Serial.println(F("[4] Monitoreo continuo Sensores y Pulsador"));
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
      int rawIzq = digitalRead(SENSOR_IZQ);
      int rawDer = digitalRead(SENSOR_DER);
      SensorInputs norm = LineFollowerController::normalizeSensors(rawIzq, rawDer, POLARIDAD_PISTA);

      Serial.print(F("[SENSORES] Izq: "));
      Serial.print(rawIzq);
      Serial.print(norm.leftDetected ? F(" (LINEA)") : F(" (FONDO)"));
      Serial.print(F(" | Der: "));
      Serial.print(rawDer);
      Serial.print(norm.rightDetected ? F(" (LINEA)") : F(" (FONDO)"));
      Serial.print(F(" | Pulsador: "));
      Serial.println(pulsadorPresionado ? F("PRESIONADO") : F("LIBRE"));
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

  // Configuración de sensores
  pinMode(SENSOR_IZQ, INPUT);
  pinMode(SENSOR_DER, INPUT);

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

  // Lectura física de sensores ópticos TCRT5000
  int rawIzq = digitalRead(SENSOR_IZQ);
  int rawDer = digitalRead(SENSOR_DER);

  // Actualización del controlador de dominio puro con normalización unificada
  controller.updateRaw(pulsadorPresionado, rawIzq, rawDer, millis());

  // Aplicar salidas calculadas al hardware
  applyOutputs(controller.getOutputs());
#endif
}
