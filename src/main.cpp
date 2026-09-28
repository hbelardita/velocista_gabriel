#include <Arduino.h>
#include "LineFollowerController.h"

// Pines de control del Motor Izquierdo (Driver B)
const int ENA = 6;  // Pin PWM para velocidad izquierda
const int IN1 = 9;  // Dirección izquierda
const int IN2 = 10; // Dirección izquierda

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

// Configuración de polaridad de pista
const int VALOR_BLANCO = LOW;
const int VALOR_NEGRO = HIGH;
const bool LINEA_BLANCA = true; // true: línea blanca sobre fondo negro

// Instancia del controlador de dominio puro
LineFollowerController controller;

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
}

void loop() {
  // Lectura del pulsador de largada (activo en bajo por INPUT_PULLUP)
  bool pulsadorPresionado = (digitalRead(PIN_PULSADOR_LARGADA) == LOW);

  // Lectura y normalización de sensores según polaridad de pista
  int rawIzq = digitalRead(SENSOR_IZQ);
  int rawDer = digitalRead(SENSOR_DER);
  SensorInputs sensors;
  sensors.leftDetected = (rawIzq == (LINEA_BLANCA ? VALOR_BLANCO : VALOR_NEGRO));
  sensors.rightDetected = (rawDer == (LINEA_BLANCA ? VALOR_BLANCO : VALOR_NEGRO));

  // Actualización del controlador de dominio puro
  controller.update(pulsadorPresionado, sensors, millis());

  // Aplicar salidas calculadas al hardware
  applyOutputs(controller.getOutputs());
}
