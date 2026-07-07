// --- MODO DE PRUEBA / DEBUG ---
// Descomentá la siguiente línea para activar la prueba secuencial de motores.
// En este modo, el seguidor ignorará los sensores y ejecutará una rutina para
// verificar el giro correcto de cada rueda.
//#define DEBUG_TEST_MODE

// Pines de control del Motor Izquierdo (A)
const int ENA = 5;  // Pin PWM para velocidad izquierda
const int IN1 = 7;  // Dirección izquierda
const int IN2 = 8;  // Dirección izquierda

// Pines de control del Motor Derecho (B)
const int ENB = 6;  // Pin PWM para velocidad derecha
const int IN3 = 9;  // Dirección derecha
const int IN4 = 10; // Dirección derecha

// Pines de los sensores TCRT5000 (Digitales)
const int SENSOR_IZQ = 2;
const int SENSOR_DER = 3;

// --- CONSTANTES DE CALIBRACIÓN ---
const int VELOCIDAD_BASE = 150; // Velocidad para ir recto
const int VELOCIDAD_GIRO = 50;  // Velocidad de la rueda que frena al girar (0 a 100)

// Configuración de lectura de sensores
const int VALOR_BLANCO = LOW;  
const int VALOR_NEGRO = HIGH;  

void setup() {
  #ifdef DEBUG_TEST_MODE
  Serial.begin(9600);
  Serial.println(F("--- MODO CONTROL SERIAL ACTIVO ---"));
  Serial.println(F("Envia comandos por monitor serie:"));
  Serial.println(F("  W / F : Avanzar"));
  Serial.println(F("  S / B : Retroceder"));
  Serial.println(F("  A / L : Girar Izquierda"));
  Serial.println(F("  D / R : Girar Derecha"));
  Serial.println(F("  X / Espacio : Detener"));
  #endif

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

  // Aseguramos que los motores estén detenidos
  detenerMotores();

  // Esperamos 2 segundos al encender el interruptor para que te dé 
  // tiempo de sacar la mano antes de que el robot salga disparado.
  delay(2000); 
}

void loop() {
  #ifdef DEBUG_TEST_MODE
  procesarComandosSerial();
  #else
  int izq = digitalRead(SENSOR_IZQ);
  int der = digitalRead(SENSOR_DER);

  // Lógica principal: Línea BLANCA sobre fondo NEGRO.
  if (izq == VALOR_NEGRO && der == VALOR_NEGRO) {
    avanzar();
  } 
  else if (izq == VALOR_BLANCO && der == VALOR_NEGRO) {
    girarIzquierda();
  } 
  else if (izq == VALOR_NEGRO && der == VALOR_BLANCO) {
    girarDerecha();
  }
  else if (izq == VALOR_BLANCO && der == VALOR_BLANCO) {
    avanzar();
  }
  #endif
}

// --- FUNCIONES DE MOVIMIENTO ---

void avanzar() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, VELOCIDAD_BASE);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, VELOCIDAD_BASE);
}

void girarIzquierda() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, VELOCIDAD_GIRO); 

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, VELOCIDAD_BASE);
}

void girarDerecha() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, VELOCIDAD_BASE);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, VELOCIDAD_GIRO);
}

void detenerMotores() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void retroceder() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, VELOCIDAD_BASE);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, VELOCIDAD_BASE);
}

void rotarIzquierda() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, VELOCIDAD_BASE);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, VELOCIDAD_BASE);
}

void rotarDerecha() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, VELOCIDAD_BASE);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, VELOCIDAD_BASE);
}

#ifdef DEBUG_TEST_MODE
void procesarComandosSerial() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Ignorar retornos de carro o salto de línea
    if (cmd == '\n' || cmd == '\r') {
      return;
    }
    
    switch (cmd) {
      case 'w':
      case 'W':
      case 'f':
      case 'F':
        Serial.println(F("Comando: AVANZAR"));
        avanzar();
        break;
      case 's':
      case 'S':
      case 'b':
      case 'B':
        Serial.println(F("Comando: RETROCEDER"));
        retroceder();
        break;
      case 'a':
      case 'A':
      case 'l':
      case 'L':
        Serial.println(F("Comando: GIRAR IZQUIERDA"));
        rotarIzquierda();
        break;
      case 'd':
      case 'D':
      case 'r':
      case 'R':
        Serial.println(F("Comando: GIRAR DERECHA"));
        rotarDerecha();
        break;
      case 'x':
      case 'X':
      case ' ':
        Serial.println(F("Comando: DETENER"));
        detenerMotores();
        break;
      default:
        Serial.print(F("Comando desconocido: '"));
        Serial.print(cmd);
        Serial.println(F("'"));
        Serial.println(F("Usa: W/F (adelante), S/B (atrás), A/L (izq), D/R (der), X/Espacio (detener)"));
        break;
    }
  }
}
#endif
