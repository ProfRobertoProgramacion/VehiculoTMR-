/*
 * ==============================================================================
 * Controlador de Bajo Nivel Arduino para Vehículo Autónomo TMR
 * ==============================================================================
 * Recibe tramas por puerto serie desde la Odroid M1S y comanda:
 *  - Pin 9: Servo de dirección (Ackermann). Rango típico 60° a 120° (Centro 90°).
 *  - Pin 10: ESC / Motor de tracción mediante señal PWM tipo servo (1000us - 2000us).
 *
 * Formato de trama entrante: <steering_deg,throttle_us>
 * Ejemplo:
 *   <90,1500>   -> Dirección al centro (90°), motor detenido (1500 us).
 *   <75,1580>   -> Giro suave a la izquierda (75°), avance lento adelante (1580 us).
 *   <105,1580>  -> Giro suave a la derecha (105°), avance lento adelante (1580 us).
 *
 * Seguridad (Watchdog):
 *   Si no recibe una trama válida en 400 ms, detiene el motor automáticamente.
 * ==============================================================================
 */

#include <Servo.h>

// --- Configuración de Pines ---
const int PIN_SERVO_STEERING = 9;
const int PIN_ESC_THROTTLE   = 10;
const int PIN_LED_STATUS     = 13;

// --- Parámetros de Calibración de Dirección ---
const int SERVO_CENTER_DEG = 90;
const int SERVO_MIN_DEG    = 55;   // Límite físico mecánico giro izquierda
const int SERVO_MAX_DEG    = 125;  // Límite físico mecánico giro derecha

// --- Parámetros de Calibración de Motor ESC ---
// Para ESC estándar de coche RC: 1500us = Neutro, >1500us = Adelante, <1500us = Reversa/Freno
const int ESC_NEUTRAL_US   = 1500;
const int ESC_MIN_US       = 1000;
const int ESC_MAX_US       = 2000;
const int ESC_SAFE_FORWARD = 1650; // Límite máximo de velocidad por software para pruebas seguras

// --- Watchdog de Seguridad ---
const unsigned long TIMEOUT_MS = 400; // Detener si se pierde comunicación por más de 400ms
unsigned long last_cmd_time = 0;

Servo servoSteering;
Servo escThrottle;

// Variables de recepción serie
const byte BUFFER_SIZE = 32;
char receivedChars[BUFFER_SIZE];
boolean newData = false;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

  // Inicializar actuadores
  servoSteering.attach(PIN_SERVO_STEERING);
  escThrottle.attach(PIN_ESC_THROTTLE);

  // Posición inicial segura: centro y motor apagado
  servoSteering.write(SERVO_CENTER_DEG);
  escThrottle.writeMicroseconds(ESC_NEUTRAL_US);

  delay(1000); // Dar tiempo de armado al ESC
  last_cmd_time = millis();
  Serial.println(F("[ARDUINO READY] TMR Auto Controller Initialized"));
}

void loop() {
  readSerialData();

  if (newData) {
    parseAndExecuteCommand();
    newData = false;
  }

  // Comprobar Fail-Safe (Timeout de seguridad)
  if (millis() - last_cmd_time > TIMEOUT_MS) {
    // Apagar motor por seguridad si se perdió conexión con Odroid/ROS
    escThrottle.writeMicroseconds(ESC_NEUTRAL_US);
    digitalWrite(PIN_LED_STATUS, LOW);
  } else {
    digitalWrite(PIN_LED_STATUS, HIGH);
  }
}

// Lectura de trama no bloqueante con delimitadores '<' y '>'
void readSerialData() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && !newData) {
    rc = Serial.read();

    if (recvInProgress) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= BUFFER_SIZE) {
          ndx = BUFFER_SIZE - 1;
        }
      } else {
        receivedChars[ndx] = '\0'; // Terminar cadena
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

// Procesar cadena tipo "steering,throttle"
void parseAndExecuteCommand() {
  char *strtokIndx;

  // Extraer primer valor: Ángulo de dirección en grados
  strtokIndx = strtok(receivedChars, ",");
  if (strtokIndx == NULL) return;
  int target_steering = atoi(strtokIndx);

  // Extraer segundo valor: Pulso de aceleración en microsegundos
  strtokIndx = strtok(NULL, ",");
  if (strtokIndx == NULL) return;
  int target_throttle = atoi(strtokIndx);

  // Aplicar límites de seguridad (clamping)
  target_steering = constrain(target_steering, SERVO_MIN_DEG, SERVO_MAX_DEG);
  target_throttle = constrain(target_throttle, ESC_MIN_US, ESC_SAFE_FORWARD);

  // Comandar actuadores
  servoSteering.write(target_steering);
  escThrottle.writeMicroseconds(target_throttle);

  last_cmd_time = millis(); // Refrescar watchdog
}
