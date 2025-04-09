// Definición de pines según la selección final

// Sensores de fin de carrera (entrada con pull-up)
#define Sensor_IR_Inicio 39
#define Sensor_IR_Final 35

// Sensores IR (entrada con pull-up)
#define SENSOR_IR_BAJO 32
#define SENSOR_IR_ALTO 33

// Indicadores de nivel (LEDs - salida)
#define LED_VERDE 25
#define LED_AMARILLO 26
#define LED_ROJO 27

// Botones de inicio y paro (entrada con pull-up)
#define BOTON_INICIO 13
#define BOTON_PARO 14

// PWM del puente H (salida PWM)
#define PWM_MOTOR_1 23
#define PWM_MOTOR_2 16

// Dirección del puente H (salida)
#define DIR_MOTOR_1_A 19
#define DIR_MOTOR_1_B 22
#define DIR_MOTOR_2_A 5
#define DIR_MOTOR_2_B 17

// Habilitación del puente H (salida)
#define HABILITACION_H 18

//#################-----DEFINICION DE ESATDOS----###############################
enum Estado {
  INICIO,
  ESPERA_ACCIONAR_BANDA,
  AVANZA_LLENADO,
  LLENADO_RECIPIENTE_BAJO,
  LLENADO_RECIPIENTE_ALTO,
  AVANZA_FINAL,
  POCISION_FINAL,
  PARO_EMERGENCIA
} estadoActual = INICIO;
enum Subestado {
  PARPADEO_LED,
  ENCENDIDO_MOTOR,
  PARPADEO_FINAL,
  TERMINADO
} subestado = PARPADEO_LED;

//#################-----VARIABLES-----##########################################
int tiempoLlenadoBAjo = 2500;
int tiempoLlenadoAlto = 4500;

int DUTY_CYCLE_BANDA = 153;
int DUTY_CYCLE_BOMBA = 127;
unsigned long previousMillis = 0;
unsigned long tiempoAnterior = 0;
bool ledState = false;
volatile unsigned long ultimaInterrupcion = 0;  // Estado actual del LED
volatile bool emergenciaActivada = false;       // Variable volátil para la interrupción
//#################-----FUNCIONES------###########################################

void IRAM_ATTR interrupcionParoEmergencia() {
    emergenciaActivada = true;
}

void esatdo_Inicio() {
  controlLED(LED_VERDE, 1000);
  int lectIrInicio = digitalRead(Sensor_IR_Inicio);
  int lectBotonInicio = digitalRead(BOTON_INICIO);
  int lectIrFinal = digitalRead(Sensor_IR_Final);
  //Serial.printf("Sensor IR Inicio %d, boton de inicio %d, Sensor Ir Final %d \n", lectIrInicio, lectBotonInicio, lectIrFinal);
  if (lectIrInicio == 0 && lectBotonInicio == 0 && lectIrFinal == 1) {
    estadoActual = ESPERA_ACCIONAR_BANDA;
    tiempoAnterior = millis();
  }
  int lect_Boton_Paro = digitalRead(BOTON_PARO);
  return;
}

void esatdo_ESPERA_ACCIONAR_BANDA() {
  controlLED(LED_VERDE, 100);
  unsigned long tiempoActual = millis();
  int lectIrInicio = digitalRead(Sensor_IR_Inicio);
  if (lectIrInicio == 1) {
    estadoActual = INICIO;
  }
  if (tiempoActual - tiempoAnterior >= 5000) {
    estadoActual = AVANZA_LLENADO;
    digitalWrite(LED_VERDE, 0);
  }
  int lect_Boton_Paro = digitalRead(BOTON_PARO);
  return;
}

void estado_AVANZA_LLENADO() {
  controlLED(LED_AMARILLO, 100);
  analogWrite(PWM_MOTOR_1, DUTY_CYCLE_BANDA);
  digitalWrite(HABILITACION_H, 1);
  int lect_IR_Llenado_Bajo = digitalRead(SENSOR_IR_BAJO);
  int lect_IR_Llenado_Alto = digitalRead(SENSOR_IR_ALTO);

  Serial.printf("Ir nivel bajo %d, Ir nivel Alto %d \n", lect_IR_Llenado_Bajo, lect_IR_Llenado_Alto);
  if (lect_IR_Llenado_Bajo == 0) {
    estadoActual = LLENADO_RECIPIENTE_ALTO;
    analogWrite(PWM_MOTOR_1, 0);
    digitalWrite(HABILITACION_H, 0);
    tiempoAnterior = millis();
  }
  if (lect_IR_Llenado_Alto == 0) {
    estadoActual = LLENADO_RECIPIENTE_BAJO;
    analogWrite(PWM_MOTOR_1, 0);
    digitalWrite(HABILITACION_H, 0);
    tiempoAnterior = millis();
  }
  int lect_Boton_Paro = digitalRead(BOTON_PARO);
  return;
}

void estado_LLENADO_RECIPIENTE_BAJO() {
  static unsigned long tiempoReferencia = 0;
  static bool ledAmarilloState = false;

  switch (subestado) {
    case PARPADEO_LED:
      // Parpadeo del LED amarillo a 50ms durante 1 segundo (20 ciclos)
      if (millis() - tiempoReferencia >= 50) {
        tiempoReferencia = millis();
        ledAmarilloState = !ledAmarilloState;
        digitalWrite(LED_AMARILLO, ledAmarilloState);

        static int ciclos = 0;
        ciclos++;
        if (ciclos >= 60) {  // 20 ciclos * 50ms = 1 segundo
          ciclos = 0;
          subestado = ENCENDIDO_MOTOR;
          tiempoReferencia = millis();
          detachInterrupt(digitalPinToInterrupt(BOTON_PARO));
          digitalWrite(LED_AMARILLO, HIGH);  // LED fijo encendido
          analogWrite(PWM_MOTOR_1, 0);
          analogWrite(PWM_MOTOR_2, DUTY_CYCLE_BOMBA);
          digitalWrite(HABILITACION_H, 1);
        }
      }
      break;

    case ENCENDIDO_MOTOR:
      // Motor encendido por 2500ms con LED fijo
      if (millis() - tiempoReferencia >= tiempoLlenadoBAjo) {
        analogWrite(PWM_MOTOR_1, 0);
        analogWrite(PWM_MOTOR_2, 0);
        digitalWrite(HABILITACION_H, 0);
        subestado = PARPADEO_FINAL;
        tiempoReferencia = millis();
      }
      if(!digitalRead(BOTON_PARO)){
        estadoActual=PARO_EMERGENCIA;
      }
      break;

    case PARPADEO_FINAL:
      attachInterrupt(digitalPinToInterrupt(BOTON_PARO), interrupcionParoEmergencia, FALLING);
      // Parpadeo final del LED amarillo a 50ms durante 3 segundos (60 ciclos)
      if (millis() - tiempoReferencia >= 50) {
        tiempoReferencia = millis();
        ledAmarilloState = !ledAmarilloState;
        digitalWrite(LED_AMARILLO, ledAmarilloState);
        static int ciclosFinal = 0;
        ciclosFinal++;
        if (ciclosFinal >= 60) {  // 60 ciclos * 50ms = 3000ms (3 segundos)
          ciclosFinal = 0;
          subestado = TERMINADO;
        }
      }
      break;

    case TERMINADO:

      // Transición al siguiente estado
      subestado = PARPADEO_LED;
      estadoActual = AVANZA_FINAL;
      break;
  }
}

void estado_LLENADO_RECIPIENTE_ALTO() {

  static unsigned long tiempoReferencia = 0;
  static bool ledAmarilloState = false;

  switch (subestado) {
    case PARPADEO_LED:
      // Parpadeo del LED amarillo a 50ms durante 1 segundo (20 ciclos)
      if (millis() - tiempoReferencia >= 50) {
        tiempoReferencia = millis();
        ledAmarilloState = !ledAmarilloState;
        digitalWrite(LED_AMARILLO, ledAmarilloState);

        static int ciclos = 0;
        ciclos++;
        if (ciclos >= 60) {  // 20 ciclos * 50ms = 1 segundo
          ciclos = 0;
          subestado = ENCENDIDO_MOTOR;
          tiempoReferencia = millis();
          detachInterrupt(digitalPinToInterrupt(BOTON_PARO));
          digitalWrite(LED_AMARILLO, HIGH);  // LED fijo encendido
          analogWrite(PWM_MOTOR_1, 0);
          analogWrite(PWM_MOTOR_2, DUTY_CYCLE_BOMBA);
          digitalWrite(HABILITACION_H, 1);
        }
      }
      break;

    case ENCENDIDO_MOTOR:
      // Motor encendido por 2500ms con LED fijo
      if (millis() - tiempoReferencia >= tiempoLlenadoAlto) {
        analogWrite(PWM_MOTOR_1, 0);
        analogWrite(PWM_MOTOR_2, 0);
        digitalWrite(HABILITACION_H, 0);
        subestado = PARPADEO_FINAL;
        tiempoReferencia = millis();
      }
      if(!digitalRead(BOTON_PARO)){
        estadoActual=PARO_EMERGENCIA;
      }
      break;

    case PARPADEO_FINAL:
    attachInterrupt(digitalPinToInterrupt(BOTON_PARO), interrupcionParoEmergencia, FALLING);
      // Parpadeo final del LED amarillo a 50ms
      if (millis() - tiempoReferencia >= 50) {
        tiempoReferencia = millis();
        ledAmarilloState = !ledAmarilloState;
        digitalWrite(LED_AMARILLO, ledAmarilloState);
        static int ciclosFinal = 0;
        ciclosFinal++;
        if (ciclosFinal >= 60) {  // 1 segundo de parpadeo final
          ciclosFinal = 0;
          subestado = TERMINADO;
        }
      }
      break;

    case TERMINADO:

      // Transición al siguiente estado
      subestado = PARPADEO_LED;
      estadoActual = AVANZA_FINAL;
      break;
  }

  int lect_Boton_Paro = digitalRead(BOTON_PARO);
}

void esatdo_AVANZA_FINAL() {
  analogWrite(PWM_MOTOR_1, DUTY_CYCLE_BANDA);
  digitalWrite(HABILITACION_H, 1);
  controlLED(LED_AMARILLO, 100);
  int lectIrFinal = digitalRead(Sensor_IR_Final);
  if (lectIrFinal == 0) {
    analogWrite(PWM_MOTOR_1, 0);
    digitalWrite(HABILITACION_H, 0);
    digitalWrite(LED_AMARILLO, 0);
    estadoActual = POCISION_FINAL;
  }
  int lect_Boton_Paro = digitalRead(BOTON_PARO);

  return;
}

void estado_POCISION_FINAL() {
  digitalWrite(LED_VERDE, 1);
  int lectIrFinal = digitalRead(Sensor_IR_Final);
  if (lectIrFinal == 1) {
    estadoActual = INICIO;
  }
  int lect_Boton_Paro = digitalRead(BOTON_PARO);
  return;
}

void estado_PARO_EMERGENCIA() {
  analogWrite(PWM_MOTOR_1, 0);
  analogWrite(PWM_MOTOR_2, 0);
  digitalWrite(HABILITACION_H, 0);
  delay(100);
  digitalWrite(LED_VERDE, 0);
  digitalWrite(LED_AMARILLO, 0);
  controlLED(LED_ROJO, 100);
  int lect_Boton_Inicio = digitalRead(BOTON_INICIO);
  if (lect_Boton_Inicio == 0) {
    digitalWrite(LED_ROJO, 0);
    delay(100);
    digitalWrite(LED_VERDE, 1);
    delay(100);
    digitalWrite(LED_AMARILLO, 1);
    delay(100);
    digitalWrite(LED_ROJO, 1);
    delay(100);
    digitalWrite(LED_ROJO, 0);
    delay(100);
    digitalWrite(LED_AMARILLO, 0);
    delay(100);
    digitalWrite(LED_VERDE, 0);
    delay(100);
    estadoActual = INICIO;
    subestado = PARPADEO_LED;
  }

  return;
}

void SFM() {
  Serial.println(estadoActual);
  if (emergenciaActivada) {
    estadoActual = PARO_EMERGENCIA;
    emergenciaActivada = false;  // Resetear la bandera
  }
  switch (estadoActual) {
    case INICIO:
      esatdo_Inicio();
      break;
    case ESPERA_ACCIONAR_BANDA:
      esatdo_ESPERA_ACCIONAR_BANDA();
      break;
    case AVANZA_LLENADO:
      estado_AVANZA_LLENADO();
      break;
    case LLENADO_RECIPIENTE_BAJO:
      estado_LLENADO_RECIPIENTE_BAJO();
      break;
    case LLENADO_RECIPIENTE_ALTO:
      estado_LLENADO_RECIPIENTE_ALTO();
      break;
    case AVANZA_FINAL:
      esatdo_AVANZA_FINAL();
      break;
    case POCISION_FINAL:
      estado_POCISION_FINAL();
      break;
    case PARO_EMERGENCIA:
      estado_PARO_EMERGENCIA();
      break;
  }
}

//#################-----CONTROL LED---##########################################
void controlLED(int pin, unsigned long interval) {
  unsigned long currentMillis = millis();

  // Configurar el pin como salida si no está configurado
  pinMode(pin, OUTPUT);

  // Verificar si ha pasado el tiempo especificado
  if (currentMillis - previousMillis >= interval) {
    // Guardar el último tiempo de cambio
    previousMillis = currentMillis;

    // Cambiar el estado del LED
    ledState = !ledState;

    // Escribir el nuevo estado al LED
    digitalWrite(pin, ledState);
  }
}

void setup() {
  Serial.begin(115200);
  // Configurar sensores de fin de carrera como entrada con pull-up
  pinMode(Sensor_IR_Inicio, INPUT_PULLUP);
  pinMode(Sensor_IR_Final, INPUT_PULLUP);

  // Configurar sensores IR como entrada con pull-up
  pinMode(SENSOR_IR_BAJO, INPUT_PULLUP);
  pinMode(SENSOR_IR_ALTO, INPUT_PULLUP);

  // Configurar botones de inicio y paro como entrada con pull-up
  pinMode(BOTON_INICIO, INPUT_PULLUP);
  pinMode(BOTON_PARO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BOTON_PARO), interrupcionParoEmergencia, FALLING);
  // Configurar LEDs indicadores como salida
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_AMARILLO, OUTPUT);
  pinMode(LED_ROJO, OUTPUT);

  // Configurar pines de dirección del puente H como salida
  pinMode(DIR_MOTOR_1_A, OUTPUT);
  pinMode(DIR_MOTOR_1_B, OUTPUT);
  pinMode(DIR_MOTOR_2_A, OUTPUT);
  pinMode(DIR_MOTOR_2_B, OUTPUT);

  // Configurar pines PWM del puente H como salida
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);

  // Configurar habilitación del puente H como salida
  pinMode(HABILITACION_H, OUTPUT);

  // Opcional: Inicializar estados por defecto
  digitalWrite(LED_VERDE, LOW);
  digitalWrite(LED_AMARILLO, LOW);
  digitalWrite(LED_ROJO, LOW);
  digitalWrite(DIR_MOTOR_1_A, LOW);
  digitalWrite(DIR_MOTOR_1_B, LOW);
  digitalWrite(DIR_MOTOR_2_A, LOW);
  digitalWrite(DIR_MOTOR_2_B, LOW);
  digitalWrite(PWM_MOTOR_1, LOW);
  digitalWrite(PWM_MOTOR_2, LOW);
  digitalWrite(HABILITACION_H, LOW);

  // Definir dirección de los motores
  digitalWrite(DIR_MOTOR_1_A, HIGH);
  digitalWrite(DIR_MOTOR_1_B, LOW);
  digitalWrite(DIR_MOTOR_2_A, HIGH);
  digitalWrite(DIR_MOTOR_2_B, LOW);

  // Inicializa el esatdo incial
  estadoActual = INICIO;
  delay(300);
}

void loop() {
  SFM();
}
