// Definición de pines según la selección final

// Sensores de fin de carrera (entrada con pull-up)
#define FIN_CARRERA_1  39
#define FIN_CARRERA_2  34
#define FIN_CARRERA_3  35

// Sensores IR (entrada con pull-up)
#define SENSOR_IR_1  32
#define SENSOR_IR_2  33

// Indicadores de nivel (LEDs - salida)
#define LED_VERDE  25
#define LED_AMARILLO  26
#define LED_ROJO  27

// Botones de inicio y paro (entrada con pull-up)
#define BOTON_INICIO  14
#define BOTON_PARO    13

// PWM del puente H (salida PWM)
#define PWM_MOTOR_1  23
#define PWM_MOTOR_2  16

// Dirección del puente H (salida)
#define DIR_MOTOR_1_A  19
#define DIR_MOTOR_1_B  22
#define DIR_MOTOR_2_A  5
#define DIR_MOTOR_2_B  17

// Habilitación del puente H (salida)
#define HABILITACION_H  18

#define DUTY_CYCLE static_cast<int>((50.0 / 100.0) * 255)

void setup() {
    // Configurar sensores de fin de carrera como entrada con pull-up
    pinMode(FIN_CARRERA_1, INPUT_PULLUP);
    pinMode(FIN_CARRERA_2, INPUT_PULLUP);
    pinMode(FIN_CARRERA_3, INPUT_PULLUP);

    // Configurar sensores IR como entrada con pull-up
    pinMode(SENSOR_IR_1, INPUT_PULLUP);
    pinMode(SENSOR_IR_2, INPUT_PULLUP);

    // Configurar botones de inicio y paro como entrada con pull-up
    pinMode(BOTON_INICIO, INPUT_PULLUP);
    pinMode(BOTON_PARO, INPUT_PULLUP);

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

    // Asignar un duty cycle del 50% a los motores
    analogWrite(PWM_MOTOR_1, 0);
    analogWrite(PWM_MOTOR_2, 127);

    // Activar el puente H
    digitalWrite(HABILITACION_H, HIGH);
}

void loop() {
                        // Espera 500ms
}

