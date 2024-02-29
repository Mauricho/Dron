/**
 * Receptor. (Dron)
 * En este código se toma la información tipo estructura.
 * Los datos para los distitos ejes de movimiento ya estan en grados (-30;30). Y para la aceleración
 * 1000 a 2000.
 * Se agrega la configuración del MPU6050. Implica la inicialización, toma de datos.
 * Y los datos obtenidos del sensor se los prepara para poder trabajar con ellos. 
 * Se agregó que solo recibe cuando entra en la función recibir_datos.
 *¨Se agrego para generar señales PWM para los ESC.
 * Se pretende reducir espacio de memoria haciendo variables locales a las variables de tiempo ya que son 
 las que más espacio ocupan.
 * Se esta leyendo el mando desde dentro de la función PWM
*/
//----------------------------------------------------------------------------------
bool visu = 1;        // Visualizar variables por canal serie. En vuelo a 0!! Para monitor serie a 1
int visu_select = 4;  // 0: mando RC, 1: giro, 2: acc, 3: ang, 4: esc, 5:tiempo_ejecucion_MPU6050, 6:nivel batería
bool MODO_vuelo = 1;  // 0: Modo acrobatico, 1: Modo estable (por defecto MODO_vuelo = 1)
//----------------------------------------------------------------------------------
//Librerias
#include <Arduino.h>  //Al final eliminarla y probar
#include <SPI.h>
#include <RF24.h>               //Para el módulo NRF24L01
#include <Wire.h>               //Para el sensor MPU6050
#include <LiquidCrystal_I2C.h>  //Pantalla LCD con I2C
//----------------------------------------------------------------------------------
//Definiciones
#define usCiclo 4000         // Ciclo de ejecución de software en microsegundos
#define MPU6050_adress 0x68  // Dirección del sensor
#define pin_motor1 13        // Pin motor 1
#define pin_motor2 16        // Pin motor 2
#define pin_motor3 17        // Pin motor 3
#define pin_motor4 25        // Pin motor 4
//----------------------------------------------------------------------------------
//Objetos
RF24 radio(4, 5);                    // Configura CE y CSN para el módulo NRF24L01
LiquidCrystal_I2C lcd(0x27, 16, 2);  //set LCD address, columnas y filas
//----------------------------------------------------------------------------------
//Variables
// Estructura para los datos //Puedo tener hasta 32 canales cada uno de 1 byte = 8 bits
struct JoystickData {  //Debo ajustar estos valores
  int leftX = 0;
  int leftY = 1500;
  int rightX = 0;
  int rightY = 0;
  bool button1Pressed;
  bool button2Pressed;
};  //En cada parametro de la estructura guardo los valores recibidos.

JoystickData joystickData;

int gx, gy, gz;                            //variables para el Giroscopio MPU6050
float ax, ay, az, temperature;             //variables para Acelerómetro y temperatura MPU6050
float gyro_X_cal, gyro_Y_cal, gyro_Z_cal;  //variables de calibración Giroscopio
float acc_X_cal, acc_Y_cal, acc_Z_cal;     //variables de calibración Acelerómetro
float gyro_X, gyro_Y, gyro_Z;              //variables para obtenes la velocidad de rotación
float gyro_X_ant, gyro_Y_ant, gyro_Z_ant;  //variables para calcular la parte derivativa del PID
float angulo_pitch_acc, angulo_roll_acc;   //variables para obtener la aceleración
float angulo_pitch, angulo_roll;           //variables para obtener el ángulo
float angulo_pitch_ant, angulo_roll_ant;   //variables para calcular la parte derivativa del PID
float acc_total_vector;                    //variable para calcular la aceleración total
bool set_gyro_angles, accCalibOK = false;  //variables para procesar datos MPU6050 y me indica si la calibración se realizo

//variables para medir el tiempo
unsigned long loop_timer, tiempo_ejecucion_MPU6050, tiempo_MPU6050_1 = 0, ultimoTiempoRecibido;

// variables para calcular las señales PWM
float ESC1_us, ESC2_us, ESC3_us, ESC4_us;

// AJUSTE DE PIDs
// Modificar estos parámetros apara ajustar los PID
float Roll_ang_Kp = 0.5, Roll_ang_Ki = 0.05, Roll_ang_Kd = 10;
float Pitch_ang_Kp = 0.5, Pitch_ang_Ki = 0.05, Pitch_ang_Kd = 10;
float Pitch_W_Kp = 2, Pitch_W_Ki = 0.02, Pitch_W_Kd = 0;
float Roll_W_Kp = 2, Roll_W_Ki = 0.02, Roll_W_Kd = 0;
float Yaw_W_Kp = 1, Yaw_W_Ki = 0.05, Yaw_W_Kd = 0;

int PID_W_sat1 = 380;    // Limitar parte integral PID velocidad
int PID_W_sat2 = 380;    // Limitar salida del PID velocidad
int PID_ang_sat1 = 130;  // Limitar parte integral PID ángulo
int PID_ang_sat2 = 130;  // Limitar salida del PID ángulo

float PID_ang_Pitch_error, PID_ang_Pitch_P, PID_ang_Pitch_I, PID_ang_Pitch_D, PID_ang_Pitch_OUT;
float PID_ang_Roll_error, PID_ang_Roll_P, PID_ang_Roll_I, PID_ang_Roll_D, PID_ang_Roll_OUT;
float PID_ang_Yaw_error, PID_ang_Yaw_P, PID_ang_Yaw_I, PID_ang_Yaw_D, PID_ang_Yaw_OUT;
float PID_W_Pitch_error, PID_W_Pitch_P, PID_W_Pitch_I, PID_W_Pitch_D, PID_W_Pitch_OUT;
float PID_W_Roll_error, PID_W_Roll_P, PID_W_Roll_I, PID_W_Roll_D, PID_W_Roll_OUT;
float PID_W_Yaw_error, PID_W_Yaw_P, PID_W_Yaw_I, PID_W_Yaw_D, PID_W_Yaw_OUT;
float PID_W_Pitch_consigna, PID_W_Roll_consigna;

//Manejo de los leds
int batteryPercentage, batCont = 0, loopCant = 0;
//----------------------------------------------------------------------------------
// Constantes
const int pinBateria = 34;  //Pin conectado al divisor de tensión
const int ledPin = 27;      //Pin conectado al LED indicador batería
const int ledPort = 33;     //Pin habilitado el puerto serie
const int ledError = 32;    //Pin indicador de error
const int ledLoop = 26;     //Pin indicador del LOOP
//----------------------------------------------------------------------------------
//Interrupciones
//----------------------------------------------------------------------------------
void setup() {

  Wire.begin();  //1° iniciar comunicación I2C.

  //Declaración de pines-------------------------------------------------------------
  //Pines salida
  pinMode(ledPin, OUTPUT);
  pinMode(ledPort, OUTPUT);
  pinMode(ledError, OUTPUT);
  pinMode(ledLoop, OUTPUT);

  // Declaración de los pines de los motores
  pinMode(pin_motor1, OUTPUT);  //Motor 1
  pinMode(pin_motor2, OUTPUT);  //Motor 2
  pinMode(pin_motor3, OUTPUT);  //Motor 3
  pinMode(pin_motor4, OUTPUT);  //Motor 4
  // Forzar los pines a estado LOW
  digitalWrite(pin_motor1, LOW);
  digitalWrite(pin_motor2, LOW);
  digitalWrite(pin_motor3, LOW);
  digitalWrite(pin_motor4, LOW);

  //Pines entrada
  pinMode(pinBateria, INPUT);

  //----------------------------------------------------------------------------
  // Solo iniciamos el monitor serie si alguno de los modos de visualización está activo
  if (visu == 1) {
    Serial.begin(9600);
    digitalWrite(ledPort, HIGH);
  }
  //NRF24L01----------------------------------------------------------------------------
  radio.begin();
  // Número de canal (puede ser cualquier valor entre 0 y 125) Frecuencia = 2400 + Canal
  uint8_t channel = 105;  // Cambiar este valor según las necesidades
  radio.setChannel(channel);
  // Configura la velocidad de datos a 250Kbps
  radio.setDataRate(RF24_250KBPS);
  // Potencia de salida
  radio.setPALevel(RF24_PA_MIN);  // Establece la potencia de salida mínima --> luego cambiar por RF24_PA_MAX
  // Dirección del canal de comunicación
  radio.openReadingPipe(0, 0xA1B2C3E4F0LL);  //pipes del 0 al 5
  radio.startListening();                    // Cambia al modo de escucha //Lo coloco aquí para que se ejecute solo al principio

  //LCD16X2----------------------------------------------------------------------------
  lcd.init();  //Inicializamos LCD
  lcd.backlight();  //Encenderluz de fondo //Podemos apagarla para ahorrar batería!

  lcd.setCursor(0, 0);
  lcd.print("Iniciando Dron");
  delay(5000);  //Delay de 5 segundos!

  digitalWrite(ledLoop, HIGH);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Encender mando");
  lcd.setCursor(0, 1);
  if (MODO_vuelo == 1) lcd.print("-MODO Estable-");
  else lcd.print("-MODO Acro-");

  // Para poder avanzar hay que encender el mando y bajar Throttle al mínimo
  while (joystickData.leftY < 950 || joystickData.leftY > 1050) recibir_los_datos();
  digitalWrite(ledLoop, LOW);

  //MPU6050-----------------------------------------------------------------------------
  MPU6050_iniciar();
  MPU6050_calibrar();

  //----------------------------------- Arrancar Programa -----------------------------
  // Para entrar el loop principal hay que mover el stick de Roll a la derecha
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ROLL -->");
  while (joystickData.rightX < 10) {
    Lectura_tension_bateria();  // Leer Vbat
    recibir_los_datos();
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Buen viaje!");
  delay(2000);  // Espera 2 segundos
  lcd.clear();
  lcd.noBacklight();  //Apagar luz de fondo
  lcd.noDisplay();  // Apaga la pantalla LCD

  loop_timer = micros();
}

void loop() {
  // Si se supera el tiempo de ciclo, se activa el LED rojo 2
  if (micros() - loop_timer > usCiclo + 50) digitalWrite(ledError, HIGH);

  // Ecuaciones de procesamiento
  // Directamente ya recibe los valores procesados estan guardados en la estructura joystickData

  while (micros() - loop_timer < usCiclo)
    ;  //Espero hasta que haya pasado el tiempo definido por usCiclo
  //tiempo_ejecucion = (micros() - loop_timer) / 1000; //se calcula el tiempo que ha pasado durante la ejecución del ciclo en milisegundos
  loop_timer = micros();  //marca el tiempo de inicio del próximo ciclo.

  PWM();               // Generar señales PWM para los motores
  MPU6050_leer();      // Leer sensor MPU6050
  MPU6050_procesar();  // Procesar datos del sensor MPU6050

  if (MODO_vuelo == 1) PID_ang();  // Obtener salida de los PID de inclinación
  PID_w();                         // Obtener salida de los PID de velocidad
  Modulador();                     // Calcula el tiempo en alto de la señal PWM

  // Guardamos las lecturas del sensor MPU6050 para el siguiente ciclo (necesario para los PID), creo que esta lineas se pueden sacar, es donde se hace el guardado de las variables anteriorees pero ya lo hace la función PID.
  angulo_pitch_ant = angulo_pitch;
  angulo_roll_ant = angulo_roll;

  gyro_X_ant = gyro_X;  // Pitch
  gyro_Y_ant = gyro_Y;  // Roll
  gyro_Z_ant = gyro_Z;  // Yaw

  // Visualización de variables
  if (visu == 1) Visualizaciones();
}

// Función para recibir los datos.
// Guarda los datos recibidos en la estructura
// y guarda el tiempo
void recibir_los_datos() {
  // Check if RF is connected and packet is available
  while (radio.available() && radio.isChipConnected()) {
    radio.read(&joystickData, sizeof(joystickData));  // Recibe los valores del transmisor
    ultimoTiempoRecibido = micros();                  // Guardo el tiempo desde que se esta ejecutando el programa
  }
}

//Función que permite reiniciar los datos para cuando no se detecta entrada de radio.
//Todos los datos a utilizar como ordenes del usuario se guardar en la estructura joystickData
void reiniciar_los_datos() {
  joystickData.leftX = 0;     // Mantiene la posición en Yaw   (valor medio)
  joystickData.leftY = 1500;  // Tomo un valor medio para la aceleración (valor medio para que se mantenga en el aire)
  joystickData.rightX = 0;    // Mantiene la posición en Roll  (valor medio)
  joystickData.rightY = 0;    // Mantiene la posición en Pitch (valor medio)
  joystickData.button1Pressed = false;
  joystickData.button2Pressed = false;
}

// Función para iniciar el sensor MPU6050
// configura diversos parámetros del sensor MPU6050, como la activación del giroscopio,
// la configuración del rango del giroscopio y acelerómetro, y la configuración del filtro pasa bajos (LPF)
// es necesario inicializarlo tras cada apagado para poder utilizarlo.
/*
Elige el rango que mejor se adapte a las tasas de giro que esperas medir. 
Por ejemplo, si tu dron realiza giros rápidos, es posible que desees utilizar un rango más alto.
La elección de la sensibilidad adecuada depende de las características de movimiento específicas 
de tu dron y del tipo de datos que necesitas para tu aplicación.
*/
//Analizar este código comparlo con el final y el que esta en el blog creo que se puede reducir!!
void MPU6050_iniciar() {
  //Configuramos el sensor para modificar parámetros internos.
  Wire.beginTransmission(MPU6050_adress);  //Prepara al sensor para recibir comandos por I2C
  Wire.write(0x6B);                        //Register 6B -> 00
  Wire.write(0x00);                        //Activo Giroscopio y desactivo Sleep Mode
  Wire.endTransmission();                  //Finaliza la transmisión I2C

  //Configuramos el Giroscopio
  /*
    Sensibilidad giroscopio:
    0x00: +/- 250  °/s : 131  LSB(°/s) 
    0x08: +/- 500  °/s : 65.5 LSB(°/s) 
    0x10: +/- 1000 °/s : 32.8 LSB(°/s) 
    0x18: +/- 2000 °/s : 16.4 LSB(°/s) 
  */
  Wire.beginTransmission(MPU6050_adress);  //Prepara al sensor para recibir comandos por I2C
  Wire.write(0x1B);                        //Register 1B -> 08
  Wire.write(0x08);                        //Giroscopio a 500dps (full scale)
  Wire.endTransmission();                  //Finaliza la transmisión I2C

  //Configuramos el acelerómetro
  /*
    Sensibilidad acelerómetro:
    0x00: +/- 2g  : 16384 LSB/g
    0x08: +/- 4g  : 8192  LSB/g
    0x10: +/- 8g  : 4096  LSB/g
    0x18: +/- 16g : 2048  LSB/g
  */
  Wire.beginTransmission(MPU6050_adress);  //Prepara al sensor para recibir comandos por I2C
  Wire.write(0x1C);                        //Register 1C -> 10
  Wire.write(0x10);                        //Acelerómetro a  +/- 8g (full scale range)
  Wire.endTransmission();                  //Finaliza la transmisión I2C

  //Esperamos...
  Wire.beginTransmission(MPU6050_adress);  //Prepara al sensor para recibir comandos por I2C
  Wire.write(0x1B);                        //Leo el contenido de 1B
  Wire.endTransmission();                  //Finaliza la transmisión I2C

  Wire.requestFrom(MPU6050_adress, 1);  //Leo 1 Byte
  while (Wire.available() < 1)
    ;  //Espera hasta que este disponible al menos 1 byte para ser leído

  /*
  SEGURIDAD AQUÍ POR SI NO SE PUEDE INICIALIZAR EL SENSOR
  */
  // Si hay un error en el sensor MPU6050 avisamos y enclavamos el programa
  if (Wire.read() != 0x08) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 error");
    while (1) {
      digitalWrite(ledError, LOW);
      delay(500);
      digitalWrite(ledError, HIGH);
      delay(500);
    }
  }

  //Configuramos el filtro pasa bajos LPF
  /*
    Frecuencia de corte del filtro pasa bajos:
    256Hz(0ms)  : 0x00
    188Hz(2ms)  : 0x01
    98Hz(3ms)   : 0x02
    42Hz(4.9ms) : 0x03
    20Hz(8.5ms) : 0x04
    10Hz(13.8ms): 0x05
    5Hz(19ms)   : 0x06
  */
  Wire.beginTransmission(MPU6050_adress);  //Prepara al sensor para recibir comandos por I2C
  Wire.write(0x1A);                        //Register 1A -> 04
  Wire.write(0x04);                        //LPF a 20Hz
  Wire.endTransmission();                  //Finaliza la transmisión I2C
}

// Calibrar giroscopio y acelerómetro. El sensor tiene que estar inmovil y en una supercifie plana.
// Leer los datos del MPU6050 5000 veces y calcular el valor medio
//Podemos despues reducirlo a 3000 veces!
void MPU6050_calibrar() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" Calibrando MPU ");

  for (int muestras = 0; muestras < 5000; muestras++) {
    MPU6050_leer();  // Leer sensor MPU6050
    gyro_X_cal += gx;
    gyro_Y_cal += gy;
    gyro_Z_cal += gz;
    acc_X_cal += ax;
    acc_Y_cal += ay;
    acc_Z_cal += az;
    delayMicroseconds(100);
  }
  gyro_X_cal = gyro_X_cal / 5000;
  gyro_Y_cal = gyro_Y_cal / 5000;
  gyro_Z_cal = gyro_Z_cal / 5000;
  acc_X_cal = acc_X_cal / 5000;
  acc_Y_cal = acc_Y_cal / 5000;
  acc_Z_cal = acc_Z_cal / 5000;

  accCalibOK = true;  //Esta variable no la utilizo, solo cambia el valor y listo, no verifica nada

  loop_timer = micros();  //Creo que se puede reducir una variable intentar hacer eso al final
}

// Función para leer sensor MPU6050
void MPU6050_leer() {
  // Los datos del giroscopio y el acelerómetro se encuentran de la dirección 3B a la 14
  Wire.beginTransmission(MPU6050_adress);  // Empezamos comunicación
  Wire.write(0x3B);                        // Pedir el registro 0x3B (AcX) // czo de la lectura de datos del acelerómetro y giroscopio
  Wire.endTransmission();                  //Finaliza la transmisión I2C

  Wire.requestFrom(MPU6050_adress, 14);  // Solicitar un total de 14 registros
  while (Wire.available() < 14)
    ;  // Esperamos hasta recibir los 14 bytes

  //Lecturas de los ejes X Y Z del acelerómetro
  ax = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  //Lectura de la temperatura
  temperature = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  //Lecturas de los ejes X Y Z del giroscopio
  gx = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //La notación Wire.read() << 8 | Wire.read() se utiliza para combinar dos bytes consecutivos
  //en un entero de 16 bits, ya que los datos del sensor están distribuidos en registros de 8 bits
  //cada uno.

  // Restar valores de calibración del acelerómetro
  if (accCalibOK == true) {
    ax -= acc_X_cal;
    ay -= acc_Y_cal;
    az -= acc_Z_cal;
    az = az + 4096;  //Debemos tener en cuenta la aceleración terrestre por eso la suma !! Ver esto y analizarlo
  }
}

// Cálculo de velocidad angular (º/s) y ángulo (º)
// Realiza el procesamiento de los datos provenientes del sensor MPU6050 para calcular la
// velocidad angular y el ángulo de inclinación del dron en función del tiempo.
void MPU6050_procesar() {
  // Restar valores de calibración del giroscopio y calcular
  // velocidad angular en º/s. Leer 65.5 en raw equivale a 1º/s
  // Divido por 65.5 ya que estoy tomando una sensibilidad del giroscopio de: +/- 500 °/s
  gyro_X = (gx - gyro_X_cal) / 65.5;
  gyro_Y = (gy - gyro_Y_cal) / 65.5;
  gyro_Z = (gz - gyro_Z_cal) / 65.5;

  // Calculamos exactamente cuánto tiempo ha pasado desde que se ha ejecutado el cálculo del ángulo.
  // Al tener señales PWM variables entre 1 y 2ms, este cálculo del ángulo no se ejecuta siempre
  // con un periodo constante.
  tiempo_ejecucion_MPU6050 = (micros() - tiempo_MPU6050_1) / 1000;  // Milisegundos

  // Calcular ángulo de inclinación con datos del giroscopio
  // grados de inclinación (º) = velocidad (º/s) * tiempo (s)
  angulo_pitch += gyro_X * tiempo_ejecucion_MPU6050 / 1000;
  angulo_roll += gyro_Y * tiempo_ejecucion_MPU6050 / 1000;
  // Tengo en cuenta la componente del movimiento en un eje que influye sobre el otro     
  // respectivamente. Donde: 0.000000266 = 1/65.5  * pi/180 * 1/1000
  angulo_pitch += angulo_roll * sin((gz - gyro_Z_cal) * tiempo_ejecucion_MPU6050 * 0.000000266);
  angulo_roll -= angulo_pitch * sin((gz - gyro_Z_cal) * tiempo_ejecucion_MPU6050 * 0.000000266);
  tiempo_MPU6050_1 = micros();

  // Calcular vector de aceleración  //Para entender las ecuaciones ver el archivo AN3461.pdf
  // 57.2958 = Conversion de radianes a grados 180/PI
  acc_total_vector = sqrt(pow(ay, 2) + pow(ax, 2) + pow(az, 2));
  angulo_pitch_acc = asin((float)ay / acc_total_vector) * 57.2958;
  angulo_roll_acc = asin((float)ax / acc_total_vector) * -57.2958;
  //angulo_yaw_acc -> necesitaria un magnetometro

  // Filtro complementario
  // Se aplica un filtro complementario para combinar la información del giroscopio y el acelerómetro.
  // Esto ayuda a reducir el error acumulado a largo plazo y mejora la precisión del ángulo calculado.
  // La combinación se realiza utilizando un promedio ponderado de los ángulos calculados
  // (angulo_pitch y angulo_roll) y los ángulos calculados a partir del acelerómetro
  // (angulo_pitch_acc y angulo_roll_acc)
  if (set_gyro_angles) {
    angulo_pitch = angulo_pitch * 0.995 + angulo_pitch_acc * 0.005;  // Angulo Pitch de inclinacion
    angulo_roll = angulo_roll * 0.995 + angulo_roll_acc * 0.005;     // Angulo Roll de inclinacion
  }                                                                  //HPF: Giroscopio | LPF: Acelerometro
  else {                                                             //Cambia el valor la primera vez y no entra más!
    angulo_pitch = angulo_pitch_acc;
    angulo_roll = angulo_roll_acc;
    set_gyro_angles = true;
  }
}

/**
 * Función que genera la señal PWM.
*/
void PWM() {
  unsigned long tiempo_ON, tiempo_1, tiempo_2, tiempo_motores_start;  //variables para medir el tiempo

  // Para generar las 4 señales PWM, el primer paso es poner estas señales a 1 (HIGH).
  digitalWrite(pin_motor1, HIGH);
  digitalWrite(pin_motor2, HIGH);
  digitalWrite(pin_motor3, HIGH);
  digitalWrite(pin_motor4, HIGH);
  tiempo_motores_start = micros();

  // --------------------------------- ¡¡1ms max!! --------------------------------------

  tiempo_1 = micros();

  recibir_los_datos();  // Recibe los datos de la radio

  // Este pequeño if reseteará los datos si la señal se pierde durante 12 mS
  if (micros() - ultimoTiempoRecibido >= 12000) {  // ¿Se perdió la señal? //Bajar este tiempo al volarlo, una vez que saquemos los delay y prints
    reiniciar_los_datos();
  }

  LED_blink();                // LED parpadeo
  Lectura_tension_bateria();  // Leer Vbat

  // Si la duracion entre tiempo_1 y tiempo_2 ha sido mayor de 900us, encender LED de aviso.
  // Nunca hay que sobrepasar 1ms de tiempo en estado HIGH.
  tiempo_2 = micros();
  tiempo_ON = tiempo_2 - tiempo_1;  //Puedo sobreescribir una variable y ahorraria más espacio en memoria!
  if (tiempo_ON > 900) digitalWrite(ledError, HIGH);

  // --------------------------------- ¡¡1ms max!! --------------------------------------

  // Pasamos las señales PWM a estado LOW cuando haya transcurrido el tiempo definido en las variables ESCx_us
  while (digitalRead(pin_motor1) == HIGH || digitalRead(pin_motor2) == HIGH || digitalRead(pin_motor3) == HIGH || digitalRead(pin_motor4) == HIGH) {
    if (tiempo_motores_start + ESC1_us <= micros()) digitalWrite(pin_motor1, LOW);
    if (tiempo_motores_start + ESC2_us <= micros()) digitalWrite(pin_motor2, LOW);
    if (tiempo_motores_start + ESC3_us <= micros()) digitalWrite(pin_motor3, LOW);
    if (tiempo_motores_start + ESC4_us <= micros()) digitalWrite(pin_motor4, LOW);
  }
}

/*
Se desactiva el control de estabilidad si el Throttle es menor a 1300uS. Si Throttle es mayor se activa.
Además se aplican limites para que los motores no salgan de sus rangos aceptables.
*/
void Modulador() {
  // Si el Throttle es menos a 1300us, el control de estabilidad se desactiva. La parte integral
  // de los controladores PID se fuerza a 0.
  if (joystickData.leftY <= 1300) {
    PID_W_Pitch_I = 0;
    PID_W_Roll_I = 0;
    PID_W_Yaw_I = 0;
    PID_ang_Pitch_I = 0;
    PID_ang_Roll_I = 0;

    ESC1_us = joystickData.leftY;
    ESC2_us = joystickData.leftY;
    ESC3_us = joystickData.leftY;
    ESC4_us = joystickData.leftY;

    // Si lo motores giran con el stick de Throttle al mínimo, recudir el valor de 950us
    if (ESC1_us < 1000) ESC1_us = 950;  //Probar con 1000, si giran reducirlo
    if (ESC2_us < 1000) ESC2_us = 950;
    if (ESC3_us < 1000) ESC3_us = 950;
    if (ESC4_us < 1000) ESC4_us = 950;
  }

  // Si el throttle es mayor a 1300us, el control de estabilidad se activa.
  else {
    // Limitar throttle a 1800 para dejar margen a los PID
    if (joystickData.leftY > 1800) joystickData.leftY = 1800;  //Ver de mejorar esto, debuggear analizar

    // Modulador
    //ESC1_us = joystickData.leftY + PID_W_Pitch_OUT - PID_W_Roll_OUT - PID_W_Yaw_OUT;  // Motor 1
    //ESC2_us = joystickData.leftY + PID_W_Pitch_OUT + PID_W_Roll_OUT + PID_W_Yaw_OUT;  // Motor 2
    //ESC3_us = joystickData.leftY - PID_W_Pitch_OUT + PID_W_Roll_OUT - PID_W_Yaw_OUT;  // Motor 3
    //ESC4_us = joystickData.leftY - PID_W_Pitch_OUT - PID_W_Roll_OUT + PID_W_Yaw_OUT;  // Motor 4
        ESC1_us = joystickData.leftY; // Solo para testeos
        ESC2_us = joystickData.leftY;
        ESC3_us = joystickData.leftY;
        ESC4_us = joystickData.leftY;

    //Limitar valores !!
    // Evitamos que alguno de los motores de detenga completamente en pleno vuelo
    if (ESC1_us < 1100) ESC1_us = 1100;
    if (ESC2_us < 1100) ESC2_us = 1100;
    if (ESC3_us < 1100) ESC3_us = 1100;
    if (ESC4_us < 1100) ESC4_us = 1100;
    // Evitamos mandar consignas mayores a 2000us a los motores
    if (ESC1_us > 2000) ESC1_us = 2000;
    if (ESC2_us > 2000) ESC2_us = 2000;
    if (ESC3_us > 2000) ESC3_us = 2000;
    if (ESC4_us > 2000) ESC4_us = 2000;
  }
}

void calcularPID(const float &error, float &P, float &I, float &D, float &PID_OUT,
                 float Kp, float Ki, float Kd, float &integral, float &prevError,
                 float sat1, float sat2) {
  float proporcional = Kp * error;              //Parte proporcional
  integral += Ki * error;                       //Parte integral (sumatoria del error en el tiempo)
  integral = constrain(integral, -sat1, sat1);  //Limitar parte integral
  float derivativa = Kd * (error - prevError);  //Parte derivativa (diferencia entre el error actual y el anterior)

  PID_OUT = proporcional + integral + derivativa;  //Salida PID
  PID_OUT = constrain(PID_OUT, -sat2, sat2);       //Limitar salida del PID

  prevError = error;
}

void PID_ang() {
  calcularPID(joystickData.rightY - angulo_pitch, PID_ang_Pitch_P, PID_ang_Pitch_I, PID_ang_Pitch_D, PID_ang_Pitch_OUT,
              Pitch_ang_Kp, Pitch_ang_Ki, Pitch_ang_Kd, PID_ang_Pitch_I, angulo_pitch_ant, PID_ang_sat1, PID_ang_sat2);

  calcularPID(joystickData.rightX - angulo_roll, PID_ang_Roll_P, PID_ang_Roll_I, PID_ang_Roll_D, PID_ang_Roll_OUT,
              Roll_ang_Kp, Roll_ang_Ki, Roll_ang_Kd, PID_ang_Roll_I, angulo_roll_ant, PID_ang_sat1, PID_ang_sat2);
}

void PID_w() {
  float pitchConsigna, rollConsigna;

  if (MODO_vuelo == 0) {
    pitchConsigna = joystickData.rightY;
    rollConsigna = joystickData.rightX;
  } else {
    pitchConsigna = PID_ang_Pitch_OUT;
    rollConsigna = PID_ang_Roll_OUT;
  }

  calcularPID(pitchConsigna - gyro_X, PID_W_Pitch_P, PID_W_Pitch_I, PID_W_Pitch_D, PID_W_Pitch_OUT,
              Pitch_W_Kp, Pitch_W_Ki, Pitch_W_Kd, PID_W_Pitch_I, gyro_X_ant, PID_W_sat1, PID_W_sat2);

  calcularPID(rollConsigna - gyro_Y, PID_W_Roll_P, PID_W_Roll_I, PID_W_Roll_D, PID_W_Roll_OUT,
              Roll_W_Kp, Roll_W_Ki, Roll_W_Kd, PID_W_Roll_I, gyro_Y_ant, PID_W_sat1, PID_W_sat2);

  calcularPID(joystickData.leftX - gyro_Z, PID_W_Yaw_P, PID_W_Yaw_I, PID_W_Yaw_D, PID_W_Yaw_OUT,
              Yaw_W_Kp, Yaw_W_Ki, Yaw_W_Kd, PID_W_Yaw_I, gyro_Z_ant, PID_W_sat1, PID_W_sat2);
}

/**
  * Función para medir la carga de la batería.
*/
void Lectura_tension_bateria() {
  float maxVoltage = 3.35;  // Voltaje máximo de la batería  //Medido desde el punto divisor de tensión
  float minVoltage = 3.22;  // Voltaje mínimo de la batería  //Medido desde el punto divisor de tensión

  // Convierte el valor analógico a voltaje
  float voltage = analogRead(pinBateria) * (maxVoltage / 4095.0);

  // Calcula el porcentaje de batería
  batteryPercentage = map(voltage * 1000, minVoltage * 1000, maxVoltage * 1000, 0, 100);

  // Apaga el LED si el nivel de batería es inferior al 20%
  if (batteryPercentage <= 30) {  //Si empieza a parpadear o se apaga la batería ya se acabo!
    digitalWrite(ledPin, HIGH);
  } else {
    if (batCont >= 100) {
      digitalWrite(ledPin, LOW);
      batCont = 0;
    }
    batCont++;
  }
}

/**
  * Función para prender y apagar un led, indicando que todo esta ejecutandose.
*/
void LED_blink() {
  loopCant++;

  if (loopCant >= 50) {
    // Cambia el estado del LED después de 50 ciclos
    digitalWrite(ledLoop, !digitalRead(ledLoop));
    loopCant = 0;  // Reinicia el contador
  }
}

//Función para testear el código y visualizar las variables por Monitor Serie
// NOTA: al visualizar variables por el Monitor Serie es posible que se sobrepase el tiempo de
// ciclo establecido debido al tiempo extra que conlleva visualizar las variables.
// Esta es la razón de desactivar esta opción en
// vuelo.
void Visualizaciones() {
  // Hay que seleccionar qué variable visualizar con visu_select
  if (visu == 1) {
    if (visu_select == 0) {
      //Valores recibidos por el mando
      Serial.print(joystickData.leftX);  //Yaw
      Serial.print(':');
      Serial.print(joystickData.leftY);  //Throttle
      Serial.print(':');
      Serial.print(joystickData.rightX);  //Roll
      Serial.print(':');
      Serial.print(joystickData.rightY);  //Pitch
      Serial.print(':');
      Serial.print(joystickData.button1Pressed);  //Botón1
      Serial.print(':');
      Serial.println(joystickData.button2Pressed);  //Botón2
    }
    if (visu_select == 1) {  //Lecturas del sensor MPU6050 : Giroscopio
      Serial.print(gyro_X);
      Serial.print("\t");
      Serial.print(gyro_Y);
      Serial.print("\t");
      Serial.println(gyro_Z);
    }
    if (visu_select == 2) {  //Lecturas del sensor MPU6050 : Acelerómetro
      Serial.print(ax / 4096);
      Serial.print("\t");
      Serial.print(ay / 4096);
      Serial.print("\t");
      Serial.println(az / 4096);
    }
    if (visu_select == 3) {  //Obtención de los ángulos
      Serial.print(angulo_pitch);
      Serial.print("\t");
      Serial.println(angulo_roll);
    }
    if (visu_select == 4) {
      Serial.print(ESC1_us);
      Serial.print("\t");
      Serial.print(ESC2_us);
      Serial.print("\t");
      Serial.print(ESC3_us);
      Serial.print("\t");
      Serial.println(ESC4_us);

      Pitch_W_Ki = 0;
      Roll_W_Ki = 0;
      Yaw_W_Ki = 0;
      Pitch_ang_Ki = 0;
      Roll_ang_Ki = 0;
    }
    if (visu_select == 5) {
      Serial.print("Tiempo ejecución MPU6050: ");
      Serial.println(tiempo_ejecucion_MPU6050);  //Tiempo en milisegundos
    }
    if (visu_select == 6) {
      // Puedes imprimir el nivel de batería si lo deseas
      Serial.print("Nivel de batería: ");
      Serial.print(batteryPercentage);
      Serial.println("%");
    }
  }
}




