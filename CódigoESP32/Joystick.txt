/**
 * En este código se agrego la correción del punto en el origen de los mandos.
 * Se arreglo el problema de inversión de los ejes
 * No Detecta las interrupciones de los botones. Botón 1 y 2 de los analógicos.
 * Se dispone la información de los analógicos en tipo estructura.
 * El programa del mando ya estaría terminado. Con las siguientes especificaciones:
 * 2 stick del joystick   -> 4 canales analógicos
 * 2 botones del joystick -> 2 canales digitales
 * Se agrego el muestreo de los valores para obtener datos suavizados.
*/
bool visu = 0;        // Visualizar variables por canal serie. En vuelo a 0!!
int visu_select = 0;  // 0: mando RC, 1: bateria

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

RF24 radio(4, 5);  // Configura CE y CSN para el módulo NRF24L01 //Puede ser cualquier pin digital

//Constantes
const int pinJoyX1 = 12;
const int pinJoyY1 = 13;
const int pinJoyButton1 = 17;

const int pinJoyX2 = 25;
const int pinJoyY2 = 26;
const int pinJoyButton2 = 16;

const int pinBateria = 34;  //Pin conectado al divisor de tensión
const int ledPin = 27;      //Pin conectado al LED
const int ledPort = 33;     //Pin habilitado el puerto serie

const int muestreo = 64;  //cantidad de veces que se muestrean los valores del adc para los joystick 

//Interrupciones
volatile bool button1Pressed = false;
volatile bool button2Pressed = false;

//Interrupción Botón 1
void IRAM_ATTR button1ISR() {
  button1Pressed = true;
}

//Interrupción Botón 2
void IRAM_ATTR button2ISR() {
  button2Pressed = true;
}

// Estructura para los datos //Puedo tener hasta 32 canales cada uno de 1 byte = 8 bits
struct JoystickData {
  int leftX;
  int leftY;
  int rightX;
  int rightY;
  bool button1Pressed;
  bool button2Pressed;
};

// Crear una instancia de la estructura
JoystickData joystickData;

int batteryPercentage;

// Programa Principal ---------------------------------------------------------------
void setup() {
  // Declaración de pines
  pinMode(ledPin, OUTPUT);
  pinMode(ledPort,OUTPUT);

  // Configurar pines para botones con resistencias pull-up internas
  pinMode(pinJoyButton1, INPUT_PULLUP);
  pinMode(pinJoyButton2, INPUT_PULLUP);
  // Configurar pines como entrada
  pinMode(pinJoyX1, INPUT);
  pinMode(pinJoyY1, INPUT);
  pinMode(pinJoyX2, INPUT);
  pinMode(pinJoyY2, INPUT);

  pinMode(pinBateria, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinJoyButton1), button1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinJoyButton2), button2ISR, FALLING);

  radio.begin();
  // Número de canal (puede ser cualquier valor entre 0 y 125) Frecuencia = 2400 + Canal
  uint8_t channel = 105;  // Cambiar este valor según las necesidades
  radio.setChannel(channel);
  // Dirección del canal de comunicación
  radio.openWritingPipe(0xA1B2C3E4F0LL);
  // Configura la velocidad de datos a 250Kbps
  radio.setDataRate(RF24_250KBPS);
  // Potencia de salida
  radio.setPALevel(RF24_PA_MIN);  // Establece la potencia de salida mínima --> luego cambiar por RF24_PA_MAX
  radio.stopListening();  // Cambia al modo de transmisión: modo "No escucha"

  if (visu == 1) {
    // Solo iniciamos el monitor serie y aguno de los modos de visualización está activo
    Serial.begin(9600);
    digitalWrite(ledPort, HIGH); //HIGH
  }
}

void loop() {
  // Declarar variables locales para almacenar las sumas de las muestras
  int sumaX1 = 0;
  int sumaY1 = 0;
  int sumaX2 = 0;
  int sumaY2 = 0;

  // Actualizar los valores de la estructura
  // Se intercambiaron los ejes debido a que con las conexiones correctas se produce inversión !!! -> Parece ser culpa de la función map
  // Leer valores del joystick y aplicar el muestreo
  
  // Tomar muestreo
  for(int i = 0; i < muestreo; i++) {
    // Sumar los valores leídos del joystick
    sumaX1 += analogRead(pinJoyY1);
    sumaY1 += analogRead(pinJoyX1);
    sumaX2 += analogRead(pinJoyY2);
    sumaY2 += analogRead(pinJoyX2);
  }
  sumaX1 /= muestreo;
  sumaY1 /= muestreo;
  sumaX2 /= muestreo;
  sumaY2 /= muestreo;

  if(sumaX1 > 4095){sumaX1 = 4095;}
  if(sumaY1 > 4095){sumaX1 = 4095;}
  if(sumaX2 > 4095){sumaX1 = 4095;}
  if(sumaY2 > 4095){sumaX1 = 4095;}

  // Valor máximo que puede tomar Yaw = 30; valor mínimo = -30
  joystickData.leftX = map(sumaX1, 0, 4095, -30, 30);
  // Valor máximo que puede tomar Throttle = 2000; valor mínimo = 1000
  joystickData.leftY = map(sumaY1, 0, 4095, 1000, 2000);
  // Valor máximo que puede tomar Roll = 30; valor mínimo = -30
  joystickData.rightX = map(sumaX2, 0, 4095, -30, 30);
  // Valor máximo que puede tomar Pitch = -30; valor mínimo = 30
  joystickData.rightY = map(sumaY2, 0, 4095, 30, -30);

  // Comprobar si el joystick está cerca del punto de reposo y ajustar los valores
  // Ver si esto se puede mejorar un poco, creo que se puede reducir y que quede más profesional
  if (abs(joystickData.leftX - 0) < 3) {
    joystickData.leftX = 0;
  }
  if (abs(joystickData.leftY - 1500) < 35) {
    joystickData.leftY = 1500;
  }
  if (abs(joystickData.rightX - 0) < 3) {
    joystickData.rightX = 0;
  }
  if (abs(joystickData.rightY - 0) < 3) {
    joystickData.rightY = 0;
  }

  // Actualizar los valores de los botones en la estructura
  joystickData.button1Pressed = button1Pressed;
  joystickData.button2Pressed = button2Pressed;

  // Envía los valores al receptor
  // Enviar la estructura
  radio.write(&joystickData, sizeof(joystickData));

  //Acomodo los botones
  // Acomodo los botones
  button1Pressed = (button1Pressed) ? false : button1Pressed;
  button2Pressed = (button2Pressed) ? false : button2Pressed;

  Lectura_tension_bateria();
  //delay(200); // Reducido para mejorar la capacidad de respuesta

  // Visualización de variables
  if (visu == 1) Visualizaciones();
}

void Lectura_tension_bateria() {
  float maxVoltage = 6;  // Voltaje máximo de la batería  //Medido desde el punto divisor de tensión
  float minVoltage = 4.1;  // Voltaje mínimo de la batería  //Medido desde el punto divisor de tensión	

  // Convierte el valor analógico a voltaje
  float voltage = analogRead(pinBateria) * (maxVoltage / 4095.0);

  // Calcula el porcentaje de batería
  batteryPercentage = map(voltage * 1000, minVoltage * 1000, maxVoltage * 1000, 0, 100);

  // Apaga el LED si el nivel de batería es inferior al 20%
  if (batteryPercentage < 30) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}

void Visualizaciones() {
  // Visualizar variables por canal serie
  // Hay que seleccionar qué variable visualizar con visu_select
  if (visu == 1) {
    if (visu_select == 0) {
      //Me permite graficar los valores a usar enviar
      Serial.print(joystickData.leftX);  //Yaw
      Serial.print(':');
      Serial.print(joystickData.leftY);  //Throttle
      Serial.print(':');
      Serial.print(joystickData.rightX);  //Roll
      Serial.print(':');
      Serial.print(joystickData.rightY);  //Pitch
      Serial.print(':');
      Serial.print(joystickData.button1Pressed);
      Serial.print(':');
      Serial.println(joystickData.button2Pressed);
    }

    if (visu_select == 1) {
      // Puedes imprimir el nivel de batería si lo deseas
      Serial.print("Nivel de batería: ");
      Serial.print(batteryPercentage);
      Serial.println("%");
    }
  }
}




