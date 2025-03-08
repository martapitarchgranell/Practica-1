

Modificar el programa para que incluya el envio de datos (ON y OFF) al puerto serie. Añadir la iunicialización del puerto serie y el envio cada vez que cambia el estado del led

Iniciar pin de led como salida
Iniciar el terminal serie
bucle infinito
encender led
sacar por puerto serie mensaje ON
espera de 1000 milisegundos
apagar led
sacar por puesto serie mensaje OFF
espera de 1000 milisegundos

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>  

int led = 2;  

void setup() {
  pinMode(led, OUTPUT);
  Serial.begin(115200);  

  while (!Serial) {
    ;  
  }
}
void loop() {
 
  digitalWrite(led, HIGH);
  Serial.println("ON");  
  delay(1000);  

  digitalWrite(led, LOW);
  Serial.println("OFF");  
  delay(1000);  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
3. Modificar el programa para que actue directamente sobre los registros de los puertos de entrada y salida

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h> 
// #define LED_BUILTIN 2
#define DELAY 500


// Definir el pin de salida
#define LED_PIN 2


void setup() {
// Configurar el pin del LED como salida utilizando el registro GPIO
pinMode(LED_PIN, OUTPUT);

// Iniciar el puerto serie
Serial.begin(115200);
}

void loop() {
// Caso 1: Con envío por puerto serie y funciones de Arduino
digitalWrite(LED_PIN, HIGH);
Serial.println("ON");
digitalWrite(LED_PIN, LOW);
Serial.println("OFF");

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
4. Eliminar los delay modificar el pin de salida a uno cualquiera de los que estan disponibles i medir con el osciloscopio cual es la màxima frecuencia de apagado encendido que permite el microcontrolador. Medir la frecuencia en estos cuatro casos:
Con el envio por el puerto série del mensaje i utilizando las funciones de Arduino Con el envio por el puerto série y accedirendo directamente a los registros
Sin el envio por el puerto série del mensaje i utilizando las funciones de Arduino Sin el envio por el puerto série y accedirendo directamente a los registros


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#define GPIO_OUT_REG   0x3FF44004  // Registro de salida de GPIO
#define GPIO_IN_REG    0x3FF44000  // Registro de entrada de GPIO
#define GPIO_ENABLE_REG 0x3FF44008 // Registro de habilitación de GPIO

#define GPIO2 (1 << 2)    // Pin GPIO2 para salida (LED)
#define GPIO15 (1 << 15)  // Pin GPIO15 para entrada (botón)


// Definir el pin donde está conectado el LED
int led = 2;  // GPIO2 en la mayoría de las placas ESP32

// Configurar el pin GPIO para salida
void setupPinAsOutput(int pin) {
  volatile uint32_t *gpio_enable = (uint32_t *)GPIO_ENABLE_REG;
  *gpio_enable |= pin;  // Habilitar el pin como salida
}

// Configurar el pin GPIO para entrada
void setupPinAsInput(int pin) {
  volatile uint32_t *gpio_enable = (uint32_t *)GPIO_ENABLE_REG;
  *gpio_enable &= ~pin;  // Habilitar el pin como entrada
}

// La función setup se ejecuta una vez al inicio:
void setup() {
  // Inicializar el pin como salida
  pinMode(led, OUTPUT);

  // Inicializar el puerto serie para la comunicación con la PC
  Serial.begin(115200);  // La velocidad de transmisión en baudios (115200 es una opción común)
  
  // Esperar un poco para asegurarse de que el puerto serie esté listo
  while (!Serial) {
    ;  // Esperar por la conexión serie
  }
} 

// La función loop se ejecuta en un ciclo continuo:
void loop() {
  // Encender el LED (HIGH es el nivel de voltaje)
  digitalWrite(led, HIGH);
  Serial.println("ON");  // Enviar mensaje "ON" por el puerto serie
  
  // Apagar el LED (LOW es el nivel de voltaje)
  digitalWrite(led, LOW);
  Serial.println("OFF");  // Enviar mensaje "OFF" por el puerto serie
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
5. Generar un informe fichero informe.MD ( markdown ) donde se muestre el codigo, un diagrama de flujo y un diagrama de tiempos
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>  // Incluir la librería de Arduino

// Definir el pin donde está conectado el LED
int led = 2;  // GPIO2 en la mayoría de las placas ESP32

// La función setup se ejecuta una vez al inicio
void setup() {
  // Inicializar el pin como salida
  pinMode(led, OUTPUT);

  // Inicializar el puerto serie para la comunicación con la PC
  Serial.begin(115200);  // La velocidad de transmisión en baudios (115200 es una opción común)

  // Esperar un poco para asegurarse de que el puerto serie esté listo
  while (!Serial) {
    ;  // Esperar por la conexión serie
  }
}

// La función loop se ejecuta en un ciclo continuo
void loop() {
  // Encender el LED (HIGH es el nivel de voltaje)
  digitalWrite(led, HIGH);
  delay(1000);  // Esperar 1 segundo

  // Apagar el LED (LOW es el nivel de voltaje)
  digitalWrite(led, LOW);
  delay(1000);  // Esperar 1 segundo
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DIAGRAMA DE FLUJO 

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

EXERCICI VOLUNTARI

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>

int led = 1;

#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

void setup() { 
  Serial.begin(115200);
  pinMode(led, OUTPUT);
}

void loop() {
  digitalWrite(led, HIGH);
  Serial.print("Led On  ");
  delay(500);
  digitalWrite(led, LOW);
  Serial.print("Led Off ");
  delay(500);

   Serial.print("Temperature: ");

   Serial.print((temprature_sens_read() - 32) / 1.8);
   Serial.println(" C");
   delay(5000);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
