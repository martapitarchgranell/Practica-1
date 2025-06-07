#include <Arduino.h>

const int pin_ADC = 34;  // Pin d'entrada ADC (ha de ser un GPIO amb ADC)
const int pin_DAC = 25;  // Pin de sortida DAC (només vàlid per DAC)

// Configuració inicial
void setup() {
    Serial.begin(115200);       // Iniciar comunicació sèrie
    analogReadResolution(12);   // Configurar ADC a 12 bits (0-4095)
}

// Bucle principal
void loop() {
    int valorADC = analogRead(pin_ADC); // Llegir valor analògic (0-4095)
    int valorDAC = map(valorADC, 0, 4095, 0, 255); // Convertir a 8 bits (0-255)

    dacWrite(pin_DAC, valorDAC); // Escriure el valor al DAC

    // Mostrar els valors al port sèrie
    Serial.print("Valor ADC: ");
    Serial.print(valorADC);
    Serial.print(" -> Valor DAC: ");
    Serial.println(valorDAC);

    delay(100); // Retard per evitar lectures massa ràpides
}
