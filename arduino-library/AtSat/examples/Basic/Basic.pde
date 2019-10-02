#include <AtSat.h>

AtSat satelite(true);

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    satelite.init();
}

void loop()
{
    float temperatura = 25.3;
    unsigned int humedad = 67;

    // Invertimos el estado del LED para hacerlo parpadear
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    // Envío sensor de temperatura
    satelite.sendSensor(20, temperatura);

    // Envío sensor de humedad
    satelite.sendSensor(21, humedad);

    delay(1000);
}

