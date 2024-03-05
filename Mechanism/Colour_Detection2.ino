#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Example code for the Adafruit TCS34725 breakout library */

/* Connect SCL to analog 5
Connect SDA to analog 4
Connect VDD to 3.3V DC
Connect GROUND to common ground */

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

void setup(void) {
Serial.begin(9600);

if (tcs.begin()) {
Serial.println("Found sensor");
} else {
Serial.println("No TCS34725 found ... check your connections");
while (1);
}

// Now we're ready to get readings!
}

void color_detection() {
    uint16_t colorTemp;
    // uint16_t r, g, b, c, lux;

    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    // lux = tcs.calculateLux(r, g, b);


    if (colorTemp > 100000) {
        // turn on red LED for 3 seconds
        Serial.println("Red");
        digitalWrite(LED_Red, HIGH);
        delay(3000);
        cube_color = 0
    }
    else if (colorTemp > 10000) {
        // turn on green LED for 3 seconds
        Serial.println("Black");
        digitalWrite(LED_Green, HIGH);
        delay(3000);
        cube_color = 1
    }
    else if (colorTemp > 1000) {
        // turn on blue LED for 3 seconds
        Serial.println("Nothing");
        digitalWrite(LED_Blue, HIGH);
        delay(3000);
        cube_color = 2
    }
    else {
        Serial.println("Error");
    }
}
// Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
// Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
// Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
// Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
// Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
// Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
// Serial.println(" ");
// }


// const uint8_t LED_Red = 7;
// const uint8_t LED_Green = 8;
// const uint8_t LED_Blue = 10;




