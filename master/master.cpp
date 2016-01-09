#include <Arduino.h>
#include <Wire.h>
#include "ESP8266.h"

union {
    float f;
    byte bytes[4];
} motor_command[3];

int code = 0; // the code that has been read
float speed = -200.0;
ESP8266 wifi(Serial, 115200);

void setup() {
    Wire.begin(); // join i2c bus (address optional for master)
    for(uint8_t device = 1; device <= 3; device++)
        motor_command[device-1].f = 0; // transmit to device

    Serial.begin(115200);

    wifi.restart();
    wifi.enableMUX();
    wifi.startTCPServer(8080);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void stop()
{
    motor_command[0].f = 0;
    motor_command[1].f = 0;
    motor_command[2].f = 0;
}

void processCode(byte code) {
    switch(code) {
        case 0: stop(); break;
        case 1: motor_command[2].f = speed; motor_command[1].f = -speed; break;
        case 2: motor_command[0].f = speed; motor_command[1].f = -speed; break;
        case 3: motor_command[0].f = speed; motor_command[2].f = -speed; break;
        case 4: motor_command[1].f = speed; motor_command[2].f = -speed; break;
        case 5: motor_command[1].f = speed; motor_command[0].f = -speed; break;
        case 6: motor_command[2].f = speed; motor_command[0].f = -speed; break;
        case 8: motor_command[0].f = -speed; motor_command[1].f = -speed; motor_command[2].f = -speed; break;
        case 9: motor_command[0].f = speed; motor_command[1].f = speed; motor_command[2].f = speed; break;
    }
}

bool codeWasRead = false; // if the code has been read

void loop() {

    uint8_t buffer[128] = {0};
    uint8_t mux_id;
    uint32_t len = wifi.recv(&mux_id, buffer, sizeof(buffer), 100);
    for(int i = 0; i < len; i++) {
        if(buffer[i] == 1)
            digitalWrite(LED_BUILTIN, HIGH);
        if(!codeWasRead) {
            code = buffer[i];
            codeWasRead = true;
        }
        else {
            if(buffer[i] == '\n') {
                processCode(code);
                codeWasRead = false;
            }
        }
    }
    if(len == 0)
        digitalWrite(LED_BUILTIN, LOW);


    for(uint8_t device = 1; device <= 3; device++) {
        Wire.beginTransmission(device); // transmit to device

        for (uint8_t i = 0; i < 4; i++)
            Wire.write(motor_command[device-1].bytes[i]);

        Wire.endTransmission();    // stop transmitting
    }

    delay(25);
}
