// Bibliothèque Arduino de base
#include <Arduino.h>
// Bibliothèques pour communiquer en I2C
#include <Wire.h>
// Pour faire des digitalRead très rapides
#include "digitalWriteFast.h"
// Permet de changer la fréquence du PWM
#include "pwmFrequency.h"
// Contantes communes à tous les moteurs
#include "constants.h"

// VARIABLES

volatile int16_t motor_encoder_count;
float motor_command; // La commande actuelle du moteur (-255 à 255)
float motor_angle; //L'angle actuel de la roue (-360 à 360)
union {
    float as_float;
    byte as_bytes[4]; // 1 float = 4 bytes
} motor_setpoint; // Consigne (en degrés par seconde)
unsigned long last_tick;

// FONCTIONS INTERRUPTIONS (doivent être les plus courtes possibles)

void i2c_receive_interrupt(int howMany) {
    digitalWriteFast(PIN_I2C_LED, !digitalReadFast(PIN_I2C_LED));
    // On lit chaque paquet de 4 bytes, qui correspondent à un float
    while(Wire.available() >= 4) {
        for(uint8_t i = 0; i < 4; i++)
            motor_setpoint.as_bytes[i] = Wire.read();
    }
}

void motor_encoder_interrupt()
{
    if(digitalReadFast(PIN_ENCODER_B) == 0)
        motor_encoder_count--;
    else
        motor_encoder_count++;
}

// FONCTION D'INITIALISATION DE LA CARTE

void setup() {
    // Connexion au bus I2C avec l'adresse I2C_DEVICE
    Wire.begin(I2C_DEVICE);
    Wire.onReceive(i2c_receive_interrupt);
    pinMode(PIN_I2C_LED, OUTPUT);

    // Setup pont H
    pinMode(PIN_HBRIDGE_FWD, OUTPUT);
    analogWrite(PIN_HBRIDGE_FWD, 0);
    setPwmFrequency(PIN_HBRIDGE_FWD, 1);
    pinMode(PIN_HBRIDGE_REV, OUTPUT);
    analogWrite(PIN_HBRIDGE_REV, 0);
    setPwmFrequency(PIN_HBRIDGE_REV, 1);

    // Setup encodeur incrémental
    pinMode(PIN_ENCODER_A, INPUT);
    pinMode(PIN_ENCODER_B, INPUT);
    attachInterrupt(INT_ENCODER, motor_encoder_interrupt, RISING);

    // Initialisation de l'asserv
    motor_encoder_count = 0;
    motor_command = 0;
    motor_angle = 0;
    motor_setpoint.as_float = 0;
    last_tick = micros();
}

// FONCTION PRINCIPALE

void loop() {
    // Désactivation des interruptions pour répupérer le compteur
    // (on évite qu'une interruption se déclenche et réécrive la valeur pendant qu'on la lit)
    uint8_t oldSREG = SREG;
    cli();
    int16_t current_encoder_count = motor_encoder_count;
    motor_encoder_count = 0;
    SREG = oldSREG;

    // Calcul de la vitesse angulaire
    float angle_delta = 360.0 * current_encoder_count / MOTOR_ppr / MOTOR_Kr;
    float new_angle = motor_angle + angle_delta;
    unsigned long current_tick = micros();
    float angular_speed = 1000.0 * 1000.0 * angle_delta / (current_tick - last_tick);

    // Correction
    float error = motor_setpoint.as_float - angular_speed;
    float new_command = motor_command + CORRECTOR_Kp * error;
    new_command = constrain(new_command, -255.0, +255.0);

    // Commande du moteur
    if(new_command > 0) {
        analogWrite(PIN_HBRIDGE_FWD, new_command); // Sortie PWM
        analogWrite(PIN_HBRIDGE_REV, 0); // Sortie à 0
    }
    else {
        analogWrite(PIN_HBRIDGE_FWD, 0); // Sortie à 0
        analogWrite(PIN_HBRIDGE_REV, fabs(new_command)); // Sortie PWM
    }

    // Stockage de l'état actuel pour la prochaine boucle
    last_tick = current_tick;
    motor_angle = fmod(new_angle, 360.0);
    motor_command = new_command;

    // Affichage e quelques valeurs pour déboguer : attention coute du temps
    Serial.print(motor_setpoint.as_float);
    Serial.print(' ');
    Serial.print(angle_delta);
    Serial.print(' ');
    Serial.print(angular_speed);
    Serial.print(' ');
    Serial.println(motor_command);
}

