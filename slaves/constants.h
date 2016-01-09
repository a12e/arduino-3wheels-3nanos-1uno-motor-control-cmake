// La LED utilisée pour dire qu'un message I2C est arrivé
#define PIN_I2C_LED A0

// Les pins de l'encodeur du moteur
#define PIN_ENCODER_A 2
#define PIN_ENCODER_B 3

// Numéro de l'interruption de l'encodeur (dépend de PIN_ENCODER_A)
#define INT_ENCODER 0

// Les pins du pont H
#define PIN_HBRIDGE_FWD 5
#define PIN_HBRIDGE_REV 6

// Valeurs du moteur
// > rapport de réduction du réducteur en aval du moteur
#define MOTOR_Kr 18.75
// > nombre de fronts détéctés par tour
#define MOTOR_ppr 500

// Valeurs du correcteur
// > coefficient proportionnel
#define CORRECTOR_Kp 0.9
