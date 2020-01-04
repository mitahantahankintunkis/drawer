// Saves the state of a single stepper motor
struct stepper {
    // Pins on the arduino. The pins are ordered in such a way that
    // the stepper rotates clockwise when the pins are activated in order
    unsigned char pins[4];

    // Index used for the array above
    char step_index;

    unsigned int stepsPerRevolution;
    float angle;

    // How many steps are needed to get the arm moving after changing directions
    unsigned int slack_steps;

    // Length of the arm attached in millimeters
    float length;

    // Variables used for updating the stepper
    int steps_to_go;
    bool prev_dir;
    float millis_per_step;
    unsigned long long prev_activation;
};

typedef struct stepper Stepper;

// Primary steppr
Stepper stepper0 = {
    .pins               = { 9, 7, 8, 6 },
    .step_index         = 0,
    .stepsPerRevolution = 1800,
    .angle              = PI / 2,
    .slack_steps        = 3,
    .length             = 175.0f,
    .steps_to_go        = 0,
    .prev_dir           = true,
    .millis_per_step    = 0.0f,
    .prev_activation    = 0,
};

// Secondary stepper
Stepper stepper1 = {
    .pins               = { 10, 14, 16, 15 },
    .step_index         = 0,
    .stepsPerRevolution = 1800,
    .angle              = 0,
    .slack_steps        = 2,
    .length             = 125.0f,
    .steps_to_go        = 0,
    .prev_dir           = true,
    .millis_per_step    = 0.0f,
    .prev_activation    = 0,
};

// Updates the given stepper if it has some steps to go and if enough
// time has passed since the previous activation
void updateStepper(Stepper &stepper) {
    if (stepper.steps_to_go && millis() - stepper.prev_activation >= stepper.millis_per_step) {
        bool dir = stepper.steps_to_go < 0;

        // Removing slack
        if (stepper.prev_dir != dir) {
            stepper.steps_to_go += dir ? -stepper.slack_steps : stepper.slack_steps;
            stepper.prev_dir = dir;
        }

        // Setting the previous pin low
        digitalWrite(stepper.pins[stepper.step_index], LOW);

        // Calculating the new index
        stepper.step_index += stepper.prev_dir ? 1 : -1;
        stepper.step_index += 4;
        stepper.step_index %= 4;

        // Setting the next pin high
        digitalWrite(stepper.pins[stepper.step_index], HIGH);

        stepper.steps_to_go += stepper.prev_dir ? 1 : -1;
        stepper.prev_activation = millis();
    }
}


// Rotates both of the motors so that they arrive to the given angles at the same time
void rotateTo(float angle0, float angle1) {
    // Angle deltas
    float distance0 = angle0 - stepper0.angle;
    float distance1 = angle1 - stepper1.angle;

    stepper0.steps_to_go = stepper0.stepsPerRevolution * (distance0 / TWO_PI);
    stepper1.steps_to_go = stepper1.stepsPerRevolution * (distance1 / TWO_PI);

    stepper0.millis_per_step = 1.0f;
    stepper1.millis_per_step = 1.0f;

    // Slowing down the stepper with a shorter distance to go
    if (abs(stepper0.steps_to_go) > abs(stepper1.steps_to_go)) {
        stepper1.millis_per_step = abs(((float)stepper0.steps_to_go / (float)stepper1.steps_to_go));
    } else {
        stepper0.millis_per_step = abs(((float)stepper1.steps_to_go / (float)stepper0.steps_to_go));
    }

    float min_millis_per_step = 16.0f;
    stepper0.millis_per_step *= min_millis_per_step;
    stepper1.millis_per_step *= min_millis_per_step;

    // Calculating the real angle that the steppers are going to end up at
    stepper0.angle += (float)stepper0.steps_to_go / (float)stepper0.stepsPerRevolution * TWO_PI;
    stepper1.angle += (float)stepper1.steps_to_go / (float)stepper1.stepsPerRevolution * TWO_PI;

    // Rotating the motors
    while (stepper0.steps_to_go || stepper1.steps_to_go) {
        updateStepper(stepper0);
        updateStepper(stepper1);
    }
}


void setup() {
    for (int i = 0; i < 4; ++i) {
        pinMode(stepper0.pins[i], OUTPUT);
        pinMode(stepper1.pins[i], OUTPUT);
    }

    Serial.begin(115200);
}


void loop() {
    // Waiting for a response
    Serial.flush();
    delay(1);

    // Reading two floats (stepper0 angle, stepper1 angle) from serial
    if (Serial.available() >= sizeof(float) * 2) {
        float stepper0_angle = 0.0;
        float stepper1_angle = 0.0;
        unsigned char serialBuffer[sizeof(float) * 2];

        // Reading bytes
        if (Serial.readBytes(serialBuffer, sizeof(float) * 2) == sizeof(float) * 2) {
            memcpy(&stepper0_angle, serialBuffer, sizeof(float));
            memcpy(&stepper1_angle, serialBuffer + sizeof(float), sizeof(float));
        }

        rotateTo(stepper0_angle, stepper1_angle);
    } else {
        // Telling the PC that we are ready for new input
        Serial.write(1);
    }
}
