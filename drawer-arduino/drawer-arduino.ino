// Saves a state of a single stepper motor
struct stepper {
    // Pins on the arduino
    unsigned char pins[4];

    // Activation order of the pins represented in binary
    unsigned char order[4];

    // Index used for the array above
    char step;

    unsigned int stepsPerRevolution;
    float angle;
    unsigned int slack_steps;

    // Length of the arm attached in millimeters
    float length;

    // Variables used for updating the stepper
    int steps_to_go;
    bool dir;
    float millis_per_step;
    unsigned long long prev_activation;
};

typedef struct stepper Stepper;

// Primary steppr
Stepper stepper0 = {
    .pins               = { 6, 7, 8, 9 },
    .order              = { 0b1000, 0b0010, 0b0100, 0b0001 },
    //.order              = { 0b0001, 0b0100, 0b0010, 0b1000 },
    .step               = 0,
    .stepsPerRevolution = 1800,
    .angle              = PI / 2,
    .slack_steps        = 3,
    .length             = 175.0f,
    .steps_to_go        = 0,
    .dir                = true,
    .millis_per_step    = 0.0f,
    .prev_activation    = 0,
};

// Secondary stepper
Stepper stepper1 = {
    .pins               = { 15, 14, 16, 10 },
    .order              = { 0b1000, 0b0010, 0b0100, 0b0001 },
    .step               = 0,
    .stepsPerRevolution = 1800,
    .angle              = 0,
    .slack_steps        = 2,
    .length             = 125.0f,
    .steps_to_go        = 0,
    .dir                = true,
    .millis_per_step    = 0.0f,
    .prev_activation    = 0,
};


void singleStep(Stepper &stepper) {
    // Changing direction
    if (!stepper.dir) stepper.step -= 2;

    // Wrapping index
    stepper.step += 4;
    stepper.step %= 4;

    // Most likely not needed, but prevents very short
    // duration short circuits in the H-bridges. I'm already pushing way
    // too much current for stepper0, so I'd like to not take any changes
    for (int i = 0; i < 4; ++i) {
        digitalWrite(stepper.pins[i], LOW);
    }

    // Activating pins according to the order array
    for (int i = 0; i < 4; ++i) {
        digitalWrite(stepper.pins[i], (stepper.order[stepper.step] >> i) & 1);
    }

    ++stepper.step;
}

// Updates the given stepper if it has some steps to go and if enough
// time has passed since the previous activation
void updateStepper(Stepper &stepper) {
    if (stepper.steps_to_go && millis() - stepper.prev_activation >= stepper.millis_per_step) {
        bool dir = stepper.steps_to_go < 0;

        // Waiting for the device to settle a bit if the direction has changed
        // Otherwise there's a change that the steppers don't have enough torque
        // to change directions
        if (stepper.dir != dir) {
            stepper.dir = dir;
            stepper.steps_to_go += dir ? -stepper.slack_steps : stepper.slack_steps;
        }

        singleStep(stepper);
        stepper.steps_to_go += stepper.dir ? 1 : -1;
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


// I have to move this into python, since floating point
// errors start to add up after ~15 minutes of running time.
// Arduino Pro Micro doesn't support doubles
void moveTo(float x, float y) {
    // Helper values for the formulas below
    float dist       = sqrt(x*x + y*y);
    float dist_sqr   = dist * dist;
    float s0_len_sqr = stepper0.length * stepper0.length;
    float s1_len_sqr = stepper1.length * stepper1.length;

    // Making sure that the requested point is reachable
    if (dist > stepper0.length + stepper1.length || dist < stepper0.length - stepper1.length) {
        return;
    }

    // Calculating new angles for the stepper motors
    // The two steppers and the pen form a triangle with sides of lengths
    // stepper0.length, stepper1.length and dist. We can use this information
    // to calculate the necessary angles
    // https://en.wikipedia.org/wiki/Solution_of_triangles#Three_sides_given_(SSS)
    float s0_angle =  acos((s0_len_sqr + dist_sqr - s1_len_sqr) / (2 * stepper0.length * dist));
    float s1_angle = -acos((s1_len_sqr + dist_sqr - s0_len_sqr) / (2 * stepper1.length * dist));

    // The formula above only works if stepper0 and the pen have the same
    // y coordinate. We can fix this by adding some offset to both of the angles
    float angle_offset = atan(y / x);
    s0_angle += angle_offset;
    s1_angle += angle_offset;

    // Substracting stepper1's angle fron stepper0, since stepper0
    // also rotates stepper1
    s1_angle -= s0_angle;

    // Finally rotating the motors to the calculated angles
    rotateTo(s0_angle, s1_angle);
}


void setup() {
    for (int i = 0; i < 4; ++i) {
        pinMode(stepper0.pins[i], OUTPUT);
        pinMode(stepper1.pins[i], OUTPUT);
    }

    Serial.begin(115200);
}


void loop() {
    // Waiting for response
    Serial.flush();
    delay(1);

    // Reading two floats (x, y) from serial and moving the pen
    // to those coordinates
    if (Serial.available() >= sizeof(float) * 2) {
        float a0 = 0.0;
        float a1 = 0.0;
        unsigned char serialBuffer[sizeof(float) * 2];

        if (Serial.readBytes(serialBuffer, sizeof(float) * 2) == sizeof(float) * 2) {
            memcpy(&a0, serialBuffer, sizeof(float));
            memcpy(&a1, serialBuffer + sizeof(float), sizeof(float));
        }

        rotateTo(a0, a1);
    } else {
        // Telling the PC that we are ready for new coordinates
        Serial.write(1);
    }
}

            //if (to_a1) memcpy(&a1, serialBuffer, sizeof(float));
            //else       memcpy(&a0, serialBuffer, sizeof(float));


    //int tempstepperpins[] = { 15, 14, 16, 10 };
    //int stepper0pins[] = { 6,  7,  8,  9  };

    //int tempstepperorder[] = {
    //    0, 0, 0, 1,
    //    0, 1, 0, 0,
    //    0, 0, 1, 0,
    //    1, 0, 0, 0,
    //};

    //int stepper0order[] = {
    //    0, 0, 0, 1,
    //    0, 1, 0, 0,
    //    0, 0, 1, 0,
    //    1, 0, 0, 0,
    //};

    //stepper1.step = 0;
    //stepper0.step = 0;

    //stepper1.angle = -PI / 2;
    //stepper0.angle = PI / 2;

    //stepper1.stepsPerRevolution = 1800;
    //stepper0.stepsPerRevolution = 1800;

    //for (int i = 0; i < 4; ++i) {
    //    //stepper1.pins[i] = tempstepperpins[i];
    //    stepper0.pins[i] = stepper0pins[i];
    //}

    //for (int i = 0; i < 16; ++i) {
    //    //stepper1.order[i] = tempstepperorder[i];
    //    stepper0.order[i] = stepper0order[i];
    //}

