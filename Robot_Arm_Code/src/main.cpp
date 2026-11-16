#include <Arduino.h>

// Servo configuration
#define NUM_SERVOS 5

const int servoPins[NUM_SERVOS] = {
    1, 2, 42, 40, 38
};

#define SERVO_FREQ       50
#define SERVO_RESOLUTION 14

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

#define PWM_PERIOD_US 20000

// Servo angles
float theta[NUM_SERVOS] = {
    0, 0, 0, 0, 0
};

float thetadot[NUM_SERVOS] = {
    0, 0, 0, 0, 0
};

// Timer configuration
hw_timer_t *timer = NULL;

volatile bool timer20msFlag = false;

#define CYCLE_TIME 0.02f




// --------------------------------------------------
// SERVO INITIALIZATION
// --------------------------------------------------

void initServos()
{
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        bool success = ledcAttach(
            servoPins[i],
            SERVO_FREQ,
            SERVO_RESOLUTION
        );

        if (!success)
        {
            Serial.printf("Servo %d initialization failed!\n", i);
        }
    }
}


// --------------------------------------------------
// SERVO CONTROL
// --------------------------------------------------

void setServoAngle(int index, float angle)
{
    if (index < 0 || index >= NUM_SERVOS)
        return;

    angle = constrain(angle, 0.0f, 180.0f);

    // Convert angle to pulse width
    float pulseWidth = SERVO_MIN_US +
        (angle / 180.0f) *
        (SERVO_MAX_US - SERVO_MIN_US);

    // Convert pulse width to LEDC duty
    uint32_t maxDuty = (1UL << SERVO_RESOLUTION) - 1;

    uint32_t duty = (uint32_t)(
        pulseWidth * maxDuty / PWM_PERIOD_US
    );

    ledcWrite(servoPins[index], duty);
}


// --------------------------------------------------
// UPDATE ALL SERVOS
// --------------------------------------------------

void setJointAngles()
{
    setServoAngle(0, theta[0]+90.0f);
    setServoAngle(1, theta[1]+86.0f);
    setServoAngle(2, -theta[2]-90.0f);
    setServoAngle(3, theta[3]+92.0f);
    setServoAngle(4, theta[4]+90.0f);
}


// --------------------------------------------------
// JOINT CALCULATIONS
// --------------------------------------------------

void calcNewAngles()
{
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        theta[i] += thetadot[i] * CYCLE_TIME;
    }
}



// --------------------------------------------------
// Velocity Kinamatics
// --------------------------------------------------

void calcVelKinamatics() {

}




// --------------------------------------------------
// TIMER INTERRUPT
// --------------------------------------------------

void IRAM_ATTR onTimer()
{
    timer20msFlag = true;
    
}


// --------------------------------------------------
// SETUP
// --------------------------------------------------

void setup()
{
    Serial.begin(115200);

    initServos();

    setJointAngles();

    // Initialize 20ms timer
    timer = timerBegin(1000000);

    timerAttachInterrupt(timer, &onTimer);

    timerAlarm(timer, 20000, true, 0);

    Serial.println("Robot arm initialized!");
}


// --------------------------------------------------
// MAIN LOOP
// --------------------------------------------------

void loop()
{
    if (timer20msFlag) {
        timer20msFlag = false;    
        calcVelKinamatics();
        calcNewAngles();
        setJointAngles();    
    }

}