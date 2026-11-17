#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// ==================================================
// PCA9685 CONFIGURATION
// ==================================================

#define NUM_SERVOS 5

#define I2C_SDA 8
#define I2C_SCL 9

#define PCA9685_ADDRESS 0x40

#define SERVO_FREQ 50

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

// PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// PCA9685 channels assigned to servos
const int servoChannels[NUM_SERVOS] = {
    0, 1, 2, 3, 4
};


// ==================================================
// TIMER CONFIGURATION
// ==================================================

hw_timer_t *timer = NULL;

volatile bool timer20msFlag = false;

#define CYCLE_TIME 0.02f


// ==================================================
// JOINT VARIABLES
// ==================================================

float theta[NUM_SERVOS] = {
    0, 0, 0, 0, 0
};

float thetadot[NUM_SERVOS] = {
    0, 0, 0, 0, 0
};


// ==================================================
// PCA9685 INITIALIZATION
// ==================================================

void initServos()
{
    Wire.begin(I2C_SDA, I2C_SCL);

    pwm.begin();

    pwm.setPWMFreq(SERVO_FREQ);

    delay(10);

    // Initialize all servos to their home positions
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        pwm.writeMicroseconds(servoChannels[i], 1500);
    }

    Serial.println("PCA9685 initialized!");
    Serial.println("All servos initialized!");
}


// ==================================================
// SERVO CONTROL
// ==================================================

void setServoAngle(int index, float angle)
{
    if (index < 0 || index >= NUM_SERVOS)
        return;

    // Limit servo angle
    // angle = constrain(angle, 0.0f, 180.0f);

    // Convert angle to pulse width
    float pulseWidth = SERVO_MIN_US +
        (angle / 180.0f) *
        (SERVO_MAX_US - SERVO_MIN_US);

    // Send pulse width to PCA9685
    pwm.writeMicroseconds(
        servoChannels[index],
        (uint16_t)pulseWidth
    );
}


// ==================================================
// UPDATE ALL JOINT ANGLES
// ==================================================

void setJointAngles()
{
    setServoAngle(0, theta[0] +90);
    setServoAngle(1, theta[1] +102);
    setServoAngle(2, -theta[2] +104);
    setServoAngle(3, theta[3] +106);
    setServoAngle(4, theta[4] +90);
}


// ==================================================
// JOINT CALCULATIONS
// ==================================================

void calcNewAngles()
{
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        theta[i] += thetadot[i] * CYCLE_TIME;
    }
}


// ==================================================
// VELOCITY KINEMATICS
// ==================================================

void calcVelKinamatics()
{

}


// ==================================================
// TIMER INTERRUPT
// ==================================================

void IRAM_ATTR onTimer()
{
    timer20msFlag = true;
}


// ==================================================
// SETUP
// ==================================================

void setup()
{
    Serial.begin(115200);

    // Initialize PCA9685
    initServos();

    // Set initial joint positions
    setJointAngles();

    // Initialize 20ms hardware timer
    timer = timerBegin(1000000);

    timerAttachInterrupt(timer, &onTimer);

    timerAlarm(timer, 20000, true, 0);

    Serial.println("20ms Timer Started");
    Serial.println("Robot arm initialized!");

    delay(1000);
}


// ==================================================
// MAIN LOOP
// ==================================================

void loop()
{
    if (timer20msFlag)
    {
        timer20msFlag = false;

        calcVelKinamatics();

        calcNewAngles();

        setJointAngles();
    }
}