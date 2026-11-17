#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "matrix.h"
#include "math.h"



float r_ = 0.0f;
float z_ = 0.0f;
float alpha_ = 0.0f;
float theta_ = 0.0f;


bool angle_limit = false;

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
    0, 0, 45, 45, 0
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

    angle = constrain(angle, 0.0f, 180.0f);

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

float offset[NUM_SERVOS] = {90.0f, 102.0f, -160.0f, 106.0f, 90.0f};

void setJointAngles()
{
    setServoAngle(0, theta[0] +offset[0]);
    setServoAngle(1, theta[1] +offset[1]);
    setServoAngle(2, -(theta[2] +offset[2]));
    setServoAngle(3, theta[3] +offset[3]);
    setServoAngle(4, theta[4] +offset[4]);
}


// ==================================================
// JOINT CALCULATIONS
// ==================================================

void calcNewAngles()
{
    float thetatt[NUM_SERVOS] = {theta[0], theta[1], theta[2], theta[3], theta[4]};
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        if(i == 2) {
            thetatt[i] += thetadot[i] * CYCLE_TIME;
            if (-(thetatt[i]+offset[i]) < 0.0f || -(thetatt[i]+offset[i]) > 180.0f) {
                angle_limit = true;
                return;
            }
        } 
        else {
            thetatt[i] += thetadot[i] * CYCLE_TIME;
            if (thetatt[i]+offset[i] < 0.0f || thetatt[i]+offset[i] > 180.0f) {
                angle_limit = true;
                return;
            }
        }
    }

    for (int i = 0; i < NUM_SERVOS; i++)
        theta[i] = thetatt[i];
    
}


// ==================================================
// VELOCITY KINEMATICS
// ==================================================


void calcVelKinamatics()
{
    // float d1 = 83.42;
    float d2 = 101.95;
    float d3 = 61.95;
    float d4 = 72.00;

    float theta1rad = theta[1] * DEG_TO_RAD;
    float theta2rad = theta[2] * DEG_TO_RAD;
    float theta3rad = theta[3] * DEG_TO_RAD;

    float J[3][3] = {
        {d2*cosf(theta1rad)+d3*cosf(theta1rad+theta2rad)+d4*cosf(theta1rad+theta2rad+theta3rad), d3*cosf(theta1rad+theta2rad)+d4*cosf(theta1rad+theta2rad+theta3rad), d4*cosf(theta1rad+theta2rad+theta3rad)},
        {-d2*sinf(theta1rad)-d3*sinf(theta1rad+theta2rad)-d4*sinf(theta1rad+theta2rad+theta3rad), -d3*sinf(theta1rad+theta2rad)-d4*sinf(theta1rad+theta2rad+theta3rad), -d4*sinf(theta1rad+theta2rad+theta3rad)},
        {-1, -1, -1}
    };

    float Xdot[3] = {
        r_, z_, alpha_
    }; //(r,z,alpha)

    float Jinv[3][3];
    float qdot[3];

    if (matrixInverse(J, Jinv))
    {
        matrixVectorMultiply(Jinv, Xdot, qdot);

        // Serial.println("Joint velocities:");

        // for (int i = 0; i < 3; i++)
        // {
        //     // Serial.println(qdot[i]);
        // }

        thetadot[1] = qdot[0] *RAD_TO_DEG;
        thetadot[2] = qdot[1] *RAD_TO_DEG;
        thetadot[3] = qdot[2] *RAD_TO_DEG;
        thetadot[0] = theta_;
    }
    else
    {
        // Serial.println("Jacobian is singular!");
    }




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

uint32_t tt = 0;

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

    r_ = -60.0f;
    z_ = 0.0f;
    alpha_ = 0.0f;
    theta_ = 0.0f;

    tt = millis();

}


// ==================================================
// MAIN LOOP
// ==================================================
int counter = 0;

void loop()
{
    if (timer20msFlag)
    {
        timer20msFlag = false;
        calcVelKinamatics();
        calcNewAngles();
        setJointAngles();
    }

    if (angle_limit) {
        r_ = 0.0f;
        z_ = 0.0f;
        alpha_ = 0.0f;
        theta_ = 0.0f;

        // if(counter == 0) {
        //     r_ = +40.0f;
        // }
        // else if(counter == 1) {
        //     z_ = -40.0f;
        // }
        // else if(counter == 2) {
        //     z_ = +40.0f;
        // }
        // counter++;
        // angle_limit = false;
    }

    if(millis() - tt < 2100) {
        r_ = -60.0f;
        z_ = 0.0f;
        alpha_ = 0.0f;
        theta_ = 0.0f;
    }
    else if (millis() - tt < 4000) {
        r_ = +60.0f;
        z_ = 0.0f;
        alpha_ = 0.0f;
        theta_ = 0.0f;
    }
    else if (millis() - tt < 6000) {
        r_ = 0.0f;
        z_ = -60.0f;
        alpha_ = 0.0f;
        theta_ = 0.0f;
    }
    else if (millis() - tt < 8000) {
        r_ = 0.0f;
        z_ = +40.0f;
        alpha_ = 0.0f;
        theta_ = 0.0f;
    }
    else{
        r_ = 0.0f;
        z_ = +0.0f;
        alpha_ = 0.0f;
        theta_ = 0.0f;
    }


}