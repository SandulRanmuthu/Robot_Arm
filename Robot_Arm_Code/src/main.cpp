// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>
// #include "matrix.h"
// #include "math.h"



// float r_ = 0.0f;
// float z_ = 0.0f;
// float alpha_ = 0.0f;
// float theta_ = 0.0f;


// bool angle_limit = false;

// // ==================================================
// // PCA9685 CONFIGURATION
// // ==================================================

// #define NUM_SERVOS 5

// #define I2C_SDA 8
// #define I2C_SCL 9

// #define PCA9685_ADDRESS 0x40

// #define SERVO_FREQ 50

// #define SERVO_MIN_US 500
// #define SERVO_MAX_US 2500

// // PCA9685 object
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// // PCA9685 channels assigned to servos
// const int servoChannels[NUM_SERVOS] = {
//     0, 1, 2, 3, 4
// };


// // ==================================================
// // TIMER CONFIGURATION
// // ==================================================

// hw_timer_t *timer = NULL;

// volatile bool timer20msFlag = false;

// #define CYCLE_TIME 0.02f


// // ==================================================
// // JOINT VARIABLES
// // ==================================================

// float theta[NUM_SERVOS] = {
//     0, 0, 45, 45, 0
// };

// float thetadot[NUM_SERVOS] = {
//     0, 0, 0, 0, 0
// };


// // ==================================================
// // PCA9685 INITIALIZATION
// // ==================================================

// void initServos()
// {
//     Wire.begin(I2C_SDA, I2C_SCL);

//     pwm.begin();

//     pwm.setPWMFreq(SERVO_FREQ);

//     delay(10);

//     // Initialize all servos to their home positions
//     for (int i = 0; i < NUM_SERVOS; i++)
//     {
//         pwm.writeMicroseconds(servoChannels[i], 1500);
//     }

//     Serial.println("PCA9685 initialized!");
//     Serial.println("All servos initialized!");
// }


// // ==================================================
// // SERVO CONTROL
// // ==================================================

// void setServoAngle(int index, float angle)
// {
//     if (index < 0 || index >= NUM_SERVOS)
//         return;

//     angle = constrain(angle, 0.0f, 180.0f);

//     // Convert angle to pulse width
//     float pulseWidth = SERVO_MIN_US +
//         (angle / 180.0f) *
//         (SERVO_MAX_US - SERVO_MIN_US);

//     // Send pulse width to PCA9685
//     pwm.writeMicroseconds(
//         servoChannels[index],
//         (uint16_t)pulseWidth
//     );
// }


// // ==================================================
// // UPDATE ALL JOINT ANGLES
// // ==================================================

// float offset[NUM_SERVOS] = {90.0f, 102.0f, -160.0f, 106.0f, 90.0f};

// void setJointAngles()
// {
//     setServoAngle(0, theta[0] +offset[0]);
//     setServoAngle(1, theta[1] +offset[1]);
//     setServoAngle(2, -(theta[2] +offset[2]));
//     setServoAngle(3, theta[3] +offset[3]);
//     setServoAngle(4, theta[4] +offset[4]);
// }


// // ==================================================
// // JOINT CALCULATIONS
// // ==================================================

// void calcNewAngles()
// {
//     float thetatt[NUM_SERVOS] = {theta[0], theta[1], theta[2], theta[3], theta[4]};
//     for (int i = 0; i < NUM_SERVOS; i++)
//     {
//         if(i == 2) {
//             thetatt[i] += thetadot[i] * CYCLE_TIME;
//             if (-(thetatt[i]+offset[i]) < 0.0f || -(thetatt[i]+offset[i]) > 180.0f) {
//                 angle_limit = true;
//                 return;
//             }
//         } 
//         else {
//             thetatt[i] += thetadot[i] * CYCLE_TIME;
//             if (thetatt[i]+offset[i] < 0.0f || thetatt[i]+offset[i] > 180.0f) {
//                 angle_limit = true;
//                 return;
//             }
//         }
//     }

//     for (int i = 0; i < NUM_SERVOS; i++)
//         theta[i] = thetatt[i];
    
// }


// // ==================================================
// // VELOCITY KINEMATICS
// // ==================================================


// void calcVelKinamatics()
// {
//     // float d1 = 83.42;
//     float d2 = 101.95;
//     float d3 = 61.95;
//     float d4 = 72.00;

//     float theta1rad = theta[1] * DEG_TO_RAD;
//     float theta2rad = theta[2] * DEG_TO_RAD;
//     float theta3rad = theta[3] * DEG_TO_RAD;

//     float J[3][3] = {
//         {d2*cosf(theta1rad)+d3*cosf(theta1rad+theta2rad)+d4*cosf(theta1rad+theta2rad+theta3rad), d3*cosf(theta1rad+theta2rad)+d4*cosf(theta1rad+theta2rad+theta3rad), d4*cosf(theta1rad+theta2rad+theta3rad)},
//         {-d2*sinf(theta1rad)-d3*sinf(theta1rad+theta2rad)-d4*sinf(theta1rad+theta2rad+theta3rad), -d3*sinf(theta1rad+theta2rad)-d4*sinf(theta1rad+theta2rad+theta3rad), -d4*sinf(theta1rad+theta2rad+theta3rad)},
//         {-1, -1, -1}
//     };

//     float Xdot[3] = {
//         r_, z_, alpha_
//     }; //(r,z,alpha)

//     float Jinv[3][3];
//     float qdot[3];

//     if (matrixInverse(J, Jinv))
//     {
//         matrixVectorMultiply(Jinv, Xdot, qdot);

//         // Serial.println("Joint velocities:");

//         // for (int i = 0; i < 3; i++)
//         // {
//         //     // Serial.println(qdot[i]);
//         // }

//         thetadot[1] = qdot[0] *RAD_TO_DEG;
//         thetadot[2] = qdot[1] *RAD_TO_DEG;
//         thetadot[3] = qdot[2] *RAD_TO_DEG;
//         thetadot[0] = theta_;
//     }
//     else
//     {
//         // Serial.println("Jacobian is singular!");
//     }




// }


// // ==================================================
// // TIMER INTERRUPT
// // ==================================================

// void IRAM_ATTR onTimer()
// {
//     timer20msFlag = true;
// }


// // ==================================================
// // SETUP
// // ==================================================

// uint32_t tt = 0;

// void setup()
// {
//     Serial.begin(115200);

//     // Initialize PCA9685
//     initServos();

//     // Set initial joint positions
//     setJointAngles();

//     // Initialize 20ms hardware timer
//     timer = timerBegin(1000000);

//     timerAttachInterrupt(timer, &onTimer);

//     timerAlarm(timer, 20000, true, 0);

//     Serial.println("20ms Timer Started");
//     Serial.println("Robot arm initialized!");

//     delay(1000);

//     r_ = -60.0f;
//     z_ = 0.0f;
//     alpha_ = 0.0f;
//     theta_ = 0.0f;

//     tt = millis();

// }


// // ==================================================
// // MAIN LOOP
// // ==================================================
// int counter = 0;

// void loop()
// {
//     if (timer20msFlag)
//     {
//         timer20msFlag = false;
//         calcVelKinamatics();
//         calcNewAngles();
//         setJointAngles();
//     }

//     if (angle_limit) {
//         r_ = 0.0f;
//         z_ = 0.0f;
//         alpha_ = 0.0f;
//         theta_ = 0.0f;

//         // if(counter == 0) {
//         //     r_ = +40.0f;
//         // }
//         // else if(counter == 1) {
//         //     z_ = -40.0f;
//         // }
//         // else if(counter == 2) {
//         //     z_ = +40.0f;
//         // }
//         // counter++;
//         // angle_limit = false;
//     }

//     if(millis() - tt < 2100) {
//         r_ = -60.0f;
//         z_ = 0.0f;
//         alpha_ = 0.0f;
//         theta_ = 0.0f;
//     }
//     else if (millis() - tt < 4000) {
//         r_ = +60.0f;
//         z_ = 0.0f;
//         alpha_ = 0.0f;
//         theta_ = 0.0f;
//     }
//     else if (millis() - tt < 6000) {
//         r_ = 0.0f;
//         z_ = -60.0f;
//         alpha_ = 0.0f;
//         theta_ = 0.0f;
//     }
//     else if (millis() - tt < 8000) {
//         r_ = 0.0f;
//         z_ = +40.0f;
//         alpha_ = 0.0f;
//         theta_ = 0.0f;
//     }
//     else{
//         r_ = 0.0f;
//         z_ = +0.0f;
//         alpha_ = 0.0f;
//         theta_ = 0.0f;
//     }


// }
















#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>


// ============================================================
// ROBOT CONFIGURATION
// ============================================================

#define NUM_SERVOS 5

#define I2C_SDA 8
#define I2C_SCL 9

#define PCA9685_ADDRESS 0x40

#define SERVO_FREQ 50

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

#define CYCLE_TIME 0.02f

// Link dimensions (mm)

const float d1 = 83.42f;
const float a2 = 101.95f;
const float a3 = 61.95f;
const float a4 = 72.00f;


// ============================================================
// SERVO CONFIGURATION
// ============================================================

Adafruit_PWMServoDriver pwm =
    Adafruit_PWMServoDriver(PCA9685_ADDRESS);

const int servoChannels[NUM_SERVOS] = {
    0, 1, 2, 3, 4
};

// Servo offsets

const float offset[NUM_SERVOS] = {
    90.0f,
    102.0f,
    -160.0f,
    106.0f,
    90.0f
};

// Gripper physical servo angles
// Adjust these according to your gripper mechanism.

#define GRIPPER_OPEN_DEG    30.0f
#define GRIPPER_CLOSED_DEG  75.0f


// ============================================================
// JOINT VARIABLES
// ============================================================

// Initial logical joint angles.
// Set these to the actual physical starting configuration.
// The example configuration has alpha = 0 degrees.

float theta[NUM_SERVOS] = {
    0.0f,
    30.0f,
    60.0f,
    0.0f,
    0.0f
};

float thetadot[NUM_SERVOS] = {
    0, 0, 0, 0, 0
};


// ============================================================
// CYLINDRICAL POSITION STRUCTURE
// ============================================================

struct CylindricalPosition
{
    float r;
    float base;
    float z;
    float alpha;
};


// ============================================================
// TARGET POSITIONS
// ============================================================

const CylindricalPosition START_POS = {
    100.0f,
    0.0f,
    200.0f,
    0.0f
};

const CylindricalPosition PICKUP_POS = {
    200.0f,
    0.0f,
    40.0f,
    0.0f
};

const CylindricalPosition DROPOFF_POS = {
    100.0f,
    90.0f,
    220.0f,
    0.0f
};


// ============================================================
// MOTION CONFIGURATION
// ============================================================

// Movement durations

#define START_MOVE_TIME     3000
#define PICKUP_MOVE_TIME    2500
#define DROPOFF_MOVE_TIME   3000

#define GRIPPER_DELAY       1000

// Select elbow configuration
// Change to false if the other configuration is needed.

#define ELBOW_POSITIVE true


// ============================================================
// TIMER CONFIGURATION
// ============================================================

hw_timer_t *timer = NULL;

volatile bool timer20msFlag = false;


void IRAM_ATTR onTimer()
{
    timer20msFlag = true;
}


// ============================================================
// MOTION STATE MACHINE
// ============================================================

enum RobotState
{
    MOVE_TO_START,
    MOVE_TO_PICKUP,
    CLOSE_GRIPPER,
    MOVE_TO_DROPOFF,
    OPEN_GRIPPER,
    FINISHED,
    ERROR
};

RobotState state;

bool motionActive = false;

uint32_t motionStartTime = 0;
uint32_t stateStartTime = 0;

uint32_t motionDuration = 0;

CylindricalPosition motionStart;
CylindricalPosition motionTarget;


// ============================================================
// SERVO INITIALIZATION
// ============================================================

void initServos()
{
    Wire.begin(I2C_SDA, I2C_SCL);

    pwm.begin();

    pwm.setPWMFreq(SERVO_FREQ);

    delay(10);

    Serial.println("PCA9685 initialized!");
}


// ============================================================
// SERVO CONTROL
// ============================================================

void setServoAngle(int index, float angle)
{
    if (index < 0 || index >= NUM_SERVOS)
        return;

    angle = constrain(angle, 0.0f, 180.0f);

    float pulseWidth = SERVO_MIN_US +
        (angle / 180.0f) *
        (SERVO_MAX_US - SERVO_MIN_US);

    pwm.writeMicroseconds(
        servoChannels[index],
        (uint16_t)pulseWidth
    );
}


// ============================================================
// UPDATE JOINT ANGLES
// ============================================================

void setJointAngles()
{
    setServoAngle(0, theta[0] + offset[0]);

    setServoAngle(1, theta[1] + offset[1]);

    setServoAngle(2, -(theta[2] + offset[2]));

    setServoAngle(3, theta[3] + offset[3]);
}


// ============================================================
// GRIPPER CONTROL
// ============================================================

void openGripper()
{
    setServoAngle(4, GRIPPER_OPEN_DEG);

    Serial.println("Gripper OPEN");
}


void closeGripper()
{
    setServoAngle(4, GRIPPER_CLOSED_DEG);

    Serial.println("Gripper CLOSED");
}


// ============================================================
// FORWARD KINEMATICS
// ============================================================

CylindricalPosition forwardKinematics()
{
    float q1 = theta[1] * DEG_TO_RAD;

    float q2 = theta[2] * DEG_TO_RAD;

    float q3 = theta[3] * DEG_TO_RAD;

    float phi1 = q1;

    float phi2 = q1 + q2;

    float phi3 = q1 + q2 + q3;


    CylindricalPosition position;

    position.r =
        a2 * sinf(phi1) +
        a3 * sinf(phi2) +
        a4 * sinf(phi3);

    position.z =
        d1 +
        a2 * cosf(phi1) +
        a3 * cosf(phi2) +
        a4 * cosf(phi3);

    position.base = theta[0];

    position.alpha =
        theta[1] + theta[2] + theta[3] - 90.0f;

    return position;
}


// ============================================================
// INVERSE KINEMATICS
// ============================================================

bool inverseKinematics(
    CylindricalPosition pos,
    float result[4]
)
{
    // ----------------------------------------
    // 1. Calculate wrist position
    // ----------------------------------------

    float alphaRad = pos.alpha * DEG_TO_RAD;

    float rw = pos.r - a4 * cosf(alphaRad);

    float zw = pos.z - d1 + a4 * sinf(alphaRad);


    // ----------------------------------------
    // 2. Calculate elbow angle
    // ----------------------------------------

    float D =
        (rw * rw + zw * zw - a2 * a2 - a3 * a3)
        /
        (2.0f * a2 * a3);


    // Workspace check

    if (D < -1.0f || D > 1.0f)
    {
        return false;
    }

    D = constrain(D, -1.0f, 1.0f);

    float q2 = acosf(D);

    if (!ELBOW_POSITIVE)
    {
        q2 = -q2;
    }


    // ----------------------------------------
    // 3. Calculate shoulder angle
    // ----------------------------------------

    float q1 =
        atan2f(rw, zw) -
        atan2f(
            a3 * sinf(q2),
            a2 + a3 * cosf(q2)
        );


    // ----------------------------------------
    // 4. Calculate wrist angle
    // ----------------------------------------

    float q3 =
        90.0f + pos.alpha -
        (q1 + q2) * RAD_TO_DEG;


    // ----------------------------------------
    // 5. Convert to degrees
    // ----------------------------------------

    float baseDeg = pos.base;

    float shoulderDeg = q1 * RAD_TO_DEG;

    float elbowDeg = q2 * RAD_TO_DEG;

    float wristDeg = q3;


    // ----------------------------------------
    // 6. Calculate physical servo angles
    // ----------------------------------------

    float servoAngle[4];

    servoAngle[0] = baseDeg + offset[0];

    servoAngle[1] = shoulderDeg + offset[1];

    servoAngle[2] = -(elbowDeg + offset[2]);

    servoAngle[3] = wristDeg + offset[3];


    // ----------------------------------------
    // 7. Servo limit checking
    // ----------------------------------------

    for (int i = 0; i < 4; i++)
    {
        if (servoAngle[i] < 0.0f ||
            servoAngle[i] > 180.0f)
        {
            return false;
        }
    }


    // ----------------------------------------
    // 8. Store joint angles
    // ----------------------------------------

    result[0] = baseDeg;

    result[1] = shoulderDeg;

    result[2] = elbowDeg;

    result[3] = wristDeg;

    return true;
}


// ============================================================
// APPLY INVERSE KINEMATICS
// ============================================================

bool setCartesianPosition(CylindricalPosition pos)
{
    float newAngles[4];

    if (!inverseKinematics(pos, newAngles))
    {
        return false;
    }

    for (int i = 0; i < 4; i++)
    {
        theta[i] = newAngles[i];
    }

    setJointAngles();

    return true;
}


// ============================================================
// ANGLE INTERPOLATION
// ============================================================

float interpolateAngle(float start, float target, float t)
{
    float difference = target - start;

    // Shortest angular path

    while (difference > 180.0f)
        difference -= 360.0f;

    while (difference < -180.0f)
        difference += 360.0f;

    return start + difference * t;
}


// ============================================================
// BEGIN CARTESIAN MOVEMENT
// ============================================================

void beginMovement(
    CylindricalPosition target,
    uint32_t duration,
    RobotState newState
)
{
    motionStart = forwardKinematics();

    motionTarget = target;

    motionDuration = duration;

    motionStartTime = millis();

    state = newState;

    motionActive = true;

    Serial.println("Starting Cartesian movement...");
}


// ============================================================
// SMOOTHSTEP TRAJECTORY
// ============================================================

float smoothStep(float t)
{
    return 3.0f * t * t - 2.0f * t * t * t;
}


// ============================================================
// UPDATE CARTESIAN TRAJECTORY
// ============================================================

void updateMovement()
{
    uint32_t elapsed = millis() - motionStartTime;

    float t = (float)elapsed / motionDuration;

    if (t >= 1.0f)
        t = 1.0f;

    float s = smoothStep(t);


    CylindricalPosition current;

    current.r =
        motionStart.r +
        (motionTarget.r - motionStart.r) * s;

    current.base =
        interpolateAngle(
            motionStart.base,
            motionTarget.base,
            s
        );

    current.z =
        motionStart.z +
        (motionTarget.z - motionStart.z) * s;

    current.alpha =
        motionStart.alpha +
        (motionTarget.alpha - motionStart.alpha) * s;


    // Calculate joint angles using IK

    if (!setCartesianPosition(current))
    {
        Serial.println("ERROR: IK failed during trajectory!");

        motionActive = false;

        state = ERROR;

        return;
    }


    // Movement completed

    if (t >= 1.0f)
    {
        motionActive = false;

        Serial.println("Movement completed!");

        switch (state)
        {
            case MOVE_TO_START:

                beginMovement(
                    PICKUP_POS,
                    PICKUP_MOVE_TIME,
                    MOVE_TO_PICKUP
                );

                break;


            case MOVE_TO_PICKUP:

                closeGripper();

                stateStartTime = millis();

                state = CLOSE_GRIPPER;

                break;


            case MOVE_TO_DROPOFF:

                openGripper();

                stateStartTime = millis();

                state = OPEN_GRIPPER;

                break;


            default:
                break;
        }
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    Serial.println();

    Serial.println("================================");
    Serial.println("4-DOF ROBOT ARM CONTROLLER");
    Serial.println("================================");


    // Initialize PCA9685

    initServos();


    // Set initial joint position

    setJointAngles();


    // Open gripper

    openGripper();


    // Initialize hardware timer

    timer = timerBegin(1000000);

    timerAttachInterrupt(timer, &onTimer);

    timerAlarm(timer, 20000, true, 0);


    Serial.println("20ms control loop started!");

    delay(1000);


    // Start pick-and-place sequence

    beginMovement(
        START_POS,
        START_MOVE_TIME,
        MOVE_TO_START
    );
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    // ----------------------------------------
    // 20ms control loop
    // ----------------------------------------

    if (timer20msFlag)
    {
        timer20msFlag = false;

        if (motionActive)
        {
            updateMovement();
        }
    }


    // ----------------------------------------
    // Gripper closing delay
    // ----------------------------------------

    if (state == CLOSE_GRIPPER)
    {
        if (millis() - stateStartTime >= GRIPPER_DELAY)
        {
            beginMovement(
                DROPOFF_POS,
                DROPOFF_MOVE_TIME,
                MOVE_TO_DROPOFF
            );
        }
    }


    // ----------------------------------------
    // Gripper opening delay
    // ----------------------------------------

    if (state == OPEN_GRIPPER)
    {
        if (millis() - stateStartTime >= GRIPPER_DELAY)
        {
            state = FINISHED;

            Serial.println();
            Serial.println("==============================");
            Serial.println("PICK AND PLACE COMPLETED!");
            Serial.println("==============================");
        }
    }


    // ----------------------------------------
    // Error handling
    // ----------------------------------------

    if (state == ERROR)
    {
        // Stop generating new movement commands.

        motionActive = false;
    }
}