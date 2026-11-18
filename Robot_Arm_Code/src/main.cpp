#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ============================================================
// 1. ROBOT CONFIGURATION
// ============================================================

#define NUM_SERVOS 5

#define I2C_SDA 8
#define I2C_SCL 9

#define PCA9685_ADDRESS 0x40
#define SERVO_FREQ 50

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

#define CONTROL_PERIOD_US 20000UL  // 20 ms

// Link dimensions (mm)
const float d1 = 83.42f;
const float a2 = 101.95f;
const float a3 = 61.95f;
const float a4 = 72.00f;

// ============================================================
// 2. PCA9685 CONFIGURATION
// ============================================================

Adafruit_PWMServoDriver pwm(PCA9685_ADDRESS);

const int servoChannels[NUM_SERVOS] = {0, 1, 2, 3, 4};

// Servo calibration offsets
const float offset[NUM_SERVOS] = {90.0f, 54.0f, -160.0f, 106.0f, 90.0f};

// Gripper positions (physical servo angles)
#define GRIPPER_OPEN_DEG       60.0f
#define GRIPPER_CLOSED_DEG    180.0f

// ============================================================
// 3. JOINT VARIABLES
// ============================================================

// theta[0] = base
// theta[1] = shoulder
// theta[2] = elbow
// theta[3] = wrist
// theta[4] = gripper (not included in kinematics)

float theta[NUM_SERVOS] = {
    0.0f, 30.0f, 60.0f, 0.0f, 0.0f
};

// ============================================================
// 4. CYLINDRICAL POSITION STRUCTURE
// ============================================================

struct CylindricalPosition {
    float r;       // radial distance, mm
    float base;    // base rotation, degrees
    float z;       // vertical position, mm
    float alpha;   // end-effector orientation, degrees
};

// ============================================================
// 5. TARGET POSITIONS
// ============================================================

// All distances are in mm.
// All angles are in degrees.
// Alpha = 0 means the orientation specified by your report.

const CylindricalPosition START_POS = {
    170.0f, 0.0f, 10.0f, 0.0f
};

const CylindricalPosition PICKUP_POS = {
    200.0f, 0.0f, 10.0f, 0.0f
};

const CylindricalPosition DROPOFF_POS = {
    150.0f, 90.0f, 220.0f, 0.0f
};

const CylindricalPosition END_POS = {
    0.0f, 90.0f, 220.0f, 0.0f
};

// ============================================================
// 6. CYLINDRICAL TRAJECTORY CONFIGURATION
// ============================================================

// These are independent maximum limits for each coordinate.
// Tune these values to suit your mechanics and servo speed.

#define R_MAX_VELOCITY       200.0f  // mm/s
#define R_MAX_ACCEL          300.0f  // mm/s^2

#define THETA_MAX_VELOCITY    90.0f  // deg/s
#define THETA_MAX_ACCEL      180.0f  // deg/s^2

#define Z_MAX_VELOCITY       200.0f  // mm/s
#define Z_MAX_ACCEL          300.0f  // mm/s^2

#define ALPHA_MAX_VELOCITY    90.0f  // deg/s
#define ALPHA_MAX_ACCEL      180.0f  // deg/s^2

#define GRIPPER_DELAY_CLOSE 1000UL // wait 1 second after closing gripper 
#define GRIPPER_DELAY_OPEN 200UL // wait 0.2 second after opening gripper

// Select elbow configuration
#define ELBOW_POSITIVE true

// ============================================================
// 7. TIMER CONFIGURATION
// ============================================================

hw_timer_t *timer = nullptr;

volatile uint32_t timerTickCount = 0;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    timerTickCount++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

// ============================================================
// 8. MOTION STATE MACHINE
// ============================================================

enum RobotState {
    MOVE_TO_START,
    MOVE_TO_PICKUP,
    CLOSE_GRIPPER,
    MOVE_TO_DROPOFF,
    OPEN_GRIPPER,
    Move_TO_END,
    FINISHED,
    ERROR
};

RobotState state = MOVE_TO_START;

bool motionActive = false;

uint32_t motionStartTime = 0;
uint32_t stateStartTime = 0;

// ============================================================
// 9. CYLINDRICAL TRAJECTORY VARIABLES
// ============================================================

CylindricalPosition motionStart;
CylindricalPosition motionTarget;

struct TrapezoidalProfile {
    float distance;
    float direction;
    float acceleration;
    float maxVelocity;
    float peakVelocity;
    float accelerationTime;
    float constantVelocityTime;
    float accelerationDistance;
    float totalTime;
};

TrapezoidalProfile profileR;
TrapezoidalProfile profileTheta;
TrapezoidalProfile profileZ;
TrapezoidalProfile profileAlpha;

float totalTrajectoryTime = 0.0f;

// ============================================================
// 10. SERVO INITIALIZATION
// ============================================================

void initServos() {
    Wire.begin(I2C_SDA, I2C_SCL);
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
    delay(10);

    Serial.println("PCA9685 initialized!");
}

// ============================================================
// 11. SERVO CONTROL
// ============================================================

void setServoAngle(int index, float angle) {
    if (index < 0 || index >= NUM_SERVOS) {
        return;
    }

    angle = constrain(angle, 0.0f, 180.0f);

    float pulseWidth =
        SERVO_MIN_US +
        (angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);

    pwm.writeMicroseconds(
        servoChannels[index],
        (uint16_t)pulseWidth
    );
}

// ============================================================
// 12. UPDATE ARM JOINTS
// ============================================================

void setJointAngles() {
    setServoAngle(0, theta[0] + offset[0]);
    setServoAngle(1, theta[1] + offset[1]);
    setServoAngle(2, -(theta[2] + offset[2]));
    setServoAngle(3, theta[3] + offset[3]);
}

// ============================================================
// 13. GRIPPER CONTROL
// ============================================================

void openGripper() {
    setServoAngle(4, GRIPPER_OPEN_DEG);
    Serial.println("Gripper OPEN");
}

void closeGripper() {
    setServoAngle(4, GRIPPER_CLOSED_DEG);
    Serial.println("Gripper CLOSED");
}

// ============================================================
// 14. FORWARD KINEMATICS
// ============================================================
// Returns the current cylindrical position of the end-effector
CylindricalPosition forwardKinematics() {
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
// 15. INVERSE KINEMATICS
// ============================================================
// calculates the joint angles for a given cylindrical position
// Returns true if the position is reachable, false otherwise
bool inverseKinematics(CylindricalPosition pos, float result[4]) {
    // Wrist position
    float alphaRad = pos.alpha * DEG_TO_RAD;

    float rw = pos.r - a4 * cosf(alphaRad);
    float zw = pos.z - d1 + a4 * sinf(alphaRad);

    // Elbow angle
    float D =
        (rw * rw + zw * zw - a2 * a2 - a3 * a3) /
        (2.0f * a2 * a3);

    // Workspace check
    if (D < -1.0f || D > 1.0f) {
        return false;
    }

    D = constrain(D, -1.0f, 1.0f);

    float q2 = acosf(D);

    if (!ELBOW_POSITIVE) {
        q2 = -q2; // Elbow down configuration
    }

    // Shoulder angle
    float q1 =
        atan2f(rw, zw) -
        atan2f(
            a3 * sinf(q2),
            a2 + a3 * cosf(q2)
        );

    // Wrist angle
    float q3 =
        90.0f + pos.alpha -
        (q1 + q2) * RAD_TO_DEG;

    // Convert to degrees
    float baseDeg = pos.base;
    float shoulderDeg = q1 * RAD_TO_DEG;
    float elbowDeg = q2 * RAD_TO_DEG;
    float wristDeg = q3;

    // Physical servo angles
    float servoAngle[4];

    servoAngle[0] = baseDeg + offset[0];
    servoAngle[1] = shoulderDeg + offset[1];
    servoAngle[2] = -(elbowDeg + offset[2]);
    servoAngle[3] = wristDeg + offset[3];

    // Servo limits
    for (int i = 0; i < 4; i++) {
        if (servoAngle[i] < 0.0f || servoAngle[i] > 180.0f) {
            return false;
        }
    }

    // Store logical joint angles
    result[0] = baseDeg;
    result[1] = shoulderDeg;
    result[2] = elbowDeg;
    result[3] = wristDeg;

    return true;
}

// ============================================================
// 16. APPLY INVERSE KINEMATICS
// ============================================================
// Sets the joint angles for a given cylindrical position by applying inverse kinematics
bool setCartesianPosition(CylindricalPosition pos) {
    float newAngles[4];

    if (!inverseKinematics(pos, newAngles)) {
        return false;
    }

    for (int i = 0; i < 4; i++) {
        theta[i] = newAngles[i];
    }

    setJointAngles();
    return true;
}

// ============================================================
// 17. CALCULATE AN INDIVIDUAL TRAPEZOIDAL PROFILE
// ============================================================

void calculateProfile(
    TrapezoidalProfile &profile,
    float signedDistance,
    float maxVelocity,
    float acceleration
) {
    profile.distance = fabsf(signedDistance);
    profile.direction = (signedDistance >= 0.0f) ? 1.0f : -1.0f;
    profile.maxVelocity = maxVelocity;
    profile.acceleration = acceleration;

    if (profile.distance < 0.001f) { // Handle zero distance case
        profile.peakVelocity = 0.0f;
        profile.accelerationTime = 0.0f;
        profile.constantVelocityTime = 0.0f;
        profile.accelerationDistance = 0.0f;
        profile.totalTime = 0.0f;
        return;
    }

    // Distance required to accelerate to maxVelocity
    float requiredDistance =
        (maxVelocity * maxVelocity) / (2.0f * acceleration);

    // TRIANGULAR PROFILE
    if (2.0f * requiredDistance >= profile.distance) {
        profile.peakVelocity =
            sqrtf(acceleration * profile.distance);

        profile.accelerationTime =
            profile.peakVelocity / acceleration;

        profile.accelerationDistance =
            0.5f * acceleration *
            profile.accelerationTime *
            profile.accelerationTime;

        profile.constantVelocityTime = 0.0f;
    }
    // TRAPEZOIDAL PROFILE
    else {
        profile.peakVelocity = maxVelocity;

        profile.accelerationTime =
            maxVelocity / acceleration;

        profile.accelerationDistance =
            0.5f * acceleration *
            profile.accelerationTime *
            profile.accelerationTime;

        float constantDistance =
            profile.distance - 2.0f * profile.accelerationDistance;

        profile.constantVelocityTime =
            constantDistance / maxVelocity;
    }

    profile.totalTime =
        2.0f * profile.accelerationTime +
        profile.constantVelocityTime;
}

// ============================================================
// 18. CALCULATE POSITION ALONG AN INDIVIDUAL PROFILE
// ============================================================
// Returns the position along the profile at time t (seconds)
float calculateProfilePosition(
    const TrapezoidalProfile &profile,
    float t
) {
    if (profile.distance < 0.001f) { // Handle zero distance case
        return 0.0f;
    }

    if (t <= 0.0f) { // Handle negative time case
        return 0.0f;
    }

    if (t >= profile.totalTime) { // Handle time beyond total time case
        return profile.distance * profile.direction;
    }

    const float ta = profile.accelerationTime;
    const float tc = profile.constantVelocityTime;
    const float accel = profile.acceleration;

    float travelled = 0.0f;

    // ACCELERATION
    if (t < ta) {
        travelled = 0.5f * accel * t * t;
    }
    // CONSTANT VELOCITY
    else if (t < ta + tc) {
        float constantTime = t - ta;

        travelled =
            profile.accelerationDistance +
            profile.peakVelocity * constantTime;
    }
    // DECELERATION
    else {
        float remainingTime = profile.totalTime - t;

        travelled =
            profile.distance -
            0.5f * accel * remainingTime * remainingTime;
    }

    return travelled * profile.direction; // always return positive distance, regardless of direction
}

// ============================================================
// 19. CALCULATE ALL CYLINDRICAL TRAJECTORIES
// ============================================================

void calculateCylindricalTrajectory() {
    float dr = motionTarget.r - motionStart.r;
    float dTheta = motionTarget.base - motionStart.base;
    float dz = motionTarget.z - motionStart.z;
    float dAlpha = motionTarget.alpha - motionStart.alpha;

    calculateProfile(
        profileR,
        dr,
        R_MAX_VELOCITY,
        R_MAX_ACCEL
    );

    calculateProfile(
        profileTheta,
        dTheta,
        THETA_MAX_VELOCITY,
        THETA_MAX_ACCEL
    );

    calculateProfile(
        profileZ,
        dz,
        Z_MAX_VELOCITY,
        Z_MAX_ACCEL
    );

    calculateProfile(
        profileAlpha,
        dAlpha,
        ALPHA_MAX_VELOCITY,
        ALPHA_MAX_ACCEL
    );

    // The overall movement ends when the slowest coordinate finishes.
    totalTrajectoryTime = max(
        max(profileR.totalTime, profileTheta.totalTime),
        max(profileZ.totalTime, profileAlpha.totalTime)
    );

    Serial.println();
    Serial.println("CYLINDRICAL TRAJECTORY");

    Serial.printf("Radial movement: %.2f mm\n", dr);
    Serial.printf("Base rotation: %.2f deg\n", dTheta);
    Serial.printf("Vertical movement: %.2f mm\n", dz);
    Serial.printf("Orientation change: %.2f deg\n", dAlpha);

    Serial.printf(
        "R time: %.3f s, Theta time: %.3f s, Z time: %.3f s, Alpha time: %.3f s\n",
        profileR.totalTime,
        profileTheta.totalTime,
        profileZ.totalTime,
        profileAlpha.totalTime
    );

    Serial.printf(
        "Total movement time: %.3f s\n",
        totalTrajectoryTime
    );
}

// ============================================================
// 20. BEGIN MOVEMENT
// ============================================================
// Starts a new movement to the specified target position with the given state
void beginMovement(
    CylindricalPosition target,
    RobotState newState
) {
    // Get the current cylindrical position from forward kinematics.
    motionStart = forwardKinematics();
    motionTarget = target;

    state = newState;
    motionActive = true;

    motionStartTime = micros();

    calculateCylindricalTrajectory();

    Serial.println();
    Serial.println("Starting cylindrical movement...");
}

// ============================================================
// 21. HANDLE MOVEMENT COMPLETION
// ============================================================
// Called when the current movement is completed, to transition to the next state
void handleMovementComplete() {
    motionActive = false;

    Serial.println("Movement completed!");

    switch (state) {
        case MOVE_TO_START:
            beginMovement(PICKUP_POS, MOVE_TO_PICKUP);
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

        case Move_TO_END:
            state = FINISHED;
            Serial.println();
            Serial.println("==============================");
            Serial.println("PICK AND PLACE COMPLETED!");
            Serial.println("==============================");
            
            break;

        default:
            break;
    }
}

// ============================================================
// 22. UPDATE CYLINDRICAL TRAJECTORY
// ============================================================
// Updates the current position along the trajectory based on elapsed time(write to servos)
void updateMovement() {
    // Elapsed time in seconds. Unsigned subtraction handles micros() rollover.
    float elapsed =
        (uint32_t)(micros() - motionStartTime) * 0.000001f;

    CylindricalPosition current;

    current.r =
        motionStart.r +
        calculateProfilePosition(profileR, elapsed);

    current.base =
        motionStart.base +
        calculateProfilePosition(profileTheta, elapsed);

    current.z =
        motionStart.z +
        calculateProfilePosition(profileZ, elapsed);

    current.alpha =
        motionStart.alpha +
        calculateProfilePosition(profileAlpha, elapsed);

    // Apply inverse kinematics to the current cylindrical setpoint. And write to the servos.
    if (!setCartesianPosition(current)) {
        Serial.println("ERROR: IK failed during movement!");

        motionActive = false;
        state = ERROR;
        return;
    }

    // Finish exactly at the requested target.
    if (elapsed >= totalTrajectoryTime) {
        if (!setCartesianPosition(motionTarget)) {
            Serial.println("ERROR: IK failed at final target!");

            motionActive = false;
            state = ERROR;
            return;
        }

        handleMovementComplete();
    }
}

// ============================================================
// 23. SETUP
// ============================================================

void setup() {
    Serial.begin(115200);

    Serial.println();
    Serial.println("================================");
    Serial.println("4-DOF ROBOT ARM CONTROLLER");
    Serial.println("WITH CYLINDRICAL TRAJECTORY PLANNING");
    Serial.println("================================");

    // Initialize PCA9685
    initServos();

    // Initialize gripper
    openGripper();

    // Set initial joint positions
    setJointAngles();

    delay(1000);

    // Initialize 20 ms hardware timer (Arduino ESP32 Core 3.x API).
    timer = timerBegin(1000000);

    if (timer == nullptr) {
        Serial.println("ERROR: Hardware timer initialization failed!");
        state = ERROR;
        return;
    }

    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, CONTROL_PERIOD_US, true, 0);

    Serial.println("20 ms timer started!");
    Serial.println("Robot arm initialized!");

    delay(1000);

    // Start pick-and-place sequence.
    beginMovement(START_POS, MOVE_TO_START);
}

// ============================================================
// 24. MAIN LOOP
// ============================================================

void loop() {
    // Read timer ticks atomically.
    uint32_t ticks;

    portENTER_CRITICAL(&timerMux);
    ticks = timerTickCount;
    timerTickCount = 0;
    portEXIT_CRITICAL(&timerMux);

    // One update is sufficient because the trajectory uses elapsed
    // wall-clock time rather than incrementing a position per tick.
    if (ticks > 0 && motionActive) {
        updateMovement();
    }

    // Gripper closing delay.
    if (state == CLOSE_GRIPPER) {
        if ((uint32_t)(millis() - stateStartTime) >= GRIPPER_DELAY_CLOSE) {
            beginMovement(DROPOFF_POS, MOVE_TO_DROPOFF);
        }
    }

    // Gripper opening delay.
    if (state == OPEN_GRIPPER) {
        if ((uint32_t)(millis() - stateStartTime) >= GRIPPER_DELAY_OPEN) {
            beginMovement(END_POS, Move_TO_END);
        }
    }

    // Error handling: do not issue further motion commands.
    if (state == ERROR) {
        motionActive = false;
    }
}
