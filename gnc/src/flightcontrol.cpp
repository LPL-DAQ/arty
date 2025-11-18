#include "flightcontrol.h"

LOG_MODULE_REGISTER(flightcontrol, CONFIG_LOG_DEFAULT_LEVEL);

// ---------- Constants ----------

const float maxThrottleN = 2.5; // newtons
const float maxGimble = 12.0;   // degrees
const float maxTiltDeg = 15.0;  // What we dont want the rocket to tilt more than (deg)
const float angleXNeutral = 54.0f;
const float angleYNeutral = -18.0f;
const double loopFreq = 100.0; // Hz
constexpr float DEG2RAD_F = 0.0174532925f;

// Mounting offset between IMU sensor frame and rocket body frame (deg)
constexpr float SENSOR_TO_BODY_YAW_DEG = 45.0f;
constexpr float SENSOR_TO_BODY_PITCH_DEG = -90.0f;
constexpr float SENSOR_TO_BODY_ROLL_DEG = 0.0f;

PID pidRoll(0.8, 0.1, 0.0);  // needs tuning
PID pidPitch(0.8, 0.1, 0.0); // needs tuning
PID pidHeading(0, 0, 0);     // needs tuning
PID pidX(0, 0, 0);           // needs tuning
PID pidY(0, 0, 0);           // needs tuning
PID pidZ(0, 0, 0);           // needs tuning
PID pidZVelocity(0, 0, 0);   // needs tuning
// ---------- Simple runtime state ----------

struct Ref
{
    float yaw0{0}, pitch0{0}, roll0{0};
    float x0{0}, y0{0}, z0{0};
    bool set{false};
} ref;

struct State
{
    float pitch{0}, roll{0};
    float heading{0}, tiltX{0}, tiltY{0};
    float x{0}, y{0}, z{0};
    float xVel{0}, yVel{0}, zVel{0};
} state;

struct desState
{
    float pitch{0}, roll{0};
    float heading{0}, tiltX{0}, tiltY{0};
    float x{0}, y{0}, z{0};
    float zVel{0};
} desState;

// ---------- Global state ----------

double loopTime = 0.0;
double prevIMUTimestampUs = 0.0;
int loopCount = 0;
bool stopped = false;

// ---------- Main program ----------

// Utility that doesn’t belong to a single device: basic centering & neutralization
void centerTVC()
{
    set_desired_thrust_deg(0,0);

}

int flight_init()
{

    int err = servos_init();
    if (err){
        LOG_ERR("Failed to initialize servos");
        return 1;
    }
    // Bring servos to neutral
    centerTVC();

    err = esc_init();
    if (err){
        LOG_ERR("Failed to initialize esc's");
        return 1;
    }

    // IMU: TBD, reference OldDrone code


    pidRoll.setIntegralLimits(-maxGimble / 3, maxGimble / 3);
    pidPitch.setIntegralLimits(-maxGimble / 3, maxGimble / 3);

    return 0;
}

// returns {xOutput, yOutput} for thrust angles
std::array<float, 2> rotationalPID(float dt)
{
    // Compute PID outputs
    std::array<float, 2> out{};
    out[0] = pidRoll.calculate(desState.roll, state.roll, dt);
    out[1] = pidPitch.calculate(desState.pitch, state.pitch, dt);

    return out;
}

// returns {xOutput, yOutput} for thruster angles
std::array<float, 2> lateralPID(float dt)
{
    // outerloop on position
    if (loopCount % 3 == 0)
    {
        // Compute desired tilt in world frame
        desState.tiltX = pidX.calculate(desState.x, 0, dt);
        desState.tiltY = pidY.calculate(desState.y, 0, dt);

        // Rotate desired tilt from world frame to body frame
        const float headingRad = state.heading * DEG2RAD_F;
        desState.roll = cosf(headingRad) * desState.tiltX + sinf(headingRad) * desState.tiltY;
        desState.pitch = -sinf(headingRad) * desState.tiltX + cosf(headingRad) * desState.tiltY;
        desState.roll = util::clamp(desState.roll, -maxGimble, +maxGimble);
        desState.pitch = util::clamp(desState.pitch, -maxGimble, +maxGimble);
    }
    // innerloop on rotation
    return rotationalPID(dt);
}

// returns desired throttle in newtons
float verticalPID(float dt)
{
    // outerloop on position
    if (loopCount % 3 == 0)
    {
        desState.zVel = pidZ.calculate(desState.z, 0, dt);
    }
    // innerloop on velocity
    float desThrottle = pidZVelocity.calculate(desState.zVel, 0, dt);
    return util::clamp(desThrottle, 0.0f, 1.0f);
}

// returns a desired constant, -1..1 to add to ESC 1, subtract from ESC 2
float headingPID(float dt)
{
    float headingError = desState.heading - state.heading;
    if (headingError > 180.0f)
        headingError -= 360.0f;
    if (headingError < -180.0f)
        headingError += 360.0f;

    float out = pidHeading.calculate(0.0f, -headingError, dt); // negative error maybe?
    return out;
}

void updateState()
{

    //auto e = imu.euler();
    //state.pitch = e.pitch - ref.pitch0;
    //state.roll = e.roll - ref.roll0;

    //auto a = imu.projectedAngles();
    //state.tiltX = a.tiltAboutX;
    //state.tiltY = a.tiltAboutY;
   // state.heading = a.heading;

    // MEKF data for position and rates

    // auto la = imu.accelerometer();
    // state.xAcc = la.x;
    // state.yAcc = la.y;
    // state.zAcc = la.z;
}
// dt = milliseconds since last loop

int flight_control_loop_step(double dt)
{
    LOG_INF("Got delta time: ");
    LOG_INF(dt);


    if (stopped)
    {
        esc_idle();
        centerTVC();
        return 1;
    }

    //imu.update();
    // // Refresh IMU data
    updateState();

    // get desired thrust angles
    std::array<float, 2> pidOut = lateralPID(dt); // level flight at origin

    // send input to servos
    set_desired_thrust_deg(pidOut[0], pidOut[1]);

    // get base throttle
    float throttle01= verticalPID(dt);

    // calculate yaw correction
    float yawAdjust = headingPID(dt);
    float throttle01a = util::clamp(throttle01 + yawAdjust, 0.0f, 1.0f);
    float throttle01b = util::clamp(throttle01 - yawAdjust, 0.0f, 1.0f);

    // send input to ESC's'
    set_desired_thrust_newtons(throttle01a * maxThrottleN,throttle01b * maxThrottleN, maxThrottleN);

    loopCount++;
    LOG_INF("loop count completed: ");
    LOG_INF(loopCount);
    LOG_INF("\n");

    return 0;
}


void flight_stop(){
    stopped = true;
}