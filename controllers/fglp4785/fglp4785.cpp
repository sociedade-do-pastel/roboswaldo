#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include <webots/Compass.hpp>
#include <webots/Device.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Field.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

using namespace webots;

#define TIME_STEP 128
#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 5.2
#define WHEEL_DIFF 2.5

// control variables
std::unique_ptr<Supervisor> supervisor = std::make_unique<Supervisor>();

Node* robot_node = supervisor->getFromDef("e-Puck");
Node* track_node = supervisor->getFromDef("Track");

Field* trans_field = robot_node->getField("translation");
Field* track_field = track_node->getField("translation");

Motor* left_motor  = supervisor->getMotor("left wheel motor");
Motor* right_motor = supervisor->getMotor("right wheel motor");

DistanceSensor* left_ground_sensor  = supervisor->getDistanceSensor("IR1");
DistanceSensor* right_ground_sensor = supervisor->getDistanceSensor("IR0");

Compass* compass = supervisor->getCompass("compass");

double velocities[2];

double left_ground_sensor_value{0};
double right_ground_sensor_value{0};

const double* get_robot_position()
{
    return trans_field->getSFVec3f();
}

float bearing_in_degrees()
{
    const double* north = compass->getValues();
    double rad          = atan2(north[1], north[2]);
    double bearing      = (rad - 1.5708) / M_PI * 180;
    if (bearing < 0)
        bearing += 360;
    return bearing;
}

double calculate_threshold(double property, double value, double threshold)
{
    return ((property >= value - threshold) || (property <= value + threshold));
}

double theta_to(const double* objective)
{
    auto current = get_robot_position();
    return atan2((-objective[2]) - (-current[2]), objective[0] - current[0])
           * (180.0 / 3.141592653589793238463);
}

double turn_theta(double heading, double theta)
{
    auto turn = theta - heading;
    if (turn > 180)
        turn = -(360 - turn);
    else if (turn < -180)
        turn += 360;
    return turn;
}

void rotate_heading(float theta)
{
    if (calculate_threshold(theta, 0, 0.00001)) {
        double rotation_time
            = std::abs(theta) / ((360 * (3.14 * 0.0205)) / (M_PI * 0.052)) + 0.1;
        if (theta > 0) {
            left_motor->setVelocity(-0.5 * MAX_SPEED);
            right_motor->setVelocity(0.5 * MAX_SPEED);
        }
        else if (theta < 0) {
            left_motor->setVelocity(0.5 * MAX_SPEED);
            right_motor->setVelocity(-0.5 * MAX_SPEED);
        }
        double start_time = supervisor->getTime();
        while (supervisor->getTime() - start_time <= rotation_time)
            supervisor->step(TIME_STEP);
    }
}

double get_robot_heading()
{
    auto bearing = bearing_in_degrees();
    auto heading = 360 - bearing + 90;
    if (heading > 360) {
        heading -= 360;
    }

    return heading;
}

bool balance_heading(const double* objective)
{
    double heading = get_robot_heading();
    double theta   = theta_to(objective);
    double turn    = turn_theta(heading, theta);
    if (turn > 1)
        return true;
    return false;
}

void walk(double velocity)
{
    velocities[LEFT]  = velocity;
    velocities[RIGHT] = velocity;
}

void go_to(const double* objective)
{
    double heading = get_robot_heading();
    double theta   = theta_to(objective);
    double turn    = turn_theta(heading, theta);

    rotate_heading(turn);
    walk(MAX_SPEED);
}

void find_track(bool& foundLine, bool& foundDirection,
               const double* track_position)
{
    // the 9 top range is for the startup compass spike
    if ((right_ground_sensor_value > 5 && right_ground_sensor_value < 9)
        || (left_ground_sensor_value > 5 && left_ground_sensor_value < 9)) {
        foundLine = true;
    }

    if (!foundDirection && !foundLine) {
        go_to(track_position);

        foundDirection = true;
    }
}

void align_in_track(int& steps, bool& aligned)
{
    if (right_ground_sensor_value < 5) {
        velocities[LEFT]  = 0;
        velocities[RIGHT] = MAX_SPEED;
    }
    else if (left_ground_sensor_value < 5) {
        velocities[LEFT]  = MAX_SPEED;
        velocities[RIGHT] = 0;
    }
    else {
        velocities[LEFT]  = MAX_SPEED;
        velocities[RIGHT] = 0;
    }

    --steps;
    if (steps == 0) {
        aligned = true;
        steps   = 30;
    }
}

void follow_track(int& steps, bool& foundLine, bool& foundDirection,
                 bool& aligned)
{
    if (right_ground_sensor_value < 5) {
        velocities[LEFT]  = MAX_SPEED - WHEEL_DIFF;
        velocities[RIGHT] = MAX_SPEED;
    }
    else if (left_ground_sensor_value < 5) {
        velocities[LEFT]  = MAX_SPEED;
        velocities[RIGHT] = MAX_SPEED - WHEEL_DIFF;
    }
    else {
        velocities[LEFT]  = MAX_SPEED;
        velocities[RIGHT] = MAX_SPEED;
    }

    // if the robots stays 30 steps out of the line, refind it
    if (left_ground_sensor_value < 5 && right_ground_sensor_value < 5)
        --steps;
    else
        steps = 30;

    if (steps == 0) {
        foundLine      = false;
        foundDirection = false;
        aligned        = false;
        steps          = 30;
    }
}

int main(int argc, char** argv)
{

    left_motor->setPosition(INFINITY);
    right_motor->setPosition(INFINITY);

    left_motor->setVelocity(0);
    right_motor->setVelocity(0);

    left_ground_sensor->enable(supervisor->getBasicTimeStep());
    right_ground_sensor->enable(supervisor->getBasicTimeStep());

    compass->enable(1);

    const double* robot_position{nullptr};
    const double* track_position{nullptr};

    bool found_line{false};
    bool found_direction{false};
    bool aligned{false};
    int steps{30};

    while (supervisor->step(TIME_STEP) != -1) {
        robot_position = trans_field->getSFVec3f();
        track_position = track_field->getSFVec3f();

        left_ground_sensor_value  = left_ground_sensor->getValue();
        right_ground_sensor_value = right_ground_sensor->getValue();

        if (!found_line)
            find_track(found_line, found_direction, track_position);
        else if (!aligned)
            align_in_track(steps, aligned);
        else
            follow_track(steps, found_line, found_direction, aligned);

        left_motor->setVelocity(velocities[LEFT]);
        right_motor->setVelocity(velocities[RIGHT]);

        printf("velocities: %6.2f  %6.2f  |  Posição Robo: %6.2f  %6.2f |  "
               "Sensores de Pista: Esq:%6.2f  Dir:%6.2f\n",
               velocities[LEFT], velocities[RIGHT], robot_position[0],
               robot_position[2], left_ground_sensor_value,
               right_ground_sensor_value);
    };

    robot_node->cleanup();

    return 0;
}
