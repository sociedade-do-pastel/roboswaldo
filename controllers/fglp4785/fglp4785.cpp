#include <cmath>
#include <iostream>
#include <string>

#include </usr/local/webots/include/controller/cpp/webots/Compass.hpp>
#include </usr/local/webots/include/controller/cpp/webots/Device.hpp>
#include </usr/local/webots/include/controller/cpp/webots/DistanceSensor.hpp>
#include </usr/local/webots/include/controller/cpp/webots/Field.hpp>
#include </usr/local/webots/include/controller/cpp/webots/Motor.hpp>
#include </usr/local/webots/include/controller/cpp/webots/Node.hpp>
#include </usr/local/webots/include/controller/cpp/webots/Robot.hpp>
#include </usr/local/webots/include/controller/cpp/webots/Supervisor.hpp>

using namespace webots;

#define TIME_STEP 128
#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 5

static const float c_RotationSpeed{3.27};

Supervisor* supervisor = new Supervisor();
Node* robot_node       = supervisor->getFromDef("e-Puck");
Node* track_node       = supervisor->getFromDef("Track");

Field* trans_field = robot_node->getField("translation");
Field* track_field = track_node->getField("translation");

Motor* leftMotor  = supervisor->getMotor("left wheel motor");
Motor* rightMotor = supervisor->getMotor("right wheel motor");

DistanceSensor* leftGroundSensor  = supervisor->getDistanceSensor("IR1");
DistanceSensor* rightGroundSensor = supervisor->getDistanceSensor("IR0");

Compass* compass = supervisor->getCompass("compass");

void go_to(const double* objective);

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

double distance_to(const double* objective)
{
    const double* current = get_robot_position();
    double dx             = current[0] - objective[0];
    double dz             = -current[2] * objective[2];
    return sqrt(dx * dx + dz * dz);
}

double distance_to_time(double velocity, double distance)
{
    double speed = velocity * 0.0205;
    return distance / speed;
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
            = abs(theta) / ((360 * (3.14 * 0.0205)) / (M_PI * 0.052)) + 0.2;
        if (theta > 0) {
            leftMotor->setVelocity(-0.5 * MAX_SPEED);
            rightMotor->setVelocity(0.5 * MAX_SPEED);
        }
        else if (theta < 0) {
            leftMotor->setVelocity(0.5 * MAX_SPEED);
            rightMotor->setVelocity(-0.5 * MAX_SPEED);
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

void walk(double time, double velocity, const double* objective)
{
    leftMotor->setVelocity(velocity);
    rightMotor->setVelocity(velocity);
    double start_time = supervisor->getTime();
    while (supervisor->getTime() - start_time < time) {
        if (balance_heading(objective)) {
            go_to(objective);
            break;
        }
        supervisor->step(TIME_STEP);
    }
}

void go_to(const double* objective)
{
    double heading = get_robot_heading();
    double theta   = theta_to(objective);
    double turn    = turn_theta(heading, theta);

    rotate_heading(turn);

    double distance = distance_to(objective);
    double time     = distance_to_time(MAX_SPEED, distance);

    walk(time, MAX_SPEED, objective);
}

int main(int argc, char** argv)
{
    double leftGroundSensorValue{0};
    double rightGroundSensorValue{0};

    double velocities[2];

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);

    leftGroundSensor->enable(supervisor->getBasicTimeStep());
    rightGroundSensor->enable(supervisor->getBasicTimeStep());

    compass->enable(TIME_STEP);

    const double* robot_position{nullptr};
    const double* track_position{nullptr};

    bool foundLine{false};
    bool foundDirection{false};
    bool aligned{false};
    int wait{4};
    int steps{20};

    while (supervisor->step(TIME_STEP) != -1) {
        robot_position = trans_field->getSFVec3f();
        track_position = track_field->getSFVec3f();

        leftGroundSensorValue  = leftGroundSensor->getValue();
        rightGroundSensorValue = rightGroundSensor->getValue();

        if (wait) {
            --wait;
            continue;
        }

        if (!foundLine) {
            if (rightGroundSensorValue > 5 || leftGroundSensorValue > 5) {
                foundLine = true;
                continue;
            }

            if (!foundDirection) {
                go_to(track_position);

                velocities[LEFT]  = MAX_SPEED;
                velocities[RIGHT] = MAX_SPEED;

                foundDirection = true;
            }
        }
        else if (!aligned) {
            if (rightGroundSensorValue < 5) {
                velocities[LEFT]  = 0;
                velocities[RIGHT] = 5.2;
            }
            else if (leftGroundSensorValue < 5) {
                velocities[LEFT]  = 5.2;
                velocities[RIGHT] = 0;
            }
            else {
                velocities[LEFT]  = 5.2;
                velocities[RIGHT] = 0;
            }

            --steps;
            if (steps == 0)
                aligned = true;
        }
        else {
            if (rightGroundSensorValue < 5) {
                velocities[LEFT]  = 2.7;
                velocities[RIGHT] = 5.2;
            }
            else if (leftGroundSensorValue < 5) {
                velocities[LEFT]  = 5.2;
                velocities[RIGHT] = 2.7;
            }
            else {
                velocities[LEFT]  = MAX_SPEED;
                velocities[RIGHT] = MAX_SPEED;
            }
        }

        leftMotor->setVelocity(velocities[LEFT]);
        rightMotor->setVelocity(velocities[RIGHT]);

        // posicao_Caixa1 = wb_supervisor_field_get_sf_vec3f(Caixa1_field);
        printf("velocities: %6.2f  %6.2f  |  Posição Robo: %6.2f  %6.2f |  "
               "Sensores de Pista: Esq:%6.2f  Dir:%6.2f\n",
               velocities[LEFT], velocities[RIGHT], robot_position[0],
               robot_position[2], leftGroundSensorValue,
               rightGroundSensorValue);
    };

    robot_node->cleanup();

    return 0;
}
