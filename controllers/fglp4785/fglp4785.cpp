#include <iostream>
#include <string>

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
#define MAX_SPEED 4.6

static const int c_QtddSensorDist{8};

static double weights[c_QtddSensorDist][2]
    = {{-1.3, -1.0}, {-1.3, -1.0}, {-0.5, 0.5}, {0.0, 0.0},
       {0.0, 0.0},   {0.05, -0.5}, {-0.75, 0},  {-0.75, 0}};
static double offsets[2] = {0.5 * MAX_SPEED, 0.5 * MAX_SPEED};

int main(int argc, char** argv)
{
    Supervisor* supervisor = new Supervisor();
    Node* robot_node       = supervisor->getFromDef("e-Puck");

    Field* trans_field = robot_node->getField("translation");
    Field* rot_field   = robot_node->getField("rotation");

    Motor* leftMotor  = supervisor->getMotor("left wheel motor");
    Motor* rightMotor = supervisor->getMotor("right wheel motor");

    DistanceSensor* leftGroundSensor  = supervisor->getDistanceSensor("IR1");
    DistanceSensor* rightGroundSensor = supervisor->getDistanceSensor("IR0");

    double leftGroundSensorValue{0};
    double rightGroundSensorValue{0};

    double velocities[2];

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);

    DistanceSensor* distSensor[c_QtddSensorDist]
        = {supervisor->getDistanceSensor("ps0"),
           supervisor->getDistanceSensor("ps1"),
           supervisor->getDistanceSensor("ps2"),
           supervisor->getDistanceSensor("ps3"),
           supervisor->getDistanceSensor("ps4"),
           supervisor->getDistanceSensor("ps5"),
           supervisor->getDistanceSensor("ps6"),
           supervisor->getDistanceSensor("ps7")};

    double distSensorValues[c_QtddSensorDist];

    for (int i{0}; i < c_QtddSensorDist; ++i)
        distSensor[i]->enable(supervisor->getBasicTimeStep());

    leftGroundSensor->enable(supervisor->getBasicTimeStep());
    rightGroundSensor->enable(supervisor->getBasicTimeStep());

    const double* robot_position{nullptr};

    while (supervisor->step(TIME_STEP) != -1) {
        for (int i{0}; i < c_QtddSensorDist; ++i)
            distSensorValues[i] = distSensor[i]->getValue() / 4096;

        for (int i{0}; i < 2; ++i) {
            velocities[i] = 0.0;
            for (int j{0}; j < c_QtddSensorDist; ++j)
                velocities[i] += distSensorValues[j] * weights[j][i];

            velocities[i] = offsets[i] + (velocities[i] * MAX_SPEED)
                            + ((double)rand() / RAND_MAX - .5);

            if (velocities[i] > MAX_SPEED)
                velocities[i] = MAX_SPEED;
            else if (velocities[i] < -MAX_SPEED)
                velocities[i] = -MAX_SPEED;
        }

        robot_position = trans_field->getSFVec3f();

        leftGroundSensorValue  = leftGroundSensor->getValue();
        rightGroundSensorValue = rightGroundSensor->getValue();

        if (rightGroundSensorValue < 5) {
            velocities[LEFT]  = 2.3;
            velocities[RIGHT] = 4.8;
        }
        else if (leftGroundSensorValue < 5) {
            velocities[LEFT]  = 4.8;
            velocities[RIGHT] = 2.3;
        }
        else {
            velocities[LEFT]  = MAX_SPEED;
            velocities[RIGHT] = MAX_SPEED;
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
