#include <iostream>
#include <string>
#include <unistd.h>

#include <doosan_control.h>

int main()
{

    DoosanRobot robot;
    if (!robot.init()) {
        std::cout << "[doosan_control] Error initializing robot" << std::endl;
        return -1;
    }

    // Move home
    robot.move_home();

    // Joint movements
    std::array<float, NUM_JOINT> testJointTarget = {0, 0, 90, 0, 90, 0};
    float maxSpeed = 100;
    float maxAccel = 100;

    std::cout << "Move joint" << std::endl;
    if (!robot.movej(testJointTarget, maxSpeed, maxAccel)) {
        std::cout << "something wrong" << std::endl;
    }

    robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    robot.set_robot_speed_mode(SPEED_NORMAL_MODE);

    // Linear movements
    std::array<float, 2> maxSpeedL = {1800, 360};
    std::array<float, 2> maxAccelL = {1000, 360};
    std::array<float, NUM_TASK> currentPose;
    std::array<float, NUM_TASK> targetPose;

    robot.get_current_posx(currentPose);
    for (size_t i = 0; i < NUM_TASK; ++i) {
        std::cout << currentPose[i] << ",";
    }
    std::cout << std::endl;

    targetPose = currentPose;
    targetPose[0] += 50;
    robot.movel(targetPose, maxSpeedL, maxAccelL);

    targetPose[1] += 50;
    robot.movel(targetPose, maxSpeedL, maxAccelL);

    targetPose[0] -= 50;
    robot.movel(targetPose, maxSpeedL, maxAccelL);

    targetPose[1] -= 50;
    robot.movel(targetPose, maxSpeedL, maxAccelL);

}
