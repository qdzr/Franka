// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/examples_common.hpp>

//bool homing = 0;
//double grasping_width=0.079;
double speed_factor = 0.1;
std::string robot_ip="172.16.0.2";

int main(int argc, char** argv) {
  try {
//without the input of ip
    franka::Robot robot(robot_ip);
    franka::Gripper gripper(robot_ip);
    setDefaultBehavior(robot);
    std::cout<<"connected to robot!"<<std::endl;
    std::cout<<"careful! panda will move to start pose"<<std::endl;
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));
    std::array<double, 7> ready_pose={0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    MotionGenerator motion_generator(speed_factor, ready_pose);
    robot.control(motion_generator);
    gripper.move(0.08, 0.1);
    std::cout << "panda is at ready pose" << std::endl;
   }
  catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
