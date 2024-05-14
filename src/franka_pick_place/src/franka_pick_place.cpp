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

bool homing = 0;
double grasping_width=0.079;
double speed_factor = 0.1;
std::string robot_ip="172.16.0.2";
/**
 * @example joint_point_to_point_motion.cpp
 * An example that moves the robot to a target position by commanding joint positions.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  //if (argc != 5) {
   // std::cerr << "Usage: " << argv[0] << " <robot-hostname> "
             // << "<joint0> <joint1> <joint2> <joint3> <joint4> <joint5> <joint6> "
             // << "<speed-factor>" <<"<homing>"<<"<grasping_width>"<< std::endl
             // << "joint0 to joint6 are joint angles in [rad]." << std::endl
             // << "speed-factor must be between zero and one." << std::endl;
   // return -1;
 // }
   std::cerr << "default parameters: " << std::endl<<"robot_ip = 172.16.0.2 "<< std::endl
             << "speed-factor = 0.1  " << std::endl<<"homing = 0 "<< std::endl
             << "you can try to grasp object with any grasping_width." << std::endl
             << "pose joint0 to joint6 are joint angles in [rad]." << std::endl
             << "speed-factor must be between zero and one." << std::endl;
 
   std::cout<<"If you don't want to use the default speed, please enter 0. "<< std::endl
             <<"other numbers will use the default."<< std::endl;
   bool choose=1;
   double speed;
   int num;
   std::cout<<"Please enter the number:";
   std::cin>>num;
   choose=num;
   if(!choose)
   {
     std::cout<<"speed-factor must be between zero and one."<< std::endl
              <<"Please give new speed: ";
     std::cin>>speed;
     std::cout<<std::endl;
     speed_factor=speed;
   }

  try {
//without the input of ip
    franka::Robot robot(robot_ip);
    franka::Gripper gripper(robot_ip);
    setDefaultBehavior(robot);

    std::array<double, 7> ready_pose={0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    std::array<double, 7> pose1={-0.238,-0.768,-0.053,-2.540,-0.032,1.823,0.782};
    std::array<double, 7> pose2={-0.413,-0.389,-0.084,-2.556,-0.058,2.178,0.563};
    std::array<double, 7> pose3={-0.421,0.012,-0.075,-2.625,-0.055,2.659,0.585};
    std::array<double, 7> pose4={-0.422,0.195,-0.064,-2.609,-0.0545,2.793,0.604};
    std::array<double, 7> pose5={-0.424,-0.074,-0.066,-2.670,-0.0549,2.686,0.589};
    std::array<double, 7> pose6={-0.255,-0.675,0.163,-2.686,0.113,2.046,0.717};
    std::array<double, 7> pose7={-0.002,-0.695,0.342,-2.558,0.225,1.963,0.979};
    std::array<double, 7> pose8={-0.019,-0.351,0.435,-2.572,0.223,2.244,1.102};
    std::array<double, 7> pose9={-0.021,0.024,0.475,-2.629,-0.050,2.592,1.315};
    std::array<double, 7> pose10={-0.002,-0.442,0.459,-2.577,0.205,2.200,1.162};
    std::array<double, 7> pose11={-0.014,-0.800,0.340,-2.579,0.200,1.915,0.991};
    //for (size_t i = 0; i < 7; i++) {
      //  q_goal[i] = std::stod(argv[i + 2]);
    //}
    //double speed_factor = std::stod(argv[2]);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    MotionGenerator motion_generator0(speed_factor, ready_pose);
    gripper.move(0.08, 0.1);
    std::cout << "gripper open" << std::endl;
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator0);
    std::cout << "panda is at ready pose" << std::endl;
    
    MotionGenerator motion_generator1(speed_factor, pose1);
    robot.control(motion_generator1);
    std::cout << "panda is at pose1" << std::endl;

    MotionGenerator motion_generator2(speed_factor, pose2);
    robot.control(motion_generator2);
    std::cout << "panda is at pose2" << std::endl;

    MotionGenerator motion_generator3(speed_factor, pose3);
    robot.control(motion_generator3);
    std::cout << "panda is at pose3" << std::endl;

    MotionGenerator motion_generator4(speed_factor, pose4);
    robot.control(motion_generator4);
    std::cout << "panda is at pose4" << std::endl;

  try {
    //franka::Gripper gripper("172.16.0.2");
   
    //double grasping_width = std::stod(argv[4]);
    //std::stringstream ss(argv[3]);

    //bool homing;
    //if (!(ss >> homing)) {
     // std::cerr << "<homing> can be 0 or 1." << std::endl;
     // return -1;
   // }

    if (homing) {
      // Do a homing in order to estimate the maximum grasping width with the current fingers.
      gripper.homing();
    }

    // Check for the maximum grasping width.
    franka::GripperState gripper_state = gripper.readOnce();
    //std::cout << gripper_state<< std::endl;
    if (gripper_state.max_width < grasping_width) {
      std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
      return -1;
    }

    // Grasp the object.
    gripper.grasp(grasping_width, 0.1, 40, 0.06, 0.06);
    //gripper.move(grasping_width, 0.1);
    std::cout << "Successfully grasp object." << std::endl;
    //std::cout << gripper_state<< std::endl;
   // if (!gripper.grasp(grasping_width, 0.1, 40, 0.01, 0.01)) {
      //std::cout << "Failed to grasp object." << std::endl;
     // return -1;
    //}
    //else{std::cout << "Successfully grasp object." << std::endl;}

// Wait 3s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));

    gripper_state = gripper.readOnce();
    if (!gripper_state.is_grasped) {
      std::cout << "Please check if object lost." << std::endl;
      //return -1;
    }
    
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
   
//move the panda arm
    MotionGenerator motion_generator5(speed_factor, pose5);
    robot.control(motion_generator5);
    std::cout << "panda is at pose5" << std::endl;

    MotionGenerator motion_generator6(speed_factor, pose6);
    robot.control(motion_generator6);
    std::cout << "panda is at pose6" << std::endl;

    MotionGenerator motion_generator7(speed_factor, pose7);
    robot.control(motion_generator7);
    std::cout << "panda is at pose7" << std::endl;

    MotionGenerator motion_generator8(speed_factor, pose8);
    robot.control(motion_generator8);
    std::cout << "panda is at pose8" << std::endl;

    MotionGenerator motion_generator9(speed_factor, pose9);
    robot.control(motion_generator9);
    std::cout << "panda is at pose9" << std::endl;

//grasp stop --release object
    std::cout << "Grasped object, will release it now." << std::endl;
    //std::cin.ignore();
    gripper.stop();
    gripper.move(0.08, 0.1);

//back to ready pose
    std::cout << "Panda arm will return to ready pose." << std::endl;

    MotionGenerator motion_generator10(speed_factor, pose10);
    robot.control(motion_generator10);
    std::cout << "panda is at pose10" << std::endl;

    MotionGenerator motion_generator11(speed_factor, pose11);
    robot.control(motion_generator11);
    std::cout << "panda is at pose11" << std::endl;

    MotionGenerator motion_generator12(speed_factor, ready_pose);
    robot.control(motion_generator12);
    std::cout << "panda is at ready pose" << std::endl;

    std::cout << "Motion finished" << std::endl;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    //return -1;
  }

  return 0;
}

//grasp object
  //if (argc != 5) {
   // std::cerr << "Usage: ./grasp_object <gripper-hostname> <speed> <homing> <object-width>" << std::endl;
   // return -1;
 // }

