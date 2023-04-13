/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
   Desc:   A demo to show MoveIt Task Constructor in action
*/

// ROS
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <franka_gripper/franka_gripper.h>
#include <franka_gripper/GraspAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iostream>

// MTC pick/place demo implementation
#include <moveit_task_constructor_demo/franka_pick_task.h>

constexpr char LOGNAME[] = "moveit_task_constructor_demo";

bool close_gripper() {	
	actionlib::SimpleActionClient<franka_gripper::GraspAction> ac("/franko/franka_gripper/grasp", true);

	// # Waits until the action server has started up and started
	// # listening for goals.
	ac.waitForServer();

	// # Creates a goal to send to the action server.
	franka_gripper::GraspGoal action_goal;
	action_goal.width = 0.01;
	action_goal.epsilon.inner = 0.005;
	action_goal.epsilon.outer = 0.005;
	action_goal.speed = 0.1;
	action_goal.force = 5;


	// # Sends the goal to the action server.
	ac.sendGoal(action_goal);

	// # Waits for the server to finish performing the action.
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	// # Prints out the result of executing the action
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
		return true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		ac.cancelGoal();
		return false;
	}
	//return true;
}

bool open_gripper() {	
	actionlib::SimpleActionClient<franka_gripper::MoveAction> ac("/franko/franka_gripper/move", true);
	// client = actionlib.SimpleActionClient('/franko/franka_gripper/grasp', franka_gripper.msg.GraspAction)

	// # Waits until the action server has started up and started
	// # listening for goals.
	ac.waitForServer();

	// # Creates a goal to send to the action server.
	franka_gripper::MoveGoal action_goal;
	action_goal.width = 0.07;
	action_goal.speed = 0.1;

	// # Sends the goal to the action server.
	ac.sendGoal(action_goal);

	// # Waits for the server to finish performing the action.
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	// # Prints out the result of executing the action
	// return client.get_result()
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
		return true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		ac.cancelGoal();
		return false;
	}
	//return true;
}

bool pick(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");

	moveit_task_constructor_demo::resetDemoScene(pnh);
	moveit_task_constructor_demo::spawnPipe(pnh, "pipe");

	// Construct and run pick/place task
	moveit_task_constructor_demo::FrankaPickTask franka_pick_task("franka_pick_task", pnh);
	if (!franka_pick_task.init()) {
		ROS_INFO_NAMED(LOGNAME, "Initialization failed");
		return false;
	}

	bool gripper_open = open_gripper();

	if (gripper_open && franka_pick_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeded");
		if (pnh.param("execute", false)) {
			//getchar();
			if (open_gripper()) {
				franka_pick_task.execute();
				ROS_INFO_NAMED(LOGNAME, "Execution complete");
				if (close_gripper()) {
					ROS_INFO_NAMED(LOGNAME, "Execution complete");
					// moveit_task_constructor_demo::MoveHomeTask move_home_task("move_home_task", pnh);
					// if (!move_home_task.init()) {
					// 	ROS_INFO_NAMED(LOGNAME, "Initialization failed");
					// 	return false;
					// }
					// if (move_home_task.plan()) {
					// 	ROS_INFO_NAMED(LOGNAME, "Planning succeded");
					// 	move_home_task.execute();
					// }
				} else {
					ROS_INFO_NAMED(LOGNAME, "Execution failed");
					ros::waitForShutdown();
					return false;
				}
			} else {
				ROS_INFO_NAMED(LOGNAME, "Execution failed");
				ros::waitForShutdown();
				return false;
			}
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
		ros::waitForShutdown();
		return false;
	}
	ros::waitForShutdown();
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");
	ros::NodeHandle nh, pnh("~");

	// Handle Task introspection requests from RViz & feedback during execution
	ros::AsyncSpinner spinner(2);
	spinner.start();

	//moveit_task_constructor_demo::setupDemoScene(pnh);

	ros::ServiceServer assembly_service = nh.advertiseService("franko_assemble", pick);
	ROS_INFO("Franko: ready to assemble.");

	// Keep introspection alive
	ros::waitForShutdown();
	return 0;
}
