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
#include <sensor_msgs/JointState.h>
#include <franka_gripper/franka_gripper.h>
#include <franka_gripper/GraspAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iostream>

// MTC pick/place demo implementation
#include <moveit_task_constructor_demo/franka_pick_pipe_task.h>
#include <moveit_task_constructor_demo/franka_place_task.h>
#include <moveit_task_constructor_demo/move_home_task.h>

constexpr char LOGNAME[] = "franka_moveit_task_constructor_demo";
sensor_msgs::JointState gripper_state;
int pipe_id = 1;
int pick_places = 6;
int place_id = 1;


bool close_gripper() {	
	actionlib::SimpleActionClient<franka_gripper::GraspAction> ac("/franka_gripper/grasp", true);

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

bool grasp_successful() {
	// successfull grip [0.003919643349945545, 0.003919643349945545]
	// unsuccessfull grip position: [0.00037331500789150596, 0.00037331500789150596]
	if (gripper_state.position[0] > 0.002 && gripper_state.position[1] > 0.002 && gripper_state.position[0] < 0.005 && gripper_state.position[1] < 0.005) {
		return true;
	}
	ROS_ERROR_NAMED(LOGNAME, "grasp unsuccessful gripper state is [%f, %f]", gripper_state.position[0], gripper_state.position[1]);
	return false;
}

bool open_gripper() {	
	actionlib::SimpleActionClient<franka_gripper::MoveAction> ac("/franka_gripper/move", true);
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

bool setupDemo(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");

	ROS_INFO_NAMED(LOGNAME, "setupDemoScene");

	franka_moveit_task_constructor_demo::setupDemoScene(pnh);
}

bool pick(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");

	// ROS_INFO_NAMED(LOGNAME, "setupDemoScene");

	// franka_moveit_task_constructor_demo::setupDemoScene(pnh);

	ROS_INFO_NAMED(LOGNAME, "pick_place_task init");

	// Construct and run pick/place task
	franka_moveit_task_constructor_demo::FrankaPickPlaceTask pick_place_task("franka_pick_pipe_place_task", pnh);
	if (!pick_place_task.init("pipe" + std::to_string(pipe_id))) {
		ROS_INFO_NAMED(LOGNAME, "FrankaPickPlaceTask Initialization failed");
		//return false;
		res.success = false;
		res.message = "FrankaPickPlaceTask Initialization failed";
		return true;
	}

	bool gripper_open = open_gripper();

	if (gripper_open && pick_place_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "FrankaPickPlaceTask Planning succeded");
		if (pnh.param("execute", false)) {
			if (open_gripper()) {
				pick_place_task.execute();
				ROS_INFO_NAMED(LOGNAME, "FrankaPickPlaceTask Execution complete");
				if (close_gripper()) {
					ROS_INFO_NAMED(LOGNAME, "closed gripper");
					moveit_task_constructor_demo::MoveHomeTask move_home_task("move_home_task", pnh);
					if (!move_home_task.init("pipe", pipe_id, true)) {
						ROS_INFO_NAMED(LOGNAME, "MoveHomeTask Initialization failed");
						res.success = false;
						res.message = "MoveHomeTask Initialization failed";
						return true;
					}
					if (move_home_task.plan()) {
						ROS_INFO_NAMED(LOGNAME, "MoveHomeTask Planning succeded");
						move_home_task.execute();
						ROS_INFO_NAMED(LOGNAME, "MoveHomeTask Execution complete");
						if (!grasp_successful()) {
							ROS_INFO_NAMED(LOGNAME, "Grasp failed gripper closed to little or to much.");
							//return false;
							pipe_id += 1;
							if (pipe_id > pick_places) {
								pipe_id = 1;
							}
							//ros::waitForShutdown();
							res.success = false;
							res.message = "Grasp failed gripper closed to little or to much.";
							return true;
						}
					}
				} else {
					ROS_INFO_NAMED(LOGNAME, "Gripper Execution failed");
					// ros::waitForShutdown();
					//return false;
					res.success = false;
					res.message = "Gripper Execution failed";
					return true;
				}
			} else {
				ROS_INFO_NAMED(LOGNAME, "Execution failed");
				// ros::waitForShutdown();
				//return false;
				res.success = false;
				res.message = "Execution failed";
				return true;
			}
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
			//return false;
			res.success = false;
			res.message = "Execution disabled";
			return true;
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
		ros::waitForShutdown();
		//return false;
		res.success = false;
		res.message = "Planning failed";
		return true;
	}
	//ros::waitForShutdown();
	pipe_id += 1;
	if (pipe_id > pick_places) {
		pipe_id = 1;
	}
	res.success = true;
	res.message = "I've picked up a pipe.";
	return true;
}

bool place(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");

	// ROS_INFO_NAMED(LOGNAME, "setupDemoScene");

	// franka_moveit_task_constructor_demo::setupDemoScene(pnh);

	ROS_INFO_NAMED(LOGNAME, "place_task init");

	// Construct and run place task
	franka_moveit_task_constructor_demo::FrankaPlaceTask place_task("franka_place_task", "place_pose" + std::to_string(place_id), pnh);
	if (!place_task.init()) {
		ROS_INFO_NAMED(LOGNAME, "FrankaPlaceTask Initialization failed");
		//return false;
		res.success = false;
		res.message = "FrankaPlaceTask Initialization failed";
		return true;
	}

	if (place_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "FrankaPlaceTask Planning succeded");
		if (pnh.param("execute", false)) {
			place_task.execute();
			ROS_INFO_NAMED(LOGNAME, "FrankaPlaceTask Execution complete");
			if (open_gripper()) {
				ROS_INFO_NAMED(LOGNAME, "opened gripper");
				if (place_id == 1) {
					place_id = 2;
				} else {
					place_id = 1;
				}
				moveit_task_constructor_demo::MoveHomeTask move_home_task("move_home_task", pnh);
				if (!move_home_task.init("pipe", pipe_id, false)) {
					ROS_INFO_NAMED(LOGNAME, "MoveHomeTask Initialization failed");
					res.success = false;
					res.message = "MoveHomeTask Initialization failed";
					return true;
				}
				if (move_home_task.plan()) {
					ROS_INFO_NAMED(LOGNAME, "MoveHomeTask Planning succeded");
					move_home_task.execute();
					ROS_INFO_NAMED(LOGNAME, "MoveHomeTask Execution complete");
				} else {
					ROS_INFO_NAMED(LOGNAME, "Planning moveHome failed");
					ros::waitForShutdown();
					//return false;
					res.success = false;
					res.message = "Planning failed";
					return true;
				}
			} else {
				ROS_INFO_NAMED(LOGNAME, "Gripper Execution failed");
				// ros::waitForShutdown();
				//return false;
				res.success = false;
				res.message = "Gripper Execution failed";
				return true;
			}
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
			ros::waitForShutdown();
			//return false;
			res.success = false;
			res.message = "Execution disabled";
			return true;
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning place failed");
		ros::waitForShutdown();
		//return false;
		res.success = false;
		res.message = "Planning failed";
		return true;
	}
	//ros::waitForShutdown();
	// pipe_id += 1;
	// if (pipe_id > pick_places) {
	// 	pipe_id = 1;
	// }
	res.success = true;
	res.message = "I've placed up a pipe.";
	return true;
}

void gripperCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	gripper_state = *msg;
	//ROS_INFO("I heard: [%f, %f]", msg->position[0], msg->position[1]);
	ROS_DEBUG("I heard: [%f, %f]", gripper_state.position[0], gripper_state.position[1]);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "franka_mtc_place_tutorial");
	ros::NodeHandle nh, pnh("~");

	ros::Subscriber gripper_sub = nh.subscribe("/franka_gripper/joint_states", 1, gripperCallback);

	// Handle Task introspection requests from RViz & feedback during execution
	ros::AsyncSpinner spinner(2);
	spinner.start();

	franka_moveit_task_constructor_demo::setupDemoScene(pnh);
	ros::ServiceServer setup_service = nh.advertiseService("franka_mtc_setupDemo", setupDemo);
	ROS_INFO("Franka: ready to setup Demo Scene.");

	ros::ServiceServer pick_service = nh.advertiseService("franka_mtc_pick_pipe", pick);
	ROS_INFO("Franka: ready to pick a pipe.");

	ros::ServiceServer place_service = nh.advertiseService("franka_mtc_place_pipe", place);
	ROS_INFO("Franka: ready to place a pipe.");

	// Keep introspection alive
	ros::waitForShutdown();
	return 0;
}
