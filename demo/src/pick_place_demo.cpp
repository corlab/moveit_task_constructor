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
#include <moveit_task_constructor_demo/pick_place_task.h>
#include <moveit_task_constructor_demo/move_home_task.h>
#include <moveit_task_constructor_demo/hold_task.h>

constexpr char LOGNAME[] = "moveit_task_constructor_demo";
sensor_msgs::JointState gripper_state;
int t_id = 1;
int l_id = 1;


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

	moveit_task_constructor_demo::setupDemoScene(pnh);

	// Construct and run pick/place task
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", pnh);
	if (!pick_place_task.init("objectT" + std::to_string(t_id))) {
		ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Initialization failed");
		//return false;
		res.success = false;
		res.message = "PickPlaceTask Initialization failed";
		return true;
	}

	bool gripper_open = open_gripper();

	if (gripper_open && pick_place_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Planning succeded");
		if (pnh.param("execute", false)) {
			if (open_gripper()) {
				pick_place_task.execute();
				ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Execution complete");
				if (close_gripper()) {
					moveit_task_constructor_demo::MoveHomeTask move_home_task("move_home_task", pnh);
					if (!move_home_task.init("objectT" + std::to_string(t_id), "T")) {
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
							t_id += 1;
							if (t_id > 4) {
								t_id = 1;
							}
							res.success = false;
							res.message = "Grasp failed gripper closed to little or to much.";
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
				} else {
					ROS_INFO_NAMED(LOGNAME, "Gripper Execution failed");
					//ros::waitForShutdown();
					//return false;
					res.success = false;
					res.message = "Gripper Execution failed";
					return true;
				}
			} else {
				ROS_INFO_NAMED(LOGNAME, "Execution failed");
				//ros::waitForShutdown();
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
		//ros::waitForShutdown();
		//return false;
		res.success = false;
		res.message = "Planning failed";
		return true;
	}
	//ros::waitForShutdown();
	t_id += 1;
	if (t_id > 4) {
		t_id = 1;
	}
	res.success = true;
	res.message = "I've got T.";
	return true;
}

bool pickL(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");

	moveit_task_constructor_demo::setupDemoScene(pnh);

	// Construct and run pick/place task
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", pnh);
	if (!pick_place_task.init("objectL" + std::to_string(l_id))) {
		ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Initialization failed");
		//return false;
		res.success = false;
		res.message = "PickPlaceTask Initialization failed";
		return true;
	}

	bool gripper_open = open_gripper();

	if (gripper_open && pick_place_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Planning succeded");
		if (pnh.param("execute", false)) {
			if (open_gripper()) {
				pick_place_task.execute();
				ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Execution complete");
				if (close_gripper()) {
					moveit_task_constructor_demo::MoveHomeTask move_home_task("move_home_task", pnh);
					if (!move_home_task.init("objectL" + std::to_string(l_id), "L")) {
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
							l_id += 1;
							if (l_id > 4) {
								l_id = 1;
							}
							res.success = false;
							res.message = "Grasp failed gripper closed to little or to much.";
							return true;
						}
					}
				} else {
					ROS_INFO_NAMED(LOGNAME, "Gripper Execution failed");
					//ros::waitForShutdown();
					//return false;
					res.success = false;
					res.message = "Gripper Execution failed";
					return true;
				}
			} else {
				ROS_INFO_NAMED(LOGNAME, "Execution failed");
				//ros::waitForShutdown();
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
		//ros::waitForShutdown();
		//return false;
		res.success = false;
		res.message = "Planning failed";
		return true;
	}
	//ros::waitForShutdown();
	l_id += 1;
	if (l_id > 4) {
		l_id = 1;
	}
	res.success = true;
	res.message = "I've got T.";
	return true;
}

bool hold(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");
	// Construct and run pick/place task
	std::string place_name = "place_pose1";
	moveit_task_constructor_demo::HoldTask hold_task("hold_task_bottom", place_name, pnh);
	if (!hold_task.init()) {
		ROS_INFO_NAMED(LOGNAME, "Initialization failed");
		res.success = false;
		res.message = "not holding";
		return true;
	}

	if (hold_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeded");
		if (pnh.param("execute", false)) {
			//getchar();
			hold_task.execute();
			ROS_INFO_NAMED(LOGNAME, "Execution complete");
			moveit_task_constructor_demo::spawnPipe(pnh, "assembly_object");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
		//ros::waitForShutdown();
		res.success = false;
		res.message = "not holding.";
		return true;	
	}
	//ros::waitForShutdown();
	res.success = true;
	res.message = "holding";
	return true;
}

bool hold1(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");
	// Construct and run pick/place task
	std::string place_name = "place_pose2";
	moveit_task_constructor_demo::HoldTask hold_task("hold_task_front", place_name, pnh);
	if (!hold_task.init()) {
		ROS_INFO_NAMED(LOGNAME, "Initialization failed");
		res.success = false;
		res.message = "not holding.";
		return true;
	}

	if (hold_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeded");
		if (pnh.param("execute", false)) {
			//getchar();
			hold_task.execute();
			ROS_INFO_NAMED(LOGNAME, "Execution complete");
			moveit_task_constructor_demo::spawnPipe(pnh, "assembly_object1");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
		//ros::waitForShutdown();
		res.success = false;
		res.message = "not holding";
		return true;
	}
	//ros::waitForShutdown();
	res.success = true;
	res.message = "holding.";
	return true;
}

bool hold2(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");
	// Construct and run pick/place task
	std::string place_name = "place_pose3";
	moveit_task_constructor_demo::HoldTask hold_task("hold_task_back", place_name, pnh);
	if (!hold_task.init()) {
		ROS_INFO_NAMED(LOGNAME, "Initialization failed");
		res.success = false;
		res.message = "not holding.";
		return true;
	}

	if (hold_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeded");
		if (pnh.param("execute", false)) {
			//getchar();
			hold_task.execute();
			ROS_INFO_NAMED(LOGNAME, "Execution complete");
			moveit_task_constructor_demo::spawnPipe(pnh, "assembly_object1");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
		//ros::waitForShutdown();
		res.success = false;
		res.message = "not holding.";
		return true;
	}
	//ros::waitForShutdown();
	res.success = true;
	res.message = "holding.";
	return true;
}

void gripperCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	gripper_state = *msg;
	//ROS_INFO("I heard: [%f, %f]", msg->position[0], msg->position[1]);
	ROS_DEBUG("I heard: [%f, %f]", gripper_state.position[0], gripper_state.position[1]);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");
	ros::NodeHandle nh, pnh("~");

	ros::Subscriber gripper_sub = nh.subscribe("/franko/franka_gripper/joint_states", 1, gripperCallback);

	// Handle Task introspection requests from RViz & feedback during execution
	ros::AsyncSpinner spinner(2);
	spinner.start();

	//moveit_task_constructor_demo::setupDemoScene(pnh);

	ros::ServiceServer pick_back_service = nh.advertiseService("franko_mtc_pick_back", pick);
	ROS_INFO("Franko: ready to pick T.");
	//ros::spin();*/

	ros::ServiceServer pick_front_service = nh.advertiseService("franko_mtc_pick_front", pickL);
	ROS_INFO("Franko: ready to pick L.");

	ros::ServiceServer hold_service = nh.advertiseService("franko_hold_bottom", hold);
	ROS_INFO("Franko: ready to hold.");

	ros::ServiceServer hold1_service = nh.advertiseService("franko_hold_front", hold1);
	ROS_INFO("Franko: ready to hold. Again");

	ros::ServiceServer hold2_service = nh.advertiseService("franko_hold_back", hold2);
	ROS_INFO("Franko: ready to hold. Again");

	// Construct and run pick/place task
	// moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", pnh);
	// if (!pick_place_task.init()) {
	//    ROS_INFO_NAMED(LOGNAME, "Initialization failed");
	//    return 1;
	// }

	// if (pick_place_task.plan()) {
	//    ROS_INFO_NAMED(LOGNAME, "Planning succeded");
	//    if (pnh.param("execute", false)) {
		  
	//       pick_place_task.execute();
	//       ROS_INFO_NAMED(LOGNAME, "Execution complete");
	//    } else {
	//       ROS_INFO_NAMED(LOGNAME, "Execution disabled");
	//    }
	// } else {
	//    ROS_INFO_NAMED(LOGNAME, "Planning failed");
	// }

	// Keep introspection alive
	ros::waitForShutdown();
	return 0;
}
