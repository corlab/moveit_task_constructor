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
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometric_shapes/shape_operations.h>
#include <map>
#include <tuple>

constexpr char LOGNAME[] = "moveit_task_constructor_demo";
sensor_msgs::JointState gripper_state;
int t_id = 1;
int l_id = 1;
int t_assembled = 1;
std::map<std::string, std::vector<std::string>> pick_objects;
std::tuple <std::string, std::vector<std::string>> picked_objects;

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
	if (psi.applyCollisionObject(object)) {
		pick_objects[object.id] = {object.id};
	} else {
		throw std::runtime_error("Failed to spawn object: " + object.id);
	}
}

void spawnAttatchedObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::AttachedCollisionObject& object) {
	if (psi.applyAttachedCollisionObject(object)) {
		pick_objects[std::get<0>(picked_objects)].push_back(object.object.id);
		std::get<1>(picked_objects).push_back(object.object.id);
	} else {
		throw std::runtime_error("Failed to spawn attatched object: " + object.object.id);
	}
}

moveit_msgs::CollisionObject createTable(const ros::NodeHandle& pnh) {
	std::string table_name, table_reference_frame;
	std::vector<double> table_dimensions;
	geometry_msgs::Pose pose;
	std::size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_name", table_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_reference_frame", table_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_dimensions", table_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

	moveit_msgs::CollisionObject object;
	object.id = table_name;
	object.header.frame_id = table_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	object.primitives[0].dimensions = table_dimensions;
	pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::CollisionObject createMagazine(const ros::NodeHandle& pnh, const std::string& type) {
	std::string magazine_name, magazine_reference_frame, magazine_file;
	//std::vector<double> magazine_dimensions;
	geometry_msgs::Pose pose;
	std::size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazine_name", magazine_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazine_reference_frame", magazine_reference_frame);
	//errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazine_dimensions", magazine_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazine_file", magazine_file);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazine_pose" + type, pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

	moveit_msgs::CollisionObject object;
	object.id = magazine_name + type;
	object.header.frame_id = magazine_reference_frame;
	Eigen::Vector3d scale_vec(0.001, 0.001, 0.001);
	shapes::Mesh* mesh = shapes::createMeshFromResource("file:" + magazine_file, scale_vec);
	ROS_INFO("mesh loaded");
	shape_msgs::Mesh obj_mesh;
	shapes::ShapeMsg mesh_msg;  
	shapes::constructMsgFromShape(mesh, mesh_msg);    
	obj_mesh = boost::get<shape_msgs::Mesh>(mesh_msg);  
	object.meshes.resize(1);
	object.mesh_poses.resize(1);
	object.meshes[0] = obj_mesh;
	// object.primitives.resize(1);
	// object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	// object.primitives[0].dimensions = magazine_dimensions;
	// pose.position.z -= 0.5 * magazine_dimensions[2];  // align surface with world
	// object.primitive_poses.push_back(pose);
	object.mesh_poses[0] = pose;
	return object;
}

moveit_msgs::CollisionObject createObject(const ros::NodeHandle& pnh, const std::string object_id) {
	std::string object_name, object_reference_frame, object_file;
	//std::vector<double> object_dimensions;
	geometry_msgs::Pose pose;
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(LOGNAME, pnh, object_id + "_name", object_name);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
	//error += !rosparam_shortcuts::get(LOGNAME, pnh, object_id + "_dimensions", object_dimensions);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, object_id + "_file", object_file);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, object_id + "_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, error);

	moveit_msgs::CollisionObject object;
	object.id = object_name;
	object.header.frame_id = object_reference_frame;
	Eigen::Vector3d scale_vec(0.001, 0.001, 0.001);
	shapes::Mesh* mesh = shapes::createMeshFromResource("file:" + object_file, scale_vec);
	//mesh->scaleAndPadd(0.001,0);
	ROS_INFO("mesh loaded");
	shape_msgs::Mesh obj_mesh;
	shapes::ShapeMsg mesh_msg;  
	shapes::constructMsgFromShape(mesh, mesh_msg);    
	obj_mesh = boost::get<shape_msgs::Mesh>(mesh_msg);  
	object.meshes.resize(1);
	object.mesh_poses.resize(1);
	object.meshes[0] = obj_mesh;
	//object.primitives.resize(1);
	//object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	//object.primitives[0].dimensions = object_dimensions;
	//pose.position.z += 0.5 * object_dimensions[0];
	//object.primitive_poses.push_back(pose);
	object.mesh_poses[0] = pose;
	return object;
}

moveit_msgs::AttachedCollisionObject createAssemblyObject(const ros::NodeHandle& pnh, const std::string& name) {
	ROS_INFO_STREAM_NAMED(LOGNAME, "spawn Assembly Object " + name);
	//std::string assembly_object_name;
	std::string assembly_object_reference_frame;
	std::vector<double> assembly_object_dimensions;
	geometry_msgs::Pose assembly_pose;
	std::string link_name;
	std::size_t error = 0;
	//error += !rosparam_shortcuts::get(LOGNAME, pnh, "assembly_object_name", assembly_object_name);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", assembly_object_reference_frame);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "assembly_object_dimensions", assembly_object_dimensions);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, name + "_pose", assembly_pose);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", link_name);
	rosparam_shortcuts::shutdownIfError(LOGNAME, error);

	moveit_msgs::AttachedCollisionObject attatched_object;
	//attatched_object.object.id = assembly_object_name;
	attatched_object.object.id = name;
	attatched_object.object.header.frame_id = assembly_object_reference_frame;
	attatched_object.link_name = link_name;
	attatched_object.object.primitives.resize(1);
	attatched_object.object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	attatched_object.object.primitives[0].dimensions = assembly_object_dimensions;
	assembly_pose.position.z += 0.5 * assembly_object_dimensions[0];
	attatched_object.object.primitive_poses.push_back(assembly_pose);
	return attatched_object;
}

void detatch_objects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
	std::map<std::string, moveit_msgs::AttachedCollisionObject> objects = planning_scene_interface.getAttachedObjects();
	ROS_INFO_STREAM_NAMED("detatch objects", "got all known attached objects: " << objects.size());
	std::vector<std::string> ids = {};
	for (std::pair<std::string, moveit_msgs::AttachedCollisionObject> object : objects) {
		moveit_msgs::AttachedCollisionObject detatch_object = object.second;
		detatch_object.object.operation = detatch_object.object.REMOVE;
		planning_scene_interface.applyAttachedCollisionObject(detatch_object);
	}
	std::map<std::string, moveit_msgs::AttachedCollisionObject> remaining_objects = planning_scene_interface.getAttachedObjects();
	ROS_DEBUG_STREAM_NAMED("detatch objects", "got all known attached objects after clearing: " << remaining_objects.size());
	picked_objects = {};
}


void clear_planning_scene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
	std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface.getObjects();
	ROS_INFO_STREAM_NAMED("clear planning scene", "got all known objects: " << objects.size());
	std::vector<std::string> ids = {};
	for (std::pair<std::string, moveit_msgs::CollisionObject> object : objects) {
		moveit_msgs::CollisionObject remove_object = object.second;
		remove_object.operation = remove_object.REMOVE;
		planning_scene_interface.applyCollisionObject(remove_object);
	}
	std::map<std::string, moveit_msgs::CollisionObject> remaining_objects = planning_scene_interface.getObjects();
	ROS_INFO_STREAM_NAMED("clear planning scene", "got all known objects after clearing: " << remaining_objects.size());
	pick_objects = {};
}

void setupDemoScene(ros::NodeHandle& pnh) {
	// Add table and object to planning scene
	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	//addCollisionObjects(psi);
	detatch_objects(psi);
	clear_planning_scene(psi);
	if (pnh.param("spawn_table", true))
	   spawnObject(psi, createTable(pnh));
	if (pnh.param("spawn_magazines", true)) {
	   spawnObject(psi, createMagazine(pnh, "T"));
	   spawnObject(psi, createMagazine(pnh, "L"));
	}
	spawnObject(psi, createObject(pnh, "objectT1"));
	spawnObject(psi, createObject(pnh, "objectT2"));
	spawnObject(psi, createObject(pnh, "objectT3"));
	spawnObject(psi, createObject(pnh, "objectT4"));
	spawnObject(psi, createObject(pnh, "objectL1"));
	spawnObject(psi, createObject(pnh, "objectL2"));
	spawnObject(psi, createObject(pnh, "objectL3"));
	spawnObject(psi, createObject(pnh, "objectL4"));
    // pick_objects["objectT2"] = {"objectT2"};
    // pick_objects["objectT3"] = {"objectT3"};
	// pick_objects["objectT4"] = {"objectT4"};
	// pick_objects["objectL1"] = {"objectL1"};
    // pick_objects["objectL2"] = {"objectL2"};
    // pick_objects["objectL3"] = {"objectL3"};
	// pick_objects["objectL4"] = {"objectL4"};
	//spawnObject(psi, createObject(pnh, "objectI"));
	//spawnObject(psi, createAssemblyObject(pnh));
}

void spawnPipe(ros::NodeHandle& pnh, const std::string& name) {
	ros::Duration(1.0).sleep();
	moveit::planning_interface::PlanningSceneInterface psi;
	spawnAttatchedObject(psi, createAssemblyObject(pnh, name));
}

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

bool resetWorld(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");
	setupDemoScene(pnh);
	res.success = false;
	res.message = "Groundhog Day";
	return true;
}

bool pick(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");

	std::string object_name = "objectT" + std::to_string(t_id);
	ROS_INFO_STREAM_NAMED(LOGNAME, "Picking " << object_name);

	// Construct and run pick/place task
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", pnh);
	if (!pick_place_task.init(object_name)) {
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
					if (!move_home_task.init(object_name, "T")) {
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
	if (!(std::get<0>(picked_objects) == "")) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "World model already contains picked object");
	}
	//picked_objects.push_back(object_name);
	//pick_objects[std::get<0>(picked_objects)].push_back(object.object.id);
	std::get<0>(picked_objects) = object_name;
	std::get<1>(picked_objects).push_back(object_name);
	res.success = true;
	res.message = "I've got T.";
	return true;
}

bool pickL(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");

	std::string object_name = "objectL" + std::to_string(l_id);
	ROS_INFO_STREAM_NAMED(LOGNAME, "Picking " << object_name);

	// moveit_task_constructor_demo::setupDemoScene(pnh);
	// pick_objects["objectT1"] = {"objectT1"};
    // pick_objects["objectT2"] = {"objectT2"};
    // pick_objects["objectT3"] = {"objectT3"};
	// pick_objects["objectT4"] = {"objectT4"};
	// pick_objects["objectL1"] = {"objectL1"};
    // pick_objects["objectL2"] = {"objectL2"};
    // pick_objects["objectL3"] = {"objectL3"};
	// pick_objects["objectL4"] = {"objectL4"};

	// Construct and run pick/place task
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", pnh);
	if (!pick_place_task.init(object_name)) {
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
					if (!move_home_task.init(object_name, "L")) {
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
	if (!(std::get<0>(picked_objects) == "")) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "World model already contains picked object");
	}
	//picked_objects.push_back(object_name);
	//pick_objects[std::get<0>(picked_objects)].push_back(object.object.id);
	std::get<0>(picked_objects) = object_name;
	std::get<1>(picked_objects).push_back(object_name);
	res.success = true;
	res.message = "I've got T.";
	return true;
}

bool pickAssembledT(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");

	//moveit_task_constructor_demo::setupDemoScene(pnh);
	if (!(std::get<0>(picked_objects) == "")) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "World model already contains picked object");
	}
	std::string object_name;
	for (const auto& po : pick_objects) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, po.first << " has value " << po.second.at(0));
		if ((po.first.find("objectT") != std::string::npos) && (po.second.size() > 1)) {
			object_name = po.first;
			ROS_ERROR_STREAM_NAMED(LOGNAME, "Found assembled T " << po.first << " with parts:");
			for (auto part : po.second) {
				ROS_ERROR_STREAM_NAMED(LOGNAME, part);
			}
			break;
		}
	}
	if (object_name == "" || object_name.empty()) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "World doesn't contain an assembled T-group to pick");
		res.success = false;
		res.message = "World doesn't contain an assembled T-group to pick";
		return true;
	}

	// Construct and run pick/place task
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", pnh);
	if (!pick_place_task.init("assembledT" + std::to_string(t_id))) {
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
					if (!move_home_task.init("assembledT" + std::to_string(t_assembled), "Tgroup")) {
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
							t_assembled == 1 ? 2 : 1;
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
		ros::waitForShutdown();
		//return false;
		res.success = false;
		res.message = "Planning failed";
		return true;
	}
	//ros::waitForShutdown();
	t_assembled == 1 ? 2 : 1;
	res.success = true;
	res.message = "I've got T group.";
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
			spawnPipe(pnh, "assembly_object");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
		ros::waitForShutdown();
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
			spawnPipe(pnh, "assembly_object1");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
		ros::waitForShutdown();
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
			spawnPipe(pnh, "assembly_object2");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
		ros::waitForShutdown();
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
	setupDemoScene(pnh);

	ros::ServiceServer pick_back_service = nh.advertiseService("franko_mtc_pick_back", pick);
	ROS_INFO("Franko: ready to pick T.");

	ros::ServiceServer pick_back_service = nh.advertiseService("franko_mtc_pick_T", pickAssembledT);
	ROS_INFO("Franko: ready to pick T-group.");

	ros::ServiceServer pick_front_service = nh.advertiseService("franko_mtc_pick_front", pickL);
	ROS_INFO("Franko: ready to pick L.");

	ros::ServiceServer hold_service = nh.advertiseService("franko_hold_bottom", hold);
	ROS_INFO("Franko: ready to hold.");

	ros::ServiceServer hold1_service = nh.advertiseService("franko_hold_front", hold1);
	ROS_INFO("Franko: ready to hold. Again");

	ros::ServiceServer hold2_service = nh.advertiseService("franko_hold_back", hold2);
	ROS_INFO("Franko: ready to hold. Again");

	ros::ServiceServer hold2_service = nh.advertiseService("franko_reset_world", resetWorld);
	ROS_INFO("Franko: ready to reset");

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
