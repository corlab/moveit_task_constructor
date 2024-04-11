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
#include <moveit_task_constructor_demo/place_task.h>
#include <moveit_task_constructor_demo/move_home_task.h>
#include <moveit_task_constructor_demo/lift_task.h>
#include <moveit_task_constructor_demo/hold_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometric_shapes/shape_operations.h>
#include <humation_messages/FrankoWorldSetup.h>
#include <map>
#include <tuple>

constexpr char LOGNAME[] = "moveit_task_constructor_demo";
sensor_msgs::JointState gripper_state;
int t_id = 1;
int l_id = 1;
int t_assembled_pick = 1;
int t_assembled_place = 0;
int pipe_count = 0;
std::map<std::string, std::vector<std::string>> pick_objects;
std::tuple <std::string, std::vector<std::string>> picked_objects;

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
	if (psi.applyCollisionObject(object)) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "spawn Object ");
		pick_objects[object.id] = {object.id};
		ROS_ERROR_STREAM_NAMED(LOGNAME, "picked object: " << std::get<0>(picked_objects));
		for (auto &&j : std::get<1>(picked_objects))
		{
			ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
		}
		for (auto &&i : pick_objects)
		{
			ROS_ERROR_STREAM_NAMED(LOGNAME, "pick object name: " << i.first);
			for (auto &&j : i.second)
			{
				ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
			}
			

		}
	} else {
		throw std::runtime_error("Failed to spawn object: " + object.id);
	}
}

void spawnAttatchedObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::AttachedCollisionObject& object) {
	if (psi.applyAttachedCollisionObject(object)) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "spawn attached Object ");
		pick_objects[std::get<0>(picked_objects)].push_back(object.object.id);
		std::get<1>(picked_objects).push_back(object.object.id);
		ROS_ERROR_STREAM_NAMED(LOGNAME, "picked object: " << std::get<0>(picked_objects));
		for (auto &&j : std::get<1>(picked_objects))
		{
			ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
		}
		for (auto &&i : pick_objects)
		{
			ROS_ERROR_STREAM_NAMED(LOGNAME, "pick object name: " << i.first);
			for (auto &&j : i.second)
			{
				ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
			}
			

		}
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

moveit_msgs::CollisionObject createMagazineTAssembled(const ros::NodeHandle& pnh) {
	//std::string magazine_name, magazine_reference_frame, magazine_file;
	std::string magazine_reference_frame, magazine_file;
	//std::vector<double> magazine_dimensions;
	geometry_msgs::Pose pose;
	std::size_t errors = 0;
	//errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazine_name", magazine_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazine_reference_frame", magazine_reference_frame);
	//errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazine_dimensions", magazine_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazineT_assembled_file", magazine_file);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazineT_assembled_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

	moveit_msgs::CollisionObject object;
	object.id = "magazineT_assembled";
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
	attatched_object.object.id = name + "_" + std::to_string(pipe_count);
	pipe_count++;
	attatched_object.object.header.frame_id = assembly_object_reference_frame;
	attatched_object.link_name = link_name;
	attatched_object.object.primitives.resize(1);
	attatched_object.object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	attatched_object.object.primitives[0].dimensions = assembly_object_dimensions;
	assembly_pose.position.z += 0.5 * assembly_object_dimensions[0];
	attatched_object.object.primitive_poses.push_back(assembly_pose);
	return attatched_object;
}

moveit_msgs::CollisionObject createAssembledMeshObject(const ros::NodeHandle& pnh, const std::string& object_name, const int place) {
	// std::string object_name, object_reference_frame, object_file;
	std::string object_reference_frame, object_file;
	// //std::vector<double> object_dimensions;
	geometry_msgs::Pose pose;
	std::size_t error = 0;
	// //error += !rosparam_shortcuts::get(LOGNAME, pnh, name + "_name", object_name);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
	// //error += !rosparam_shortcuts::get(LOGNAME, pnh, object_id + "_dimensions", object_dimensions);
	// if (!mesh_file.empty()) {
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "objectT1_file", object_file);
	// }
	error += !rosparam_shortcuts::get(LOGNAME, pnh, object_name + "_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, error);

	moveit_msgs::CollisionObject object;
	object.id = object_name + std::to_string(pipe_count);
	pipe_count++;
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
	pose.position.x += place * 0.07;
	object.mesh_poses[0] = pose;

	return object;
}

moveit_msgs::CollisionObject createAssembledObject(const ros::NodeHandle& pnh, const std::string& name, const int place) {
	// std::string object_name, object_reference_frame, object_file;
	std::string object_reference_frame;
	std::vector<double> object_dimensions;
	geometry_msgs::Pose pose;
	std::size_t error = 0;
	// //error += !rosparam_shortcuts::get(LOGNAME, pnh, name + "_name", object_name);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "assembly_object_dimensions", object_dimensions);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, name + "_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, error);

	moveit_msgs::CollisionObject object;
	//attatched_object.object.id = assembly_object_name;
	object.id = name + std::to_string(pipe_count);
	pipe_count++;
	object.header.frame_id = object_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = object_dimensions;
	pose.position.z += 0.5 * object_dimensions[0];
	pose.position.x += place * 0.07;
	object.primitive_poses.push_back(pose);
	return object;
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
	pipe_count = 0;
	ROS_ERROR_STREAM_NAMED(LOGNAME, "picked object: " << std::get<0>(picked_objects));
	for (auto &&j : std::get<1>(picked_objects))
	{
		ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
	}
	for (auto &&i : pick_objects)
	{
		ROS_ERROR_STREAM_NAMED(LOGNAME, "pick object name: " << i.first);
		for (auto &&j : i.second)
		{
			ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
		}
	}
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
	   spawnObject(psi, createMagazineTAssembled(pnh));
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
	res.success = true;
	res.message = "Groundhog Day";
	return true;
}

bool configureWorld(humation_messages::FrankoWorldSetupRequest& req, humation_messages::FrankoWorldSetupResponse& res) {
	ros::NodeHandle nh, pnh("~");
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
	   spawnObject(psi, createMagazineTAssembled(pnh));
	}
	for (size_t i = 0; i < req.tMagazine.size(); i++)
	{
		if (req.tMagazine[i]) {
			spawnObject(psi, createObject(pnh, "objectT" + std::to_string(i + 1)));
		}
	}

	for (size_t i = 0; i < req.lMagazine.size(); i++)
	{
		if (req.lMagazine[i]) {
			spawnObject(psi, createObject(pnh, "objectL" + std::to_string(i + 1)));
		}
	}

	for (int i = 0; i < req.tAssembledMagazine.size(); i++)
	{
		if (req.tAssembledMagazine[i]) {
			moveit_msgs::CollisionObject root = createAssembledMeshObject(pnh, "assembled_T", i);
			pick_objects[root.id] = {root.id};
			spawnObject(psi, root);
			pick_objects[root.id].push_back("assembled_pipe1" + std::to_string(pipe_count));
			spawnObject(psi, createAssembledObject(pnh, "assembled_pipe1", i));
			pick_objects[root.id].push_back("assembled_pipe2" + std::to_string(pipe_count));
			spawnObject(psi, createAssembledObject(pnh, "assembled_pipe2", i));
			pick_objects[root.id].push_back("assembled_pipe3" + std::to_string(pipe_count));
			spawnObject(psi, createAssembledObject(pnh, "assembled_pipe3", i));
		}
	}
	
	// spawnObject(psi, createObject(pnh, "objectT1"));
	// spawnObject(psi, createObject(pnh, "objectT2"));
	// spawnObject(psi, createObject(pnh, "objectT3"));
	// spawnObject(psi, createObject(pnh, "objectT4"));
	// spawnObject(psi, createObject(pnh, "objectL1"));
	// spawnObject(psi, createObject(pnh, "objectL2"));
	// spawnObject(psi, createObject(pnh, "objectL3"));
	// spawnObject(psi, createObject(pnh, "objectL4"));
	//spawnObject(psi, createObject(pnh, "objectI"));
	//spawnObject(psi, createAssemblyObject(pnh));
	res.success = false;
	res.message = "Groundhog Day";
	return true;
}

bool pick(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");

	setupDemoScene(pnh);

	ROS_ERROR_STREAM_NAMED(LOGNAME, "pick t connector");

	std::string object_name = "objectT" + std::to_string(t_id);
	ROS_INFO_STREAM_NAMED(LOGNAME, "Picking " << object_name);

	// Construct and run pick/place task
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", pnh);
	if (!pick_place_task.init({object_name, {object_name}})) {
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
				if (pick_place_task.execute()) {
					ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Execution complete");
					if (close_gripper()) {
						moveit_task_constructor_demo::LiftTask lift_task("lift_task", pnh);
						if (!lift_task.init({object_name, {object_name}}, "T", true)) {
							ROS_INFO_NAMED(LOGNAME, "LiftTask Initialization failed");
							res.success = false;
							res.message = "LiftTask Initialization failed";
							return true;
						}
						if (lift_task.plan()) {
							ROS_INFO_NAMED(LOGNAME, "LiftTask Planning succeded");
							lift_task.execute();
							ROS_INFO_NAMED(LOGNAME, "LiftTask Execution complete");
							if (!grasp_successful()) {
								ROS_INFO_NAMED(LOGNAME, "Grasp failed gripper closed to little or to much.");
								//return false;
								t_id += 1;
								if (t_id > 4) {
									t_id = 1;
								}
								if (t_id == 2) {
									t_id = 3;
								}
								setupDemoScene(pnh);
								res.success = false;
								res.message = "Grasp failed gripper closed to little or to much.";
								return true;
							}
						} else {
							ROS_INFO_NAMED(LOGNAME, "LiftTask Planning failed");
							//ros::waitForShutdown();
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
					ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Execution failed");
					//ros::waitForShutdown();
					//return false;
					res.success = false;
					res.message = "Execution failed";
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
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
			//return false;
			res.success = false;
			res.message = "Execution disabled";
			return true;
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Planning or Gripper Execution failed");
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
	if (t_id == 2) {
		t_id = 3;
	}
	if (!(std::get<0>(picked_objects) == "")) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "World model already contains picked object");
	}
	//picked_objects.push_back(object_name);
	//pick_objects[std::get<0>(picked_objects)].push_back(object.object.id);
	std::get<0>(picked_objects) = object_name;
	std::get<1>(picked_objects).push_back(object_name);
	ROS_ERROR_STREAM_NAMED(LOGNAME, "picked object: " << std::get<0>(picked_objects));
	for (auto &&j : std::get<1>(picked_objects))
	{
		ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
	}
	for (auto &&i : pick_objects)
	{
		ROS_ERROR_STREAM_NAMED(LOGNAME, "pick object name: " << i.first);
		for (auto &&j : i.second)
		{
			ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
		}
		

	}
	res.success = true;
	res.message = "I've got T.";
	return true;
}

bool pickL(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");

	setupDemoScene(pnh);

	ROS_ERROR_STREAM_NAMED(LOGNAME, "pick l connector");

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
	if (!pick_place_task.init({object_name, {object_name}})) {
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
				if (pick_place_task.execute()) {
					ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Execution complete");
					if (close_gripper()) {
						moveit_task_constructor_demo::LiftTask lift_task("lift_task", pnh);
						if (!lift_task.init({object_name, {object_name}}, "L", true)) {
							ROS_INFO_NAMED(LOGNAME, "LiftTask Initialization failed");
							res.success = false;
							res.message = "LiftTask Initialization failed";
							return true;
						}
						if (lift_task.plan()) {
							ROS_INFO_NAMED(LOGNAME, "LiftTask Planning succeded");
							lift_task.execute();
							ROS_INFO_NAMED(LOGNAME, "LiftTask Execution complete");
							if (!grasp_successful()) {
								ROS_INFO_NAMED(LOGNAME, "Grasp failed gripper closed to little or to much.");
								//return false;
								l_id += 1;
								if (l_id > 4) {
									l_id = 1;
								}
								setupDemoScene(pnh);
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
					ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Execution failed");
					//ros::waitForShutdown();
					//return false;
					res.success = false;
					res.message = "Execution failed";
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
	ROS_ERROR_STREAM_NAMED(LOGNAME, "picked object: " << std::get<0>(picked_objects));
	for (auto &&j : std::get<1>(picked_objects))
	{
		ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
	}
	for (auto &&i : pick_objects)
	{
		ROS_ERROR_STREAM_NAMED(LOGNAME, "pick object name: " << i.first);
		for (auto &&j : i.second)
		{
			ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
		}
		

	}
	res.success = true;
	res.message = "I've got T.";
	return true;
}

void printAttachedObject(std::pair<std::string, moveit_msgs::AttachedCollisionObject> object) {
	ROS_WARN_STREAM_NAMED(LOGNAME, "-------------------------------");
	ROS_WARN_STREAM_NAMED(LOGNAME, "id: " << object.first << " = " << object.second.object.id);
	ROS_WARN_STREAM_NAMED(LOGNAME, "link_name: " << object.second.link_name);
	ROS_WARN_STREAM_NAMED(LOGNAME, "frame_id: " << object.second.object.header.frame_id);
	ROS_WARN_STREAM_NAMED(LOGNAME, "position relative to the header frame x: " << object.second.object.pose.position.x << ", y: " << object.second.object.pose.position.y << ", z: " << object.second.object.pose.position.z);
	ROS_WARN_STREAM_NAMED(LOGNAME, "orientation relative to the header frame x: " << object.second.object.pose.orientation.x << ", y: " << object.second.object.pose.orientation.y << ", z: " << object.second.object.pose.orientation.z << ", w: " << object.second.object.pose.orientation.w);
	ROS_WARN_STREAM_NAMED(LOGNAME, "-------------------------------");
}

void printObject(std::pair<std::string, moveit_msgs::CollisionObject> object) {
	ROS_WARN_STREAM_NAMED(LOGNAME, "-------------------------------");
	ROS_WARN_STREAM_NAMED(LOGNAME, "id: " << object.first << " = " << object.second.id);
	//ROS_WARN_STREAM_NAMED(LOGNAME, "link_name: " << object.second.link_name);
	ROS_WARN_STREAM_NAMED(LOGNAME, "frame_id: " << object.second.header.frame_id);
	ROS_WARN_STREAM_NAMED(LOGNAME, "position relative to the header frame x: " << object.second.pose.position.x << ", y: " << object.second.pose.position.y << ", z: " << object.second.pose.position.z);
	ROS_WARN_STREAM_NAMED(LOGNAME, "orientation relative to the header frame x: " << object.second.pose.orientation.x << ", y: " << object.second.pose.orientation.y << ", z: " << object.second.pose.orientation.z << ", w: " << object.second.pose.orientation.w);
	ROS_WARN_STREAM_NAMED(LOGNAME, "-------------------------------");
}

bool placeAssembledT(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	ros::NodeHandle nh, pnh("~");
	ROS_ERROR_STREAM_NAMED(LOGNAME, "place assembled T");

	//moveit_task_constructor_demo::setupDemoScene(pnh);
	if (std::get<0>(picked_objects) == "") {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "World model has no picked objects");
		ROS_ERROR_STREAM_NAMED(LOGNAME, "picked object: " << std::get<0>(picked_objects));
		for (auto &&j : std::get<1>(picked_objects))
		{
			ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
		}
		for (auto &&i : pick_objects)
		{
			ROS_ERROR_STREAM_NAMED(LOGNAME, "pick object name: " << i.first);
			for (auto &&j : i.second)
			{
				ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
			}
			

		}
		res.success = false;
		res.message = "World doesn't contain an assembled T-group to pick";
		return true;
	}

	// Construct and run pick/place task
	moveit_task_constructor_demo::PlaceTask place_task("place_task", pnh);
	if (!place_task.init(picked_objects, t_assembled_place)) {
		ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Initialization failed");
		//return false;
		res.success = false;
		res.message = "PickPlaceTask Initialization failed";
		return true;
	}

	//bool gripper_open = open_gripper();
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	if (place_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "PlaceTask Planning succeded");
		if (pnh.param("execute", false)) {
			//if (open_gripper()) {
			if (place_task.execute()) {
				ROS_INFO_NAMED(LOGNAME, "PlaceTask Execution complete");
				if (open_gripper()) {
					ROS_INFO_NAMED(LOGNAME, "opened Gripper");

					std::vector<std::string> picked_object_ids;
					std::map<std::string, moveit_msgs::AttachedCollisionObject> objects = planning_scene_interface.getAttachedObjects();
					ROS_INFO_STREAM_NAMED(LOGNAME, "got all known attached objects: " << objects.size());
					for (std::pair<std::string, moveit_msgs::AttachedCollisionObject> object : objects) {
						printAttachedObject(object);
						picked_object_ids.push_back(object.first);
					}

					// res.success = true;
					// res.message = "LiftTask Initialization preemted";
					// return true;
					moveit_task_constructor_demo::LiftTask lift_task("lift_task", pnh);
					if (!lift_task.init(picked_objects, "T_assembled", false)) {
						ROS_INFO_NAMED(LOGNAME, "LiftTask Initialization failed");
						res.success = false;
						res.message = "LiftTask Initialization failed";
						return true;
					}
					if (lift_task.plan()) {
						ROS_INFO_NAMED(LOGNAME, "LiftTask Planning succeded");
						// ros::waitForShutdown();
						lift_task.execute();
						ROS_INFO_NAMED(LOGNAME, "LiftTask Execution complete");

						std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface.getObjects(picked_object_ids);
						ROS_INFO_STREAM_NAMED(LOGNAME, "got all known attached objects: " << objects.size());
						for (std::pair<std::string, moveit_msgs::CollisionObject> object : objects) {
							printObject(object);
						}
						// ----------------
						// if (!grasp_successful()) {
						// 	ROS_INFO_NAMED(LOGNAME, "Grasp failed gripper closed to little or to much.");
						// 	//return false;
						// 	t_assembled == 1 ? 2 : 1;
						// 	res.success = false;
						// 	res.message = "Grasp failed gripper closed to little or to much.";
						// 	return true;
						// }
					} else {
						ROS_INFO_NAMED(LOGNAME, "LiftTask Planning failed");
						//ros::waitForShutdown();
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
				ROS_INFO_NAMED(LOGNAME, "PlaceTask Execution failed");
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
	t_assembled_place = t_assembled_place == 0 ? 1 : 0;
	picked_objects = {};
	ROS_WARN_STREAM_NAMED(LOGNAME, "t_assembled_place = " << std::to_string(t_assembled_place));
	res.success = true;
	res.message = "Put down T group.";
	return true;
}

bool pickAssembledT(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	// TODO fix always picking from back
	ros::NodeHandle nh, pnh("~");
	ROS_ERROR_STREAM_NAMED(LOGNAME, "pick assembled T ");

	moveit::planning_interface::PlanningSceneInterface psi;
	detatch_objects(psi);
	clear_planning_scene(psi);
	if (pnh.param("spawn_table", true))
	   spawnObject(psi, createTable(pnh));
	if (pnh.param("spawn_magazines", true)) {
	   spawnObject(psi, createMagazine(pnh, "T"));
	   spawnObject(psi, createMagazine(pnh, "L"));
	   spawnObject(psi, createMagazineTAssembled(pnh));
	}
	for (size_t i = 1; i < 5; i++)
	{
		spawnObject(psi, createObject(pnh, "objectT" + std::to_string(i)));
	}

	for (size_t i = 1; i < 5; i++)
	{
		spawnObject(psi, createObject(pnh, "objectL" + std::to_string(i)));
	}

	for (int i = 0; i < 2; i++)
	{
		moveit_msgs::CollisionObject root = createAssembledMeshObject(pnh, "assembled_T", i);
		pick_objects[root.id] = {root.id};
		spawnObject(psi, root);
		pick_objects[root.id].push_back("assembled_pipe1" + std::to_string(pipe_count));
		spawnObject(psi, createAssembledObject(pnh, "assembled_pipe1", i));
		pick_objects[root.id].push_back("assembled_pipe2" + std::to_string(pipe_count));
		spawnObject(psi, createAssembledObject(pnh, "assembled_pipe2", i));
		pick_objects[root.id].push_back("assembled_pipe3" + std::to_string(pipe_count));
		spawnObject(psi, createAssembledObject(pnh, "assembled_pipe3", i));
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Created assembled T " << root.id << " with " << pick_objects[root.id].size() << " parts:");
		for (auto part : pick_objects[root.id]) {
			ROS_ERROR_STREAM_NAMED(LOGNAME, "- part: " << part);
		}
	}

	//moveit_task_constructor_demo::setupDemoScene(pnh);
	if (!(std::get<0>(picked_objects) == "")) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "World model already contains picked object");
	}
	ROS_ERROR_STREAM_NAMED(LOGNAME, "picked object: " << std::get<0>(picked_objects));
	for (auto &&j : std::get<1>(picked_objects))
	{
		ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
	}
	for (auto &&i : pick_objects)
	{
		ROS_ERROR_STREAM_NAMED(LOGNAME, "pick object name: " << i.first);
		for (auto &&j : i.second)
		{
			ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
		}
		

	}
	std::string object_name;
	for (const auto& po : pick_objects) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, po.first << " has value " << po.second.at(0) << " #parts: " << po.second.size());
		if ((po.first.find("assembled_T") != std::string::npos) && (po.second.size() > 1)) {
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
	if (!pick_place_task.init({object_name, {pick_objects[object_name]}})) {
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
				if (pick_place_task.execute()) {
					ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Execution complete");
					if (close_gripper()) {
						moveit_task_constructor_demo::LiftTask lift_task("lift_task", pnh);
						if (!lift_task.init({object_name, {pick_objects[object_name]}}, "Tgroup", true)) {
							ROS_INFO_NAMED(LOGNAME, "LiftTask Initialization failed");
							res.success = false;
							res.message = "LiftTask Initialization failed";
							return true;
						}
						if (lift_task.plan()) {
							ROS_INFO_NAMED(LOGNAME, "LiftTask Planning succeded");
							lift_task.execute();
							ROS_INFO_NAMED(LOGNAME, "LiftTask Execution complete");
							if (!grasp_successful()) {
								ROS_INFO_NAMED(LOGNAME, "Grasp failed gripper closed to little or to much.");
								//return false;
								t_assembled_pick = t_assembled_pick == 1 ? 2 : 1;
								res.success = false;
								res.message = "Grasp failed gripper closed to little or to much.";
								return true;
							}
						} else {
							ROS_INFO_NAMED(LOGNAME, "LiftTask Planning failed");
							//ros::waitForShutdown();
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
					ROS_INFO_NAMED(LOGNAME, "PickPlaceTask Execution failed");
					//ros::waitForShutdown();
					//return false;
					res.success = false;
					res.message = "Execution failed";
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
	t_assembled_pick = t_assembled_pick == 1 ? 2 : 1;
	picked_objects = {object_name, pick_objects[object_name]};
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
			if (hold_task.execute()) {
				ROS_INFO_NAMED(LOGNAME, "Execution complete");
				spawnPipe(pnh, "assembly_object");
			} else {
				ROS_INFO_NAMED(LOGNAME, "HoldTask Execution failed");
				//ros::waitForShutdown();
				//return false;
				res.success = false;
				res.message = "Execution failed";
				return true;
			}
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
			if (hold_task.execute()) {
				ROS_INFO_NAMED(LOGNAME, "Execution complete");
				spawnPipe(pnh, "assembly_object1");
			} else {
				ROS_INFO_NAMED(LOGNAME, "HoldTask Execution failed");
				//ros::waitForShutdown();
				//return false;
				res.success = false;
				res.message = "Execution failed";
				return true;
			}
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
			if (hold_task.execute()) {
				ROS_INFO_NAMED(LOGNAME, "Execution complete");
				spawnPipe(pnh, "assembly_object2");
			} else {
				ROS_INFO_NAMED(LOGNAME, "HoldTask Execution failed");
				//ros::waitForShutdown();
				//return false;
				res.success = false;
				res.message = "Execution failed";
				return true;
			}
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
	setupDemoScene(pnh);

	ROS_ERROR_STREAM_NAMED(LOGNAME, "after setupDemo ");
	ROS_ERROR_STREAM_NAMED(LOGNAME, "picked object: " << std::get<0>(picked_objects));
	for (auto &&j : std::get<1>(picked_objects))
	{
		ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
	}
	for (auto &&i : pick_objects)
	{
		ROS_ERROR_STREAM_NAMED(LOGNAME, "pick object name: " << i.first);
		for (auto &&j : i.second)
		{
			ROS_ERROR_STREAM_NAMED(LOGNAME, "object part: " << j);
		}
		

	}

	ros::ServiceServer pick_back_service = nh.advertiseService("franko_mtc_pick_back", pick);
	ROS_INFO("Franko: ready to pick T.");

	ros::ServiceServer pick_front_service = nh.advertiseService("franko_mtc_pick_front", pickL);
	ROS_INFO("Franko: ready to pick L.");

	ros::ServiceServer pick_T_assembled_service = nh.advertiseService("franko_mtc_place_T", placeAssembledT);
	ROS_INFO("Franko: ready to place T-group.");

	ros::ServiceServer place_T_assembled_service = nh.advertiseService("franko_mtc_pick_T", pickAssembledT);
	ROS_INFO("Franko: ready to pick T-group.");

	ros::ServiceServer hold_service = nh.advertiseService("franko_hold_bottom", hold);
	ROS_INFO("Franko: ready to hold.");

	ros::ServiceServer hold1_service = nh.advertiseService("franko_hold_front", hold1);
	ROS_INFO("Franko: ready to hold. Again");

	ros::ServiceServer hold2_service = nh.advertiseService("franko_hold_back", hold2);
	ROS_INFO("Franko: ready to hold. Again");

	ros::ServiceServer reset_service = nh.advertiseService("franko_reset_world", resetWorld);
	ROS_INFO("Franko: ready to reset");

	ros::ServiceServer configure_world_service = nh.advertiseService("franko_configure_world", configureWorld);
	ROS_INFO("Franko: ready to configure my world");

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
