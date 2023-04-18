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

#include <moveit_task_constructor_demo/pick_place_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometric_shapes/shape_operations.h>

namespace moveit_task_constructor_demo {

constexpr char LOGNAME[] = "moveit_task_constructor_demo";
constexpr char PickPlaceTask::LOGNAME[];

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
	if (!psi.applyCollisionObject(object)) {
		throw std::runtime_error("Failed to spawn object: " + object.id);
	}
}

void spawnAttatchedObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::AttachedCollisionObject& object) {
	if (!psi.applyAttachedCollisionObject(object)) {
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

moveit_msgs::CollisionObject createMagazine(const ros::NodeHandle& pnh) {
	std::string magazine_name, table_reference_frame, magazine_file;
	//std::vector<double> magazine_dimensions;
	geometry_msgs::Pose pose;
	std::size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazine_name", magazine_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_reference_frame", table_reference_frame);
	//errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazine_dimensions", magazine_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazine_file", magazine_file);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "magazine_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

	moveit_msgs::CollisionObject object;
	object.id = magazine_name;
	object.header.frame_id = table_reference_frame;
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
	std::string assembly_object_name, assembly_object_reference_frame;
	std::vector<double> assembly_object_dimensions;
	geometry_msgs::Pose assembly_pose;
	std::string link_name;
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "assembly_object_name", assembly_object_name);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", assembly_object_reference_frame);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "assembly_object_dimensions", assembly_object_dimensions);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "assembly_object_pose", assembly_pose);
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
	if (pnh.param("spawn_magazine", true))
	   spawnObject(psi, createMagazine(pnh));
	spawnObject(psi, createObject(pnh, "objectT1"));
	spawnObject(psi, createObject(pnh, "objectT2"));
	spawnObject(psi, createObject(pnh, "objectT3"));
	spawnObject(psi, createObject(pnh, "objectT4"));
	spawnObject(psi, createObject(pnh, "objectL1"));
	spawnObject(psi, createObject(pnh, "objectL2"));
	spawnObject(psi, createObject(pnh, "objectL3"));
	spawnObject(psi, createObject(pnh, "objectL4"));
	//spawnObject(psi, createObject(pnh, "objectI"));
	//spawnObject(psi, createAssemblyObject(pnh));
}

void spawnPipe(ros::NodeHandle& pnh, const std::string& name) {
	ros::Duration(1.0).sleep();
	moveit::planning_interface::PlanningSceneInterface psi;
	spawnAttatchedObject(psi, createAssemblyObject(pnh, name));
}

PickPlaceTask::PickPlaceTask(const std::string& task_name, const ros::NodeHandle& pnh)
  : pnh_(pnh), task_name_(task_name) {
	loadParameters();
}

void PickPlaceTask::loadParameters() {
	/****************************************************
	 *                                                  *
	 *               Load Parameters                    *
	 *                                                  *
	 ***************************************************/
	ROS_INFO_NAMED(LOGNAME, "Loading task parameters");

	// Planning group properties
	size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_group_name", arm_group_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_group_name", hand_group_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "eef_name", eef_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_frame", hand_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "world_frame", world_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "grasp_frame_transform", grasp_frame_transform_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "assemble_frame_transform", assemble_frame_transform_);

	// Predefined pose targets
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_open_pose", hand_open_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_close_pose", hand_close_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_home_pose", arm_home_pose_);

	// Pick object
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectT1_name", objectT1_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectT2_name", objectT2_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectT3_name", objectT3_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectT4_name", objectT4_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectL1_name", objectL1_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectL2_name", objectL2_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectL3_name", objectL3_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectL4_name", objectL4_name_);
	//errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object1_dimensions", object1_dimensions_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectI_name", objectI_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectT1_file", objectT1_file_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectT2_file", objectT2_file_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectT3_file", objectT3_file_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectT4_file", objectT4_file_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectL1_file", objectL1_file_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectL2_file", objectL2_file_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectL3_file", objectL3_file_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectL4_file", objectL4_file_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectI_file", objectI_file_);
	//errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object2_dimensions", object2_dimensions_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_reference_frame", object_reference_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_link", surface_link_);
	support_surfaces_ = { surface_link_ };

	// Assembly object
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "assembly_object_name", assembly_object_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "assembly_object_dimensions", assembly_object_dimensions_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_link", surface_link_);
	support_surfaces_ = { surface_link_ };

	// Pick/Place metrics
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_min_dist", approach_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_max_dist", approach_object_max_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_min_dist", lift_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_max_dist", lift_object_max_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_surface_offset", place_surface_offset_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_pose1", place_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "assembly_object_pose", assembly_pose_);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

bool PickPlaceTask::init(std::string object_name) {
	ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
	const std::string object = object_name;
	// const std::string objectT = objectT_name_;
	// const std::string objectL = objectL_name_;
	// const std::string objectI = objectI_name_;
	const std::string assembly_object = assembly_object_name_;

	// Reset ROS introspection before constructing the new object
	// TODO(v4hn): global storage for Introspection services to enable one-liner
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());

	Task& t = *task_;
	t.stages()->setName(task_name_);
	t.loadRobotModel();

	// Sampling planner
	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	// Cartesian planner
	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScalingFactor(1.0);
	cartesian_planner->setMaxAccelerationScalingFactor(1.0);
	cartesian_planner->setStepSize(.01);

	// Set task properties
	t.setProperty("group", arm_group_name_);
	t.setProperty("eef", eef_name_);
	t.setProperty("hand", hand_group_name_);
	t.setProperty("hand_grasping_frame", hand_frame_);
	t.setProperty("ik_frame", hand_frame_);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	//Stage* initial_state_ptr = nullptr;
	{
		auto current_state = std::make_unique<stages::CurrentState>("current state");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
		applicability_filter->setPredicate([object](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
			    ROS_ERROR_STREAM_NAMED(LOGNAME, "object with id '" << object << "' is already attached and cannot be picked");
				comment = "object with id '" + object + "' is already attached and cannot be picked";
				return false;
			}
			return true;
		});
		//initial_state_ptr = current_state.get();  // remember start state for monitoring grasp pose generator
		t.add(std::move(applicability_filter));
	}

	// /****************************************************
	//  *                                                  *
	//  *               Open Hand                          *
	//  *                                                  *
	//  ***************************************************/
	// Stage* initial_state_ptr = nullptr;
	// {  // Open Hand
	// 	auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
	// 	stage->setGroup(hand_group_name_);
	// 	stage->setGoal(hand_open_pose_);
	// 	initial_state_ptr = stage.get();  // remember start state for monitoring grasp pose generator
	// 	t.add(std::move(stage));
	// }

	/****************************************************
	 *                                                  *
	 *               memorise                           *
	 *                                                  *
	 ***************************************************/
	Stage* initial_state_ptr = nullptr;
	{  // Open Hand
		auto stage = std::make_unique<stages::MoveRelative>("memorise", sampling_planner);
		stage->properties().set("marker_ns", "memorise");
		stage->properties().set("link", hand_frame_);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setMinMaxDistance(0.0, 0.03);

		// Set hand forward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = hand_frame_;
		vec.vector.z = -0.005;
		stage->setDirection(vec);
		initial_state_ptr = stage.get();  // remember start state for monitoring grasp pose generator
		t.add(std::move(stage));
	}


	/****************************************************
	 *                                                  *
	 *               Move to Pick                       *
	 *                                                  *
	 ***************************************************/
	{  // Move-to pre-grasp
		auto stage = std::make_unique<stages::Connect>(
		    "move to pick", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
		stage->setTimeout(10.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		//initial_state_ptr = stage.get();
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Pick Object                        *
	 *                                                  *
	 ***************************************************/
	Stage* pick_stage_ptr = nullptr;
	{
		auto grasp = std::make_unique<SerialContainer>("pick object");
		t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

		/****************************************************
  ---- *               Approach Object                    *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
			stage->properties().set("marker_ns", "approach_object");
			stage->properties().set("link", hand_frame_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(approach_object_min_dist_-0.02, approach_object_max_dist_);

			// Set hand forward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = hand_frame_;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Generate Grasp Pose                *
		 ***************************************************/
		{
			// Sample grasp pose
			auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->properties().set("marker_ns", "grasp_pose");
			stage->setPreGraspPose(hand_open_pose_);
			//stage->setObject(objectT);
			stage->setObject(object);
			//stage->setAngleDelta(M_PI / 6);
			stage->setAngleDelta(M_PI / 60);
			stage->setMonitoredStage(initial_state_ptr);  // hook into successful initial-phase solutions

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
			//wrapper->setIKFrame(grasp_frame_transform_, "franko_fr3_hand_tcp");
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			grasp->insert(std::move(wrapper));
		}
	//}

		/****************************************************
  ---- *               Allow Collision (hand object)   *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(
			    object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			grasp->insert(std::move(stage));
		}

		// Add grasp container to task
		t.add(std::move(grasp));
	}

	// prepare Task structure for planning
	try {
		t.init();
	} catch (InitStageException& e) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
		return false;
	}

	return true;
}

bool PickPlaceTask::plan() {
	ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
	int max_solutions = pnh_.param<int>("max_solutions", 10);

	return static_cast<bool>(task_->plan(max_solutions));
}

bool PickPlaceTask::execute() {
	ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
	moveit_msgs::MoveItErrorCodes execute_result;

	execute_result = task_->execute(*task_->solutions().front());
	// // If you want to inspect the goal message, use this instead:
	// actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
	// execute("execute_task_solution", true); execute.waitForServer();
	// moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
	// task_->solutions().front()->fillMessage(execute_goal.solution);
	// execute.sendGoalAndWait(execute_goal);
	// execute_result = execute.getResult()->error_code;

	if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_result.val);
		return false;
	}

	return true;
}
}  // namespace moveit_task_constructor_demo
