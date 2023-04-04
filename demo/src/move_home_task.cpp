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

#include <moveit_task_constructor_demo/move_home_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometric_shapes/shape_operations.h>

namespace moveit_task_constructor_demo {

constexpr char LOGNAME[] = "moveit_task_constructor_demo";
constexpr char MoveHomeTask::LOGNAME[];

MoveHomeTask::MoveHomeTask(const std::string& task_name, const ros::NodeHandle& pnh)
  : pnh_(pnh), task_name_(task_name) {
	loadParameters();
}

void MoveHomeTask::loadParameters() {
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
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectT_name", objectT_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectL_name", objectL_name_);
	//errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object1_dimensions", object1_dimensions_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectI_name", objectI_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectT_file", objectT_file_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "objectL_file", objectL_file_);
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

bool MoveHomeTask::init() {
	ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
	const std::string objectT = objectT_name_;
	const std::string objectL = objectL_name_;
	const std::string objectI = objectI_name_;
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
		applicability_filter->setPredicate([objectT, objectL, objectI](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(objectT) || s.start()->scene()->getCurrentState().hasAttachedBody(objectL) || s.start()->scene()->getCurrentState().hasAttachedBody(objectI)) {
			    ROS_ERROR_STREAM_NAMED(LOGNAME, "object with id '" << objectT << "' and/or '"  << objectL << "' and/or '"  << objectI  << "' is already attached and cannot be picked");
				comment = "object with id '" + objectT + "' and/or '" + objectL + "' and/or '" + objectI + "' is already attached and cannot be picked";
				return false;
			}
			return true;
		});
		//initial_state_ptr = current_state.get();  // remember start state for monitoring grasp pose generator
		t.add(std::move(applicability_filter));
	}

	/****************************************************
 	*               Attach Object                      *
	***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("attach objectT");
		stage->attachObject(objectT, hand_frame_);
		t.add(std::move(stage));
	}

	/****************************************************
 	*               Allow collision (object support)   *
	***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (objectT,support)");
		stage->allowCollisions({ objectT }, support_surfaces_, true);
		t.add(std::move(stage));
	}

	/****************************************************
 	*               Lift object                        *
	***************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("lift objectT", cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
		stage->setIKFrame(hand_frame_);
		stage->properties().set("marker_ns", "lift_object");

		// Set upward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = world_frame_;
		//vec.vector.z = 1.0;
		vec.vector.z = 1.0;
		stage->setDirection(vec);
		t.add(std::move(stage));
	}

	/****************************************************
    *               Forbid collision (object support)  *
	***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (objectT,support)");
		stage->allowCollisions({ objectT }, support_surfaces_, false);
		t.add(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Home                              *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setGoal(arm_home_pose_);
		stage->restrictDirection(stages::MoveTo::FORWARD);
		t.add(std::move(stage));
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

bool MoveHomeTask::plan() {
	ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
	int max_solutions = pnh_.param<int>("max_solutions", 10);

	return static_cast<bool>(task_->plan(max_solutions));
}

bool MoveHomeTask::execute() {
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