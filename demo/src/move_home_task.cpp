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

	// Predefined pose targets
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_home_pose", arm_home_pose_);

	// Pick object
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_link", surface_link_);

	// Pick/Place metrics
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_min_dist", lift_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_max_dist", lift_object_max_dist_);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

bool MoveHomeTask::init(std::string object_name, int object_id, bool pick) {
	ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
	std::string object = "";
	std::vector<std::string> objects;
	ROS_INFO_STREAM_NAMED(LOGNAME, "before assigning object or objects ");
	ros::Duration(1).sleep();
	if (pick) {
		object = object_name + std::to_string(object_id);
		if (0 < object_id && object_id <= 3) {
			surface_link_ = surface_link_ + "1";
		} else if (3 < object_id && object_id <= 6) {
			surface_link_ = surface_link_ + "2";
		}
	} else {
		ROS_INFO_STREAM_NAMED(LOGNAME, "get all known attached objects for placing: ");
		moveit::planning_interface::PlanningSceneInterface psi;
		ROS_INFO_STREAM_NAMED(LOGNAME, "got psi ");
		std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects = psi.getAttachedObjects();
		ROS_INFO_STREAM_NAMED(LOGNAME, "got attached objects ");
		if (attached_objects.size() > 0) {
			ROS_INFO_STREAM_NAMED(LOGNAME, "got all known attached objects: " << attached_objects.size());
			for (std::pair<std::string, moveit_msgs::AttachedCollisionObject> object : attached_objects) {
				objects.push_back(object.first);
			}
		} else {
			ROS_INFO_STREAM_NAMED(LOGNAME, "got all known attached objects: EMPTY! ");
		}
	}
	ROS_INFO_NAMED(LOGNAME, "Got support_surface %s", surface_link_.c_str());
	support_surfaces_ = { surface_link_ };
	
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
	if (pick) {
		auto current_state = std::make_unique<stages::CurrentState>("current state for picking");

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
	} else {
		auto current_state = std::make_unique<stages::CurrentState>("current state for placing");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
		applicability_filter->setPredicate([objects](const SolutionBase& s, std::string& comment) {
			// if (objects.size() == 0) {
			//     ROS_ERROR_STREAM_NAMED(LOGNAME, objects.size() << " objects attached and cannot be placed");
			// 	comment = objects.size() + " objects attached and cannot be placed";
			// 	return false;
			// }
			ROS_ERROR_STREAM_NAMED(LOGNAME, objects.size() << " objects attached");
			return true;
		});
		//initial_state_ptr = current_state.get();  // remember start state for monitoring grasp pose generator
		t.add(std::move(applicability_filter));
	}

	ROS_ERROR_STREAM_NAMED(LOGNAME, "object = '" << object << "'");
	if (object != "" || objects.size() > 0) {
		/****************************************************
		*               Attach / Detache Object                      *
		***************************************************/
		{
			if (pick) {
				auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object ");
				stage->attachObject(object, hand_frame_);
				t.add(std::move(stage));
			} else {
				auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
				// for (std::string object : objects) {
				// 	stage->detachObject(object, hand_frame_);
				// }
				stage->detachObject(objects[0], hand_frame_);
				t.add(std::move(stage));
			}
		}

		/****************************************************
		*               Allow collision (object support)   *
		***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			if (pick) {
				ROS_ERROR_STREAM_NAMED(LOGNAME, "object with id '" << object << "' and support '" << support_surfaces_[0].c_str() << "'");
				stage->allowCollisions({ object }, support_surfaces_, true);
			} else {
				ROS_ERROR_STREAM_NAMED(LOGNAME, "objects first with id '" << objects.at(0).c_str() << "' and support '" << support_surfaces_[0].c_str() << "'");
				stage->allowCollisions(objects, support_surfaces_, true);
			}
			t.add(std::move(stage));
		}
	}

	/****************************************************
	*               Lift object / hand                        *
	***************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("lift hand", cartesian_planner);
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
	if (object != "") {
		auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,support)");
		stage->allowCollisions({ object }, support_surfaces_, false);
		t.add(std::move(stage));
	} else if (objects.size() > 0) {
		auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (objects,support)");
		stage->allowCollisions(objects, support_surfaces_, false);	
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
