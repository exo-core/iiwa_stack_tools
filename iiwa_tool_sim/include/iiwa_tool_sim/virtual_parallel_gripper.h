#ifndef VIRTUAL_PARALLEL_GRIPPER_H
#define VIRTUAL_PARALLEL_GRIPPER_H

#include <ros/ros.h>
#include <iiwa_tool_msgs/MoveGripperAction.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/time.h>

namespace iiwa_tool_sim {
	class VirtualParallelGripper {
		public:
			VirtualParallelGripper();
			virtual ~VirtualParallelGripper();

			void moveGripperGoalCB();

			void gripperStateCallback(const std_msgs::Float64& msg);
			void spin();

		protected:
			void updateGripperPosition();
			bool currentGoalReached() const;
			double getSimulatedMoveDistance(const ros::Time& last, const ros::Time& now) const;
			iiwa_tool_msgs::GripperState getCurrentGripperState() const;

			ros::NodeHandle _nh;

		private:
			ros::Publisher _jointStatePub;
			actionlib::SimpleActionServer<iiwa_tool_msgs::MoveGripperAction> _moveGripperActionServer;

			sensor_msgs::JointState _currentState;
			iiwa_tool_msgs::MoveGripperGoal _target;

			double _maxJointPosition;
			double _minJointPosition;
			double _maxVelocity;

			std::string _wristFrame;

			std::string _jointName1;
			std::string _jointName2;

			ros::Time _lastPoseUpdate;
	};
}

#endif