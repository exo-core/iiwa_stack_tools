#include <iiwa_tool_sim/virtual_parallel_gripper.h>
#include "../include/iiwa_tool_sim/virtual_parallel_gripper.h"

#define EPSILON 0.001
#define RATE 30

iiwa_tool_sim::VirtualParallelGripper::VirtualParallelGripper()
: _moveGripperActionServer(_nh, "action/move_gripper", false) {

	ROS_INFO("[virtual_parallel_gripper] Starting node...");

	_moveGripperActionServer.registerGoalCallback(boost::bind(&iiwa_tool_sim::VirtualParallelGripper::moveGripperGoalCB, this));
	_jointStatePub = _nh.advertise<sensor_msgs::JointState>("joint_states", 1);

	_nh.param<std::string>("finger_1_joint", _jointName1, "zimmer_r840_rail_1");
	_nh.param<std::string>("finger_2_joint", _jointName2, "zimmer_r840_rail_2");
	_nh.param<std::string>("wrist_frame", _wristFrame, "zimmer_r840_base");

	_nh.param<double>("min_joint_position", _minJointPosition, 0.);
	_nh.param<double>("max_joint_position", _maxJointPosition, 0.04);
	_nh.param<double>("max_velocity", _maxVelocity, 0.08);

	_currentState.header.frame_id = _wristFrame;
	_currentState.header.seq = 0;
	_currentState.name.push_back(_jointName1);
	_currentState.name.push_back(_jointName2);
	_currentState.position.resize(2, 0.0);
	_currentState.velocity.resize(2, 0.0);
	_currentState.effort.resize(2, 0.0);

	_moveGripperActionServer.start();
	_lastPoseUpdate = ros::Time::now();
}

iiwa_tool_sim::VirtualParallelGripper::~VirtualParallelGripper() {
}

void iiwa_tool_sim::VirtualParallelGripper::spin() {
	ros::Rate rate(RATE);

	while (ros::ok()) {
		updateGripperPosition();
		if (_moveGripperActionServer.isActive() && currentGoalReached()) {
			iiwa_tool_msgs::MoveGripperResult result;
			result.state = getCurrentGripperState();
			_moveGripperActionServer.setSucceeded(result);
			ROS_DEBUG("[virtual_parallel_gripper] Gripper motion completed.");
		}

		ros::spinOnce();
	}
}

void iiwa_tool_sim::VirtualParallelGripper::moveGripperGoalCB() {
	ROS_DEBUG("[virtual_parallel_gripper] Move gripper action received");

	if (_moveGripperActionServer.isActive()) {
		iiwa_tool_msgs::MoveGripperResult result;
		result.state.state = iiwa_tool_msgs::GripperState::UNKNOWN;
		_moveGripperActionServer.setAborted(result, "New goal received.");
	}

	_target = *_moveGripperActionServer.acceptNewGoal();

	if (_target.position < _minJointPosition) {
		_target.position = _minJointPosition;
		ROS_WARN("[virtual_parallel_gripper] Target position is below minimum joint position.");
	}
	else if (_target.position > _maxJointPosition) {
		_target.position = _maxJointPosition;
		ROS_WARN("[virtual_parallel_gripper] Target position is above maximum joint position.");
	}
}

void iiwa_tool_sim::VirtualParallelGripper::updateGripperPosition() {
	ros::Time now = ros::Time::now();

	const double currentPos = _currentState.position[0];
	const double targetPose = _target.position ;

	double dist = getSimulatedMoveDistance(_lastPoseUpdate, now);

	if (targetPose < currentPos) {
		dist *= -1.0;
		if (currentPos + dist < targetPose) {
			dist = targetPose-currentPos;
		}
	}
	else if (_target.position > currentPos) {
		if (currentPos + dist > targetPose) {
			dist = targetPose - dist;
		}
	}
	else {
		dist = 0.0;
	}

	double velocity = (dist < EPSILON) ? 0.0 : dist/(now-_lastPoseUpdate).toSec() ;

	_currentState.header.stamp = now;
	_currentState.header.seq++;
	_currentState.position[0] = currentPos + dist;
	_currentState.position[1] = currentPos + dist;
	_currentState.velocity[0] = velocity;
	_currentState.velocity[1] = velocity;

	_jointStatePub.publish(_currentState);
	_lastPoseUpdate = now;
}

bool iiwa_tool_sim::VirtualParallelGripper::currentGoalReached() const {
	return std::fabs(_target.position - _currentState.position[0]) < EPSILON;
}

double iiwa_tool_sim::VirtualParallelGripper::getSimulatedMoveDistance(const ros::Time& last, const ros::Time& now) const {
	ros::Duration t = now-last;
	double velocity = (_target.velocity > EPSILON) ? std::fmin(_maxVelocity, _target.velocity) : _maxVelocity;
	double dist = t.toSec()*velocity;
	return dist;
}

iiwa_tool_msgs::GripperState iiwa_tool_sim::VirtualParallelGripper::getCurrentGripperState() const {
	iiwa_tool_msgs::GripperState state;
	state.state = iiwa_tool_msgs::GripperState::UNKNOWN;

	if (std::fabs(_currentState.position[0] - _minJointPosition) < EPSILON) {
		state.state = iiwa_tool_msgs::GripperState::CLOSED;
	}
	else if (std::fabs(_currentState.position[0] - _maxJointPosition) < EPSILON) {
		state.state = iiwa_tool_msgs::GripperState::OPEN;
	}

	return state;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "virtual_parallel_gripper");

	iiwa_tool_sim::VirtualParallelGripper node;
	node.spin();

	return 0;
}