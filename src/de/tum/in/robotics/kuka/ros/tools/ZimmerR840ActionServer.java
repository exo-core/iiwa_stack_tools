/**  
 * Copyright (C) 2018 Arne Peters - arne.peters@tum.de
 * Technische Universität München
 * Chair for Robotics, Artificial Intelligence and Embedded Systems 
 * Fakultät für Informatik / I6, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://www6.in.tum.de
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

package de.tum.in.robotics.kuka.ros.tools;

import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import actionlib_msgs.GoalID;
import actionlib_msgs.GoalStatus;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;

import com.github.rosjava_actionlib.ActionServer;
import com.github.rosjava_actionlib.ActionServerListener;

import de.tum.in.camp.kuka.ros.Configuration;

import iiwa_tool_msgs.GripperState;
import iiwa_tool_msgs.MoveGripperActionFeedback;
import iiwa_tool_msgs.MoveGripperActionGoal;
import iiwa_tool_msgs.MoveGripperActionResult;

import org.ros.internal.message.Message;

public class ZimmerR840ActionServer extends AbstractNodeMain {
	public enum GoalType {
		MOVE_GRIPPER
	}
	
	public class Goal<T_ACTION_GOAL extends Message> {
		public Goal(GoalType goalType, T_ACTION_GOAL goal, String goalId) {
			this.goalType = goalType;
			this.goal = goal;
			this.goalId = goalId;
		}
		
		public GoalType goalType;
		public T_ACTION_GOAL goal;
		public String goalId;
	}
	
	public abstract class GripperActionServerListener<T_ACTION_GOAL extends Message> implements ActionServerListener<T_ACTION_GOAL> {
		private GoalType goalType;
		private ZimmerR840ActionServer server;
		
		public GripperActionServerListener(ZimmerR840ActionServer server, GoalType goalType) {
			this.server = server;
			this.goalType = goalType;
		}
		
		/**
		 * Gets called after a new has been received.
		 * We initially accept all goals kill the old ones afterwards.
		 */
		@Override
		public boolean acceptGoal(T_ACTION_GOAL arg0) {
			return true;
		}

		/**
		 * Goal got canceled by remote callback.
		 */
		@Override
		public void cancelReceived(GoalID arg0) {
			server.markCurrentGoalFailed("Goal execution canceled by client.");
		}

		/**
		 * Goal received callback. Adds the received goal to the goal queue
		 */
		@Override
		public void goalReceived(T_ACTION_GOAL goal) {
			server.goalQueue.add(new Goal<T_ACTION_GOAL>(goalType, goal, this.getGoalId(goal)));
		}
		
		public abstract String getGoalId(T_ACTION_GOAL goal);
	}
	
	private ActionServer<MoveGripperActionGoal, MoveGripperActionFeedback, MoveGripperActionResult> moveGripperServer = null;
	Queue<Goal<?>> goalQueue;
	Goal<?> currentGoal;
	
	private ConnectedNode node = null;
	
	// Name to use to build the name of the ROS topics
	private String iiwaName = "iiwa";
	
	public ZimmerR840ActionServer(String robotName, Configuration configuration) {
		iiwaName = robotName;
		goalQueue = new LinkedBlockingQueue<ZimmerR840ActionServer.Goal<?>>();
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		node = connectedNode;
		goalQueue.clear();

		moveGripperServer = new ActionServer<MoveGripperActionGoal, MoveGripperActionFeedback, MoveGripperActionResult>(node, iiwaName+"/action/move_gripper", MoveGripperActionGoal._TYPE, MoveGripperActionFeedback._TYPE, MoveGripperActionResult._TYPE);
		moveGripperServer.attachListener(new GripperActionServerListener<MoveGripperActionGoal>(this, GoalType.MOVE_GRIPPER) {
			@Override
			public String getGoalId(MoveGripperActionGoal goal) {
				return goal.getGoalId().getId();
			}
		});
	}
	
	/**
	 * @see org.ros.node.NodeMain#getDefaultNodeName()
	 */
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(iiwaName + "/tool/action_server");
	}

	/**
	 * Sets current goal to succeeded and publishes result message
	 */
	public void markCurrentGoalReached() {
		markCurrentGoal(true, "");
	}

	/**
	 * Sets current goal to aborted and publishes result message
	 */
	public void markCurrentGoalFailed(String error_msg) {
		markCurrentGoal(false, error_msg);
	}
	
	private synchronized void markCurrentGoal(boolean succeeded, String error_msg) {
		if (hasCurrentGoal()) {
			switch (currentGoal.goalType) {
				case MOVE_GRIPPER: {
					MoveGripperActionResult result = moveGripperServer.newResultMessage();
					result.getResult().getState().setState(GripperState.UNKNOWN);
					if (succeeded) {
						result.getStatus().setStatus(GoalStatus.SUCCEEDED);
						moveGripperServer.setSucceed(currentGoal.goalId);
					}
					else {
						result.getStatus().setStatus(GoalStatus.ABORTED);
						moveGripperServer.setAborted(currentGoal.goalId);
					}
					moveGripperServer.sendResult(result);
					moveGripperServer.setGoalStatus(result.getStatus(), currentGoal.goalId);
					break;
				}
				default:
					System.out.println("Unknown goal type: "+currentGoal.goalType);
					break;
			}
			
			currentGoal = null;
		}
	}
	
	/**
	 * True if a new goal has been received
	 * @return
	 */
	public synchronized boolean newGoalAvailable() {
		return !goalQueue.isEmpty();
	}
	
	/**
	 * True if a goal is active at the moment
	 * @return
	 */
	public synchronized boolean hasCurrentGoal() {
		return currentGoal != null;
	}
	
	public synchronized Goal<?> getCurrentGoal() {
		return currentGoal;
	}
	
	public Goal<?> getNextGoal() {
		return goalQueue.peek();
	}
	
	public synchronized Goal<?> acceptNewGoal() {
		currentGoal = goalQueue.poll();
		return currentGoal;
	}
	
	/**
	 * Send heartbeat to action clients
	 */
	public synchronized void publishCurrentState() {
		if (hasCurrentGoal()) {
			switch (currentGoal.goalType) {
			case MOVE_GRIPPER:
				moveGripperServer.sendStatusTick();
				break;
			default:
				break;
			}
		}
	}
}
