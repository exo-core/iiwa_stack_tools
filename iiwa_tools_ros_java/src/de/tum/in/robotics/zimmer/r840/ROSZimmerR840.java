/**
 * Copyright (C) 2018 Arne Peters - arne.peters@tum.de
 * Technische Universit�t M�nchen
 * Chair for Robotics, Artificial Intelligence and Embedded Systems
 * Fakult�t f�r Informatik / I6, Boltzmannstra�e 3, 85748 Garching bei M�nchen, Germany
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

package de.tum.in.robotics.zimmer.r840;

import iiwa_tool_msgs.MoveGripperActionGoal;

import java.net.URI;

import javax.inject.Inject;

import org.ros.address.BindAddress;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import com.kuka.grippertoolbox.api.state.GripperState;
import com.kuka.grippertoolbox.gripper.zimmer.ZimmerR840;

import de.tum.in.camp.kuka.ros.ActiveTool;
import de.tum.in.camp.kuka.ros.Configuration;
import de.tum.in.camp.kuka.ros.Logger;
import de.tum.in.robotics.zimmer.r840.ZimmerR840ActionServer.Goal;

public class ROSZimmerR840 implements ActiveTool {
    @Inject
    private ZimmerR840 _gripper;
	
	private ZimmerR840Publisher publisher;
	protected NodeConfiguration nodeConfPublisher;
	
	private ZimmerR840ActionServer actionServer;
	protected NodeConfiguration nodeConfActionServer;
		
	Goal<MoveGripperActionGoal> _goal = null;
	
	/**
	 * Constructor
	 */
	public ROSZimmerR840() {
		
	}
	
	@Override
	public void initialize(Configuration configuration, NodeMainExecutor mainExecutor) {
		synchronized (_gripper) {
			_gripper.initialize();
		}

		publisher = new ZimmerR840Publisher(configuration);
		actionServer = new ZimmerR840ActionServer(configuration);
		
		try {
			URI uri = configuration.getMasterURI();
			
			nodeConfActionServer = NodeConfiguration.newPublic(configuration.getRobotIp());
			nodeConfActionServer.setTimeProvider(configuration.getTimeProvider());
			nodeConfActionServer.setNodeName(configuration.getRobotName() + "/tool_action_server");
			nodeConfActionServer.setMasterUri(uri);	
			nodeConfActionServer.setTcpRosBindAddress(BindAddress.newPublic(30008));
			nodeConfActionServer.setXmlRpcBindAddress(BindAddress.newPublic(30009));

			nodeConfPublisher = NodeConfiguration.newPublic(configuration.getRobotIp());
			nodeConfPublisher.setTimeProvider(configuration.getTimeProvider());
			nodeConfPublisher.setNodeName(configuration.getRobotName() + "/tool_publisher");
			nodeConfPublisher.setMasterUri(uri);
			nodeConfPublisher.setTcpRosBindAddress(BindAddress.newPublic(30010));
			nodeConfPublisher.setXmlRpcBindAddress(BindAddress.newPublic(30011));
		}
		catch (Exception e) {
			Logger.info(e.toString());
			return;
		}
		
		mainExecutor.execute(actionServer, nodeConfActionServer);
		mainExecutor.execute(publisher, nodeConfPublisher);
		
		synchronized (_gripper) {
			// create a default release mode
	        _gripper.setDefaultReleaseMode(ZimmerR840.createAbsoluteMode(0)
	                .setVelocity(30.0)
	                );
	
	        // create a default grip mode
	        _gripper.setDefaultGripMode(ZimmerR840.createForceMode()
	                .setMaxCurrent(1.0)
	                .setVelocity(30.0)
	                );
		}
	}
	
	/**
	 * Method is supposed to be called periodically for every control loop iteration
	 */
	@Override
	public void moveTool() {
		if (actionServer.newGoalAvailable()) {
			synchronized (_gripper) {
			//System.out.println("Received new gripper goal.");
			if (actionServer.hasCurrentGoal()) {
				actionServer.markCurrentGoalFailed(_gripper.getGripperState(), "New goal received");
			}
			
			Goal<MoveGripperActionGoal> goal = (Goal<MoveGripperActionGoal>)actionServer.acceptNewGoal();
			_gripper.gripAsync(ZimmerR840.createAbsoluteMode(goal.goal.getGoal().getPosition())
	                .setVelocity(goal.goal.getGoal().getVelocity())
	                );
			}
		}
	}

	@Override
	public void publishCurrentState() throws InterruptedException {
		synchronized (_gripper) {
			double actualPosition = _gripper.getCurrentJawPosition();
			publisher.publishJointState(actualPosition, 0);
	
			actionServer.publishCurrentState();
			
			if (isMoving()) {
				GripperState state = _gripper.getGripperState();
				double targetPosition = _goal.goal.getGoal().getPosition();
				
				if (targetPosition - 0.1 < actualPosition && actualPosition < targetPosition + 0.1 ) {
					actionServer.markCurrentGoalReached(state);
				}
				else if (targetPosition < actualPosition && state == GripperState.GRIPPED_ITEM) {
					actionServer.markCurrentGoalReached(state);
				}
			}	
		}
	}

	public boolean isMoving() {
		return _goal != null;
	}
	
}
