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

package de.tum.in.robotics.schunk.egn100;

import iiwa_tool_msgs.MoveGripperActionGoal;

import java.net.URI;

import javax.annotation.PostConstruct;
import javax.annotation.PreDestroy;
import javax.inject.Inject;

import org.ros.address.BindAddress;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.SchunkGripperIOGroup;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.ExecutionService;
import com.kuka.roboticsAPI.controllerModel.RequestService;

import de.tum.in.camp.kuka.ros.ActiveTool;
import de.tum.in.camp.kuka.ros.AddressGeneration;
import de.tum.in.camp.kuka.ros.Configuration;
import de.tum.in.camp.kuka.ros.Logger;
import de.tum.in.camp.kuka.ros.iiwaActionServer;
import de.tum.in.camp.kuka.ros.iiwaPublisher;
import de.tum.in.robotics.SchunkEGNActionServer.Goal;

public class SchunkEGN100 implements ActiveTool {
	
	public static final int CW_COMMAND_ENABLE_A_BIT           = Integer.parseInt("0000000000000001", 2);
	public static final int CW_COMMAND_ENABLE_B_BIT           = Integer.parseInt("0000000000000010", 2);
	public static final int CW_ACCEPT_JERK_BIT                = Integer.parseInt("0000000000000100", 2);
	public static final int CW_ACCEPT_ACCELERATION_BIT        = Integer.parseInt("0000000000001000", 2);
	public static final int CW_ACCEPT_CURRENT_BIT             = Integer.parseInt("0000000000010000", 2);
	public static final int CW_ACCEPT_SPEED_BIT               = Integer.parseInt("0000000000100000", 2);
	public static final int CW_APPROACH_POSITION_BIT          = Integer.parseInt("0000000001000000", 2);
	public static final int CW_APPROACH_RELATIVE_POSITION_BIT = Integer.parseInt("0000000010000000", 2);
	public static final int CW_REFERENCING_BIT                = Integer.parseInt("0000001000000000", 2);
	public static final int CW_RESTART_MODULE_BIT             = Integer.parseInt("0001000000000000", 2);
	public static final int CW_ACKNOWLEDGE_ERROR_BIT          = Integer.parseInt("0010000000000000", 2);
	public static final int CW_STOP_BIT                       = Integer.parseInt("0100000000000000", 2);
	public static final int CW_FAST_STOP_BIT                  = Integer.parseInt("1000000000000000", 2);
	
	public static final int SW_COMMAND_RELEASE_A_BIT          = Integer.parseInt("0000000000000001", 2);
	public static final int SW_COMMAND_RELEASE_B_BIT          = Integer.parseInt("0000000000000010", 2);
	public static final int SW_MODULE_READY_BIT               = Integer.parseInt("0000000001000000", 2);
	public static final int SW_MODULE_MOVING_BIT              = Integer.parseInt("0000000100000000", 2);
	public static final int SW_TARGET_POSITION_REACHED_BIT    = Integer.parseInt("0000001000000000", 2);
	public static final int SW_MOTION_BLOCKED_BIT             = Integer.parseInt("0000010000000000", 2);
	public static final int SW_BRAKE_ENGAGED_BIT              = Integer.parseInt("0000100000000000", 2);
	public static final int SW_MODULE_REFERENCED_BIT          = Integer.parseInt("0001000000000000", 2);
	public static final int SW_INFO_BIT                       = Integer.parseInt("0010000000000000", 2);
	public static final int SW_WARNING_BIT                    = Integer.parseInt("0100000000000000", 2);
	public static final int SW_ERROR_BIT                      = Integer.parseInt("1000000000000000", 2);

	private class SendCommandThread extends Thread {
		boolean running = true;
		boolean sendReset = false;
		boolean sendMotion = false;
		private MoveGripperActionGoal goal;
		
		private int sendTimeout = 20;
		
		public void run() {
			while(running) {
				ThreadUtil.milliSleep(sendTimeout);
				
				if (sendReset) {
					resetControlWord();
					sendReset = false;
				}
				
				if(sendMotion) {
					moveToAbsolutePosition();
					sendMotion = false;
				}
			}
		}
		
		public synchronized void sendReset() {
			sendReset = true;
		}
		
		public synchronized void sendMotion(MoveGripperActionGoal goal) {
			this.goal = goal;
			sendMotion = true;
		}
		
		public synchronized void stopCommunictation() {
			running = false;
		}
		
		public synchronized MoveGripperActionGoal getMotionGoal() {
			return goal;
		}
		
		@PostConstruct
		protected void resetControlWord() {
			boolean send = false;
			
			while(!send) {
				ioGroup.setControlWord(0);
				waitOneTick();
				Integer statusWord = ioGroup.getStatusWord();
				//System.out.println("statusWord before reset "+Integer.toBinaryString(statusWord));
				
				if (!isStatusWordBitSet(statusWord, SW_COMMAND_RELEASE_A_BIT) && !isStatusWordBitSet(statusWord, SW_COMMAND_RELEASE_B_BIT)) {
					send = true;
				}
				else {
					ThreadUtil.milliSleep(sendTimeout);
				}	
			}

			send = false;
			while(!send) {
				ioGroup.setControlWord(CW_FAST_STOP_BIT | CW_STOP_BIT | CW_ACKNOWLEDGE_ERROR_BIT | CW_COMMAND_ENABLE_B_BIT);
				Integer statusWord = ioGroup.getStatusWord();
				waitOneTick();
				//System.out.println("statusWord before reset "+Integer.toBinaryString(statusWord));
				if (isStatusWordBitSet(statusWord, SW_MODULE_READY_BIT) && isStatusWordBitSet(statusWord, SW_COMMAND_RELEASE_B_BIT)) {
					send = true;
				}
				else {
					ThreadUtil.milliSleep(sendTimeout);
				}	
			}
			
			//System.out.println("statusWord after reset "+Integer.toBinaryString(statusWord));
			//ioGroup.setControlWord(getBasicControlWord());
		}
		
		/**
		 * @param position : Target Position in Millimeters
		 * @param speed :    Movement speed in Millimeters per Second
		 */
		protected void moveToAbsolutePosition() {
			float position = (float)goal.getGoal().getPosition();
			float speed = (float)goal.getGoal().getVelocity();
			if (isMoving()) {
				sendMotionFailedSignal();
			}

			float actualPosition = convertSchunkToJava(ioGroup.getActualPossition().intValue());
			if (actualPosition - 0.01 < position && position < actualPosition + 0.01) {
				// current position and target position are identical
				sendMotionFinishedSignal();
				return;
			}
			
			resetControlWord();
			System.out.println("Gripper reset complete.");
			Integer lastStatusWord = ioGroup.getStatusWord();
			
			boolean motionStarted = false;
			while(!motionStarted) {
				Integer controlWord = (CW_COMMAND_ENABLE_A_BIT | CW_APPROACH_POSITION_BIT | CW_STOP_BIT | CW_FAST_STOP_BIT);
				ioGroup.setDesiredPossintion((long) convertJavaToSchunk(position));
		
				if (speed > 0.01) {
					controlWord |= CW_ACCEPT_SPEED_BIT;
					ioGroup.setDesiredSpeed((long) convertJavaToSchunk(speed));
				}
				
				ioGroup.setControlWord(controlWord);
				waitOneTick();
				Integer statusWord = ioGroup.getStatusWord();
				
				//System.out.println("statusWord before motion start "+Integer.toBinaryString(statusWord));
				if (isStatusWordBitSet(statusWord, SW_MODULE_MOVING_BIT)) {
					System.out.println("Gripper motion started.");
					motionStarted = true;
				}
				else if (!isStatusWordBitSet(statusWord, SW_MODULE_READY_BIT) || isStatusWordBitSet(statusWord, SW_COMMAND_RELEASE_A_BIT)) {
					resetControlWord();
					System.out.println("Gripper reset complete.");	
				}
				else {
					ThreadUtil.milliSleep(sendTimeout);
					
					if (statusWord.equals(lastStatusWord)) {
						System.out.println("Status word: "+Integer.toBinaryString(statusWord));
						lastStatusWord = statusWord;
					}
				}
			}
			
			moving = true;
			//System.out.println("statusWord after motion start "+Integer.toBinaryString(statusWord));
		}

		
		public void waitOneTick() {
			ThreadUtil.milliSleep(50);
		}
	}
	
	private SendCommandThread communicationThread;
	
	@Inject private SchunkGripperIOGroup ioGroup;
	
	private SchunkEGNPublisher publisher;
	protected NodeConfiguration nodeConfPublisher;
	
	private SchunkEGNActionServer actionServer;
	protected NodeConfiguration nodeConfActionServer;
		
	private boolean moving = false;
	
	/**
	 * Constructor
	 */
	public SchunkEGN100() {
		
	}
	
	@Override
	public void initialize(Configuration configuration, NodeMainExecutor mainExecutor) {
		startCommunicationThread();

		publisher = new SchunkEGNPublisher(configuration.getRobotName(), configuration);
		actionServer = new SchunkEGNActionServer(configuration.getRobotName(), configuration);
		
		try {
			URI uri = new URI(configuration.getMasterURI());
			
			nodeConfActionServer = NodeConfiguration.newPublic(configuration.getRobotIp());
			nodeConfActionServer.setTimeProvider(configuration.getTimeProvider());
			nodeConfActionServer.setNodeName(configuration.getRobotName() + "/tool_action_server");
			nodeConfActionServer.setMasterUri(uri);	
			nodeConfActionServer.setTcpRosBindAddress(BindAddress.newPublic(AddressGeneration.getNewAddress()));
			nodeConfActionServer.setXmlRpcBindAddress(BindAddress.newPublic(AddressGeneration.getNewAddress()));

			nodeConfPublisher = NodeConfiguration.newPublic(configuration.getRobotIp());
			nodeConfPublisher.setTimeProvider(configuration.getTimeProvider());
			nodeConfPublisher.setNodeName(configuration.getRobotName() + "/tool_publisher");
			nodeConfPublisher.setMasterUri(uri);
			nodeConfPublisher.setTcpRosBindAddress(BindAddress.newPublic(AddressGeneration.getNewAddress()));
			nodeConfPublisher.setXmlRpcBindAddress(BindAddress.newPublic(AddressGeneration.getNewAddress()));
		}
		catch (Exception e) {
			Logger.info(e.toString());
			return;
		}
		
		mainExecutor.execute(actionServer, nodeConfActionServer);
		mainExecutor.execute(publisher, nodeConfPublisher);
	}

	@PostConstruct
	protected void startCommunicationThread() {
		communicationThread = new SendCommandThread();
		communicationThread.start();
	}
	
	public Integer getBasicControlWord() {
		return (CW_STOP_BIT | CW_FAST_STOP_BIT);
	}
	
	public boolean isMoving() {
		return moving;
	}
	
	/**
	 * Check if a particular bit is set in the status word
	 * @param statusWord
	 * @param statusWordBit
	 * @return
	 */
	public boolean isStatusWordBitSet(Integer statusWord, int statusWordBit) {
		return (statusWord & statusWordBit) == statusWordBit;
	}
	
	/**
	 * Method is supposed to be called periodically for every control loop iteration
	 */
	@Override
	public void moveTool() {
		if (actionServer.newGoalAvailable()) {
			//System.out.println("Received new gripper goal.");
			if (actionServer.hasCurrentGoal()) {
				actionServer.markCurrentGoalFailed("New goal received");
			}
			
			Goal<MoveGripperActionGoal> goal = (Goal<MoveGripperActionGoal>)actionServer.acceptNewGoal();
			communicationThread.sendMotion(goal.goal);
		}
	}
	
	public void sendMotionFinishedSignal() {
		if (actionServer.hasCurrentGoal()) {
			System.out.println("Sending goal reached signal.");
			actionServer.markCurrentGoalReached();
		}
	}

	public void sendMotionFailedSignal() {
		System.err.println("Gripper motion failed!");
		if (actionServer.hasCurrentGoal()) {
			System.out.println("Sending goal failed signal.");
			actionServer.markCurrentGoalFailed("Motion failed");
		}
	}

	public static Integer convertJavaToSchunk(float value) {
		return Integer.reverseBytes(Float.floatToIntBits(value));
	}

	public static float convertSchunkToJava(Integer value) {
		return Float.intBitsToFloat(Integer.reverseBytes(value));
	}

	@Override
	public void publishCurrentState() throws InterruptedException {
		float actualPosition = convertSchunkToJava(ioGroup.getActualPossition().intValue());
		float actualVelocity = convertSchunkToJava(ioGroup.getActualSpeed().intValue());
		publisher.publishJointState((double)actualPosition, (double)actualVelocity);

		actionServer.publishCurrentState();
		
		Integer statusWord = ioGroup.getStatusWord();
		
		if (isMoving()) {
			//System.out.println("Actual Position: "+actualPosition+" ("+Long.toHexString(ioGroup.getActualPossition())+")");
			//System.out.println("Status Word: "+Long.toBinaryString(statusWord)+", isMoving(): "+isMovinng());
			
			float targetPosition = (float)communicationThread.getMotionGoal().getGoal().getPosition();

			if (isStatusWordBitSet(statusWord, SW_TARGET_POSITION_REACHED_BIT) || (targetPosition - 0.1f < actualPosition && actualPosition < targetPosition + 0.1f)) {
				moving = false;
				sendMotionFinishedSignal();
			}
			else if (isStatusWordBitSet(statusWord, SW_MOTION_BLOCKED_BIT) || isStatusWordBitSet(statusWord, SW_ERROR_BIT)) {
				moving = false;
				sendMotionFailedSignal();
			}
			else if (!isStatusWordBitSet(statusWord, SW_MODULE_MOVING_BIT) || isStatusWordBitSet(statusWord, SW_BRAKE_ENGAGED_BIT)) {
				moving = false;
				sendMotionFailedSignal();
			}
		}
	}
	
	@PreDestroy
	public void cleanUp() throws Exception {
		communicationThread.stopCommunictation();
	}
}
