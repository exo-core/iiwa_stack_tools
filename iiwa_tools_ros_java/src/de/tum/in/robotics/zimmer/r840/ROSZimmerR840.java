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
import com.kuka.grippertoolbox.gripper.zimmer.AbsoluteMode;
import com.kuka.grippertoolbox.gripper.zimmer.ZimmerR840;

import de.tum.in.camp.kuka.ros.ActiveTool;
import de.tum.in.camp.kuka.ros.Configuration;
import de.tum.in.camp.kuka.ros.Logger;
import de.tum.in.robotics.zimmer.r840.ZimmerR840ActionServer.Goal;

public class ROSZimmerR840 implements ActiveTool {
  private class MotionGoal {
    public MotionGoal() {};
    
    public MotionGoal(Goal<MoveGripperActionGoal> goal) {
      this.position = goal.goal.getGoal().getPosition();
      this.velocity = goal.goal.getGoal().getVelocity();
    }
    
    public double position = 0.0;
    public double velocity = 0.0;
  }
  
  private final int GRIPPER_COMMUNICATION_TIMEOUT = 100; // ms
  private final boolean REFERENCE_ON_STARTUP = true;
  //when difference between target position and current position is smaller than this, a goal reached message is send
  private final double EPSILON = 0.7; // mm
  private final int GOAL_RESEND_TRIALS = 3;
  
  @Inject
  private ZimmerR840 _gripper;
	
	private ZimmerR840Publisher _publisher;
	protected NodeConfiguration _nodeConfPublisher;
	
	private ZimmerR840ActionServer _actionServer;
	protected NodeConfiguration _nodeConfActionServer;
		
	MotionGoal _goal = null;
	
	/**
	 * Constructor
	 */
	public ROSZimmerR840() {
		
	}
	
	@Override
	public void initialize(Configuration configuration, NodeMainExecutor mainExecutor) {
		_publisher = new ZimmerR840Publisher(configuration);
		_actionServer = new ZimmerR840ActionServer(configuration);
		
		try {
			URI uri = configuration.getMasterURI();
			
			_nodeConfActionServer = NodeConfiguration.newPublic(configuration.getRobotIp());
			_nodeConfActionServer.setTimeProvider(configuration.getTimeProvider());
			_nodeConfActionServer.setNodeName(configuration.getRobotName() + "/tool_action_server");
			_nodeConfActionServer.setMasterUri(uri);	
			_nodeConfActionServer.setTcpRosBindAddress(BindAddress.newPublic(30008));
			_nodeConfActionServer.setXmlRpcBindAddress(BindAddress.newPublic(30009));

			_nodeConfPublisher = NodeConfiguration.newPublic(configuration.getRobotIp());
			_nodeConfPublisher.setTimeProvider(configuration.getTimeProvider());
			_nodeConfPublisher.setNodeName(configuration.getRobotName() + "/tool_publisher");
			_nodeConfPublisher.setMasterUri(uri);
			_nodeConfPublisher.setTcpRosBindAddress(BindAddress.newPublic(30010));
			_nodeConfPublisher.setXmlRpcBindAddress(BindAddress.newPublic(30011));
		}
		catch (Exception e) {
			Logger.error(e.toString());
			return;
		}

    mainExecutor.execute(_actionServer, _nodeConfActionServer);
    mainExecutor.execute(_publisher, _nodeConfPublisher);
    
    synchronized(this) {
      try {
        _gripper.initialize();
        checkInitialization();
      
        // create a default release mode
        _gripper.setDefaultReleaseMode(ZimmerR840.createAbsoluteMode(0)
                .setVelocity(30.0)
                );
  
        // create a default grip mode
        _gripper.setDefaultGripMode(ZimmerR840.createForceMode()
                .setMaxCurrent(1.0)
                .setVelocity(30.0)
                );
      
        if (REFERENCE_ON_STARTUP) {
          referenceGripper();
        }
        else if (_gripper.getGripperState() == GripperState.UNDEFINED) {
          Logger.info("Gripper state is unknown. Referencing gripper...");
          referenceGripper();
        }
        else {
          doNullMotion();
        }
      }
      catch (Exception e) {
        Logger.error("Error during gripper initialization: "+e.getMessage());
      }
    }
    
    Logger.debug("Gripper initialization complete.");
	}
	
	/**
	 * Method is supposed to be called periodically for every control loop iteration
	 */
	@Override
	public void moveTool() {
		if (_actionServer.newGoalAvailable()) {
		  Goal<?> goal;
			synchronized(this) {
		    Logger.debug("Received new gripper goal.");
		    if (_actionServer.hasCurrentGoal()) {
		      _actionServer.markCurrentGoalFailed(_gripper.getGripperState(), "New goal received");
		      _goal = null;
		    }
		    
		    goal = _actionServer.acceptNewGoal();
			}
			
	    boolean success = false;
	    //for (int i=0; success || i<3; i++) {
	      success = excuteGoal(goal);
	    //}
	    if (!success) {
	      Logger.error("Gripper goal execution failed");
	      if (_actionServer.hasCurrentGoal()) {
  	      _actionServer.markCurrentGoalFailed(_gripper.getGripperState(), "Goal execution failed");
          _goal = null;
	      }
	    }
		}
	}
	
	private boolean excuteGoal(Goal<?> goal) {
	  try {
      switch (goal.goalType) {
        case MOVE_GRIPPER:
          _goal = new MotionGoal((Goal<MoveGripperActionGoal>)goal);
          move(_goal);
          break;
        case REFERENCE_GRIPPER:
          synchronized(this) {
            referenceGripper();
          }
          _actionServer.markCurrentGoalReached(_gripper.getGripperState());
          break;
        default:
          Logger.warn("Unknwon action type: "+goal.goalType);
          break;
      }
      
      return true;
    }
    catch (Exception e) {
      Logger.error("Error in gripper action: "+e.getStackTrace());
      return false;
    }
	}
	
	private void move(MotionGoal goal) {
    AbsoluteMode gripperTask;
    
    synchronized(this) {
      gripperTask = ZimmerR840.createAbsoluteMode(goal.position);
      if (goal.velocity > 0.0) {            
        gripperTask.setVelocity(goal.velocity);
      }

      checkInitialization();
      _gripper.stopMotion();
      _gripper.gripAsync(gripperTask);
    }
    
    for (int i=0; i<GOAL_RESEND_TRIALS && !checkForGoalReached(goal); i++) {
      waitUntil(GRIPPER_COMMUNICATION_TIMEOUT);
      
      synchronized(this) {
        //_gripper.stopMotion();
        //checkInitialization();
        _gripper.gripAsync(gripperTask);
      } 
    }
	}
	
	private void referenceGripper() throws InterruptedException {
	  Logger.debug("Referencing gripper...");
    checkInitialization();
    _gripper.reference();
    //waitUntil(GRIPPER_COMMUNICATION_TIMEOUT);
    checkInitialization();
    doNullMotion();
    Logger.debug("Referencing complete.");
	}
	
	/**
	 * For some reason the gripper does not execute the first couple of commands after initialization.
	 * This motion forces it to close a little and opening again, resulting in a system executing commands
	 * in a correct way.
	 * @throws InterruptedException
	 */
	private void doNullMotion() throws InterruptedException {
    MotionGoal goal = new MotionGoal();
    goal.position = 10.0;
    boolean goalReached = false;
    for (int i=0; i<10 && !goalReached; i++) {
      //Logger.debug("Closing...");
      try {
        move(goal);
        waitUntil(1000);
        goalReached = checkForGoalReached(goal);
      }
      catch (Exception e) {
        // TODO: handle exception
      }
    }

    goal = new MotionGoal();
    goal.position = 0.0;
    goalReached = false;
    for (int i=0; i<10 && !checkForGoalReached(goal); i++) {
      //Logger.debug("Opening...");
      try {
        move(goal);
        waitUntil(1000);
        goalReached = checkForGoalReached(goal);
      }
      catch (Exception e) {
        // TODO: handle exception
      }
    }
	}
	
	private void checkInitialization() {
	  while(!_gripper.isInitialized()) {
	    waitUntil(GRIPPER_COMMUNICATION_TIMEOUT);
      _gripper.initialize();
    }
	}

	@Override
	public void publishCurrentState() throws InterruptedException {
		synchronized(this) {
			double actualPosition = _gripper.getCurrentJawPosition();
			_publisher.publishJointState(actualPosition, 0);
	
			_actionServer.publishCurrentState();
			
			if (isMoving() && checkForGoalReached(_goal)) {
		    GripperState state = _gripper.getGripperState();
				_actionServer.markCurrentGoalReached(state);
				_goal = null;
			}
		}
	}
	
	private boolean checkForGoalReached(MotionGoal goal) {
	  double actualPosition = _gripper.getCurrentJawPosition();
    GripperState state = _gripper.getGripperState();
    double targetPosition = goal.position;
    
	  if (Math.abs(actualPosition - targetPosition) < EPSILON) {
      return true;
    }
    else if (targetPosition > actualPosition && state == GripperState.GRIPPED_ITEM) {
      return true;
    }
	  
	  return false;
	}

	public boolean isMoving() {
		return _goal != null;
	}

  /**
   * Sleeps until a specific number of Milliseconds have passed
   * @param ms
   */
  private void waitUntil(long ms) {
    long _before = System.currentTimeMillis();
    long _now = 0;
    
    do {
      _now = System.currentTimeMillis();
      try {
        Thread.sleep(1);
      }
      catch (InterruptedException e) {
        // ignore - we simply loop until the requested duration is over...
      }
    }
    while ((_now - _before) < ms);
  }
}
