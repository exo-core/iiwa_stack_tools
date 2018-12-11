package de.tum.in.robotics.kuka.ros.tools;

import iiwa_tool_msgs.MoveGripperActionGoal;

import java.net.URI;

import javax.inject.Inject;

import org.ros.address.BindAddress;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import com.kuka.grippertoolbox.api.state.GripperState;
import com.kuka.grippertoolbox.gripper.zimmer.ZimmerR840;

import de.tum.in.camp.kuka.ros.AddressGeneration;
import de.tum.in.camp.kuka.ros.Configuration;
import de.tum.in.camp.kuka.ros.Logger;
import de.tum.in.camp.kuka.ros.ROSTool;
import de.tum.in.robotics.kuka.ros.tools.ZimmerR840ActionServer.Goal;

public class ROSZimmerR840 implements ROSTool {
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
        _gripper.initialize();

		publisher = new ZimmerR840Publisher(Configuration.getRobotName(), configuration);
		actionServer = new ZimmerR840ActionServer(Configuration.getRobotName(), configuration);
		
		try {
			URI uri = new URI(Configuration.getMasterURI());
			
			nodeConfActionServer = NodeConfiguration.newPublic(Configuration.getRobotIp());
			nodeConfActionServer.setTimeProvider(configuration.getTimeProvider());
			nodeConfActionServer.setNodeName(Configuration.getRobotName() + "/tool_action_server");
			nodeConfActionServer.setMasterUri(uri);	
			nodeConfActionServer.setTcpRosBindAddress(BindAddress.newPublic(AddressGeneration.getNewAddress()));
			nodeConfActionServer.setXmlRpcBindAddress(BindAddress.newPublic(AddressGeneration.getNewAddress()));

			nodeConfPublisher = NodeConfiguration.newPublic(Configuration.getRobotIp());
			nodeConfPublisher.setTimeProvider(configuration.getTimeProvider());
			nodeConfPublisher.setNodeName(Configuration.getRobotName() + "/tool_publisher");
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
	
	/**
	 * Method is supposed to be called periodically for every control loop iteration
	 */
	@Override
	public void moveTool() {
		if (actionServer.newGoalAvailable()) {
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

	@Override
	public void publishCurrentState() throws InterruptedException {
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

	public boolean isMoving() {
		return _goal != null;
	}
	
}
