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

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import de.tum.in.camp.kuka.ros.Configuration;
import de.tum.in.camp.kuka.ros.MessageGenerator;

/**
 * This class implements a ROS Node that publishes the current state of the robot. <br>
 * Messages will be send via topics in this format : <robot name>/state/<iiwa_msgs type> (e.g. MyIIWA/state/CartesianPosition)
 */
public class ZimmerR840Publisher extends AbstractNodeMain {

	// ROSJava Publishers for iiwa_msgs
	// Joint Message Publishers
	private Publisher<sensor_msgs.JointState> jointStatePublisher;
	
	// Object to easily build iiwa_msgs from the current robot state
	private MessageGenerator helper;
	
	private ConnectedNode node = null;
	private String robotName;

	// Cache objects
	private sensor_msgs.JointState js;
	
	// Configuration
	private boolean publishJointStates = true;

	/**
	 * Create a ROS node with publishers for a robot state. <br>
	 * Node will be running when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.<br>
	 * 
	 * @param robotName : name of the robot, topics will be created accordingly : <robot name>/state/<iiwa_msgs type> (e.g. MyIIWA/state/CartesianPosition)
	 */
	public ZimmerR840Publisher(Configuration configuration) {
		this.robotName = configuration.getRobotName();
		//this.publishJointStates = configuration.getPublishJointStates();
		helper = new MessageGenerator(robotName, configuration.getTimeProvider());

		js = helper.buildMessage(sensor_msgs.JointState._TYPE);
		js.getName().add("zimmer_r840_rail_1");
		js.getName().add("zimmer_r840_rail_2");
		js.setPosition(new double[js.getName().size()]);
		js.setVelocity(new double[js.getName().size()]);
		js.setEffort(new double[js.getName().size()]);
	}

	/**
	 * Returns the current name used to compose the ROS topics' names for the publishers. <p>
	 * e.g. returning "dummy" means that the topics' names will be "dummy/state/...". <br>
	 * The creation of the nodes is performed when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.
	 * @return the current name to use for ROS topics.
	 */
	public String getRobotName() {
		return robotName;
	}

	/**
	 * @see org.ros.node.NodeMain#getDefaultNodeName()
	 */
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(robotName + "/tool/publisher");
	}

	/**
	 * This method is called when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.<br>
	 * Do <b>NOT</b> manually call this. <p> 
	 * @see org.ros.node.AbstractNodeMain#onStart(org.ros.node.ConnectedNode)
	 */
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		node = connectedNode;

		jointStatePublisher = connectedNode.newPublisher(robotName + "/joint_states", sensor_msgs.JointState._TYPE);
	}

	/**
	 * Publishes to the respective topics all the iiwa_msgs with the values they are currently set to.<p>
	 * Only the nodes that currently have subscribers will publish the messages.<br>
	 * 
	 * @param robot : the state of this robot will be published
	 * @param motion : the dynamic of this motion will be published
	 * @param frame : the Cartesian information published will be relative to this frame
	 * @throws InterruptedException
	 */
	public void publishJointState(double position, double velocity) throws InterruptedException {
		if (publishJointStates && jointStatePublisher.getNumberOfSubscribers() > 0) {
			helper.incrementSeqNumber(js.getHeader());
			js.getHeader().setStamp(helper.getCurrentTime());
			
			for (int i=0; i<js.getName().size(); i++) {
				js.getPosition()[i] = position/1000.0;
				js.getVelocity()[i] = velocity/1000.0;
			}
			jointStatePublisher.publish(js);
		}
	}
}
