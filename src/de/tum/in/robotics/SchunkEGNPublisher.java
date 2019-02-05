/**  
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
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

package de.tum.in.robotics;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

import de.tum.in.camp.kuka.ros.Configuration;
import de.tum.in.camp.kuka.ros.MessageGenerator;

/**
 * This class implements a ROS Node that publishes the current state of the robot tool.
 */
public class SchunkEGNPublisher extends AbstractNodeMain {

	// ROSJava Publishers for iiwa_msgs
	// Joint Message Publishers
	private Publisher<sensor_msgs.JointState> jointStatePublisher;
	
	// Object to easily build iiwa_msgs from the current robot state
	private MessageGenerator helper;
	
	private ConnectedNode node = null;
	private String robotName;

	// Cache objects
	private sensor_msgs.JointState js;

	/**
	 * Create a ROS node with publishers for a robot state. <br>
	 * Node will be running when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.<br>
	 * 
	 * @param robotName : name of the robot, topics will be created accordingly : <robot name>/state/<iiwa_msgs type> (e.g. MyIIWA/state/CartesianPosition)
	 */
	public SchunkEGNPublisher(String robotName, Configuration configuration) {
		this.robotName = robotName;
		helper = new MessageGenerator(robotName, configuration);

		js = helper.buildMessage(sensor_msgs.JointState._TYPE);
		js.getName().add("schunk_egn100_rail_1");
		js.getName().add("schunk_egn100_rail_2");
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

		if (jointStatePublisher.getNumberOfSubscribers() > 0) {
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
