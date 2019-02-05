# iiwa_stack_tools

This repository provides ROS wrappers for some tools that we have been using so far together with our iiwa.
For installation check out the following steps:

1. Use KUKA WorkVisual to set up the correct IO configuration.
   For the Schunk EGN100 gripper we have been using KUKA Sunrise Assembly Toolbox, the Zimmer R800 gripper runs with KUKA Sunrise Gripper Toolbox.
2. Check out the correct branch for your tool.
3. Copy the files from this repository into the `src` folder of your iiwa_stack Sunrise project.
4. Open `/src/de/tum/in/camp/kuka/ros/app/ROSBaseApplication.java` and replace the line
   ```
   protected ROSTool rosTool = null;
   ```
   with your tool. E.g.:
   ```
	 @Inject protected SchunkEGN100 rosTool;
   ```
5. Open the object template view (usually on the right side of Sunrise workbench) and look for your tool. Create a frame called `<TOOL_NAME>_link_ee` (e.g. `schunk_egn100_link_ee`)
6. Synchronize your project with the robot controller. 
7. Start ROSSmartServo application as usual. Make sure you have `/iiwa/toolName` parameter configured in ROS:
   ```<param name="/iiwa/toolName" type="string" value="$(arg tool_name)" />```
