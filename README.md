# iiwa_stack_tools

This repository provides ROS wrappers for some tools that we have been using so far together with our iiwa.
For installation check out the following steps:

1.  Install [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack) on your robot.https://github.com/IFL-CAMP/iiwa_stack
2.  Clone this ropository into your catkin workspace
3.  Run `catkin_make`
4.  Use KUKA [WorkVisual](https://www.kuka.com/de-de/produkte-leistungen/robotersysteme/software/systemsoftware/kuka_systemsoftware/kuka_work-visual) to set up the correct IO configuration.
    Please note that there is usually only one specific version of WorkVisual that is compatible with a particular Sunrise release.
    For the Schunk EGN100 gripper we have been using KUKA Sunrise Assembly Toolbox, the Zimmer R800 gripper runs with KUKA Sunrise Gripper Toolbox.
5.  Go to `iiwa_stack_tools/iiwa_tools_ros_java/src` and find the right package for the tool you are using.
    The packages are organized by the following naming scheme: `de/tum/in/robotics/<MANUFACTURER>/<PRODUCT>`
6.  Copy the according package into the `src` folder of your iiwa_stack Sunrise project.
7.  Copy the libraries from `iiwa_stack_tools/iiwa_tools_ros_java/ROSJavaLib` to the `ROSJavaLib` folder of your iiwa_stack Sunrise project and add them to your build path.
8.  Open `/src/de/tum/in/camp/kuka/ros/app/ROSBaseApplication.java` and replace the line
    ```
    protected ROSTool rosTool = null;
    ```
    with your tool. E.g.:
    ```
	  @Inject protected SchunkEGN100 rosTool;
    ```
9.  Open the object template view (usually on the right side of Sunrise workbench) and look for your tool. Create a frame called `<TOOL_NAME>_link_ee` (e.g. `schunk_egn100_link_ee`)
10.  Synchronize your project with the robot controller.
11. Make sure you have `/iiwa/toolName` parameter configured in ROS:
    ```
         <param name="/iiwa/toolName" type="string" value="TOOL_NAME" />
    ```
12. If you want Sunrise to interpret and publish the CartesianPose of the iiwa in tool coordinates set the correct endpoint frame:
    ```
        <param name="/iiwa/endpointFrame" type="string" value="TOOL_NAME_link_ee" />
    ```
13https://github.com/IFL-CAMP/iiwa_stackhttps://github.com/IFL-CAMP/iiwa_stackhttps://github.com/IFL-CAMP/iiwa_stackhttps://github.com/IFL-CAMP/iiwa_stackhttps://github.com/IFL-CAMP/iiwa_stack. Start the ROSSmartServo application as usual.
