1# iiwa_stack_tools

This repository provides ROS wrappers for some tools that we have been using so far together with our iiwa.
For installation check out the following steps:

1.  Clone this ropository into your catkin workspace
2.  Run `catkin_make`
3.  Use KUKA [WorkVisual](https://www.kuka.com/de-de/produkte-leistungen/robotersysteme/software/systemsoftware/kuka_systemsoftware/kuka_work-visual) to set up the correct IO configuration.
    Please note that there is usually only one specific version of WorkVisual that is compatible with a particular Sunrise release.
    For the Schunk EGN100 gripper we have been using KUKA Sunrise Assembly Toolbox, the Zimmer R800 gripper runs with KUKA Sunrise Gripper Toolbox.
4.  Go to `iiwa_stack_tools/iiwa_tools_ros_java/src` and find the right package for the tool you are using.
    The packages are organized by the following naming scheme: `de/tum/in/robotics/<MANUFACTURER>/<PRODUCT>`
5.  Copy the according package into the `src` folder of your iiwa_stack Sunrise project.
6.  Copy the libraries from `iiwa_stack_tools/iiwa_tools_ros_java/ROSJavaLib` to the `ROSJavaLib` folder of your iiwa_stack Sunrise project and add them to your build path.
7.  Open `/src/de/tum/in/camp/kuka/ros/app/ROSBaseApplication.java` and replace the line
    ```
    protected ROSTool rosTool = null;
    ```
    with your tool. E.g.:
    ```
	  @Inject protected SchunkEGN100 rosTool;
    ```
8.  Open the object template view (usually on the right side of Sunrise workbench) and look for your tool. Create a frame called `<TOOL_NAME>_link_ee` (e.g. `schunk_egn100_link_ee`)
9.  Synchronize your project with the robot controller.
10. Make sure you have `/iiwa/toolName` parameter configured in ROS:
    ```
         <param name="/iiwa/toolName" type="string" value="TOOL_NAME" />
    ```
11. If you want Sunrise to interpret and publish the CartesianPose of the iiwa in tool coordinates set the correct endpoint frame:
    ```
        <param name="/iiwa/endpointFrame" type="string" value="TOOL_NAME_link_ee" />
    ```
12. Start the ROSSmartServo application as usual.
