# motion_ifc
This code provides a proto-type implemention of a mid-level controller, implemented using crtk_api.
This underlying code uses the command-pattern and binds each topic to a methods with a similar name. 
## State-Command Structure
Some things to consider while browsing this code:
1. The user commands the **motion_ifc** using rostopics. This is handled by **robot_cmd_ifc**. Hence user commands are 
called **Robot Commands**
2. To accomplish the user's command requests, the **motion_ifc** needs access to robot states, this is implemented in 
**robot_state_ifc**. The measured states of the robot are referred to as **Robot States**
3. **crkt_common** implements crtk_specific methods such as probing the crtk_api grammar from rostopics
4. **controllers** implements the three basic controllers **interp**, **servo** and **move** as documented in the 
crtk motion API.
5. All the **controllers** and **states** implement dictionaries with (crtk_api styled) string keys that point to 
the underlying function. This is then used to retrieve the method handles from the class
6. The **communication_ifc** is the base class that wraps the ros-subscriber methods and provides functionalities such 
as watch-dog and crtk grammar introspection. This class is then inherited by **CmdIfc**
and **StateIfc** which are the communication bridges for **RobotCmdIfc** and **RobotStateIfc** respectively.