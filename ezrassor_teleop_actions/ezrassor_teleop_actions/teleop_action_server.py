#Note: left side of >>>> is ROS1 function. 
#Right side of >>>> is ROS2 equivalent function that I switched the code to
import imp
from rclpy.action import ActionServer  # replaces >>>> import actionlib
import rclpy  # replaces >>>> import rospy
import time
import math
from rclpy.node import Node
# from ezrassor_teleop_msgs.msg import TeleopAction
# from ezrassor_teleop_msgs.msg import TeleopGoal
# from ezrassor_teleop_msgs.msg import TeleopResult
# from ezrassor_teleop_msgs.msg import TeleopFeedback
from ezrassor_teleop_interfaces.action import Teleop
# from gazebo_msgs.msg import LinkStates
from ezrassor_teleop_interfaces.msg import LinkStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
#TODO: Fix below line to properly import the tf2 equivalent of tf.transformations. Or to just import tf
#from tf import transformations

ALL_STOP = Twist()
QUEUE_SIZE = 10


class TeleopActionServer:
    def __init__(self):
        rclpy.init()
        # Create a new action server node
        # replaces >>>> rospy.init_node("teleop_action_server")
        self.teleop_node = rclpy.create_node("teleop_action_server")
        self.teleop_goal = Teleop.Goal()
        # Initial assumption is that the server is not currently executing any operation
        self.executing_goal = False
        self._server = ActionServer(
            self.teleop_node,
            Teleop,
            'teleop',
            self.on_goal)
        print("Message From Line 37: ServerInitialized") #Andy: added for testing purposes
        # above line replaces >>>> {
        # ROS Actionlib Library
        # self._server = actionlib.SimpleActionServer(
        #     "teleop_action_server",
        #     TeleopAction,
        #     execute_cb=self.on_goal,
        #     auto_start=False,
        # )
        # }

        # Set up publishers
        # rospy.Publisher( >>>> self.teleop_node.create_publisher(
        self.wheel_instructions = self.teleop_node.create_publisher(
            # rospy.get_param(rospy.get_name() + >>>> self.teleop_node.get_parameter(self.teleop_node.get_name() +
            Twist,
            #removed the below two lines for testing purposes:
            #self.teleop_node.get_parameter(
            #    self.teleop_node.get_name() + "/wheel_instructions_topic"),
            #and replaced them with the below line:
            self.teleop_node.get_name() + "/wheel_instructions_topic",
            QUEUE_SIZE
        )
        # rospy.Publisher( >>>> self.teleop_node.create_publisher(
        self.front_arm_instructions = self.teleop_node.create_publisher(
            # rospy.get_param(rospy.get_name() + >>>> self.teleop_node.get_parameter(self.teleop_node.get_name() +
            Float32,
            #removed the below two lines for testing purposes:
            #self.teleop_node.get_parameter(
            #    self.teleop_node.get_name() + "/front_arm_instructions_topic"),
            #and replaced them with the below line:
            self.teleop_node.get_name() + "/front_arm_instructions_topic",
            QUEUE_SIZE
        )
        # rospy.Publisher( >>>> self.teleop_node.create_publisher(
        self.back_arm_instructions = self.teleop_node.create_publisher(
            # rospy.get_param(rospy.get_name() +
            # >>>>
            # self.teleop_node.get_parameter(self.teleop_node.get_name() +
            Float32,
            #removed the below two lines for testing purposes:
            #self.teleop_node.get_parameter(
            #    self.teleop_node.get_name() + "/back_arm_instructions_topic"),
            #and replaced them with the below line:
            self.teleop_node.get_name() + "/back_arm_instructions_topic",
            QUEUE_SIZE
        )
        # rospy.Publisher( >>>> self.teleop_node.create_publisher(
        self.front_drum_instructions = self.teleop_node.create_publisher(
            # rospy.get_param(rospy.get_name() +
            # >>>>
            # self.teleop_node.get_parameter(self.teleop_node.get_name() +
            Float32,
            #removed the below two lines for testing purposes:
            # self.teleop_node.get_parameter(
            #     self.teleop_node.get_name() + "/front_drum_instructions_topic"
            # ),
            #and replaced them with the below line:
            self.teleop_node.get_name() + "/front_drum_instructions_topic",
            QUEUE_SIZE
        )
        # rospy.Publisher( >>>> self.teleop_node.create_publisher(
        self.back_drum_instructions = self.teleop_node.create_publisher(
            # rospy.get_param(rospy.get_name() +
            # >>>>
            # self.teleop_node.get_parameter(self.teleop_node.get_name() +
            Float32,
            #removed the below two lines for testing purposes:
            # self.teleop_node.get_parameter(
            #     self.teleop_node.get_name() + "/back_drum_instructions_topic"),
            #and replaced them with the below line:
            self.teleop_node.get_name() + "/back_drum_instructions_topic",
            QUEUE_SIZE
        )

        # Make sure there are subscribers on the other end
        time.sleep(5)
        self.wheel_instructions.publish(ALL_STOP)
        self.front_arm_instructions.publish(0.0)
        self.back_arm_instructions.publish(0.0)
        self.front_drum_instructions.publish(0.0)
        self.back_drum_instructions.publish(0.0)

        # Subscribe to the link states so we can get the xy position from Gazebo
        # rospy.Subscriber( >>>> self.teleop_node.create_subscription(
        
        self.teleop_node.create_subscription(
            LinkStates, "/gazebo/link_states", self.gazebo_state_callback
        )

        # Init the timer counter
        self._counter = 0

        # Initialize positional/state data.
        self.x = 0
        self.y = 0
        self.heading = ""

        # Give a success message!
        # rospy.loginfo >>>> self.teleop_node.get_logger().info(
        self.teleop_node.get_logger().info(
            "EZRASSOR Teleop Action Server has been initialized")

    def spin(self):
        """Start the node"""
        self._server.start()  # TODO: Check back on this. Start might not be a method of _server
        # rospy.spin() >>>> rclpy.spin(self.teleop_node)
        rclpy.spin(self.teleop_node)

    def gazebo_state_callback(self, data):
        """Saves the Gazebo link state (position) data for use in the server.
        Adapted from the ai_objects.py implementation"""

        self.x = 0
        self.y = 0
        self.heading = ""

        # Identify the index containing the position link state
        index = 0
        # namespace = rospy.get_namespace() >>>> self.teleop_node.get_namespace()
        namespace = self.teleop_node.get_namespace()
        namespace = namespace[1:-1] + "::base_link"

        try:
            index = data.name.index(namespace)
        except Exception:
            # rospy.logdebug( >>>> self.teleop_node.get_logger().debug(
            self.teleop_node.get_logger().debug("Failed to get index. Skipping...")
            return

        # Extract the information
        self.x = data.pose[index].position.x
        self.y = data.pose[index].position.y
        heading = self.quaternion_to_yaw(data.pose[index]) * 180 / math.pi

        if heading > 0:
            self.heading = heading
        else:
            self.heading = 360 + heading

    def quaternion_to_yaw(self, pose):
        """Helper function since Gazebo returns the forward kinematics of the robot
        as a quaternion"""
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        #TODO: Refactor below lines once you understand the use of tf2 in euler transformations
        #euler = transformations.euler_from_quaternion(quaternion)
        return #euler[2]

    def execution_timer_callback(self, event):
        """If this callback is called, it is to stop execution"""
        self.executing_goal = False

    def on_goal(self, goal):
        """Define all of the scenarios for handling a new goal from an action client."""

        # Python2 way to enforce type checking
        # if not isinstance(goal, TeleopGoal):
        #     result = TeleopResult()
        #Above two lines replaced with below two lines:
        if not isinstance(goal, Teleop):
            result = Teleop()
            # rospy.logerr( >>>> self.teleop_node.get_logger().error(
            self.teleop_node.get_logger().error("Unknown goal received")
            self._server.set_aborted(result)  # Returns failure
            return

        self.executing_goal = True

        msg = Twist()

        operation = goal.operation
        duration = goal.duration
        #rospy.loginfo >>>> self.teleop_node.get_logger().info(
        self.teleop_node.get_logger().info(
            "new action: {0}, duration: {1}".format(operation).format(duration)
        )

        if operation == self.teleop_goal.MOVE_FORWARD_OPERATION:
            msg.linear.x = 1
            self.wheel_instructions.publish(msg)

        elif operation == self.teleop_goal.MOVE_BACKWARD_OPERATION:
            msg.linear.x = -1
            self.wheel_instructions.publish(msg)

        elif operation == self.teleop_goal.ROTATE_LEFT_OPERATION:
            msg.angular.z = 1
            self.wheel_instructions.publish(msg)

        elif operation == self.teleop_goal.ROTATE_RIGHT_OPERATION:
            msg.angular.z = -1
            self.wheel_instructions.publish(msg)

        elif operation == self.teleop_goal.RAISE_FRONT_ARM_OPERATION:
            self.front_arm_instructions.publish(1.0)

        elif operation == self.teleop_goal.LOWER_FRONT_ARM_OPERATION:
            self.front_arm_instructions.publish(-1.0)

        elif operation == self.teleop_goal.RAISE_BACK_ARM_OPERATION:
            self.back_arm_instructions.publish(1.0)

        elif operation == self.teleop_goal.LOWER_BACK_ARM_OPERATION:
            self.back_arm_instructions.publish(-1.0)

        elif operation == self.teleop_goal.DIG_FRONT_DRUM_OPERATION:
            self.front_drum_instructions.publish(1.0)

        elif operation == self.teleop_goal.DUMP_FRONT_DRUM_OPERATION:
            self.front_drum_instructions.publish(-1.0)

        elif operation == self.teleop_goal.DIG_BACK_DRUM_OPERATION:
            self.back_drum_instructions.publish(1.0)

        elif operation == self.teleop_goal.DUMP_BACK_DRUM_OPERATION:
            self.back_drum_instructions.publish(-1.0)

        else:
            # Otherwise, we must be stopping
            self.wheel_instructions.publish(ALL_STOP)
            self.front_arm_instructions.publish(0.0)
            self.back_arm_instructions.publish(0.0)
            self.front_drum_instructions.publish(0.0)
            self.back_drum_instructions.publish(0.0)

        # Time counter
        self._counter = 0

        # Start new timer for operation
        #rospy.Timer( >>>> self.teleop_node.create_timer(
        self.teleop_node.create_timer(
            duration,
            self.execution_timer_callback
            #oneshot=True,
        )
        t0 = time.time()

        # Feedback
        #rospy.is_shutdown() >>>> self.teleop_node.context().ok() #TODO: Fix this. Running context().ok() causes error
        while self.teleop_node.context().ok() and self.executing_goal:

            elapsed = time.time() - t0
            #rospy.loginfo >>>> self.teleop_node.get_logger().info(
            self.teleop_node.get_logger().info(
                "operation: {0}, duration: {1}, elapsed: {2}"
                .format(operation)
                .format(duration)
                .format(elapsed)
            )

            if self._server.is_preempt_requested():
                #rospy.loginfo >>>> self.teleop_node.get_logger().info(
                self.teleop_node.get_logger().info("Preempted")
                self.executing_goal = False
                self._server.set_preempted(result)
                return

            #feedback = TeleopFeedback()
            #above line replaced with below line:
            feedback = Teleop()
            feedback.x = self.x
            feedback.y = self.y
            feedback.heading = str("{} degrees".format(self.heading))
            try:
                self._server.publish_feedback(feedback)
            #except rospy.ROSException: TODO: Find Ros2 Equivalent for ROSException (if it exists)
            #above line replaced with below line:
            except:
                self._server.set_aborted(
                    None, text="Unable to publish feedback. Has ROS stopped?"
                )
                return

        try:
            # Stop the robot after every action
            self.wheel_instructions.publish(ALL_STOP)
            self.front_arm_instructions.publish(0.0)
            self.back_arm_instructions.publish(0.0)
            self.front_drum_instructions.publish(0.0)
            self.back_drum_instructions.publish(0.0)
        #except rospy.ROSException: TODO: Find Ros2 Equivalent for ROSException (if it exists)
        #above line replaced with below line:
        except:
            self._server.set_aborted(
                None, text="Unable to publish all stop. Has ROS stopped?"
            )
            return

        # Result
        #result = TeleopResult()
        #above line replaced with below line:
        result = Teleop() #TODO: Determine if this line will cause an error. IDK if it's correct after all
        result.x = self.x #TODO: Fix float assertion error here
        result.y = self.y #TODO: Fix float assertion error here

        # Return success result to the client
        #rospy.loginfo >>>> self.teleop_node.get_logger().info(
        self.teleop_node.get_logger().info("Success")
        self._server.set_succeeded(result)
