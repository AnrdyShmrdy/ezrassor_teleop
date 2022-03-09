from ezrassor_teleop_interfaces.action import Teleop
#from ezrassor_teleop_interfaces.action import TeleopAction #replaces --> from ezrassor_teleop_msgs.msg import TeleopAction
#from ezrassor_teleop_interfaces.action import TeleopGoal #replaces --> from ezrassor_teleop_msgs.msg import TeleopGoal

from rclpy.action import ActionClient #replaces --> import actionlib
import rclpy #replaces --> import rospy 
import time

#changes: any mention of rospy is replaced with rclpy
class TeleopActionClient:
    def __init__(self):
        print("test init") #Added by Andy for testing
        rclpy.init()
        self.teleop_node = rclpy.create_node("teleop_action_client") #replaces --> rospy.init_node("teleop_action_client")
        self._client = ActionClient(self.teleop_node, Teleop, 'teleop')
        #above line replaces --> {
        #ROS Actionlib Library
        #self._client = actionlib.SimpleActionClient(
        #    "teleop_action_server", TeleopAction
        #)
        #}
        self.teleop_goal = Teleop.Goal()

        # wait_for_server() occasionally fails instantly despite being given
        # a timeout. To prevent these failures we briefly sleep before waiting.
        # Hopefully this will be fixed in ROS 2.
        time.sleep(1)

        connected = self._client.wait_for_server(5)
        #above line replaces:
        #connected = self._client.wait_for_server(timeout=rospy.Duration(5))
        #Andy's Note: I do not yet know the ROS2 equivalent of rospy.Duration()
        if not connected:
            #rospy.logerr -->  self.teleop_node.get_logger().error(
                
            self.teleop_node.get_logger().error("Unable to connect to Teleop Action Server.")
            self.teleop_node.get_logger().error(
                "Ensure the action server is running and"
                + " you have the ROS_NAMESPACE env var set"
                + " to the namespace of your action server."
            )
            return
        #rospy.loginfo --> self.teleop_node.get_logger().info(
        self.teleop_node.get_logger().info("Connected to Teleop Action Server.")

    def read_instructions(self, instructions_file):
        print("test read_instructions") #Added by Andy for testing
        """Parse in a text file to identify new goals.
        Lines that start with # are considered comments and are ignored."""
        actions = []
        with open(instructions_file, "r") as reader:
            for line in reader.readlines():
                if line.startswith("#"):
                    continue
                line = line.rstrip()
                actions.append(line.lower())

        return actions

    def validate(self, action_list):
        print("test validate") #Added by Andy for testing
        """Check the list of parsed actions to ensure that they are all valid actions.
        If any single action is invalid, then the function will return False and the
        program will exit early."""

        if not type(action_list) is list:
            #rospy.logerr -->  self.teleop_node.get_logger().error(
            self.teleop_node.get_logger().error(
                "Teleop Client script was not given an object of type list"
            )
            return False

        valid_operations = [
            self.teleop_goal.MOVE_FORWARD_OPERATION,
            self.teleop_goal.MOVE_BACKWARD_OPERATION,
            self.teleop_goal.ROTATE_LEFT_OPERATION,
            self.teleop_goal.ROTATE_RIGHT_OPERATION,
            self.teleop_goal.RAISE_FRONT_ARM_OPERATION,
            self.teleop_goal.LOWER_FRONT_ARM_OPERATION,
            self.teleop_goal.RAISE_BACK_ARM_OPERATION,
            self.teleop_goal.LOWER_BACK_ARM_OPERATION,
            self.teleop_goal.DUMP_FRONT_DRUM_OPERATION,
            self.teleop_goal.DIG_FRONT_DRUM_OPERATION,
            self.teleop_goal.DUMP_BACK_DRUM_OPERATION,
            self.teleop_goal.DIG_BACK_DRUM_OPERATION,
            self.teleop_goal.STOP_OPERATION,
        ]

        # Validate every single action
        for action in action_list:

            action_components = action.split(" ")

            if len(action_components) != 2:
                #rospy.logerr -->  self.teleop_node.get_logger().error(
                self.teleop_node.get_logger().error(
                    "Incorrect number of instructions given. Expected 2 got %s\n%s",
                    len(action_components),
                    action,
                )
                return False

            op = action_components[0]
            duration = action_components[1]

            if op not in valid_operations:
                #rospy.logerr -->  self.teleop_node.get_logger().error(
                self.teleop_node.get_logger().error('Invalid operation received: {0}'.format(action))
                return False

            # Since Python2 does not support isnumeric, check for cast exceptions
            try:
                float(duration)
            except ValueError:
                #rospy.logerr -->  self.teleop_node.get_logger().error(
                self.teleop_node.get_logger().error(
                    "Non-number passed in for time duration: \n%s", action
                )

        return True

    def send_movement_goal(self, actions):
        movement_goal = Teleop.Goal()
        print("test send movement") #Added by Andy for testing
        """Accepts a list of actions to send to the action server.
        Feedback returned indicates the robot's current x, y, and heading."""
        for action in actions:
            print("actionLoop") #Added by Andy for testing
            instruction = action.split(" ")
            print(str(instruction)) #Added by Andy for testing
            direction = instruction[0]
            print(str(direction)) #Added by Andy for testing
            duration = float(instruction[1])
            print(str(duration)) #Added by Andy for testing
            #below two lines replace: goal = Teleop(direction, duration)
            movement_goal.operation = direction
            movement_goal.duration = duration
            self._client.send_goal(
                movement_goal,
                feedback_callback=self.feedback_callback
            )


            #rospy.loginfo --> self.teleop_node.get_logger().info(
            self.teleop_node.get_logger().info("{0} has been sent.".format(direction))

            # Wait an additional 2 seconds to account for any potential lag
            
            #Andy's Note: 
            #I can't find an equivalent in rclpy to actionlib's "wait_for_result" method for action clients
            #As a result, I will comment out the below six lines for testing purposes....
            #success = self._client.wait_for_result(
            #    rclpy.Duration(duration + 2.0)
            #)
            #if not success:
            #    rospy.logwarn("HOUSTON, WE HAVE A PROBLEM")
            #    break
            #...And just run a sleep function for two seconds to make up for the lag:
            time.sleep(2)
            #rospy.loginfo --> self.teleop_node.get_logger().info(
            self.teleop_node.get_logger().info(format(self._client._get_result()))

    def done_callback(self, status, result):
        print("test done callback") #Added by Andy for testing
        """When the goal is marked as finished, log the info to console."""
        #rospy.loginfo --> self.teleop_node.get_logger().info(
        self.teleop_node.get_logger().info("Status: ")
        self.teleop_node.get_logger().info(str(status))

        self.teleop_node.get_logger().info("Result: ")
        self.teleop_node.get_logger().info(str(result))

    def feedback_callback(self, feedback):
        print("test feedback callback") #Added by Andy for testing
        """Feedback comes formatted from the action server."""
        #rospy.loginfo --> self.teleop_node.get_logger().info(
        self.teleop_node.get_logger().info(feedback)
