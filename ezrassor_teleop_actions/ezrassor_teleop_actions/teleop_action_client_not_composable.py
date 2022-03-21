# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from action_msgs.msg import GoalStatus
from ezrassor_teleop_interfaces.action import Teleop
from threading import Thread
import time
import rclpy
from rclpy.action import ActionClient


# def feedback_cb(logger, feedback):
#     logger.info('Received feedback: {0}'.format(feedback.feedback.sequence))

def feedback_cb(logger, feedback):
    feedback_msg = feedback.feedback
    logger.info("feedback heading: {0} feedback x: {1} feedback y: {2}"
    .format(feedback_msg.heading, feedback_msg.x, feedback_msg.y))

def cancel_goal(goal_handle):
    """
    This function only works if you pass a variable of the form Future().result() to it. 
    If you try something like this:
        x = send_goal_future.result() 
        cancel_goal(x) 
    You'll get the following Error:
    AttributeError: 'NoneType' object has no attribute 'cancel_goal_async'
    """
    time.sleep(2.5)
    goal_handle.cancel_goal_async()

    # if len(cancel_response.goals_canceling) > 0:
    #     print('Goal successfully canceled')
    # else:
    #     print('Goal failed to cancel')

    #rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    node = rclpy.create_node('teleop_action_client')

    action_client = ActionClient(node, Teleop, 'Teleop')

    node.get_logger().info('Waiting for action server...')

    action_client.wait_for_server()

    goal_msg = Teleop.Goal()
    goal_msg.operation = "move-forward"
    goal_msg.duration = 3.0
    node.get_logger().info('Sending goal request...')

    send_goal_future = action_client.send_goal_async(
        goal_msg, feedback_callback=lambda feedback: feedback_cb(node.get_logger(), feedback))
    
    send_goal_thread = Thread(rclpy.spin_until_future_complete(node, send_goal_future))

    cancel_goal_thread = Thread(cancel_goal(send_goal_future.result())) 
    send_goal_thread.start()
    cancel_goal_thread.start()
    goal_handle = send_goal_future.result()
    if not goal_handle.accepted: #TODO: Fix this to output all possible goal handles statuses
        node.get_logger().info('Goal rejected :(')
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info('Goal accepted :)')

    get_result_future = goal_handle.get_result_async()

    rclpy.spin_until_future_complete(node, get_result_future)

    result = get_result_future.result().result
    status = get_result_future.result().status
    if status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info(
           'Goal succeeded!\nResult x: {0}\nResult y: {1}'.format(result.x, result.y))
    else:
        node.get_logger().info('Goal failed with status code: {0}'.format(status))

    action_client.destroy()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
