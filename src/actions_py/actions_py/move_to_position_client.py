#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import MoveToPos
from my_robot_interfaces.msg import Trigger


class MoveToPosClientNode(Node):
    def __init__(self):
        super().__init__("move_to_pos_client")
        self.goal_handle_ = None
        self.move_to_pos_client_ = ActionClient(self, MoveToPos, "move_to_pos")
        self.cancel_goal_sub_ = self.create_subscription(Trigger, "cancel_move_to_pos_goal", self.callback_cancel_move,10)

        self.get_logger().info("move_to_pos_client has been started")

    def send_goal(self, target_position, velocity):
        #wait for the server
        while not self.move_to_pos_client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Server....")
            
        #creat a goal
        
        goal = MoveToPos.Goal()
        goal.target_position = target_position
        goal.velocity = velocity
        
        #sending the gaol
        
        self.get_logger().info("sending the goal")
        self.move_to_pos_client_.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).\
                                add_done_callback(self.goal_resopnse_callback)
                                

    def callback_cancel_move(self, msg):
        self.cancel_goal()
        

    def cancel_goal(self):
        if self.goal_handle_ is not None:
            self.get_logger().info("send a cancel request")
            self.goal_handle_.cancel_goal_async()

        
        
        
    def goal_resopnse_callback(self, future):
        self.goal_handle_: ClientGoalHandle= future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal got rejected")
            
    def goal_result_callback(self,future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("result:" + "Position: " + str(result.position) + " " + "Massage: " + result.message)
        
    def goal_feedback_callback(self, feedback_msg):
        position = feedback_msg.feedback.current_position
        self.get_logger().info("got feedback: " + str(position))
        


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPosClientNode()
    node.send_goal(80, 1)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()