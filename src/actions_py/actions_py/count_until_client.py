#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from my_robot_interfaces.action import CountUntil


class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client")

        self.count_until_client_ = ActionClient(self, CountUntil, "count_until")

        self.get_logger().info("count_until_client has been started")

    def send_goal(self, target_number, period):
        #wait for the server
        while not self.count_until_client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Server....")
            
        #creat a goal
        
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period
        
        #sending the gaol
        
        self.get_logger().info("sending the gaol")
        self.count_until_client_.send_goal_async(goal).\
                                add_done_callback(self.goal_resopnse_callback)
        
    def goal_resopnse_callback(self, future):
        self.goal_handle_= future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
            
    def goal_result_callback(self,future):
        result = future.result().result
        
        self.get_logger().info("result:" + str(result.reached_number))
        


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(6, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()