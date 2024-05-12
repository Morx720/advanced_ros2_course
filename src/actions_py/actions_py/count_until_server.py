#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from my_robot_interfaces.action import CountUntil

class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        
        self.count_until_server_ = ActionServer(self, 
                                                CountUntil, 
                                                "count_until", 
                                                execute_callback=self.execute_callback,
                                                cancel_callback= self.cancel_callback,
                                                goal_callback=self.goal_callback,
                                                callback_group=ReentrantCallbackGroup())
        

        self.get_logger().info("count_until_server has been started")

    def goal_callback(self, goal_request:CountUntil.Goal):
        self.get_logger().info("Recieved goal")
        
        #validate the goal request 
        if goal_request.target_number <= 0:
            self.get_logger().info("invalid goal, rejection the goal")
            return GoalResponse.REJECT
        
        self.get_logger().info("valid goal, accepting the goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("reseived cancel request")
        return CancelResponse.ACCEPT #or reject

    def execute_callback(self, goal_handle:ServerGoalHandle):
        #get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period
        
        #Execute the action
        self.get_logger().info("Executing the goal")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0
        for i in range(target_number):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("canceling goal")
                goal_handle.canceled()
                result.reached_number = counter
                return result
            counter += 1
            self.get_logger().info(str(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)
            
        #once done, set goal final state 
        
        goal_handle.succeed()
        
        # and send the result
        
        result.reached_number = counter
        return result
            

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()