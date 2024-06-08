#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import MoveToPos

import time

class MoveToPositionNode(Node):
    def __init__(self):
        super().__init__("move_to_position")
        
        #Parameter
        self.declare_parameter("min_pos", 0)
        self.min_pos_ = self.get_parameter("min_pos").value
        self.declare_parameter("max_pos", 100)
        self.max_pos_ = self.get_parameter("max_pos").value
        self.declare_parameter("start_pos", 50)
        self.position_ = self.get_parameter("start_pos").value
        
        
        self.goal_handle_ : ServerGoalHandle = None
        self.move_to_position_server_ = ActionServer(self, MoveToPos, "move_to_pos", execute_callback=self.execute_callback)

        self.get_logger().info("move_to_position has been started")

    def execute_callback(self, goal_handle:ServerGoalHandle):
        #get request from goal
        target_position=goal_handle.request.target_position
        vel=goal_handle.request.velocity
        rest_pos =abs(target_position - self.position_)
        
        feedback = MoveToPos.Feedback()
        result = MoveToPos.Result()
        
        self.get_logger().info("Executing the goal")
        while self.position_ != target_position:
            if target_position > self.position_:
                # plus
                if rest_pos >= vel:
                    self.position_ += vel
                else:
                    self.position_ += rest_pos
                time.sleep(1)
            elif target_position < self.position_:
                # minus
                if rest_pos >= vel:
                    self.position_ -= vel
                else:
                    self.position_ -= rest_pos
                time.sleep(1)
                
        goal_handle.succeed()
        result.position = self.position_
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPositionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main() 
    