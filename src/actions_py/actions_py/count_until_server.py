#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from my_robot_interfaces.action import CountUntil

class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ =[]
        self.count_until_server_ = ActionServer(self, 
                                                CountUntil, 
                                                "count_until", 
                                                goal_callback=self.goal_callback,
                                                handle_accepted_callback=self.handle_accepted_callback,
                                                execute_callback=self.execute_callback,
                                                cancel_callback= self.cancel_callback,
                                                callback_group=ReentrantCallbackGroup())
        

        self.get_logger().info("count_until_server has been started")

    def goal_callback(self, goal_request:CountUntil.Goal):
        self.get_logger().info("Recieved goal")
        
        #Policy: refuse new goal if current goal still active
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("A goal is already active, rejecting the new goal")
        #         return GoalResponse.REJECT
        
        
        
        #validate the goal request 
        if goal_request.target_number <= 0:
            self.get_logger().info("invalid goal, rejection the goal")
            return GoalResponse.REJECT
        
        #policy: preempt exsisting goal when reseving new goal
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Abort current goal and accepting new goal")
        #         self.goal_handle_.abort()
                
        
        self.get_logger().info("valid goal, accepting the goal")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle : ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.get_logger().info("new goal had been added to the queue")
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("reseived cancel request")
        return CancelResponse.ACCEPT #or reject

    def execute_callback(self, goal_handle:ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        
        #get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period
        
        #Execute the action
        self.get_logger().info("Executing the goal")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0
        for i in range(target_number):
            if not goal_handle.is_active:
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            if goal_handle.is_cancel_requested:
                self.get_logger().info("canceling goal")
                goal_handle.canceled()
                result.reached_number = counter
                self.process_next_goal_in_queue()
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
        self.process_next_goal_in_queue()
        return result
    
    def process_next_goal_in_queue(self):
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None
                

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()