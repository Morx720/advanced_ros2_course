#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.lifecycle import LifecycleNode
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from my_robot_interfaces.action import MoveToPos
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

class MoveToPositionNode(LifecycleNode):
    def __init__(self):
        super().__init__("move_to_position")
        
        self.goal_policy_ = 2 
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ =[]
        self.server_activated_ = False

    def on_configure(self, state: LifecycleState):
        
        #Parameter
        self.declare_parameter("min_pos", 0)
        self.min_pos_ = self.get_parameter("min_pos").value
        self.declare_parameter("max_pos", 100)
        self.max_pos_ = self.get_parameter("max_pos").value
        self.declare_parameter("start_pos", 50)
        self.position_ = self.get_parameter("start_pos").value
        self.declare_parameter("robot_name", "robot1")
        self.robot_name_ = self.get_parameter("robot_name").value
        
        self.move_to_position_ = ActionServer(self, 
                                        MoveToPos, 
                                        "move_"+self.robot_name_+"_to_pos", 
                                        goal_callback=self.goal_callback,
                                        handle_accepted_callback=self.handle_accepted_callback,
                                        execute_callback=self.execute_callback,
                                        cancel_callback= self.cancel_callback,
                                        callback_group=ReentrantCallbackGroup())
        self.get_logger().info("move_to_position_server has been started")
        self.get_logger().info("current position: "+ str(self.position_))

        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState):
        self.get_logger().info("activate node")
        self.server_activated_ = True
        return super().on_activate(state)
    
    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info("deactivate node")
        self.server_activated_ = False
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal")
                self.goal_handle_.abort()
        return super().on_deactivate(state)
    
    def on_cleanup(self, state: LifecycleState):
        self.undeclare_parameter("min_pos")
        self.undeclare_parameter("max_pos")
        self.undeclare_parameter("start_pos")
        self.undeclare_parameter("robot_name")
        
        self.move_to_position_.destroy()
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: LifecycleState):
        self.undeclare_parameter("min_pos")
        self.undeclare_parameter("max_pos")
        self.undeclare_parameter("start_pos")
        self.undeclare_parameter("robot_name")
        self.move_to_position_.destroy()
        return TransitionCallbackReturn.SUCCESS
    
    
    def goal_callback(self, goal_request:MoveToPos.Goal):
        self.get_logger().info("Recieved goal")
        
        if not self.server_activated_:
            self.get_logger().warn("Server inactive, rejecting the goal")
            return GoalResponse.REJECT
        
        #Policy: refuse new goal if current goal still active
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("A goal is already active, rejecting the new goal")
        #         return GoalResponse.REJECT
        
        
        
        #validate the goal request 
        if not (self.min_pos_ <= goal_request.target_position <= self.max_pos_) or goal_request.velocity <= 0:
            self.get_logger().info("invalid goal, rejection the goal")
            return GoalResponse.REJECT
        
        #policy: preempt exsisting goal when reseving new goal
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accepting new goal")
                self.goal_handle_.abort()
                
        
        self.get_logger().info("valid goal, accepting the goal")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle : ServerGoalHandle):
        if not self.goal_policy_ == 3:
            goal_handle.execute()
            return
        
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
        target_position = goal_handle.request.target_position
        velocity = goal_handle.request.velocity
        
        #Execute the action
        self.get_logger().info("Executing the goal")
        feedback = MoveToPos.Feedback()
        result = MoveToPos.Result()

        while rclpy.ok():
            if not goal_handle.is_active:
                result.position = self.position_
                result.message = "Preempted by another goal, or node deactivated"
                if self.goal_policy_ ==3:
                    self.process_next_goal_in_queue()
                return result
            if goal_handle.is_cancel_requested:
                self.get_logger().info("canceling goal")
                goal_handle.canceled()
                result.position = self.position_
                result.message = "goal canceled or aborted"
                if self.goal_policy_ == 3:
                    self.process_next_goal_in_queue()
                return result
            
            rem_dest =abs(target_position - self.position_)
            
            self.get_logger().debug(f"current pos: {self.position_}")
            self.get_logger().debug(f"rem dis: {rem_dest}")
            
            
            #prosess goal
            if target_position > self.position_:
                # plus
                if rem_dest >= velocity:
                    self.position_ += velocity
                else:
                    self.position_ += rem_dest
                
            elif target_position < self.position_:
                # minus
                if rem_dest >= velocity:
                    self.position_ -= velocity
                else:
                    self.position_ -= rem_dest
            time.sleep(1)
            
            self.get_logger().info(f"position: {self.position_}")
            feedback.current_position = self.position_
            goal_handle.publish_feedback(feedback=feedback)
            
        #once done, set goal final state 
        
        goal_handle.succeed()
        
        # and send the result
        
        result.position = self.position_
        result.message = "position reatched"
        if self.goal_policy_ == 3:
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
    node = MoveToPositionNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()