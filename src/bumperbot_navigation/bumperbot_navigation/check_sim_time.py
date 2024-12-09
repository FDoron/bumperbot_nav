#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

def check_sim_time():
    rclpy.init()
    node = Node("sim_time_checker")
    
    node_names = node.get_node_names()
    for name in node_names:
        try:
            value = node.get_parameter('use_sim_time')
            print(f"{name}: {value}")
        except:
            pass
    
    rclpy.shutdown()