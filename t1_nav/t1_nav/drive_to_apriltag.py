import rclpy
from rclpy.node import Node

import tf2_ros

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

import math
import numpy as np
import transforms3d as t3d

class FrameListener(Node):
    def __init__(self):
        super().__init__('drive_to_apriltag')

        # Apriltag listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.6, self.timer_callback)

        self.at_tag = False  # Flag to track if at a tag
        self.publisher = self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def timer_callback(self):
        if not self.at_tag:
            # If not at a tag, stop searching for AprilTags
            return

        from_frame = "base_footprint"
        to_frame = "tagStandard41h12"

        msg = Twist()
        try:
            # do a lookup transform between 'base_footprint' and apriltag frame
            trans = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())
            self.tf_buffer.clear()

            # Calculate angle and distance to the tag
            angle = math.atan2(trans.transform.translation.y, trans.transform.translation.x) * 180 / math.pi  # In degrees
            distance = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

            ANGLETOLERANCE = 20
            DISTANCETOLERANCE = 0.3
            print('\nAngle: %.1f Distance: %.3f' % (angle, distance))

            # Check if within tolerance to go to the tag
            if distance <= DISTANCETOLERANCE:
                print("Reached AprilTag, stopping.")
                self.at_tag = False
                return

            # Adjust movement based on angle and distance
            scale_rotation_rate = 0.02
            scale_forward_speed = 0.4

            if (angle > ANGLETOLERANCE) or (angle < -ANGLETOLERANCE):
                if angle > 0:
                    msg.angular.z = scale_rotation_rate * angle + 0.3
                    print("Turning right")
                else:
                    msg.angular.z = scale_rotation_rate * angle - 0.3
                    print("Turning left")
            else:
                msg.linear.x = scale_forward_speed * distance
                print("Moving forward")
            
            self.publisher.publish(msg)
            
        except:
            # If lookup fails, stop and print error
            msg.linear.x = 0
            msg.angular.z = 0
            self.publisher.publish(msg)
            print("Error: AprilTag not found")
