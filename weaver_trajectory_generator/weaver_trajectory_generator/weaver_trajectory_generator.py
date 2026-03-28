#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from weaver_interfaces.srv import WeaverTrajectory
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory

import configparser
import cv2
from math import sin, cos, radians

class WeaverTrajectoryNode(Node):
    def __init__(self):
        super().__init__('weaver_trajectory_node')

        # Declare the service
        self.srv = self.create_service(WeaverTrajectory, 'trajectory_generator_service', self.generate_trajectory_cb)

        # Load configuration
        config_path = os.path.join(
                get_package_share_directory('weaver_trajectory_generator'),
                'config',
                'weaver_settings.config')

        # if len(sys.argv) > 1:
        #     config_path = sys.argv[1]
        # else:
        #     config_path = default_fallback_path

        self.config = configparser.ConfigParser()
        self.config.read(config_path)

        if not self.config.sections():
            self.get_logger().error("Config file not loaded or empty!")


        self.nails = int(self.config['parameters']['nails'])
        self.diameter = float(self.config['parameters']['diameter_mm'])
        self.image_path = self.config['image']['path']
        self.max_lines = int(self.config['parameters']['max_lines'])
        self.offset_mm = float(self.config['parameters'].get('offset_mm', 5.0))

        self.get_logger().info('Trajectory server ready.')

    def generate_trajectory_cb(self, request, response):
        image = cv2.imread(self.image_path)
        cleaned = cv2.imread(self.image_path)
        cleaned[:, :] = 255  # clear for visualization

        center = [image.shape[1] // 2, image.shape[0] // 2]
        radius = self.diameter / 2.0
        angle_step = 360 / self.nails

        nail_positions = []
        for i in range(self.nails):
            angle = radians(i * angle_step)
            x = int(center[0] + ((center[0] - 1) * sin(-angle)))
            y = int(center[1] + ((center[0] - 1) * cos(-angle)))
            nail_positions.append([y, x])

        actual_point = nail_positions[0]
        actual_index = 0
        lines = 0

        x_array = []
        y_array = []

        waypoints = []

        while lines < self.max_lines:
            bigger = [0, actual_point, actual_point]
            next_index = actual_index
            for i in range(self.nails):
                candidate = nail_positions[i]
                black_pixels = self.pixels_analysis(image, actual_point, candidate)
                if black_pixels > bigger[0]:
                    bigger = [black_pixels, actual_point, candidate]
                    next_index = i

            if bigger[0] == 0:
                break

            prev_index = (next_index - 1) % self.nails
            after_index = (next_index + 1) % self.nails

            mid1_angle = radians((next_index + after_index) / 2 * angle_step)
            mid1_x = (radius * cos(mid1_angle)) / 1000
            mid1_y = (radius * sin(mid1_angle)) / 1000

            angle_nail = radians(next_index * angle_step)
            back_x = ((radius + self.offset_mm) * cos(angle_nail)) / 1000
            back_y = ((radius + self.offset_mm) * sin(angle_nail)) / 1000

            mid2_angle = radians((next_index + prev_index) / 2 * angle_step)
            mid2_x = (radius * cos(mid2_angle)) / 1000
            mid2_y = (radius * sin(mid2_angle)) / 1000

            for x, y in [(mid1_x, mid1_y), (back_x, back_y), (mid2_x, mid2_y)]:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "board"  # or your reference frame
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = 0.01
                pose_stamped.pose.orientation.w = 1.0
                waypoints.append(pose_stamped)
                x_array.append(x)
                y_array.append(y)

            actual_point = bigger[2]
            actual_index = next_index

            cv2.line(image, (bigger[1][1], bigger[1][0]), (bigger[2][1], bigger[2][0]), (255, 255, 255), 1)
            cv2.line(cleaned, (bigger[1][1], bigger[1][0]), (bigger[2][1], bigger[2][0]), (0, 0, 0), 1)

            cv2.imwrite("debug_img.jpg", cleaned)
            lines += 1

        response.waypoints = waypoints
        self.get_logger().info(f'Returning {len(waypoints)} poses')
        self.get_logger().info(f'x mean {np.mean(np.array(x_array))} y mean {np.mean(np.array(y_array))}')
        self.get_logger().info(f'x max {max(x_array)} y max {max(y_array)}')
        self.get_logger().info(f'x min {min(x_array)} y min {min(y_array)}')
        return response

    def pixels_analysis(self, image, point_1, point_2):
        yd = point_2[0] - point_1[0]
        xd = point_2[1] - point_1[1]
        step = max(abs(yd), abs(xd))
        black_pixels = 0
        for pixel in range(1, step):
            y = int(round(point_1[0] + (yd * (pixel / step))))
            x = int(round(point_1[1] + (xd * (pixel / step))))
            if all(image[y, x] == 0):
                black_pixels += 1
        return black_pixels

def main(args=None):
    rclpy.init(args=args)
    node = WeaverTrajectoryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
