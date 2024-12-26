import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import cv2
import math
import time
import threading
from rclpy.qos import QoSProfile

class AgvAutomation(Node):
    def __init__(self):
        super().__init__('agv_automation')

        # Aboneliklerin tanımlanması
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.occupancy_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Publisher tanımlanması
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Haritayı görselleştirmek için ayrı bir thread başlatılması
        threading.Thread(target=self.display_occupancy_grid, daemon=True).start()



    def goal_pose_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def lidar_callback(self, msg):
        self.scan = msg.ranges

    def occupancy_callback(self, msg):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = np.array(self.map_data.data).reshape((self.height, self.width))

    def display_occupancy_grid(self):
        while True:
            if not hasattr(self, 'map_data') or not hasattr(self, 'x') or not hasattr(self, 'y'):
                time.sleep(0.1)
                continue

            # Haritayı normalize et ve renkli hale getir
            normalized_grid = 255 - (self.data / 100.0 * 255).astype(np.uint8)
            bgr_grid = cv2.cvtColor(normalized_grid, cv2.COLOR_GRAY2BGR)

            # Aracın ve hedefin harita üzerindeki konumlarının ölçeklenmesi
            vehicle_x = int((self.x - self.originX) / self.resolution)
            vehicle_y = int((self.y - self.originY) / self.resolution)
            if 0 <= vehicle_x < self.width and 0 <= vehicle_y < self.height:
                cv2.circle(bgr_grid, (vehicle_x, vehicle_y), 10, (0, 255, 255), -1)

            if hasattr(self, 'goal'):
                goal_x = int((self.goal[0] - self.originX) / self.resolution)
                goal_y = int((self.goal[1] - self.originY) / self.resolution)
                cv2.circle(bgr_grid, (goal_x, goal_y), 5, (0, 255, 0), -1)
                cv2.line(bgr_grid, (vehicle_x, vehicle_y), (goal_x, goal_y), (0, 255, 0), thickness=1)

            # Haritayı yansıt ve göster
            bgr_grid = cv2.flip(bgr_grid, 0)
            cv2.imshow("Occupancy Grid", bgr_grid)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC ile çıkış
                break

        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    agv_automation = AgvAutomation()
    rclpy.spin(agv_automation)
    agv_automation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
