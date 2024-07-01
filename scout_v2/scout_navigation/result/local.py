import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import json
import csv
from scipy.spatial.distance import euclidean
from datetime import datetime
import pandas as pd
import os
import re
from action_msgs.msg import GoalStatusArray
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from scipy.spatial.transform import Rotation as R
class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goals = [
            {'x': 8.04, 'y': 10.81, 'z': 0.70, 'w': 0.71}, {'x': 5.45, 'y': 18.76, 'z': 0.99, 'w': 0.03},
              {'x': -2.96, 'y': 23.45, 'z': 0.70, 'w': 0.71}, {'x': 5.06, 'y': 25.5, 'z': -0.02, 'w': 0.99} # Example goals
        ]
        self.current_goal_index = 0
        self.execution_folder = self.create_execution_folder()
        self.send_next_goal()
        # Initialize self.data as an empty list to hold your data points
        self.data = []
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.odom_data = None  # Add this line to initialize odom_data
        self.path_data = None
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 100)
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.start_time = self.get_clock().now()  # Initialize start time here
        self.last_position = None
        # Additional attributes for AOL calculation
        self.last_pose = None  # Store the last pose (position and orientation)
        self.total_heading_change = 0.0  # Total heading change (radians)
        self.total_path_length = 0.0  # Total path length (meters)

        self.total_distance = 0.0  # Initialize total distance
        self.last_yaw = None  # To store the yaw (heading) from the last odometry message
        self.total_aol = 0.0  # Initialize total Absolute Orientation Loss (AOL)

    def cmd_vel_callback(self, msg):
        self.latest_linear_velocity = msg.linear.x
        self.latest_angular_velocity = msg.angular.z


    def send_next_goal(self):
        if self.current_goal_index < len(self.goals):
            goal_dict = self.goals[self.current_goal_index]
            goal_msg = NavigateToPose.Goal()

            # Correctly create a PoseStamped message for the goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "odom"  # Use "map" or another appropriate frame_id
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_dict['x']
            goal_pose.pose.position.y = goal_dict['y']
            goal_pose.pose.orientation.z = goal_dict['z']
            goal_pose.pose.orientation.w = 1.0  # Assuming the goal orientation is neutral; adjust if necessary            
            # goal_pose.pose.orientation.z = goal_dict['z']
            # goal_pose.pose.orientation.w = 1.0  # Assuming the goal orientation is neutral; adjust if necessary

            goal_msg.pose = goal_pose

            self.action_client.wait_for_server()
            self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
            self.current_goal_index += 1
        else:
            # No more goals, shut down
            self.shutdown()
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)



    def timer_callback(self):
        if self.odom_data is not None:
            current_time = self.get_clock().now()
            elapsed_time = current_time - self.start_time
            elapsed_time_sec = elapsed_time.nanoseconds / 1e9  # Time in seconds
            current_pose = self.odom_data.pose.pose

            # Update path length and heading change
            if self.last_pose is not None:
                current_position = self.odom_data.pose.pose.position
                last_position = self.last_pose.position
                distance = self.calculate_distance(last_position, current_position)
                self.total_path_length += distance
                heading_change = self.calculate_heading_change(self.last_pose, current_pose)
                self.total_heading_change += heading_change

            # Save current pose for the next iteration
            self.last_pose = current_pose

            # Calculate AOL and add to data_point if path length is non-zero
            if self.total_path_length > 0:
                aol = self.total_heading_change / self.total_path_length
            else:
                aol = 0.0


            data_point = {
                'Goal Number': self.current_goal_index,
                'Time': elapsed_time_sec,
                'X': self.odom_data.pose.pose.position.x,
                'Y': self.odom_data.pose.pose.position.y,
                'Z': self.odom_data.pose.pose.position.z,
                'linear_velocity': self.odom_data.twist.twist.linear.x,
                'angular_velocity': self.odom_data.twist.twist.angular.z,
                'Total Distance': self.total_distance,
                'AOL': aol  # Add AOL to the data being saved
            
            }
            self.data.append(data_point)

    def quaternion_to_yaw(self, quaternion):
        """Converts a quaternion to yaw angle in radians."""
        # Use scipy's Rotation class for quaternion to Euler conversion
        r = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        yaw = r.as_euler('zyx')[0]  # Extract the yaw component (rotation around z-axis)
        return yaw

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal succeeded!')
            if self.current_goal_index >= len(self.goals):
                # Last goal reached, perform shutdown procedures
                self.save_data_to_excel()
                self.save_data_to_json()  # Ensure JSON data is also saved
                self.shutdown()
            else:
                self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        # You can implement feedback handling here if needed
        pass

    def quaternion_to_yaw(self, quaternion):
        """Converts a quaternion to yaw angle in radians."""
        # Use scipy's Rotation class for quaternion to Euler conversion
        r = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        yaw = r.as_euler('zyx')[0]  # Extract the yaw component (rotation around z-axis)
        return yaw    
    def odom_callback(self, msg: Odometry):
        self.odom_data = msg
        current_position = msg.pose.pose.position
        if self.last_position is not None:
            # Calculate the distance between the last position and the current position
            # This works if calculate_distance is an instance method
            distance = self.calculate_distance(self.last_position, current_position)


            self.total_distance += distance
        self.last_position = current_position  # Update the last known position


        # New method to calculate the distance between two points
    def calculate_distance(self, last_position, current_position):
        """Calculate the Euclidean distance between two points."""
        dx = current_position.x - last_position.x
        dy = current_position.y - last_position.y
        dz = current_position.z - last_position.z
        return np.sqrt(dx**2 + dy**2 + dz**2)



    # New method to calculate the heading change between two poses
    def calculate_heading_change(self, pose1, pose2):
        # Extract the yaw from the pose orientations
        quat1 = pose1.orientation
        euler1 = R.from_quat([quat1.x, quat1.y, quat1.z, quat1.w]).as_euler('xyz')
        yaw1 = euler1[2]

        quat2 = pose2.orientation
        euler2 = R.from_quat([quat2.x, quat2.y, quat2.z, quat2.w]).as_euler('xyz')
        yaw2 = euler2[2]

        # Calculate the difference in yaw
        heading_change = np.abs(yaw2 - yaw1)

        # Adjust for passing through 2π boundary
        if heading_change > np.pi:
            heading_change = (2 * np.pi) - heading_change

        return heading_change

    def shutdown(self):
        # Call this method to perform the shutdown
        self.save_data_to_excel()  # Save data before shutting down
        self.save_data_to_json()  # Ensure JSON data is also saved
        self.get_logger().info('Shutting down, saving data.')
        rclpy.shutdown()


    def create_execution_folder(self):
        # Get list of existing directories in the current directory
        dirs = [d for d in os.listdir(os.getcwd()) if os.path.isdir(d) and d.startswith('test')]
        
        # Find the highest number used so far
        max_num = 0
        for d in dirs:
            # Find the digits in the directory name and convert to integer
            match = re.search(r'test(\d+)', d)
            if match:
                num = int(match.group(1))
                max_num = max(max_num, num)
        
        # Increment the highest number for the new directory
        new_dir_name = f"test{max_num + 1}"
        new_dir_path = os.path.join(os.getcwd(), new_dir_name)
        os.makedirs(new_dir_path, exist_ok=True)
        return new_dir_path

    def save_data_to_excel(self):
        # This function now saves the velocity data along with the rest
        filename = os.path.join(self.execution_folder, 'data.xlsx')
        df = pd.DataFrame(self.data)
        df.to_excel(filename, index=False)
    def save_data_to_json(self):
        json_path = os.path.join(self.execution_folder, 'data.json')
        with open(json_path, 'w') as json_file:
            json.dump(self.data, json_file, indent=4)



def main(args=None):
    rclpy.init(args=args)
    navigator = RobotNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()

if __name__ == '__main__':
    rclpy.init()
    navigator = RobotNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()





















#     import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path, Odometry
# from geometry_msgs.msg import PoseStamped, Twist
# import numpy as np
# import json
# import csv
# from scipy.spatial.distance import euclidean
# from datetime import datetime
# import pandas as pd
# import os
# from action_msgs.msg import GoalStatusArray
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose

# class RobotNavigator(Node):
#     def __init__(self):
#         super().__init__('robot_navigator')
#         self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
#         self.goals = [
#             {'x': -1.0, 'y': -2.0}, {'x': -3.0, 'y': -4.0}  # Example goals
#         ]
#         self.current_goal_index = 0
#         self.execution_folder = self.create_execution_folder()
#         self.aol_file_path = os.path.join(self.execution_folder, 'path_aol.json')
#         self.cusps_file_path = os.path.join(self.execution_folder, 'path_cusps.csv')
#         self.cusps_csv_writer = self.setup_cusps_csv()
#         self.send_next_goal()
#         # Initialize self.data as an empty list to hold your data points
#         self.data = []
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.odom_data = None  # Add this line to initialize odom_data
#         self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 100)
#         self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
    
    
#     def create_execution_folder(self):
#         folder_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
#         path = os.path.join(os.getcwd(), folder_name)
#         os.makedirs(path, exist_ok=True)
#         return path
#     def cmd_vel_callback(self, msg):
#         self.latest_linear_velocity = msg.linear.x
#         self.latest_angular_velocity = msg.angular.z
#     def setup_cusps_csv(self):
#         file_path = self.cusps_file_path
#         cusps_file = open(file_path, 'w', newline='')
#         csv_writer = csv.writer(cusps_file)
#         csv_writer.writerow(['timestamp', 'x', 'y'])  # CSV Header
#         return csv_writer

#     def send_next_goal(self):
#         if self.current_goal_index < len(self.goals):
#             goal_dict = self.goals[self.current_goal_index]
#             goal_msg = NavigateToPose.Goal()

#             # Correctly create a PoseStamped message for the goal pose
#             goal_pose = PoseStamped()
#             goal_pose.header.frame_id = "odom"  # Use "map" or another appropriate frame_id
#             goal_pose.header.stamp = self.get_clock().now().to_msg()
#             goal_pose.pose.position.x = goal_dict['x']
#             goal_pose.pose.position.y = goal_dict['y']
#             goal_pose.pose.orientation.w = 1.0  # Assuming the goal orientation is neutral; adjust if necessary

#             goal_msg.pose = goal_pose

#             self.action_client.wait_for_server()
#             self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#             self.send_goal_future.add_done_callback(self.goal_response_callback)
#             self.current_goal_index += 1
#         else:
#             # No more goals, shut down
#             self.shutdown()
#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected')
#             return
#         self.get_result_future = goal_handle.get_result_async()
#         self.get_result_future.add_done_callback(self.get_result_callback)
#     def timer_callback(self):
#         # This will be called every 0.1 seconds
#         if self.odom_data:  # Make sure we have odometry data
#             current_time = self.get_clock().now().to_msg()
#             self.data.append({
#                 'timestamp': str(current_time),
#                 'x': self.odom_data.pose.pose.position.x,
#                 'y': self.odom_data.pose.pose.position.y,
#                 'z': self.odom_data.pose.pose.position.z,
#                 'linear_velocity': self.odom_data.twist.twist.linear.x,
#                 'angular_velocity': self.odom_data.twist.twist.angular.z
#             })
#             # Save to JSON regularly if needed
#             self.save_data_to_json()

#     def get_result_callback(self, future):
#         result = future.result().result
#         if result:
#             self.get_logger().info('Goal succeeded!')
#             if self.current_goal_index >= len(self.goals):
#                 # Last goal reached, perform shutdown procedures
#                 self.save_data_to_excel()
#                 self.save_data_to_json()  # Ensure JSON data is also saved
#                 self.shutdown()
#             else:
#                 self.send_next_goal()

#     def feedback_callback(self, feedback_msg):
#         # You can implement feedback handling here if needed
#         pass
#     # Modified odom_callback to store latest odometry data
#     def odom_callback(self, msg: Odometry):
#         self.odom_data = msg



#     def path_callback(self, msg: Path):
#         aol, cusps = self.calculate_aol_and_cusps(msg)
#         with open(self.aol_file_path, 'a') as aol_file:
#             aol_data = {
#                 'timestamp': str(datetime.now()),
#                 'aol': aol
#             }
#             aol_file.write(json.dumps(aol_data) + '\n')
        
#         for cusp in cusps:
#             self.cusps_csv_writer.writerow([str(datetime.now()), cusp[0], cusp[1]])

#     @staticmethod
#     def calculate_aol_and_cusps(path_msg: Path):
#         points = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
#         total_angle_change = 0.0
#         total_length = 0.0
#         cusps = []

#         for i in range(1, len(points) - 1):
#             p1, p2, p3 = points[i - 1], points[i], points[i + 1]
#             seg_len1 = euclidean(p1, p2)
#             seg_len2 = euclidean(p2, p3)
#             heading1 = np.array(p2) - np.array(p1)
#             heading2 = np.array(p3) - np.array(p2)
#             norm_heading1 = heading1 / np.linalg.norm(heading1) if np.linalg.norm(heading1) else heading1
#             norm_heading2 = heading2 / np.linalg.norm(heading2) if np.linalg.norm(heading2) else heading2
#             angle_change = np.arccos(np.clip(np.dot(norm_heading1, norm_heading2), -1.0, 1.0))

#             if angle_change > np.pi / 2:
#                 cusps.append(p2)

#             total_angle_change += angle_change
#             total_length += (seg_len1 + seg_len2) / 2

#         aol = total_angle_change / total_length if total_length > 0 else float('inf')
#         return aol, cusps

#     def shutdown(self):
#         # Call this method to perform the shutdown
#         self.save_data_to_excel()  # Save data before shutting down
#         self.save_data_to_json()  # Ensure JSON data is also saved
#         self.get_logger().info('Shutting down, saving data.')
#         rclpy.shutdown()

#     def save_data_to_excel(self):
#         df = pd.DataFrame(self.data, columns=['Goal Number', 'Time', 'X', 'Y', 'Z', 'Linear Velocity', 'Angular Velocity'])
#         df.to_excel(os.path.join(self.execution_folder, 'data.xlsx'), index=False)
#     def save_data_to_json(self):
#         json_path = os.path.join(self.execution_folder, 'data.json')
#         with open(json_path, 'w') as json_file:
#             json.dump(self.data, json_file, indent=4)



# def main(args=None):
#     rclpy.init(args=args)
#     navigator = RobotNavigator()
#     rclpy.spin(navigator)
#     navigator.destroy_node()

# if __name__ == '__main__':
#     rclpy.init()
#     navigator = RobotNavigator()
#     rclpy.spin(navigator)
#     navigator.destroy_node()
#     rclpy.shutdown()
