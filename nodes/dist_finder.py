#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from f112th_sim_2402_bravo.msg import AngleDistance  # Ensure this import is correct
import math
import matplotlib.pyplot as plt

class AngleDistancesReader(Node):
    def __init__(self):
        super().__init__('angle_distances_reader')
        self.subscription = self.create_subscription(
            AngleDistance,
            'angle_distances',
            self.angle_distances_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'error_signal', 10)
        self.get_logger().info('AngleDistancesReader node has been started.')

        # Initialize variables for plotting
        self.error_history = []
        self.alpha_history = []
        self.distance_history_75 = []
        self.distance_history_90 = []
        self.distance_history_105 = []
        
        plt.ion()  # Enable interactive mode for real-time plotting
        self.fig, self.axs = plt.subplots(3, 1, figsize=(8, 8))

    def angle_distances_callback(self, msg):
        self.get_logger().info('Received angle distances message.')
        
        # Use a specific value from distances_front (e.g., middle index) for the check
        if msg.distances_front[5] > 0.5:  # Using the middle front distance for the check
            # Use the right-side distances (75°, 90°, 105°)
            if len(msg.distances_right) >= 3:
                d_75 = msg.distances_right[2]
                d_90 = msg.distances_right[1]
                d_105 = msg.distances_right[0]
                self.get_logger().info(f'd_75: {d_75}, d_90: {d_90}, d_105: {d_105}')
                CD, alpha = self.calculate_CD_distance(d_75, d_105,d_90)
                self.get_logger().info(f'CD: {CD}, alpha: {alpha}')
                error = self.calculate_error(CD, alpha, d_90)
                
                if error is not None:
                    #self.get_logger().info(f'error_signal: {error}')
                    self.publish_error(error)
                    
                    # Update plot data
                    self.update_plot(d_75, d_90, d_105, error, alpha)
                else:
                    self.get_logger().warn('Could not calculate error signal.')
            else:
                self.get_logger().warn('Not enough distances_right values to calculate CD distance.')
        else:
            # Correct the call to publish_error (without passing self)
            self.publish_error(-1.0)

    def calculate_CD_distance(self, d_75, d_105,d_90):
        angle_diff = math.radians(30)  # Difference between 75° and 105°

        # Ensure the distances are reasonable and non-zero
        if d_75 > 0 and d_105 > 0:
            try:
                # Calculate the angle alpha
                alpha = math.atan2(d_75 - d_105, 0.3)  # Approximate distance between points in the X-axis
                # Calculate the distance CD to the wall as the average of d_75 and d_105
                CD = (d_75 + d_105) / 2
                #self.get_logger().info(f'Calculated alpha: {alpha} radians, {math.degrees(alpha)} degrees, CD distance: {CD}')
                return CD, alpha
            except ZeroDivisionError:
                self.get_logger().error('ZeroDivisionError: Cannot calculate alpha.')
                return None, None
        else:
            return 0, 0.1  # Default values if measurements are erroneous

    def calculate_error(self, CD, alpha, d_90):
        target_distance = 0.5  # Desired distance from the wall
        error = (target_distance - CD)
        #self.get_logger().info(f'Calculated error: {error}')
        return error

    def publish_error(self, error):
        msg = Float32()
        msg.data = error
        self.publisher_.publish(msg)
        self.get_logger().info('Published error to error_signal topic.')

    def update_plot(self, d_75, d_90, d_105, error, alpha):
        # Store data points in history
        self.distance_history_75.append(d_75)
        self.distance_history_90.append(d_90)
        self.distance_history_105.append(d_105)
        self.error_history.append(error)
        self.alpha_history.append(alpha)

        # Clear previous plots
        for ax in self.axs:
            ax.cla()

        # Plot distances
        self.axs[0].plot(self.distance_history_75, label="d_75 (75°)", color='blue')
        self.axs[0].plot(self.distance_history_90, label="d_90 (90°)", color='green')
        self.axs[0].plot(self.distance_history_105, label="d_105 (105°)", color='red')
        self.axs[0].set_ylabel('Distance (m)')
        self.axs[0].legend()

        # Plot error signal
        self.axs[1].plot(self.error_history, label="Error", color='orange')
        self.axs[1].set_ylabel('Error Signal')
        self.axs[1].legend()

        # Plot alpha (angle)
        self.axs[2].plot([math.degrees(a) for a in self.alpha_history], label="Alpha (degrees)", color='purple')
        self.axs[2].set_ylabel('Alpha (degrees)')
        self.axs[2].legend()

        # Draw updated plots
        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    angle_distances_reader = AngleDistancesReader()
    rclpy.spin(angle_distances_reader)
    angle_distances_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
