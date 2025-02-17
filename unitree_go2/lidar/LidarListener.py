import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

import threading
import time
import argparse

from abc import abstractmethod

class PostProcessingTransform:
    def __init__(self, transform_name):
        self.transform_name = transform_name
    
    @abstractmethod
    def apply(self, data):
        pass

class RangeFilter(PostProcessingTransform):
    def __init__(self, x_min, x_max, y_min, y_max):
        super().__init__("range filter")

        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

    def create_filter(self, data):
        x_mask = np.logical_or(abs(data[:, 0]) < self.x_min,
               abs(data[:, 0]) > self.x_max)
        y_mask = np.logical_or(abs(data[:, 1]) < self.y_min,
               abs(data[:, 1]) > self.y_max)

        return np.logical_or(x_mask, y_mask)

    def apply(self, data):
        mask = self.create_filter(data) 
        return data[~mask]


class PostProcessingPipeline:
    def __init__(self):
        self.transforms = []
    
    def add_transform(self, transform):
        assert issubclass(type(transform), PostProcessingTransform)
        self.transforms.append(transform)

    def apply(self, data):
        for transform in self.transforms:
            data = transform.apply(data)

        return data


class LidarProcessor(Node):
    def __init__(self):
        super().__init__("lidar_processor")

        # subscribe to the topic
        self.subscriber = self.create_subscription(
                PointCloud2,
                '/utlidar/cloud_deskewed',
                self.lidar_callback,
                10)

        # initialise empty data
        self.data = np.empty((0, 4))

        # initialise pipeline
        self.pipeline = PostProcessingPipeline()

        self.pipeline.add_transform(RangeFilter(0.5, 1.0, 0.5, 1.0))

    # callback for receiving lidar data
    def lidar_callback(self, msg):
        # self.get_logger().info(f"Received PointCloud2 message with {len(msg.data)} bytes")

        # process lidar data
        self.process_lidar_data(msg.data)

    # process lidar data from raw into structured format
    def process_lidar_data(self, raw_data):
        # extract points, type fp32
        data = np.frombuffer(raw_data, dtype=np.float32)
        
        # reshape the data into [[x, y, z, intensity]]
        data = data.reshape(-1, 4)
        
        # apply transformation pipeline
        data = self.pipeline.apply(data)

        # convert into the required shape
        self.data = data.transpose()

# live plot data
def plot_data(node):
    # interactive plot
    plt.ion()

    # plot settings
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    sc = ax.scatter([], [], [], c=[], cmap='viridis', marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Live Lidar Point Cloud')

    ax.set_xlim3d(-5, 5)
    ax.set_ylim3d(-5, 5)
    ax.set_zlim3d(-5, 5)

    # read from the node
    while rclpy.ok():
        if node.data.shape[0] > 0:
            # extract data
            x, y, z, intensity = node.data

            # update scatter plot
            sc._offsets3d = (x, y, z)

            # set the colour based on intensity
            sc.set_array(intensity)

            # draw the updates
            plt.draw()
        
        plt.pause(0.5)

def print_data(node):
    while rclpy.ok():
        time.sleep(0.5)
        print(node.data)

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description="CLI for the Lidar Processor")

    arg_parser.add_argument('--print_data', action="store_true", help="Print LIDAR Data into the console")
    arg_parser.add_argument('--plot_data', action="store_true", help="3D plot of Lidar Data using Matplotlib")

    args = arg_parser.parse_args()

    # init
    rclpy.init()
    lidar_processor = LidarProcessor()

    ros_thread = threading.Thread(target=rclpy.spin, args=(lidar_processor,), daemon=True)
    ros_thread.start()

    if args.print_data and args.plot_data:
        print("Error: either print or plot")
        quit()

    if args.print_data:
        print_data(lidar_processor)

    if args.plot_data:
        plot_data(lidar_processor)

    input("Press any key to quit")

    lidar_processor.destroy_node()
    rclpy.shutdown()
