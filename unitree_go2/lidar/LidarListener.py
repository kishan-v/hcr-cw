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


'''Base Class for Transforms'''
class PostProcessingTransform:
    def __init__(self, transform_name):
        self.transform_name = transform_name
    
    @abstractmethod
    def apply(self, data, local_data = None):
        pass



''' Find the midpoint of the global data, and center the global map around this'''
class RemoveOffset(PostProcessingTransform):
    def __init__(self, v_max):
        super().__init__("remove offset")
        self.initialised = False

        self.v_max = v_max

        # TODO: fix this hacky way of recording time
        self.sum_time = 0
        self.last_time = time.time()
        self.ctr = 0

        self.x_k = np.zeros((1,3))

       
    def apply(self, data, local_data = None):
        if self.initialised:
            # calculate avg time
            self.sum_time += time.time() - self.last_time
            self.last_time = time.time() 
            self.ctr += 1

            print("hallo: ", self.sum_time / self.ctr, " - ", time.time())
            # calculate max radius moved
            max_moved = max(self.sum_time * self.v_max / self.ctr, 1)
            
            # filter everything outside the last radius
            filt = RangeFilter(0, max_moved, 0, max_moved)
            centroid_data = local_data[:, :3] - self.x_k 
            centroid_data = filt.apply(centroid_data)
        else:
            centroid_data = local_data
            self._initialised = True

        # estimate the new centroid
        self.x_k = np.median(centroid_data[:, :3], axis=0)
        
    
        out = data
        out[:, :3] -= self.x_k 
        
        return out 


'''Remove all points within a certain range'''
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

        return np.logical_and(x_mask, y_mask)

    def apply(self, data, local_data = None):
        mask = self.create_filter(data) 
        return data[~mask]


'''Convert discrete cartesian points into an occupancy grid'''
class ConvertToOccupancyGrid(PostProcessingTransform):
    def __init__(self, x_width, y_width, z_width, voxel_width):
        super().__init__("occupancy converter")

        self.x_width = x_width
        self.y_width = y_width
        self.z_width = z_width

        self.voxel_width = voxel_width

    def apply(self, data, local_data = None):
        # rounded_data = np.around(data, decimals = 1)
        rounded_data = (data[:, :3] / self.voxel_width).astype(int)
        # print(f"Got max: {np.max(rounded_data)} with size: {rounded_data.shape}")

        voxel_centroids = np.unique(rounded_data, axis=0)
        # print(f"voxel_cent: {voxel_centroids.shape}")

        # normalise
        min = np.min(voxel_centroids, axis=0).reshape((1,3))
        # print(f"sizes: {rounded_data.shape}, {voxel_centroids.shape}, {min.shape}")
        voxel_centroids -= min.reshape((1,3))
        # print(f"sizes: {rounded_data.shape}, {voxel_centroids.shape}, {min.shape}")

        # print(f"max: {np.max(voxel_centroids)}")


        # create grid
        grid = np.zeros((self.x_width, self.y_width, self.z_width), dtype=bool)
        
        # populate_grid
        grid[voxel_centroids] = True

        return grid



class PostProcessingPipeline:
    def __init__(self):
        self.transforms = []
    
    def add_transform(self, transform):
        assert issubclass(type(transform), PostProcessingTransform)
        self.transforms.append(transform)

    def apply(self, data, local_data=None):
        for transform in self.transforms:
            data = transform.apply(data, local_data=local_data)

        return data


class LidarProcessor(Node):
    def __init__(self):
        super().__init__("lidar_processor")

        self.last_time = time.time()
        self.time_diff = 0
        self.ctr = 0

        # subscribe to the topic
        self.subscriber = self.create_subscription(
                PointCloud2,
                '/utlidar/cloud_deskewed',
                self.lidar_callback,
                10)

        # initialise empty data
        self.data = np.empty((4, 0))
        self.user_data = np.empty((3, 0))

        # initialise pipeline
        self.pipeline = PostProcessingPipeline()
        self.voxel_pipeline = PostProcessingPipeline()

        self.pipeline.add_transform(RemoveOffset(20 / (60 * 60)))
        # self.pipeline.add_transform(RangeFilter(0.2, 5.0, 0.2, 5.0))

        self.voxel_pipeline.add_transform(ConvertToOccupancyGrid(1001, 1001, 3, 0.1))

        self.movement = np.zeros((1, 3))

        self.point_limit = 10000

        self.occupancy_grid = np.empty((3, 0))

    # callback for receiving lidar data
    def lidar_callback(self, msg):

        self.get_logger().info(f"Received PointCloud2 message with {len(msg.data)} bytes.")

        # process lidar data
        self.process_lidar_data(msg.data)

    # process lidar data from raw into structured format
    def process_lidar_data(self, raw_data):
        # extract points, type fp32
        original_data = np.frombuffer(raw_data, dtype=np.float32)
        
        # reshape the data into [[x, y, z, intensity]]
        original_data = original_data.reshape(-1, 4)
        #test movement
        # original_data[:, :3] = original_data[:, :3] + self.movement
        # self.movement += np.random.uniform(-5, 5, size=(1, 3))

        # 1. add all data to the current map
        data = np.append(self.data, original_data.transpose(), axis=1)

        #remove first n points
        if data.shape[1] > self.point_limit:
            remove = data.shape[1]-self.point_limit
            self.data = data[:, remove:]
        else:
            self.data = data


        # print(f"shape of data {self.data.shape}")

        # get the current centroid
        self.user_data = self.pipeline.apply(np.copy(self.data).transpose(), original_data).transpose()
        # print(f"shape of user data {self.user_data.shape}")

        self.occupancy_grid = self.voxel_pipeline.apply(self.user_data.transpose())
        # print(f"Occupancy: {self.occupancy_grid.shape}, with data: {self.occupancy_grid}")


# live plot data
def plot_data(node):
    # interactive plot
    plt.ion()

    # plot settings
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    sc = ax.scatter([], [], [], c=[], cmap='viridis', marker='o', s=1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Live Lidar Point Cloud')

    ax.set_xlim3d(-5, 5)
    ax.set_ylim3d(-5, 5)
    ax.set_zlim3d(-5, 5)

    # read from the node
    while rclpy.ok():
        if node.user_data.shape[0] > 0:
            # extract data
            x, y, z, intensity = node.user_data

            # update scatter plot
            sc._offsets3d = (x, y, z)

            # set the colour based on intensity
            sc.set_array(intensity)

            # draw the updates
            plt.draw()
        
        plt.pause(0.5)

def plot_voxels(node):
    # Turn on interactive mode
    plt.ion()  # Enable interactive mode

    # Set up the 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Create an initially empty scatter plot for the voxel centers
    sc = ax.scatter([], [], [], c=[], cmap='viridis', marker='o', s=1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Live Voxel Grid Center Points')

    ax.set_xlim3d(0, 500)
    ax.set_ylim3d(0, 500)
    ax.set_zlim3d(0, 5)

    # Main loop: update the plot as long as rclpy is OK
    while rclpy.ok():
        # Make sure the occupancy grid exists and has data
        if node.occupancy_grid is not None and node.occupancy_grid.size > 0:
            # Find the indices of all occupied voxels (True values)
            idx_x, idx_y, idx_z = np.nonzero(node.occupancy_grid)
            if idx_x.size > 0:
                # Assuming each voxel spans 1 unit, adding 0.5 gives the center.
                x = idx_x + 0.5
                y = idx_y + 0.5
                z = idx_z + 0.5
                print(f"{idx_x.shape}")


                print(f"{x[0]}, {y[0]}, {z[0]}")

                # Update the scatter plot with the new center points
                sc._offsets3d = (x, y, z)
                sc.set_array(1)
                plt.draw()

        plt.pause(0.5) 

def print_data(node):
    while rclpy.ok():
        time.sleep(0.5)
        print(node.occupancy_grid.shape)

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description="CLI for the Lidar Processor")

    arg_parser.add_argument('--print_data', action="store_true", help="Print LIDAR Data into the console")
    arg_parser.add_argument('--plot_data', action="store_true", help="3D plot of Lidar Data using Matplotlib")
    arg_parser.add_argument('--plot_voxels', action="store_true", help="3D plot of Occupancy Grid")

    args = arg_parser.parse_args()

    # init
    rclpy.init()
    lidar_processor = LidarProcessor()

    ros_thread = threading.Thread(target=rclpy.spin, args=(lidar_processor,), daemon=True)
    ros_thread.start()

    if args.print_data and args.plot_data and args.plot_voxels:
        print("Error: either print or plot")
        quit()

    if args.print_data:
        print_data(lidar_processor)

    if args.plot_data:
        plot_data(lidar_processor)

    if args.plot_voxels:
        plot_voxels(lidar_processor)

    input("Press any key to quit")

    lidar_processor.destroy_node()
    rclpy.shutdown()
