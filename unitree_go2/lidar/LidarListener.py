import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String 
from nav_msgs.msg import Odometry

import numpy as np
import matplotlib

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import threading
import time
import argparse
import json

from abc import abstractmethod

# 5m x 5 x 2 = 50m^3
# 0.1 * 0.1 * 0.1 = 0.001
# total number of cubes -> 125000
# 1 bit per item in the array
# 50000bits -> 50000/8000 = 6.25kb per box 

# formula for compressing box:
#   amount per row = width / size
#   amount per col = depth / size
#   amount per height = height / size
#   row_offset = x / step_size
#   col_offset = (y / step_size) * amount_per_row
#   height_offset = (z / step_size) * (amount_per_col * amount_per_height)
# idx = height_offset + col_offset + row_offset

def serialise_occupancy_grid(data, step_size=0.10)->str:
    json_data = {
        "world_dims": {
            "width": 4,
            "depth": 4,
            "height": 2,
            "step_size": step_size,
        },
        "timestamp": int(time.time()),
        "box_vals": data,
    }

    return json.dumps(json_data)


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

            # calculate max radius moved
            max_moved = max(self.sum_time * self.v_max / self.ctr, 1)
            
            # filter everything outside the last radius
            filt = RangeFilter(0, max_moved, 0, max_moved)
            centroid_data = local_data[:, :3] - self.x_k 
            centroid_data = filt.apply(centroid_data)
        else:
            centroid_data = local_data
            self.initialised = True

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

        lower_bound_filter = np.logical_and(abs(data[:, 0]) < self.x_min, abs(data[:, 1]) < self.y_min)
        upper_bound_filter = np.logical_or(abs(data[:, 0]) > self.x_max, abs(data[:, 1]) >self.y_max)
        z_mask = np.logical_or(data[:, 2] < -0.31, data[:,2] > (-0.31 + 2))

        mask = np.logical_or(lower_bound_filter, upper_bound_filter)
        return np.logical_or(mask, z_mask)

    def apply(self, data, local_data = None):
        mask = self.create_filter(data) 
        return data[~mask]



'''Convert discrete cartesian points into an occupancy grid'''
class ConvertToOccupancyGrid(PostProcessingTransform):
    def __init__(self, x_width, y_width, z_width, step_size):
        super().__init__("occupancy converter")

        self.x_width = x_width
        self.y_width = y_width
        self.z_width = z_width

        self.num_rows = x_width / step_size
        self.num_cols = y_width / step_size
        self.num_heights = z_width / step_size

        self.step_size = step_size 

    ''' Convert the cartesian coordinates to grid index '''
    def cartesian_to_grid_idx(self, x, y, z):
       row_offset = x / self.step_size 
       col_offset = (y / self.step_size) * self.num_rows
       height_offset = (z / self.step_size) * (self.num_rows * self.num_cols)

       idx = row_offset + col_offset + height_offset
        
       # assert idx <= self.num_rows * self.num_cols * self.num_heights

       return np.asarray(idx, dtype=int)


    def rounding_policy(self, data, dist):
         # TODO: investigate if this also works in cartesian space
         return (data + dist // 2) // dist * dist 


    def apply(self, data, local_data = None):
        # rounded_data = np.around(data, decimals = 1)

        # downsample by rounding
        # rounded_data = (data[:, :3] / self.step_size).astype(int)
        data = data[:, :3]
        data = self.rounding_policy(data, self.step_size)
        data = np.unique(data, axis=0)

        # recenter the data
        data[:, 0] += self.x_width / 2
        data[:, 1] += self.y_width / 2
        data[:, 2] += self.z_width / 2

        idx_arrays = self.cartesian_to_grid_idx(data[:, 0], data[:, 1], data[:, 2])

        # # create grid
        grid = np.zeros(int(self.x_width * self.y_width * self.z_width / (self.step_size ** 3)), dtype=bool)
        
        # # populate_grid
        grid[idx_arrays] = True

        return grid 


'''Convert discrete cartesian points into an occupancy grid'''
class RoundData(PostProcessingTransform):
    def __init__(self, x_width, y_width, z_width, step_size):
        super().__init__("occupancy converter")

        self.x_width = x_width
        self.y_width = y_width
        self.z_width = z_width

        self.num_rows = x_width / step_size
        self.num_cols = y_width / step_size
        self.num_heights = z_width / step_size

        self.step_size = step_size 

    ''' Convert the cartesian coordinates to grid index '''
    def cartesian_to_grid_idx(self, x, y, z):
       row_offset = x / self.step_size 
       col_offset = (y / self.step_size) * self.num_rows
       height_offset = (z / self.step_size) * (self.num_rows * self.num_cols)

       idx = row_offset + col_offset + height_offset
        
       # assert idx <= self.num_rows * self.num_cols * self.num_heights

       return np.asarray(idx, dtype=int)


    def rounding_policy(self, data, dist):
         # TODO: investigate if this also works in cartesian space
         return (data + dist // 2) // dist * dist 

    def apply(self, data, local_data = None):
        # rounded_data = np.around(data, decimals = 1)

        # downsample by rounding
        # rounded_data = (data[:, :3] / self.step_size).astype(int)
        data = data[:, :3]
        data = self.rounding_policy(data, self.step_size)
        data = np.unique(data, axis=0)

        # recenter the data
        # data[:, 0] += self.x_width / 2
        # data[:, 1] += self.y_width / 2
        # data[:, 2] += self.z_width / 2

        return data 


def test_data_recovery(data):
    data = json.loads(data)

    x_area = data["world_dims"]["width"] * (1 / data["world_dims"]["step_size"])
    y_area = data["world_dims"]["depth"] * (1 / data["world_dims"]["step_size"])

    y_area *= x_area

    step_size = data["world_dims"]["step_size"]

    compressed_data = np.array(data["box_vals"])

    indices = np.where(compressed_data)[0]

    # reverse the calculation
    
    #z = floor(index / z_width)
    #y = floor((index % z_width) / y_width)
    #x = (((index % z_width) % y_width) / x_width) (assert its an int)

    z = np.floor(indices / y_area)
    y = np.floor((indices % y_area) / x_area)
    x = np.floor((indices % y_area) % x_area)

    x *= data["world_dims"]["step_size"]
    y *= data["world_dims"]["step_size"]
    z *= data["world_dims"]["step_size"]

    x -= data["world_dims"]["width"] / 2
    y -= data["world_dims"]["depth"] / 2
    z -= data["world_dims"]["height"] / 2

    return np.array([x, y, z])


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
        self.pc_subscriber = self.create_subscription(
                PointCloud2,
                '/utlidar/cloud_deskewed',
                self.lidar_callback,
                10)

        self.odom_subscriber = self.create_subscription(
                Odometry,
                '/utlidar/robot_odom',
                self.odom_callback,
                10
            )
        
        self.recovered = np.empty((3, 0))

        self.occupancy_pub = self.create_publisher(String, "/occupancy_grid", 10)

        # initialise empty data
        self.data = np.empty((4, 0))
        self.user_data = np.empty((3, 0))

        # initialise pipeline
        self.pipeline = PostProcessingPipeline()
        self.voxel_pipeline = PostProcessingPipeline()
        self.round_pipeline = PostProcessingPipeline()

        # self.pipeline.add_transform(RemoveOffset(20 / (60 * 60)))
        self.pipeline.add_transform(RangeFilter(0.2, 2.0, 0.2, 2.0))

        self.voxel_pipeline.add_transform(ConvertToOccupancyGrid(4, 4, 2, 0.1))
        self.round_pipeline.add_transform(RoundData(4, 4, 2, 0.1))

        self.movement = np.zeros((1, 3))

        self.point_limit = 15000

        self.occupancy_grid = np.empty((3, 0))

        self.offset = np.zeros(3)
        self.dir = 0


    """ process odometry data """
    def odom_callback(self, msg):
        # self.get_logger().info(f"Received Odometry message")# with pose: {msg.pose.pose.position} bytes.")
        pos = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.offset = np.array([pos.x, pos.y, pos.z])

        d = np.array([2 * (orientation.x * orientation.z + orientation.y * orientation.w), 2 * (orientation.y * orientation.z - orientation.x * orientation.w), 1 - 2 * (orientation.x **2 + orientation.y **2)])

        # theta = np.arccos(dot / norm_d)
        theta = np.arctan2(d[1] , d[0])


    """ process lidar data """
    def lidar_callback(self, msg):
        # self.get_logger().info(f"Received PointCloud2 message with {len(msg.data)} bytes.")

        # process lidar data
        self.process_lidar_data(msg.data)

    # process lidar data from raw into structured format
    def process_lidar_data(self, raw_data):
        # extract points, type fp32
        original_data = np.frombuffer(raw_data, dtype=np.float32)
        
        # reshape the data into [[x, y, z, intensity]]
        original_data = original_data.reshape(-1, 4)

        # add all data to the current map
        data = np.append(self.data, original_data.transpose(), axis=1)

        #remove first n points
        if data.shape[1] > self.point_limit:
            remove = data.shape[1]-self.point_limit
            self.data = data[:, remove:]
        else:
            self.data = data


        # get the current centroid and center the data
        usr_data = np.copy(self.data).transpose()
        usr_data[:, :3] -= self.offset

        # apply data processing pipeline
        self.user_data = self.pipeline.apply(usr_data, original_data).transpose()
        self.occupancy_grid = self.voxel_pipeline.apply(self.user_data.transpose())
        self.rounded_data = self.round_pipeline.apply(self.user_data.transpose())

        # publish to topic
        ros_msg = String()
        ros_msg.data = serialise_occupancy_grid(self.occupancy_grid.tolist())
        with open("test_data.txt", "w") as file:
            file.write(serialise_occupancy_grid(self.occupancy_grid.tolist()))
        self.recovered = test_data_recovery(ros_msg.data)
        self.occupancy_pub.publish(ros_msg)



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

    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)

    # read from the node
    while rclpy.ok():
        if node.user_data.shape[1] > 0:
            # extract data
            x, y, z = node.recovered

            # update scatter plot
            sc._offsets3d = (x, y, z)

            # set the colour based on intensity
            sc.set_array(100)

            # draw the updates
            plt.draw()
        
        plt.pause(0.5)



def plot_voxels(node):
    plt.ion()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Live Lidar Voxel Grid')

    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)

    cube_size = 0.1

    while rclpy.ok():
        if node.user_data.shape[1] > 0:
            x, y, z = node.recovered  
            ax.collections.clear()

            # draw a cube
            def draw_cube(ax, center, size, color):
                x, y, z = center
                d = size / 2

                # cube vertices
                vertices = [
                    [(x-d, y-d, z-d), (x+d, y-d, z-d), (x+d, y+d, z-d), (x-d, y+d, z-d)],  # bottom
                    [(x-d, y-d, z+d), (x+d, y-d, z+d), (x+d, y+d, z+d), (x-d, y+d, z+d)],  # top
                    [(x-d, y-d, z-d), (x-d, y+d, z-d), (x-d, y+d, z+d), (x-d, y-d, z+d)],  # left
                    [(x+d, y-d, z-d), (x+d, y+d, z-d), (x+d, y+d, z+d), (x+d, y-d, z+d)],  # right
                    [(x-d, y-d, z-d), (x+d, y-d, z-d), (x+d, y-d, z+d), (x-d, y-d, z+d)],  # front
                    [(x-d, y+d, z-d), (x+d, y+d, z-d), (x+d, y+d, z+d), (x-d, y+d, z+d)],  # back
                ]

                # Create the cube
                cube = Poly3DCollection(vertices, facecolors=color, edgecolors='black', alpha=0.8)
                ax.add_collection3d(cube)

            color = plt.cm.viridis(100)

            # draw the cubes
            for xi, yi, zi in zip(x, y, z):
                draw_cube(ax, (xi, yi, zi), cube_size, color)

            # redraw the visible plot
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
