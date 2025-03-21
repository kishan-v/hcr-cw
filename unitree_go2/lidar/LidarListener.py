import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import numpy as np
import matplotlib

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import threading
import time
import argparse
import json

from abc import abstractmethod

## Helper functions ##

''' Serialise data to an occupancy grid '''
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


''' Test Data Recovery (from occupancy grid) '''
def test_data_recovery(data):
    data = json.loads(data)

    x_area = data["world_dims"]["width"] * (1 / data["world_dims"]["step_size"])
    y_area = data["world_dims"]["depth"] * (1 / data["world_dims"]["step_size"])

    y_area *= x_area

    step_size = data["world_dims"]["step_size"]

    compressed_data = np.array(data["box_vals"])

    indices = np.where(compressed_data)[0]

    # reverse the calculation
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

# formula for compressing box:
#   amount per row = width / size
#   amount per col = depth / size
#   amount per height = height / size
#   row_offset = x / step_size
#   col_offset = (y / step_size) * amount_per_row
#   height_offset = (z / step_size) * (amount_per_col * amount_per_height)
# idx = height_offset + col_offset + row_offset



## Data Transforms ##

'''Base Class for Transforms'''
class PostProcessingTransform:
    def __init__(self, transform_name):
        self.transform_name = transform_name

    @abstractmethod
    def apply(self, data, extra_params = None):
        pass


''' Subtract known midpoint from the data '''
class RemoveKnownOffset(PostProcessingTransform):
    def __init__(self):
        super().__init__("remove_known_offset")

    def apply(self, data, extra_params = None):
        assert "Offset" in extra_params.keys()

        return data[:, :3] - extra_params["Offset"]


''' Rotate data along the XY Plane '''
class RemoveAngularOffset(PostProcessingTransform):
    def __init__(self):
        super().__init__("RemoveAngularOffset")

    def apply(self, data, extra_params = None):
        assert "OffsetAngleRadians" in extra_params.keys()

        # rotate back towards the origin (i.e undo the rotation)
        angle = -extra_params["OffsetAngleRadians"]

        # generate rotation matrix
        rot_mat = np.array([
            [np.cos(angle), np.sin(angle)],
            [-np.sin(angle), np.cos(angle)]
        ])

        # perform rotation
        data[:, :2] = data[:, :2] @ rot_mat.T

        return data


''' Find the midpoint of the global data, and center the global map around this'''
class RemoveOffsetNaive(PostProcessingTransform):
    # Note: now deprecated
    def __init__(self, v_max):
        super().__init__("remove offset")
        self.initialised = False

        self.v_max = v_max

        self.sum_time = 0
        self.last_time = time.time()
        self.ctr = 0

        self.x_k = np.zeros((1,3))


    def apply(self, data, extra_params = None):
        local_data = extra_params["local_data"]
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

    def apply(self, data, extra_params = None):
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


        return np.asarray(idx, dtype=int)

    def apply(self, data, extra_params=None):
        # recenter the data
        data[:, 0] += self.x_width / 2
        data[:, 1] += self.y_width / 2
        data[:, 2] -= np.min(data[:, 2])

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

    def rounding_policy(self, data, dist):
         return (data + dist // 2) // dist * dist 

    def apply(self, data, extra_params = None):
        # downsample by rounding
        data = data[:, :3]
        data = self.rounding_policy(data, self.step_size)
        data = np.unique(data, axis=0)

        return data



## Pipeline Class ##

class PostProcessingPipeline:
    def __init__(self):
        self.transforms = []

    def add_transform(self, transform):
        assert issubclass(type(transform), PostProcessingTransform)
        self.transforms.append(transform)

    def apply(self, data, extra_params=None):
        for transform in self.transforms:
            data = transform.apply(data, extra_params=extra_params)

        return data


## LidarProcessor class ##

class LidarProcessor(Node):
    def __init__(self):
        super().__init__("lidar_processor")

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

        self.occupancy_pub = self.create_publisher(String, "/occupancy_grid", 10)

        # initialise class data state (to store between updates)
        self.data = np.empty((4, 0))

        # store post_processed_data separately for quick debugging
        self.post_processed_data = np.empty((3, 0))

        # initialise main post-processing pipeline
        self.pipeline = PostProcessingPipeline()
        self.pipeline.add_transform(RemoveKnownOffset())
        self.pipeline.add_transform(RangeFilter(0.2, 2.0, 0.2, 2.0))
        # self.pipeline.add_transform(RemoveAngularOffset())

        # initialise voxelisation pipeline
        self.voxel_pipeline = PostProcessingPipeline()
        self.voxel_pipeline.add_transform(RoundData(4, 4, 2, 0.1))
        self.voxel_pipeline.add_transform(ConvertToOccupancyGrid(4, 4, 2, 0.1))

        # data history storage limit
        self.point_limit = 15000

        # occupancy_grid state
        self.occupancy_grid = np.empty((3, 0))

        # lidar offset (relative to the dog's starting position)
        self.offset = np.zeros(3)

        # dog_rotation (relative to the dog's starting orientation)
        self.dog_rotation = 0


    """ process odometry data """
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.offset = np.array([pos.x, pos.y, pos.z])

        d = np.array([2 * (orientation.x * orientation.z + orientation.y * orientation.w), 1 - 2 * (orientation.x **2 + orientation.z **2)])

        self.dog_rotation = np.arctan2(d[0], d[1])

        print(f"Dog Rotation: {self.dog_rotation}")


    """ process lidar data """
    def lidar_callback(self, msg):
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

        # apply data processing pipeline
        self.post_processed_data = self.pipeline.apply(self.data.transpose().copy(), extra_params = {
            "Offset": self.offset,
            "OffsetAngleRadians": self.dog_rotation,
            }).transpose()
        self.occupancy_grid = self.voxel_pipeline.apply(self.post_processed_data.transpose())

        # publish to occupancy topic (internal)
        ros_msg = String()
        ros_msg.data = serialise_occupancy_grid(self.occupancy_grid.tolist())
        self.occupancy_pub.publish(ros_msg)



## Data Visualisation Options ##

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
        if node.post_processed_data.shape[1] > 0:
            # extract data
            x, y, z = node.post_processed_data#test_data_recovery(serialise_occupancy_grid(node.occupancy_grid.tolist()))

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
        if node.post_processed_data.shape[1] > 0:
            x, y, z = test_data_recovery(serialise_occupancy_grid(node.occupancy_grid.tolist()))
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


## Entrypoint ##

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

    if args.plot_voxels or args.plot_data:
        matplotlib.use("TkAgg")

    if args.print_data:
        print_data(lidar_processor)

    if args.plot_data:
        plot_data(lidar_processor)

    if args.plot_voxels:
        plot_voxels(lidar_processor)

    input("Press any key to quit")

    lidar_processor.destroy_node()
    rclpy.shutdown()
