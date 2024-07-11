#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Joy 

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        joy_topic = "/joy"

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)

        self.subscription_laser = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            10
        )

        self.subscription_joy = self.create_subscription(
            Joy, joy_topic, self.joy_callback, qos
        )

        self.publisher_ = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

        self.drive_speed = 0.5 # set to drive slower while running tests
        self.safe = False

    def joy_callback(self, msg):
        print(f"Button states: {msg.buttons}")
        deadman = 0  # current index of desired deadmann button
        if msg.buttons[deadman] == 1:  # is pressed
            self.safe = True
        else:
            self.safe = False
        

    def change_car_behavior(self, best_angle, best_dist):
        """ uses the angle of the best direction """
        drive_msg = AckermannDriveStamped()
        # pretty sure angles to the right are negative
           
        max_steering_angle = np.radians(40)
        control_effort = np.clip(best_angle, -max_steering_angle, max_steering_angle)
        drive_msg.drive.steering_angle = control_effort
        # print(best_angle)
        drive_msg.drive.speed = best_dist*self.drive_speed
        if not self.safe:
            drive_msg.drive.speed = 0.0
            # for now still calculates everything, just doesnt move
            # could change later
        self.publisher_.publish(drive_msg)

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        window_size = 5
        max_distance = 3  # meters
        smoothed_ranges = np.convolve(ranges, np.ones(window_size)/window_size, mode="same")
        proc_ranges = np.where(smoothed_ranges > max_distance, max_distance, smoothed_ranges)
        return proc_ranges
    

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        if free_space_ranges is None:
            return None, None
        
        # find max gap implementation is currently the problem, 
        # would likely be better if the gap is a "static" value, 
        # for example, things beyond a certain depth are just considered gaps

        min_gap_range = 1.5  # 
        current_gap_length = 1
        max_gap_length = 1

        # create an index list
        # print(free_space_ranges)
        gap_indices = [i for i, distance in enumerate(free_space_ranges) if distance >=min_gap_range]
        max_gap_start = gap_indices[0]
        max_gap_end = gap_indices[0]
        current_gap_start = gap_indices[0]

        for i in range(1, len(gap_indices)):
            if gap_indices[i] == gap_indices[i-1]+1:
                current_gap_length +=1
            else:
                if current_gap_length > max_gap_length:
                    max_gap_length = current_gap_length
                    max_gap_start = current_gap_start
                    max_gap_end = gap_indices[i-1]
                current_gap_start = gap_indices[i]
                current_gap_length=1
        if current_gap_length > max_gap_length:
            max_gap_start = current_gap_start
            max_gap_end = gap_indices[-1]
        return max_gap_start, max_gap_end

    


    # TODO - NEED TO WORK ON THIS SECTION, RUNS INTO THE SIDES A LOT AT LOWER SPEEDS ,
    # should instead check for the position of the closest point! 
    
    def find_best_point(self, start_i, end_i, ranges=None):
        # I don't think ranges is needed so I won't set it a value
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        # for now (somewhat naive), just choose the middle of the valid point range
        if start_i is not None and end_i is not None:
            before_d = ranges[max(0, start_i-1)]
            after_d = ranges[min(end_i+1, len(ranges)-1)]
            side_diff = abs(before_d - after_d)
            min_dist = .15
            ratio = 1/3  # decides how extreme to make the adjustment 
            needed = bool(before_d < 1 or after_d < 1)
            # needed=True
            #  first check to see if there is a noticeably closer one
            if (side_diff > min_dist) and needed:   
                # gotta vear more to one side
                if (before_d < after_d):
                    # prioritize turning towards left
                    # remember indexing goes counterclockwise
                    # better to make it proportional to the difference
                    point_index = min((start_i + end_i)//2 + (abs(start_i - end_i)//2) + int(min_dist/side_diff), end_i)
                    # is side_diff/min_dist better than min_dist/side_diff?     + int(min_dist/side_diff)
                    # might be better to say something like... how much of the side difference is from this side? so before_d//side_diff #int((side_diff/min_dist)*
                    # print(f"{side_diff/ratio=}")
                else:
                    # prioritize turning right
                    point_index = max((start_i + end_i)//2 - (abs(start_i - end_i)//2) - int(min_dist/side_diff), start_i) # int((side_diff/min_dist)*  - int(min_dist/side_diff)
            else:
                # pretty much the same, just go for the middle
                point_index = (start_i + end_i)//2
            return point_index
        return None
    
    def find_closest_point(self, ranges):
        """ finds the closest point and returns its index"""
        return min(enumerate(ranges), key=lambda x:x[1])

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # finds the closest point to the lidar and establishes bubble
        angle_increment = data.angle_increment
        angle_min = data.angle_min
        closest_point_index, closest_point_range = self.find_closest_point(proc_ranges)
        closest_point_angle = (closest_point_index * angle_increment) + angle_min
        # print(f"{closest_point_angle=}")
        bubble_radius = .15 # 20 cm
        angle_dif = np.arctan(bubble_radius/closest_point_range)
        # print(f"{closest_point_range=}")
        bubble_zone_index = \
            (max(int(((closest_point_angle-angle_dif)-angle_min)/angle_increment), 0),
            min(int(((closest_point_angle+angle_dif)-angle_min)/angle_increment), len(proc_ranges)-1))
        # print(f"{bubble_zone_index=}")
        for i in range(bubble_zone_index[0], bubble_zone_index[1]+1):
            proc_ranges[i]=0

        gap_min, gap_max = self.find_max_gap(proc_ranges)
        best_index = self.find_best_point(gap_min, gap_max, proc_ranges)
        # print(f"{best_index=}")
        best_dist = proc_ranges[best_index]
        # print(f"{best_dist=}")
        best_angle = (best_index*angle_increment) + angle_min
        print(f"{best_angle=}")

        # want to send angle and distance of the point?
        # distance of the point would let us change the speed --##  __ -##
        self.change_car_behavior(best_angle, best_dist)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()