#!/usr/bin/env python

import rospy
from tf import TransformListener
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped, PointStamped, Twist
from visualization_msgs.msg import Marker
from object_recognition_msgs.msg import TableArray
from object_recognition_msgs.msg import Table
import math
import sys
from numpy import *
from qhull_2d import *
from min_bounding_rect import *
import time

def tableposecallback(data):
    global count
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    if(not(math.isnan(data.tables[0].pose.orientation.x))):

        tf_listener = TransformListener()
        table_pose = data.tables[0].pose
        tpose = PoseStamped()
        tpose.header.frame_id = "/camera1_color_optical_frame"
        tpose.pose = table_pose
        tpose_in_base = tf_listener.transformPose("/base_link", p1)
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", math.isnan(data.tables[0].pose.orientation.x))
        point_list = data.tables[0].convex_hull
        t_point_list = []
        temp_point = PointStamped()
        for i in range(0,len(point_list)):
                temp_point.header.frame_id = "/camera_link_optical_1"
                temp_point.point.x = point_list[i].x
                temp_point.point.y = point_list[i].y
                temp_point.point.z = point_list[i].z
                t_point = tf_listener.transformPoint("/base_link", temp_point)
                t_point_list.append([point_list[i].x, point_list[i].y])
        #remove outliers
        count  = count + 1
        xy_points = array(t_point_list)
        # Find convex hull
        hull_points = qhull2D(xy_points)
        # Find minimum area bounding rectangle
        (rot_angle, area, width, height, center_point, corner_points) = minBoundingRect(hull_points)

        # Verbose output of return data
        '''
        print "Minimum area bounding box:"
        print "Rotation angle:", rot_angle, "rad  (", rot_angle*(180/math.pi), "deg )"
        print "Width:", width, " Height:", height, "  Area:", area
        print "Center point: \n", center_point # numpy array
        print "Corner points: \n", corner_points, "\n"  # numpy array

        print("Table Pose wrt base link")
        print(p_in_base.pose.position.x)
        print(p_in_base.pose.position.y)
        print(p_in_base.pose.position.z)
        table_angle = rot_angle*(180/math.pi)

        triplePoints = []
        marker = Marker()
        marker.header.frame_id = "/base_link";
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1


        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.b = 1.0
        marker.color.a = 1.0


        #transform from x,y points to x,y,z points
        for i in range(0,len(corner_points)):
                p = Point()
                p.x = corner_points[i,0]
                p.y = corner_points[i,1]
                p.z = 1
                marker.points.append(p)
        '''
        twist = Twist()


        if(count >=20):
                count = 1
                calculated_time = abs(p_in_base.pose.position.y)/0.1
                print(calculated_time)
                y_corr_start_time = time.time()
                print(y_corr_start_time)
                while((time.time()-y_corr_start_time) < calculated_time):
                        #print("Moving in y")
                        twist.linear.x = 0
                        if(p_in_base.pose.position.y<0):
                            twist.linear.y = -0.1
                        else:
                            twist.linear.y = 0.1
                        twist.linear.z = 0
                        twist.angular.x = 0
                        twist.angular.y = 0
                        twist.angular.z = 0
                        pub.publish(twist)

                calculated_time = (p_in_base.pose.position.x-0.85)/0.1
                x_corr_start_time = time.time()
                while((time.time()-x_corr_start_time) < calculated_time):
                        #print("Moving in x")
                        twist.linear.x = 0.1
                        twist.linear.y = 0
                        twist.linear.z = 0
                        twist.angular.x = 0
                        twist.angular.y = 0
                        twist.angular.z = 0
                        pub.publish(twist)

                if(table_angle>45):
                        #print("Rotate robot clockwise")
                        #print("Angle",(90-table_angle))
                        calculated_time = (abs(90-table_angle))*(math.pi/180)/0.2
                        th_corr_start_time = time.time()
                        while((time.time()-th_corr_start_time) < calculated_time):
                                twist.linear.x = 0
                                twist.linear.y = 0
                                twist.linear.z = 0
                                twist.angular.x = 0
                                twist.angular.y = 0
                                twist.angular.z = -0.1
                                pub.publish(twist)

                else:
                        #print("Rotate robot anticlockwise")
                        #print("Angle",(table_angle))
                        calculated_time = (abs(table_angle))*(math.pi/180)/0.2
                        th_corr_start_time = time.time()
                        while((time.time()-th_corr_start_time)< calculated_time):
                                twist.linear.x = 0
                                twist.linear.y = 0
                                twist.linear.z = 0
                                twist.angular.x = 0
                                twist.angular.y = 0
                                twist.angular.z = 0.1
                                pub.publish(twist)

                print("Robot pose corrected\n")

def adjust():

    rospy.init_node('ridgeback_adjust', anonymous=True)
    rospy.Subscriber("/table_array", TableArray, tableposecallback)
    global count
    count = 1
    rospy.spin()

if __name__ == '__main__':
    adjust()
