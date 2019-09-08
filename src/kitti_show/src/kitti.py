#!/usr/bin/env python2
# -*- coding: UTF-8 -*-
'''
import rospy
import cv2
import os
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from cv_bridge import CvBridge
from std_msgs.msg import String
import numpy as np
'''
import os
import rospy
from data_utils import *
from publish_utils import *
from kitti_utils import *

from collections import deque

DATA_PATH = '/home/hao/data/kittidata/2011_09_26/2011_09_26_drive_0005_sync'
#/home/hao/data/kittidata/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data
'''
if __name__ == '__main__':
	frame = 0
	rospy.init_node('kitti_node', anonymous=True)
	cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
	pcl_pub = rospy.Publisher('kitti_pcl', PointCloud2, queue_size=10)
	rate = rospy.Rate(10)
	bridge = CvBridge()
	while not rospy.is_shutdown():
		img = cv2.imread(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
		point_cloud = np.fromfile(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame),dtype=np.float32).reshape(-1, 4)
		cam_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = "map"
		pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))
		rospy.loginfo('published')
		rate.sleep()
		frame += 1
		frame %= 154
'''

EGOCAR = np.array([[2.15,0.9,-1.73], [2.15,-0.9, -1.73], [-1.95,-0.9,-1.73], [-1.95, 0.9, -1.73],
[2.15, 0.9, -0.23], [2.15, -0.9, -0.23], [-1.95, -0.9, -0.23], [-1.95, 0.9, -0.23]])

class Object():
	def __init__(self, center):
		self.location = deque(maxlen=10)
		self.location.appendleft(center)

	def update(self, center, displacement, yaw):
		for i in range(len(self.location)):
			x0, y0 = self.location[i]
			x1 = x0 * np.cos(yaw) + y0 * np.sin(yaw) - displacement
			y1 = - x0 * np.sin(yaw) + y0 * np.cos(yaw)
			self.location[i] = np.array([x1, y1])
		if center is not None:
			self.location.appendleft(center)
		#self.location += [np.array([0,0])]
		#self.location = self.location[-20:]

	def reset(self):
		self.location = deque(maxlen=10)

if __name__ == "__main__":
	frame = 0
	rospy.init_node('kitti_node', anonymous=True)

	cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
	pcl_pub = rospy.Publisher('kitti_pcl', PointCloud2, queue_size=10)
	#ego_pub = rospy.Publisher('kitti_ego_car', Marker, queue_size=10)
	#model_pub = rospy.Publisher('kitti_model_car', Marker, queue_size=10)
	ego_model_pub = rospy.Publisher('kitti_ego_model', MarkerArray, queue_size=10)
	imu_pub = rospy.Publisher('kitti_imu', Imu, queue_size=10)
	gps_pub = rospy.Publisher('kitti_gps', NavSatFix, queue_size=10)
	box3d_pub = rospy.Publisher('kitti_3d', MarkerArray, queue_size=10)
	loc_pub = rospy.Publisher('kitti_loc', MarkerArray, queue_size=10)
	dist_pub = rospy.Publisher('kitti_dist', MarkerArray, queue_size=10)

	bridge = CvBridge()
	rate = rospy.Rate(10)

	df_tracking = read_tracking('/home/hao/data/kittidata/training/label_02/0000.txt')
	calib = Calibration('/home/hao/data/kittidata/2011_09_26_calib/2011_09_26',from_video=True)

	def compute_3d_box_cam2(h, w, l, x, y, z, yaw):
		R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0,1,0], [-np.sin(yaw), 0, np.cos(yaw)]])
		x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]
		y_corners = [0,0,0,0,-h,-h,-h,-h]
		z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]
		corners_3d_cam2 = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
		corners_3d_cam2 += np.vstack([x,y,z])
		return corners_3d_cam2
		pass

	def distance_point_to_segment(P, A, B):
		AP = P-A
		BP = P-B
		AB = B-A
		BA = A-B
		if np.dot(AP, AB)>=0 and np.dot(BA,BP)>=0:
			return np.abs(np.cross(AP, AB))/np.linalg.norm(AB), np.dot(AP, AB)/np.dot(AB,AB)*AB +A
		d_PA = np.linalg.norm(AP)
		d_PB = np.linalg.norm(BP)
		if d_PA < d_PB:
			return d_PA, A
		return d_PB, B
		pass

	def min_distance_cuboids(cub1, cub2):
		minD = 1e5
		for i in range(4):
			for j in range(4):
				d, Q = distance_point_to_segment(cub1[i, :2], cub2[j,:2], cub2[j+1, :2])
				if d < minD:
					minD = d
					minP = cub1[i, :2]
					minQ = Q
		for i in range(4):
			for j in range(4):
				d, Q = distance_point_to_segment(cub2[i, :2], cub1[j,:2], cub1[j+1, :2])
				if d < minD:
					minD = d
					minP = cub2[i, :2]
					minQ = Q
		return minP, minQ, minD
		pass

	#ego_car = Object()
	tracker = {}# tracker_id : Object
	prev_imu_data = None

	while not rospy.is_shutdown():

		image = read_camera(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
		point_cloud = read_point_cloud(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame))
		imu_data = read_imu(os.path.join(DATA_PATH, 'oxts/data/%010d.txt'%frame))

		boxes_2d = np.array(df_tracking[df_tracking.frame==frame][['bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom']])
		types = np.array(df_tracking[df_tracking.frame==frame]['type'])
		boxes_3d = np.array(df_tracking[df_tracking.frame ==frame][['height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']])
		track_ids = np.array(df_tracking[df_tracking.frame==frame]['track_id'])

		corners_3d_velos = []
		centers = {}
		centers[-1] = np.array([0,0])
		minPQDs=[]
		for track_id, box_3d in zip(track_ids, boxes_3d):
			corners_3d_cam2 = compute_3d_box_cam2(*box_3d)#*作用为将array转换为该函数的输入八个参数形式
			corners_3d_velo = calib.project_rect_to_velo(corners_3d_cam2.T)
			minPQDs += [min_distance_cuboids(EGOCAR, corners_3d_velo)]
			corners_3d_velos += [corners_3d_velo]
			centers[track_id] = np.mean(corners_3d_velo, axis=0)[:2]

		if prev_imu_data is None:
			for track_id in centers:
				tracker[track_id] = Object(centers[track_id])
		else:
			displacement = 0.1*np.linalg.norm(imu_data[['vf', 'vl']])
			yaw_change = float(imu_data.yaw - prev_imu_data.yaw)
			for track_id in centers:
				if track_id in tracker:
					tracker[track_id].update(centers[track_id], displacement, yaw_change)
				else:
					tracker[track_id] = Object(centers[track_id])
			for track_id in tracker:
				if track_id not in centers:
					tracker[track_id].update(None, displacement, yaw_change)
		prev_imu_data = imu_data

		publish_camera(cam_pub, bridge, image, boxes_2d, types)
		publish_point_cloud(pcl_pub, point_cloud)
		#publish_ego_car(ego_pub)
		#publish_car_model(model_pub)
		publish_maker_array(ego_model_pub)
		publish_imu_data(imu_pub, imu_data)
		publish_gps(gps_pub, imu_data)
		publish_3dbox(box3d_pub, corners_3d_velos, types, track_ids)
		publish_loc(loc_pub, tracker, centers)
		publish_dist(dist_pub, minPQDs)
		rospy.loginfo('published')
		rate.sleep()
		frame += 1
		if frame == 154:
			frame = 0
			for track_id in tracker:
				tracker[track_id].reset()
		frame %= 154
	pass