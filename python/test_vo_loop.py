from map_odom import localizeFromMap
import pdb
from math import sin, cos, tan, pi, degrees, radians, asin, acos, floor
import cv2
from sys import argv
import numpy as np
import csv
import argparse

# usage
# python3 test_vo_loop.py ~/Documents/School/CMU/SLAM/project/pa_img/map.png ~/Documents/School/CMU/SLAM/project/flight_2/images/ ~/Documents/School/CMU/SLAM/project/flight_2/flight2_vo_metric_relative.csv ~/Documents/School/CMU/SLAM/project/flight_2/gt_traj.csv 0.3 1.2217 1.2217 40 drone_data_dlk_poses.csv
def test_vo_path():

	parser = argparse.ArgumentParser()
	parser.add_argument("map_path",help="path to map image")
	parser.add_argument("frame_path",help="images/ folder to extract frames")
	parser.add_argument("vo_path",help="visual odometry csv in format: frame #, dx, dy, dz, dheading")
	parser.add_argument("gt_path",help="ground truth csv in format: frame #, x, y, z, heading")
	parser.add_argument("map_mpp",help="meters-per-pixel scale of map")
	parser.add_argument("fov_x",help="horizontal field of view")
	parser.add_argument("fov_y",help="vertical field of view")
	parser.add_argument("n_update",help="after n_update number of frames, use DeepLK to align")
	parser.add_argument("out_file",help="path to output file for DeepLK poses")
	parser.add_argument("img_type",help="frame image type (png or jpg)")
	parser.add_argument("start_frame",help="begin at frame other than 1")

	args = parser.parse_args()

	map_path = args.map_path
	frame_path = args.frame_path
	vo_path = args.vo_path
	gt_path = args.gt_path
	map_mpp = float(args.map_mpp)
	fov_x = float(args.fov_x)
	fov_y = float(args.fov_y)
	fov_x = fov_x / 180 * pi
	fov_y = fov_y / 180 * pi
	n_update = int(args.n_update)
	out_file = args.out_file
	img_type = args.img_type
	start_frame = int(args.start_frame)

	# outputs:
	# 	at every n frames, a tuple (x, y, z, h) calculated with DLK

	gt_data = np.genfromtxt(gt_path,delimiter=',')
	vo_data = np.genfromtxt(vo_path,delimiter=',')

	total_frames = vo_data.shape[0]
	num_dlk_poses = floor(total_frames / n_update)

	dlk_poses = np.zeros([num_dlk_poses, 5],dtype=float)
	dlk_pose_ind = 0

	x_curr = gt_data[0][1]
	y_curr = gt_data[0][2]
	z_curr = gt_data[0][3]
	h_curr = gt_data[0][4]

	for frame_num in range(start_frame, total_frames):
		x_curr = x_curr + vo_data[frame_num - 1][1]
		y_curr = y_curr + vo_data[frame_num - 1][2]
		z_curr = z_curr + vo_data[frame_num - 1][3]
		h_curr = h_curr + vo_data[frame_num - 1][4]

		if (frame_num % n_update == 0):
			print('processing {:04d}/{:d}'.format(frame_num, total_frames))
			img_path = frame_path + 'frame_{:04d}'.format(frame_num) + '.' + img_type

			xm, ym, zm, hm = localizeFromMap(
				x_curr, y_curr, z_curr, h_curr, img_path, map_path, map_mpp, fov_x, fov_y)

			dlk_poses[dlk_pose_ind][0] = frame_num
			dlk_poses[dlk_pose_ind][1] = xm
			dlk_poses[dlk_pose_ind][2] = ym
			dlk_poses[dlk_pose_ind][3] = zm
			dlk_poses[dlk_pose_ind][4] = hm

			x_curr = float(xm)
			y_curr = float(ym)
			z_curr = float(zm)
			h_curr = float(hm)

			dlk_pose_ind = dlk_pose_ind + 1

	np.savetxt(out_file, dlk_poses, delimiter=',')


if __name__ == "__main__":
	test_vo_path()