import DeepLKBatch as dlk
import torch
from torch.autograd import Variable
from PIL import Image
import pdb
from math import sin, cos, tan, pi, degrees, radians, asin, acos
import cv2
from sys import argv
import numpy as np
import sift_ransac_homography as srh

###

FIXED_HEIGHT = 250 # fixed height (pixels) for DLK
DLK_ITR = 75 # max LK iterations for DLK
DLK_TOL = 1e-3 # min warp parameter magnitude change for DLK

USE_CUDA = torch.cuda.is_available()
MODEL_PATH = '../models/conv_02_17_18_1833.pth'

if USE_CUDA:
	dlk_model = dlk.DeepLK(dlk.custom_net(MODEL_PATH)).cuda()
else:
	dlk_model = dlk.DeepLK(dlk.custom_net(MODEL_PATH))

###

def wrapTo2Pi(angle):
	w = angle % (2 * pi)

	if (angle > 0) and (w == 0):
		return 2 * pi
	else:
		return w



def wrapToPi(angle):
	if (angle < -pi) or (angle > pi):
		return wrapTo2Pi(angle + pi) - pi
	else:
		return angle



def localizeFromMap(x, y, z, h, img_path, map_path, map_mpp, fov_x, fov_y):
	# Aligns an input UAV image with the map prior using DeepLK, and outputs the corrected UAV pose

	# Inputs:
	# x [double]: UAV pose belief (meters) horizontal from top left corner of map image
	# y [double]: UAV pose belief (meters) vertical from the top left corner of the map image
	# z [double]: UAV pose belief (meters) of height in environment
	# h [double]: UAV heading angle (radians) on map axis-aligned unit circle
	# img_path [path string]: path to img
	# map_path [path string]: path to map
	# map_mpp [double]: meters-per-pixel in planar map prior
	# fov_x [double]: horizontal field-of-view (radians) of UAV camera
	# fov_y [double]: vertical field-of-view (radians) of UAV camera

	# Outputs:
	# xm [double]: Corrected UAV position horizontal from top left corner of map
	# ym [double]: Corrected UAV position vertical from top left corner of map
	# zm [double]: Corrected UAV altitude
	# hm [double]: Corrected UAV 

	img = cv2.imread(img_path)
	full_map = cv2.imread(map_path)


	img_h, img_w, c = img.shape
	map_h, map_w, c = full_map.shape

	# z and fov_x and fov_y, calculate height and width of UAV imaging area in meters
	tmpl_h_meters = 2 * z * tan(fov_y / 2)
	tmpl_w_meters = 2 * z * tan(fov_x / 2)

	# using height and width of imaging area in meters, multiply by 1/map_mpp to get pixel area in map
	tmpl_h_p = tmpl_h_meters / map_mpp
	tmpl_w_p = tmpl_w_meters / map_mpp

	x_pix = x / map_mpp
	y_pix = y / map_mpp

	# using heading angle, x, y, and size of the image to be extracted, extract the template from the map
	map_angle = -1 * degrees(wrapToPi(h - pi / 2))

	M = cv2.getRotationMatrix2D((x_pix, y_pix), map_angle, 1)

	map_warp = cv2.warpAffine(full_map, M, (map_w, map_h))

	# cv2.imshow('map',map_warp)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	y_start_ind = int(round(y_pix - tmpl_h_p / 2))
	y_end_ind = int(round(y_pix + tmpl_h_p / 2))
	x_start_ind = int(round(x_pix - tmpl_w_p / 2))
	x_end_ind = int(round(x_pix + tmpl_w_p / 2))

	tmpl = map_warp[y_start_ind:y_end_ind, x_start_ind:x_end_ind]
	# tmpl = map_warp[x_start_ind:x_end_ind, y_start_ind:y_end_ind]

	### Testing

	# M1 = np.float32([[0,0,0],[0,0,0]])
	# M2 = cv2.getRotationMatrix2D((round(tmpl_w_p/2), round(tmpl_h_p/2)), 15, 1.2)
	# M = M1 + M2
	# img = cv2.warpAffine(tmpl,M,(tmpl.shape[1],tmpl.shape[0]))

	# M = np.float32([[1,0,0],[0,1,0],[-0.001,0,1]])
	# img = cv2.warpPerspective(tmpl,M,(tmpl.shape[1],tmpl.shape[0]))

	# cv2.imshow('img',tmpl)
	# cv2.imshow('img2',img)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	###

	# set template and image to same size
	aspect_ratio = tmpl_w_p / tmpl_h_p
	fixed_width = int(round(aspect_ratio * FIXED_HEIGHT))

	img_rz = cv2.resize(img, (fixed_width, FIXED_HEIGHT))
	tmpl_rz = cv2.resize(tmpl, (fixed_width, FIXED_HEIGHT))

	tmpl_og_rz = tmpl_rz
	img_og_rz = img_rz

	# cv2.imshow('img1',tmpl_rz)
	# cv2.imshow('img2',img_rz)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	# call DeepLK alignment on img and template
	tmpl_rz = np.swapaxes(tmpl_rz, 0, 2)
	tmpl_rz = np.swapaxes(tmpl_rz, 1, 2)
	img_rz = np.swapaxes(img_rz, 0, 2)
	img_rz = np.swapaxes(img_rz, 1, 2)

	img_tens = torch.from_numpy(img_rz)
	tmpl_tens = torch.from_numpy(tmpl_rz)

	if USE_CUDA:
		img_tens = Variable(img_tens.unsqueeze(0).float().cuda())
		tmpl_tens = Variable(tmpl_tens.unsqueeze(0).float().cuda())
	else:
		img_tens = Variable(img_tens.unsqueeze(0).float())
		tmpl_tens = Variable(tmpl_tens.unsqueeze(0).float())

	# normalize
	img_tens_nmlz = dlk.normalize_img_batch(img_tens)
	tmpl_tens_nmlz = dlk.normalize_img_batch(tmpl_tens)

	p_tens, H_tens = dlk_model(img_tens_nmlz, tmpl_tens_nmlz, tol=DLK_TOL, max_itr=DLK_ITR, conv_flag=1)
	p_tens = srh.get_param(img_tens, tmpl_tens, FIXED_HEIGHT)

	p_tens = p_tens.squeeze(0)
	H_tens = H_tens.squeeze(0)
	# p_tens_srh = p_tens_srh.squeeze(0)

	if USE_CUDA:
		p = p_tens.data.cpu().numpy()
		H = H_tens.data.cpu().numpy()
	else:
		p = p_tens.data.numpy()
		H = H_tens.data.numpy()
	# p_srh = p_tens_srh.data.numpy()

	# pdb.set_trace()

	# print(H)
	# print(p)
	# print(p_srh)
	# extract rotation, translation, and scaling from DeepLK result
	rot = (asin(-p[1]) + asin(p[3])) / 2
	trans_x = p[2]
	trans_y = p[5]
	scaling = ((1 + p[0]) / cos(rot) + (1 + p[4]) / cos(rot)) / 2

	# adjust x and y using translation, adjust heading rotation, and adjust z using scaling
	lk_scaling_factor = tmpl_h_p / FIXED_HEIGHT

	# apply rotation matrix to trans_x and trans_y
	q = wrapToPi(pi/2 - h)
	x_tmp = trans_x * cos(q) - trans_y * sin(q)
	y_tmp = trans_x * sin(q) + trans_y * cos(q)

	xm = x - x_tmp * lk_scaling_factor * map_mpp
	ym = y - y_tmp * lk_scaling_factor * map_mpp
	hm = wrapToPi(h + rot)
	zm = z / scaling

	print('rotation: {:6.3f}'.format(float(rot)))
	print('trans_x: {:6.3f}'.format(float(trans_x)))
	print('trans_y: {:6.3f}'.format(float(trans_y)))
	print('scaling: {:6.3f}'.format(float(scaling)))
	print('lk_scaling_factor: {:6.3f}'.format(lk_scaling_factor))
	print('x : {:6.3f}, y : {:6.3f}, h : {:6.3f}, z : {:6.3f}'.format(x, y, h, z))
	print('xm: {:6.3f}, ym: {:6.3f}, hm: {:6.3f}, zm: {:6.3f}'.format(float(xm), float(ym), hm, float(zm)))

	#### Testing
	
	# M1 = np.float32([[0,0,trans_x],[0,0,trans_y]])
	# M2 = cv2.getRotationMatrix2D((round(tmpl_rz.shape[2]/2), round(tmpl_rz.shape[1]/2)), degrees(-rot), scaling)

	# tmpl_warp = cv2.warpAffine(tmpl_og_rz, M1 + M2, (tmpl_rz.shape[2], tmpl_rz.shape[1]))

	# cv2.imshow('img1',tmpl_og_rz)
	# cv2.imshow('img2',img_og_rz)
	# cv2.imshow('img3',tmpl_warp)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	####
	# pdb.set_trace()


	return xm, ym, zm, hm



def test():
	# usage
	# python3 map_odom.py 250 350 500 0 ../sat_data/woodbridge_sm/images/im1.png ../sat_data/woodbridge_sm/images/im1.png 1 50 50
	x = float(argv[1])
	y = float(argv[2])
	z = float(argv[3])
	h = float(argv[4])
	img_path = argv[5]
	map_path = argv[6]
	map_mpp = float(argv[7])
	fov_x = float(argv[8])
	fov_y = float(argv[9])

	fov_x = radians(fov_x)
	fov_y = radians(fov_y)

	localizeFromMap(x, y, z, h, img_path, map_path, map_mpp, fov_x, fov_y)


if __name__ == "__main__":
	test()
