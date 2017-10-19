############################################################
# Module  : plot point cloud
# Date    : October 16th, 2017
# Author  : Xiao Ling
# plot    : https://matplotlib.org/mpl_toolkits/mplot3d/tutorial.html
# openSfm : https://github.com/mapillary/OpenSfM
# featurematching: https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_matcher/py_matcher.html
# iphone camera intrinsics: https://developer.apple.com/documentation/arkit/arcamera/2875730-intrinsics
############################################################

from cv2 import *
from app_class import App

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

############################################################
# plot points

app = App()

cloud_paths = [ p for p in app.raw_data_paths() 
                if ".txt" in p and "packed" in p ] 

def load_cloud(path):

	cloud = []

	with open(path) as h:
		for v in h:
			vs = v.split("\t")
			vs = [float(v.strip()) for v in vs]
			cloud.append(vs)

	xs = [x for x,_,_ in cloud]
	ys = [y for _,y,_ in cloud]
	zs = [z for _,_,z in cloud]

	return xs, ys, zs

def plot_cloud(xs,ys,zs, color='r', marker='o'):

	fig = plt.figure()
	ax  = fig.add_subplot(111, projection='3d')

	ax.scatter(xs, ys, zs, c=color, marker=marker)

	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')

	plt.show()

xs_1,ys_1,zs_1 = load_cloud(cloud_paths[0])
xs_2,ys_2,zs_2 = load_cloud(cloud_paths[1])

if False:
	plot_cloud(xs_1, ys_1, zs_1, color = 'b', marker = 'o')
	plot_cloud(xs_2, ys_2, zs_2, color = 'r', marker = 'o')

############################################################
# grab orb features 

# load image
im_paths = app.image_paths()
img1     = imread(im_paths[0])
img2     = imread(im_paths[1])

# @Use: given img :: np.ndarray, output keypoints and orb features
def get_feature(img, plot = True):
	# get features
	orb     = ORB_create  ()
	kp      = orb.detect  (img, None)
	kp, des = orb.compute (img,kp)

	if plot:
		img2 = drawKeypoints(img,kp,None,color=(0,255,0), flags=0)
		plt.imshow(img2)
		plt.show()

	return kp, des

kp1, des1 = get_feature(img1, plot = False)
kp2, des2 = get_feature(img2, plot = False)


# for test set image name first
# img2 = img1
# kp2  = kp1
# des2 = des1

	# run something on image wrt to self to confirm R = I and r = 0
bf = BFMatcher(NORM_HAMMING, crossCheck = True)

# bruteforce matching of points
matches = bf.match(des1, des2)
matches = sorted(matches, key = lambda x: x.distance)

# plot for sanity check
if False:
	img3 = drawMatches(img1,kp1,img2,kp2, matches, None, flags=2)
	plt.imshow(img3)
	plt.show()

# grab the matched points
pts1 = np.int32([ kp1[m.queryIdx].pt for m in matches ])
pts2 = np.int32([ kp2[n.trainIdx].pt for n in matches ])

# compute the essential matrix

# scamera intrinsics
K = np.array([[1043.57, 0.0    , 0.0],
		     [0.0     , 1043.57, 0.0],
		     [635.435 , 342.025, 1.0]])

# K = K.T

# camera intrinsics
focal = K[0][0]
cx,cy = K[0][-1], K[1][-1]

E , mask = findEssentialMat(pts1, pts2, focal=1.0  , pp = (cx,cy), method = RANSAC, prob = 0.999, threshold = 1.0)
# E1, mask = findEssentialMat(pts1, pts2, focal=focal, pp = (cx,cy), method = RANSAC, prob = 0.999, threshold = 1.0)

R1,R2,t = decomposeEssentialMat(E)

def rod_vector(R):

	k = 1/np.sqrt(1 - (np.trace(R) - 1)/2)
	M = k*(R - R.T)/2
	v = [ -M[1,-1], M[0,-1], M[1,0] ]
	return v

r1 = rod_vector(R1)
r2 = rod_vector(R2)

# right now we have the rotation matrix and translation vector, we need to test against ground truth.
# note when we try to compute on the thing on itself, R1 is identity as desired, but t is way off by 1/2
# but is relative anyways?


# compute fundamental matrix
F, mask = findFundamentalMat(pts1, pts2)


# # select inlier points
# pts1 = pts1[ mask.ravel() == 1 ]
# pts1 = pts2[ mask.ravel() == 1 ]











































