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

# run something on image wrt to self to confirm R = I and r = 0
bf = BFMatcher(NORM_HAMMING, crossCheck = True)

# try identity match first
matches = bf.match(des1, des2)
matches = sorted(matches, key = lambda x: x.distance)

# plot for sanity check
img3 = drawMatches(img1,kp1,img2,kp2, matches, None, flags=2)
plt.imshow(img3)
plt.show()

# now find R and t





































