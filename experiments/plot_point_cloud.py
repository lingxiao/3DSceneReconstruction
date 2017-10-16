
############################################################
# Module  : plot point cloud
# Date    : October 16th, 2017
# Author  : Xiao Ling
############################################################

import os
import cv2
import yaml 

from app_class import App

############################################################
# paths

app = App()

cloud_paths = [ p for p in app.raw_data_paths() 
                if ".txt" in p and "packed" in p ] 

                