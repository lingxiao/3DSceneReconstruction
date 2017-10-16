############################################################
# Module  : Application manager
# Date    : October 16th, 2017
# Author  : Xiao Ling
############################################################

import os
import yaml 

class App:

	def __init__(self):

		data_dir_key = 'data-dir'
		yaml_path = os.path.join(os.getcwd(), "config.yml")

		if os.path.exists(yaml_path):
			with open(yaml_path, 'r') as h:
				try:
					env = yaml.load(h)
					if data_dir_key in env:
						data_dir = env[data_dir_key]
						self.data_root = data_dir
						self.ENV = dict(get_all_data(data_dir_key, env))
				except yaml.YAMLError as e:
					print e

	def raw_data_paths(self):
		return [ v for _,v in self.ENV.iteritems() ]

	def image_paths(self):
		return [ p for p in self.raw_data_paths() if ".jpg" in p or ".png" in p ]

	def data_dir(self):
		return self.data_root

def get_all_data(data_dir_key, env):

	print('\n>> loading all data paths, this will take a minute ...')			

	if data_dir_key in env:
		data_dir = env[data_dir_key]
		paths    = [ (p,os.path.join(data_dir,p)) for p in os.listdir(data_dir) if '.DS_Store' not in p ]
		return paths			

