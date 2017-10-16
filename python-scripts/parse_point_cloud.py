
############################################################
# Module  : parse point cloud
# Date    : October 16th, 2017
# Author  : Xiao Ling
############################################################

import os
import yaml
import shutil
import pickle

############################################################
# paths

current_dir = os.getcwd()
top_dir = "/".join(current_dir.split("/")[0:-1])


############################################################
# functions

def process_all_raw(top_dir):

	if not os.path.exists(top_dir):
		print "path not found error"
	else:
		data_dir = os.path.join(top_dir, "data")

		raw_paths = [ p for p in os.listdir(data_dir)
		            if ".txt" in p and "raw" in p ]


		processed = dict()

		for raw_path in raw_paths:

			name     = raw_path.replace(".txt", "")
			in_path  = os.path.join(data_dir, raw_path)
			out_path = os.path.join(data_dir, name.replace("raw", "packed.txt"))
			packed   = process_raw(open(in_path).read())

			with open(out_path,'w') as o:
				o.write(packed)


def process_raw(xs):
	raw  = xs.lower().replace("frame: ", "").replace("optional", "").replace("\n","").replace("([", "").replace(")]","").strip()
	raws = raw.split("float3")
	raws = [r.replace("(","").replace(")","") for r in raws]
	raws = [r.replace(", ", "\t").strip() for r in raws]
	raws = [r for r in raws if r]
	return "\n".join(raws)


process_all_raw(top_dir)






