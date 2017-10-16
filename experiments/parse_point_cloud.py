

############################################################
# Module  : parse point cloud
# Date    : October 16th, 2017
# Author  : Xiao Ling
############################################################

import os
from app_class import App

############################################################
# functions

def process_all_raw(top_dir):

	app = App()

	raw_paths = [ p for p in app.raw_data_paths() if ".txt" in p and "raw" in p ]

	processed = dict()

	for raw_path in raw_paths:

		out_path = os.path.join(raw_path.replace("raw", "packed"))
		packed   = process_raw(open(raw_path).read())

		with open(out_path,'w') as o:
			o.write(packed)

def process_raw(xs):
	raw  = xs.lower().replace("frame: ", "").replace("optional", "").replace("\n","").replace("([", "").replace(")]","").strip()
	raws = raw.split("float3")
	raws = [r.replace("(","").replace(")","") for r in raws]
	raws = [r.replace(", ", "\t").strip() for r in raws]
	raws = [r for r in raws if r]
	return "\n".join(raws)


############################################################
# run function

process_all_raw(top_dir)






