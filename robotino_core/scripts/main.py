#!/usr/bin/env python3

import sys
from robotino_core.agv.AGV_Main import AGV_Main

if __name__== '__main__':

	# Get id from command line
	if len(sys.argv) > 1:
		id = int(sys.argv[1])
		ta_agent = sys.argv[2] == 'True'
		ro_agent = sys.argv[3] == 'True'
		rm_agent = sys.argv[4] == 'True'
	else:
		id = 15
		ta_agent = True
		ro_agent = True
		rm_agent = False

	# Start AGV
	params_file = 'setup' + str(id) + '.yaml'
	agv = AGV_Main(params_file, ta_agent, ro_agent, rm_agent)
