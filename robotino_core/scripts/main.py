#!/usr/bin/env python3

import sys
import signal
from robotino_core.agv.AGV_Main import AGV_Main
from robotino_core.agv.AGV_TA_agent import AGV_TA_agent
from robotino_core.agv.AGV_RO_agent import AGV_RO_agent
from robotino_core.agv.AGV_RM_agent import AGV_RM_agent


if __name__== '__main__':

	# Get id from command line
	if len(sys.argv) > 1:
		id = int(sys.argv[1])
		ta_agent = sys.argv[2] == 'True'
		ro_agent = sys.argv[3] == 'True'
		rm_agent = sys.argv[4] == 'True'
	else:
		print("No valid input given, please specify robot ID and agent booleans.")
		exit(0)

	# Init
	events = []
	threads = []

	# Params file
	params_file = 'setup' + str(id) + '.yaml'

	# Start AGV main
	agv_main = AGV_Main(params_file)
	threads.extend(agv_main.threads)
	events.append(agv_main.exit_event)

	# Start AGV agents
	if ta_agent: 
		ta_agent = AGV_TA_agent(params_file)
		threads.extend(ta_agent.threads)
		events.append(ta_agent.exit_event)
	if ro_agent: 
		ro_agent = AGV_RO_agent(params_file)
		threads.extend(ro_agent.threads)
		events.append(ro_agent.exit_event)
	if rm_agent: 
		rm_agent = AGV_RM_agent(params_file)
		threads.extend(rm_agent.threads)
		events.append(rm_agent.exit_event)

	# Start processes
	for thread in threads:
		thread.start()

	# Exit handler function
	def signal_handler(_, __):
		for event in events:
			event.set()

	# Exit handler
	signal.signal(signal.SIGINT, signal_handler)

