#!/usr/bin/env python3

import sys
import signal
from robotino_core.agv.AGV_RM_agent import AGV_RM_agent

if __name__== '__main__':

	# Get id from command line
	if len(sys.argv) > 1:
		id = int(sys.argv[1])
	else:
		print("No valid input given, please specify robot ID.")
		exit(0)

	# Params file
	params_file = 'setup' + str(id) + '.yaml'

	# Start RM agent
	rm_agent = AGV_RM_agent(params_file)
	threads = rm_agent.threads
	
	# Start processes
	for thread in threads:
		thread.start()

	# Exit handler function
	def signal_handler(_, __):
		rm_agent.exit_event.set()

	# Exit handler
	signal.signal(signal.SIGINT, signal_handler)

