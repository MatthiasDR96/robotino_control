#!/usr/bin/env python3

from robotino_core.agv.AGV_Main import AGV

if __name__== '__main__':

    # Start AGV
    AGV('localhost', 10015, 15, 'localhost', 'matthias', 'matthias', 'kb', (10, 30))