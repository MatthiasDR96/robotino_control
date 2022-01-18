@echo off
title Experiment C1.1

cd C:\Users\matth\Documents\robotino_server\scripts
start "Webserver" python .\main.py

cd C:\Users\matth\Documents\robotino_factory\scripts
python .\clear_database.py 1

cd C:\Users\matth\Documents\robotino_control\robotino_core\scripts
start "Robot15" python .\main_agv.py 15 
start "TA_Agent15" python .\main_ta_agent.py 15 
start "RO_Agent15" python .\main_ro_agent.py 15 

cd C:\Users\matth\Documents\robotino_factory\scripts
python .\client.py 16

pause

cd C:\Users\matth\Documents\robotino_control\robotino_core\test
python .\delete_graph_node.py

pause

python .\add_graph_node.py