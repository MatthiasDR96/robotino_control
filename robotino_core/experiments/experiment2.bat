@echo off
title Experiment 2

cd C:\Users\matth\Documents\robotino_server\scripts
start "Webserver" python .\main.py

cd C:\Users\matth\Documents\robotino_factory\scripts
python .\clear_database.py 1

cd C:\Users\matth\Documents\robotino_control\robotino_core\scripts
start "Robot15" python .\main_agv.py 15 
start "TA_Agent15" python .\main_ta_agent.py 15 

cd C:\Users\matth\Documents\robotino_factory\scripts

pause

python .\client.py 5

pause

cd C:\Users\matth\Documents\robotino_control\robotino_core\scripts
start "Robot16" python .\main_agv.py 16 
start "TA_Agent16" python .\main_ta_agent.py 16 