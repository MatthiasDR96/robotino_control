@echo off
title Experiment C1.1

cd C:\Users\matth\Documents\robotino_server\scripts
start "Webserver" python .\main.py

cd C:\Users\matth\Documents\robotino_factory\scripts
python .\clear_database.py 1

cd C:\Users\matth\Documents\robotino_control\robotino_core\scripts
start "Robot15" python .\main_agv.py 15 
start "Robot16" python .\main_agv.py 16 
start "Robot17" python .\main_agv.py 17 
start "TA_Agent15" python .\main_ta_agent.py 15 
start "TA_Agent16" python .\main_ta_agent.py 16 
start "TA_Agent17" python .\main_ta_agent.py 17 

cd C:\Users\matth\Documents\robotino_factory\scripts

python .\client.py 8

python .\client.py 11

python .\client.py 4

python .\client.py 16

pause 
cd C:\Users\matth\Documents\robotino_control\robotino_core\scripts
start "Robot15" python .\main_agv.py 15 
start "TA_Agent15" python .\main_ta_agent.py 15 

pause 

start "Robot16" python .\main_agv.py 16 
start "TA_Agent16" python .\main_ta_agent.py 16 

pause 

start "Robot17" python .\main_agv.py 17 
start "TA_Agent17" python .\main_ta_agent.py 17 