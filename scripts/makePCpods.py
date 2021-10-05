import os 
from pathlib import Path


# agents = [2,4,6]
agents = [4, 6]
tasks = [2, 3, 4]
degrees = [1]
obstacle_files = ['empty', 'easy', 'hard', 'warehouse_easy']

for agent in agents:
	for task in tasks:
		# if agent > task: continue
		for o in obstacle_files:
			o_file = f'/home/kushal/ros_ws/src/CMAPF/data/obstacles/{o}'

			p_file = f'/home/kushal/ros_ws/src/CMAPF/data/PC_Pods/{o}/{agent}_{task}_1/'

			Path(p_file+'/CBS').mkdir(exist_ok = True, parents=True)
			Path(p_file+'/ICTS').mkdir(exist_ok = True, parents=True)

			command = f'./pods -n {agent} -m {task} -c 1 -o {o_file} -p {p_file}'
			print(command)
			# input()
			# input()
			os.system(command)
			# input()