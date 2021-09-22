import os 
from pathlib import Path


agents = [3, 4, 6, 9]
tasks = [3, 6, 9, 12]
degrees = [1, 2, 3]
obstacle_files = ['easy']

for agent in agents:
	for task in tasks:
		for degree in degrees:
			if agent > task*degree: continue
			for o in obstacle_files:
				o_file = '/home/kushal/ros_ws/src/CMAPF/data/obstacles/100'

				p_file = f'/home/kushal/ros_ws/src/CMAPF/data/greedy/empty/{agent}_{task}_{degree}/'

				Path(p_file+'/CBS').mkdir(exist_ok = True, parents=True)
				Path(p_file+'/ICTS').mkdir(exist_ok = True, parents=True)

				command = f'./greedyAssignment -n {agent} -m {task} -c {degree} -o {o_file} -p {p_file}'
				print(command)
				# input()
				# input()
				os.system(command)