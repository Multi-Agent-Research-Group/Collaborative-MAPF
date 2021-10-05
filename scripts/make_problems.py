import os 
from pathlib import Path


agents = [3,4, 6]
tasks = [2,4, 6,8]
degrees = [1, 2,3]

problems = ['3_2_3', '3_4_3', '3_6_3',\
			'6_2_3', '6_4_3', '6_6_3',\
			'4_4_2', '4_6_2', '4_8_2',\
			'6_4_2', '6_6_2', '6_8_2']
obstacle_files = ['empty', 'easy', 'warehouse_easy', 'hard']

for agent in agents:
	for task in tasks:
		for degree in degrees:
			if agent > task*degree: continue
			problem = f'{agent}_{task}_{degree}'
			# print(problem)
			if problem not in problems: continue
			print(problem)
			for o in obstacle_files:
				o_file = f'/home/kushal/ros_ws/src/CMAPF/data/obstacles/{o}'

				p_file = f'/home/kushal/ros_ws/src/CMAPF/data/greedy/{o}/{agent}_{task}_{degree}/'

				Path(p_file+'/CBS').mkdir(exist_ok = True, parents=True)
				Path(p_file+'/ICTS').mkdir(exist_ok = True, parents=True)

				command = f'./greedyAssignment -n {agent} -m {task} -c {degree} -o {o_file} -p {p_file}'
				print(command)
				# input()
				# input()
				os.system(command)
				# input()