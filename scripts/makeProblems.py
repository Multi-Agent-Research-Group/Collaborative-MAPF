import os 
from pathlib import Path

# map_file = '../data/maps/room-64-64-16.map'
# graph_file = '../data/graphs/room.graphml'
# weights_file = '../data/weights/room.txt'
# output_folder = '../data/problems/room/'

map_file = '../data/maps/warehouse-10-20-10-2-2.map'
graph_file = '../data/graphs/warehouse.graphml'
weights_file = '../data/weights/warehouse.txt'
output_folder = '../data/problems/warehouse/'

for agent in [20, 40, 60, 80, 100]:
	total_tasks = agent*5
	double_tasks = int(total_tasks*0.1)
	single_tasks = total_tasks-double_tasks
	precedence_constraints = int(total_tasks*0.2)

	out_file = f'{output_folder}/agents/{agent}'
	Path(out_file).mkdir(exist_ok = True, parents=True)

	command = f'./mp -w {weights_file} \
	-m {map_file} -g {graph_file} -o {out_file}\
	-n {agent} -s {single_tasks} -d {double_tasks} -p {precedence_constraints}'
	print(command)
	os.system(command)

for total_tasks in [50, 100, 150, 200, 250]:
	agent = 20
	double_tasks = int(total_tasks*0.1)
	single_tasks = total_tasks-double_tasks
	precedence_constraints = int(total_tasks*0.2)

	out_file = f'{output_folder}/Tasks/{total_tasks}'
	Path(out_file).mkdir(exist_ok = True, parents=True)

	command = f'./mp -w {weights_file} \
	-m {map_file} -g {graph_file} -o {out_file}\
	-n {agent} -s {single_tasks} -d {double_tasks} -p {precedence_constraints}'
	print(command)
	os.system(command)

for percent_pc in [0, 0.25, 0.5, 0.75, 1.00]:
	agent = 20
	total_tasks = 100
	double_tasks = int(total_tasks*0.1)
	single_tasks = total_tasks-double_tasks
	precedence_constraints = int(total_tasks*percent_pc)

	out_file = f'{output_folder}/PC/{int(percent_pc*100)}'
	Path(out_file).mkdir(exist_ok = True, parents=True)

	command = f'./mp -w {weights_file} \
	-m {map_file} -g {graph_file} -o {out_file}\
	-n {agent} -s {single_tasks} -d {double_tasks} -p {precedence_constraints}'
	print(command)
	os.system(command)

for percent_colab in [0, 0.25, 0.5, 0.75, 1.00]:
	agent = 20
	total_tasks = 100
	double_tasks = int(total_tasks*percent_colab)
	single_tasks = total_tasks-double_tasks
	precedence_constraints = int(total_tasks*0.2)

	out_file = f'{output_folder}/Colab/{int(percent_colab*100)}'
	Path(out_file).mkdir(exist_ok = True, parents=True)

	command = f'./mp -w {weights_file} \
	-m {map_file} -g {graph_file} -o {out_file}\
	-n {agent} -s {single_tasks} -d {double_tasks} -p {precedence_constraints}'
	print(command)
	os.system(command)