import os

problemset = ["3_5_2"]
problemtype = ["hard"]

for pt in problemtype:
	for ps in problemset:
		for i in range(1,101):
			command = "/home/rajat/melodic_ws/build/CMAPF/examplePCSolverHCBS -f /home/rajat/melodic_ws/src/CMAPF/data/warehouse/" + pt + "/" + ps + "/ICTS/" + str(i) +".txt -g /home/rajat/melodic_ws/src/CMAPF/data/test_graphs/graph0.graphml -o /home/rajat/melodic_ws/src/CMAPF/data/obstacles/warehouse_" + pt + ".png "
			# print(command)
			os.system(command)