results_paths = ["/home/rajat/melodic_ws/src/C-MINT/data/results/problem_set_05_200_results_01.txt",
	"/home/rajat/melodic_ws/src/C-MINT/data/results/problem_set_05_200_results_005.txt",
	"/home/rajat/melodic_ws/src/C-MINT/data/results/problem_set_05_200_results_0025.txt"]

planning_times_path = "/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/planning_times/"
num_vertices_path = "/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/num_vertices/"
num_edges_path = "/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/num_edges/"

C_MINT_planning_times_file = open(planning_times_path + "C_MINT_2.txt", "w")
# DENSIFY_planning_times_file = open(planning_times_path + "DENSIFY.txt", "w")
# TRIVIAL_planning_times_file = open(planning_times_path + "TRIVIAL.txt", "w")

C_MINT_num_vertices_file = open(num_vertices_path + "C_MINT_2.txt", "w")
# DENSIFY_num_vertices_file = open(num_vertices_path + "DENSIFY.txt", "w")
# TRIVIAL_num_vertices_file = open(num_vertices_path + "TRIVIAL.txt", "w")

C_MINT_num_edges_file = open(num_edges_path + "C_MINT_2.txt", "w")
# DENSIFY_num_edges_file = open(num_edges_path + "DENSIFY.txt", "w")
# TRIVIAL_num_edges_file = open(num_edges_path + "TRIVIAL.txt", "w")

for results_path in results_paths:

	line_no = 1
	with open(results_path) as fp:
		for line in fp:
			words = line.strip().split(',')
			# print(float(words[1]),float(words[7]),float(words[13]))
			# if float(words[1]) != float(words[7]):
			# 	print('ERROR!!!',line_no, float(words[1]), float(words[7]))

			C_MINT_planning_times_file.write(words[2] + " ")
			# DENSIFY_planning_times_file.write(words[8] + " ")
			# TRIVIAL_planning_times_file.write(words[14] + " ")

			C_MINT_num_vertices_file.write(words[3] + " ")
			# DENSIFY_num_vertices_file.write(words[9] + " ")
			# TRIVIAL_num_vertices_file.write(words[15] + " ")

			C_MINT_num_edges_file.write(words[4] + " ")
			# DENSIFY_num_edges_file.write(words[10] + " ")
			# TRIVIAL_num_edges_file.write(words[16] + " ")

			line_no += 1

	C_MINT_planning_times_file.write("\n")
	# DENSIFY_planning_times_file.write("\n")
	# TRIVIAL_planning_times_file.write("\n")

	C_MINT_num_vertices_file.write("\n")
	# DENSIFY_num_vertices_file.write("\n")
	# TRIVIAL_num_vertices_file.write("\n")

	C_MINT_num_edges_file.write("\n")
	# DENSIFY_num_edges_file.write("\n")
	# TRIVIAL_num_edges_file.write("\n")