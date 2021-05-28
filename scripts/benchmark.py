import os

problemset_path = "/home/rajat/melodic_ws/src/C-MINT/data/testing/CMINT_test_set.txt"

with open(problemset_path) as fp:
	for line in fp:
		words = line.strip().split(' ')
		command = "/home/rajat/melodic_ws/build/C-MINT/exampleTwoAgentsC_MINT -k " + words[0] + " -d -o /home/rajat/melodic_ws/src/C-MINT/data/obstacles/2D_Images/"+words[1] +".png --graph /home/rajat/melodic_ws/src/C-MINT/data/sparse_problems/graphs/graph"+words[2]+".graphml -s " 

		for i in range(int(words[0])*2):
			command += words[i+3] + " "

		command += " -t "

		for i in range(int(words[0])*2):
			command += words[i+3+int(words[0])*2] + " "

		command += " --dijkstra"
		
		print(command)
		os.system(command)
		# os.system("/home/rajat/melodic_ws/build/C-MINT/exampleTwoAgentsC_MINT_2 -s " + words[3] + " " + words[4] + " " +  words[5] +" " + words[6] + " -t " + words[7] + " " + words[8] + " " +  words[9] +" " + words[10] + " -d -o /home/rajat/melodic_ws/src/C-MINT/data/obstacles/2D_Images/"+words[1] +".png --left_graph /home/rajat/melodic_ws/src/C-MINT/data/graphs/epsilon_graphs_05_200/graph"+words[2]+".graphml --right_graph /home/rajat/melodic_ws/src/C-MINT/data/graphs/epsilon_graphs_05_200/graph"+words[2]+".graphml")
		# os.system("/home/rajat/melodic_ws/build/C-MINT/exampleTwoAgentsTRIVIAL -s " + words[2] + " " + words[3] + " " +  words[4] +" " + words[5] + " -t " + words[6] + " " + words[7] + " " +  words[8] +" " + words[9] + " -d -o /home/rajat/melodic_ws/src/C-MINT/data/obstacles/2D_Images/"+words[0] +".png --left_graph /home/rajat/melodic_ws/src/C-MINT/data/graphs/epsilon_graphs_05_200/graph"+words[1]+".graphml --right_graph /home/rajat/melodic_ws/src/C-MINT/data/graphs/epsilon_graphs_05_200/graph"+words[1]+".graphml")


