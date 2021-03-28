problemset_path = "/home/rajat/melodic_ws/src/C-MINT/data/results/e_roadmaps.txt"

line_no = 1
with open(problemset_path) as fp:
	for line in fp:
		words = line.strip().split(',')
		# print(float(words[1]),float(words[7]),float(words[13]))
		if float(words[1]) != float(words[7]):
			print(line_no, float(words[1]), float(words[7]))
		line_no += 1

