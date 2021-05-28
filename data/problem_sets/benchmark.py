import os

# a = [] 
# b = [] 
# c = [] 
# d = [] 
# e = [] 
# f = [] 

problemset = ["3_8_25","3_10_25","3_12_25","7_8_25","7_10_25","7_12_25", \
	"4_8_35","4_10_35","4_12_35","5_8_35","5_10_35","5_12_35","6_8_35","6_10_35","6_12_35", \
	"7_8_35","7_10_35","7_12_35" ]

for ps in problemset:
	for i in range(1,101):
		command = "./build/PCICTS/examplePCSolverPCICTS -f src/PCICTS/data/problem_sets/" + ps + "/CBS/" + str(i) +".txt -g test_graphs/graph0.graphml -o src/PCICTS/data/obstacles/0.png"
		# print(command)
		os.system(command)

# for i in range(1,101):
# 	if (i-1) not in a:
# 		command = "./build/PCICTS/examplePCSolverPCICTS -f test_problems/05_03/CBS/" + str(i) +".txt -g test_graphs/graph0.graphml -o src/PCICTS/data/obstacles/0.png"
# 		os.system(command)
# 	else:
# 		print("0 0 0")

# for i in range(1,101):
# 	if (i-1) not in b:
# 		command = "./build/PCICTS/examplePCSolverPCICTS -f test_problems/05_04/CBS/" + str(i) +".txt -g test_graphs/graph0.graphml -o src/PCICTS/data/obstacles/0.png"
# 		os.system(command)
# 	else:
# 		print("0 0 0")

# for i in range(1,101):
# 	if (i-1) not in c:
# 		command = "./build/PCICTS/examplePCSolverPCICTS -f test_problems/05_05/CBS/" + str(i) +".txt -g test_graphs/graph0.graphml -o src/PCICTS/data/obstacles/0.png"
# 		os.system(command)
# 	else:
# 		print("0 0 0")

# for i in range(1,101):
# 	if (i-1) not in d:
# 		command = "./build/PCICTS/examplePCSolverPCICTS -f test_problems/06_03/CBS/" + str(i) +".txt -g test_graphs/graph0.graphml -o src/PCICTS/data/obstacles/0.png"
# 		os.system(command)
# 	else:
# 		print("0 0 0")

# for i in range(1,101):
# 	if (i-1) not in e:
# 		command = "./build/PCICTS/examplePCSolverPCICTS -f test_problems/06_04/CBS/" + str(i) +".txt -g test_graphs/graph0.graphml -o src/PCICTS/data/obstacles/0.png"
# 		os.system(command)
# 	else:
# 		print("0 0 0")

# for i in range(1,101):
# 	if (i-1) not in f:
# 		command = "./build/PCICTS/examplePCSolverPCICTS -f test_problems/06_05/CBS/" + str(i) +".txt -g test_graphs/graph0.graphml -o src/PCICTS/data/obstacles/0.png"
# 		os.system(command)
# 	else:
# 		print("0 0 0")



