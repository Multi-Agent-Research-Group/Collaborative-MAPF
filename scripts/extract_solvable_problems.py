input_filename = "/home/rajat/melodic_ws/src/C-MINT/data/planning_problems/problem_set_05_200.txt"
result_filename = "/home/rajat/melodic_ws/src/C-MINT/data/results/problem_set_05_200_results.txt"
output_filename = "/home/rajat/melodic_ws/src/C-MINT/data/planning_problems/solvable_problem_set_05_200.txt"

unsolvable_marker = "C_MINT, SOLUTION NOT FOUND!!"

input_file = open(input_filename, 'r')
result_file = open(result_filename, 'r')
output_file = open(output_filename, "w")

input_lines = input_file.readlines() 
result_lines = result_file.readlines()

# Strips the newline character 
for line_no in range(len(result_lines)): 
    if result_lines[line_no].find(unsolvable_marker) == -1:
    	output_file.write(input_lines[line_no])
    else:
    	print(line_no,result_lines[line_no],input_lines[line_no])

input_file.close()
result_file.close()
output_file.close()
