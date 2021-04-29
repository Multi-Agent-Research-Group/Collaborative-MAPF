// Standard C++ libraries
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>
// #include <boost/graph/transpose_graph.hpp>
#include <boost/graph/topological_sort.hpp>

#include <chrono>
using namespace std::chrono;

// Boost libraries
// #include <boost/shared_ptr.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/function.hpp>
#include <boost/program_options.hpp>

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Custom header files
#include "CMAPF/CBS.hpp"

using namespace boost;
using namespace CMAPF;


// int main(int argc, char *argv[])
// {
// 	Pair edge_array[4] = { Pair(0,1), Pair(1,2),
// 													Pair(3, 4), Pair(4, 5) };
		
// 	PrecedenceConstraintGraph G(6);

// 	property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
		
// 	float eps = 0.0625;

// 	name[0] = meta_data (std::make_pair(eps*5, eps*2), std::make_pair(eps*6, eps*5), std::vector <int> (1, 0), 1);
// 	name[1] = meta_data (std::make_pair(eps*6, eps*5), std::make_pair(eps*6, eps*6), std::vector <int> (1, 0), 2);
// 	name[2] = meta_data (std::make_pair(eps*6, eps*6), std::make_pair(eps*6, eps*7), std::vector <int> (1, 0), 3);

// 	name[3] = meta_data (std::make_pair(eps*5, eps*1), std::make_pair(eps*6, eps*3), std::vector <int> (1, 1), 4);
// 	name[4] = meta_data (std::make_pair(eps*6, eps*3), std::make_pair(eps*6, eps*4), std::vector <int> (1, 1), 5);
// 	name[5] = meta_data (std::make_pair(eps*6, eps*4), std::make_pair(eps*6, eps*8), std::vector <int> (1, 1), 6);

//   	for (int i = 0; i < 4; ++i)
// 		add_edge(edge_array[i].first, edge_array[i].second, G);

// 	// PCSolver p;

// 	Eigen::VectorXd init_config(4);
// 	init_config << eps*5,  eps*2, eps*5, eps*1;
	

// 	int numAgents = 2;
// 	int numTasks = 6;
// 	std::vector<std::vector<std::pair<int,std::pair<Eigen::VectorXd,Eigen::VectorXd>>>> _tasks_list(numAgents);

// 	std::vector< PCVertex > c;

// 	topological_sort(G, std::back_inserter(c));
// 	// property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);

// 	for ( std::vector< PCVertex >::reverse_iterator ii=c.rbegin(); ii!=c.rend(); ++ii)
// 	{
// 		// std::cout << std::endl;
// 		meta_data vertex = get(name, *ii);

// 		int task_id = vertex.task_id;
// 		// std::cout << task_id << std::endl;

// 		Eigen::VectorXd start_config(2);
// 		start_config[0] = vertex.start.first;
// 		start_config[1] = vertex.start.second;

// 		Eigen::VectorXd goal_config(2);
// 		goal_config[0] = vertex.goal.first;
// 		goal_config[1] = vertex.goal.second;

// 		std::vector <int> agent_list = vertex.agent_list;
// 		for (auto agentNum: agent_list){
// 			// std::cout << agentNum << std::endl;
// 			_tasks_list[agentNum].push_back(std::make_pair(task_id, std::make_pair(start_config, goal_config)));
// 		}
// 		// std::cout << std::endl;
// 	}

// 	PCSolver p;

// 	auto start = high_resolution_clock::now();
// 	p.solve(init_config, _tasks_list);
// 	auto stop = high_resolution_clock::now();
// 	std::chrono::duration<double, std::micro> dur = (stop - start);
// 	std::cout << dur.count()/1000000.0 << std::endl;

// 	return 0;	
// }

// int main(int argc, char *argv[])
// {
// 	Pair edge_array[8] = { Pair(0,1), Pair(1,2),
// 	                  Pair(3, 4), Pair(4, 5),
// 	                  Pair(6, 7), Pair(7, 8),
// 	                  Pair(9, 10), Pair(10, 11) };

// 	PrecedenceConstraintGraph G(12);

// 	property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);

// 	float eps = 0.0625;

// 	name[0] = meta_data (std::make_pair(eps*5, eps*2), std::make_pair(eps*6, eps*9), std::vector <int> (1, 0), 0);
// 	name[1] = meta_data (std::make_pair(eps*6, eps*9), std::make_pair(eps*6, eps*10), std::vector <int> (1, 0), 1);
// 	name[2] = meta_data (std::make_pair(eps*6, eps*10), std::make_pair(eps*6, eps*11), std::vector <int> (1, 0), 2);

// 	name[3] = meta_data (std::make_pair(eps*8, eps*1), std::make_pair(eps*6, eps*7), std::vector <int> (1, 1), 3);
// 	name[4] = meta_data (std::make_pair(eps*6, eps*7), std::make_pair(eps*6, eps*8), std::vector <int> (1, 1), 4);
// 	name[5] = meta_data (std::make_pair(eps*6, eps*8), std::make_pair(eps*6, eps*12), std::vector <int> (1, 1), 5);

// 	name[6] = meta_data (std::make_pair(eps*5, eps*1), std::make_pair(eps*6, eps*5), std::vector <int> (1, 2), 6);
// 	name[7] = meta_data (std::make_pair(eps*6, eps*5), std::make_pair(eps*6, eps*6), std::vector <int> (1, 2), 7);
// 	name[8] = meta_data (std::make_pair(eps*6, eps*6), std::make_pair(eps*6, eps*13), std::vector <int> (1, 2), 8);

// 	name[9] = meta_data (std::make_pair(eps*8, eps*4), std::make_pair(eps*6, eps*3), std::vector <int> (1, 3), 9);
// 	name[10] = meta_data (std::make_pair(eps*6, eps*3), std::make_pair(eps*6, eps*4), std::vector <int> (1, 3), 10);
// 	name[11] = meta_data (std::make_pair(eps*6, eps*4), std::make_pair(eps*6, eps*14), std::vector <int> (1, 3), 11);

// 	for (int i = 0; i < 8; ++i)
// 	add_edge(edge_array[i].first, edge_array[i].second, G);

// 	Eigen::VectorXd init_config(8);
// 	init_config << eps*5,  eps*2, eps*8, eps*1, eps*5, eps*1, eps*8, eps*4;
	

// 	int numAgents = 4;
// 	int numTasks = 12;
// 	std::vector<std::vector<std::pair<int,std::pair<Eigen::VectorXd,Eigen::VectorXd>>>> _tasks_list(numAgents);

// 	std::vector< PCVertex > c;

// 	topological_sort(G, std::back_inserter(c));
// 	// property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);

// 	for ( std::vector< PCVertex >::reverse_iterator ii=c.rbegin(); ii!=c.rend(); ++ii)
// 	{
// 		// std::cout << std::endl;
// 		meta_data vertex = get(name, *ii);

// 		int task_id = vertex.task_id;
// 		// std::cout << task_id << std::endl;

// 		Eigen::VectorXd start_config(2);
// 		start_config[0] = vertex.start.first;
// 		start_config[1] = vertex.start.second;

// 		Eigen::VectorXd goal_config(2);
// 		goal_config[0] = vertex.goal.first;
// 		goal_config[1] = vertex.goal.second;

// 		std::vector <int> agent_list = vertex.agent_list;
// 		for (auto agentNum: agent_list){
// 			// std::cout << agentNum << std::endl;
// 			_tasks_list[agentNum].push_back(std::make_pair(task_id, std::make_pair(start_config, goal_config)));
// 		}
// 		// std::cout << std::endl;
// 	}

// 	std::vector<std::vector<std::pair<int,int>>> _tasks_to_agents_list(12);

// 	_tasks_to_agents_list[0] = std::vector<std::pair<int,int>>(1,std::make_pair(0,0));
// 	_tasks_to_agents_list[1] = std::vector<std::pair<int,int>>(1,std::make_pair(0,1));
// 	_tasks_to_agents_list[2] = std::vector<std::pair<int,int>>(1,std::make_pair(0,2));
// 	_tasks_to_agents_list[3] = std::vector<std::pair<int,int>>(1,std::make_pair(1,0));
// 	_tasks_to_agents_list[4] = std::vector<std::pair<int,int>>(1,std::make_pair(1,1));
// 	_tasks_to_agents_list[5] = std::vector<std::pair<int,int>>(1,std::make_pair(1,2));
// 	_tasks_to_agents_list[6] = std::vector<std::pair<int,int>>(1,std::make_pair(2,0));
// 	_tasks_to_agents_list[7] = std::vector<std::pair<int,int>>(1,std::make_pair(2,1));
// 	_tasks_to_agents_list[8] = std::vector<std::pair<int,int>>(1,std::make_pair(2,2));
// 	_tasks_to_agents_list[9] = std::vector<std::pair<int,int>>(1,std::make_pair(3,0));
// 	_tasks_to_agents_list[10] = std::vector<std::pair<int,int>>(1,std::make_pair(3,1));
// 	_tasks_to_agents_list[11] = std::vector<std::pair<int,int>>(1,std::make_pair(3,2));

// 	PCSolver p;

// 	auto start = high_resolution_clock::now();
// 	p.solve(init_config, _tasks_list, _tasks_to_agents_list);
// 	auto stop = high_resolution_clock::now();
// 	std::chrono::duration<double, std::micro> dur = (stop - start);
// 	std::cout << dur.count()/1000000.0 << std::endl;

// 	return 0;
// }

// int main(int argc, char *argv[])
// {
// 	Pair edge_array[4] = { Pair(0,1), Pair(1,2), Pair(3, 5), Pair(4, 5) };
		
// 	PrecedenceConstraintGraph G(6);

// 	property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
		
// 	float eps = 0.0625;

// 	name[0] = meta_data (std::make_pair(eps*5, eps*2), std::make_pair(eps*6, eps*5), std::vector <int> (1, 0), 0);
// 	name[1] = meta_data (std::make_pair(eps*6, eps*5), std::make_pair(eps*6, eps*6), std::vector <int> (1, 0), 1);
// 	name[2] = meta_data (std::make_pair(eps*6, eps*6), std::make_pair(eps*6, eps*7), std::vector <int> (1, 0), 2);

// 	name[3] = meta_data (std::make_pair(eps*5, eps*1), std::make_pair(eps*6, eps*2), std::vector <int> (1, 1), 3);
// 	name[4] = meta_data (std::make_pair(eps*6, eps*1), std::make_pair(eps*6, eps*2), std::vector <int> (1, 2), 4);

// 	std::vector <int> agent_list(2, 1);
// 	agent_list[1]=2;
// 	name[5] = meta_data (std::make_pair(eps*6, eps*2), std::make_pair(eps*6, eps*8), agent_list, 5);
//   	for (int i = 0; i < 4; ++i)
// 		add_edge(edge_array[i].first, edge_array[i].second, G);

// 	int numAgents = 3;
// 	int numTasks = 6;

// 	Eigen::VectorXd init_config(2*numAgents);
// 	init_config << eps*5,  eps*2, eps*5, eps*1, eps*6, eps*1;

// 	std::vector<std::vector<std::pair<int,std::pair<Eigen::VectorXd,Eigen::VectorXd>>>> _tasks_list(numAgents);
// 	std::vector< PCVertex > c;
// 	topological_sort(G, std::back_inserter(c));

// 	for ( std::vector< PCVertex >::reverse_iterator ii=c.rbegin(); ii!=c.rend(); ++ii)
// 	{
// 		// std::cout << std::endl;
// 		meta_data vertex = get(name, *ii);

// 		int task_id = vertex.task_id;
// 		// std::cout << task_id << std::endl;

// 		Eigen::VectorXd start_config(2);
// 		start_config[0] = vertex.start.first;
// 		start_config[1] = vertex.start.second;

// 		Eigen::VectorXd goal_config(2);
// 		goal_config[0] = vertex.goal.first;
// 		goal_config[1] = vertex.goal.second;

// 		std::vector <int> agent_list = vertex.agent_list;
// 		for (auto agentNum: agent_list){
// 			// std::cout << agentNum << std::endl;
// 			_tasks_list[agentNum].push_back(std::make_pair(task_id, std::make_pair(start_config, goal_config)));
// 		}
// 		// std::cout << std::endl;
// 	}

// 	std::vector<std::vector<std::pair<int,int>>> _tasks_to_agents_list(6);

// 	_tasks_to_agents_list[0] = std::vector<std::pair<int,int>>(1,std::make_pair(0,0));
// 	_tasks_to_agents_list[1] = std::vector<std::pair<int,int>>(1,std::make_pair(0,1));
// 	_tasks_to_agents_list[2] = std::vector<std::pair<int,int>>(1,std::make_pair(0,2));
// 	_tasks_to_agents_list[3] = std::vector<std::pair<int,int>>(1,std::make_pair(1,0));
// 	_tasks_to_agents_list[4] = std::vector<std::pair<int,int>>(1,std::make_pair(2,0));
// 	_tasks_to_agents_list[5] = std::vector<std::pair<int,int>>{std::make_pair(1,1),std::make_pair(2,1)};

// 	PCSolver p;

// 	auto start = high_resolution_clock::now();
// 	p.solve(init_config, _tasks_list,_tasks_to_agents_list);
// 	auto stop = high_resolution_clock::now();
// 	std::chrono::duration<double, std::micro> dur = (stop - start);
// 	std::cout << dur.count()/1000000.0 << std::endl;
// 	return 0;	
// }

int main(int argc, char *argv[])
{
	Pair edge_array[4] = { Pair(0,1), Pair(1,4), Pair(1, 3), Pair(2, 3) };
		
	PrecedenceConstraintGraph G(5);

	property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
		
	float eps = 0.0625;

	name[0] = meta_data (std::make_pair(eps*1, eps*2), std::make_pair(eps*3, eps*2), std::vector <int> (1, 0), 0);

	name[1] = meta_data (std::make_pair(eps*4, eps*3), std::make_pair(eps*6, eps*2), std::vector <int> {0,1}, 1);

	name[2] = meta_data (std::make_pair(eps*4, eps*4), std::make_pair(eps*5, eps*2), std::vector <int> (1, 2), 2);

	name[3] = meta_data (std::make_pair(eps*6, eps*3), std::make_pair(eps*5, eps*1), std::vector <int> {1,2}, 3);

	name[4] = meta_data (std::make_pair(eps*7, eps*4), std::make_pair(eps*6, eps*1), std::vector <int> (1, 0), 4);

  	for (int i = 0; i < 4; ++i)
		add_edge(edge_array[i].first, edge_array[i].second, G);

	int numAgents = 3;

	Eigen::VectorXd init_config(2*numAgents);
	init_config << eps*1, eps*1, eps*4, eps*1, eps*3, eps*1;

	// Space Information
	// cv::Mat image = cv::imread("./src/CMAPF/include/CMAPF/test_final.png", 0);
	cv::Mat image = cv::imread("./src/CMAPF/data/obstacles/0.png", 0);
	std::string graph_file = std::string("./src/CMAPF/data/graphs/graph0.graphml");

	std::vector<std::string> graph_files;
	for(int agent_id=0; agent_id<numAgents;agent_id++)
		graph_files.push_back(graph_file);
	
	// Setup planner
	CBS planner(G,image,numAgents,graph_files,init_config);

	auto start = high_resolution_clock::now();
	std::vector<std::vector<Eigen::VectorXd>> path = planner.solve();
	auto stop = high_resolution_clock::now();
	std::chrono::duration<double, std::micro> dur = (stop - start);
	planner.printStats();
	std::cout << dur.count()/1000000.0 << std::endl;
	return 0;	
}