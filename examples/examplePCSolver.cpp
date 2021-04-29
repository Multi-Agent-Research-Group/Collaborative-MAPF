// Standard C++ libraries
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>

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
#include "CMAPF/PCSolver.hpp"

namespace po = boost::program_options;
using namespace boost;
using namespace CMAPF;


// int main(int argc, char *argv[])
// {
// 	Pair edge_array[4] = { Pair(0,1), Pair(1,2),
// 													Pair(3, 5), Pair(4, 5) };
		
// 	PrecedenceConstraintGraph G(6);

// 	property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
		
// 	float eps = 0.0625;

// 	name[0] = meta_data (std::make_pair(eps*8, eps*2), std::make_pair(eps*6, eps*5), std::vector <int> (1, 0), 0, 5, 0);
// 	name[1] = meta_data (std::make_pair(eps*6, eps*5), std::make_pair(eps*6, eps*6), std::vector <int> (1, 0), 5, 6, 0);
// 	name[2] = meta_data (std::make_pair(eps*6, eps*6), std::make_pair(eps*6, eps*7), std::vector <int> (1, 0), 6, 7, 1);

// 	name[3] = meta_data (std::make_pair(eps*5, eps*1), std::make_pair(eps*6, eps*2), std::vector <int> (1, 1), 0, 2, 0);
// 	name[4] = meta_data (std::make_pair(eps*6, eps*1), std::make_pair(eps*6, eps*2), std::vector <int> (1, 2), 0, 1, 1);

// 	name[5] = meta_data (std::make_pair(eps*6, eps*2), std::make_pair(eps*6, eps*8), std::vector <int> {1,2}, 2, 8, 0);

//   	for (int i = 0; i < 4; ++i)
// 		add_edge(edge_array[i].first, edge_array[i].second, G);

// 	PCSolver p;

// 	auto start = high_resolution_clock::now();
// 	p.ICTS(G, 1, 6, 3);
// 	auto stop = high_resolution_clock::now();
// 	auto duration = duration_cast<milliseconds>(stop - start);
// 	cout << duration.count() << endl;

// 	return 0;	
// }

// int main(int argc, char *argv[])
// {
// 	Pair edge_array[4] = { Pair(0,1), Pair(1,2),
// 													Pair(3, 4), Pair(4, 5) };
		
// 	PrecedenceConstraintGraph G(6);

// 	property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
		
// 	float eps = 0.0625;

// 	name[0] = meta_data (std::make_pair(eps*5, eps*2), std::make_pair(eps*6, eps*5), std::vector <int> (1, 0), 0, 4, 0);
// 	name[1] = meta_data (std::make_pair(eps*6, eps*5), std::make_pair(eps*6, eps*6), std::vector <int> (1, 0), 4, 5, 0);
// 	name[2] = meta_data (std::make_pair(eps*6, eps*6), std::make_pair(eps*6, eps*7), std::vector <int> (1, 0), 5, 6, 2);

// 	name[3] = meta_data (std::make_pair(eps*5, eps*1), std::make_pair(eps*6, eps*3), std::vector <int> (1, 1), 0, 3, 0);
// 	name[4] = meta_data (std::make_pair(eps*6, eps*3), std::make_pair(eps*6, eps*4), std::vector <int> (1, 1), 3, 4, 0);
// 	name[5] = meta_data (std::make_pair(eps*6, eps*4), std::make_pair(eps*6, eps*8), std::vector <int> (1, 1), 4, 8, 0);

//   	for (int i = 0; i < 4; ++i)
// 		add_edge(edge_array[i].first, edge_array[i].second, G);

// 	PCSolver p;

// 	auto start = high_resolution_clock::now();
// 	p.ICTS(G, 1, 6, 2);
// 	auto stop = high_resolution_clock::now();
// 	auto duration = duration_cast<milliseconds>(stop - start);
// 	cout << duration.count() << endl;

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

// 	name[0] = meta_data (std::make_pair(eps*5, eps*2), std::make_pair(eps*6, eps*9), std::vector <int> (1, 0), 0, 8, 0);
// 	name[1] = meta_data (std::make_pair(eps*6, eps*9), std::make_pair(eps*6, eps*10), std::vector <int> (1, 0), 8, 9, 0);
// 	name[2] = meta_data (std::make_pair(eps*6, eps*10), std::make_pair(eps*6, eps*11), std::vector <int> (1, 0), 9, 10, 6);

// 	name[3] = meta_data (std::make_pair(eps*8, eps*1), std::make_pair(eps*6, eps*7), std::vector <int> (1, 1), 0, 8, 0);
// 	name[4] = meta_data (std::make_pair(eps*6, eps*7), std::make_pair(eps*6, eps*8), std::vector <int> (1, 1), 8, 9, 0);
// 	name[5] = meta_data (std::make_pair(eps*6, eps*8), std::make_pair(eps*6, eps*12), std::vector <int> (1, 1), 9, 13, 3);

// 	name[6] = meta_data (std::make_pair(eps*5, eps*1), std::make_pair(eps*6, eps*5), std::vector <int> (1, 2), 0, 5, 0);
// 	name[7] = meta_data (std::make_pair(eps*6, eps*5), std::make_pair(eps*6, eps*6), std::vector <int> (1, 2), 5, 6, 0);
// 	name[8] = meta_data (std::make_pair(eps*6, eps*6), std::make_pair(eps*6, eps*13), std::vector <int> (1, 2), 6, 13, 3);

// 	name[9] = meta_data (std::make_pair(eps*8, eps*4), std::make_pair(eps*6, eps*3), std::vector <int> (1, 3), 0, 5, 0);
// 	name[10] = meta_data (std::make_pair(eps*6, eps*3), std::make_pair(eps*6, eps*4), std::vector <int> (1, 3), 5, 6, 0);
// 	name[11] = meta_data (std::make_pair(eps*6, eps*4), std::make_pair(eps*6, eps*14), std::vector <int> (1, 3), 6, 16, 0);

// 	for (int i = 0; i < 8; ++i)
// 	add_edge(edge_array[i].first, edge_array[i].second, G);

// 	auto start = high_resolution_clock::now();
// 	PCSolver p(G, 1, 12, 4);
// 	auto stop = high_resolution_clock::now();
// 	auto duration = duration_cast<milliseconds>(stop - start);
// 	cout << duration.count() << endl;

// 	return 0;
// 	// std::cout << count << std::endl;
// }

// int main(int argc, char *argv[])
// {
// 	Pair edge_array[11] = { Pair(0,1), Pair(1,2), Pair(2, 4), Pair(3, 4), 
// 							Pair(4, 5), Pair(5, 6), Pair(4, 10), Pair(10, 11),
// 							Pair(7, 8), Pair(8, 9), Pair(9, 11)};
		
// 	PrecedenceConstraintGraph G(12);

// 	property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
		
// 	float eps = 0.0625;

// 	name[0] = meta_data (std::make_pair(eps*1, eps*1), std::make_pair(eps*1, eps*2), std::vector <int> (1, 0), 0, 1, 0);
// 	name[1] = meta_data (std::make_pair(eps*1, eps*2), std::make_pair(eps*3, eps*2), std::vector <int> (1, 0), 1, 3, 0);
// 	name[2] = meta_data (std::make_pair(eps*3, eps*2), std::make_pair(eps*4, eps*3), std::vector <int> (1, 0), 3, 5, 0);

// 	name[3] = meta_data (std::make_pair(eps*4, eps*1), std::make_pair(eps*4, eps*3), std::vector <int> (1, 1), 0, 2, 3);

// 	name[4] = meta_data (std::make_pair(eps*4, eps*3), std::make_pair(eps*6, eps*2), std::vector <int> {0,1}, 5, 8, 0);


// 	name[5] = meta_data (std::make_pair(eps*6, eps*2), std::make_pair(eps*7, eps*4), std::vector <int> (1, 0), 8, 11, 0);
// 	name[6] = meta_data (std::make_pair(eps*7, eps*4), std::make_pair(eps*6, eps*1), std::vector <int> (1, 0), 11, 15, 0);

// 	name[7] = meta_data (std::make_pair(eps*3, eps*1), std::make_pair(eps*4, eps*4), std::vector <int> (1, 2), 0, 4, 0);
// 	name[8] = meta_data (std::make_pair(eps*4, eps*4), std::make_pair(eps*5, eps*2), std::vector <int> (1, 2), 4, 7, 0);
// 	name[9] = meta_data (std::make_pair(eps*5, eps*2), std::make_pair(eps*6, eps*3), std::vector <int> (1, 2), 7, 9, 0);

// 	name[10] = meta_data (std::make_pair(eps*6, eps*2), std::make_pair(eps*6, eps*3), std::vector <int> (1, 1), 8, 9, 0);

// 	name[11] = meta_data (std::make_pair(eps*6, eps*3), std::make_pair(eps*5, eps*1), std::vector <int> {1,2}, 9, 12, 3);

//   	for (int i = 0; i < 11; ++i)
// 		add_edge(edge_array[i].first, edge_array[i].second, G);


// 	// std::cout << "hello\n"; std::cin.get();
// 	auto start = high_resolution_clock::now();
// 	PCSolver p(G, 1, 12, 3);
// 	auto stop = high_resolution_clock::now();
// 	auto duration = duration_cast<milliseconds>(stop - start);
// 	cout << duration.count() << endl;

// 	return 0;
// 	// std::cout << count << std::endl;
// }

int main(int argc, char *argv[])
{
	po::options_description desc("ICTS-CBS Planner Options");
	desc.add_options()
			("help,h", "produce help message")
			("file,f", po::value<std::string>()->default_value("./src/CMAPF/data/sample_problems/test_2.txt"), "Path to PC Graph Metadata File")
			("graph,g", po::value<std::string>()->default_value("./src/CMAPF/data/graphs/graph0.graphml"), "Path to Graph File")
			("obstacles,o", po::value<std::string>()->default_value("./src/CMAPF/data/obstacles/env_obstacles.png"), "Path to Obstacle Image File")
	;

	// Read arguments
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help"))
	{
			std::cout << desc << std::endl;
			return 1;
	}

	std::string file_name(vm["file"].as<std::string>());
	if (file_name == "")
		file_name = "./src/CMAPF/data/sample_problems/test_2.txt";

	std::string graph_file_name(vm["graph"].as<std::string>());
	if (graph_file_name == "")
		graph_file_name = "./src/CMAPF/data/graphs/graph0.graphml";

	std::string obstacle_file_name(vm["obstacles"].as<std::string>());
	if (file_name == "")
		obstacle_file_name = "./src/CMAPF/data/obstacles/env_obstacles.png";

	ifstream cin(file_name);

	int num_agents; cin >> num_agents;
	int num_edges; cin >> num_edges;
	std::cout << num_agents << std::endl;
	std::cout << num_edges << std::endl;

	float eps; cin >> eps;
	std::cout << eps << std::endl;

	int max_iter, num_robots; cin >> max_iter >> num_robots;

	Pair edge_array[num_edges];
	for(int i=0; i<num_edges; i++){
		int v1, v2; cin >> v1 >> v2;
		edge_array[i] = Pair(v1, v2);
	}
		
	PrecedenceConstraintGraph G(num_agents);

	property_map<PrecedenceConstraintGraph, meta_data_t>::type data = get(meta_data_t(), G);
		
	for(int i=0; i<num_agents; i++){
		//Read Start Point
		int x1, y1; cin >> x1 >> y1;
		//Read Goal Point
		int x2, y2; cin >> x2 >> y2;
		//Read Collaborating Agents
		int num_colab; cin >> num_colab; vector < int > agent_list(num_colab);
		for(int j=0; j<num_colab; j++) cin >> agent_list[j];
		//Read Start Time, End Time, Slack
		int start, goal, slack; cin >> start >> goal >> slack;

		data[i] = meta_data (std::make_pair(eps*x1, eps*y1), std::make_pair(eps*x2, eps*y2), agent_list, start, goal, slack);
	}

  	for (int i = 0; i < num_edges; ++i)
		add_edge(edge_array[i].first, edge_array[i].second, G);


	// std::cout << "hello\n"; std::cin.get();
	auto start = high_resolution_clock::now();
	PCSolver p(G, max_iter, num_agents, num_robots, graph_file_name, obstacle_file_name);
	p.solve();
	auto stop = high_resolution_clock::now();
	auto duration = duration_cast<milliseconds>(stop - start);
	std::cout << "Time Taken by the algorithm = " << duration.count() << " milli seconds" << std::endl;

	return 0;
	// std::cout << count << std::endl;
}