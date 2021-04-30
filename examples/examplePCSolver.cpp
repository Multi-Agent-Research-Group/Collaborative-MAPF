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

int main(int argc, char *argv[])
{
	po::options_description desc("ICTS-CBS Planner Options");
	desc.add_options()
			("help,h", "produce help message")
			("file,f", po::value<std::string>()->default_value("./src/CMAPF/data/sample_problems/ICTS/9.txt"), "Path to PC Graph Metadata File")
			("graph,g", po::value<std::string>()->default_value("./src/CMAPF/data/new_graphs/graph0.graphml"), "Path to Graph File")
			("obstacles,o", po::value<std::string>()->default_value("./src/CMAPF/data/obstacles/0.png"), "Path to Obstacle Image File")
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
		file_name = "./src/CMAPF/data/sample_problems/ICTS/9.txt";

	std::string graph_file_name(vm["graph"].as<std::string>());
	if (graph_file_name == "")
		graph_file_name = "./src/CMAPF/data/new_graphs/graph0.graphml";

	std::string obstacle_file_name(vm["obstacles"].as<std::string>());
	if (file_name == "")
		obstacle_file_name = "./src/CMAPF/data/obstacles/0.png";

	ifstream cin(file_name);

	int num_agents; cin >> num_agents;
	int num_edges; cin >> num_edges;
	std::cout << num_agents << std::endl;
	std::cout << num_edges << std::endl;

	double eps; cin >> eps;
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