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
#include "PCCBS/CBS.hpp"

namespace po = boost::program_options;
using namespace boost;
using namespace PCCBS;

int main(int argc, char *argv[])
{
	po::options_description desc("Collaborative CBS Planner Options");
	desc.add_options()
			("help,h", "produce help message")
			("file,f", po::value<std::string>()->default_value("./src/CMAPF/data/sample_problems/test_6.txt"), "Path to PC Graph Metadata File")
			("graph,g", po::value<std::string>()->default_value("./src/CMAPF/data/new_graphs/graph0.graphml"), "Path to Graph File")
			("obstacles,o", po::value<std::string>()->default_value("./src/CMAPF/data/obstacles/env_obstacles.png"), "Path to Obstacle Image File")
			("image,i", po::value<std::string>()->default_value("videos/"), "Path to Storing Images")
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
		file_name = "./src/CMAPF/data/sample_problems/test_6.txt";

	std::string graph_file_name(vm["graph"].as<std::string>());
	if (graph_file_name == "")
		graph_file_name = "./src/CMAPF/data/new_graphs/graph0.graphml";

	std::string obstacle_file_name(vm["obstacles"].as<std::string>());
	if (file_name == "")
		obstacle_file_name = "./src/CMAPF/data/obstacles/env_obstacles.png";

	ifstream cin(file_name);

	int num_agents; cin >> num_agents;
	int num_edges; cin >> num_edges;
	// std::cout << num_agents << std::endl;
	// std::cout << num_edges << std::endl;

	double eps; cin >> eps;
	// std::cout << eps << std::endl;

	int max_iter, num_robots; cin >> max_iter >> num_robots;

	Eigen::VectorXd init_config(2*num_robots);

	for(int i=0; i<num_robots; i++)
	{
		int x,y;
		cin>>x>>y;
		init_config[i*2] = x*eps;
		init_config[i*2+1] = y*eps;
	}

	Pair edge_array[num_edges];
	for(int i=0; i<num_edges; i++){
		int v1, v2; cin >> v1 >> v2;
		edge_array[i] = Pair(v1, v2);
	}
		
	PrecedenceConstraintGraph G(num_agents);

	property_map<PrecedenceConstraintGraph, meta_data_t>::type data = get(meta_data_t(), G);

	std::vector<int> start_times;
	std::vector<int> end_times;
		
	for(int i=0; i<num_agents; i++){
		//Read Start Point
		int x1, y1; cin >> x1 >> y1;
		//Read Goal Point
		int x2, y2; cin >> x2 >> y2;
		//Read Collaborating Agents
		int num_colab; cin >> num_colab; vector < int > agent_list(num_colab);
		for(int j=0; j<num_colab; j++) cin >> agent_list[j];

		int start_time, goal_time;
		cin>>start_time >>goal_time;
		start_times.push_back(start_time);
		end_times.push_back(goal_time);
		//Read Task Id
		int task_id; cin >> task_id;

		data[i] = meta_data (std::make_pair(eps*x1, eps*y1), std::make_pair(eps*x2, eps*y2), agent_list, task_id);
	}

  	for (int i = 0; i < num_edges; ++i)
		add_edge(edge_array[i].first, edge_array[i].second, G);

	cv::Mat image = cv::imread(obstacle_file_name, 0);

	std::vector<std::string> graph_files;
	for(int agent_id=0; agent_id<num_robots;agent_id++)
		graph_files.push_back(graph_file_name);
	
	// Setup planner
	// std::cerr<<"setup!";
	CBS planner(G,image,num_robots,graph_files,init_config,
		start_times, end_times, vm["image"].as<std::string>());

	auto start = high_resolution_clock::now();
	// std::cerr<<"calling solve!";
	std::vector<std::vector<Eigen::VectorXd>> path = planner.solve();
	auto stop = high_resolution_clock::now();
	std::chrono::duration<double, std::micro> dur = (stop - start);
	// std::cout<<path.size()-1<<" ";
	planner.printStats();
	// std::cout << dur.count()/1000000.0 << std::endl;
	return 0;
	// std::cout << count << std::endl;
}