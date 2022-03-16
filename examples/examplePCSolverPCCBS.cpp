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

std::vector<std::vector<bool>> extract_map(const std::string map_fname)
{
    std::vector<std::vector<bool>> extracted_map;
    std::ifstream ifs(map_fname);
    std::string line;
    int height, width;
    
    std::getline(ifs,line); // Assuming map type is octile, so skip
    
    std::getline(ifs,line);
    int i = 0;
    for (; i < line.length(); i++) if (std::isdigit(line[i])) break;
    line = line.substr(i,line.length() - i);
    height = std::atoi(line.c_str());
    // std::cout << "Height: " << height;

    std::getline(ifs,line);    
    for (i = 0; i < line.length(); i++) if (std::isdigit(line[i])) break;
    line = line.substr(i,line.length() - i);
    width = std::atoi(line.c_str());
    // std::cout << " Width: " << width << std::endl;

    std::getline(ifs,line); // This is the line that says "map"

    for (i = 0; i < height; i++)
    {
        std::vector<bool> map_line;
        std::getline(ifs,line);
        for (int j = 0; j < width; j++)
        {
            map_line.push_back(line[j] == '.');
            // std::cout << map_line.back() << " ";
        }
        // std::cout << std::endl;

        extracted_map.push_back(map_line);
    }

    return extracted_map;
}

std::vector < std::vector < int>> preprocess_graph(std::string weights_file, int mNumVertices)
{
	std::vector < std::vector < int>> edgeWeights(mNumVertices, std::vector <int> (mNumVertices, INF));
	std::ifstream ifs(weights_file);
	for(int i=0; i<mNumVertices; i++){
		for(int j=0; j<mNumVertices; j++){
			int v1; ifs >> v1;
			int v2; ifs >> v2;
			std::string weight; ifs >> weight;
			edgeWeights[v1][v2] = std::stoi(weight);
			edgeWeights[v2][v1] = std::stoi(weight);
		}
	}
	return edgeWeights;
}

int main(int argc, char *argv[])
{
	po::options_description desc("Collaborative CBS Planner Options");
	desc.add_options()
			("help,h", "produce help message")
			("map_file,m",po::value<std::string>()->required(), "path to map file")
			("graph_file,g",po::value<std::string>()->required(), "path to map file")
			("weights_file,w",po::value<std::string>()->required(), "path to map file")
			("file,f", po::value<std::string>()->default_value("./src/CMAPF/data/sample_problems/test_6.txt"), "Path to PC Graph Metadata File")
			("type,t", po::value<std::string>()->default_value("astar"), "Planner Type")
			("image_file,i", po::value<std::string>()->default_value("videos_pccbs/"), "Planner Type")
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

	std::string problem_file(vm["file"].as<std::string>());
	std::string image_file(vm["image_file"].as<std::string>());
	std::string graph_file(vm["graph_file"].as<std::string>());
	std::string map_file(vm["map_file"].as<std::string>());
	std::string weights_file(vm["weights_file"].as<std::string>());
	std::string type(vm["type"].as<std::string>());

	ifstream cin(problem_file);

	int num_tasks; cin >> num_tasks;
	int num_edges; cin >> num_edges;
	// std::cout << num_agents << std::endl;
	// std::cout << num_edges << std::endl;

	int num_robots; cin >> num_robots;

	Eigen::VectorXd init_config(2*num_robots);

	Pair edge_array[num_edges];
	for(int i=0; i<num_edges; i++){
		int v1, v2; cin >> v1 >> v2;
		// std::cout << v1 << " " << v2 << std::endl;
		edge_array[i] = Pair(v1, v2);
	}
		
	PrecedenceConstraintGraph PCG(num_tasks);

	property_map<PrecedenceConstraintGraph, meta_data_t>::type data = get(meta_data_t(), PCG);
		
	for(int i=0; i<num_tasks; i++){
		//Read Start Point
		int x1, y1; cin >> x1 >> y1;
		//Read Goal Point
		int x2, y2; cin >> x2 >> y2;
		//Read Collaborating Agents
		int num_colab; cin >> num_colab; vector < int > agent_list(num_colab);
		for(int j=0; j<num_colab; j++) cin >> agent_list[j];
		// std::cout << x1 << " " << y1 << std::endl;
		data[i] = meta_data (std::make_pair(x1, y1), std::make_pair(x2, y2), agent_list);
	}

  	for (int i = 0; i < num_edges; ++i)
		add_edge(edge_array[i].first, edge_array[i].second, PCG);

	// std::vector<std::string> graph_files;
	// for(int agent_id=0; agent_id<num_robots;agent_id++)
	// 	graph_files.push_back(graph_file_name);
	
	Graph graph;
	create_vertices(graph,get(&VProp::state,graph),graph_file,2,get(&EProp::prior,graph));
	create_edges(graph,get(&EProp::length,graph));

	int mNumVertices = num_vertices(graph);

	std::vector <std::vector <bool>> map = extract_map(map_file);

	std::vector < std::vector < int>> edgeWeights = preprocess_graph(weights_file, mNumVertices);

	cv::Mat image = cv::imread(image_file, 0);

	CBS planner(PCG,graph,map,num_robots,edgeWeights,type, image);

	auto start = high_resolution_clock::now();
	std::vector<std::vector<Eigen::VectorXd>> path = planner.solve();
	auto stop = high_resolution_clock::now();
	std::chrono::duration<double, std::micro> dur = (stop - start);
	planner.printStats();
	return 0;
}