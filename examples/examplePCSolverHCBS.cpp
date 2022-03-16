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
#include "HCBS/PCSolver.hpp"

namespace po = boost::program_options;
using namespace boost;
using namespace HCBS;

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

int main(int argc, char *argv[])
{
	po::options_description desc("HCBS Planner Options");
	desc.add_options()
			("help,h", "produce help message")
			("map_file,m",po::value<std::string>()->required(), "path to map file")
			("graph_file,g",po::value<std::string>()->required(), "path to map file")
			("file,f", po::value<std::string>()->default_value("./src/CMAPF/data/sample_problems/test_6.txt"), "Path to PC Graph Metadata File")
			("image_file,i", po::value<std::string>()->default_value("videos_hcbs/"), "Planner Type")
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
	std::string graph_file_name(vm["graph_file"].as<std::string>());
	std::string map_file_name(vm["map_file"].as<std::string>());

	std::vector <std::vector <bool>> map = extract_map(map_file_name);
	ifstream cin(file_name);

	int num_agents; cin >> num_agents;
	int num_edges; cin >> num_edges;
	// std::cout << num_agents << std::endl;
	// std::cout << num_edges << std::endl;

	// double eps; cin >> eps;
	// std::cout << eps << std::endl;

	int num_robots; cin >> num_robots;

	int max_iter = 10000;

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
		int start, goal, slack; 
		// cin >> start >> goal >> slack;
		start = 0; goal = 0; slack = 0;

		data[i] = meta_data (std::make_pair(x1, y1), 
			std::make_pair(x2, y2), agent_list, start, goal, slack);
	}

  	for (int i = 0; i < num_edges; ++i)
		add_edge(edge_array[i].first, edge_array[i].second, G);


	// std::cout << "hello\n"; std::cin.get();
	PCSolver p(G, max_iter, num_agents, num_robots, 
		graph_file_name, map, vm["image_file"].as<std::string>());
	auto start = high_resolution_clock::now();
	p.solve();
	// std::cerr<<"solved!";
	auto stop = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(stop - start);
	std::cout << duration.count()/1000.0<< std::endl;

	return 0;
	// std::cout << count << std::endl;
}