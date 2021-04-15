// Standard C++ libraries
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>

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
#include "C_MINT/C_MINT.hpp"

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
	po::options_description desc("2D Map Planner Options");
	desc.add_options()
			("help,h", "produce help message")
			("graph,g", po::value<std::string>()->default_value(""), "Path to Graph")
			("num_agents,k", po::value<int>()->default_value(2), "Number of Agents")
			("obstaclefile,o", po::value<std::string>()->default_value(""), "Path to Obstacles File")
			("source,s", po::value<std::vector<double> >()->multitoken(), "source configuration")
			("target,t", po::value<std::vector<double> >()->multitoken(), "target configuration")
			("display,d", po::bool_switch()->default_value(false), "Enable to display final path")
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

	int numAgents = vm["num_agents"].as<int>();
	std::string graph_file(vm["graph"].as<std::string>());
	if (graph_file == "")
		graph_file = "/home/rajat/melodic_ws/src/C-MINT/data/graphs/halton_2d_withedges.graphml";
	std::string obstacle_file(vm["obstaclefile"].as<std::string>());
	if (obstacle_file == "")
		obstacle_file = "/home/rajat/melodic_ws/src/C-MINT/data/obstacles/circle2D.png";
	std::vector<double> source(vm["source"].as<std::vector< double> >());
	std::vector<double> target(vm["target"].as<std::vector< double> >());
	bool display(vm["display"].as<bool>());

	// std::cout<<"LEL"<<std::endl;

	// Space Information
	cv::Mat image = cv::imread(obstacle_file, 0);
	
	Eigen::VectorXd start_config(numAgents*2);
	for(int i=0; i<2*numAgents;i++)
		start_config[i] = source[i];

	Eigen::VectorXd goal_config(numAgents*2);
	for(int i=0; i<2*numAgents;i++)
		goal_config[i] = target[i];

	std::vector<std::string> graph_files;
	for(int agent_id=0; agent_id<numAgents;agent_id++)
		graph_files.push_back(graph_file);
	
	// Setup planner
	CBS::CBS planner(image,numAgents,graph_files,start_config,goal_config);

	std::cout<<"CALLING SOLVE!"<<std::endl;
	std::vector<Eigen::VectorXd> path = planner.solve();

	// if (display)
	// 	displayPath(obstacle_file, path);
	// else
		std::cout << "Exiting Cleanly" << std::endl;
	return 0;
}