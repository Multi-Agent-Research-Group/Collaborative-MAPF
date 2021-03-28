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
#include "DENSIFY/DENSIFY.hpp"

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
	po::options_description desc("2D Map Planner Options");
	desc.add_options()
			("help,h", "produce help message")
			("left_graph,l", po::value<std::string>()->default_value(""), "Path to Left Graph")
			("right_graph,r", po::value<std::string>()->default_value(""), "Path to Right Graph")
			("obstaclefile,o", po::value<std::string>()->default_value(""), "Path to Obstacles File")
			("source,s", po::value<std::vector<double> >()->multitoken(), "source configuration")
			("target,t", po::value<std::vector<double> >()->multitoken(), "target configuration")
			("display,d", po::bool_switch()->default_value(false), "Enable to display final path")
			("insert_start_goal,i", po::bool_switch()->default_value(false), "Enable to insert start and goal into graphs")
			("dijkstra", po::bool_switch()->default_value(false), "Enable to use dijkstra to precompute heuristic for A*")
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

	std::string left_graph_file(vm["left_graph"].as<std::string>());
	std::string right_graph_file(vm["right_graph"].as<std::string>());
	if (left_graph_file == "")
		left_graph_file = "/home/rajat/melodic_ws/src/C-MINT/data/graphs/halton_2d_withedges.graphml";
	if (right_graph_file == "")
		right_graph_file = "/home/rajat/melodic_ws/src/C-MINT/data/graphs/halton_2d_withedges.graphml";
	std::string obstacle_file(vm["obstaclefile"].as<std::string>());
	if (obstacle_file == "")
		obstacle_file = "/home/rajat/melodic_ws/src/C-MINT/data/obstacles/circle2D.png";
	std::vector<double> source(vm["source"].as<std::vector< double> >());
	std::vector<double> target(vm["target"].as<std::vector< double> >());
	bool display(vm["display"].as<bool>());
	bool insert_start_goal(vm["insert_start_goal"].as<bool>());
	bool dijkstra(vm["dijkstra"].as<bool>());

	// std::cout<<"LEL"<<std::endl;

	// Space Information
	cv::Mat image = cv::imread(obstacle_file, 0);
	Eigen::VectorXd start_config(4);
	start_config << source[0], source[1], source[2], source[3];

	Eigen::VectorXd goal_config(4);
	goal_config << target[0], target[1], target[2], target[3];
	
	// Setup planner
	DENSIFY::DENSIFY planner(image,left_graph_file,right_graph_file,start_config,goal_config,insert_start_goal,dijkstra);

	std::cout<<"CALLING SOLVE!"<<std::endl;
	std::vector<Eigen::VectorXd> path = planner.solve();

	// if (display)
	// 	displayPath(obstacle_file, path);
	// else
		std::cout << "Exiting Cleanly" << std::endl;
	return 0;
}