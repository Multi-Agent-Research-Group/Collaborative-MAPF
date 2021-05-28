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
#include "../include/utils/graphFileUtils.hpp"

namespace po = boost::program_options;
using namespace BGL_DEFINITIONS;

int main(int argc, char *argv[])
{
	po::options_description desc("2D Map Planner Options");
	desc.add_options()
			("help,h", "produce help message")
			("graph,g", po::value<std::string>()->default_value(""), "Path to Graph File")
			("obstaclefile,o", po::value<std::string>()->default_value(""), "Path to Obstacles File")
			("source,s", po::value<std::vector<double> >()->multitoken(), "source configuration")
			("target,t", po::value<std::vector<double> >()->multitoken(), "target configuration")
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

	std::string graph_file_name(vm["graph"].as<std::string>());
	if (graph_file_name == "")
		graph_file_name = "/home/rajat/melodic_ws/src/MINT/data/graphs/halton_2d_withedges.graphml";
	
	std::string obstacle_file(vm["obstaclefile"].as<std::string>());
	if (obstacle_file == "")
		obstacle_file = "/home/rajat/melodic_ws/src/MINT/data/obstacles/circle2D.png";

	std::vector<double> source_config(vm["source"].as<std::vector< double> >());
	std::vector<double> target_config(vm["target"].as<std::vector< double> >());

	// Space Information

	cv::Mat image = cv::imread(obstacle_file, 1);
	int numberOfRows = image.rows;
	int numberOfColumns = image.cols;

	Graph graph;
	create_vertices(graph,get(&VProp::state,graph),graph_file_name,2,get(&EProp::prior,graph));
	create_edges(graph,get(&EProp::length,graph));

	VertexIter vi, vi_end;
	size_t i=0;

	Eigen::VectorXd left_start_config(2);
	left_start_config << source_config[0], source_config[1];

	Eigen::VectorXd right_start_config(2);
	right_start_config << source_config[2], source_config[3];

	Eigen::VectorXd left_goal_config(2);
	left_goal_config << target_config[0], target_config[1];

	Eigen::VectorXd right_goal_config(2);
	right_goal_config << target_config[2], target_config[3];

	int left_start_index = -1;
	int left_goal_index = -1;

	int right_start_index = -1;
	int right_goal_index = -1;

	for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi,++i)
	{
		put(&VProp::vertex_index,graph,*vi,i);
		if(left_start_config.isApprox(graph[*vi].state))
			left_start_index = i;
		if(left_goal_config.isApprox(graph[*vi].state))
			left_goal_index = i;
		if(right_start_config.isApprox(graph[*vi].state))
			right_start_index = i;
		if(right_goal_config.isApprox(graph[*vi].state))
			right_goal_index = i;
	}

	std::cout<<"Left Start Index: "<<left_start_index<<std::endl;
	std::cout<<"Left Goal Index: "<<left_goal_index<<std::endl;

	std::cout<<"Right Start Index: "<<right_start_index<<std::endl;
	std::cout<<"Right Goal Index: "<<right_goal_index<<std::endl;

	EdgeIter ei, ei_end;
	for(boost::tie(ei,ei_end) = edges(graph); ei!=ei_end;++ei)
	{
		cv::Point source_Point((int)(graph[source(*ei,graph)].state[0]*numberOfColumns), 
			(int)((1-graph[source(*ei,graph)].state[1])*numberOfColumns));
		cv::Point target_Point((int)(graph[target(*ei,graph)].state[0]*numberOfColumns), 
			(int)((1-graph[target(*ei,graph)].state[1])*numberOfColumns));
		cv::line(image, source_Point, target_Point, cv::Scalar(0, 255, 255), 2);
	}

	// VertexIter vi, vi_end;
	for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
	{
		double x_point = graph[*vi].state[0]*numberOfColumns;
		double y_point = (1 - graph[*vi].state[1])*numberOfRows;
		cv::Point centre_Point((int)x_point, (int)y_point);
		cv::circle(image, centre_Point, 3,  cv::Scalar(0, 255, 0), -1);
	}
	{
		double x_point = left_start_config[0]*numberOfColumns;
		double y_point = (1 - left_start_config[1])*numberOfRows;
		cv::Point centre_Point((int)x_point, (int)y_point);
		cv::circle(image, centre_Point, 5,  cv::Scalar(0, 0, 200), -1);		
	}
	{
		double x_point = left_goal_config[0]*numberOfColumns;
		double y_point = (1 - left_goal_config[1])*numberOfRows;
		cv::Point centre_Point((int)x_point, (int)y_point);
		cv::circle(image, centre_Point, 5,  cv::Scalar(0, 0, 255), -1);		
	}
	{
		double x_point = right_start_config[0]*numberOfColumns;
		double y_point = (1 - right_start_config[1])*numberOfRows;
		cv::Point centre_Point((int)x_point, (int)y_point);
		cv::circle(image, centre_Point, 5,  cv::Scalar(200, 0, 0), -1);		
	}
	{
		double x_point = right_goal_config[0]*numberOfColumns;
		double y_point = (1 - right_goal_config[1])*numberOfRows;
		cv::Point centre_Point((int)x_point, (int)y_point);
		cv::circle(image, centre_Point, 5,  cv::Scalar(255, 0, 0), -1);		
	}		


	cv::namedWindow("Graph Visualization",cv::WINDOW_NORMAL);
	cv::imshow("Graph Visualization", image);
	cv::waitKey(0);

	// display_graph(graph);
	
	return 0;
}