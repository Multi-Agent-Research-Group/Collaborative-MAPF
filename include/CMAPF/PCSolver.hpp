 
/* Authors: Rajat Kumar Jenamani */

#ifndef _PCSolver_HPP
#define _PCSolver_HPP

// STL headers
#include <vector>
#include <string> 
#include <unordered_set>
#include <queue>
#include <exception>

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/transpose_graph.hpp>
#include <boost/graph/topological_sort.hpp>

#include <algorithm>        // std::reverse
#include <cmath>            // pow, sqrt
#include <set>              // std::set
#include <assert.h>         // debug
#include <fstream>          // log
#include <chrono>           // time

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "BGLDefinitions.hpp"
#include "PCDefinitions.hpp"
#include "time_priority_queue.hpp"
#include "CBSDefinitions.hpp"
#include "LoadGraphfromFile.hpp"

#include "CBS.hpp"

#define INF std::numeric_limits<double>::infinity()

namespace CMAPF {

using namespace BGL_DEFINITIONS;

class PCSolver
{

public:
	bool solve(std::vector<std::vector<std::pair<int,std::pair<Eigen::VectorXd,Eigen::VectorXd>>>> _tasks_list)
	{
		int numAgents = _tasks_list.size();
		return false;
		// int numAgents = 12;
		// Eigen::VectorXd start_config(numAgents*2);
		// Eigen::VectorXd goal_config(numAgents*2);

		// std::vector<int> startTimesteps;
		// std::vector<int> goalTimesteps;

		// int j = 0;
		// for ( container::reverse_iterator ii=c.rbegin(); ii!=c.rend(); ++ii)
		// {
		// 	// std::cout << std::endl;
		// 	meta_data vertex = get(name, *ii);

		// 	start_config[j] = vertex.start.first;
		// 	start_config[j+1] = vertex.start.second;

		// 	goal_config[j] = vertex.goal.first;
		// 	goal_config[j+1] = vertex.goal.second;

		// 	j+=2;
		// }
		
		// std::cout <<std::endl;


		// Space Information
		cv::Mat image = cv::imread("./src/CMAPF/include/CMAPF/test_final.png", 0);
		// cv::Mat image = cv::imread("./src/CMAPF/data/obstacles/0.png", 0);
		std::string graph_file = std::string("./src/CMAPF/data/graphs/graph0.graphml");

		std::vector<std::string> graph_files;
		for(int agent_id=0; agent_id<numAgents;agent_id++)
			graph_files.push_back(graph_file);

		std::vector <pair < int , int > 
		
		
		// Setup planner
		CBS planner(image,numAgents,graph_files,start_config,_tasks_list);

		// std::cout<<"PRESS [ENTER} TO CALL SOLVE!"<<std::endl;std::cin.get();
		std::vector<std::vector<Eigen::VectorXd>> path = planner.solve();
		
		if(path[0].size() == 0)
			return false;

		std::vector<std::vector< Eigen::VectorXd>> agent_paths(4,std::vector< Eigen::VectorXd>());

		int task_count = 0;
		for ( container::reverse_iterator ii=c.rbegin(); ii!=c.rend(); ++ii)
		{
			// std::cout << std::endl;
			meta_data vertex = get(name, *ii);
			if(agent_paths[vertex.agent_list[0]].size() == 0)
				agent_paths[vertex.agent_list[0]] = path[task_count];
			else
			{
				for(int i=1; i<path[task_count].size(); i++)
					agent_paths[vertex.agent_list[0]].push_back(path[task_count][i]);
			}
			task_count++;
		}

		for(int i=0; i<agent_paths.size(); i++)
			std::cout<<"Path size for agent "<<i<<" = "<<agent_paths[i].size()<<std::endl;
		std::cin.get();

		std::vector<Eigen::VectorXd> path_configs;

		for(int i=0; i<agent_paths[0].size(); i++)
		{
			Eigen::VectorXd config(agent_paths.size()*2);
			for(int j=0; j<agent_paths.size(); j++)
			{
				config[2*j]=agent_paths[j][i][0];
				config[2*j+1]=agent_paths[j][i][1];
			}
			path_configs.push_back(config);
		}
		std::cout<<"Path config: "<<path_configs[0]<<std::endl;

		std::cout<<"Press [ENTER] to display path: ";
		std::cin.get();
		planner.mNumAgents = 4;
		planner.displayPath(path_configs);

		return true;
	}

#endif 
