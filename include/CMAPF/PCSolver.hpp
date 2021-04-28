 
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

	int mCount = 0;

	bool checkpathpossible(PrecedenceConstraintGraph &G, int &numAgents, int &numRobots)
	{
		mCount +=1;
		std::cout << "PC Iteration: "<<mCount<<std::endl;
		// if(mCount < 2800)
		// 	return false;

		container c;
		topological_sort(G, std::back_inserter(c));
		property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);

		// int numAgents = 12;
		Eigen::VectorXd start_config(numAgents*2);
		Eigen::VectorXd goal_config(numAgents*2);

		std::vector<int> startTimesteps;
		std::vector<int> goalTimesteps;


		int j = 0;
		for ( container::reverse_iterator ii=c.rbegin(); ii!=c.rend(); ++ii)
		{
			// std::cout << std::endl;
			meta_data vertex = get(name, *ii);
			startTimesteps.push_back(vertex.start_time);
			goalTimesteps.push_back(vertex.end_time + vertex.slack);
			// std::cout << vertex.start_time << " ";
			// std::cout << vertex.end_time << " ";   
			// for(auto agent: vertex.agent_list){
			// 	std::cout << agent << " ";   
			// }
			// std::cout << vertex.slack << " ";   
			// cout << boost::index(*ii) << std::endl;

			start_config[j] = vertex.start.first;
			start_config[j+1] = vertex.start.second;

			goal_config[j] = vertex.goal.first;
			goal_config[j+1] = vertex.goal.second;

			j+=2;
		}
		// std::cout <<std::endl;


		// Space Information
		cv::Mat image = cv::imread("./src/CMAPF/include/CMAPF/test_final.png", 0);
		// cv::Mat image = cv::imread("./src/CMAPF/data/obstacles/0.png", 0);
		std::string graph_file = std::string("./src/CMAPF/data/graphs/graph0.graphml");

		std::vector<std::string> graph_files;
		for(int agent_id=0; agent_id<numAgents;agent_id++)
			graph_files.push_back(graph_file);
		
		// Setup planner
		CBS planner(image,numAgents,graph_files,start_config,goal_config,startTimesteps,goalTimesteps);

		// std::cout<<"PRESS [ENTER} TO CALL SOLVE!"<<std::endl;std::cin.get();
		std::vector<std::vector<Eigen::VectorXd>> path = planner.solve();
		
		if(path[0].size() == 0)
			return false;

		std::vector<std::vector< Eigen::VectorXd>> agent_paths(numRobots,std::vector< Eigen::VectorXd>());

		int task_count = 0;
		for ( container::reverse_iterator ii=c.rbegin(); ii!=c.rend(); ++ii)
		{
			// std::cout << std::endl;
			meta_data vertex = get(name, *ii);
			for(int agent = 0; agent < vertex.agent_list.size(); agent++){
				if(agent_paths[vertex.agent_list[agent]].size() == 0)
					agent_paths[vertex.agent_list[agent]] = path[task_count];
				else
				{
					for(int i=1; i<path[task_count].size(); i++)
					{
						agent_paths[vertex.agent_list[agent]].push_back(path[task_count][i]);
					}
				}
			}
			
			task_count++;
		}

		for(int i=0; i<agent_paths.size(); i++)
			std::cout<<"Path size for agent "<<i<<" = "<<agent_paths[i].size()<<std::endl;
		// std::cin.get();

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

		std::cout<<"Press [ENTER] to display path: \n";
		std::cin.get();
		planner.mNumAgents = numRobots;
		planner.displayPath(path_configs);

		return true;
	}

	bool generatePaths(PrecedenceConstraintGraph &G, PrecedenceConstraintGraph &G_T, container &c, container::iterator ii, int &numAgents, int &numRobots){
		property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
		property_map<PrecedenceConstraintGraph, meta_data_t>::type name_t = get(meta_data_t(), G_T);
			
		if(c.rbegin().base()-1 == ii){
			return checkpathpossible(G, numAgents, numRobots);
		}

		container predecessors;
		PCOutEdgeIter ei, ei_end;
		for (boost::tie(ei, ei_end) = out_edges(*ii, G_T); ei != ei_end; ++ei) 
		{
				PCVertex curPred = target(*ei, G_T);
				predecessors.push_back(curPred);
		}

		if(predecessors.size() == 0){
			return generatePaths(G, G_T, c, ii+1, numAgents, numRobots);
		}

		meta_data *curr_vertex = &get(name, *ii); 

		if(generatePaths(G, G_T, c, ii+1, numAgents, numRobots))
			return true;
		
		int slack = curr_vertex->slack;
		int start_time = curr_vertex->start_time;
		int end_time = curr_vertex->end_time;

		while(curr_vertex->slack){
				curr_vertex->slack--;
				curr_vertex->start_time++;
				curr_vertex->end_time++;
				for(auto pred:predecessors){
					meta_data *vertex = &get(name, pred);
					vertex->slack+=1;
				}
				if(generatePaths(G, G_T, c, ii+1, numAgents, numRobots))
					return true;
		}
		curr_vertex->slack = slack;
		curr_vertex->start_time = start_time;
		curr_vertex->end_time = end_time;
		for(auto pred:predecessors){
			meta_data *vertex = &get(name, pred);
			vertex->slack-=slack;
		}
		return false;
	}

	bool ICTS(PrecedenceConstraintGraph &G, int maxIter, int numAgents, int numRobots){
		property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
		container c;
		topological_sort(G, std::back_inserter(c));

		PrecedenceConstraintGraph G_T;

		transpose_graph(G, G_T);
		for(int i=0; i<maxIter; i++){

			if(generatePaths(G, G_T, c, c.begin(), numAgents, numRobots)){
				return true;
			}

			PCVertexIter v, vend;
			for (boost::tie(v, vend) = vertices(G); v != vend; ++v) {

				container successors;
				PCOutEdgeIter ei, ei_end;

				for (boost::tie(ei, ei_end) = out_edges(*v, G); ei != ei_end; ++ei) 
				{
						PCVertex curSuc = target(*ei, G);
						successors.push_back(curSuc);
				}

				if(successors.size() == 0){
					meta_data *vertex = &get(name, *v);
					vertex->slack+=1;
				}

			}

		}
		
		return false;
	}
};


} // namespace CMAPF

#endif 
