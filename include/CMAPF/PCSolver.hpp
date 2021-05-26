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

#include <chrono>
using namespace std::chrono;

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

	std::chrono::duration<double, std::micro> mPreprocessTime ;
	std::chrono::duration<double, std::micro> mPlanningTime;

	high_resolution_clock::time_point mSolveStartTime;

	/// Number of agents (tasks)
	int mNumAgents; 

	/// Number of robots 
	int mNumRobots;

	container mTopologicalOrder;
	/// Path to the roadmap files.
	std::vector<std::string> mRoadmapFileNames;

	/// Source vertex.
	std::vector<Eigen::VectorXd> mStartConfig;
	std::vector<Vertex> mStartVertex;

	/// Goal vertex.
	std::vector<Eigen::VectorXd> mGoalConfig;
	std::vector<Vertex> mGoalVertex;

	PrecedenceConstraintGraph &mPCGraph;
	int mMaxIter;

	double mUnitEdgeLength = 0.04;

	PCSolver(PrecedenceConstraintGraph &G, int maxIter, int numAgents, int numRobots, std::string graph_file, std::string obstacle_file)
		: mPCGraph(G)
		, mMaxIter(maxIter)
		, mNumAgents(numAgents)
		, mNumRobots(numRobots)
	{
		topological_sort(G, std::back_inserter(mTopologicalOrder));
		property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);

		// int numAgents = 12;
		Eigen::VectorXd start_config(numAgents*2);
		Eigen::VectorXd goal_config(numAgents*2);

		std::vector<int> startTimesteps;
		std::vector<int> goalTimesteps;

		int j = 0;
		for (int i=0; i<mNumAgents; i++)
		{
			meta_data vertex = get(name, i);
			start_config[j] = vertex.start.first;
			start_config[j+1] = vertex.start.second;
			goal_config[j] = vertex.goal.first;
			goal_config[j+1] = vertex.goal.second;
			j+=2;
		}

		// std::cout << "PC Iteration: "<<mCount<<std::endl; std::cin.get();

		// Space Information
		// mImage = cv::imread("./src/CMAPF/include/CMAPF/test_final.png", 0);
		// mImage = cv::imread("./src/CMAPF/data/obstacles/0.png", 0);
		mImage = cv::imread(obstacle_file, 0);
		// std::string graph_file = std::string("./src/CMAPF/data/graphs/graph0.graphml");

		std::vector<std::string> graph_files;
		for(int agent_id=0; agent_id<numAgents;agent_id++)
			graph_files.push_back(graph_file);

		mRoadmapFileNames = graph_files;

		for(int i=0; i<mNumAgents;i++)
		{
			Eigen::VectorXd _start_config(2);
			for (int ui = i*2; ui < i*2+2; ui++)
				_start_config[ui-i*2] = start_config[ui];
			mStartConfig.push_back(_start_config);
		}

		for(int i=0; i<mNumAgents;i++)
		{
			Eigen::VectorXd _goal_config(2);
			for (int ui = i*2; ui < i*2+2; ui++)
				_goal_config[ui-i*2] = goal_config[ui];
			mGoalConfig.push_back(_goal_config);
		}

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			Graph graph;
			Vertex start_vertex;
			Vertex goal_vertex;

			create_vertices(graph,get(&VProp::state,graph),mRoadmapFileNames[agent_id],2,get(&EProp::prior,graph));
			create_edges(graph,get(&EProp::length,graph));

			VertexIter ind_vi, ind_vi_end;
			int i=0;
			for (boost::tie(ind_vi, ind_vi_end) = vertices(graph); ind_vi != ind_vi_end; ++ind_vi,++i)
			{
				put(&VProp::vertex_index,graph,*ind_vi,i);
				if(mStartConfig[agent_id].isApprox(graph[*ind_vi].state))
					start_vertex = *ind_vi;
				if(mGoalConfig[agent_id].isApprox(graph[*ind_vi].state))
					goal_vertex = *ind_vi;  
			}

			mGraphs.push_back(graph);
			mStartVertex.push_back(start_vertex);
			mGoalVertex.push_back(goal_vertex);
		}

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			preprocess_graph(mGraphs[agent_id], mGoalVertex[agent_id]);

		// std::cout << "PC Iteration: "<<mCount<<std::endl; 
		// std::cin.get();

		// ICTS(mPCGraph, mMaxIter, mNumAgents, mNumRobots);
	}

	bool evaluateIndividualConfig(Eigen::VectorXd config)
	{
		int numberOfRows = mImage.rows;
		int numberOfColumns = mImage.cols;

		// agent
		double x_point = config[0]*numberOfColumns + 0.000001;
		double y_point = (1 - config[1])*numberOfRows + 0.000001;
		cv::Point point((int)x_point, (int)y_point);

		// Collision Check for agent with environment
		int intensity = (int)mImage.at<uchar>(point.y, point.x);
		if (intensity == 0) // Pixel is black
			return false;

		return true;
	}

	bool evaluateIndividualEdge(Graph &graph, Edge& e) // returns false if in collision
	{
		graph[e].isEvaluated = true;

		Vertex source_vertex = source(e, graph);
		Vertex target_vertex = target(e, graph);

		Eigen::VectorXd sourceState(2);
		sourceState << graph[source_vertex].state;

		Eigen::VectorXd targetState(2);
		targetState << graph[target_vertex].state;

		double resolution = 0.0025;
		unsigned int nStates = std::ceil(graph[e].length / resolution-0.000000001)+1;

		// Just start and goal
		if(nStates < 2u)
		{
			nStates = 2u;
		}
		// std::cout<<"nStates:"<<nStates<<std::endl;

		bool checkResult = true;
		
		if (checkResult && !evaluateIndividualConfig(sourceState))
		{
			graph[source_vertex].status = CollisionStatus::BLOCKED;
			graph[e].status = CollisionStatus::BLOCKED;
			graph[e].length = INF;
			checkResult = false;
		}

		if (checkResult && !evaluateIndividualConfig(targetState))
		{
			graph[target_vertex].status = CollisionStatus::BLOCKED;
			graph[e].status = CollisionStatus::BLOCKED;
			graph[e].length = INF;
			checkResult = false;
		}

		if (checkResult)
		{
			// Evaluate the States in between
			for (unsigned int i = 1; i < nStates-1; i++)
			{

				if(!evaluateIndividualConfig(sourceState + (resolution*i/graph[e].length)*(targetState-sourceState) ))
				{
					graph[e].status = CollisionStatus::BLOCKED;
					graph[e].length = INF;
					checkResult = false;
					break;
				}
			}
		}

		return checkResult;
	}

	void preprocess_graph(Graph &g, Vertex &_goal)
	{
		boost::unordered_map<Vertex,bool> sptSet; 
		boost::unordered_map<Vertex,Vertex> parentMap;
		boost::unordered_map<Vertex,double> distanceMap;

		Vertex goal_vertex=_goal;

		VertexIter vi, viend;
		
		int numVertices=0;
		for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi) 
		{
			numVertices++;
			distanceMap[*vi]=std::numeric_limits<double>::infinity();
		}

		parentMap.clear();
		sptSet.clear();
		distanceMap[goal_vertex]=0;
		int totalVertices=numVertices+1;
		
		while(numVertices>0)
		{
			double min_dist= std::numeric_limits<double>::infinity();
			Vertex min_vertex;
			for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi) 
			{
				if(!sptSet.count(*vi) && distanceMap[*vi]<=min_dist)
				{
					min_dist = distanceMap[*vi];
					min_vertex = *vi;
				}
			}

			Vertex node = min_vertex;
			sptSet[node]=1;

			std::vector<Vertex> successors;
			OutEdgeIter ei, ei_end;

			for (boost::tie(ei, ei_end) = out_edges(node, g); ei != ei_end; ++ei) 
			{
				Vertex curSucc = target(*ei, g);
				Edge e = *ei;
				if(!g[e].isEvaluated)
					evaluateIndividualEdge(g,e);
				if(g[e].status == CollisionStatus::FREE)
					successors.push_back(curSucc);
			}

			for(Vertex &successor : successors )
			{
				Edge uv;
				bool edgeExists;
				boost::tie(uv, edgeExists) = edge(node, successor, g);

				if( !sptSet.count(successor) && (distanceMap[successor] > distanceMap[node] + g[uv].length) )
				{
					distanceMap[successor] = distanceMap[node] + g[uv].length;
					parentMap[successor]=node;
				}
			}
			numVertices--;
		}

		for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi) 
		{
			g[*vi].heuristic = distanceMap[*vi];
		}   
	}

	bool solve(){
		property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), mPCGraph);
		PrecedenceConstraintGraph G_T;

		mSolveStartTime = high_resolution_clock::now();
		// std::cout<<"in solve!!"<<std::endl;

		transpose_graph(mPCGraph, G_T);

		std::vector<int> startTimesteps;
		std::vector<int> goalTimesteps;

		// std::vector<int> sp_tasks_list(numRobots);
		std::vector<Eigen::VectorXd> sp_goal(mNumRobots);
		std::vector<int> sp_goal_timestep(mNumRobots);

		int makespan = 0;
		
		int id=0;
		for ( int i=0; i<mNumAgents; i++)
		{
			meta_data vertex = get(name, i);
			startTimesteps.push_back(vertex.start_time);
			goalTimesteps.push_back(vertex.end_time+vertex.slack);
			makespan = std::max(makespan,vertex.end_time + vertex.slack);

			// std::cout<<"Agent ID: "<<id<<" "<<startTimesteps[id]<<" "<<goalTimesteps[id]<<std::endl;
			id++;

			Eigen::VectorXd goal_config(2);
			goal_config[0] = vertex.goal.first;
			goal_config[1] = vertex.goal.second;

			std::vector <int> agent_list = vertex.agent_list;
			for (auto robotNum: agent_list)
			{
				sp_goal[robotNum] = goal_config;
				sp_goal_timestep[robotNum] = vertex.end_time+vertex.slack;
			}
		}

		// int makespan = 0;
		// for(int robot_id=0; robot_id<numRobots; robot_id++)
		// 	makespan = std::max(makespan,sp_goal_timestep[robot_id]);

		// std::cerr<<"Makespan :"<<makespan<<std::endl;

		std::vector<std::pair<Eigen::VectorXd,std::pair<int,int>>> stationary_agents;
		std::vector<int> s_ids;

		for(int robot_id=0; robot_id<mNumRobots; robot_id++)
			if(makespan != sp_goal_timestep[robot_id])
			{
				stationary_agents.push_back(std::make_pair(sp_goal[robot_id],
					std::make_pair(sp_goal_timestep[robot_id],makespan)));
				s_ids.push_back(robot_id);
				// std::cerr<<"Agent: "<<robot_id<<" Goal: "<<sp_goal[robot_id][0]<<" "
					// <<sp_goal[robot_id][1]<<" "<<"Times: "<<sp_goal_timestep[robot_id]<<" "<<makespan<<std::endl;
			}
		// std::cerr << stationary_agents.size() << std::endl;
		// std::cin.get();
		// Setup planner
		// std::cout<<"PRESS [ENTER} TO CALL SOLVE!"<<std::endl;std::cin.get();
		CBS planner(mNumAgents,mRoadmapFileNames,mStartConfig,mGoalConfig,startTimesteps,goalTimesteps, 
			mStartVertex, mGoalVertex, stationary_agents, mPCGraph, G_T);
		// std::cout<<"PRESS [ENTER} TO CALL SOLVE!"<<std::endl;std::cin.get();
		std::vector<std::vector<Eigen::VectorXd>> path = planner.solve(mSolveStartTime);
		
		std::cerr<<"returned!"<<std::endl;
		if(path[0].size() == 0)
		{
			// std::cout<<"N";
			return false;
		}
		std::cerr<<"returned!"<<std::endl;
		// std::cout<<"Y";
		std::vector<std::vector< Eigen::VectorXd>> agent_paths(mNumRobots,std::vector< Eigen::VectorXd>());

		int task_count = 0;
		id = 0;
		for ( container::reverse_iterator ii=mTopologicalOrder.rbegin(); ii!=mTopologicalOrder.rend(); ++ii)
		{
			// std::cout << std::endl;
			// std::cout<<"Agent ID: "<<id<<" "<<startTimesteps[id]<<" "<<goalTimesteps[id]
				// <<" "<<goalTimesteps[id]-startTimesteps[id]+1<<" "<<path[task_count].size()<<std::endl;

			id++;
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
				// if(vertex.agent_list[agent] == 1)
				// 	std::cout<<"tis me! - "<<agent_paths[vertex.agent_list[agent]].size()<<std::endl;
			}
			
			task_count++;
		}

		// int ret_makespan = 0;
		// for(int i=0; i<agent_paths.size(); i++)
		// 	ret_makespan = std::max(ret_makespan,(int)agent_paths[i].size());
		// std::cout<<"R Makespan: "<<ret_makespan<<std::endl;

		for(int i=0; i<s_ids.size(); i++)
		{
			for(int j = stationary_agents[i].second.first; j < stationary_agents[i].second.second; j++)
				agent_paths[s_ids[i]].push_back(stationary_agents[i].first);
		}

		// std::cerr<<"agent path found!"<<std::endl;

		int path_cost =0;
		for(int i=0; i<agent_paths.size(); i++)
		{
			std::cout<<"Path size for agent "<<i<<" = "<<agent_paths[i].size()<<std::endl;
			path_cost = std::max(path_cost,(int)agent_paths[i].size());
		}
		// std::cin.get();
		// std::cerr<<path_cost<<" ";
		// std::cin.get();
		std::vector<Eigen::VectorXd> path_configs;

		std::cerr<<agent_paths[0][0][0] << std::endl;

		makespan = 0;
		for(int i=0; i<agent_paths.size(); i++)
		{
			if(agent_paths[i].size() > makespan) makespan = agent_paths[i].size();
		}
		std::cerr<<makespan<<" ";
		for(int i=0; i<makespan; i++)
		{
			Eigen::VectorXd config(agent_paths.size()*2);
			for(int j=0; j<agent_paths.size(); j++)
			{
				if(i < agent_paths[j].size()){
					config[2*j]=agent_paths[j][i][0];
					config[2*j+1]=agent_paths[j][i][1];
				}
				else{
					config[2*j]=agent_paths[j][agent_paths[j].size()-1][0];
					config[2*j+1]=agent_paths[j][agent_paths[j].size()-1][1];
				}	
				std:: cout << "Time " << i << " Agent Id: " << j << " Position x-" << config[2*j] << " Position y-" << config[2*j+1] << std::endl;
			}
			path_configs.push_back(config);
		}

		
		// std::cout<<"Path config: "<<path_configs[0]<<std::endl;

		std::cout<<"Press [ENTER] to display path: \n";
		std::cin.get();
		planner.mNumAgents = mNumRobots;
		planner.displayPath(path_configs);

		// std::cout<<"true!";

		return true;
	}

};


} // namespace CMAPF

#endif 
