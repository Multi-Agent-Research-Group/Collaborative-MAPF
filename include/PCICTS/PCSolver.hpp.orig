
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
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/imgproc/imgproc_c.h"

#include "BGLDefinitions.hpp"
#include "PCDefinitions.hpp"
#include "time_priority_queue.hpp"
#include "CBSDefinitions.hpp"
#include "LoadGraphfromFile.hpp"

#include "CBS.hpp"

#define INF std::numeric_limits<double>::infinity()

namespace PCICTS {

using namespace BGL_DEFINITIONS;

class PCSolver
{

public:

	int mCount = 0;

	std::chrono::duration<double, std::micro> mPreprocessTime ;
	std::chrono::duration<double, std::micro> mPlanningTime;

	high_resolution_clock::time_point mSolveStartTime;

	cv::Mat mImage;

	/// The fixed graphs denoting individual environment of corresponding agents
	std::vector<Graph> mGraphs;

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

	vector <vector <int>> mPredecessors;
	vector <vector <int>> mSuccessors;
	vector <set <int>> mNeighbours;
	// vector <vector <int>> mSuccessors;

	/// Goal vertex.
	std::vector<Eigen::VectorXd> mGoalConfig;
	std::vector<Vertex> mGoalVertex;

	PrecedenceConstraintGraph mPCGraph;
	PrecedenceConstraintGraph mPCGraph_T;
	int mMaxIter;

	double mUnitEdgeLength = 0.1;

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
		for ( container::reverse_iterator ii=mTopologicalOrder.rbegin(); ii!=mTopologicalOrder.rend(); ++ii)
		{
			meta_data vertex = get(name, *ii);
			start_config[j] = vertex.start.first;
			start_config[j+1] = vertex.start.second;
			goal_config[j] = vertex.goal.first;
			goal_config[j+1] = vertex.goal.second;
			j+=2;
		}

		transpose_graph(mPCGraph, mPCGraph_T);


		for(int i=0; i<mNumAgents; i++){
			PCOutEdgeIter ei, ei_end;
			vector <int> predecessors;
			for (boost::tie(ei, ei_end) = out_edges(i, mPCGraph_T); ei != ei_end; ++ei) 
			{
				PCVertex curPred = target(*ei, mPCGraph_T);
				predecessors.push_back(curPred);
			}

			vector <int> successors;
			for (boost::tie(ei, ei_end) = out_edges(i, mPCGraph); ei != ei_end; ++ei) 
			{
				PCVertex curSuc = target(*ei, mPCGraph);
				successors.push_back(curSuc);
			}
			mPredecessors.push_back(predecessors);
			mSuccessors.push_back(successors);


		
		}

		for(int i=0; i<mNumAgents; i++){
			set <int> neighbours;
			for(auto pred:mPredecessors[i]){
				for (auto succ: mSuccessors[pred]){
					neighbours.insert(succ);
					// meta_data *vertex = &get(name, succ);
					// upstream_slack = std::min(upstream_slack, vertex->slack_prop);
				}
			}
			mNeighbours.push_back(neighbours);
		}
		
		// std::cout << "PC Iteration: "<<mCount<<std::endl; std::cin.get();

		// Space Information
		// mImage = cv::imread("./src/PCICTS/include/PCICTS/test_final.png", 0);
		// mImage = cv::imread("./src/PCICTS/data/obstacles/0.png", 0);
		mImage = cv::imread(obstacle_file, 0);
		// std::string graph_file = std::string("./src/PCICTS/data/graphs/graph0.graphml");

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
		for(int i=0; i<mMaxIter; i++){
			// std::cout <<i<<"\n";
			if(generatePaths(mPCGraph, G_T, mTopologicalOrder, mTopologicalOrder.begin(), mNumAgents, mNumRobots)){
				std::cout <<mCount<<" ";
				return true;
			}

			// std::cout<<i<<std::endl;
			auto stop = high_resolution_clock::now();
			std::chrono::duration<double, std::micro> timespent = stop - mSolveStartTime;
			if (timespent.count() > 30000000)
			{
				std::cout<<"0 ";
				// std::cout<<"Broke!";
				break;
			}

			PCVertexIter v, vend;
			for (boost::tie(v, vend) = vertices(mPCGraph); v != vend; ++v) {

				container successors;
				PCOutEdgeIter ei, ei_end;

				for (boost::tie(ei, ei_end) = out_edges(*v, mPCGraph); ei != ei_end; ++ei) 
				{
						PCVertex curSuc = target(*ei, mPCGraph);
						successors.push_back(curSuc);
				}

				if(successors.size() == 0){
					meta_data *vertex = &get(name, *v);
					vertex->slack+=1;
				}

			}

		}
		
		std::cout <<mCount<<" ";
		return false;
	}

	bool checkpathpossible(PrecedenceConstraintGraph &G, int &numAgents, int &numRobots)
	{
		mCount +=1;
		// std::cout << "PC Iteration: "<<mCount<<std::endl;

		std::vector<int> startTimesteps;
		std::vector<int> goalTimesteps;

		property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);

		// std::vector<int> sp_tasks_list(numRobots);
		std::vector<Eigen::VectorXd> sp_goal(numRobots);
		std::vector<int> sp_goal_timestep(numRobots);

		int makespan = 0;
		
		int id=0;
		for ( container::reverse_iterator ii=mTopologicalOrder.rbegin(); ii!=mTopologicalOrder.rend(); ++ii)
		{
			meta_data vertex = get(name, *ii);
			startTimesteps.push_back(vertex.start_time);
			goalTimesteps.push_back(vertex.end_time + vertex.slack);
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
				sp_goal_timestep[robotNum] = vertex.end_time + vertex.slack;
			}
		}

		// int makespan = 0;
		// for(int robot_id=0; robot_id<numRobots; robot_id++)
		// 	makespan = std::max(makespan,sp_goal_timestep[robot_id]);

		// std::cerr<<"Makespan :"<<makespan<<std::endl;

		std::vector<std::pair<Eigen::VectorXd,std::pair<int,int>>> stationary_agents;
		std::vector<int> s_ids;

		for(int robot_id=0; robot_id<numRobots; robot_id++)
			if(makespan != sp_goal_timestep[robot_id])
			{
				stationary_agents.push_back(std::make_pair(sp_goal[robot_id],
					std::make_pair(sp_goal_timestep[robot_id],makespan)));
				s_ids.push_back(robot_id);
				// std::cerr<<"Agent: "<<robot_id<<" Goal: "<<sp_goal[robot_id][0]<<" "
					// <<sp_goal[robot_id][1]<<" "<<"Times: "<<sp_goal_timestep[robot_id]<<" "<<makespan<<std::endl;
			}

		// Setup planner
		// std::cout<<"PRESS [ENTER} TO CALL SOLVE!"<<std::endl;std::cin.get();
		CBS planner(mImage,mNumAgents,mRoadmapFileNames,mStartConfig,mGoalConfig,startTimesteps,goalTimesteps, mGraphs, mStartVertex, mGoalVertex, stationary_agents);
		// std::cout<<"PRESS [ENTER} TO CALL SOLVE!"<<std::endl;std::cin.get();
		std::vector<std::vector<Eigen::VectorXd>> path = planner.solve(mSolveStartTime);
		
		// std::cerr<<"returned!"<<std::endl;
		if(path[0].size() == 0)
		{
			// std::cout<<"N";
			return false;
		}

		// std::cout<<"Y";
		std::vector<std::vector< Eigen::VectorXd>> agent_paths(numRobots,std::vector< Eigen::VectorXd>());

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
			// std::cout<<"Path size for agent "<<i<<" = "<<agent_paths[i].size()<<std::endl;
			path_cost = std::max(path_cost,(int)agent_paths[i].size());
		}
		// std::cin.get();
		std::cerr<<path_cost<<" ";
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

		// std::cerr<<"returning!"<<std::endl;
		// std::cout<<"Path config: "<<path_configs[0]<<std::endl;

		// std::cout<<"Press [ENTER] to display path: \n";
		// std::cin.get();
		// planner.mNumAgents = numRobots;
		// planner.displayPath(path_configs);

		// std::cout<<"true!";

		return true;
	}

	bool generatePaths(PrecedenceConstraintGraph &G, PrecedenceConstraintGraph &G_T, container &c, container::iterator ii, int &numAgents, int &numRobots)
	{
		auto stop = high_resolution_clock::now();
		std::chrono::duration<double, std::micro> timespent = stop - mSolveStartTime;
		if (timespent.count() > 30000000)
		{
			return false;
		}

		property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
		property_map<PrecedenceConstraintGraph, meta_data_t>::type name_t = get(meta_data_t(), G_T);
			

		if(c.rbegin().base()-1 == ii){
			return checkpathpossible(G, numAgents, numRobots);
		}

		int agent_id = *ii;
		// container predecessors;
		// PCOutEdgeIter ei, ei_end;
		// for (boost::tie(ei, ei_end) = out_edges(*ii, G_T); ei != ei_end; ++ei) 
		// {
		// 		PCVertex curPred = target(*ei, G_T);
		// 		predecessors.push_back(curPred);
		// }

		vector <int> predecessors = mPredecessors[agent_id];
		if(predecessors.size() == 0){
			return generatePaths(G, G_T, c, ii+1, numAgents, numRobots);
		}

		meta_data *curr_vertex = &get(name, *ii); 

		if(generatePaths(G, G_T, c, ii+1, numAgents, numRobots))
			return true;
		
		int slack = curr_vertex->slack;
		int start_time = curr_vertex->start_time;
		int end_time = curr_vertex->end_time;

		if(curr_vertex->slack_prop){
			std::cout << "There is something wrong with slack prop  " << curr_vertex << std::endl;
		}
		int upstream_slack = curr_vertex -> slack_prop;
		int prev_prop_slack = 0;


		while(curr_vertex->slack_prop != slack){
			// std::cout << "stuck here\n";
			// std:: cout << "prop_slack =" << curr_vertex -> slack_prop << std::endl;
			// std::cin.get();

			
			curr_vertex->slack_prop += 1;
			upstream_slack = curr_vertex->slack_prop;
			
			
			// for(auto pred:predecessors){
			// for (auto succ: mNeighbours[agent_id]){
			// 	meta_data *vertex = &get(name, succ);
			// 	upstream_slack = std::min(upstream_slack, vertex->slack);
			// }

			for (auto succ: mNeighbours[agent_id]){
				meta_data *vertex = &get(name, succ);
				upstream_slack = std::min(upstream_slack, vertex->slack_prop);
			}

			// }
			// std::cout << "upstream_slack = " << upstream_slack << std::endl;
			// std::cout << "prev_prop_slack = " << prev_prop_slack << std::endl;
			for(auto pred:predecessors){
				meta_data *vertex = &get(name, pred);
				vertex->slack+=upstream_slack-prev_prop_slack;
			}

			// std::cout << "upstream_slack = " << upstream_slack << std::endl;
			// if(upstream_slack == 1000000) upstream_slack = 0;
			prev_prop_slack = upstream_slack;

			curr_vertex->slack = slack-upstream_slack;
			curr_vertex->start_time = start_time + upstream_slack;
			// std::cout << "upstream_slack = " << curr_vertex->end_time  << std::endl;
			curr_vertex->end_time = end_time + upstream_slack;
			// std::cout << "upstream_slack = " << curr_vertex->end_time  << std::endl;

			if(generatePaths(G, G_T, c, ii+1, numAgents, numRobots))
				return true;
		}
		curr_vertex->slack_prop = 0;
		curr_vertex->slack = slack;
		// std::cout << "slack = " << curr_vertex->slack  << std::endl;
		curr_vertex->start_time = start_time;
		curr_vertex->end_time = end_time;
		// std::cout << "end_time = " << curr_vertex->end_time  << std::endl;
		// std::cout << "upstream_slack = " << upstream_slack << std::endl;
		for(auto pred:predecessors){
			meta_data *vertex = &get(name, pred);
			vertex->slack-=upstream_slack;
		}
		return false;
	}
};


} // namespace PCICTS

#endif 
// 6_8_25 - 60