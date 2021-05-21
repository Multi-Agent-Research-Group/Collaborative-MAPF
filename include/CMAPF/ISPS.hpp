 
/* Authors: Rajat Kumar Jenamani */

#ifndef _ISPS_HPP
#define _ISPS_HPP

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
#include "time_priority_queue.hpp"
#include "CBSDefinitions.hpp"
#include "ISPSDefinitions.hpp"
#include "LoadGraphfromFile.hpp"
#include "PCSolver.hpp"
#include "PCDefinitions.hpp"

// #define PRINT if (cerr_disabled) {} else std::cout
// #define DEBUG if (cerr_disabled) {} else 
// bool cerr_disabled = true;

#include <chrono>
using namespace std::chrono;

#define INF std::numeric_limits<double>::infinity()

namespace CMAPF {

using namespace BGL_DEFINITIONS;

class ISPS
{

public:

	/// Environment
	cv::Mat mImage;

	/// The fixed graphs denoting individual environment of corresponding agents
	std::vector<Graph> mGraphs;

	/// Number of agents
	int mNumAgents; 

	/// Path to the roadmap files.
	std::vector<std::string> mRoadmapFileNames;

	/// Source vertex.
	std::vector<Eigen::VectorXd> mStartConfig;
	std::vector<Vertex> mStartVertex;

	/// Goal vertex.
	std::vector<Eigen::VectorXd> mGoalConfig;
	std::vector<Vertex> mGoalVertex;

	PrecedenceConstraintGraph mPCGraph;
	PrecedenceConstraintGraph mPCGraph_T;

	std::vector<std::vector<Constraint>> mConstraints;

	ISPSPriorityQueue mPQ;
	std::set<int> mClosedSet;

	std::vector<std::pair<Eigen::VectorXd,std::pair<int,int>>> mStationaryAgents;

	vector <vector <int>> mPredecessors;
	vector <vector <int>> mSuccessors;

	std::vector <int> mCosts;
	container mTopologicalOrder;
	std::vector< std::vector<Vertex> > mComputedPaths;

	double mUnitEdgeLength = 0.1;

	property_map<PrecedenceConstraintGraph, meta_data_t>::type mProp;
	property_map<PrecedenceConstraintGraph, meta_data_t>::type mProp_T;

	ISPS(cv::Mat img, int numAgents, std::vector<std::string> roadmapFileNames, std::vector<Eigen::VectorXd> startConfig, std::vector<Eigen::VectorXd> goalConfig, 
		PrecedenceConstraintGraph &PCGraph, PrecedenceConstraintGraph &PCGraph_T, std::vector<Graph> graphs, std::vector<Vertex> startVertex, std::vector<Vertex> goalVertex,
		std::vector<std::pair<Eigen::VectorXd,std::pair<int,int>>>& stationaryAgents, std::vector<std::vector<Constraint>> constraints)
		: mImage(img)
		, mNumAgents(numAgents)
		, mRoadmapFileNames(roadmapFileNames)
		, mPCGraph(PCGraph)
		, mPCGraph_T(PCGraph_T)
		, mStartConfig(startConfig)
		, mGoalConfig(goalConfig)
		, mGraphs(graphs)
		, mStartVertex(startVertex)
		, mGoalVertex(goalVertex) 
		, mStationaryAgents(stationaryAgents)
		, mConstraints(constraints)

		{
			mProp = get(meta_data_t(), mPCGraph);
			mProp_T = get(meta_data_t(), mPCGraph_T);
			topological_sort(mPCGraph, std::back_inserter(mTopologicalOrder));
			
			for(int i=0; i<mNumAgents; i++){
				std::vector <Vertex> path = computeShortestPath(mGraphs[i], mStartVertex[i], 
											mGoalVertex[i], mConstraints[i], 0);
				mComputedPaths.push_back(path);
				mCosts.push_back(path.size());
			}

			std::cerr << "here\n";

			for(int i=0; i<mNumAgents; i++){
				meta_data *vertex = &get(mProp, i);
				vertex->slack = 10000;
				vertex->start_time = 0;
				vertex->end_time = 0;

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

			std::cerr << "here1\n";

			updateSchedule();
			initQueue();
			std::cerr << "here1\n";
		}

	std::vector< std::vector<Vertex> > solve(){
		std::cerr << "here2solve\n";
		while(mPQ.PQsize()!=0){
			slackElement node = mPQ.pop();

			int agent_id = node.agent_id;
			meta_data *vertex = &get(mProp, agent_id);

			std::vector<Vertex> path = computeShortestPath(mGraphs[agent_id], mStartVertex[agent_id], mGoalVertex[agent_id],
											mConstraints[agent_id], vertex->start_time);

			mCosts[agent_id] = path.size();
			vertex->end_time = vertex->start_time + mCosts[agent_id]-1; //update vertex final time
			mComputedPaths[agent_id] = path; // update route plan
			mClosedSet.insert(agent_id);

			updateSchedule();
			initQueue();
		}
		std::cerr << "here1\n";
		std::cerr << mNumAgents << std::endl;
		// for (auto path1:mComputedPaths){
		// 	for(auto vertex1: path1){
		// 		std::cerr << vertex1 << " ";
		// 	}
		// 	std::cerr << std::endl;
		// }
		std::cerr << mComputedPaths.size() << std::endl;
		return mComputedPaths;
	}


	void initQueue(){
		for(int i=0; i<mNumAgents; i++){
			if(mClosedSet.find(i)!=mClosedSet.end()) continue;

			bool closedPred = true;
			for(auto pred:mPredecessors[i]){
				if(mClosedSet.find(pred)==mClosedSet.end()){
					closedPred = false;
					break;
				}
			}

			if(closedPred) {
				meta_data *vertex = &get(mProp, i);
				mPQ.insert(i, vertex->slack);
			}
		}
	}

	void updateSchedule(){
		for (container::iterator ii=mTopologicalOrder.begin(); ii!=mTopologicalOrder.end(); ++ii)
		{
			int agent_id = *ii;
			meta_data *vertex = &get(mProp, agent_id);
			vector <int> predecessors = mPredecessors[agent_id];
			if(predecessors.size() == 0){
				vertex->start_time = 0;
				vertex->end_time = std::max(vertex->end_time, mCosts[agent_id]-1);
			}
			else{
				int makespan = -1;
				for(auto pred: predecessors){
					meta_data *pred_vertex = &get(mProp, pred);
					if(pred_vertex->end_time>makespan){makespan = pred_vertex->end_time;}
				}
				vertex->start_time = std::max(makespan, vertex->start_time);
				vertex->end_time = std::max(vertex->end_time, makespan+mCosts[agent_id]-1);
			}
		}

		int makespan = 0;

		for(int i=0; i<mNumAgents; i++){
			meta_data *vertex = &get(mProp, i);
			makespan = std::max(makespan, vertex->end_time);
		}

		for (container::reverse_iterator ii=mTopologicalOrder.rbegin(); ii!=mTopologicalOrder.rend(); ++ii)
		{
			int agent_id = *ii;
			meta_data *vertex = &get(mProp, agent_id);
			vector <int> successors = mSuccessors[agent_id];
			if(successors.size() == 0){
				vertex->slack = std::min(vertex->slack, makespan-vertex->end_time);
			}
			else{
				int makespan = -1;
				for(auto suc: successors){
					meta_data *suc_vertex = &get(mProp, suc);
					vertex->slack = std::min(vertex->slack, suc_vertex->start_time + suc_vertex->slack - vertex->end_time);
				}
			}
		}
	}


	std::vector<Vertex> getNeighbors(Graph &graph, Vertex &v)
	{
		std::vector<Vertex> neighbors;
		OutEdgeIter ei, ei_end;

		for (boost::tie(ei, ei_end) = out_edges(v, graph); ei != ei_end; ++ei) 
		{
			Vertex curSucc = target(*ei, graph);
			Edge e = *ei;
			if(!graph[e].isEvaluated)
				evaluateIndividualEdge(graph,e);
			if(graph[e].status == CollisionStatus::FREE)
				neighbors.push_back(curSucc);
		}

		// std::cout<<"neighbors size: "<<neighbors.size()<<std::endl;
		return neighbors;
	}

	std::vector<Vertex> computeShortestPath(Graph &graph, Vertex &start, Vertex &goal, 
		std::vector<Constraint> &constraints, int initial_timestep)
	{
		double costOut;
		timePriorityQueue pq;
		boost::unordered_map<std::pair<Vertex, int>, double, pair_hash> mDistance;
		boost::unordered_map<std::pair<Vertex, int> , std::pair<Vertex, int>, pair_hash > mPrev;
		boost::unordered_map<int , Vertex> nodeMap;

		pq.insert(graph[start].vertex_index,initial_timestep,graph[start].heuristic,0.0);
		nodeMap[graph[start].vertex_index]=start;

		VertexIter vi, viend;
		mDistance[std::make_pair(start,initial_timestep)]=0;

		int numSearches = 0;
		int maximum_timestep = 10000;

		int goal_timestep = -1;
		while(pq.PQsize()!=0)
		{
			numSearches++;
			std::pair<int,int> top_element = pq.pop();
			int index = top_element.first;
			int timeStep = top_element.second;
			// if(timeStep > final_timestep)
			// 	continue;
			if(index == graph[goal].vertex_index)
			{
				goal_timestep = timeStep;
				costOut = mDistance[std::make_pair(goal,goal_timestep)];
				break;
			}
			Vertex curr_node = nodeMap[index];
			std::vector<Vertex> neighbors = getNeighbors(graph,curr_node);
			neighbors.push_back(curr_node);
			// std::cout<<"No. of neighbors :"<<neighbors.size()<<std::endl;

			for (auto successor : neighbors) 
			{
				if(successor == curr_node)
				{
					bool col = false;
					for( Constraint c: constraints)
					{
						if( c.constraint_type == 1 && successor == c.v && c.t == timeStep + 1)
						{
							// std::cout<<"Constraint Encountered! "<<std::endl;
							col =true;
							break;
						}
					}

					if(!col)
					{
						double new_cost = mDistance[std::make_pair(curr_node,timeStep)] + mUnitEdgeLength;
						
						if(mDistance.count(std::make_pair(successor,timeStep+1))==0 || 
							new_cost < mDistance[std::make_pair(successor,timeStep+1)])
						{
							mDistance[std::make_pair(successor,timeStep+1)]= new_cost;
							double priority;
							// std::cout<<"FOUND FREE EDGE!!"<<std::endl;
							priority = new_cost + graph[successor].heuristic;
							pq.insert(graph[successor].vertex_index,timeStep+1,priority,0.0);
							if(nodeMap.count(graph[successor].vertex_index)==0)
								nodeMap[graph[successor].vertex_index]=successor;
							mPrev[std::make_pair(successor,timeStep+1)]=std::make_pair(curr_node,timeStep);
						}
					}
				}
				else
				{
					Edge uv_edge = boost::edge(curr_node, successor, graph).first;

					bool col = false;
					for( Constraint c: constraints)
					{
						if( (c.constraint_type == 1 && successor == c.v && c.t == timeStep + 1) || (c.constraint_type == 2 && uv_edge == c.e && c.t == timeStep + 1) )
						{
							// std::cout<<"Constraint Encountered! "<<std::endl;
							col =true;
							break;
						}
					}

					if(!col)
					{                   
						double new_cost = mDistance[std::make_pair(curr_node,timeStep)] + mUnitEdgeLength;
						if(mDistance.count(std::make_pair(successor,timeStep+1))==0 || new_cost < mDistance[std::make_pair(successor,timeStep+1)])
						{
							mDistance[std::make_pair(successor,timeStep+1)]= new_cost;
							double priority;
							priority = new_cost + graph[successor].heuristic;
							pq.insert(graph[successor].vertex_index,timeStep+1,priority,0.0);
							if(nodeMap.count(graph[successor].vertex_index)==0)
								nodeMap[graph[successor].vertex_index]=successor;
							mPrev[std::make_pair(successor,timeStep+1)]=std::make_pair(curr_node,timeStep);
						}
					}
				}
			}
		}

		if(goal_timestep == -1)
		{
			costOut = INF;
			return std::vector<Vertex>();
		}

		std::vector<Vertex> finalPath;
		Vertex node = goal;

		while(!(node == start && goal_timestep == initial_timestep))
		{
			finalPath.push_back(node);
			Vertex temp_node = node;
			int temp_timestep = goal_timestep;
			node=mPrev[std::make_pair(temp_node,temp_timestep)].first;
			goal_timestep=mPrev[std::make_pair(temp_node,temp_timestep)].second;
		}
		finalPath.push_back(start);
		std::reverse(finalPath.begin(), finalPath.end());

		return finalPath;
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
};

} // namespace CMAPF

#endif 
