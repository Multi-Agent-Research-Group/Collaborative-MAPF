 
/* Authors: Rajat Kumar Jenamani */

#ifndef _CBS_HPP
#define _CBS_HPP

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
#include "LoadGraphfromFile.hpp"

#define PRINT if (cerr_disabled) {} else std::cout
#define DEBUG if (cerr_disabled) {} else 
bool cerr_disabled = true;

#include <chrono>
using namespace std::chrono;

#define INF std::numeric_limits<double>::infinity()

namespace CMAPF {

using namespace BGL_DEFINITIONS;

class ISPSs
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

	std::set<size_t> mClosedSet;

	std::vector<std::pair<Eigen::VectorXd,std::pair<int,int>>> mStationaryAgents;

	vector <vector <size_t>> mPredecessors;
	vector <vector <size_t>> mSuccessors;

	container mTopologicalOrder;

	double mUnitEdgeLength = 0.1;

	property_map<PrecedenceConstraintGraph, meta_data_t>::type mProp; 
	property_map<PrecedenceConstraintGraph, meta_data_t>::type mProp_T; = get(meta_data_t(), G_T);

	ISPS(cv::Mat img, int numAgents, std::vector<std::string> roadmapFileNames, std::vector<Eigen::VectorXd> startConfig, std::vector<Eigen::VectorXd> goalConfig, 
		PrecedenceConstraintGraph PCGraph, PrecedenceConstraintGraph PCGraph_T, std::vector<Graph> graphs, std::vector<Vertex> startVertex, std::vector<Vertex> goalVertex,
		std::vector<std::pair<Eigen::VectorXd,std::pair<int,int>>>& stationaryAgents, std::vector<std::vector<Constraint>> constraints)
		: mImage(img)
		, mNumAgents(numAgents)
		, mRoadmapFileNames(roadmapFileNames)
		, mStartTimestep(startTimesteps)
		, mPCGraph(goalTimesteps)
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

			std::vector <double> costs;

			for(int i=0; i<mNumAgents; i++){
				std::vector <Vertex> path = computeShortestPath(mGraphs[i], mStartVertex[i], 
											mGoalVertex[i], mConstraints[i], 0);
				costs[i] = path.size();
			}

			topological_sort(mPCGraph, std::back_inserter(mTopologicalOrder));
			std::vector< std::vector<Vertex> > start_shortestPaths = computeDecoupledPaths(costs);

			for(int i=0; i<mNumAgents; i++){
				PCOutEdgeIter ei, ei_end;

				vector <size_t> predecessors;
				for (boost::tie(ei, ei_end) = out_edges(i, mPCGraph_T); ei != ei_end; ++ei) 
				{
					PCVertex curPred = target(*ei, mPCGraph_T);
					predecessors.push_back(curPred);
				}

				vector <size_t> successors;
				for (boost::tie(ei, ei_end) = out_edges(i, mPCGraph); ei != ei_end; ++ei) 
				{
					PCVertex curSuc = target(*ei, mPCGraph_T);
					successors.push_back(curSuc);
				}

				mPredecessors[i] = predecessors;
				mSuccessors[i] = successors;
			}

			for (container::iterator ii=mTopologicalOrder.begin(); ii!=mTopologicalOrder.end(); ++ii)
			{
				size_t agent_id = *ii;
				meta_data *vertex = &get(mProp, agent_id);
				vector <size_t> predecessors;
				if(predecessors.size() == 0){

				}
			}

			initPCGraph();
		}

	std::vector< std::vector<Vertex> > solve(){
		while mPQ.size(){
			s = mPQ.pop();
			meta_data *vertex = &get(Prop, pred);
			std::vector<Vertex> path = computeShortestPath();
			vertex.end_time = vertex.start_time + path.size(); //update vertex final time
			computedPaths = addPath(computedPaths, path) // update route plan
			mClosedSet.insert(vertex);
			updateSchedule();
			initQueue();
		}
		return computedPaths;
	}

	void initPCGraph(){
		
	}

	void initQueue(){

	}

	void updateSchedule(){

	}


	std::vector<Vertex> computeShortestPath(Graph &graph, Vertex &start, Vertex &goal, std::vector<Constraint> &constraints, int initial_timestep, double& costOut)
	{
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
			// std::cout<<"Queue pop no: "<<numSearches<<std::endl;
			// if(numSearches%1000 == 0)
			// {
			// 	std::cout<<"CSP numSearches: "<<numSearches<<std::endl;
			// }
			std::pair<int,int> top_element = pq.pop();
			int index = top_element.first;
			int timeStep = top_element.second;
			if(timeStep > final_timestep)
				continue;
			if(index == graph[goal].vertex_index && timeStep == final_timestep)
			{
				// std::cout<<"Timestep goal was found: "<<final_timestep<<std::endl;
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

		// std::cout<<"Goal Time: "<<goal_timestep<<std::endl;
		std::vector<Vertex> finalPath;
		Vertex node = goal;

		// std::cout<<"timesteps: ";
		while(!(node == start && goal_timestep == initial_timestep))
		{
			// std::cin.get();
			// std::cout<<"INF LOOP LOL!";
			// std::cout<<goal_timestep<<" ";
			finalPath.push_back(node);
			Vertex temp_node = node;
			int temp_timestep = goal_timestep;
			node=mPrev[std::make_pair(temp_node,temp_timestep)].first;
			goal_timestep=mPrev[std::make_pair(temp_node,temp_timestep)].second;
		}
		// std::cout<<std::endl;
		finalPath.push_back(start);
		std::reverse(finalPath.begin(), finalPath.end());

		// std::cout<<"CSP: "<<initial_timestep<<" "<<final_timestep<<" C: "<<costOut<<std::endl;
		return finalPath;
	}
} // namespace CMAPF

#endif 
