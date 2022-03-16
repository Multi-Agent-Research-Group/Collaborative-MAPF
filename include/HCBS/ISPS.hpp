
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

namespace HCBS {

using namespace BGL_DEFINITIONS;

/// Environment
cv::Mat mImage;

/// The fixed graphs denoting individual environment of corresponding agents
std::vector<Graph> mGraphs;

PrecedenceConstraintGraph mPCGraph;
PrecedenceConstraintGraph mPCGraph_T;

property_map<PrecedenceConstraintGraph, meta_data_t>::type mProp;
property_map<PrecedenceConstraintGraph, meta_data_t>::type mProp_T;

vector <vector <int>> mPredecessors;
vector <vector <int>> mSuccessors;

class ISPS
{

public:

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

	std::vector<std::vector<Constraint>> mConstraints;

	ISPSPriorityQueue mPQ;
	std::set<int> mClosedSet;

	std::vector <int> mCosts;
	container mTopologicalOrder;
	std::vector< std::vector<Vertex> > mComputedPaths;
	std::vector <std::vector <bool>> mMap;
	double mUnitEdgeLength = 1.0;

	ISPS(int numAgents, std::vector<std::string> roadmapFileNames, 
		std::vector<Eigen::VectorXd> startConfig, 
		std::vector<Eigen::VectorXd> goalConfig, 
		std::vector<Vertex> startVertex, 
		std::vector<Vertex> goalVertex, 
		std::vector<std::vector<Constraint>> constraints,
		std::vector <std::vector <bool>> map)
		: mNumAgents(numAgents)
		, mRoadmapFileNames(roadmapFileNames)
		, mStartConfig(startConfig)
		, mGoalConfig(goalConfig)
		, mStartVertex(startVertex)
		, mGoalVertex(goalVertex) 
		, mConstraints(constraints)
		, mMap(map)
	{
		mPQ.reset();
		mProp = get(meta_data_t(), mPCGraph);
		mProp_T = get(meta_data_t(), mPCGraph_T);
		topological_sort(mPCGraph, std::back_inserter(mTopologicalOrder));
		
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
				// std::cout << "Edges source = " << curPred << " Edge target = " << i << std::endl;
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

		// std::cerr << "here\n";

		mComputedPaths = std::vector< std::vector<Vertex> > (mNumAgents,std::vector<Vertex> {}) ;
		mCosts = std::vector <int> (mNumAgents);

		for(int i=0; i<mNumAgents; i++){
			meta_data *vertex = &get(mProp, i);
			std::vector<Constraint> v;
			std::vector <Vertex> path = computeShortestPath(mGraphs[i], mStartVertex[i], 
										mGoalVertex[i], v, vertex->start_time, vertex->end_time,
										i, vertex->slack);
			mComputedPaths[i] = path;
		}
		updateSchedule();
		initQueue();
	}

	std::vector< std::vector<Vertex> > solve(double &costOut)
	{
		// std::cerr<<"I";
		// std::cout<<"Checking start times: \n\n "<<std::endl;
		while(mPQ.PQsize()!=0){
			slackElement node = mPQ.pop();

			int agent_id = node.agent_id;
			meta_data *vertex = &get(mProp, agent_id);

			std::vector<Vertex> path = computeShortestPath(mGraphs[agent_id], mStartVertex[agent_id], mGoalVertex[agent_id],
											mConstraints[agent_id], vertex->start_time, vertex->end_time,
											agent_id, vertex->slack);
			if(path.size() == 0)
			{
				return std::vector< std::vector<Vertex> > (mNumAgents,std::vector<Vertex>());
			}

			mComputedPaths[agent_id] = path; // update route plan
			vertex->end_time = vertex->start_time + mComputedPaths[agent_id].size()-1; //update vertex final time
			
			mClosedSet.insert(agent_id);
			updateSchedule();
			initQueue();
		}

		int makespan = 0;

		for(int i=0; i<mNumAgents; i++){
			meta_data *vertex = &get(mProp, i);
			makespan = std::max(makespan, vertex->end_time);
		}

		int maximum_timestep = 0;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			meta_data *vertex = &get(mProp, agent_id);
			maximum_timestep = std::max(maximum_timestep, int(vertex->start_time + mComputedPaths[agent_id].size()-1));
		}

		if(maximum_timestep!=makespan)
		{
			std::cout << makespan << " " << maximum_timestep << std::endl;
			std::cout<<"not equal!";
			std::cin.get();
		}

		// std::cerr << "FINAL MAKESPAN ================= " << makespan << std::endl;

		costOut = makespan*mUnitEdgeLength;

		for (container::reverse_iterator ii=mTopologicalOrder.rbegin(); ii!=mTopologicalOrder.rend(); ++ii)
		{
			updateSchedule();
			int agent_id = *ii;
			// std::cout<<"\n\nAgent id: "<<agent_id<<std::endl;
			bool flag = false;
			meta_data *vertex = &get(mProp, agent_id);

			if (flag) std::cout << "Path Size " << mComputedPaths[agent_id].size()<< std::endl;
			vector <int> successors = mSuccessors[agent_id];
			if(successors.size() == 0){
				int cur_path_length = mComputedPaths[agent_id].size();
				int slack = makespan-(vertex->start_time+cur_path_length-1);
				if(cur_path_length){
					for(int i=0; i<slack; i++){
						mComputedPaths[agent_id].push_back(mComputedPaths[agent_id][cur_path_length-1]);
					}
				}
				if (flag) std::cout << "Path Cost to Add!! " << slack << std::endl;
				vertex->end_time = makespan;
			}

			else{
			// std::cerr << "here1\n";
				int max_start_time = 1000000;
				for(auto suc: mSuccessors[agent_id]){
					meta_data *suc_vertex = &get(mProp, suc);
					
					max_start_time = std::min(suc_vertex->start_time, max_start_time);
				}
				// if(flag) std::cout <<max_start_time << std::endl;
				int cur_path_length = mComputedPaths[agent_id].size();
				int pathCostToAdd = max_start_time - (vertex->start_time + cur_path_length-1);
				if (flag) std::cout << "Path Cost to Add " << pathCostToAdd<< std::endl;
				for(int i=0; i<pathCostToAdd; i++){
					mComputedPaths[agent_id].push_back(mComputedPaths[agent_id][cur_path_length-1]);
				}
				vertex->end_time = max_start_time;
			// std::cerr << "here2\n";
			}
			if (flag) std::cout << "Path Size " << mComputedPaths[agent_id].size()<< std::endl;
			
			updateSchedule();
		}
		// std::cin.get();
		maximum_timestep = 0;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			meta_data *vertex = &get(mProp, agent_id);
			maximum_timestep = std::max(maximum_timestep, int(vertex->start_time + mComputedPaths[agent_id].size()-1));
		}
		if(maximum_timestep!=makespan)
		{
			std::cout << makespan << " " << maximum_timestep << std::endl;
			std::cout<<"later not equal!";
			printSchedule();
			std::cin.get();
		}
		// std::cout<<"Maximum timestep: "<<maximum_timestep<<std::endl;
		// std::cerr<<"O\n";
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
				mPQ.remove(i);
				mPQ.insert(i, vertex->slack);
			}
		}
	}

	void updateSchedule(){
		for (container::reverse_iterator ii=mTopologicalOrder.rbegin(); ii!=mTopologicalOrder.rend(); ++ii)
		{
			int agent_id = *ii;
			meta_data *vertex = &get(mProp, agent_id);
			vector <int> predecessors = mPredecessors[agent_id];
			if(predecessors.size() == 0){
				vertex->start_time = 0;
				vertex->end_time = mComputedPaths[agent_id].size()-1;
				// std::cerr << "here" << std::endl;
			}
			else{
				int makespan = 0;
				for(auto pred: predecessors){
					meta_data *pred_vertex = &get(mProp, pred);
					if(pred_vertex->end_time>makespan){
						makespan = pred_vertex->end_time;
					}
				}
				vertex->start_time = std::max(makespan, vertex->start_time);
				// vertex->end_time = vertex->start_time+mComputedPaths[agent_id].size()-1;
				vertex->end_time = std::max(vertex->end_time,
					(int)(vertex->start_time+mComputedPaths[agent_id].size()-1));
				// std::cerr << "there" << std::endl;
			}
			// std::cin.get();
		}

		int makespan = 0;
		
		for(int i=0; i<mNumAgents; i++){
			meta_data *vertex = &get(mProp, i);
			makespan = std::max(makespan, vertex->end_time);
		}
		// std::cerr << "CURRENT MAKESPAN " << makespan << std::endl;

		for (container::iterator ii=mTopologicalOrder.begin(); ii!=mTopologicalOrder.end(); ++ii)
		{
			int agent_id = *ii;
			meta_data *vertex = &get(mProp, agent_id);
			vector <int> successors = mSuccessors[agent_id];
			if(successors.size() == 0){
				vertex->slack = std::min(vertex->slack, makespan-vertex->end_time);
			}
			else{
				for(auto suc: successors){
					meta_data *suc_vertex = &get(mProp, suc);
					// vertex->end_time = std::max(suc_vertex->start_time, vertex->end_time);
					// suc_vertex->start_time = std::max(suc_vertex->start_time, vertex->end_time);
					vertex->slack = std::min(vertex->slack, suc_vertex->start_time + suc_vertex->slack - vertex->end_time);
				}
			}
		}

		// printSchedule();
	}

	void printSchedule(){
		for (container::reverse_iterator ii=mTopologicalOrder.rbegin(); ii!=mTopologicalOrder.rend(); ++ii)
		{
			int agent_id = *ii;
			meta_data *vertex = &get(mProp, agent_id);
			std::cout << "--------------Task No = " << agent_id << std::endl;
			std::cout << "Start Time = " << vertex->start_time << " End Time = " << vertex->end_time << std::endl;
		}
		std::cin.get();
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
			{
				std::cout<<"ISPS - not evaluated!"<<std::endl;
				std::cin.get();
				evaluateIndividualEdge(graph,e);
			}
			if(graph[e].status == CollisionStatus::FREE)
				neighbors.push_back(curSucc);
		}

		// std::cout<<"neighbors size: "<<neighbors.size()<<std::endl;
		return neighbors;
	}

	int countConflicts(int agent_id, Vertex &node, int timeStep){
		int numConflicts = 0;
		for(int i=0; i<mNumAgents; i++){
			if(i==agent_id) continue;
			meta_data *vertex = &get(mProp, i);
			if(timeStep < vertex->start_time || timeStep > vertex->end_time || mComputedPaths[i].size()==0) continue;
			int indexToCheck = timeStep-vertex->start_time;

			if(indexToCheck >= mComputedPaths[i].size()) indexToCheck = mComputedPaths[i].size()-1;
			Vertex check = mComputedPaths[i][indexToCheck];
			if ((int)(check) == (int)(node)) numConflicts += 1;
		}
		return numConflicts;
	}

	bool isEqualEdge(Graph &graph, Edge &a, Edge &b){

		Vertex source_vertex1 = source(a, graph);
 		Vertex target_vertex1 = target(a, graph);

 		Vertex source_vertex2 = source(b, graph);
 		Vertex target_vertex2 = target(b, graph);

 		// if(a==b)
 		if(source_vertex1 == source_vertex2 && target_vertex1 == target_vertex2) {
 			return true;
 		}
 		return false;
	}

	int getIntegerCost(double g){
		return (int)(1.00001*g/mUnitEdgeLength);
	}
	std::vector<Vertex> computeShortestPath(Graph &graph, Vertex &start, Vertex &goal, 
		std::vector<Constraint> &constraints, int initial_timestep, int final_timestep, int agent_id, int slack)
	{
		double costOut;
		timePriorityQueue pq;
		boost::unordered_map<std::pair<Vertex, int>, std::vector <int>, pair_hash> mDistance;
		boost::unordered_map<std::pair<Vertex, int> , std::pair<Vertex, int>, pair_hash > mPrev;
		boost::unordered_map<int , Vertex> nodeMap;
		// boost::unordered_map<int , std::vector <double>> heuristicValues;

		int slackHeuristic = std::max(0, initial_timestep+getIntegerCost(graph[start].heuristic)-(slack+final_timestep));
		std::vector <int> heuristics = {slackHeuristic, 0, getIntegerCost(graph[start].heuristic), getIntegerCost(graph[start].heuristic), getIntegerCost(graph[start].heuristic)};
		pq.insert(graph[start].vertex_index,initial_timestep, heuristics);
		nodeMap[graph[start].vertex_index]=start;
		mDistance[std::make_pair(start,initial_timestep)]=heuristics;

		VertexIter vi, viend;

		int numSearches = 0;
		int maximum_timestep = 10000;

		int goal_timestep = -1;

		int max_C_timestep = 0;
		for(int i=0; i<constraints.size(); i++)
		{
			max_C_timestep = std::max(max_C_timestep,(int)constraints[i].t);
		}

		while(pq.PQsize()!=0)
		{
			numSearches++;
			std::pair<int,int> top_element = pq.pop();
			int index = top_element.first;
			int timeStep = top_element.second;

			std::vector<int> currentHeuristics = mDistance[std::make_pair(index, timeStep)];

			int currentSlackHeuristic = currentHeuristics[0];
			int currentPathConflicts = currentHeuristics[1];
			int currentMoveCount = currentHeuristics[2]-currentHeuristics[3];
			int currentGValue = currentHeuristics[4]-currentHeuristics[3];
			int currentHValue = currentHeuristics[3];

			if(index == graph[goal].vertex_index && timeStep >= max_C_timestep)
			{
				goal_timestep = timeStep;
				costOut = currentGValue*mUnitEdgeLength;
				break;
			}

			Vertex curr_node = nodeMap[index];
			std::vector<Vertex> neighbors = getNeighbors(graph,curr_node);
			neighbors.push_back(curr_node);

			for (auto successor : neighbors) 
			{
				if(successor == curr_node)
				{
					bool col = false;
					for( Constraint c: constraints)
					{
						if( c.constraint_type == 1 && successor == c.v && c.t == timeStep + 1)
						{
							col =true;
							break;
						}
					}

					if(!col)
					{
						int newSlackHeuristic = std::max(0, (timeStep+1)+getIntegerCost(graph[successor].heuristic)-(slack+final_timestep));
						int newPathConflicts = currentPathConflicts + countConflicts(agent_id, successor, (timeStep+1));
						int newMoveCount = currentMoveCount;
						int newHValue = getIntegerCost(graph[successor].heuristic); 
						int newFValue = (timeStep+1)+newHValue;

						std::vector <int> newHeuristics = {newSlackHeuristic, newPathConflicts,
							newMoveCount + newHValue, newHValue, newFValue};

						if(mDistance.count(std::make_pair(successor,timeStep+1))==0 || 
							newHeuristics < mDistance[std::make_pair(successor,timeStep+1)])
						{
							mDistance[std::make_pair(successor,timeStep+1)]= newHeuristics;
							pq.insert(graph[successor].vertex_index,timeStep+1,newHeuristics);
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
						if( (c.constraint_type == 1 && successor == c.v && c.t == timeStep + 1) || (c.constraint_type == 2 && c.e==uv_edge && c.t == timeStep + 1) )
						{
							col =true;
							break;
						}
					}

					if(!col)
					{                   
						int newSlackHeuristic = std::max(0, (timeStep+1)+getIntegerCost(graph[successor].heuristic)-(slack+final_timestep));
						int newPathConflicts = currentPathConflicts + countConflicts(agent_id, successor, (timeStep+1));
						int newMoveCount = currentMoveCount+1;
						int newHValue = getIntegerCost(graph[successor].heuristic); 
						int newFValue = (timeStep+1)+newHValue;

						std::vector <int> newHeuristics = {newSlackHeuristic, newPathConflicts,
							newMoveCount + newHValue, newHValue, newFValue};

						if(mDistance.count(std::make_pair(successor,timeStep+1))==0 || 
							newHeuristics < mDistance[std::make_pair(successor,timeStep+1)])
						{
							mDistance[std::make_pair(successor,timeStep+1)]= newHeuristics;
							pq.insert(graph[successor].vertex_index,timeStep+1,newHeuristics);
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
			// std::cerr << "======================================\n=======================================\n" ;
			// std::cin.get();
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
		if(mMap[(int)config[0]][(int)config[1]])
			return true;
		return false;
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

		// std::cout<<"nStates:"<<nStates<<std::endl;

		if (!evaluateIndividualConfig(sourceState) || !evaluateIndividualConfig(targetState))
		{
			graph[target_vertex].status = CollisionStatus::BLOCKED;
			graph[e].status = CollisionStatus::BLOCKED;
			graph[e].length = INF;
			return false;
		}

		return true;
	}
};

} // namespace HCBS

#endif 
