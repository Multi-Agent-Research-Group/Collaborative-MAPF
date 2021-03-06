

/* Authors: Rajat Kumar Jenamani */

#ifndef _CBS_HPP
#define _CBS_HPP

// STL headers
#include <vector>
#include <string> 
#include <unordered_set>
#include <queue>
#include <exception>

#include <chrono>
using namespace std::chrono;

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/transpose_graph.hpp>
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
#include "opencv2/imgproc/imgproc_c.h"

#include "BGLDefinitions.hpp"
#include "time_priority_queue.hpp"
#include "CBSDefinitions.hpp"
#include "LoadGraphfromFile.hpp"
#include "PCCBS/PCDefinitions.hpp"


// #define INF std::numeric_limits<double>::infinity()
#define INF std::numeric_limits<int>::max()

bool cerr_disabled = true;
#define PRINT if (cerr_disabled) {} else std::cerr
#define DEBUG if (cerr_disabled) {} else 

bool debug_disabled = true;
// bool debug_disabled = false;
#define print if (debug_disabled) {} else std::cerr

namespace PCCBS {

using namespace BGL_DEFINITIONS;

struct pair_hash_mine
{
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2> &pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

class CBS
{

public:
	std::chrono::duration<double, std::micro> mCSPTime;
	std::chrono::duration<double, std::micro> mHeuristicsTime;
	std::chrono::duration<double, std::micro> mCollabCTime;
	std::chrono::duration<double, std::micro> mCollisionCTime;
	std::chrono::duration<double, std::micro> mGNTime;
	std::chrono::duration<double, std::micro> mQOTime;
	std::chrono::duration<double, std::micro> mCCTime;
	std::chrono::duration<double, std::micro> mCHTime;
	std::chrono::duration<double, std::micro> mPlanningTime;
	std::chrono::duration<double, std::micro> mPreprocessTime;
	std::chrono::duration<double, std::micro> mMapOperationsTime;

	std::chrono::duration<double, std::micro> mGetCollisionTime;
	std::chrono::duration<double, std::micro> mGetCollaborationTime;

	std::string mImagePath;
	std::vector<int> mTaskStartTimestep;
	std::vector<int> mTaskEndTimestep;

	boost::unordered_map <std::pair <int, std::pair<SearchState,SearchState>>, double, agent_state_pair_hash> mHValueMap;

	high_resolution_clock::time_point mSolveStartTime;

	int mCBSIterations;
	int mCollisionIterations;
	int mCollaborationIterations;
	int mCSPExpansions;
	int mCSPIterations;

	/// Environment
	cv::Mat mImage;

	std::vector<std::pair<int, int>> mTasksList;
	std::vector<std::vector<int>> mTasksToAgentsList;
	std::vector<std::vector <int>> mAgentsToTasksList;
	std::vector<std::unordered_map<Vertex,bool>> mSpecialPosition;

	/// The fixed graphs denoting individual environment of corresponding agents
	std::vector<Graph> mGraphs;
	Graph mGraph;

	/// Number of agents
	int mNumAgents; 

	/// Number of tasks
	int mNumTasks;

	/// Path to the roadmap files.
	std::vector<std::string> mRoadmapFileNames;
	PrecedenceConstraintGraph mPCGraph;
	PrecedenceConstraintGraph mPCGraph_T;

	property_map<PrecedenceConstraintGraph, meta_data_t>::type mProp;
	property_map<PrecedenceConstraintGraph, meta_data_t>::type mProp_T;

	vector <vector <int>> mPredecessors;
	vector <vector <int>> mSuccessors;
	container mTopologicalOrder;
	std::vector <std::vector <int>> mAllPairsShortestPathMap;
	std::vector <std::vector <bool>> mMap;
	int mNumVertices;

	double mUnitEdgeLength = 1.0;
	std::vector <boost::unordered_map<int, double>> mAgentHeuristics;
	std::string mSplitStrategy;
	std::string mHeuristicStrategy;
	std::string mType;
	CBS(PrecedenceConstraintGraph _pcg, 
		Graph g, 
		std::vector <std::vector <bool>> map,
		int numAgents,
		std::vector <std::vector <int>> edgeWeights,
		std::string type,
		cv::Mat image)
		: mGraph(g)
		, mNumAgents(numAgents)
		, mPCGraph(_pcg)
		, mType(type)
		, mAllPairsShortestPathMap(edgeWeights)
		, mMap(map)
		, mImage(image)
	{
		//set up transpose graph and topological sort
		transpose_graph(mPCGraph, mPCGraph_T);
		mProp = get(meta_data_t(), mPCGraph);
		mProp_T = get(meta_data_t(), mPCGraph_T);
		topological_sort(mPCGraph, std::back_inserter(mTopologicalOrder));
		mNumTasks = boost::num_vertices(_pcg);

		//set up graph preprocessing
		mNumVertices = num_vertices(mGraph);
		// std::cout << mAllPairsShortestPathMap.size() << std::endl;

		//initialise the search problem
		mPredecessors = std::vector <std::vector<int>> (mNumTasks);
		mSuccessors = std::vector <std::vector<int>> (mNumTasks);
		mTasksList = std::vector <std::pair <int,int> > (mNumTasks);
		mTasksToAgentsList = std::vector <std::vector <int>> (mNumTasks);
		mAgentsToTasksList = std::vector <std::vector <int>> (mNumAgents);
		mAgentHeuristics = std::vector <boost::unordered_map <int, double>> (mNumAgents);

		for ( std::vector< PCVertex >::reverse_iterator ii=mTopologicalOrder.rbegin(); 
			ii!=mTopologicalOrder.rend(); ++ii)
		{
			meta_data vertex = get(get(meta_data_t(), _pcg), *ii);
			int task_id = *ii; 
			std::pair <Vertex, Vertex> taskVertices = getVertexFromGraph(vertex);
			mTasksList[task_id] = taskVertices;
			mTasksToAgentsList[task_id] = vertex.agent_list;
			for(auto agent_id: vertex.agent_list){
				mAgentsToTasksList[agent_id].push_back(task_id);
			}
			PCOutEdgeIter ei, ei_end;
			vector <int> predecessors, successors;
			for (boost::tie(ei, ei_end) = out_edges(*ii, mPCGraph_T); ei != ei_end; ++ei) 
			{
				PCVertex curPred = target(*ei, mPCGraph_T);
				predecessors.push_back(curPred);
			}
			for (boost::tie(ei, ei_end) = out_edges(*ii, mPCGraph); ei != ei_end; ++ei) 
			{
				PCVertex curSuc = target(*ei, mPCGraph);
				successors.push_back(curSuc);
			}
			mPredecessors[task_id]=predecessors;
			mSuccessors[task_id]=successors;
		}

		// for(int agent=0; agent<mNumAgents; agent++){
		// 	double cost = 0;
		// 	for(int task_no = mAgentsToTasksList[agent].size()-1; task_no>=0; task_no--){
		// 		cost = cost + edgeWeights[]
		// 	}
		// }
		auto t1 = std::chrono::high_resolution_clock::now();
	    auto t2 = std::chrono::high_resolution_clock::now();
		mCSPTime = t2-t1;mGNTime = t2-t1;mQOTime = t2-t1;mCCTime = t2-t1;
		mPlanningTime = t2-t1;mPreprocessTime = t2-t1;mCSPTime = t2-t1;mHeuristicsTime = t2-t1;
		mCollabCTime = t2-t1;mCollisionCTime = t2-t1;mGNTime = t2-t1;mQOTime = t2-t1;
		mCCTime = t2-t1;mCHTime = t2-t1;mPlanningTime = t2-t1;mPreprocessTime = t2-t1;
		mMapOperationsTime = t2-t1;mGetCollisionTime = t2-t1;mGetCollaborationTime = t2-t1;
		mCollisionIterations = 0;mCollaborationIterations = 0;
		mCSPExpansions = 0;mCSPIterations = 0;

		// std::cout << "initialise the search problem\n";
		// std::cin.get();
	}

	bool isEqual(double x1, double x2){
		if(std::abs(x1-x2) < 0.0001){
			return true;
		}
		return false;
	}
	std::pair <Vertex, Vertex> getVertexFromGraph(meta_data vertex){
		Eigen::VectorXd start_config(2);
		start_config[0] = vertex.start.first;
		start_config[1] = vertex.start.second;

		Eigen::VectorXd goal_config(2);
		goal_config[0] = vertex.goal.first;
		goal_config[1] = vertex.goal.second;

		Vertex s,g; VertexIter ind_vi, ind_vi_end;
		for (boost::tie(ind_vi, ind_vi_end) = vertices(mGraph); ind_vi != ind_vi_end; ++ind_vi)
		{
			if(isEqual(start_config[0], mGraph[*ind_vi].state[0])
			 && isEqual(start_config[1], mGraph[*ind_vi].state[1])){
				// std::cerr << "found start here\n";
				s = *ind_vi;
			}
			if(isEqual(goal_config[0], mGraph[*ind_vi].state[0])
			 && isEqual(goal_config[1], mGraph[*ind_vi].state[1])){
				// std::cerr << "found goal here\n";
				g = *ind_vi;
			}
			// if(goal_config[0] == mGraph[*ind_vi].state[0] && goal_config[1] == mGraph[*ind_vi].state[1]){
			// 	std::cerr << "found goal here\n";
			// 	g = *ind_vi;
			// }
			// if(start_config.isApprox(mGraph[*ind_vi].state)){
			// 	std::cout << "found start\n";
			// 	s = *ind_vi;
			// }
			// if(goal_config.isApprox(mGraph[*ind_vi].state))
			// 	g = *ind_vi;
		}
		return std::make_pair(s,g);
	}

	int getStateHash(Eigen::VectorXd state)
	{
		return int(1.001/mUnitEdgeLength)*int((state[0]+0.001)/mUnitEdgeLength) 
		+ int((state[1]+0.001)/mUnitEdgeLength);
	}

	bool isOverFlowState(SearchState &state, SearchState &goal_state)
	{
		if(state.timestep > goal_state.timestep)
			return true;
		if(state.tasks_completed > goal_state.tasks_completed) 
			return true;
		return false;
	}

	void printStats()
	{
		std::cout<<mPlanningTime.count()/1000000.0<<" "<<mCBSIterations<<std::endl;
		// std::cout<<"computeShortestPath time: "<<mCSPTime.count()/1000000.0<<std::endl;
		// std::cout<<"Queue Operations time: "<<mQOTime.count()/1000000.0<<std::endl;
		// std::cout<<"Get Neighbors time: "<<mGNTime.count()/1000000.0<<std::endl;
		// std::cout<<"Constraints time: "<<mCCTime.count()/1000000.0<<std::endl;
		// std::cout<<"Preproccessing time: "<<mPreprocessTime.count()/1000000.0<<std::endl;
		// std::cout<<"Get heuristics time: "<<mHeuristicsTime.count()/1000000.0<<std::endl;
		// std::cout<<"Collision Hashing time: "<<mCHTime.count()/1000000.0<<std::endl;
		// std::cout<<"Map operations time: "<<mMapOperationsTime.count()/1000000.0<<std::endl;
		// std::cout<<"Count Collision ConflictsTime: "<<mCollisionCTime.count()/1000000.0<<std::endl;
		// std::cout<<"Count Collaboration Conflicts Time: "<<mCollabCTime.count()/1000000.0<<std::endl;
		// std::cout<<"Get Collision ConflictsTime: "<<mGetCollisionTime.count()/1000000.0<<std::endl;
		// std::cout<<"Get Collaboration Conflicts Time: "<<mGetCollaborationTime.count()/1000000.0<<std::endl;

		// for (auto it = mWaypointHashMap.begin(); it != mWaypointHashMap.end(); it++)
		// {
		//     printCollabConstraint(it->first);
		//     std::cout <<"Number of times = "    // string (key)
		//               << ':'
		//               << it->second   // string's value 
		//               << std::endl;
		// }
	}


	int countCollaborationConflicts(SearchState &state, int agent_id, std::vector <std::pair <int, int>> taskRanges)
	{
		if(state.tasks_completed==mAgentsToTasksList[agent_id].size()) return 0;
		int tid = mAgentsToTasksList[agent_id][state.tasks_completed];

		int max_time = -1;
		for(auto pred: mPredecessors[tid]){
			max_time = std::max(max_time, taskRanges[pred].second);
		}

		if(taskRanges[tid].first < max_time){
			return 1;
		}
		return 0;
	}

	bool getCollaborationConstraints(Element &p, int &tid, int &split)
	{
		std::vector <std::pair <int, int>> taskRanges(mNumTasks, std::make_pair(100000, -1));

		for(int i=0; i<mNumAgents; i++){
			for(int j=0; j<p.shortestPaths[i].size(); j++){
				SearchState state = p.shortestPaths[i][j];
				if(state.tasks_completed == mAgentsToTasksList[i].size()){
					continue;
				}
				int task_id = mAgentsToTasksList[i][state.tasks_completed];
				if(state.vertex == mTasksList[task_id].first){
					taskRanges[task_id].first = std::min(taskRanges[task_id].first, state.timestep);
				}
				if(state.vertex == mTasksList[task_id].second){
					taskRanges[task_id].second = std::max(taskRanges[task_id].second, state.timestep);
				}
			}
		}

		for ( std::vector< PCVertex >::reverse_iterator ii=mTopologicalOrder.rbegin(); 
			ii!=mTopologicalOrder.rend(); ++ii)
		{
			int i = *ii;
			int max_time = -1;
			int min_time = 10000;
			int max_time_task = -1;
			for(auto pred: mPredecessors[i]){
				// if(taskRanges[pred].second>max_time){
				// 	max_time_task = pred;
				// }
				max_time = std::max(max_time, taskRanges[pred].second);
				min_time = std::min(min_time, taskRanges[pred].second);
			}

			if(taskRanges[i].first < max_time){
				max_time = std::min(max_time-1, p.nonCollabMap[i].first.maxTime-1);
				min_time = std::max(min_time, p.nonCollabMap[i].first.minTime);
				int avg_time = (max_time+min_time)/2;
				split = max_time;
				// if(mSplitStrategy=="0")
				// 	split = max_time;
				// else if(mSplitStrategy=="1")
				// 	split = avg_time;
				// else if(mSplitStrategy=="2")
				// 	split = min_time;
				// else{
				// 	std::cout << "no split strategy boi\n";
				// 	std::cin.get();
				// }
				tid = i;
				// std::cout << "Agents assigned = \n";
				// for(auto agent: mTasksToAgentsList[i]){
				// 	std::cout << agent << " ";
				// }
				// std::cout << std::endl;
				// std::cout << "TASK ID = " << i << std::endl;
				// std::cout << "predecessors = \n";
				// for(auto pred: mPredecessors[i]){
				// 	std::cout << pred << " ";
				// }
				// std::cout << std::endl;
				// std::cout << "predecessor max = " << max_time_task << std::endl;
				// std::cout << taskRanges[i].first << " " << max_time << std::endl;
				return true;
			}
		}
		return false;
	}

	int countCollisionConflicts(
		boost::unordered_map <std::pair <int, int>, std::vector<SearchState>> mVertexMap,
	 	SearchState prev, SearchState curr, int task_id, int agent_id)
	{
		std::pair <int, int> k = std::make_pair(curr.vertex, curr.timestep);
		if(mVertexMap.find(k)==mVertexMap.end())
			return 0;

		std::vector <SearchState> possible = mVertexMap[k];

		std::unordered_map <int, std::vector <int>> agentMap;
		for(auto state: possible){
			agentMap[state.timestep].push_back(state.tasks_completed);
		}
		int tid_1 = task_id, tid_2 = -2;
		if(mTasksList[task_id].second==curr.vertex){
			if(curr.tasks_completed<mAgentsToTasksList[agent_id].size()-1)
				tid_2 = mAgentsToTasksList[agent_id][curr.tasks_completed+1];
		}
		if(mTasksList[task_id].first==curr.vertex){
			if(curr.tasks_completed>0)
				tid_2 = mAgentsToTasksList[agent_id][curr.tasks_completed-1];
		}
		for(auto a1: agentMap){
			bool safe = false;
			for(auto t1: a1.second){
				if(t1==tid_1 && t1!=-1) safe = true;
				if(t1==tid_2 && t1!=-1) safe = true;
			}
			if(!safe) return 1;
		}

		if(curr.vertex == prev.vertex) return 0;
		std::unordered_map <int, int> sourceMap;	
		k = std::make_pair(curr.vertex, curr.timestep-1);
		for(auto state: mVertexMap[k]) sourceMap[state.timestep] = 1;
		std::unordered_map <int, int> targetMap;
		k = std::make_pair(prev.vertex, curr.timestep);
		for(auto state: mVertexMap[k]) targetMap[state.timestep] = 1;

		for(auto it: sourceMap){
			if(sourceMap[it.first] && targetMap[it.first]) return 1;
		}
		return 0;
	}

	bool getCollisionConstraints(Element &p, 
		std::vector<int> &collaborating_agent_ids_1, std::vector <int> &collaborating_agent_ids_2,
		CollisionConstraint &constraint_1, CollisionConstraint &constraint_2,
		int makespan)
	{
		std::vector <std::vector <SearchState>> timestepMap (makespan+1);

		for(int i=0; i<mNumAgents; i++){
			for(int j=0; j<p.shortestPaths[i].size(); j++){
				SearchState state = p.shortestPaths[i][j];
				if(state.tasks_completed == mAgentsToTasksList[i].size()){
					SearchState key = SearchState(state.vertex, i, state.tasks_completed);
					timestepMap[state.timestep].push_back(key);
					if(j==p.shortestPaths[i].size()-1){
						for(int time=state.timestep+1; time <= makespan; time++){
							SearchState key = SearchState(state.vertex, i, state.tasks_completed);
							timestepMap[time].push_back(key);
						}
						break;
					}
					continue;
				}
				int task_id = mAgentsToTasksList[i][state.tasks_completed];
				SearchState key = SearchState(state.vertex, i, state.tasks_completed);
				timestepMap[state.timestep].push_back(key);
			}
		}

		//check vertex conflict
		for(int time=0; time<=makespan; time++){
			boost::unordered_map <Vertex, std::vector<SearchState>> vertexMap;
			for(auto state: timestepMap[time]){
				vertexMap[state.vertex].push_back(state);
			}

			for(auto it: vertexMap){
				Vertex v = it.first;
				std::vector <SearchState> colStates = it.second;
				std::unordered_map <int, std::vector <int>> agentMap;
				for(auto state: colStates){
					int tid_1, tid_2;
					int i = state.timestep;
					if(state.tasks_completed == mAgentsToTasksList[i].size()){
						tid_1 = -1;
						tid_2 = mAgentsToTasksList[i][mAgentsToTasksList[i].size()-1];
						agentMap[state.timestep] = std::vector <int> {tid_1, tid_2};
						continue;
					}
					else{
						tid_1 = mAgentsToTasksList[i][state.tasks_completed];
						agentMap[state.timestep] = std::vector <int> {tid_1};
					}
					if(state.vertex == mTasksList[tid_1].first){
						if(state.tasks_completed>0){
							agentMap[state.timestep].push_back(mAgentsToTasksList[i][state.tasks_completed-1]);
						}
					}
					if(state.vertex == mTasksList[tid_1].second){
						if(state.tasks_completed < mAgentsToTasksList[i].size()-1){
							agentMap[state.timestep].push_back(mAgentsToTasksList[i][state.tasks_completed+1]);
						}
					}
				}

				for(auto a1: agentMap){
					for(auto a2: agentMap){
						if(a1.first==a2.first) continue;
						int tid_1, tid_2;
						bool safe = false;
						for(auto t1: a1.second){
							for(auto t2: a2.second){
								if(t1==t2 && t1!=-1) safe = true;
							}
						}
						if(safe) continue;
						tid_1 = a1.second[0], tid_2 = a2.second[0];
						if(tid_1!=-1) collaborating_agent_ids_1 = mTasksToAgentsList[tid_1];
						else collaborating_agent_ids_1 = std::vector <int> {a1.first};

						if(tid_2!=-1) collaborating_agent_ids_2 = mTasksToAgentsList[tid_2];
						else collaborating_agent_ids_2 = std::vector <int> {a2.first};

						constraint_1 = CollisionConstraint(v, tid_1, time);
						constraint_2 = CollisionConstraint(v, tid_2, time);
						return true;
					}
				}
			}
		}

		//check edge conflict
		for(int time=0; time<=makespan-1; time++){
			std::vector <std::pair <SearchState, SearchState>> agentLocations (mNumAgents);
			for(auto state: timestepMap[time]){
				int agent_id = state.timestep;
				if(state.tasks_completed < mAgentsToTasksList[agent_id].size())
					state.tasks_completed = mAgentsToTasksList[agent_id][state.tasks_completed];
				else
					state.tasks_completed = -1;
				agentLocations[agent_id].first = state;
			}
			for(auto state: timestepMap[time+1]){
				int agent_id = state.timestep;
				if(state.tasks_completed < mAgentsToTasksList[agent_id].size())
					state.tasks_completed = mAgentsToTasksList[agent_id][state.tasks_completed];
				else
					state.tasks_completed = -1;
				agentLocations[agent_id].first = state;
			}

			for(int a1=0; a1<mNumAgents; a1++){
				for(int a2=a1+1; a2<mNumAgents; a2++){
					//check if collision
					if(agentLocations[a1].first.vertex == agentLocations[a2].second.vertex &&
						agentLocations[a2].first.vertex == agentLocations[a1].second.vertex &&
						agentLocations[a1].first.vertex != agentLocations[a1].second.vertex){
						//assign conflicts
						int tid_1 = agentLocations[a1].second.tasks_completed;
						int tid_2 = agentLocations[a2].second.tasks_completed;

						Edge edge_1 = boost::edge(agentLocations[a1].first.vertex,
							agentLocations[a1].second.vertex,mGraph).first;
						Edge edge_2 = boost::edge(agentLocations[a2].first.vertex,
							agentLocations[a2].second.vertex,mGraph).first;

						if(tid_1!=-1) collaborating_agent_ids_1 = mTasksToAgentsList[tid_1];
						else collaborating_agent_ids_1 = std::vector <int> {a1};

						if(tid_2!=-1) collaborating_agent_ids_2 = mTasksToAgentsList[tid_2];
						else collaborating_agent_ids_2 = std::vector <int> {a2};

						constraint_1 = CollisionConstraint(edge_1, tid_1, time+1);
						constraint_1.v1 = source(constraint_1.e, mGraph);
						constraint_1.v2 = target(constraint_1.e, mGraph);

						constraint_2 = CollisionConstraint(edge_2, tid_2, time+1);
						constraint_2.v1 = source(constraint_2.e, mGraph);
						constraint_2.v2 = target(constraint_2.e, mGraph);
						
						return true;
					}
				}
			}
		}
		return false;
	}

	int getMakespan(std::vector <std::vector <SearchState>> paths){
		int makespan = 0;
		for(auto path: paths){
			makespan = std::max(makespan, path.at(path.size()-1).timestep);
		}
		return makespan;
	}

	Element expandCollisionConstraint(Element p, 
		std::vector <int> collaborating_agent_ids, 
		CollisionConstraint constraint, bool &possible)
	{
		int current_makespan = getMakespan(p.shortestPaths);
		std::vector<int> consider_agents;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			if(std::find(collaborating_agent_ids.begin(),
				collaborating_agent_ids.end(), agent_id) == 
				collaborating_agent_ids.end())
				consider_agents.push_back(agent_id);

		std::vector<boost::unordered_map <CollisionConstraint, int, 
					collision_hash>> increase_constraints_c = p.nonCollisionMap;
		std::vector< std::vector<SearchState> > shortestPaths_c = p.shortestPaths;

		bool all_paths_exist = true; double cost_c;
		std::sort( collaborating_agent_ids.begin(), collaborating_agent_ids.end(), 
			[&]( int first, int second)
		{
		   return shortestPaths_c[first].back().timestep >= shortestPaths_c[second].back().timestep;
		});
		for(int i=0; i<collaborating_agent_ids.size(); i++)
		{
			increase_constraints_c[collaborating_agent_ids[i]][constraint]=1;
			shortestPaths_c[collaborating_agent_ids[i]] = 
					computeShortestPath(collaborating_agent_ids[i],
						consider_agents, cost_c, current_makespan,
						p.nonCollabMap, increase_constraints_c[collaborating_agent_ids[i]],
						shortestPaths_c);

			if(cost_c == INF)
			{
				all_paths_exist = false;
				break;
			}

			consider_agents.push_back(collaborating_agent_ids[i]);
		}
		if(all_paths_exist)
		{
			Element expandedNode(p.nonCollabMap, increase_constraints_c, shortestPaths_c);
			possible = true;
			return expandedNode;
		}
		possible = false;
		return p;
	}

	Element expandCollaborationConstraint(Element p, 
		std::vector <std::pair<allowedInterval, allowedInterval>> nonCollabMap, 
		bool &possible)
	{
		int current_makespan = getMakespan(p.shortestPaths);

		//find agents violating the nonCollabMap
		std::set <int> collaborating_agent_set;
		for(int i=0; i<mNumAgents; i++){
			for(int j=0; j<p.shortestPaths[i].size(); j++){
				SearchState state = p.shortestPaths[i][j];
				if(state.tasks_completed == mAgentsToTasksList[i].size()){
					continue;
				}
				int task_id = mAgentsToTasksList[i][state.tasks_completed];
				if(state.vertex == mTasksList[task_id].first){
					if(state.timestep<nonCollabMap[task_id].first.minTime ||
						state.timestep>nonCollabMap[task_id].first.maxTime)
							collaborating_agent_set.insert(i);
				}
				if(state.vertex == mTasksList[task_id].second){
					if(state.timestep<nonCollabMap[task_id].second.minTime ||
						state.timestep>nonCollabMap[task_id].second.maxTime)
							collaborating_agent_set.insert(i);
				}
			}
		}

		std::vector <int> collaborating_agent_ids;
		for(auto a: collaborating_agent_set) collaborating_agent_ids.push_back(a);

		std::vector<int> consider_agents;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			if(std::find(collaborating_agent_ids.begin(),collaborating_agent_ids.end(), agent_id) 
				== collaborating_agent_ids.end())
				consider_agents.push_back(agent_id);

		std::vector< std::vector<SearchState> > shortestPaths_c = p.shortestPaths;

		bool all_paths_exist = true; double cost_c;

		std::sort( collaborating_agent_ids.begin(), collaborating_agent_ids.end(), 
			[&]( int first, int second)
		{
		   return shortestPaths_c[first].back().timestep >= shortestPaths_c[second].back().timestep;
		});
		for(int i=0; i<collaborating_agent_ids.size(); i++)
		{
			shortestPaths_c[collaborating_agent_ids[i]] = 
					computeShortestPath(collaborating_agent_ids[i],
						consider_agents, cost_c, current_makespan,
						nonCollabMap, p.nonCollisionMap[collaborating_agent_ids[i]],
						shortestPaths_c);
			if(cost_c == INF)
			{
				all_paths_exist = false;
				break;
			}
			consider_agents.push_back(collaborating_agent_ids[i]);
		}
		// std::cout << "what\n";
		if(all_paths_exist)
		{
			Element expandedNode(nonCollabMap, p.nonCollisionMap, shortestPaths_c);
			possible = true;
			// printIntervals(nonCollabMap);
			// printNode(expandedNode);
			return expandedNode;
		}
		// std::cout << "what\n";
		possible = false;
		return p;
	}

	bool isValid(allowedInterval Interval){
		if(Interval.maxTime>=Interval.minTime) return true;
		return false;
	}

	void updateSchedule(std::vector <std::pair<allowedInterval, allowedInterval>> &nonCollabMap,
		bool &possible){

		//go left to right and update
		for (container::reverse_iterator ii=mTopologicalOrder.rbegin(); ii!=mTopologicalOrder.rend(); ++ii)
		{
			int task_id = *ii;
			vector <int> predecessors = mPredecessors[task_id];
			for(auto pred: predecessors){
				nonCollabMap[task_id].first.minTime  = std::max(
					nonCollabMap[task_id].first.minTime, 
					nonCollabMap[pred].second.minTime 
					);

				// nonCollabMap[task_id].first.maxTime  = std::min(
				// 	nonCollabMap[task_id].first.maxTime, 
				// 	nonCollabMap[pred].second.maxTime 
				// 	);
			}
			nonCollabMap[task_id].second.minTime  = std::max(
					nonCollabMap[task_id].second.minTime, 
					nonCollabMap[task_id].first.minTime + 
					mAllPairsShortestPathMap[mTasksList[task_id].first][mTasksList[task_id].second]
				);
		}

		//go right to left and update
		for (container::iterator ii=mTopologicalOrder.begin(); ii!=mTopologicalOrder.end(); ++ii)
		{
			int task_id = *ii;
			vector <int> successors = mSuccessors[task_id];
			for(auto succ: successors){
				// nonCollabMap[task_id].second.minTime  = std::max(
				// 	nonCollabMap[task_id].second.minTime,
				// 	nonCollabMap[succ].first.minTime 
				// 	);

				nonCollabMap[task_id].second.maxTime  = std::min(
					nonCollabMap[task_id].second.maxTime,
					nonCollabMap[succ].first.maxTime 
					);
			}
			nonCollabMap[task_id].first.maxTime  = std::min(
					nonCollabMap[task_id].first.maxTime, 
					nonCollabMap[task_id].second.maxTime - 
					mAllPairsShortestPathMap[mTasksList[task_id].first][mTasksList[task_id].second]
				);
			// if(successors.size()==0){
			// 	std::cout << nonCollabMap[task_id].second.minTime << std::endl;
			// }
		}

		//check validity
		for (int task_id = 0; task_id < mNumTasks; task_id++)
		{
			if(!isValid(nonCollabMap[task_id].first) || !isValid(nonCollabMap[task_id].second)){
				possible=false;
				return;
			}
		}
		possible = true;
	}

	std::pair <Element, Element> expandCollaborationConflict(Element p,
		int task_id, int split,
		bool &possible1, bool &possible2)
	{
		allowedInterval curInterval = p.nonCollabMap[task_id].first;

		Element maxNode = p, minNode = p; 
		std::vector <std::pair<allowedInterval, allowedInterval>> increasedMap;
		allowedInterval newInterval;

		newInterval = allowedInterval(curInterval.minTime, std::min(curInterval.maxTime-1, split));
		increasedMap = p.nonCollabMap;
		increasedMap[task_id].first = newInterval;
		// if(mType=="0")
			updateSchedule(increasedMap, possible1);
		if(possible1)
			maxNode = expandCollaborationConstraint(p, increasedMap, possible1);

		newInterval = allowedInterval(std::max(curInterval.minTime+1, split+1), curInterval.maxTime);
		increasedMap = p.nonCollabMap;
		increasedMap[task_id].first = newInterval;
		// if(mType=="0")
			updateSchedule(increasedMap, possible2);
		if(possible2)
			minNode = expandCollaborationConstraint(p, increasedMap, possible2);

		return std::make_pair(maxNode, minNode);
	}

	std::pair <Element, Element> expandCollisionConflict(Element p,
		std::vector <int> collaborating_agent_ids_1, CollisionConstraint constraint_1,
		std::vector <int> collaborating_agent_ids_2, CollisionConstraint constraint_2,
		bool &possible1, bool &possible2)
	{
		Element expandedNode1 = expandCollisionConstraint(p, collaborating_agent_ids_1,
			constraint_1, possible1);
		Element expandedNode2 = expandCollisionConstraint(p, collaborating_agent_ids_2,
			constraint_2, possible2);
		return std::make_pair(expandedNode1, expandedNode2);
	}

	std::vector<std::vector<Eigen::VectorXd>> solve()
	{
		mCBSIterations = 0;
		mSolveStartTime = high_resolution_clock::now();
		auto solve_start = high_resolution_clock::now();
		CBSPriorityQueue PQ(mNumAgents, mNumTasks);

		std::vector <std::pair <allowedInterval, allowedInterval>> nonCollabMap (mNumTasks);
		std::vector <boost::unordered_map <CollisionConstraint, int, 
			collision_hash>> nonCollisionMap(mNumAgents);
		// std::cin.get();
		bool possible;
		// if(mType=="0")

		// printIntervals(nonCollabMap);
			// std::cin.get();
		updateSchedule(nonCollabMap, possible);

		// printIntervals(nonCollabMap);
		// 	std::cin.get();
		if(!debug_disabled){
			std::cout << "Printing Initial Intervals\n";
			printIntervals(nonCollabMap);
			std::cin.get();
		}
		if(!possible){
			std::cout << "Problem is wrong.\n";
			std::cin.get();
		}

		std::vector< std::vector<SearchState> > start_shortestPaths = 
			computeDecoupledPaths(nonCollabMap, nonCollisionMap);

		// std::cerr << "Computed decoupled paths\n";
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			if(start_shortestPaths.at(agent_id).size()==0)
			{
				// std::cout << "Couldnt solve problem for agent_id = " << agent_id << std::endl;
				auto solve_stop = high_resolution_clock::now();
				mPlanningTime = (solve_stop - solve_start);
				std::cout<<0<<" ";
				return std::vector<std::vector<Eigen::VectorXd>>(mNumAgents,std::vector<Eigen::VectorXd>());
			}
		}
		
		Element p = Element(nonCollabMap, nonCollisionMap, start_shortestPaths);
		PQ.insert(p);

		int numSearches = 0;
		while(PQ.PQsize()!=0)
		{
			// std::cout << PQ.PQsize() << std::endl;
			numSearches++;
			mCBSIterations++;
			auto start = high_resolution_clock::now();
			Element p = PQ.pop();
			auto stop = high_resolution_clock::now();
			mQOTime += (stop - start);
			std::chrono::duration<double, std::micro> timespent = stop - mSolveStartTime;
			if(debug_disabled){
				if (timespent.count() > 300000000)
				{
					auto solve_stop = high_resolution_clock::now();
					mPlanningTime = (solve_stop - mSolveStartTime);
					std::cout<<0<<" ";
					return std::vector<std::vector<Eigen::VectorXd>>(mNumAgents,std::vector<Eigen::VectorXd>());
				}
			}

			std::vector<std::vector<SearchState>> paths = p.shortestPaths;
			int maximum_timestep=getMakespan(paths);		
			double total_cost = maximum_timestep*mUnitEdgeLength;
	
			if(!debug_disabled){
				std::cout<<"MT: "<<maximum_timestep<<std::endl;
				std::cout << "---------------INTERVALS-------------------"<< std::endl;
				printIntervals(p.nonCollabMap);
				std::cout << "---------------SELECTED NODE-------------------"<< std::endl;
				printNode(p);
				std::cin.get();
			}

			int task_id, split;

			if(getCollaborationConstraints(p, task_id, split))
			{
				// std::cout << "-----Colab Conflict Found-----------" << std::endl;
				// std::cin.get();
				if(!debug_disabled){
					std::cout << "-----Colab Conflict Found-----------" << std::endl;
					std::cout << "----- Task ID = " << task_id << std::endl;
					std::cout << "----- Split = " << split << std::endl;
					std::cin.get();
				}

				bool possible1 = false, possible2 = false;
				std::pair <Element, Element> colabChildren = 
					expandCollaborationConflict(p, task_id, split, possible1, possible2);
				if(possible1) PQ.insert(colabChildren.first); 
				if(possible2) PQ.insert(colabChildren.second);

				if(!debug_disabled){
					if(possible1){
						std::cout << "---------------EXPANDED NODE 1-------------------"<< std::endl;
						std::cout << "---------------INTERVALS-------------------"<< std::endl;
						printIntervals(colabChildren.first.nonCollabMap);
						printNode(colabChildren.first);
						std::cin.get();
					}
					if(possible2){
						std::cout << "---------------EXPANDED NODE 2-------------------"<< std::endl;
						std::cout << "---------------INTERVALS-------------------"<< std::endl;
						printIntervals(colabChildren.second.nonCollabMap);
						printNode(colabChildren.second);
						std::cin.get();
					}	
				}
				continue;
			}				

			PRINT<<"NO COLLABORATION CONSTRAINT!!"<<std::endl;

			std::vector <int> collaborating_agent_ids_1, collaborating_agent_ids_2;
			CollisionConstraint constraint_1, constraint_2;

			if(getCollisionConstraints(p, 
				collaborating_agent_ids_1,collaborating_agent_ids_2,
				constraint_1, constraint_2, maximum_timestep))
			{
				// std::cout << "-----Colision Conflict Found-----------" << std::endl;
				// std::cin.get();
				if(!debug_disabled){
					std::cout << "-----Colision Conflict Found-----------" << std::endl;
					printColConflict(collaborating_agent_ids_1, constraint_1, 
						collaborating_agent_ids_2, constraint_2);
					std::cin.get();
				}
				bool possible1 = false, possible2 = false;
				std::pair <Element, Element> colabChildren = 
				expandCollisionConflict(p, collaborating_agent_ids_1, constraint_1,
					collaborating_agent_ids_2, constraint_2, possible1, possible2);
				if(possible1) PQ.insert(colabChildren.first); 
				if(possible2) PQ.insert(colabChildren.second);
				if(!debug_disabled){
					if(possible1){
						std::cout << "---------------EXPANDED NODE 1-------------------"<< std::endl;
						printNode(colabChildren.first);
					}
					if(possible2){
						std::cout << "---------------EXPANDED NODE 2-------------------"<< std::endl;
						printNode(colabChildren.second);
					}
					std::cin.get();
				}
				continue;
			} 

			// NO CONFLICTS --- FORM COLLISION FREE PATHS to pass into display
			std::vector<std::vector<Eigen::VectorXd>> 
			collision_free_path(mNumAgents, std::vector<Eigen::VectorXd>());
			int current_timestep = 0;

			std::vector<int> current_path_id(mNumAgents, 0);
			while(current_timestep <= maximum_timestep)
			{
				for(int agent_id=0; agent_id<mNumAgents; agent_id++)
				{
					if(current_path_id[agent_id] == paths[agent_id].size())
					{
						collision_free_path[agent_id].push_back(mGraph[paths[agent_id].at(paths[agent_id].size()-1).vertex].state);
					}
					else
					{
						collision_free_path[agent_id].push_back(mGraph[paths[agent_id].at(current_path_id[agent_id]).vertex].state);
						while(current_path_id[agent_id] < paths[agent_id].size()
							&& paths[agent_id].at(current_path_id[agent_id]).timestep == current_timestep)
							current_path_id[agent_id]++;
					}
				}
				current_timestep++;
			}

			std::vector<Eigen::VectorXd> path_configs;

			for(int i=0; i<collision_free_path[0].size(); i++)
			{
				Eigen::VectorXd config(collision_free_path.size()*2);
				for(int j=0; j<collision_free_path.size(); j++)
				{
					config[2*j]=collision_free_path[j][i][0];
					config[2*j+1]=collision_free_path[j][i][1];
				}
				path_configs.push_back(config);
			}
			std::cout<<path_configs.size()<<" ";
			auto solve_stop = high_resolution_clock::now();
			mPlanningTime = (solve_stop - solve_start);
			if(!debug_disabled){
				std::cout<<"Press [ENTER] to display path: ";
				std::cin.get();
				displayPath(path_configs);	
				printStats();	
			}
			// std::cout<<"Press [ENTER] to display path: ";
			// std::cin.get();
			// displayPath(path_configs);
			// printStats();
			return collision_free_path;
		}
		std::cout << "CBS FAILED WTF IS WRONG\n";
		return std::vector<std::vector<Eigen::VectorXd>>(mNumAgents,std::vector<Eigen::VectorXd>());
	}

	std::vector< std::vector<SearchState> > computeDecoupledPaths(
		std::vector <std::pair <allowedInterval, allowedInterval>> nonCollabMap,
		std::vector<boost::unordered_map <CollisionConstraint, int, collision_hash>> &nonCollisionMap)
	{
		std::vector<std::vector<SearchState> > shortestPaths (mNumAgents);
		std::vector<std::vector<SearchState> > dummyPaths;
		std::vector<int> consider_agents;
		// std::vector <std::pair <allowedInterval, allowedInterval>> nonCollabMap1;
		int current_makespan = 0;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			double ind_cost;
			std::vector <SearchState> path = 
					computeShortestPath(agent_id,consider_agents,
					ind_cost, 0, 
					nonCollabMap, nonCollisionMap[agent_id],
					shortestPaths);
			shortestPaths[agent_id] = path;
			consider_agents.push_back(agent_id);
			// std::cout << "agent id = " << agent_id << " size of path = " << path.back().timestep << std::endl;
		}
		return shortestPaths;
	}

	std::vector<SearchState> computeShortestPath(int &agent_id, std::vector<int> &consider_agents, 
		double& costOut, int current_makespan, 
		std::vector <std::pair <allowedInterval, allowedInterval>> nonCollabMap,
		boost::unordered_map <CollisionConstraint, int, collision_hash> &nonCollisionMap,
		std::vector<std::vector<SearchState> > &shortestPaths)
	{
		// std::cout << "yo\n";
		auto start1 = high_resolution_clock::now();
		SearchState start_state = SearchState(mTasksList[mAgentsToTasksList[agent_id][0]].first, 0, 0);
		SearchState goal_state;

		boost::unordered_map <std::pair <int, int>, std::vector<SearchState>> mVertexMap;
		std::vector<std::pair<int,int>> taskRanges (mNumTasks, std::make_pair(100000, -1));
		for(auto i: consider_agents){
			if(i==agent_id) continue;
			for(int j=0;j<shortestPaths[i].size(); j++){
				SearchState curr = shortestPaths[i][j];
				int task_id;
				if(curr.tasks_completed == mAgentsToTasksList[i].size()){
					task_id = -1;
				}
				else{
					task_id = mAgentsToTasksList[i][curr.tasks_completed];
				}

				SearchState key = SearchState(curr.vertex, agent_id, task_id);
				std::pair <int, int> k = std::make_pair(curr.vertex, curr.timestep);
				if(mVertexMap.find(k)==mVertexMap.end()){
					mVertexMap[k] = std::vector <SearchState> {key};
				}
				else{
					mVertexMap[k].push_back(key);
				}

				SearchState state = shortestPaths[i][j];
				if(state.tasks_completed == mAgentsToTasksList[i].size()){
					continue;
				}
				if(state.vertex == mTasksList[task_id].first){
					taskRanges[task_id].first = std::min(taskRanges[task_id].first, state.timestep);
				}
				if(state.vertex == mTasksList[task_id].second){
					taskRanges[task_id].second = std::max(taskRanges[task_id].second, state.timestep);
				}
			}
		}

		timePriorityQueue pq;
		boost::unordered_map<SearchState, std::vector<double>, state_hash> mFValue;
		boost::unordered_map<SearchState , SearchState, state_hash > mPrev;

		double true_makespan = current_makespan*mUnitEdgeLength;
		
		int numSearches = 0;
		int goal_timestep = -1;
		int min_goal_timestep = 0;
		for( auto it: nonCollisionMap)
		{
			min_goal_timestep = std::max(min_goal_timestep, (it.first).timestep);
		}
		mFValue[start_state] = getHeuristics(agent_id, start_state, 0, true_makespan, 
			0, 0, 0, 0, nonCollabMap, min_goal_timestep);
		pq.insert(start_state,mFValue[start_state]);
		costOut = INF;

		// std::cout << mTasksList[mAgentsToTasksList[agent_id][0]].first << " ";
		// printVertex(mTasksList[mAgentsToTasksList[agent_id][0]].first);
		// std::cout << mTasksList[mAgentsToTasksList[agent_id].back()].first << " ";
		// printVertex(mTasksList[mAgentsToTasksList[agent_id].back()].first);
		// mTasksList[mAgentsToTasksList[agent_id].back()].second << std::endl;
		// std::cout << nonCollabMap[mAgentsToTasksList[agent_id][1]].second.minTime << std::endl;
		mCSPExpansions = 0;
		while(pq.PQsize()!=0)
		{
			// std::cerr << "Queue size = " << pq.PQsize() << std::endl;
			mCSPExpansions++;
			numSearches++;
			SearchState current_state = pq.pop();	
			// printState(current_state);
			// std::cin.get();
			Vertex current_vertex = current_state.vertex;
			int current_timestep = current_state.timestep;
			int current_tasks_completed = current_state.tasks_completed;
			std::vector<double> current_fvalue = mFValue[current_state];
			double current_gvalue = current_timestep*mUnitEdgeLength;
			double current_count_collaboration_conflicts = current_fvalue[2];
			double current_count_collision_conflicts = current_fvalue[3];
			double current_count_move_actions = current_fvalue[3] - current_fvalue[4];
			// std::cout << "H_value = " << current_fvalue[3] << std::endl;

			auto stop = high_resolution_clock::now();
			std::chrono::duration<double, std::micro> timespent = stop - mSolveStartTime;
			if(debug_disabled){
				if (timespent.count() > 300000000)
				{
					// std::cout << "CSP TIMEOUT WTF IS WRONG\n";
					auto solve_stop = high_resolution_clock::now();
					mPlanningTime = (solve_stop - mSolveStartTime);
					costOut = INF;
					return std::vector<SearchState>();
				}
			}

			if(current_tasks_completed > mAgentsToTasksList[agent_id].size())
			{
				std::cout<<"Bruh.";
				std::cin.get();
			}
			if(current_tasks_completed == mAgentsToTasksList[agent_id].size() && current_timestep>= min_goal_timestep)
			{
				// std::cout << mCSPExpansions << std::endl;
				costOut = current_gvalue;
				goal_state = current_state;
				break;
			}


			int task_id;
			if(current_tasks_completed == mAgentsToTasksList[agent_id].size()) task_id = -1;
			else {
				task_id = mAgentsToTasksList[agent_id][current_tasks_completed];
				if(current_timestep + mAllPairsShortestPathMap[current_vertex][mTasksList[task_id].second]
					>nonCollabMap[task_id].second.maxTime) { 
					continue;
				}
					
				if(current_vertex == mTasksList[task_id].second) //end point of task
				{
					int new_timestep = current_timestep;
					bool allowed = true;
					if(current_timestep < nonCollabMap[task_id].second.minTime ||
						current_timestep > nonCollabMap[task_id].second.maxTime)
							allowed = false;
					int new_task_id;
					if(current_tasks_completed == mAgentsToTasksList[agent_id].size()-1){
						new_task_id = -1;
					}
					else{
						new_task_id = mAgentsToTasksList[agent_id][current_tasks_completed+1];
						if(current_timestep < nonCollabMap[new_task_id].first.minTime)
						{
							// if(mType=="sipp" || mType == "tie-sipp"){
							// 	bool expand = allowed;
							// 	new_timestep = nonCollabMap[new_task_id].first.minTime;
							// 	if(new_timestep > nonCollabMap[new_task_id].first.maxTime){
							// 		expand = false;
							// 	}
							// 	CollisionConstraint c2(current_vertex, new_task_id, new_timestep);
							// 	if(nonCollisionMap.find(c2)==nonCollisionMap.end() && expand){
									
							// 		SearchState old_state = current_state;
							// 		for(int new_state_time = current_timestep+1; new_state_time <= new_timestep;
							// 			new_state_time++){
							// 			SearchState new_state = SearchState(current_vertex, new_state_time, 
							// 			current_tasks_completed);

							// 			CollisionConstraint c2(current_vertex, task_id, new_state_time);
							// 			if(nonCollisionMap.find(c2)!=nonCollisionMap.end()){
							// 				expand = false;
							// 				break;
							// 			}
										
							// 			std::vector<double> new_cost = getHeuristics(agent_id, new_state, 
							// 				new_state_time*mUnitEdgeLength, true_makespan, 
							// 				current_count_collaboration_conflicts, current_count_collision_conflicts, 
							// 				current_count_move_actions, current_tasks_completed, 
							// 				nonCollabMap, min_goal_timestep);
							// 			mFValue[new_state]= new_cost;
							// 			mPrev[new_state]=old_state;
							// 			old_state = new_state;
							// 		}
							// 		if(expand){
							// 			SearchState new_state = SearchState(current_vertex, new_timestep, 
							// 			current_tasks_completed+1);
							// 			std::vector<double> new_cost = getHeuristics(agent_id, new_state, 
							// 				new_timestep*mUnitEdgeLength, true_makespan, 
							// 				current_count_collaboration_conflicts, current_count_collision_conflicts, 
							// 				current_count_move_actions, current_tasks_completed+1, 
							// 				nonCollabMap, min_goal_timestep);

							// 			CollisionConstraint c2(current_vertex, new_task_id, new_timestep);
							// 			if(nonCollisionMap.find(c2)!=nonCollisionMap.end()){
							// 				expand = false;
							// 			}
							// 			// std::cout << "Tasks Completed = " << new_cost[1] << std::endl;
							// 			if(expand && (mFValue.count(new_state)==0 || compareVectors(new_cost,mFValue[new_state])))
							// 			{
							// 				mFValue[new_state]= new_cost;
							// 				mPrev[new_state]=old_state;
							// 				pq.insert(new_state, new_cost);
							// 			}
							// 		}
							// 	}
							// }
							allowed = false;
						}
						if(current_timestep > nonCollabMap[new_task_id].first.maxTime){
							allowed = false;
						}

					}
					CollisionConstraint c2(current_vertex, new_task_id, current_timestep);
					if(nonCollisionMap.find(c2)!=nonCollisionMap.end()) allowed = false;
					if(allowed)
					{
						// std::cout << "LOOOOOOOOOOK AT ME\n";
						SearchState new_state = SearchState(current_vertex, current_timestep, 
							current_tasks_completed+1);

						double new_count_collaboration_conflicts = 
							current_count_collaboration_conflicts;// +
							// countCollaborationConflicts(new_state,agent_id,taskRanges);

						std::vector<double> new_cost = getHeuristics(agent_id, new_state, 
							current_gvalue, true_makespan, 
							new_count_collaboration_conflicts, current_count_collision_conflicts, 
							current_count_move_actions, current_tasks_completed+1, 
							nonCollabMap, min_goal_timestep);

						// std::cout << "Tasks Completed = " << new_cost[1] << std::endl;
						if(mFValue.count(new_state)==0 || compareVectors(new_cost,mFValue[new_state]))
						{
							mFValue[new_state]= new_cost;
							mPrev[new_state]=current_state;
							pq.insert(new_state, new_cost);
						}
					}
				}
			}

			std::vector<Vertex> neighbors = getNeighbors(mGraph,current_vertex);
			neighbors.push_back(current_vertex);
			for (auto &successor : neighbors) 
			{
				if(task_id==-1 && successor!=current_vertex) continue;
				bool col = false;
				//check edge conflict
				if(successor != current_vertex){
					Edge uv_edge = boost::edge(current_vertex, successor, mGraph).first;
					CollisionConstraint c1(uv_edge, task_id, current_timestep+1);
					c1.v1 = source(c1.e, mGraph); c1.v2 = target(c1.e, mGraph);
					if(nonCollisionMap.find(c1)!=nonCollisionMap.end()) col=true;
				}
				//check vertex conflict
				CollisionConstraint c2(successor, task_id, current_timestep+1);
				if(nonCollisionMap.find(c2)!=nonCollisionMap.end()) col=true;
				if(!col)
				{       
					SearchState new_state = SearchState(successor, current_timestep+1, current_tasks_completed);
					double new_count_collision_conflicts = 
						current_count_collision_conflicts;  //;
						// countCollisionConflicts(mVertexMap, current_state, new_state, task_id, agent_id);
					double new_count_move_actions = current_count_move_actions;
					if(successor!=current_vertex) new_count_move_actions += mUnitEdgeLength;
					std::vector<double> new_cost = getHeuristics(agent_id, 
						new_state, current_gvalue+mUnitEdgeLength, 
						true_makespan, current_count_collaboration_conflicts, 
						new_count_collision_conflicts, new_count_move_actions, 
						current_tasks_completed, nonCollabMap,  min_goal_timestep);
					
					if(mFValue.count(new_state)==0 || compareVectors(new_cost,mFValue[new_state]))
					{
						mFValue[new_state]= new_cost;
						mPrev[new_state]=current_state;
						pq.insert(new_state, new_cost);
					}
				}
			}
		}

		if(costOut == INF)
		{
			// std::cout << "Path not found lol\n";
			// std::cout << mCSPExpansions << std::endl;
			// printIntervals(nonCollabMap);
			// std::cout << "Agent id = " << agent_id << std::endl;
			// std::cin.get();
			auto stop1 = high_resolution_clock::now();
			mCSPTime += (stop1 - start1);
			return std::vector<SearchState>();
		}
		std::vector<SearchState> finalPath;
		SearchState node = goal_state;
		while(!(node == start_state))
		{
			finalPath.push_back(node);
			node=mPrev[node];
		}
		finalPath.push_back(start_state);
		std::reverse(finalPath.begin(), finalPath.end());
		auto stop1 = high_resolution_clock::now();
		mCSPTime += (stop1 - start1);
		return finalPath;
	}

	double getDistanceToGo(int &agent_id, SearchState &state)
	{
		if(state.tasks_completed == mAgentsToTasksList[agent_id].size()) return 0;
		int task_id = mAgentsToTasksList[agent_id][state.tasks_completed];
		double heuristic= mAllPairsShortestPathMap[state.vertex][mTasksList[task_id].second];
		// std::cout << "c2g before = " << heuristic << std::endl;
		for(int i=state.tasks_completed+1; i<mAgentsToTasksList[agent_id].size(); i++)
		{
			// std::cout << "I exist" << std::endl;
			task_id = mAgentsToTasksList[agent_id][i];
			heuristic += mAllPairsShortestPathMap[mTasksList[task_id].first][mTasksList[task_id].second];
		}
		// std::cout << "c2g after = " << heuristic << std::endl;
		return heuristic;
	}

	std::vector<double> getHeuristics(int &agent_id, SearchState &state,  
		double g_value, double current_makespan,  
		double count_collaboration_conflicts, 
		double count_collision_conflicts, 
		double count_move_actions,
		int tasks_completed,
		std::vector <std::pair <allowedInterval, allowedInterval>> nonCollabMap,
		int minColTime)
	{
		auto start1 = high_resolution_clock::now();
		double distance = getDistanceToGo(agent_id, state);

		int last_task = mAgentsToTasksList[agent_id].back();
		int minTime = (nonCollabMap[last_task].second.minTime);//, minColTime);
		
		double f_value; std::vector<double> heuristics (6);
		
		if(mType=="tie" || mType == "tie-sipp"){
			f_value = std::max(g_value+distance, (double)minTime);
			heuristics[0] = std::max(0.0, f_value - current_makespan);
			heuristics[1] = distance;
			heuristics[2] = -tasks_completed;
			heuristics[3] = f_value;
		}
		else{
			f_value = g_value+distance;
			heuristics[0] = f_value;
			heuristics[1] = distance;
		}

		auto stop1 = high_resolution_clock::now();
		mHeuristicsTime += (stop1 - start1);
		return heuristics;
	}

	bool getVerticesCollisionStatus(Eigen::VectorXd left, Eigen::VectorXd right)
	{
		double distance = (left - right).norm();
		if(distance < 0.00141) // tune threshold!!
			return true;
		return false;
	}

	bool getEdgesCollisionStatus(Eigen::VectorXd left_source, Eigen::VectorXd left_target, Eigen::VectorXd right_source, Eigen::VectorXd right_target)
	{
		if ( (left_source - right_target).norm() < 0.00141 &&  (right_source - left_target).norm() < 0.00141)
			return true;
		return false;
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

	std::vector<Vertex> getNeighbors(Graph &graph, Vertex &v)
	{
		auto start = high_resolution_clock::now();
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

		auto stop = high_resolution_clock::now();
		mGNTime += (stop - start);

		// std::cout<<"neighbors size: "<<neighbors.size()<<std::endl;
		return neighbors;
	}

	void printVertex(Vertex v){
		std::cerr<<" - ("<<
			int( (mGraph[v].state[0]+0.001)/mUnitEdgeLength)<<","<<
			int( (mGraph[v].state[1]+0.001)/mUnitEdgeLength)<<") "<< std::endl;
	}

	void printEdge(Edge &a){
		Vertex s = source(a, mGraph);
 		Vertex t = target(a, mGraph);
 		std::cout << "Source Vertex: ";
 		printVertex(s);
 		std::cout << "Target Vertex: ";
 		printVertex(t);
	}

	void printNode(Element p){
		// return;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++){
			std::cout <<"Path for agent: "<<agent_id << " " << std::endl;
			for(int i=0; i<p.shortestPaths[agent_id].size(); i++){
				int task_id;
				if(p.shortestPaths[agent_id][i].tasks_completed == mAgentsToTasksList[agent_id].size()){
					task_id = -1;
				}
				else{
					task_id = mAgentsToTasksList[agent_id][p.shortestPaths[agent_id][i].tasks_completed];
				}
				std::cout<<" - (" << 
			int( (mGraph[p.shortestPaths[agent_id][i].vertex].state[0]+0.001)/mUnitEdgeLength)<<","<<
			int( (mGraph[p.shortestPaths[agent_id][i].vertex].state[1]+0.001)/mUnitEdgeLength)<<") "<<
			p.shortestPaths[agent_id][i].timestep <<" "<<p.shortestPaths[agent_id][i].tasks_completed<<" "
				<< task_id << "\t";
			}
			std::cout << std::endl;
		}
	}

	void printCollisionConstraint(CollisionConstraint c1){
		std::cout << "============Constraint=============== \n";
		if(c1.constraint_type == 1){
			std::cout << "Vertex constraint\n";
			std::cout << "Position: ";
			printVertex(c1.v);
		}
		else{
			std::cout << "Edge constraint\n";
			std::cout << "Position: \n";
			printEdge(c1.e);
		}
		std::cout << "Tasks Id: " << c1.tasks_completed << std::endl;
		std::cout << "Timestep: "<< c1.timestep << std::endl;
		std::cout << "=================================== \n";
	}

	void printColConflict(std::vector <int> agent_id_1, CollisionConstraint c1, 
		std::vector <int> agent_id_2, CollisionConstraint c2)
	{
		std::cout << "Agents involved in set 1: ";
		for(auto agent: agent_id_1){
			std::cout << agent << " ";
		}
		std::cout << std::endl;

		std::cout << "Agents involved in set 2: ";
		for(auto agent: agent_id_2){
			std::cout << agent << " ";
		}
		std::cout << std::endl;

		std::cout << "======= First constraint: ========== \n";
		if(c1.constraint_type == 1){
			std::cout << "Vertex constraint\n";
			std::cout << "Position: ";
			printVertex(c1.v);
		}
		else{
			std::cout << "Edge constraint\n";
			std::cout << "Position: \n";
			printEdge(c1.e);
		}
		std::cout << "Tasks Id: " << c1.tasks_completed << std::endl;
		std::cout << "Timestep: "<< c1.timestep << std::endl;

		std::cout << "======= Second constraint: ========== \n";
		c1 = c2;
		if(c1.constraint_type == 1){
			std::cout << "Vertex constraint\n";
			std::cout << "Position: ";
			printVertex(c1.v);
		}
		else{
			std::cout << "Edge constraint\n";
			std::cout << "Position: \n";
			printEdge(c1.e);
		}
		std::cout << "Tasks Id: " << c1.tasks_completed << std::endl;
		std::cout << "Timestep: "<< c1.timestep << std::endl;
	}

	void printIntervals(std::vector <std::pair<allowedInterval, allowedInterval>> nonCollabMap){
		for ( std::vector< PCVertex >::reverse_iterator ii=mTopologicalOrder.rbegin(); 
			ii!=mTopologicalOrder.rend(); ++ii)
		{
			int task = *ii; 
			std::cout << "======= Task ID = " << task << std::endl;
			std::cout << "Start Vertex = ";
			printVertex(mTasksList[task].first);
			std::cerr << "Goal Vertex = ";
			printVertex(mTasksList[task].second);
			std::cerr << "Agents involved in set 2: ";
			for(auto agent: mTasksToAgentsList[task]){
				std::cout << agent << " ";
			}
			std::cout << std::endl;
			std::cout << "Start Min Time = " << nonCollabMap[task].first.minTime << 
			" Start Max Time = " << nonCollabMap[task].first.maxTime << std::endl;

			std::cout << "Goal Min Time = " << nonCollabMap[task].second.minTime << 
			" Goal Max Time = " << nonCollabMap[task].second.maxTime << std::endl;
		}
	}
	void printState(SearchState c1){
		printVertex(c1.vertex);
		std::cout << "Tasks Id: " << c1.tasks_completed << std::endl;
		std::cout << "Timestep: "<< c1.timestep << std::endl;
	}
	
	void printCollisionWaypoint(std::pair <SearchState, SearchState> waypoint){

		SearchState curr_state = waypoint.first;
		SearchState next_state = waypoint.second;

		std::cout << "First State: \n";
		printState(curr_state);

		std::cout << "Second State: \n";
		printState(next_state);
	}

	bool compareVectors(std::vector <double> a, std::vector <double> b){
		for(int i=0; i<a.size(); i++){
		// for(int i=0; i<1; i++){
			if((a[i] - b[i])< -0.01)
				return true;
			else if((a[i] - b[i]) > 0.01)
				return false;
		}
		return false;
	}

	void displayPath(std::vector<Eigen::VectorXd> path)
	{
		std::cerr<<"IN display Path!";
		cv::Mat image;
		cv::cvtColor(mImage, image, CV_GRAY2BGR);
		// cv::Mat image = cv::merge(mImage,mImage,mImage);  // going from one to 3 channel
		int numberOfRows = image.rows;
		int numberOfColumns = image.cols;		

		std::vector<cv::Mat4b> number_images(mNumAgents);
		for(int i=0; i<number_images.size(); i++)
		{
			std::stringstream ss;
			ss << "/home/kushal/ros_ws/src/CMAPF/data/viz/new_images/";
			ss << i+1;
			ss << ".png";
			number_images[i] = imread(ss.str(), cv::IMREAD_UNCHANGED);
			double scale = 0.10;
			// if(i!=0)
				// scale = 0.025;
			cv::resize(number_images[i], number_images[i], cv::Size(), scale, scale);
		}

		

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			EdgeIter ei, ei_end;
			for(boost::tie(ei,ei_end) = edges(mGraph); ei!=ei_end;++ei)
			{
				cv::Point source_Point((int)(mGraph[source(*ei,mGraph)].state[0]*numberOfColumns), 
					(int)((1-mGraph[source(*ei,mGraph)].state[1])*numberOfColumns));
				cv::Point target_Point((int)(mGraph[target(*ei,mGraph)].state[0]*numberOfColumns), 
					(int)((1-mGraph[target(*ei,mGraph)].state[1])*numberOfColumns));
				cv::line(image, source_Point, target_Point, cv::Scalar(0, 255, 255), 1);
			}

			VertexIter vi, vi_end;
			for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi)
			{
				double x_point = mGraph[*vi].state[0]*numberOfColumns;
				double y_point = (1 - mGraph[*vi].state[1])*numberOfRows;
				cv::Point centre_Point((int)x_point, (int)y_point);
				cv::circle(image, centre_Point, 2,  cv::Scalar(0, 150, 255), -1);
				// cv::circle(image, centre_Point, 20,  cv::Scalar(0,0,0), 4);
			}
		}   

		

		// Get state count
		int pathSize = path.size();

		for (int i = 0; i < pathSize - 1; ++i)
		{
			Eigen::VectorXd u = path[i];
			Eigen::VectorXd v = path[i+1];

			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			{
				cv::Point uPoint((int)(u[2*agent_id]*numberOfColumns), (int)((1 - u[2*agent_id+1])*numberOfRows));
				cv::Point vPoint((int)(v[2*agent_id]*numberOfColumns), (int)((1 - v[2*agent_id+1])*numberOfRows));  
				// cv::line(image, uPoint, vPoint, cv::Scalar(0, 140, 255), 2);
			}   
		}

		

		// for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		// {
		// 	VertexIter vi, vi_end;
		// 	for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi)
		// 	{
		// 		double x_point = mGraph[*vi].state[0]*numberOfColumns;
		// 		double y_point = (1 - mGraph[*vi].state[1])*numberOfRows;
		// 		cv::Point centre_Point((int)x_point, (int)y_point);
		// 		cv::circle(image, centre_Point, 4,  cv::Scalar(0, 150, 0), -1);
		// 	}
		// } 

		

		std::vector< std::pair<std::pair<int,int>, std::pair<int,int>> >  tasks;
		for(int tid=0; tid<mTasksList.size(); tid++)
		{
			int start_x = int( (mGraph[mTasksList[tid].first].state[0]+0.0001)/mUnitEdgeLength);
			int start_y = int( (mGraph[mTasksList[tid].first].state[1]+0.0001)/mUnitEdgeLength);

			int goal_x = int( (mGraph[mTasksList[tid].second].state[0]+0.0001)/mUnitEdgeLength);
			int goal_y = int( (mGraph[mTasksList[tid].second].state[1]+0.0001)/mUnitEdgeLength);

			tasks.push_back(std::make_pair(std::make_pair(start_x,start_y),std::make_pair(goal_x,goal_y)));
		}

		for(int i=0; i<tasks.size(); i++)
		{
			
			{
				cv::Scalar col= cv::Scalar(200,0,100);
				cv::Point uPoint((int)(tasks[i].first.first*mUnitEdgeLength*numberOfColumns), (int)((1 - tasks[i].first.second*mUnitEdgeLength)*numberOfRows)); 
				std::string text = "S" + std::to_string(i);
				cv::circle(image, uPoint, 25,  col, -1);
				cv::circle(image, uPoint, 25,  cv::Scalar(0,0,0), 2);
				if(i<10)
					cv::putText(image, text, cv::Point(uPoint.x - 15,uPoint.y+7), cv::FONT_HERSHEY_PLAIN, 1.3, cvScalar(0,0,0), 2, 4);
				else	
					cv::putText(image, text, cv::Point(uPoint.x - 20,uPoint.y+7), cv::FONT_HERSHEY_PLAIN, 1.3, cvScalar(0,0,0), 2, 4);
			}

			{
				cv::Scalar col= cv::Scalar(0,200,100);
				cv::Point uPoint((int)(tasks[i].second.first*mUnitEdgeLength*numberOfColumns), (int)((1 - tasks[i].second.second*mUnitEdgeLength)*numberOfRows)); 
				std::string text = "G" + std::to_string(i);
				cv::circle(image, uPoint, 25, col, -1);
				cv::circle(image, uPoint, 25,  cv::Scalar(0,0,0), 2);
				if(i<10)
					cv::putText(image, text, cv::Point(uPoint.x - 15,uPoint.y+7), cv::FONT_HERSHEY_PLAIN, 1.3, cvScalar(0,0,0), 2, 4);
				else
					cv::putText(image, text, cv::Point(uPoint.x - 20,uPoint.y+7), cv::FONT_HERSHEY_PLAIN, 1.3, cvScalar(0,0,0), 2, 4);
			}
		}

		

		for (int i = 0; i < pathSize - 1; ++i)
		{
			Eigen::VectorXd u = path[i];
			Eigen::VectorXd v = path[i+1];

			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			{
				cv::Point uPoint((int)(u[2*agent_id]*numberOfColumns), (int)((1 - u[2*agent_id+1])*numberOfRows));
				cv::Point vPoint((int)(v[2*agent_id]*numberOfColumns), (int)((1 - v[2*agent_id+1])*numberOfRows));  
		
				// if(i==0)
				// {
				// 	std::string text = "S" + std::to_string(agent_id+1);
				// 	cv::circle(image, uPoint, 7,  cv::Scalar(255,255,255), -1);
				// 	cv::circle(image, uPoint, 8,  cv::Scalar(0,0,0), 1);
				// 	cv::putText(image, text, cv::Point(uPoint.x - 6,uPoint.y+3), cv::FONT_HERSHEY_PLAIN, 0.6, cvScalar(0,0,0), 1, 4);
				// }
				// if(i==pathSize-2)
				// {
				// 	std::string text = "G" + std::to_string(agent_id+1);
				// 	cv::circle(image, vPoint, 7,  cv::Scalar(255,255,255), -1);
				// 	cv::circle(image, vPoint, 8,  cv::Scalar(0,0,0), 1);
				// 	cv::putText(image, text, cv::Point(vPoint.x - 6,vPoint.y+3), cv::FONT_HERSHEY_PLAIN, 0.6, cvScalar(0,0,0), 1, 4);
				// }
			}   
		}

		

		bool firstTime = true;

		cv::Mat new_image;
		int num_image = 0;
		for (int i = 0; i < pathSize - 1; ++i)
		{
			
			Eigen::VectorXd u = path[i];
			Eigen::VectorXd v = path[i+1];

			double resolution = 0.005;

			std::vector<Eigen::VectorXd> source_configs(mNumAgents,Eigen::VectorXd());
			std::vector<Eigen::VectorXd> target_configs(mNumAgents,Eigen::VectorXd());
			std::vector<double> edge_lengths(mNumAgents,0);
			std::vector<unsigned int> nStates(mNumAgents,0u);

			unsigned int max_nStates = 0u;

			for(int agent_id=0;agent_id<mNumAgents;agent_id++)
			{
				source_configs[agent_id] = u.segment(2*agent_id,2);
				target_configs[agent_id] = v.segment(2*agent_id,2);
				edge_lengths[agent_id] = (source_configs[agent_id] - target_configs[agent_id]).norm();
				nStates[agent_id] = std::ceil(edge_lengths[agent_id] / resolution)+1;

				if(nStates[agent_id] < 2u)
					nStates[agent_id] = 2u;
				max_nStates = std::max(max_nStates,nStates[agent_id]);
			}

			
			for (unsigned int i = 0; i < max_nStates-1; i++)
			{
				
				new_image = image.clone();
				boost::unordered_map<std::pair<int,int>,std::vector<int>> point_to_agents;
				for(int agent_id = 0; agent_id<mNumAgents; agent_id++)
				{
					Eigen::VectorXd intermediate_config(2);
					if(i < nStates[agent_id] - 1 && !source_configs[agent_id].isApprox(target_configs[agent_id]))
						intermediate_config <<  source_configs[agent_id] + (resolution*i/edge_lengths[agent_id])*(target_configs[agent_id]-source_configs[agent_id]);
					else
						intermediate_config << target_configs[agent_id];

					double x_point = intermediate_config[0]*numberOfColumns;
					double y_point = (1 - intermediate_config[1])*numberOfRows;
					point_to_agents[std::make_pair((int)x_point, (int)y_point)].push_back(agent_id);
					// std::cerr<<x_point<<" "<<y_point<<std::endl;
				}

				for(auto &element: point_to_agents)
				{
					cv::Point _Point(element.first.first, element.first.second);
					int x_point = element.first.first;
					int y_point = element.first.second;

					if(element.second.size() == 1)
					{
						int agent_id = element.second[0];
						// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
						// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
						int x = x_point - number_images[agent_id].cols/2;
						int y = y_point - number_images[agent_id].rows/2;
						double alpha = 1.0; // alpha in [0,1]

						// std::cout<<number_images[agent_id].rows<<std::endl;

						// std::cerr<<"MK1 - "<<i<<"\n";
						// std::cout<<x_point<<" "<<y_point<<"\n"<<x<<" "<<y<<"\n";
						cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));
						// std::cerr<<"OUT - "<<i<<"\n";
						for (int r = 0; r < roi.rows; ++r)
						for (int c = 0; c < roi.cols; ++c)
						{
							
							const cv::Vec4b& vf = number_images[agent_id](r,c);
							
							if (vf[3] > 0) // alpha channel > 0
							{
								// Blending
								cv::Vec3b& vb = roi(r,c);
								
								// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
								// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
								// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
								vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
								vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
								vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
							}
						}
					}
					else if(element.second.size() == 2)
					{
						{
							int agent_id = element.second[0];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point - number_images[agent_id].cols/2 - number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
						{
							int agent_id = element.second[1];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point  - number_images[agent_id].cols/2 + number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
					}
					else if(element.second.size() == 3)
					{
						{
							int agent_id = element.second[0];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point - number_images[agent_id].cols/2 - number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2 - number_images[agent_id].rows/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
						{
							int agent_id = element.second[1];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point  - number_images[agent_id].cols/2 + number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2 - number_images[agent_id].rows/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
						{
							int agent_id = element.second[2];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point  - number_images[agent_id].cols/2;
							int y = y_point - number_images[agent_id].rows/2 + number_images[agent_id].rows/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}						
					}
					else if(element.second.size() == 4)
					{
						{
							int agent_id = element.second[0];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point - number_images[agent_id].cols/2 - number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2 - number_images[agent_id].cols/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
						{
							int agent_id = element.second[1];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point - number_images[agent_id].cols/2 - number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2 + number_images[agent_id].cols/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
						{
							int agent_id = element.second[2];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point - number_images[agent_id].cols/2 + number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2 - number_images[agent_id].cols/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
						{
							int agent_id = element.second[3];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point - number_images[agent_id].cols/2 + number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2 + number_images[agent_id].cols/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}			
					}
				}

				cv::namedWindow("Agents",cv::WINDOW_NORMAL);
				cv::imshow("Agents", new_image);
				cv::waitKey(10);
				for (int num = 0; num<1; num++){
					std::string path = mImagePath+std::to_string(num_image)+".jpg";
					cv::imwrite(path, new_image);
					num_image += 1;
				}
				
				if(firstTime)
				{
					for (int num = 0; num<10; num++){
						std::string path = mImagePath+std::to_string(num_image)+".jpg";
						cv::imwrite(path, new_image);
						num_image += 1;
					}
					sleep(5);
					firstTime = false;
				}
			}
			{
				new_image = image.clone();
				boost::unordered_map<std::pair<int,int>,std::vector<int>> point_to_agents;
				for(int agent_id = 0; agent_id<mNumAgents; agent_id++)
				{
					double x_point = v[agent_id*2]*numberOfColumns;
					double y_point = (1 - v[agent_id*2+1])*numberOfRows;
					point_to_agents[std::make_pair((int)x_point, (int)y_point)].push_back(agent_id);
				}

				for(auto &element: point_to_agents)
				{
					cv::Point _Point(element.first.first, element.first.second);
					int x_point = element.first.first;
					int y_point = element.first.second;

					if(element.second.size() == 1)
					{
						int agent_id = element.second[0];
						// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
						// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
						int x = x_point - number_images[agent_id].cols/2;
						int y = y_point - number_images[agent_id].rows/2;
						double alpha = 1.0; // alpha in [0,1]

						cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

						for (int r = 0; r < roi.rows; ++r)
						for (int c = 0; c < roi.cols; ++c)
						{
							
							const cv::Vec4b& vf = number_images[agent_id](r,c);
							
							if (vf[3] > 0) // alpha channel > 0
							{
								// Blending
								cv::Vec3b& vb = roi(r,c);
								
								// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
								// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
								// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
								vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
								vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
								vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
							}
						}
					}
					else if(element.second.size() == 2)
					{
						{
							int agent_id = element.second[0];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point - number_images[agent_id].cols/2 - number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
						{
							int agent_id = element.second[1];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point  - number_images[agent_id].cols/2 + number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
					}
					else if(element.second.size() == 3)
					{
						{
							int agent_id = element.second[0];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point - number_images[agent_id].cols/2 - number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2 - number_images[agent_id].rows/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
						{
							int agent_id = element.second[1];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point  - number_images[agent_id].cols/2 + number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2 - number_images[agent_id].rows/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
						{
							int agent_id = element.second[2];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point  - number_images[agent_id].cols/2;
							int y = y_point - number_images[agent_id].rows/2 + number_images[agent_id].rows/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}						
					}
					else if(element.second.size() == 4)
					{
						{
							int agent_id = element.second[0];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point - number_images[agent_id].cols/2 - number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2 - number_images[agent_id].cols/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
						{
							int agent_id = element.second[1];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point - number_images[agent_id].cols/2 - number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2 + number_images[agent_id].cols/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
						{
							int agent_id = element.second[2];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point - number_images[agent_id].cols/2 + number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2 - number_images[agent_id].cols/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}
						{
							int agent_id = element.second[3];
							// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
							// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
							int x = x_point - number_images[agent_id].cols/2 + number_images[agent_id].cols/4;
							int y = y_point - number_images[agent_id].rows/2 + number_images[agent_id].cols/4;
							double alpha = 1.0; // alpha in [0,1]

							cv::Mat3b roi = new_image(cv::Rect(x, y, number_images[agent_id].cols, number_images[agent_id].rows));

							for (int r = 0; r < roi.rows; ++r)
							for (int c = 0; c < roi.cols; ++c)
							{
								
								const cv::Vec4b& vf = number_images[agent_id](r,c);
								
								if (vf[3] > 0) // alpha channel > 0
								{
									// Blending
									cv::Vec3b& vb = roi(r,c);
									
									// std::cout<<alpha * vf[0] + (1 - alpha) * vb[0]<<std::endl;
									// std::cout<<alpha * vf[1] + (1 - alpha) * vb[1]<<std::endl;
									// std::cout<<alpha * vf[2] + (1 - alpha) * vb[2]<<std::endl;
									vb[0] = alpha * vf[0] + (1 - alpha) * vb[0];
									vb[1] = alpha * vf[1] + (1 - alpha) * vb[1];
									vb[2] = alpha * vf[2] + (1 - alpha) * vb[2];
								}
							}
						}			
					}

				}
				
				cv::namedWindow("Agents",cv::WINDOW_NORMAL);
				cv::imshow("Agents", new_image);
				cv::waitKey(10);
				for (int num = 0; num<1; num++){
					std::string path = mImagePath+std::to_string(num_image)+".jpg";
					cv::imwrite(path, new_image);
					num_image += 1;
				}
			}
			
		}
		cv::namedWindow("Graph Visualization",cv::WINDOW_NORMAL);
		cv::imshow("Graph Visualization", image);
		cv::waitKey(0);
	}
};


} // namespace PCCBS

#endif