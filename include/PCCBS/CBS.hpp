

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

struct collaboration_constraint_hash
{
	std::size_t operator()(const CollaborationConstraint& k) const
	{
		return size_t(k.is_pickup)+2*size_t(k.task_id)+size_t(k.v)*2*16+size_t(k.timestep)*2*16*1024;
	}
};

struct collision_conflict
{
	CollisionConstraint c1;
	CollisionConstraint c2;

	collision_conflict(CollisionConstraint _c1, CollisionConstraint _c2): 
		c1(_c1), c2(_c2){}
	bool operator==(const collision_conflict &other) const
	{ 
		return ((c1 == other.c1) && (c2 == other.c2) );
	}
};

size_t collision_hash_(CollisionConstraint k1){
	using namespace boost;
	using boost::hash_combine;
	size_t seed = 42;
	if(k1.constraint_type==2){
		hash_combine(seed, k1.v1);
		hash_combine(seed, k1.v2);
	}
	else{
		hash_combine(seed, k1.v);
		hash_combine(seed, k1.v);
	}
	hash_combine(seed, k1.tasks_completed);
	hash_combine(seed, k1.in_delivery);
	hash_combine(seed, k1.timestep);
	return seed;
}
struct collision_conflict_hash
{
	std::size_t operator()(const collision_conflict& k) const
	{
		using namespace boost;
		using boost::hash_combine;
		size_t seed = 42;
		hash_combine(seed, collision_hash_(k.c1));
		hash_combine(seed, collision_hash_(k.c2));
		return seed;
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

	std::vector<<std::pair<int, int>> mTasksList;
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
	boost::unordered_map<std::pair<Vertex,Vertex>,double> mAllPairsShortestPathMap;
	double mUnitEdgeLength = 0.1;

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
			if(start_config.isApprox(mGraph[*ind_vi].state))
				s = *ind_vi;
			if(goal_config.isApprox(mGraph[*ind_vi].state))
				g = *ind_vi;
		}
		return std::make_pair(s,g);
	}

	CBS(PrecedenceConstraintGraph _pcg, cv::Mat img, int numAgents 
		,std::vector<std::string> roadmapFileNames, std::string imagePath)
		: mImage(img)
		, mNumAgents(numAgents)
		, mRoadmapFileNames(roadmapFileNames)
		, mImagePath(imagePath)
		, mPCGraph(_pcg)
	{
		//set up transpose graph and topological sort
		transpose_graph(mPCGraph, mPCGraph_T);
		mProp = get(meta_data_t(), mPCGraph);
		mProp_T = get(meta_data_t(), mPCGraph_T);
		topological_sort(mPCGraph, std::back_inserter(mTopologicalOrder));
		int mNumTasks = boost::num_vertices(_pcg);

		//set up graph preprocessing
		create_vertices(mGraph,get(&VProp::state,mGraph),mRoadmapFileNames[0],2,get(&EProp::prior,mGraph));
		create_edges(mGraph,get(&EProp::length,mGraph));
		preprocess_graph(mGraph);

		//initialise the search problem
		mPredecessors = std::vector <std::vector<int>> (mNumTasks);
		mSuccessors = std::vector <std::vector<int>> (mNumTasks);
		mTasksList = std::vector <std::pair <int,int> > (mNumTasks);
		mTasksToAgentsList = std::vector <std::vector <int>> (mNumTasks);

		for ( std::vector< PCVertex >::reverse_iterator ii=mTopologicalOrder.rbegin(); 
			ii!=mTopologicalOrder.rend(); ++ii)
		{
			meta_data vertex = get(get(meta_data_t(), _pcg), *ii);
			int task_id = *ii; 
			std::pair <Vertex, Vertex> taskVertices = getVertexFromGraph(vertex);
			mTasksList[task_id] = taskVertices;
			mTasksToAgentsList[task_id] = vertex.agent_list;
			for(auto agent_id: vertex.agent_list){
				mAgentsToTasksList.push_back(task_id);
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
		std::cout << "yo\n";
		std::cin.get();

		PRINT<<"OUT";
		auto t1 = std::chrono::high_resolution_clock::now();
	    auto t2 = std::chrono::high_resolution_clock::now();
		mCSPTime = t2-t1;mGNTime = t2-t1;mQOTime = t2-t1;mCCTime = t2-t1;
		mPlanningTime = t2-t1;mPreprocessTime = t2-t1;mCSPTime = t2-t1;mHeuristicsTime = t2-t1;
		mCollabCTime = t2-t1;mCollisionCTime = t2-t1;mGNTime = t2-t1;mQOTime = t2-t1;
		mCCTime = t2-t1;mCHTime = t2-t1;mPlanningTime = t2-t1;mPreprocessTime = t2-t1;
		mMapOperationsTime = t2-t1;mGetCollisionTime = t2-t1;mGetCollaborationTime = t2-t1;
		mCollisionIterations = 0;mCollaborationIterations = 0;
		mCSPExpansions = 0;mCSPIterations = 0;
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
		if(state.tasks_completed == goal_state.tasks_completed && state.in_delivery == true 
			&& goal_state.in_delivery == false)
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


	int countCollaborationConflicts(SearchState &state, SearchState &new_state, 
		std::vector<std::vector<SearchState> > &paths, std::vector<int> &consider_agents)
	{
		
	}

	bool getCollaborationConstraints(Element &p, std::vector<int> &collaborating_agent_ids, 
		CollaborationConstraint &constraint_c)
	{
		
	}

	int countCollisionConflicts(
		boost::unordered_map <std::pair <int, int>, std::vector<SearchState>> mVertexMap,
	 	SearchState curr, int task_id, int agent_id){
		std::pair <int, int> k = std::make_pair(curr.vertex, curr.timestep);
		if(mVertexMap.find(k)==mVertexMap.end())
			return 0;

		std::vector <SearchState> possible = mVertexMap[k];

		if(mSpecialPosition[agent_id].count(curr.vertex)!=0) return 0;
		for(auto state: possible){
			if(state.tasks_completed!=task_id) return 1;
		}
		return 0;
	}

	bool getCollisionConstraints(Element &p, 
		std::vector<int> &collaborating_agent_ids_1, std::vector <int> &collaborating_agent_ids_2,
		CollisionConstraint &constraint_1, CollisionConstraint &constraint_2,
		int makespan)
	{
		std::vector <std::vector <int>> timestepMap (makespan+1);

		for(int i=0; i<mNumAgents; i++){
			for(int j=0; j<p.shortestPaths[i].size(); j++){
				SearchState state = p.shortestPaths[i][j];
				if(state.tasks_completed == mAgentsToTasksList.size()){
					SearchState key = SearchState(state.v, i, -1);
					timestepMap[state.timestep].push_back(key);
					if(j==p.shortestPaths.size()-1){
						for(int time=state.timestep+1; time <= makespan; time++){
							SearchState key = SearchState(state.v, i, -1);
							timestepMap[time].push_back(key);
						}
						break;
					}
					continue;
				}
				int task_id = mAgentsToTasksList[state.tasks_completed];
				SearchState key = SearchState(state.v, i, task_id);
				timestepMap[state.timestep].push_back(key);
			}
		}

		//check vertex conflict
		for(int time=0; time<=makespan; time++){
			boost::unordered_map <Vertex, std::vector<SearchState>> vertexMap;
			int task1 = -1, task2 = -1;
			for(state: timestep[time]){
				vertexMap[state.vertex].push_back(state);
			}

			for(auto it: vertexMap){
				Vertex v = it.first;
				std::vector <SearchState> colStates = it.second;
				std::unordered_map <int, int> agentMap;
				for(auto state: colStates){
					agentMap[state.timestep] = std::max(state.tasks_completed, agentMap[state.timestep]);
				}

				for(auto a1: agentMap){
					for(auto a2: agentMap){
						if(a1.first==a2.first) continue;
						if(a1.second==a2.second) continue;
						int tid_1 = a1.second, tid_2 = a2.second;
						for(auto a3: agentMap){
							if(a3.second == tid_1){
								collaborating_agent_ids_1.push_back(a3.first);
							}
							if(a3.second == tid_2){
								collaborating_agent_ids_2.push_back(a3.first);
							}
						}
						constraint_1 = CollisionConstraint(v, tid_1, time);
						constraint_2 = CollisionConstraint(v, tid_2, time);
						return true;
					}
				}
			}
		}

		//check edge conflict
		for(int time=0; time<=makespan-1; time++){
			boost::unordered_map <Vertex, std::vector<SearchState>> vertexMap;
			int task1 = -1, task2 = -1;
			for(state: timestep[time]){
				vertexMap[state.vertex].push_back(state);
			}

			for(auto it: vertexMap){
				Vertex v = it.first;
				std::vector <SearchState> colStates = it.second;
				std::unordered_map <int, int> agentMap;
				for(auto state: colStates){
					agentMap[state.timestep] = std::max(state.tasks_completed, agentMap[state.timestep]);
				}

				for(auto a1: agentMap){
					for(auto a2: agentMap){
						if(a1.first==a2.first) continue;
						if(a1.second==a2.second) continue;
						int tid_1 = a1.second, tid_2 = a2.second;
						for(auto a3: agentMap){
							if(a3.second == tid_1){
								collaborating_agent_ids_1.push_back(a3.first);
							}
							if(a3.second == tid_2){
								collaborating_agent_ids_2.push_back(a3.first);
							}
						}
						constraint_1 = CollisionConstraint(v, tid_1, time);
						constraint_2 = CollisionConstraint(v, tid_2, time);
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
		for(int i=0; i<collaborating_agent_ids.size(); i++)
		{
			// std::cout << constraint.v1 << " " << constraint.v2 << "\n";
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
		std::vector <int> collaborating_agent_ids, 
		boost::unordered_map <SearchState, allowedInterval, state_hash> nonCollabMap, 
		bool &possible)
	{
		int current_makespan = getMakespan(p.shortestPaths);
		std::vector<int> consider_agents;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			if(std::find(collaborating_agent_ids.begin(),collaborating_agent_ids.end(), agent_id) 
				== collaborating_agent_ids.end())
				consider_agents.push_back(agent_id);

		std::vector< std::vector<SearchState> > shortestPaths_c = p.shortestPaths;

		bool all_paths_exist = true; double cost_c;

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

	int getIntegerTime(double time){
		return (int)((time+0.0001)/mUnitEdgeLength);
	}
	void updateSchedule(boost::unordered_map <SearchState, allowedInterval, state_hash> &nonCollabMap,
		bool &possible){

		for (container::reverse_iterator ii=mTopologicalOrder.rbegin(); ii!=mTopologicalOrder.rend(); ++ii)
		{
			int task_id = *ii;
			// std::cout << "TASK ID = " << task_id << std::endl;
			std::pair <SearchState, SearchState> curTaskStates = mTaskToSearchStates[task_id];
			allowedInterval curStartInterval = nonCollabMap[curTaskStates.first];
			allowedInterval curGoalInterval = nonCollabMap[curTaskStates.second];
			vector <int> predecessors = mPredecessors[task_id];
			for(auto pred: predecessors){
				// std::cout << "PRED = " << pred << std::endl;
				std::pair <SearchState, SearchState> predTaskStates = mTaskToSearchStates[pred];
				allowedInterval predInterval = nonCollabMap[predTaskStates.second];
				curStartInterval.minTime = std::max(
					curStartInterval.minTime, 
					predInterval.minTime+
					getIntegerTime(
							mAllPairsShortestPathMap[
							std::make_pair(predTaskStates.second.vertex, curTaskStates.first.vertex)
							]
						)
					);
			}
			// if(task_id==0){
			// 	std::cout << curStartInterval.minTime << " " << curGoalInterval.minTime << std::endl;
			// }
			curGoalInterval.minTime = std::max(
				curGoalInterval.minTime, 
				curStartInterval.minTime+
				getIntegerTime(
					mAllPairsShortestPathMap[
						std::make_pair(curTaskStates.first.vertex, curTaskStates.second.vertex)
						]
					)
				);	
			// if(task_id==0){
			// 	std::cout << getIntegerTime(mAllPairsShortestPathMap[
			// 			std::make_pair(curTaskStates.first.vertex, curTaskStates.second.vertex)
			// 			]) << std::endl;
			// 	std::cout << curStartInterval.minTime << " " << curGoalInterval.minTime << std::endl;
			// }
			nonCollabMap[curTaskStates.first] = curStartInterval;
			nonCollabMap[curTaskStates.second] = curGoalInterval;
			// std::cin.get();
		}

		for (container::iterator ii=mTopologicalOrder.begin(); ii!=mTopologicalOrder.end(); ++ii)
		{
			int task_id = *ii;
			std::pair <SearchState, SearchState> curTaskStates = mTaskToSearchStates[task_id];
			allowedInterval curStartInterval = nonCollabMap[curTaskStates.first];
			allowedInterval curGoalInterval = nonCollabMap[curTaskStates.second];

			vector <int> successors = mSuccessors[task_id];
			for(auto succ: successors){
				std::pair <SearchState, SearchState> succTaskStates = mTaskToSearchStates[succ];
				allowedInterval succInterval = nonCollabMap[succTaskStates.first];

				curGoalInterval.maxTime = std::min(
					curGoalInterval.maxTime, 
					succInterval.maxTime-
					getIntegerTime(
						mAllPairsShortestPathMap[
							std::make_pair(curTaskStates.second.vertex, succTaskStates.first.vertex)
							]
						)
					);
			}

			curStartInterval.maxTime = std::min(
				curStartInterval.maxTime, 
				curGoalInterval.maxTime-
				getIntegerTime(
					mAllPairsShortestPathMap[
						std::make_pair(curTaskStates.first.vertex, curTaskStates.second.vertex)
						]
					)
				);
			
			nonCollabMap[curTaskStates.first] = curStartInterval;
			nonCollabMap[curTaskStates.second] = curGoalInterval;
		}

		//check validity
		for (container::reverse_iterator ii=mTopologicalOrder.rbegin(); ii!=mTopologicalOrder.rend(); ++ii)
		{
			int task_id = *ii;
			std::pair <SearchState, SearchState> curTaskStates = mTaskToSearchStates[task_id];
			allowedInterval curStartInterval = nonCollabMap[curTaskStates.first];
			allowedInterval curGoalInterval = nonCollabMap[curTaskStates.second];
			if(!isValid(curStartInterval)){
				possible=false;
				return;
			}

			if(!isValid(curGoalInterval)){
				possible=false;
				return;
			}
		}
		possible = true;
	}

	std::pair <Element, Element> expandCollaborationConflict(Element p,
		std::vector <int> collaborating_agent_ids, CollaborationConstraint constraint,
		bool &possible1, bool &possible2)
	{
		SearchState key_state = SearchState(constraint.v,-1,constraint.task_id,constraint.is_pickup);
		int max_time = constraint.timestep-1, min_time = constraint.timestep;

		allowedInterval curInterval = allowedInterval();
		if(p.nonCollabMap.find(key_state)!=p.nonCollabMap.end()){
			curInterval = p.nonCollabMap[key_state];
		}

		Element maxNode = p, minNode = p;
		if(max_time >= curInterval.minTime && max_time <= curInterval.maxTime){
			allowedInterval newInterval = allowedInterval(curInterval.minTime, max_time);
			boost::unordered_map <SearchState, allowedInterval, state_hash> increasedMap = p.nonCollabMap;
			increasedMap[key_state] = newInterval;
			updateSchedule(increasedMap, possible1);
			// printIntervals(increasedMap);
			// std::cout << "IS IT POSSIBLE? " << possible1 << std::endl;
			if(possible1)
				maxNode = expandCollaborationConstraint(p, collaborating_agent_ids, increasedMap, possible1);
			// printNode(maxNode);
			// std::cout << possible1 << std::endl;
		}
		else{
			possible1 = false;
		}
		
		if(min_time <= curInterval.maxTime && min_time >= curInterval.minTime){
			allowedInterval newInterval = allowedInterval(min_time, curInterval.maxTime);
			boost::unordered_map <SearchState, allowedInterval, state_hash> increasedMap = p.nonCollabMap;
			increasedMap[key_state] = newInterval;
			updateSchedule(increasedMap, possible2);
			// printIntervals(increasedMap);
			// std::cout << possible2 << std::endl;
			if(possible2)
				minNode = expandCollaborationConstraint(p, collaborating_agent_ids, increasedMap, possible2);
		}
		else{
			possible2 = false;
		}
		// std::cout << possible1 << " " << possible2 << std::endl;
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
		CBSPriorityQueue PQ(mNumAgents);

		std::vector <allowedInterval> nonCollabMap;
		std::vector <boost::unordered_map <CollisionConstraint, int, 
			collision_hash>> nonCollisionMap(mNumAgents);

		for(int i=0; i<mNumTasks; i++)
		{
			allowedInterval Interval = allowedInterval();
			nonCollabMap.push_back(Interval);
		}
		// std::cin.get();
		bool possible;
		updateSchedule(nonCollabMap, possible);
		if(!debug_disabled){
			printIntervals(nonCollabMap);
			std::cin.get();
		}
		if(!possible){
			std::cout << "Problem is wrong.\n";
			std::cin.get();
		}

		std::vector< std::vector<SearchState> > start_shortestPaths = 
			computeDecoupledPaths(nonCollabMap, nonCollisionMap);

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			if(start_shortestPaths.at(agent_id).size()==0)
			{
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

			int task_id;
			CollaborationConstraint constraint;

			if(getCollaborationConstraints(p, task_id))
			{
				if(!debug_disabled){
					std::cout << "-----Colab Conflict Found-----------" << std::endl;
					std::cout << "----- Task ID = " << task_id << std::endl;
					std::cin.get();
				}

				bool possible1 = false, possible2 = false;
				std::pair <Element, Element> colabChildren = 
					expandCollaborationConflict(p, task_id, possible1, possible2);
				if(possible1) PQ.insert(colabChildren.first); 
				if(possible2) PQ.insert(colabChildren.second);

				if(!debug_disabled){
					if(possible1){
						std::cout << "---------------EXPANDED NODE 1-------------------"<< std::endl;
						printNode(colabChildren.first);
						std::cout << "---------------INTERVALS-------------------"<< std::endl;
						printIntervals(colabChildren.first.nonCollabMap);
					}
					if(possible2){
						std::cout << "---------------EXPANDED NODE 2-------------------"<< std::endl;
						printNode(colabChildren.second);
						std::cout << "---------------INTERVALS-------------------"<< std::endl;
						printIntervals(colabChildren.second.nonCollabMap);
					}
					std::cin.get();
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
			std::cout<<"Press [ENTER] to display path: ";
			std::cin.get();
			displayPath(path_configs);
			printStats();
			return collision_free_path;
		}
		std::cout << "CBS FAILED WTF IS WRONG\n";
		return std::vector<std::vector<Eigen::VectorXd>>(mNumAgents,std::vector<Eigen::VectorXd>());
	}

	std::vector< std::vector<SearchState> > computeDecoupledPaths(
		boost::unordered_map <SearchState, allowedInterval, state_hash> nonCollabMap,
		std::vector<boost::unordered_map <CollisionConstraint, int, collision_hash>> &nonCollisionMap)
	{
		std::vector<std::vector<SearchState> > shortestPaths (mNumAgents);
		std::vector<std::vector<SearchState> > dummyPaths;
		std::vector<int> consider_agents;
		int current_makespan = 0;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			double ind_cost;
			std::vector <SearchState> path = 
					computeShortestPath(agent_id,consider_agents,
					ind_cost, 0, 
					nonCollabMap, nonCollisionMap[agent_id],
					dummyPaths);
			shortestPaths[agent_id] = path;
			// consider_agents.push_back(agent_id);
			// std::cerr << agent_id << std::endl;
			// if(path.size()>0)
			// 	current_makespan = std::max(current_makespan,path[path.size()-1].timestep);
		}
		return shortestPaths;
	}

	std::vector<SearchState> computeShortestPath(int &agent_id, std::vector<int> &consider_agents, 
		double& costOut, int current_makespan, 
		boost::unordered_map <SearchState, allowedInterval, state_hash> nonCollabMap,
		boost::unordered_map <CollisionConstraint, int, collision_hash> &nonCollisionMap,
		std::vector<std::vector<SearchState> > &shortestPaths)
	{
		SearchState start_state = SearchState(mStartVertex[agent_id], 0, 0, 0);
		auto start3 = high_resolution_clock::now();

		// std::cout <<"Check Problem\n";
		boost::unordered_map <std::pair <int, int>, std::vector<SearchState>> mVertexMap;
		for(int i=0; i<shortestPaths.size(); i++){
			if(i==agent_id) continue;
			for(int j=0;j<shortestPaths[i].size(); j++){
				SearchState curr = shortestPaths[i][j];
				int task_id = mTasksList[agent_id][curr.tasks_completed].first;
				SearchState key = SearchState(curr.vertex, curr.timestep, task_id, curr.in_delivery);
				std::pair <int, int> k = std::make_pair(curr.vertex, curr.timestep);
				if(mVertexMap.find(k)==mVertexMap.end()){
					mVertexMap[k] = std::vector <SearchState> {key};
				}
				else{
					mVertexMap[k].push_back(key);
				}
			}
		}

		auto stop3 = high_resolution_clock::now();
		mCHTime += (stop3 - start3);

		mCSPIterations++;
		auto start1 = high_resolution_clock::now();

		int min_goal_timestep = 0;
		bool lastSegment = true;
		for( auto it: nonCollisionMap)
		{
			min_goal_timestep = std::max(min_goal_timestep, (it.first).timestep);
		}

		timePriorityQueue pq;
		boost::unordered_map<SearchState, std::vector<double>, state_hash> mFValue;
		boost::unordered_map<SearchState , SearchState, state_hash > mPrev;
		int start_timestep=0;
		SearchState goal_state = SearchState();
		double true_makespan = current_makespan*mUnitEdgeLength;
		double g_v = start_timestep*mUnitEdgeLength;
		mFValue[start_state] = getHeuristics(agent_id, start_state, goal_state,
		 g_v, true_makespan, 
		 0, 0, 0);
		pq.insert(start_state,mFValue[start_state]);
		int numSearches = 0;
		int goal_timestep = -1;

		costOut = INF;

		while(pq.PQsize()!=0)
		{
			mCSPExpansions++;
			numSearches++;
			auto yoma1 = high_resolution_clock::now();
			SearchState current_state = pq.pop();	
			Vertex current_vertex = current_state.vertex;
			int current_timestep = current_state.timestep;
			int current_tasks_completed = current_state.tasks_completed;
			bool current_in_delivery = current_state.in_delivery;
			std::vector<double> current_fvalue = mFValue[current_state];
			double current_gvalue = current_timestep*mUnitEdgeLength;
			double current_count_collaboration_conflicts = current_fvalue[2];
			double current_count_collision_conflicts = current_fvalue[1];
			double current_count_move_actions = current_fvalue[3] - current_fvalue[4];
			auto yoma2 = high_resolution_clock::now();
			mMapOperationsTime += yoma2-yoma1;
			auto stop = high_resolution_clock::now();
			std::chrono::duration<double, std::micro> timespent = stop - mSolveStartTime;
			// std::cout << current_timestep << std::endl;

			if(debug_disabled){
				if (timespent.count() > 300000000)
				{
					// std::cout << "CSP TIMEOUT WTF IS WRONG\n";
					auto solve_stop = high_resolution_clock::now();
					mPlanningTime = (solve_stop - mSolveStartTime);
					costOut = INF;
					auto stop1 = high_resolution_clock::now();
					mCSPTime += (stop1 - start1);
					return std::vector<SearchState>();
				}
			}

			if(current_tasks_completed > mTasksList[agent_id].size())
			{
				std::cout<<"Bruh.";
				std::cin.get();
			}
			if(current_tasks_completed == mTasksList[agent_id].size() && current_timestep>= min_goal_timestep)
			{
				// std::cout<<"Timestep goal was found: "<<current_timestep<<std::endl;
				costOut = current_gvalue;
				goal_state = current_state;
				break;
			}
			if(current_tasks_completed >= mTasksList[agent_id].size())
				continue;

			std::pair <SearchState, SearchState> curTaskStates = 
				mTaskToSearchStates[mTasksList[agent_id][current_tasks_completed].first];
			
			SearchState key_state;
			if(current_in_delivery){
				key_state = curTaskStates.second;
			}	
			else{
				key_state = curTaskStates.first;
			}

			allowedInterval Interval = nonCollabMap[key_state];
			if(current_timestep>Interval.maxTime) {
				// std::cout << Interval.maxTime << " " << current_timestep << std::endl;
				continue;
			}

			// if(current_makespan && current_timestep > current_makespan+10){
			// 	// std::cout << "yo\n";
			// 	continue;
			// }

			if(mSpecialPosition[agent_id].count(current_vertex)!= 0)
			{
				if(!current_in_delivery && 
				mTasksList[agent_id][current_tasks_completed].second.first == current_vertex) //pickup point
				{
					bool allowed = true;
					auto yoma11 = high_resolution_clock::now();
					SearchState key_state = SearchState(current_vertex, -1, 
						mTasksList[agent_id][current_tasks_completed].first, current_in_delivery);	
					if(nonCollabMap.find(key_state)!=nonCollabMap.end()) {
						allowedInterval Interval = nonCollabMap[key_state];
						if(current_timestep<Interval.minTime || 
							current_timestep>Interval.maxTime){
							allowed = false;
						}
					}
					auto yoma12 = high_resolution_clock::now();
					mMapOperationsTime += yoma12-yoma11;
					if(allowed)
					{
						SearchState new_state = SearchState(current_vertex, current_timestep, 
							current_tasks_completed, true);
						if(lastSegment || !isOverFlowState(new_state,goal_state))
						{
							double new_count_collaboration_conflicts = 
								current_count_collaboration_conflicts +
								countCollaborationConflicts(agent_id,current_state,new_state,shortestPaths,consider_agents);
							std::vector<double> new_cost = getHeuristics(agent_id, new_state, 
								goal_state, current_gvalue, true_makespan, 
								new_count_collaboration_conflicts, current_count_collision_conflicts, 
								current_count_move_actions);
							if(mFValue.count(new_state)==0 || compareVectors(new_cost,mFValue[new_state]))
							{
								auto yoma11 = high_resolution_clock::now();
								mFValue[new_state]= new_cost;
								mPrev[new_state]=current_state;
								pq.insert(new_state, new_cost);
								auto yoma12 = high_resolution_clock::now();
								mMapOperationsTime += yoma12-yoma11;
							}
						}
					}
				}
				if(current_in_delivery && 
				mTasksList[agent_id][current_tasks_completed].second.second == current_vertex) //delivery point
				{
					bool allowed = true;
					auto yoma11 = high_resolution_clock::now();
					SearchState key_state = SearchState(current_vertex, -1, 
						mTasksList[agent_id][current_tasks_completed].first, current_in_delivery);	
					if(nonCollabMap.find(key_state)!=nonCollabMap.end()) {
						allowedInterval Interval = nonCollabMap[key_state];
						if(current_timestep<Interval.minTime || 
							current_timestep>Interval.maxTime){
							allowed = false;
						}
					}
					auto yoma12 = high_resolution_clock::now();
					mMapOperationsTime += yoma12-yoma11;
					if(allowed)
					{
						SearchState new_state= SearchState(current_vertex, current_timestep, 
							current_tasks_completed+1, false);
						if(lastSegment || !isOverFlowState(new_state,goal_state))
						{
							double new_count_collaboration_conflicts = 
								current_count_collaboration_conflicts +
								countCollaborationConflicts(agent_id,current_state,new_state,shortestPaths,consider_agents);
							std::vector<double> new_cost = getHeuristics(agent_id, new_state, 
								goal_state, current_gvalue, true_makespan, 
								new_count_collaboration_conflicts, current_count_collision_conflicts, 
								current_count_move_actions);
							if(mFValue.count(new_state)==0 || compareVectors(new_cost,mFValue[new_state]))
							{
								auto yoma1 = high_resolution_clock::now();
								mFValue[new_state]= new_cost;
								mPrev[new_state]=current_state;
								pq.insert(new_state, new_cost);
								auto yoma2 = high_resolution_clock::now();
								mMapOperationsTime += yoma2-yoma1;
							}
						}
					}
				}
			}

			

			{
				auto start2 = high_resolution_clock::now();
				bool col = false;

				int task_id = mTasksList[agent_id][current_tasks_completed].first;
				CollisionConstraint c2(current_vertex, task_id, current_in_delivery, current_timestep+1);
				if(nonCollisionMap.find(c2)!=nonCollisionMap.end()) col=true;
				auto stop2 = high_resolution_clock::now();
				mCCTime += (stop2 - start2);

				if(!col)
				{
					SearchState new_state = SearchState(current_vertex, current_timestep+1, 
						current_tasks_completed, current_in_delivery);
					if(lastSegment || !isOverFlowState(new_state,goal_state))
					{
						double new_count_collision_conflicts = 
							current_count_collision_conflicts +
							countCollisionConflicts(mVertexMap, new_state, task_id, agent_id);
							// countCollisionConflicts(agent_id,current_state,new_state,shortestPaths,consider_agents,mVertexCollisionPathsMap,mEdgeCollisionPathsMap);
						std::vector<double> new_cost = getHeuristics(agent_id, new_state, 
							goal_state, current_gvalue+mUnitEdgeLength, true_makespan, 
							current_count_collaboration_conflicts, 
							new_count_collision_conflicts, current_count_move_actions);
							
						if(mFValue.count(new_state)==0 || compareVectors(new_cost,mFValue[new_state]))
						{
							auto yoma1 = high_resolution_clock::now();
							mFValue[new_state]= new_cost;
							mPrev[new_state]=current_state;
							pq.insert(new_state, new_cost);
							auto yoma2 = high_resolution_clock::now();
							mMapOperationsTime += yoma2-yoma1;	
						}				
					}
				}	
			}

			std::vector<Vertex> neighbors = getNeighbors(mGraph,current_vertex);

			for (auto &successor : neighbors) 
			{
				Edge uv_edge = boost::edge(current_vertex, successor, mGraph).first;

				auto start2 = high_resolution_clock::now();
				bool col = false;

				int task_id = mTasksList[agent_id][current_tasks_completed].first;
				CollisionConstraint c1(uv_edge, task_id, current_in_delivery, current_timestep+1);
				c1.v1 = source(c1.e, mGraph); c1.v2 = target(c1.e, mGraph);

				CollisionConstraint c2(successor, task_id, current_in_delivery, current_timestep+1);
				if(nonCollisionMap.find(c1)!=nonCollisionMap.end()) col=true;
				if(nonCollisionMap.find(c2)!=nonCollisionMap.end()) col=true;
				auto stop2 = high_resolution_clock::now();
				mCCTime += (stop2 - start2);
				if(!col)
				{       
					SearchState new_state = SearchState(successor, current_timestep+1, 
						current_tasks_completed, current_in_delivery);
					if(lastSegment || !isOverFlowState(new_state,goal_state))
					{
						double new_count_collision_conflicts = 
							current_count_collision_conflicts +
							countCollisionConflicts(mVertexMap, new_state, task_id, agent_id);
							// countCollisionConflicts(agent_id,current_state,new_state,shortestPaths,consider_agents,mVertexCollisionPathsMap,mEdgeCollisionPathsMap);
						std::vector<double> new_cost = getHeuristics(agent_id, 
							new_state, goal_state, current_gvalue+mUnitEdgeLength, 
							true_makespan, current_count_collaboration_conflicts, 
							new_count_collision_conflicts, current_count_move_actions+mUnitEdgeLength);
						
						if(mFValue.count(new_state)==0 || compareVectors(new_cost,mFValue[new_state]))
						{
							auto yoma1 = high_resolution_clock::now();
							mFValue[new_state]= new_cost;
							mPrev[new_state]=current_state;
							pq.insert(new_state, new_cost);
							auto yoma2 = high_resolution_clock::now();
							mMapOperationsTime += yoma2-yoma1;
						}
					}
				}
			}
		}
		if(costOut == INF)
		{
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

	double getCostToGo(int &agent_id, SearchState &state)
	{
		double heuristic=0;
		if(state.in_delivery == true){
			heuristic += mAllPairsShortestPathMap[std::make_pair(state.vertex, 
				mTasksList[agent_id][state.tasks_completed].second.second)];
		}
		else
		{
			heuristic += mAllPairsShortestPathMap[std::make_pair(state.vertex, 
				mTasksList[agent_id][state.tasks_completed].second.first)];
			heuristic += mAllPairsShortestPathMap[std::make_pair(
				mTasksList[agent_id][state.tasks_completed].second.first, 
				mTasksList[agent_id][state.tasks_completed].second.second)];
		}
		for(int i=state.tasks_completed+1; i<mTasksList[agent_id].size(); i++)
		{
			heuristic += mAllPairsShortestPathMap[std::make_pair(
				mTasksList[agent_id][i-1].second.second,mTasksList[agent_id][i].second.first)];
			heuristic += mAllPairsShortestPathMap[std::make_pair(
				mTasksList[agent_id][i].second.first,mTasksList[agent_id][i].second.second)];
		}
		return heuristic;
	}

	std::vector<double> getHeuristics(int &agent_id, SearchState &state, SearchState &goal_state, 
		double g_value, double current_makespan,  
		double count_collaboration_conflicts, 
		double count_collision_conflicts, 
		double count_move_actions)
	{
		auto start1 = high_resolution_clock::now();
		double h_value=0;
		h_value = getCostToGo(agent_id, state);
		if(!(goal_state==SearchState())){
			h_value -= getCostToGo(agent_id, goal_state);
		} 
		std::vector<double> heuristics(6,g_value+h_value);
		heuristics[0] = std::max(0.0, g_value + h_value - current_makespan);
		heuristics[2] = count_collaboration_conflicts;
		// heuristics[1] = 0;
		heuristics[1] = count_collision_conflicts;
		heuristics[3] = count_move_actions+h_value;
		heuristics[4] = h_value;
		heuristics[5] = g_value+h_value;		

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
		std::cout<<" - ("<<
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
				std::cout<<" - ("<<
	int( (mGraph[p.shortestPaths[agent_id][i].vertex].state[0]+0.001)/mUnitEdgeLength)<<","<<
	int( (mGraph[p.shortestPaths[agent_id][i].vertex].state[1]+0.001)/mUnitEdgeLength)<<") "<<
	p.shortestPaths[agent_id][i].timestep <<" "<<p.shortestPaths[agent_id][i].tasks_completed<<" "<<
	p.shortestPaths[agent_id][i].in_delivery<<"\t";
			}
			std::cout << std::endl;
		}
	}

	void printCollabConstraint(CollaborationConstraint c){
		// return;
		std::cout << "Task ID: " << c.task_id << std::endl;
		std::cout << "Timestep: "<< c.timestep << std::endl;
		std::cout << "Is Pickup?: "<< (int)c.is_pickup << std::endl;
		std::cout << "Position: ";
		std::cout<<" - ("<<
			int( (mGraph[c.v].state[0]+0.001)/mUnitEdgeLength)<<","<<
			int( (mGraph[c.v].state[1]+0.001)/mUnitEdgeLength)<<") "<< std::endl;
	}

	void printCollabConflict(CollaborationConstraint c, std::vector <int> collaborating_agent_ids){
		// return;
		std::cout << "Agents involved in conflict: ";
		for(auto agent: collaborating_agent_ids){
			std::cout << agent << " ";
		}
		std::cout << std::endl;
		std::cout << "Task ID: " << c.task_id << std::endl;
		std::cout << "Timestep: "<< c.timestep << std::endl;
		std::cout << "Is Pickup?: "<< (int)c.is_pickup << std::endl;
		std::cout << "Position: ";
		std::cout<<" - ("<<
			int( (mGraph[c.v].state[0]+0.001)/mUnitEdgeLength)<<","<<
			int( (mGraph[c.v].state[1]+0.001)/mUnitEdgeLength)<<") "<< std::endl;
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
		std::cout << "In Delivery?: "<< (int)c1.in_delivery << std::endl;
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

		std::cout << "First constraint: \n";
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
		std::cout << "In Delivery?: "<< (int)c1.in_delivery << std::endl;

		std::cout << "Second constraint: \n";
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
		std::cout << "In Delivery?: "<< (int)c1.in_delivery << std::endl;
	}

	void printIntervals(boost::unordered_map <SearchState, allowedInterval, state_hash> nonCollabMap){
		for(auto it: nonCollabMap){
			std::cout << "State:\n";
			printState(it.first);
			std::cout << "Min Time = " << it.second.minTime << 
			" Max Time = " << it.second.maxTime << std::endl;
		}
	}
	void printState(SearchState c1){
		printVertex(c1.vertex);
		std::cout << "Tasks Id: " << c1.tasks_completed << std::endl;
		std::cout << "Timestep: "<< c1.timestep << std::endl;
		std::cout << "In Delivery?: "<< (int)c1.in_delivery << std::endl;
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
	
	void preprocess_graph(Graph &g)
	{
		auto start1 = high_resolution_clock::now();
		int V = boost::num_vertices(g);

		VertexIter vi_1, viend_1;
		VertexIter vi_2, viend_2;
		for (boost::tie(vi_1, viend_1) = vertices(g); vi_1 != viend_1; ++vi_1) 
		{
			for (boost::tie(vi_2, viend_2) = vertices(g); vi_2 != viend_2; ++vi_2) 
			{
				Vertex vertex_1 = *vi_1;
				Vertex vertex_2 = *vi_2;
				if(vertex_1==vertex_2)
					mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_2)]=0;
				else
				{
					Edge uv;
					bool edgeExists;
					boost::tie(uv, edgeExists) = edge(vertex_1, vertex_2, g);
					if(edgeExists)
						mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_2)]=mUnitEdgeLength;
					else
						mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_2)]=INF;
				}
			}
		}

		VertexIter vi_3, viend_3;
		for (boost::tie(vi_1, viend_1) = vertices(g); vi_1 != viend_1; ++vi_1) 
		for (boost::tie(vi_2, viend_2) = vertices(g); vi_2 != viend_2; ++vi_2) 
		for (boost::tie(vi_3, viend_3) = vertices(g); vi_3 != viend_3; ++vi_3) 
		{
			Vertex vertex_1 = *vi_1;
			Vertex vertex_2 = *vi_2;
			Vertex vertex_3 = *vi_3;
			if (mAllPairsShortestPathMap[std::make_pair(vertex_2,vertex_3)] + 0.00001 > 
				(mAllPairsShortestPathMap[std::make_pair(vertex_2,vertex_1)] + 
					mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_3)])
				&& (mAllPairsShortestPathMap[std::make_pair(vertex_2,vertex_1)] != INF
				&& mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_3)] != INF))
				mAllPairsShortestPathMap[std::make_pair(vertex_2,vertex_3)] = 
			mAllPairsShortestPathMap[std::make_pair(vertex_2,vertex_1)] + 
			mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_3)];
		}

		auto stop1 = high_resolution_clock::now();
		mPreprocessTime += (stop1 - start1);
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
				cv::line(image, source_Point, target_Point, cv::Scalar(0, 255, 255), 10);
			}

			VertexIter vi, vi_end;
			for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi)
			{
				double x_point = mGraph[*vi].state[0]*numberOfColumns;
				double y_point = (1 - mGraph[*vi].state[1])*numberOfRows;
				cv::Point centre_Point((int)x_point, (int)y_point);
				cv::circle(image, centre_Point, 20,  cv::Scalar(0, 150, 255), -1);
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
		for(int tid=0; tid<mTasksToAgentsList.size(); tid++)
		{
			int start_x = int( (mGraph[mTasksList[mTasksToAgentsList[tid][0].first][mTasksToAgentsList[tid][0].second].second.first].state[0]+0.0001)/mUnitEdgeLength);
			int start_y = int( (mGraph[mTasksList[mTasksToAgentsList[tid][0].first][mTasksToAgentsList[tid][0].second].second.first].state[1]+0.0001)/mUnitEdgeLength);

			int goal_x = int( (mGraph[mTasksList[mTasksToAgentsList[tid][0].first][mTasksToAgentsList[tid][0].second].second.second].state[0]+0.0001)/mUnitEdgeLength);
			int goal_y = int( (mGraph[mTasksList[mTasksToAgentsList[tid][0].first][mTasksToAgentsList[tid][0].second].second.second].state[1]+0.0001)/mUnitEdgeLength);

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