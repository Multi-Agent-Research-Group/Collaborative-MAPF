 
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
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/imgproc/imgproc_c.h"

// using namespace cv;
// #include <highgui.h>

#include "BGLDefinitions.hpp"
#include "time_priority_queue.hpp"
#include "CBSDefinitions.hpp"
#include "ISPS.hpp"
#include "PCSolver.hpp"
#include "LoadGraphfromFile.hpp"

#define PRINT if (cerr_disabled) {} else std::cout
#define DEBUG if (cerr_disabled) {} else 
bool cerr_disabled = true;

#include <chrono>
using namespace std::chrono;

#define INF std::numeric_limits<double>::infinity()

namespace HCBS {

using namespace BGL_DEFINITIONS;

class CBS
{

public:

	/// Number of agents
	int mNumAgents; 

	/// Number of Robots
	int mNumRobots;

	/// Path to the roadmap files.
	std::vector<std::string> mRoadmapFileNames;
	std::string mImagePath;

	/// Source vertex.
	std::vector<Eigen::VectorXd> mStartConfig;
	std::vector<Vertex> mStartVertex;

	/// Goal vertex.
	std::vector<Eigen::VectorXd> mGoalConfig;
	std::vector<Vertex> mGoalVertex;

	std::vector<int> mStartTimestep;
	std::vector<int> mGoalTimestep;

	// std::vector<std::pair<Eigen::VectorXd,std::pair<int,int>>> stationary_agents;

	int mHashUsed = 0;
	int mNotHashUsed = 0;
	int total_time = 0;

	double mUnitEdgeLength = 0.1;

	CBS(int numAgents, int numRobots, std::vector<std::string> roadmapFileNames, std::vector<Eigen::VectorXd> startConfig, std::vector<Eigen::VectorXd> goalConfig, 
		std::vector<int> startTimesteps, std::vector<int> goalTimesteps, std::vector<Vertex> startVertex, 
		std::vector<Vertex> goalVertex, std::string imagePath)
		: mNumAgents(numAgents)
		, mNumRobots(numRobots)
		, mRoadmapFileNames(roadmapFileNames)
		, mStartTimestep(startTimesteps)
		, mGoalTimestep(goalTimesteps)
		, mStartConfig(startConfig)
		, mGoalConfig(goalConfig)
		, mStartVertex(startVertex)
		, mGoalVertex(goalVertex)
		, mImagePath(imagePath)
		{}

	bool getVerticesCollisionStatus(Eigen::VectorXd left, Eigen::VectorXd right)
	{
		double distance = (left - right).norm();
		if(distance < 0.0141) // tune threshold!!
			return true;
		return false;
	}

	bool getEdgesCollisionStatus(Eigen::VectorXd left_source, Eigen::VectorXd left_target, Eigen::VectorXd right_source, Eigen::VectorXd right_target)
	{
		if ( (left_source - right_target).norm() < 0.0141 &&  (right_source - left_target).norm() < 0.0141)
			return true;
		return false;
	}

	bool checkCoupling(std::vector<std::vector<Vertex>> &paths, int &agent_id_1, Constraint &constraint_1, int &agent_id_2, Constraint &constraint_2)
	{
		// std::cerr<<"C-I";
		mProp = get(meta_data_t(), mPCGraph);
		int timeStep = 0;
		int maximum_timestep = 0;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++){
			meta_data *vertex = &get(mProp, agent_id);
			mStartTimestep[agent_id] = vertex->start_time;
			mGoalTimestep[agent_id] = vertex->start_time + paths[agent_id].size()-1;
			maximum_timestep = std::max(maximum_timestep, mGoalTimestep[agent_id]);
		}
		// std::cerr<<"L";
		// std::cout<<"MT: "<<maximum_timestep<<std::endl;std::cin.get();
		while(timeStep < maximum_timestep)
		{
			// std::cout<<timeStep<<std::endl;std::cin.get();
			std::vector<Vertex> source_vertices;
			std::vector<Vertex> target_vertices;
			std::vector<int> agent_ids;

			// std::cout<<mStartTimestep.size()<<" "<<mGoalTimestep.size()<<std::endl;
			// std::cin.get();
			
			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			{
				if( timeStep - mStartTimestep[agent_id] >= 0 &&  mGoalTimestep[agent_id] - timeStep >= 1)
				{
					agent_ids.push_back(agent_id);
					if(timeStep - mStartTimestep[agent_id]+1 > paths[agent_id].size())
					{
						std::cout<<"Outside memory!"; std::cin.get();
					}
					// std::cout<<"Index: "<<timeStep - mStartTimestep[agent_id];std::cin.get();

					// if(timeStep - mStartTimestep[agent_id] < paths[agent_id].size() - 1)
					// {
						source_vertices.push_back(paths[agent_id].at(timeStep - mStartTimestep[agent_id]));
						target_vertices.push_back(paths[agent_id].at(timeStep - mStartTimestep[agent_id]+1));
					// }
					// else
					// {
					//  source_vertices.push_back(paths[agent_id].at(paths[agent_id].size()-1));
					//  target_vertices.push_back(paths[agent_id].at(paths[agent_id].size()-1));
					// }
				}
			}

			// for(int i=0; i<agent_ids.size(); i++)
			//  std::cout<<agent_ids[i]<<" ";
			// std::cout<<std::endl;
			// std::cin.get();

			for(int i=0; i<agent_ids.size(); i++)
			for(int j=i+1; j<agent_ids.size(); j++)
			{
				// if(getVerticesCollisionStatus(mGraphs[agent_ids[i]][source_vertices[i]].state, mGraphs[agent_ids[j]][source_vertices[j]].state))
				// {
				//  agent_id_1 = agent_ids[i];
				//  agent_id_2 = agent_ids[j];

				//  bool agent_1_at_goal = paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == source_vertices[i];
				//  bool agent_2_at_goal = paths[agent_id_2].at(mGoalTimestep[agent_id_2]-mStartTimestep[agent_id_2]) == source_vertices[j];

				//  if(  (agent_1_at_goal && agent_2_at_goal )  && (target_vertices[i]!=source_vertices[i] || target_vertices[j]!=source_vertices[j])   ){
				// 		Edge edge_1 = boost::edge(source_vertices[i],target_vertices[i],mGraphs[agent_ids[i]]).first;
				// 		constraint_1 = Constraint(edge_1,timeStep+1);

				// 		Edge edge_2 = boost::edge(source_vertices[j],target_vertices[j],mGraphs[agent_ids[j]]).first;
				// 		constraint_2 = Constraint(edge_2,timeStep+1);
				// 		return true;
				// 	} 
				// }

				if(getVerticesCollisionStatus(mGraphs[agent_ids[i]][target_vertices[i]].state, mGraphs[agent_ids[j]][target_vertices[j]].state))
				{
					agent_id_1 = agent_ids[i];
					agent_id_2 = agent_ids[j];

					bool agent_1_safe = ((paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == target_vertices[i])
						|| (paths[agent_id_1].at(0) == target_vertices[i]));
					bool agent_2_safe = ((paths[agent_id_2].at(mGoalTimestep[agent_id_2]-mStartTimestep[agent_id_2]) == target_vertices[j])
						|| (paths[agent_id_2].at(0) == target_vertices[j]));

					if( !agent_1_safe || !agent_2_safe )
					{
						constraint_1 = Constraint(target_vertices[i],timeStep+1);
						constraint_2 = Constraint(target_vertices[j],timeStep+1);
						// std::cerr<<"O\n";
						return true;
					}
					// else{
					// 	std::cout << "same goal!\n";
					// 	continue;
					// }
				}
				
				if(getEdgesCollisionStatus(mGraphs[agent_ids[i]][source_vertices[i]].state, mGraphs[agent_ids[i]][target_vertices[i]].state, mGraphs[agent_ids[j]][source_vertices[j]].state, mGraphs[agent_ids[j]][target_vertices[j]].state))
				{
					agent_id_1 = agent_ids[i];
					agent_id_2 = agent_ids[j];

					bool agent_1_safe = ((source_vertices[i] == target_vertices[i])
						&& ((source_vertices[i] == paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]))
							|| source_vertices[i] == paths[agent_id_1].at(0)));

					bool agent_2_safe = ((source_vertices[j] == target_vertices[j])
						&& ((source_vertices[j] == paths[agent_id_2].at(mGoalTimestep[agent_id_2]-mStartTimestep[agent_id_2]))
							|| source_vertices[j] == paths[agent_id_2].at(0)));


					// bool agent_1_target_goal = paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == target_vertices[i];
					// bool agent_1_source_goal = paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == target_vertices[i];
					

					if(!agent_1_safe || !agent_2_safe)
					{
						Edge edge_1 = boost::edge(source_vertices[i],target_vertices[i],mGraphs[agent_ids[i]]).first;
						constraint_1 = Constraint(edge_1,timeStep+1);

						Edge edge_2 = boost::edge(source_vertices[j],target_vertices[j],mGraphs[agent_ids[j]]).first;
						constraint_2 = Constraint(edge_2,timeStep+1);
						// std::cerr<<"O\n";
						return true;
					}
				}
			}
			
			timeStep++;
		}

		// std::cerr<<"O\n";

		return false;
	}

	bool checkStationaryCoupling(std::vector<std::vector<Vertex>> &paths, int &agent_id_1, Constraint &constraint_1)
	{
		// std::cerr<<"S-I\n";
		std::vector<std::pair<Eigen::VectorXd,std::pair<int,int>>> stationary_agents;
		mProp = get(meta_data_t(), mPCGraph);

		int timeStep = 0;
		int maximum_timestep = 0;

		std::vector<Eigen::VectorXd> sp_goal(mNumRobots);
		std::vector<int> sp_goal_timestep(mNumRobots,0);

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			meta_data *vertex = &get(mProp, agent_id);
			mStartTimestep[agent_id] = vertex->start_time;
			mGoalTimestep[agent_id] = vertex->start_time + paths[agent_id].size()-1;
			maximum_timestep = std::max(maximum_timestep, mGoalTimestep[agent_id]);

			std::vector <int> agent_list = vertex->agent_list;
			for (auto robotNum: agent_list)
			{
				if(mGoalTimestep[agent_id] > sp_goal_timestep[robotNum])
				{
					sp_goal[robotNum] = mGoalConfig[agent_id];
					sp_goal_timestep[robotNum] = mGoalTimestep[agent_id];
				}
			}
		}

		for(int robot_id=0; robot_id<mNumRobots; robot_id++)
			if(maximum_timestep != sp_goal_timestep[robot_id])
				stationary_agents.push_back(std::make_pair(sp_goal[robot_id],
					std::make_pair(sp_goal_timestep[robot_id],maximum_timestep)));

		// std::cout<<"Stationary Conflict Makespan Timestep: "<<maximum_timestep<<std::endl;

		// std::cout<<"MT: "<<maximum_timestep<<std::endl;std::cin.get();
		while(timeStep < maximum_timestep)
		{
			std::vector<Vertex> source_vertices;
			std::vector<Vertex> target_vertices;
			std::vector<int> agent_ids;

			std::vector<int> s_ids;

			// std::cout<<mStartTimestep.size()<<" "<<mGoalTimestep.size()<<std::endl;
			// std::cin.get();
			
			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			{
				if( timeStep - mStartTimestep[agent_id] >= 0 &&  mGoalTimestep[agent_id] - timeStep >= 1)
				{
					agent_ids.push_back(agent_id);
					if(timeStep - mStartTimestep[agent_id]+1 > paths[agent_id].size())
					{
						std::cout<<"Outside memory!"; std::cin.get();
					}
					source_vertices.push_back(paths[agent_id].at(timeStep - mStartTimestep[agent_id]));
					target_vertices.push_back(paths[agent_id].at(timeStep - mStartTimestep[agent_id]+1));
				}
			}
			for(int s_id=0; s_id<stationary_agents.size(); s_id++)
			{
				if( timeStep - stationary_agents[s_id].second.first >= 0 &&  stationary_agents[s_id].second.second - timeStep >= 1)
				{
					s_ids.push_back(s_id);
				}
			}

			// for(int i=0; i<agent_ids.size(); i++)
			//  std::cout<<agent_ids[i]<<" ";
			// std::cout<<std::endl;
			// std::cin.get();

			for(int i=0; i<agent_ids.size(); i++)
			for(int j=0; j<s_ids.size(); j++)
			{
				if(getVerticesCollisionStatus(mGraphs[agent_ids[i]][target_vertices[i]].state, stationary_agents[s_ids[j]].first))
				{
					agent_id_1 = agent_ids[i];
					int agent_id_2 = s_ids[j];

					bool agent_1_safe = ((paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == target_vertices[i])
						|| (paths[agent_id_1].at(0) == target_vertices[i]));

					if( !agent_1_safe)
					{
						constraint_1 = Constraint(target_vertices[i],timeStep+1);
						// std::cerr<<"O\n";
						return true;
					}
				}
				
				if(getEdgesCollisionStatus(mGraphs[agent_ids[i]][source_vertices[i]].state, mGraphs[agent_ids[i]][target_vertices[i]].state, stationary_agents[s_ids[j]].first, stationary_agents[s_ids[j]].first))
				{
					agent_id_1 = agent_ids[i];
					int agent_id_2 = s_ids[j];

					bool agent_1_safe = ((source_vertices[i] == target_vertices[i])
						&& ((source_vertices[i] == paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]))
							|| source_vertices[i] == paths[agent_id_1].at(0)));

					// bool agent_1_target_goal = paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == target_vertices[i];
					// bool agent_1_source_goal = paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == target_vertices[i];
					

					if(!agent_1_safe)
					{
						Edge edge_1 = boost::edge(source_vertices[i],target_vertices[i],mGraphs[agent_ids[i]]).first;
						constraint_1 = Constraint(edge_1,timeStep+1);
						// std::cerr<<"O\n";
						return true;
					}
				}
			}
			
			timeStep++;
		}
		// std::cerr<<"O\n";
		return false;
	}

	std::vector<std::vector<Eigen::VectorXd>> solve(high_resolution_clock::time_point &solve_start_time,
		int &search_num)
	{
		PRINT<<"Solve called!!"<<std::endl;
		CBSPriorityQueue PQ(mNumAgents);

		std::vector<std::vector<Constraint>> constraints(mNumAgents, std::vector<Constraint>());
		double start_cost;


		ISPS planner(mNumAgents,mRoadmapFileNames,mStartConfig,mGoalConfig,
			mStartVertex, mGoalVertex, constraints);

		std::vector< std::vector<Vertex> > start_shortestPaths = planner.solve(start_cost);

		
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			if(start_shortestPaths.at(agent_id).size()==0)
			{
				// std::cerr << mNumAgents << std::endl;
				// std::cout<<"No Path exists for index "<<agent_id<<"! Press [ENTER] to exit: ";
				// std::cin.get();
				return std::vector<std::vector<Eigen::VectorXd>>(mNumAgents,std::vector<Eigen::VectorXd>());
			}

		PQ.insert(start_cost, constraints, start_shortestPaths);


		// auto solve_start_time = high_resolution_clock::now();

		int numSearches = 0;
		while(PQ.PQsize()!=0)
		{
			// PQ.print();
			numSearches++;
			search_num = numSearches;
			// std::cout << numSearches << std::endl;
			PRINT<< numSearches << std::endl;
			Element p = PQ.pop();

			auto stop = high_resolution_clock::now();
			std::chrono::duration<double, std::micro> timespent = stop - solve_start_time;
			if (timespent.count() > 300000000)
			{
				break;
			}

			// if(numSearches % 1000 == 0)
			// {
			// 	std::cout<<"CBS numSearches: "<<numSearches<<std::endl;
			// 	// break;
			// }

			// if(numSearches%100 == 0)
			{
				// std::cout<<PQ.PQsize()<<std::endl;
				// std::cerr<<"CBS numSearches: "<<numSearches<<" Cost: "<<int((p.cost+0.0001)/mUnitEdgeLength)<<std::endl;
				for(int agent_id=0; agent_id<mNumAgents; agent_id++)
				{
					PRINT<<"Agent ID: "<<agent_id<<std::endl;
					PRINT<<"Start: "<<mStartConfig[agent_id][0]<<", "<<mStartConfig[agent_id][1]<<std::endl;
					PRINT<<"Goal: "<<mGoalConfig[agent_id][0]<<", "<<mGoalConfig[agent_id][1]<<std::endl;
					PRINT<<"Start Timestep: "<<mStartTimestep[agent_id]<<" Goal Timestep: "<<mGoalTimestep[agent_id]<<std::endl;
					PRINT<<"Collision constraints size: "<<p.constraints[agent_id].size()<<std::endl;
					for(int i=0; i<p.constraints[agent_id].size(); i++)
					{
						if(p.constraints[agent_id][i].constraint_type==1)
							PRINT<<"Vertex constraint: "<<p.constraints[agent_id][i].v<<" "<<p.constraints[agent_id][i].t<<std::endl;
						else
							PRINT<<"Edge constraint: "<<p.constraints[agent_id][i].e<<" "<<p.constraints[agent_id][i].t<<std::endl;
					}

					PRINT<<"Path: "<<std::endl;
					for(int i=0; i<p.shortestPaths[agent_id].size(); i++)
						PRINT<<p.shortestPaths[agent_id][i]<<" - ("<<int( (mGraphs[agent_id][p.shortestPaths[agent_id][i]].state[0]+0.001)/mUnitEdgeLength)<<","
							<<int( (mGraphs[agent_id][p.shortestPaths[agent_id][i]].state[1]+0.001)/mUnitEdgeLength)<<") "<<std::endl;
					PRINT<<std::endl;
				}
				DEBUG std::cin.get();
				// break;
			}

			int agent_id_1 = -1;
			Constraint constraint_1;

			int agent_id_2 = -1;
			Constraint constraint_2;

			{
				container mTopologicalOrder;
				topological_sort(mPCGraph, std::back_inserter(mTopologicalOrder));
				for (container::reverse_iterator ii=mTopologicalOrder.rbegin(); ii!=mTopologicalOrder.rend(); ++ii)
				{
					int agent_id = *ii;
					meta_data *vertex = &get(mProp, agent_id);
					vector <int> predecessors = mPredecessors[agent_id];
					if(predecessors.size() == 0){
						vertex->start_time = 0;
						vertex->end_time = p.shortestPaths[agent_id].size()-1;
						// std::cerr << "here" << std::endl;
					}
					else{
						int makespan = 0;
						for(auto pred: predecessors){
							meta_data *pred_vertex = &get(mProp, pred);
							if(pred_vertex->end_time>makespan){makespan = pred_vertex->end_time;}
						}
						vertex->start_time = makespan;
						vertex->end_time = vertex->start_time+p.shortestPaths[agent_id].size()-1;
						// std::cerr << "there" << std::endl;
					}
					// std::cin.get();
				}
			}


			if(!checkCoupling(p.shortestPaths, agent_id_1, constraint_1, agent_id_2, constraint_2))
			{
				if(!checkStationaryCoupling(p.shortestPaths, agent_id_1, constraint_1))
				{
					// std::cerr<<"returning!";
					// std::cin.get();
					// std::cout<<"numSearches: "<<numSearches<<std::endl;

					PRINT<<"\n-\nCBS numSearches: "<<numSearches<<" Cost: "<<int((p.cost+0.0001)/mUnitEdgeLength)<<std::endl;
					for(int agent_id=0; agent_id<mNumAgents; agent_id++)
					{
						PRINT<<"Agent ID: "<<agent_id<<std::endl;
						PRINT<<"Start: "<<mStartConfig[agent_id][0]<<", "<<mStartConfig[agent_id][1]<<std::endl;
						PRINT<<"Goal: "<<mGoalConfig[agent_id][0]<<", "<<mGoalConfig[agent_id][1]<<std::endl;
						PRINT<<"Start Timestep: "<<mStartTimestep[agent_id]<<" Goal Timestep: "<<mGoalTimestep[agent_id]<<std::endl;
						PRINT<<"Path size: "<<p.shortestPaths[agent_id].size()<<std::endl;
					}

					std::vector<std::vector<Eigen::VectorXd>> collision_free_path(mNumAgents, std::vector<Eigen::VectorXd>());

					// std::cout<<" Path Cost: "<<p.cost<<std::endl;
					for(int agent_id=0; agent_id<mNumAgents; agent_id++)
					{
						// std::cout<<"Shortest Path Cost for index - "<<agent_id<<" : "<<p.costs[agent_id]<<std::endl;
						// std::cout<<"Shortest Path for index - "<<agent_id<<" : ";
						for(Vertex &nodes: p.shortestPaths[agent_id])
						{
							// std::cout<<mGraphs[agent_id][nodes].vertex_index<<"_"<<mGraphs[agent_id][nodes].state<<" ";
							collision_free_path[agent_id].push_back(mGraphs[agent_id][nodes].state);
						}
						// std::cout<<std::endl;
					}
					// std::cerr<<"returning!"<<std::endl;

					return collision_free_path;
				}
				PRINT<<"numSearches: "<<numSearches<<std::endl;
				PRINT<<"Stationary conflict!!";

				std::vector<std::vector<Constraint>> increase_constraints_agent_id_1 = p.constraints;

				// std::cout<<increase_constraints_agent_id_1.size()<<" "<<agent_id_1<<std::endl;
				increase_constraints_agent_id_1[agent_id_1].push_back(constraint_1);

				ISPS planner1(mNumAgents,mRoadmapFileNames,mStartConfig,mGoalConfig, 
					mStartVertex, mGoalVertex, increase_constraints_agent_id_1);
				double cost_agent_id_1;
				std::vector< std::vector<Vertex> > shortestPaths_agent_id_1 = planner1.solve(cost_agent_id_1);
				
				bool all_paths_found = true;
				for(int agent_id=0; agent_id<mNumAgents; agent_id++)
					if(shortestPaths_agent_id_1.at(agent_id).size()==0)
					{
						all_paths_found = false;
						break;
					}

				if(all_paths_found) 
					PQ.insert(cost_agent_id_1,increase_constraints_agent_id_1,shortestPaths_agent_id_1);
				// }
				continue;
			} 
			// if(numSearches%1000 == 0)
			// {
			// 	std::cout<<"CBS numSearches"<<numSearches<<std::endl;
			// 	if(constraint_1.constraint_type == 1)
			// 		std::cout<<"Vertex Constraint: ("<<int( (mGraphs[agent_id_1][constraint_1.v].state[0]+0.001)/mUnitEdgeLength)<<","
			// 			<<int( (mGraphs[agent_id_1][constraint_1.v].state[1]+0.001)/mUnitEdgeLength)<<") at "<<constraint_1.t<<std::endl;
			// 	else
			// 	{
			// 		std::cout<<"Edge Constraint: ("<<int( (mGraphs[agent_id_1][source(constraint_1.e, mGraphs[agent_id_1])].state[0]+0.001)/mUnitEdgeLength)<<","
			// 			<<int( (mGraphs[agent_id_1][source(constraint_1.e, mGraphs[agent_id_1])].state[1]+0.001)/mUnitEdgeLength)<<") , ("<<int( (mGraphs[agent_id_1][target(constraint_1.e, mGraphs[agent_id_1])].state[0]+0.001)/mUnitEdgeLength)<<","
			// 			<<int( (mGraphs[agent_id_1][target(constraint_1.e, mGraphs[agent_id_1])].state[1]+0.001)/mUnitEdgeLength)<<") at "<<constraint_1.t<<std::endl;
			// 	}
			// 	std::cin.get();
			// }

			// std::cout<<"K";std::cin.get();

			//agent_id_1

			std::vector<std::vector<Constraint>> increase_constraints_agent_id_1 = p.constraints;

			// std::cout<<increase_constraints_agent_id_1.size()<<" "<<agent_id_1<<std::endl;
			increase_constraints_agent_id_1[agent_id_1].push_back(constraint_1);
			
			ISPS planner1(mNumAgents,mRoadmapFileNames,mStartConfig,mGoalConfig,
				mStartVertex, mGoalVertex, increase_constraints_agent_id_1);
			double cost_agent_id_1;
			std::vector< std::vector<Vertex> > shortestPaths_agent_id_1 = planner1.solve(cost_agent_id_1);
			
			bool all_paths_found = true;
			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
				if(shortestPaths_agent_id_1.at(agent_id).size()==0)
				{
					all_paths_found = false;
					break;
				}

			if(all_paths_found)
				PQ.insert(cost_agent_id_1,increase_constraints_agent_id_1,shortestPaths_agent_id_1);

			std::vector<std::vector<Constraint>> increase_constraints_agent_id_2 = p.constraints;
			increase_constraints_agent_id_2[agent_id_2].push_back(constraint_2);

			ISPS planner2(mNumAgents,mRoadmapFileNames,mStartConfig,mGoalConfig,
				mStartVertex, mGoalVertex, increase_constraints_agent_id_2);
			double cost_agent_id_2;
			std::vector< std::vector<Vertex> > shortestPaths_agent_id_2 = planner2.solve(cost_agent_id_2);
			all_paths_found = true;
			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
				if(shortestPaths_agent_id_2.at(agent_id).size()==0)
				{
					all_paths_found = false;
					break;
				}

			if(all_paths_found)
				PQ.insert(cost_agent_id_2,increase_constraints_agent_id_2,shortestPaths_agent_id_2);
		}

		return std::vector<std::vector<Eigen::VectorXd>>(mNumAgents,std::vector<Eigen::VectorXd>());
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
		std::vector<Vertex> neighbors;
		OutEdgeIter ei, ei_end;

		for (boost::tie(ei, ei_end) = out_edges(v, graph); ei != ei_end; ++ei) 
		{
			Vertex curSucc = target(*ei, graph);
			Edge e = *ei;
			if(!graph[e].isEvaluated)
			{
				std::cout<<"CBS - Not evaluated!!";
				std::cin.get();
				evaluateIndividualEdge(graph,e);
			}
			if(graph[e].status == CollisionStatus::FREE)
				neighbors.push_back(curSucc);
		}

		// std::cout<<"neighbors size: "<<neighbors.size()<<std::endl;
		return neighbors;
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
			ss << "./src/CMAPF/data/viz/new_images/";
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
			for(boost::tie(ei,ei_end) = edges(mGraphs[agent_id]); ei!=ei_end;++ei)
			{
				cv::Point source_Point((int)(mGraphs[agent_id][source(*ei,mGraphs[agent_id])].state[0]*numberOfColumns), 
					(int)((1-mGraphs[agent_id][source(*ei,mGraphs[agent_id])].state[1])*numberOfColumns));
				cv::Point target_Point((int)(mGraphs[agent_id][target(*ei,mGraphs[agent_id])].state[0]*numberOfColumns), 
					(int)((1-mGraphs[agent_id][target(*ei,mGraphs[agent_id])].state[1])*numberOfColumns));
				cv::line(image, source_Point, target_Point, cv::Scalar(0, 255, 255), 10);
			}

			VertexIter vi, vi_end;
			for (boost::tie(vi, vi_end) = vertices(mGraphs[agent_id]); vi != vi_end; ++vi)
			{
				double x_point = mGraphs[agent_id][*vi].state[0]*numberOfColumns;
				double y_point = (1 - mGraphs[agent_id][*vi].state[1])*numberOfRows;
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

		// vector <vector <int>> mTasksToAgentsList;

		// for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		// {
		// 	meta_data *vertex = &get(mProp, agent_id);
		// 	mTasksToAgentsList.push_back(vertex->agent_list);
		// } 

		

		std::vector< std::pair<std::pair<int,int>, std::pair<int,int>> >  tasks;
		std::cout << mNumAgents << std::endl;
		for(int tid=0; tid<mStartVertex.size(); tid++)
		{
			int start_x = int( (mGraphs[0][mStartVertex[tid]].state[0]+0.0001)/mUnitEdgeLength);
			int start_y = int( (mGraphs[0][mStartVertex[tid]].state[1]+0.0001)/mUnitEdgeLength);

			int goal_x = int( (mGraphs[0][mGoalVertex[tid]].state[0]+0.0001)/mUnitEdgeLength);
			int goal_y = int( (mGraphs[0][mGoalVertex[tid]].state[1]+0.0001)/mUnitEdgeLength);

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
				
				for (int num = 0; num<1; num++){
					std::string path = mImagePath+std::to_string(num_image)+".jpg";
					cv::imwrite(path, new_image);
					num_image += 1;
				}
				cv::namedWindow("Agents",cv::WINDOW_NORMAL);
				cv::imshow("Agents", new_image);
				cv::waitKey(10);
			}
		}
		cv::namedWindow("Graph Visualization",cv::WINDOW_NORMAL);
		cv::imshow("Graph Visualization", image);
		cv::waitKey(0);
	}
};
} // namespace HCBS

#endif 
