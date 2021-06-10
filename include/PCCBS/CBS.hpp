
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


#define INF std::numeric_limits<int>::max()

#define PRINT if (cerr_disabled) {} else std::cerr
#define DEBUG if (cerr_disabled) {} else 
bool cerr_disabled = true;

namespace PCCBS {

using namespace BGL_DEFINITIONS;

class CBS
{

public:

	std::chrono::duration<double, std::micro> mCSPTime;
	std::chrono::duration<double, std::micro> mGNTime;
	std::chrono::duration<double, std::micro> mQOTime;
	std::chrono::duration<double, std::micro> mCCTime;
	std::chrono::duration<double, std::micro> mPlanningTime;
	std::chrono::duration<double, std::micro> mPreprocessTime;

	std::vector<int> mTaskStartTimestep;

	high_resolution_clock::time_point mSolveStartTime;

	int mCBSIterations;

	/// Environment
	cv::Mat mImage;

	std::vector<std::vector<std::pair<int,std::pair<Vertex,Vertex>>>> mTasksList;
	std::vector<std::vector<std::pair<int,int>>> mTasksToAgentsList;
	std::vector<std::unordered_map<Vertex,bool>> mSpecialPosition;

	/// The fixed graphs denoting individual environment of corresponding agents
	std::vector<Graph> mGraphs;

	/// Number of agents
	int mNumAgents; 

	double mUnitEdgeLength = 0.1;

	/// Path to the roadmap files.
	std::vector<std::string> mRoadmapFileNames;

	/// Source vertex.
	std::vector<Eigen::VectorXd> mStartConfig;
	std::vector<Vertex> mStartVertex;

	std::vector<int> mStartTimestep;
	std::vector<int> mGoalTimestep;

	boost::unordered_map<std::pair<Vertex,Vertex>,int> mAllPairsShortestPathMap;

	CBS(PrecedenceConstraintGraph _pcg, cv::Mat img, int numAgents, std::vector<std::string> roadmapFileNames, Eigen::VectorXd _start_config, std::vector<int> taskStartTimestep)
		: mImage(img)
		, mNumAgents(numAgents)
		, mRoadmapFileNames(roadmapFileNames)
		, mTaskStartTimestep(taskStartTimestep)
	{

		int numTasks = boost::num_vertices(_pcg);
		std::vector<std::vector<std::pair<int,std::pair<Eigen::VectorXd,Eigen::VectorXd>>>> _tasks_list(numAgents);
		std::vector<std::vector<std::pair<int,int>>> _tasks_to_agents_list(numTasks);
		std::vector< PCVertex > c;
		topological_sort(_pcg, std::back_inserter(c));

		for ( std::vector< PCVertex >::reverse_iterator ii=c.rbegin(); ii!=c.rend(); ++ii)
		{
			// std::cout << std::endl;
			// std::cerr<<"iter\n";std::cin.get();
			meta_data vertex = get(get(meta_data_t(), _pcg), *ii);

			int task_id = vertex.task_id;
			// std::cout << "Task id:"<<task_id << std::endl;

			Eigen::VectorXd start_config(2);
			start_config[0] = vertex.start.first;
			start_config[1] = vertex.start.second;

			Eigen::VectorXd goal_config(2);
			goal_config[0] = vertex.goal.first;
			goal_config[1] = vertex.goal.second;

			std::vector <int> agent_list = vertex.agent_list;
			for (auto agentNum: agent_list){
				PRINT << agentNum << std::endl;
				_tasks_to_agents_list[task_id].push_back(std::make_pair(agentNum,_tasks_list[agentNum].size()));
				_tasks_list[agentNum].push_back(std::make_pair(task_id, std::make_pair(start_config, goal_config)));
			}
			// PRINT << std::endl;
			// PRINT<<"loop end\n";
			// std::cin.get();
		}

		// std::cin.get();

		PRINT<<"OUT";

		mTasksToAgentsList = _tasks_to_agents_list;

		auto t1 = std::chrono::high_resolution_clock::now();
	    auto t2 = std::chrono::high_resolution_clock::now();
		mCSPTime = t2-t1;
		mGNTime = t2-t1;
		mQOTime = t2-t1;
		mCCTime = t2-t1;
		mPlanningTime = t2-t1;
		mPreprocessTime = t2-t1;

		PRINT<<"K";

		for(int i=0; i<mNumAgents;i++)
		{
			Eigen::VectorXd start_config(2);
			for (int ui = i*2; ui < i*2+2; ui++)
				start_config[ui-i*2] = _start_config[ui];
			mStartConfig.push_back(start_config);
		}

		PRINT<<"K";

		for(int i=0; i<_tasks_list.size(); i++)
		{
			std::vector<std::pair<int,std::pair<Vertex,Vertex>>> agent_tasks_list;
			for(int j=0; j<_tasks_list[i].size(); j++)
				agent_tasks_list.push_back(std::make_pair(_tasks_list[i][j].first,std::make_pair(0,0)));
			mTasksList.push_back(agent_tasks_list);
		}

		PRINT<<"K";

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			Graph graph;
			Vertex start_vertex;
			PRINT<<"L: "<<agent_id<<"\n";

			create_vertices(graph,get(&VProp::state,graph),mRoadmapFileNames[agent_id],2,get(&EProp::prior,graph));
			create_edges(graph,get(&EProp::length,graph));

			PRINT<<"M: "<<agent_id<<"\n";

			VertexIter ind_vi, ind_vi_end;
			int i=0;
			for (boost::tie(ind_vi, ind_vi_end) = vertices(graph); ind_vi != ind_vi_end; ++ind_vi,++i)
			{
				put(&VProp::vertex_index,graph,*ind_vi,i);
				if(mStartConfig[agent_id].isApprox(graph[*ind_vi].state))
					start_vertex = *ind_vi;
				for(int i=0; i<_tasks_list[agent_id].size(); i++)
				{
					if(_tasks_list[agent_id][i].second.first.isApprox(graph[*ind_vi].state))
						mTasksList[agent_id][i].second.first = *ind_vi;
					if(_tasks_list[agent_id][i].second.second.isApprox(graph[*ind_vi].state))
						mTasksList[agent_id][i].second.second = *ind_vi;
				}
			}
			PRINT<<"N: "<<agent_id<<"\n";

			mGraphs.push_back(graph);
			mStartVertex.push_back(start_vertex);
			PRINT<<"out\n";
		}

		PRINT<<"K";

		for(int i=0; i<mTasksList.size(); i++)
		{
			std::unordered_map<Vertex,bool> special_positions;
			for(int j=0; j<mTasksList[i].size(); j++)
			{
				special_positions[mTasksList[i][j].second.first]=true;
				special_positions[mTasksList[i][j].second.second]=true;
			}
			mSpecialPosition.push_back(special_positions);
		}

		PRINT<<"K";

		// for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		preprocess_graph(mGraphs[0]);
	}

	void printStats()
	{
		std::cout<<mPlanningTime.count()/1000000.0<<" "<<mCBSIterations<<std::endl;
		// std::cout<<"computeShortestPath time: "<<mCSPTime.count()/1000000.0<<std::endl;
		// std::cout<<"Queue Operations time: "<<mQOTime.count()/1000000.0<<std::endl;
		// std::cout<<"Get Neighbors time: "<<mGNTime.count()/1000000.0<<std::endl;
		// std::cout<<"Constraints time: "<<mCCTime.count()/1000000.0<<std::endl;
		// std::cout<<"Preproccessing time: "<<mPreprocessTime.count()/1000000.0<<std::endl;
	}


	std::vector< std::vector<SearchState> > computeDecoupledPaths(std::vector<std::vector<CollisionConstraint>> collision_constraints, 
		std::vector<std::vector<CollaborationConstraint>> collaboration_constraints, std::vector<std::vector<CollaborationConstraint>> non_collaboration_constraints, 
		std::vector<int> &costs)
	{
		std::vector<std::vector<SearchState> > shortestPaths;
		std::vector<int> consider_agents;
		int current_makespan = 0;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			int ind_cost;
			std::vector <SearchState> path = computeShortestPath(agent_id, collision_constraints[agent_id], collaboration_constraints[agent_id], 
				non_collaboration_constraints[agent_id], ind_cost, current_makespan, shortestPaths, consider_agents);
			shortestPaths.push_back(path);
			costs.push_back(ind_cost);
			consider_agents.push_back(agent_id);
		}

		return shortestPaths;
	}

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

	int checkCollaborationConflicts(std::vector<int> task_ids, std::vector<int> agent_ids, std::vector<std::vector<SearchState>> &paths,
		std::vector<int> &collaborating_agent_ids, CollaborationConstraint &constraint_c)
	{
		int count_conflicts=0;
		bool selected_constraint = false;

		for(int t=0; t<task_ids.size(); t++)
		{
			int tid = task_ids[t];
			Vertex pickup_vertex;
			std::set<int> pickup_timesteps;
			Vertex delivery_vertex;
			std::set<int> delivery_timesteps;

			std::vector<int> current_task_agent_ids;
			for(int k=0; k<mTasksToAgentsList[tid].size(); k++)
			{
				int agent_id = mTasksToAgentsList[tid][k].first;
				int agent_index = -1;
				for(int a=0; a<agent_ids.size(); a++)
					if(agent_id == agent_ids[a])
					{
						agent_index = a;
						break;
					}
				if(agent_index!=-1)
				{
					current_task_agent_ids.push_back(agent_id);
					int task_index = mTasksToAgentsList[tid][k].second;
					pickup_vertex = mTasksList[agent_id][task_index].second.first;
					delivery_vertex = mTasksList[agent_id][task_index].second.second;
					for(int i=1; i<paths[agent_index].size(); i++)
					{	
						if(paths[agent_index][i].tasks_completed == task_index && paths[agent_index][i].in_delivery == true
							&& paths[agent_index][i-1].in_delivery == false)
							pickup_timesteps.insert(paths[agent_index][i].timestep);
						if(paths[agent_index][i].tasks_completed == task_index+1 && paths[agent_index][i].in_delivery == false
							&& paths[agent_index][i-1].in_delivery == true)
							delivery_timesteps.insert(paths[agent_index][i].timestep);
					}
				}
			}
			if(pickup_timesteps.size()>1)
			{
				count_conflicts++;
				if(!selected_constraint)
				{
					selected_constraint = true;
					collaborating_agent_ids = current_task_agent_ids;
					int collaboration_timestep = *pickup_timesteps.begin();
					constraint_c = CollaborationConstraint(pickup_vertex, tid, true,collaboration_timestep);
				}
				return true;
			}
			if(delivery_timesteps.size()>1)
			{
				count_conflicts++;
				if(!selected_constraint)
				{
					selected_constraint = true;
					collaborating_agent_ids = current_task_agent_ids;
					int collaboration_timestep = *delivery_timesteps.begin();
					constraint_c = CollaborationConstraint(delivery_vertex, tid, false,collaboration_timestep);
				}
			}
		}
		return count_conflicts;
	}

	int checkCollisionConflicts(std::vector<int> agent_ids, std::vector<std::vector<SearchState>> &paths, std::vector<int> &agent_id_1, CollisionConstraint &constraint_1, std::vector<int> &agent_id_2, CollisionConstraint &constraint_2)
	{
		int count_conflicts=0;
		bool selected_constraint = false;

		int num_considered_agents = agent_ids.size();
		int current_timestep = 0;
		int maximum_timestep = 0;
		for(int i=0; i<num_considered_agents; i++)
			maximum_timestep = std::max(maximum_timestep, paths[i].at(paths[i].size()-1).timestep);
		// std::cout<<"MT: "<<maximum_timestep<<std::endl;
		std::vector<int> current_path_id(num_considered_agents, 0);
		while(current_timestep < maximum_timestep)
		{
			// std::cout<<"\nCT:"<<current_timestep<<std::endl;
			std::vector<int> source_task_ids(num_considered_agents, -1);
			std::vector<Vertex> source_vertices(num_considered_agents); // all source vertices
			std::vector<int> source_tasks_completed(num_considered_agents, 0);
			std::vector<bool> source_in_delivery(num_considered_agents, false);

			std::vector<int> target_task_ids(num_considered_agents, -1);
			std::vector<Vertex> target_vertices(num_considered_agents); // target vertex for collab agents
			std::vector<int> target_tasks_completed(num_considered_agents, 0);
			std::vector<bool> target_in_delivery(num_considered_agents, false);


			for(int i=0; i<num_considered_agents; i++)
			{
				int agent_id = agent_ids[i];
				if(current_path_id[i] == paths[i].size())
				{
					source_vertices[i] = paths[i].at(paths[i].size()-1).vertex;
					source_tasks_completed[i] = paths[i].at(paths[i].size()-1).tasks_completed;
					source_in_delivery[i] = paths[i].at(paths[i].size()-1).in_delivery;
				}
				else
				{
					source_vertices[i] = paths[i].at(current_path_id[i]).vertex;
					while(current_path_id[i] < paths[i].size()
						&& paths[i].at(current_path_id[i]).timestep == current_timestep)
					{ // more than one timestep means that you are either picking up at source, or delivering
						if(paths[i].at(current_path_id[i]).in_delivery == true)
							source_task_ids[i] = mTasksList[agent_id][paths[i].at(current_path_id[i]).tasks_completed].first;
						else
							source_task_ids[i] = -1;
						source_tasks_completed[i] = paths[i].at(current_path_id[i]).tasks_completed;
						source_in_delivery[i] = paths[i].at(current_path_id[i]).in_delivery;
						current_path_id[i]++;
					}
				}

				if(current_path_id[i] == paths[i].size())
				{
					target_vertices[i] = paths[i].at(paths[i].size()-1).vertex;
					target_tasks_completed[i] = paths[i].at(paths[i].size()-1).tasks_completed;
					target_in_delivery[i] = paths[i].at(paths[i].size()-1).in_delivery;
				}
				else
				{
					int target_path_id = current_path_id[i];
					target_vertices[i] = paths[i].at(target_path_id).vertex;
					target_tasks_completed[i] = paths[i].at(target_path_id).tasks_completed;
					target_in_delivery[i] = paths[i].at(target_path_id).in_delivery;
					if(paths[i].at(target_path_id).in_delivery == true)
						target_task_ids[i] = mTasksList[agent_id][paths[i].at(target_path_id).tasks_completed].first;
					else
						target_task_ids[i] = -1;

					while(target_path_id < paths[i].size()
						&& paths[i].at(target_path_id).timestep == current_timestep+1)
					{
						target_path_id++;
					}
				}
			}

			// for(int i=0; i<num_considered_agents; i++)
			// {
			// 	std::cout<<"SV: "<<source_vertices[i]<<" "<<" TV: "<<target_vertices[i]<<std::endl;
			// 	// std::cout<<source_task_ids[i]
			// }

			// for(int i=0; i<agent_ids.size(); i++)
			//  std::cout<<agent_ids[i]<<" ";
			// std::cout<<std::endl;
			// std::cin.get();

			PRINT<<"CT: "<<current_timestep<<std::endl;
			for(int i=0; i<num_considered_agents; i++)
			for(int j=i+1; j<num_considered_agents; j++)
			{
				if(target_task_ids[i]==-1 || target_task_ids[i]!=target_task_ids[j])
				{
					PRINT<<"Checking between agents: "<<agent_ids[i]<<" "<<agent_ids[j]<<std::endl;
					if(getVerticesCollisionStatus(mGraphs[agent_ids[i]][target_vertices[i]].state, mGraphs[agent_ids[j]][target_vertices[j]].state))
					{
						bool safe_i = false;
						bool safe_j = false;
						if(target_in_delivery[i] == false)
						{
							if(target_tasks_completed[i] == 0)
								safe_i = (mStartVertex[agent_ids[i]] == target_vertices[i] || mTasksList[agent_ids[i]][target_tasks_completed[i]].second.first == target_vertices[i]);
							else
								safe_i = (mTasksList[agent_ids[i]][target_tasks_completed[i]-1].second.second == target_vertices[i] || mTasksList[agent_ids[i]][target_tasks_completed[i]].second.first == target_vertices[i]);
						}
						else
							safe_i = (mTasksList[agent_ids[i]][target_tasks_completed[i]].second.first == target_vertices[i] || mTasksList[agent_ids[i]][target_tasks_completed[i]].second.second == target_vertices[i]);
						
						if(target_in_delivery[j] == false)
						{
							if(target_tasks_completed[j] == 0)
								safe_j = (mStartVertex[agent_ids[j]] == target_vertices[j] || mTasksList[agent_ids[j]][target_tasks_completed[j]].second.first == target_vertices[j]);
							else
								safe_j = (mTasksList[agent_ids[j]][target_tasks_completed[j]-1].second.second == target_vertices[j] || mTasksList[agent_ids[j]][target_tasks_completed[j]].second.first == target_vertices[j]);
						}
						else
							safe_j = (mTasksList[j][target_tasks_completed[j]].second.first == target_vertices[j] || mTasksList[agent_ids[j]][target_tasks_completed[j]].second.second == target_vertices[j]);
						
						if(safe_i == false || safe_j == false)
						{
							count_conflicts++;
							if(!selected_constraint)
							{
								selected_constraint = true;
								PRINT<<"vertex conflict!"<<std::endl;
								if(target_task_ids[i]==-1)
									agent_id_1.push_back(agent_ids[i]);
								else
									for(int k=0; k<num_considered_agents; k++)
										if(target_task_ids[k] == target_task_ids[i])
											agent_id_1.push_back(agent_ids[k]);

								if(target_task_ids[j]==-1)
									agent_id_2.push_back(agent_ids[j]);
								else
									for(int k=0; k<num_considered_agents; k++)
										if(target_task_ids[k] == target_task_ids[j])
											agent_id_2.push_back(agent_ids[k]);

								constraint_1 = CollisionConstraint(target_vertices[i], target_tasks_completed[i], target_in_delivery[i], current_timestep+1);
								constraint_2 = CollisionConstraint(target_vertices[j], target_tasks_completed[j], target_in_delivery[j], current_timestep+1);

								// return true;
							}
						}
					}
				}
				
				if(source_task_ids[i]==-1 || source_task_ids[i]!=source_task_ids[j])
				{
					PRINT<<"Checking between agents: "<<agent_ids[i]<<" "<<agent_ids[j]<<std::endl;
					if(getEdgesCollisionStatus(mGraphs[agent_ids[i]][source_vertices[i]].state, mGraphs[agent_ids[i]][target_vertices[i]].state, mGraphs[agent_ids[j]][source_vertices[j]].state, mGraphs[agent_ids[j]][target_vertices[j]].state))
					{
						bool safe_i = false;
						bool safe_j = false;
						if(target_in_delivery[i] == false)
						{
							if(target_tasks_completed[i] == 0)
								safe_i = ( (target_vertices[i] == source_vertices[i])
									&& ((mStartVertex[agent_ids[i]] == source_vertices[i]) 
										|| (mTasksList[agent_ids[i]][target_tasks_completed[i]].second.first == source_vertices[i])));
							else
								safe_i = ( (target_vertices[i] == source_vertices[i])
									&& ((mTasksList[agent_ids[i]][target_tasks_completed[i]-1].second.second == source_vertices[i]) 
										|| (mTasksList[agent_ids[i]][target_tasks_completed[i]].second.first == source_vertices[i])));
						}
						else
							safe_i = ( (target_vertices[i] == source_vertices[i])
									&& ((mTasksList[agent_ids[i]][target_tasks_completed[i]].second.first == source_vertices[i]) 
										|| (mTasksList[agent_ids[i]][target_tasks_completed[i]].second.second == source_vertices[i])));
						
						if(target_in_delivery[j] == false)
						{
							if(target_tasks_completed[j] == 0)
								safe_j = ( (target_vertices[j] == source_vertices[j])
									&& ((mStartVertex[agent_ids[j]] == source_vertices[j]) 
										|| (mTasksList[agent_ids[j]][target_tasks_completed[j]].second.first == source_vertices[j])));
							else
								safe_j = ( (target_vertices[j] == source_vertices[j])
									&& ((mTasksList[agent_ids[j]][target_tasks_completed[j]-1].second.second == source_vertices[j]) 
										|| (mTasksList[agent_ids[j]][target_tasks_completed[j]].second.first == source_vertices[j])));
						}
						else
							safe_j = ( (target_vertices[j] == source_vertices[j])
									&& ((mTasksList[agent_ids[j]][target_tasks_completed[j]].second.first == source_vertices[j]) 
										|| (mTasksList[agent_ids[j]][target_tasks_completed[j]].second.second == source_vertices[j])));
						
						
						if(safe_i == false || safe_j == false)
						{
							count_conflicts++;
							if(!selected_constraint)
							{
								selected_constraint = true;
								PRINT<<"edge conflict! - ("<<source_vertices[i]<<","<<target_vertices[i]
								<<") ("<<source_vertices[j]<<","<<target_vertices[j]<<")"<<std::endl;
								PRINT<<"agents: "<<agent_ids[i]<<" "<<agent_ids[j]<<std::endl;
								PRINT<<"source_task_ids: "<<source_task_ids[i]<<" "<<source_task_ids[j]<<std::endl;
								if(source_task_ids[i]==-1)
									agent_id_1.push_back(agent_ids[i]);
								else
									for(int k=0; k<num_considered_agents; k++)
										if(source_task_ids[k] == source_task_ids[i])
											agent_id_1.push_back(agent_ids[k]);

								if(source_task_ids[j]==-1)
									agent_id_2.push_back(agent_ids[j]);
								else
									for(int k=0; k<num_considered_agents; k++)
										if(source_task_ids[k] == source_task_ids[j])
											agent_id_2.push_back(agent_ids[k]);

								Edge edge_1 = boost::edge(source_vertices[i],target_vertices[i],mGraphs[agent_ids[i]]).first;
								constraint_1 = CollisionConstraint(edge_1, target_tasks_completed[i], target_in_delivery[i], current_timestep+1);

								Edge edge_2 = boost::edge(source_vertices[j],target_vertices[j],mGraphs[agent_ids[j]]).first;
								constraint_2 = CollisionConstraint(edge_2, target_tasks_completed[j], target_in_delivery[j], current_timestep+1);

								// return true;
							}
						}
					}
				}
			}
			
			current_timestep++;
		}
		return count_conflicts;
	}

	std::vector<std::vector<Eigen::VectorXd>> solve()
	{
		mCBSIterations = 0;
		mSolveStartTime = high_resolution_clock::now();
		auto solve_start = high_resolution_clock::now();
		CBSPriorityQueue PQ(mNumAgents);

		std::vector<std::vector<CollisionConstraint>> collision_constraints(mNumAgents, std::vector<CollisionConstraint>());
		std::vector<std::vector<CollaborationConstraint>> collaboration_constraints(mNumAgents, std::vector<CollaborationConstraint>());
		std::vector<std::vector<CollaborationConstraint>> non_collaboration_constraints(mNumAgents, std::vector<CollaborationConstraint>());
		// for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		// {
		for(int i=0; i<mTasksList.size(); i++)
		{
			for(int j=0; j<mTasksList[i].size(); j++)
			{
				int task_id = mTasksList[i][j].first;
				int task_start_time = mTaskStartTimestep[task_id];
				Vertex pickup_vertex = mTasksList[i][j].second.first;
				for(int timestep=0; timestep<task_start_time; timestep++)
				{
					CollaborationConstraint c = 
						CollaborationConstraint(pickup_vertex, task_id, true,timestep);
					non_collaboration_constraints[i].push_back(c);
				}
			}
		}
		// }
		std::vector<int> start_costs;
		std::vector< std::vector<SearchState> > start_shortestPaths = computeDecoupledPaths(collision_constraints, collaboration_constraints, non_collaboration_constraints, start_costs);

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			if(start_shortestPaths.at(agent_id).size()==0)
			{
				auto solve_stop = high_resolution_clock::now();
				mPlanningTime = (solve_stop - solve_start);
				// std::cout<<"No Path exists for index "<<agent_id<<"! Press [ENTER] to exit: ";
				// std::cin.get();
				std::cout<<0<<" ";
				return std::vector<std::vector<Eigen::VectorXd>>(mNumAgents,std::vector<Eigen::VectorXd>());
			}
		}

		
		
		PQ.insert(start_costs, collision_constraints, collaboration_constraints, non_collaboration_constraints, start_shortestPaths);

		
		int numSearches = 0;
		while(PQ.PQsize()!=0)
		{
			numSearches++;
			mCBSIterations++;

			auto start = high_resolution_clock::now();
			Element p = PQ.pop();
			auto stop = high_resolution_clock::now();
			mQOTime += (stop - start);

			std::chrono::duration<double, std::micro> timespent = stop - mSolveStartTime;

			if (timespent.count() > 200000000)
			{
				auto solve_stop = high_resolution_clock::now();
				mPlanningTime = (solve_stop - mSolveStartTime);
				std::cout<<0<<" ";
				return std::vector<std::vector<Eigen::VectorXd>>(mNumAgents,std::vector<Eigen::VectorXd>());
			}
			

			// 

			int current_makespan = 0;
			for(int i=0; i<p.costs.size(); i++)
				current_makespan = std::max(current_makespan,p.costs[i]);

			// if(numSearches%2 == 0)
			{
				// std::cout<<PQ.PQsize()<<std::endl;
				// std::cerr<<"\n-\nCBS numSearches: "<<numSearches<<" Cost: "<<current_makespan<<std::endl;
				for(int agent_id=0; agent_id<mNumAgents; agent_id++)
				{
					PRINT<<"Collision constraints size: "<<p.collision_constraints[agent_id].size()<<std::endl;
					for(int i=0; i<p.collision_constraints[agent_id].size(); i++)
					{
						if(p.collision_constraints[agent_id][i].constraint_type==1)
							PRINT<<"Vertex constraint: "<<p.collision_constraints[agent_id][i].v<<" "<<p.collision_constraints[agent_id][i].timestep
								<<" "<<p.collision_constraints[agent_id][i].tasks_completed<<" "<<p.collision_constraints[agent_id][i].in_delivery<<std::endl;
						else
							PRINT<<"Edge constraint: "<<p.collision_constraints[agent_id][i].e<<" "<<p.collision_constraints[agent_id][i].timestep
								<<" "<<p.collision_constraints[agent_id][i].tasks_completed<<" "<<p.collision_constraints[agent_id][i].in_delivery<<std::endl;
					}

					PRINT<<"Collaboration constraints size: "<<p.collaboration_constraints[agent_id].size()<<std::endl;
					for(int i=0; i<p.collaboration_constraints[agent_id].size(); i++)
					{
						PRINT<<p.collaboration_constraints[agent_id][i].v<<" "<<p.collaboration_constraints[agent_id][i].timestep
								<<" "<<p.collaboration_constraints[agent_id][i].task_id<<" "<<p.collaboration_constraints[agent_id][i].is_pickup<<std::endl;
					}

					PRINT<<"Non Collaboration constraints size: "<<p.non_collaboration_constraints[agent_id].size()<<std::endl;
					for(int i=0; i<p.non_collaboration_constraints[agent_id].size(); i++)
					{
						PRINT<<p.non_collaboration_constraints[agent_id][i].v<<" "<<p.non_collaboration_constraints[agent_id][i].timestep
								<<" "<<p.non_collaboration_constraints[agent_id][i].task_id<<" "<<p.non_collaboration_constraints[agent_id][i].is_pickup<<std::endl;
					}

					PRINT<<"Path: "<<std::endl;
					for(int i=0; i<p.shortestPaths[agent_id].size(); i++)
						PRINT<<p.shortestPaths[agent_id][i].vertex<<" - ("<<int( (mGraphs[agent_id][p.shortestPaths[agent_id][i].vertex].state[0]+0.001)/mUnitEdgeLength)<<","<<int( (mGraphs[agent_id][p.shortestPaths[agent_id][i].vertex].state[1]+0.001)/mUnitEdgeLength)<<") "<<p.shortestPaths[agent_id][i].timestep
							<<" "<<p.shortestPaths[agent_id][i].tasks_completed<<" "<<p.shortestPaths[agent_id][i].in_delivery<<std::endl;
					PRINT<<std::endl;
				}
				DEBUG std::cin.get();
				// break;
			}

			// if(numSearches == 5000)
			// 	break;

			std::vector<int> all_task_ids;
			for(int tid=0; tid<mTasksToAgentsList.size(); tid++)
				all_task_ids.push_back(tid);

			std::vector<int> all_agent_ids;
			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
				all_agent_ids.push_back(agent_id);

			std::vector<int> collaborating_agent_ids;
			CollaborationConstraint constraint_c;

			if(checkCollaborationConflicts(all_task_ids, all_agent_ids, p.shortestPaths, collaborating_agent_ids, constraint_c) != 0)
			{
				{
					std::vector<std::vector<CollaborationConstraint>> increase_constraints_c = p.collaboration_constraints;
					std::vector<int> costs_c = p.costs;
					std::vector< std::vector<SearchState> > shortestPaths_c = p.shortestPaths;

					bool all_paths_exist = true;

					std::vector<int> consider_agents;
					for(int agent_id=0; agent_id<mNumAgents; agent_id++)
						if(std::find(collaborating_agent_ids.begin(),collaborating_agent_ids.end(), agent_id) == collaborating_agent_ids.end())
							consider_agents.push_back(agent_id);

					for(int i=0; i<collaborating_agent_ids.size(); i++)
					{
						// make sure that collab and not collab do not share a constraint
						bool allowed = true;
						for( CollaborationConstraint &c: p.non_collaboration_constraints[collaborating_agent_ids[i]])
						{
							if( constraint_c.v == c.v && constraint_c.task_id == c.task_id
								&& constraint_c.is_pickup == c.is_pickup==true && constraint_c.timestep == c.timestep)
							{
								// std::cout<<"Non collaboration Constraint Encountered! "<<std::endl;
								allowed = false;
								break;
							}
						}
						if(!allowed)
						{
							all_paths_exist = false;
							break;
						}	
						increase_constraints_c[collaborating_agent_ids[i]].push_back(constraint_c);
						int cost_c;

						shortestPaths_c[collaborating_agent_ids[i]] = computeShortestPath(collaborating_agent_ids[i], p.collision_constraints[collaborating_agent_ids[i]], 
								increase_constraints_c[collaborating_agent_ids[i]], p.non_collaboration_constraints[collaborating_agent_ids[i]], cost_c, current_makespan, p.shortestPaths,consider_agents);
						costs_c[collaborating_agent_ids[i]] = cost_c;
						if(cost_c == INF)
						{
							all_paths_exist = false;
							break;
						}

						consider_agents.push_back(collaborating_agent_ids[i]);
					}

					if(all_paths_exist)
					{
						// std::cout<<"inserting left!"<<std::endl;
						PQ.insert(costs_c, p.collision_constraints, increase_constraints_c,
								p.non_collaboration_constraints, shortestPaths_c);
					}
				}
				{
					std::vector<std::vector<CollaborationConstraint>> increase_constraints_c = p.non_collaboration_constraints;

					std::vector< int> costs_c = p.costs;
					std::vector< std::vector<SearchState> > shortestPaths_c = p.shortestPaths;

					std::vector<int> consider_agents;
					for(int agent_id=0; agent_id<mNumAgents; agent_id++)
						if(std::find(collaborating_agent_ids.begin(),collaborating_agent_ids.end(), agent_id) == collaborating_agent_ids.end())
							consider_agents.push_back(agent_id);

					bool all_paths_exist = true;

					for(int i=0; i<collaborating_agent_ids.size(); i++)
					{
						// make sure that collab and not collab do not share a constraint
						bool allowed = true;
						for( CollaborationConstraint &c: p.collaboration_constraints[collaborating_agent_ids[i]])
						{
							if( constraint_c.v == c.v && constraint_c.task_id == c.task_id
								&& constraint_c.is_pickup == c.is_pickup==true && constraint_c.timestep == c.timestep)
							{
								// std::cout<<"Non collaboration Constraint Encountered! "<<std::endl;
								allowed = false;
								break;
							}
						}
						if(!allowed)
						{
							all_paths_exist = false;
							break;
						}
						increase_constraints_c[collaborating_agent_ids[i]].push_back(constraint_c);
						int cost_c;
						shortestPaths_c[collaborating_agent_ids[i]] = computeShortestPath(collaborating_agent_ids[i], p.collision_constraints[collaborating_agent_ids[i]], 
								p.collaboration_constraints[collaborating_agent_ids[i]], increase_constraints_c[collaborating_agent_ids[i]], cost_c,  current_makespan, p.shortestPaths,consider_agents);
						costs_c[collaborating_agent_ids[i]] = cost_c;
						if(cost_c == INF)
						{
							all_paths_exist = false;
							break;
						}

						consider_agents.push_back(collaborating_agent_ids[i]);
					}

					if(all_paths_exist)
					{
						PQ.insert(costs_c, p.collision_constraints, p.collaboration_constraints,
								increase_constraints_c, shortestPaths_c);
					}

				}
				continue;
			}

			PRINT<<"NO COLLABORATION CONSTRAINT!!"<<std::endl;

			std::vector<int> agent_id_1;
			CollisionConstraint constraint_1;

			std::vector<int> agent_id_2;
			CollisionConstraint constraint_2;

			// 

			// std::cout<<"Calling CollaborationConstraint!"<<std::endl;

			if(checkCollisionConflicts(all_agent_ids, p.shortestPaths, agent_id_1, constraint_1, agent_id_2, constraint_2) != 0)
			{
				//agent_id_1

				PRINT<<"Collision Conflict found between:\n { ";
				for(int i=0; i<agent_id_1.size();i++)
					PRINT<<agent_id_1[i]<<" "; 
				if(constraint_1.constraint_type==1)
					PRINT<<"} at vertex: "<<constraint_1.v<<" at timestep: "<<constraint_1.timestep<<std::endl;
				else
					PRINT<<"} at edge: "<<constraint_1.v<<" at timestep: "<<constraint_1.timestep<<std::endl;

				PRINT<<" { ";
				for(int i=0; i<agent_id_2.size();i++)
					PRINT<<agent_id_2[i]<<" ";
				if(constraint_2.constraint_type==1)
					PRINT<<"} at vertex: "<<constraint_2.v<<" at timestep: "<<constraint_2.timestep<<std::endl;
				else
					PRINT<<"} at edge: "<<constraint_2.v<<" at timestep: "<<constraint_2.timestep<<std::endl;

				

				// std::cout<<"In CollaborationConstraint!"<<std::endl;

				std::vector<std::vector<CollisionConstraint>> increase_constraints_agent_id_1 = p.collision_constraints;
				std::vector< int> costs_agent_id_1 = p.costs;
				std::vector< std::vector<SearchState> > shortestPaths_agent_id_1 = p.shortestPaths;

				std::vector<int> consider_agents;
					for(int agent_id=0; agent_id<mNumAgents; agent_id++)
						if(std::find(agent_id_1.begin(),agent_id_1.end(), agent_id) == agent_id_1.end())
							consider_agents.push_back(agent_id);

				bool all_paths_exist = true;

				for(int i=0; i<agent_id_1.size(); i++)
				{
					increase_constraints_agent_id_1[agent_id_1[i]].push_back(constraint_1);
					int cost_agent_id_1;
					shortestPaths_agent_id_1[agent_id_1[i]] = computeShortestPath(agent_id_1[i], increase_constraints_agent_id_1[agent_id_1[i]], 
						p.collaboration_constraints[agent_id_1[i]], p.non_collaboration_constraints[agent_id_1[i]], cost_agent_id_1,
						current_makespan, p.shortestPaths,consider_agents);
					costs_agent_id_1[agent_id_1[i]] = cost_agent_id_1;

					if(cost_agent_id_1 == INF)
					{
						all_paths_exist = false;
						break;
					}
					consider_agents.push_back(agent_id_1[i]);
				}

				if(all_paths_exist)
				{
					auto start1 = high_resolution_clock::now();
					// std::cout<<"inserting left!"<<std::endl;
					PQ.insert(costs_agent_id_1,increase_constraints_agent_id_1, p.collaboration_constraints, 
						p.non_collaboration_constraints, shortestPaths_agent_id_1);
					auto stop1 = high_resolution_clock::now();
					mQOTime += (stop1 - start1);
				}

				// 
				
				//agent_id_2

				std::vector<std::vector<CollisionConstraint>> increase_constraints_agent_id_2 = p.collision_constraints;
				std::vector< int> costs_agent_id_2 = p.costs;
				std::vector< std::vector<SearchState> > shortestPaths_agent_id_2 = p.shortestPaths;

				consider_agents.clear();
					for(int agent_id=0; agent_id<mNumAgents; agent_id++)
						if(std::find(agent_id_2.begin(),agent_id_2.end(), agent_id) == agent_id_2.end())
							consider_agents.push_back(agent_id);

				all_paths_exist = true;

				for(int i=0; i<agent_id_2.size(); i++)
				{
					increase_constraints_agent_id_2[agent_id_2[i]].push_back(constraint_2);
					int cost_agent_id_2;
					shortestPaths_agent_id_2[agent_id_2[i]] = computeShortestPath(agent_id_2[i], increase_constraints_agent_id_2[agent_id_2[i]], 
						p.collaboration_constraints[agent_id_2[i]], p.non_collaboration_constraints[agent_id_2[i]], cost_agent_id_2,
						current_makespan, p.shortestPaths,consider_agents);
					costs_agent_id_2[agent_id_2[i]] = cost_agent_id_2;
					
					if(cost_agent_id_2 == INF)
					{
						all_paths_exist = false;
						break;
					}

					consider_agents.push_back(agent_id_1[i]);
				}

				// std::cout<<"Agent id 2: "<<agent_id_2<<std::endl;
				if(all_paths_exist)
				{
					auto start2 = high_resolution_clock::now();
					// std::cout<<"inserting right!"<<std::endl;
					PQ.insert(costs_agent_id_2,increase_constraints_agent_id_2, p.collaboration_constraints, 
						p.non_collaboration_constraints, shortestPaths_agent_id_2);
					auto stop2 = high_resolution_clock::now();
					mQOTime += (stop2 - start2);
				}

				continue;
			} 

			// std::cout<<"Out CollaborationConstraint!"<<std::endl;

			// {
			// 	// std::cout<<PQ.PQsize()<<std::endl;
			// 	std::cout<<"\n-\nCBS numSearches: "<<numSearches<<" Cost: "<<int((total_cost+0.0001)/mUnitEdgeLength)<<std::endl;
			// 	for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			// 	{
			// 		std::cout<<"Collision constraints size: "<<p.collision_constraints[agent_id].size()<<std::endl;
			// 		for(int i=0; i<p.collision_constraints[agent_id].size(); i++)
			// 		{
			// 			if(p.collision_constraints[agent_id][i].constraint_type==1)
			// 				std::cout<<"Vertex constraint: "<<p.collision_constraints[agent_id][i].v<<" "<<p.collision_constraints[agent_id][i].timestep
			// 					<<" "<<p.collision_constraints[agent_id][i].tasks_completed<<" "<<p.collision_constraints[agent_id][i].in_delivery<<std::endl;
			// 			else
			// 				std::cout<<"Edge constraint: "<<p.collision_constraints[agent_id][i].e<<" "<<p.collision_constraints[agent_id][i].timestep
			// 					<<" "<<p.collision_constraints[agent_id][i].tasks_completed<<" "<<p.collision_constraints[agent_id][i].in_delivery<<std::endl;
			// 		}

			// 		std::cout<<"Path: "<<std::endl;
			// 		for(int i=0; i<p.shortestPaths[agent_id].size(); i++)
			// 			std::cout<<p.shortestPaths[agent_id][i].vertex<<" - ("<<int( (mGraphs[agent_id][p.shortestPaths[agent_id][i].vertex].state[0]+0.001)/mUnitEdgeLength)<<","<<int( (mGraphs[agent_id][p.shortestPaths[agent_id][i].vertex].state[1]+0.001)/mUnitEdgeLength)<<") "<<p.shortestPaths[agent_id][i].timestep
			// 				<<" "<<p.shortestPaths[agent_id][i].tasks_completed<<" "<<p.shortestPaths[agent_id][i].in_delivery<<std::endl;
			// 		std::cout<<std::endl;
			// 	}
			// 	// std::cin.get();
			// 	// break;
			// }

			std::vector<std::vector<Eigen::VectorXd>> collision_free_path(mNumAgents, std::vector<Eigen::VectorXd>());

			std::vector<std::vector<SearchState>> paths = p.shortestPaths;
			int current_timestep = 0;
			int maximum_timestep = 0;
			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
				maximum_timestep = std::max(maximum_timestep, paths[agent_id].at(paths[agent_id].size()-1).timestep);
			// std::cout<<"MT: "<<maximum_timestep<<std::endl;
			std::vector<int> current_path_id(mNumAgents, 0);
			while(current_timestep <= maximum_timestep)
			{
				for(int agent_id=0; agent_id<mNumAgents; agent_id++)
				{
					if(current_path_id[agent_id] == paths[agent_id].size())
					{
						collision_free_path[agent_id].push_back(mGraphs[agent_id][paths[agent_id].at(paths[agent_id].size()-1).vertex].state);
						PRINT<<paths[agent_id].at(paths[agent_id].size()-1).vertex<<" ";
					}
					else
					{
						collision_free_path[agent_id].push_back(mGraphs[agent_id][paths[agent_id].at(current_path_id[agent_id]).vertex].state);
						PRINT<<paths[agent_id].at(current_path_id[agent_id]).vertex<<" ";
						while(current_path_id[agent_id] < paths[agent_id].size()
							&& paths[agent_id].at(current_path_id[agent_id]).timestep == current_timestep)
							current_path_id[agent_id]++;
					}
				}
				PRINT<<std::endl;
				current_timestep++;
			}

			// std::cout<<" Path Cost: "<<total_cost<<std::endl;
			// for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			// {
			// 	std::cout<<"Shortest Path Cost for index - "<<agent_id<<" : "<<p.costs[agent_id]<<std::endl;
			// 	std::cout<<"Shortest Path for index - "<<agent_id<<" : ";
			// 	for(SearchState &nodes: p.shortestPaths[agent_id])
			// 	{
			// 		std::cout<<mGraphs[agent_id][nodes.vertex].vertex_index<<" "<<mGraphs[agent_id][nodes.vertex].state<<" ";
			// 		collision_free_path[agent_id].push_back(mGraphs[agent_id][nodes.vertex].state);
			// 	}
			// 	std::cout<<std::endl;
			// }

			// PRINT<<"POPULATING PATH CONFIGS!"<<std::endl;

			std::vector<Eigen::VectorXd> path_configs;

			for(int i=0; i<collision_free_path[0].size(); i++)
			{
				Eigen::VectorXd config(collision_free_path.size()*2);
				for(int j=0; j<collision_free_path.size(); j++)
				{
					config[2*j]=collision_free_path[j][i][0];
					config[2*j+1]=collision_free_path[j][i][1];
				}
				// std::cout<<config[0]<<"-"<<config[1]<<" "<<config[2]<<"-"<<config[3]<<" "<<config[4]<<"-"<<config[5]<<std::endl;
				path_configs.push_back(config);
			}

			std::cout<<path_configs.size()<<" ";

			auto solve_stop = high_resolution_clock::now();
			mPlanningTime = (solve_stop - solve_start);

			// std::cout<<"CBS numSearches: "<<numSearches<<std::endl;
				

			// std::cout<<"Press [ENTER] to display path: ";
			// std::cin.get();
			// displayPath(path_configs);
			// printStats();
			return collision_free_path;
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

		

		// for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		// {
		// 	VertexIter vi, vi_end;
		// 	for (boost::tie(vi, vi_end) = vertices(mGraphs[agent_id]); vi != vi_end; ++vi)
		// 	{
		// 		double x_point = mGraphs[agent_id][*vi].state[0]*numberOfColumns;
		// 		double y_point = (1 - mGraphs[agent_id][*vi].state[1])*numberOfRows;
		// 		cv::Point centre_Point((int)x_point, (int)y_point);
		// 		cv::circle(image, centre_Point, 4,  cv::Scalar(0, 150, 0), -1);
		// 	}
		// } 

		

		std::vector< std::pair<std::pair<int,int>, std::pair<int,int>> >  tasks;
		for(int tid=0; tid<mTasksToAgentsList.size(); tid++)
		{
			int start_x = int( (mGraphs[0][mTasksList[mTasksToAgentsList[tid][0].first][mTasksToAgentsList[tid][0].second].second.first].state[0]+0.0001)/mUnitEdgeLength);
			int start_y = int( (mGraphs[0][mTasksList[mTasksToAgentsList[tid][0].first][mTasksToAgentsList[tid][0].second].second.first].state[1]+0.0001)/mUnitEdgeLength);

			int goal_x = int( (mGraphs[0][mTasksList[mTasksToAgentsList[tid][0].first][mTasksToAgentsList[tid][0].second].second.second].state[0]+0.0001)/mUnitEdgeLength);
			int goal_y = int( (mGraphs[0][mTasksList[mTasksToAgentsList[tid][0].first][mTasksToAgentsList[tid][0].second].second.second].state[1]+0.0001)/mUnitEdgeLength);

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
				cv::waitKey(100);
				if(firstTime)
				{
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
				cv::waitKey(100);
			}
		}
		cv::namedWindow("Graph Visualization",cv::WINDOW_NORMAL);
		cv::imshow("Graph Visualization", image);
		cv::waitKey(0);
	}

	std::vector<SearchState> computeShortestPath(int &agent_id, std::vector<CollisionConstraint> &collision_constraints,
		std::vector<CollaborationConstraint> &collaboration_constraints, std::vector<CollaborationConstraint> &non_collaboration_constraints, 
		int& costOut, int &current_makespan, std::vector<std::vector<SearchState> > &shortestPaths, std::vector<int> &consider_agents)
	{
		costOut = 0;
		std::vector<SearchState> path;
		SearchState start = SearchState(mStartVertex[agent_id], 0, 0, 0);

		boost::unordered_map <std::pair <int, SearchState>, int, time_state_hash> nonCollabMap;

		for(CollaborationConstraint &c: non_collaboration_constraints)
		{
			SearchState state = SearchState(c.v,c.timestep,c.task_id,c.is_pickup);
			nonCollabMap[std::make_pair(c.timestep, state)] = 1;
		}

		std::vector <SearchState> wayPoints;
		for( CollaborationConstraint &c: collaboration_constraints)
		{
			int collaboration_tasks_completed=-1;
			for(int i=0; i<mTasksList[agent_id].size(); i++)
				if(c.task_id == mTasksList[agent_id][i].first)
					collaboration_tasks_completed = i;
			if(collaboration_tasks_completed==-1){
				std::cout << "TASK NOT FOUND\n";
				std::cin.get();
			}

			// if (c.is_pickup) collaboration_tasks_completed += 1;
			SearchState state;
			if(c.is_pickup == true)
				state = SearchState(c.v,c.timestep,collaboration_tasks_completed,false); // in Delivery false
			else
				state = SearchState(c.v,c.timestep,collaboration_tasks_completed,true);

			wayPoints.push_back(state);
		}

		std::sort(wayPoints.begin(), wayPoints.end(), compareCollaborationConstraints);

		int segmentCost;
		int startTimestep = 0;
		int tasks_completed = 0;

		for(auto constraintState: wayPoints){
			SearchState goal = constraintState;
			
			std::vector<SearchState> pathSegment = computeShortestPathSegment(agent_id, collision_constraints, start, goal,
				non_collaboration_constraints, segmentCost, startTimestep, constraintState.timestep, nonCollabMap, current_makespan, shortestPaths, consider_agents);
			costOut += segmentCost;
			if (pathSegment.size()==0) return std::vector<SearchState>();

			if (goal.in_delivery == true) 
				start = SearchState(goal.vertex,goal.timestep,goal.tasks_completed+1,false);
			else
				start = SearchState(goal.vertex,goal.timestep,goal.tasks_completed,true);
			
			startTimestep = constraintState.timestep;
			for (auto s:pathSegment)
				path.push_back(s);
		}

		SearchState goal = SearchState();
		std::vector<SearchState> pathSegment = computeShortestPathSegment(agent_id, collision_constraints, start, goal,
				non_collaboration_constraints, segmentCost, startTimestep, -1, nonCollabMap, current_makespan, shortestPaths, consider_agents);

		costOut += segmentCost;
		if (pathSegment.size()==0) return std::vector<SearchState>();

		// start = goal;

		
		for (auto s:pathSegment)
			path.push_back(s);

		return path;
	}

	int countCollaborationConflicts(int &agent_id, SearchState &state, SearchState &new_state, std::vector<std::vector<SearchState> > &paths, std::vector<int> &consider_agents)
	{
		if(state.timestep != new_state.timestep)
		{
			std::cout<<"[ERROR]: on pickup/delivery action! ";
			std::cin.get();
		}
		if(state.tasks_completed != new_state.tasks_completed) // delivery state
		{
			int delivery_taskid = mTasksList[agent_id][state.tasks_completed].first;
			std::set<int> delivery_timesteps;
			delivery_timesteps.insert(state.timestep);
			for(int i=0; i<consider_agents.size(); i++)
				for(int j=0; j<mTasksToAgentsList[delivery_taskid].size(); j++)
					if(mTasksToAgentsList[delivery_taskid][j].first == consider_agents[i])
					{
						int agent_id = mTasksToAgentsList[delivery_taskid][j].first;
						int agent_index = consider_agents[i];
						int task_index = mTasksToAgentsList[delivery_taskid][j].second;
						for(int k=1; k<paths[agent_index].size(); k++)
							if(paths[agent_index][k].tasks_completed == task_index+1 && paths[agent_index][k].in_delivery == false
								&& paths[agent_index][k-1].in_delivery == true)
								delivery_timesteps.insert(paths[agent_index][k].timestep);
					}
			return delivery_timesteps.size() - 1;
		}
		else // pickup state
		{
			int pickup_taskid = mTasksList[agent_id][state.tasks_completed].first;
			std::set<int> pickup_timesteps;
			pickup_timesteps.insert(state.timestep);
			for(int i=0; i<consider_agents.size(); i++)
				for(int j=0; j<mTasksToAgentsList[pickup_taskid].size(); j++)
					if(mTasksToAgentsList[pickup_taskid][j].first == consider_agents[i])
					{
						int agent_id = mTasksToAgentsList[pickup_taskid][j].first;
						int agent_index = consider_agents[i];
						int task_index = mTasksToAgentsList[pickup_taskid][j].second;
						for(int k=1; k<paths[agent_index].size(); k++)
							if(paths[agent_index][i].tasks_completed == task_index && paths[agent_index][i].in_delivery == true
								&& paths[agent_index][i-1].in_delivery == false)
								pickup_timesteps.insert(paths[agent_index][i].timestep);
					}
			return pickup_timesteps.size() - 1;
		}
	}

	int countCollisionConflicts(int &agent_id, SearchState &state, SearchState &new_state, std::vector<std::vector<SearchState> > &paths, std::vector<int> &consider_agents)
	{
		if(state.timestep + 1 != new_state.timestep)
		{
			std::cout<<"[ERROR]: on increase timestep action! ";
			std::cin.get();
		}
		return 0;
		// int count_conflicts = 0;
		// for(int i=0; i<consider_agents.size(); i++)
		// {
		// 	std::vector<int> agent_ids;
		// 	std::vector<std::vector<SearchState>> paths;

		// 	agent_ids.push_back(agent_id);
		// 	paths.push_back(currentPath);

		// 	agent_ids.push_back(consider_agents[i]);
		// 	paths.push_back(shortestPaths[consider_agents[i]]);

		// 	std::vector<int> agent_id_1;
		// 	CollisionConstraint constraint_1;

		// 	std::vector<int> agent_id_2;
		// 	CollisionConstraint constraint_2;


		// 	count_conflicts += checkCollisionConflicts(agent_ids, paths, agent_id_1, constraint_1, agent_id_2, constraint_2);
		// }
		// return count_conflicts;
	}

	std::vector<int> getHeuristics(int &agent_id, SearchState &state, int g_value, int &current_makespan,  
		int count_collaboration_conflicts, int count_collision_conflicts, int count_move_actions)
	{
		int h_value=0;
		if(state.in_delivery == true)
			h_value += mAllPairsShortestPathMap[std::make_pair(state.vertex, mTasksList[agent_id][state.tasks_completed].second.second)];
		else
		{
			h_value += mAllPairsShortestPathMap[std::make_pair(state.vertex, mTasksList[agent_id][state.tasks_completed].second.first)];
			h_value += mAllPairsShortestPathMap[std::make_pair(mTasksList[agent_id][state.tasks_completed].second.first, mTasksList[agent_id][state.tasks_completed].second.second)];
		}
		for(int i=state.tasks_completed+1; i<mTasksList[agent_id].size(); i++)
		{
			h_value += mAllPairsShortestPathMap[std::make_pair(mTasksList[agent_id][i-1].second.second,mTasksList[agent_id][i].second.first)];
			h_value += mAllPairsShortestPathMap[std::make_pair(mTasksList[agent_id][i].second.first,mTasksList[agent_id][i].second.second)];
		}

		std::vector<int> heuristics;
		heuristics.push_back(std::max(0, g_value + h_value - current_makespan));
		// heuristics.push_back(count_collaboration_conflicts);
		// heuristics.push_back(count_collision_conflicts);
		// heuristics.push_back(count_move_actions+h_value);
		// heuristics.push_back(h_value);
		// heuristics.push_back(g_value + h_value);
		return heuristics;
	}

	std::vector<SearchState> computeShortestPathSegment(int &agent_id, std::vector<CollisionConstraint> &collision_constraints,
		SearchState &start_state, SearchState &goal_state, std::vector<CollaborationConstraint> &non_collaboration_constraints, int& costOut, 
		int start_timestep, int collaboration_timestep, boost::unordered_map <std::pair <int, SearchState>, int, time_state_hash> nonCollabMap,
		int &current_makespan, std::vector<std::vector<SearchState> > &shortestPaths, std::vector<int> &consider_agents)
	{
		auto start1 = high_resolution_clock::now();

		int min_goal_timestep = 0;
		bool lastSegment = false;

		if(collaboration_timestep==-1){
			lastSegment = true;
			for( CollisionConstraint &c: collision_constraints)
			{
				min_goal_timestep = std::max(min_goal_timestep, c.timestep);
			}
		}

		timePriorityQueue pq;
		boost::unordered_map<SearchState, int, state_hash> mGValue;
		boost::unordered_map<SearchState, int, state_hash> mCountCollaborationConflicts;
		boost::unordered_map<SearchState, int, state_hash> mCountCollisionConflicts;
		boost::unordered_map<SearchState, int, state_hash> mCountMoveActions;
		boost::unordered_map<SearchState, std::vector<int>, state_hash> mFValue;
		boost::unordered_map<SearchState , SearchState, state_hash > mPrev;

		// SearchState start_state = SearchState(start,start_timestep,tasks_completed,0);

		mGValue[start_state]=0;
		mCountCollaborationConflicts[start_state]=0;
		mCountCollisionConflicts[start_state]=0;
		mCountMoveActions[start_state]=0;
		mFValue[start_state]=getHeuristics(agent_id, start_state, 0, current_makespan, 0, 0, 0);
		pq.insert(start_state,mFValue[start_state]);
		
		int numSearches = 0;
		// int maximum_timestep = 10000;

		int goal_timestep = -1;

		// SearchState goal_state = SearchState();

		costOut = INF;

		while(pq.PQsize()!=0)
		{
			
			numSearches++;
			// std::cout<<"Queue pop no: "<<numSearches<<std::endl;
			SearchState current_state = pq.pop();
			Vertex current_vertex = current_state.vertex;
			int current_timestep = current_state.timestep;
			int current_tasks_completed = current_state.tasks_completed;
			bool current_in_delivery = current_state.in_delivery;

			std::vector<int> current_fvalue = mFValue[current_state];
			int current_gvalue = mGValue[current_state];
			int current_count_collaboration_conflicts = mCountCollaborationConflicts[current_state];
			int current_count_collision_conflicts = mCountCollisionConflicts[current_state];
			int current_count_move_actions = mCountMoveActions[current_state];

			// if(current_timestep>100)
			// 	continue;

			auto stop = high_resolution_clock::now();
			std::chrono::duration<double, std::micro> timespent = stop - mSolveStartTime;

			if (timespent.count() > 200000000)
			{
				auto solve_stop = high_resolution_clock::now();
				mPlanningTime = (solve_stop - mSolveStartTime);
				costOut == INF;
				auto stop1 = high_resolution_clock::now();
				mCSPTime += (stop1 - start1);
				return std::vector<SearchState>();
			}

			// if(numSearches%10000 == 0)
			// {
			// 	std::cout<<"numSearches: "<<numSearches<<std::endl;
			// 	std::cout<<" ("<<int( (mGraphs[agent_id][current_vertex].state[0]+0.001)/mUnitEdgeLength)<<", "
			// <<int( (mGraphs[agent_id][current_vertex].state[1]+0.001)/mUnitEdgeLength)<<")"<<std::endl;
			// 	std::cout<<current_timestep<<" "<<current_tasks_completed<<" "<<current_in_delivery<<std::endl;
			// }

			
			if(lastSegment){
				if(current_tasks_completed == mTasksList[agent_id].size() && current_timestep>= min_goal_timestep)
				{
					// std::cout<<"Timestep goal was found: "<<final_timestep<<std::endl;
					costOut = current_gvalue;
					goal_state = current_state;
					break;
				}
				if(current_tasks_completed == mTasksList[agent_id].size())
				{
					continue;
				}
			}

			else{
				if(current_state==goal_state){
					costOut = current_gvalue;
					break;
				}
				if(current_timestep > collaboration_timestep || current_tasks_completed > goal_state.tasks_completed) {
					continue;
				}
			}
			
			if(mSpecialPosition[agent_id].count(current_vertex)!= 0)
			{
				if(!current_in_delivery && mTasksList[agent_id][current_tasks_completed].second.first == current_vertex) //pickup point
				{
					bool allowed = true;
					// for( CollaborationConstraint &c: non_collaboration_constraints)
					// {
					// 	if( current_vertex == c.v && mTasksList[agent_id][current_tasks_completed].first == c.task_id
					// 		&& c.is_pickup==true && c.timestep == current_timestep) //pickup object is not allowed at this timestep
					// 	{
					// 		// std::cout<<"Non collaboration Constraint Encountered! "<<std::endl;
					// 		allowed = false;
					// 		break;
					// 	}
					// }
					SearchState key_state = SearchState(current_vertex, current_timestep, mTasksList[agent_id][current_tasks_completed].first, true);
					std::pair <int, SearchState> key = std::make_pair(current_timestep, key_state);
					if(nonCollabMap.find(key)!=nonCollabMap.end()) allowed = false;
					if(allowed)
					{
						SearchState new_state = SearchState(current_vertex, current_timestep, current_tasks_completed, true);
						
						int new_count_collaboration_conflicts = current_count_collaboration_conflicts + countCollaborationConflicts(agent_id,current_state,new_state,shortestPaths,consider_agents);
						std::vector<int> new_cost = getHeuristics(agent_id, new_state, current_gvalue, current_makespan, new_count_collaboration_conflicts, current_count_collision_conflicts, current_count_move_actions);
						
						if(mFValue.count(new_state)==0 || new_cost < mFValue[new_state])
						{
							mGValue[new_state]= current_gvalue;
							mCountCollaborationConflicts[new_state]=new_count_collaboration_conflicts;
							mCountCollisionConflicts[new_state]=current_count_collision_conflicts;
							mCountMoveActions[new_state]=current_count_move_actions;
							mFValue[new_state]= new_cost;
							mPrev[new_state]=current_state;
							pq.insert(new_state, new_cost);
						}
					}
				}
				if(current_in_delivery && mTasksList[agent_id][current_tasks_completed].second.second == current_vertex) //delivery point
				{
					bool allowed = true;
					// for( CollaborationConstraint &c: non_collaboration_constraints)
					// {
					// 	if( current_vertex == c.v && mTasksList[agent_id][current_tasks_completed].first == c.task_id
					// 		&& c.is_pickup==false && c.timestep == current_timestep) //pickup object is not allowed at this timestep
					// 	{
					// 		// std::cout<<"Non collaboration Constraint Encountered! "<<std::endl;
					// 		allowed = false;
					// 		break;
					// 	}
					// }
					SearchState key_state = SearchState(current_vertex, current_timestep, mTasksList[agent_id][current_tasks_completed].first, false);
					std::pair <int, SearchState> key = std::make_pair(current_timestep, key_state);
					if(nonCollabMap.find(key)!=nonCollabMap.end()) allowed = false;
					if(allowed)
					{
						SearchState new_state= SearchState(current_vertex, current_timestep, current_tasks_completed+1, false);
						
						int new_count_collaboration_conflicts = current_count_collaboration_conflicts + countCollaborationConflicts(agent_id,current_state,new_state,shortestPaths,consider_agents);
						std::vector<int> new_cost = getHeuristics(agent_id, new_state, current_gvalue, current_makespan, new_count_collaboration_conflicts, current_count_collision_conflicts, current_count_move_actions);
						
						if(mFValue.count(new_state)==0 || new_cost < mFValue[new_state])
						{
							mGValue[new_state]= current_gvalue;
							mCountCollaborationConflicts[new_state]=new_count_collaboration_conflicts;
							mCountCollisionConflicts[new_state]=current_count_collision_conflicts;
							mCountMoveActions[new_state]=current_count_move_actions;
							mFValue[new_state]= new_cost;
							mPrev[new_state]=current_state;
							pq.insert(new_state, new_cost);
						}
					}
				}
			}

			

			{
				auto start2 = high_resolution_clock::now();
				bool col = false;
				for( CollisionConstraint &c: collision_constraints)
				{
					if( c.constraint_type == 1 && current_vertex == c.v 
						&& current_tasks_completed == c.tasks_completed && current_in_delivery == c.in_delivery
					 	&& c.timestep == current_timestep + 1)
					{
						// std::cout<<"CollisionConstraint Encountered! "<<std::endl;
						col =true;
						break;
					}
				}
				auto stop2 = high_resolution_clock::now();
				mCCTime += (stop2 - start2);

				if(!col)
				{
					SearchState new_state = SearchState(current_vertex, current_timestep+1, current_tasks_completed, current_in_delivery);
					
					int new_count_collision_conflicts = current_count_collision_conflicts + countCollisionConflicts(agent_id,current_state,new_state,shortestPaths,consider_agents);
					std::vector<int> new_cost = getHeuristics(agent_id, new_state, current_gvalue+1, current_makespan, current_count_collaboration_conflicts, new_count_collision_conflicts, current_count_move_actions);
						
					if(mFValue.count(new_state)==0 || new_cost < mFValue[new_state])
					{
						mGValue[new_state]= current_gvalue+1;
						mCountCollaborationConflicts[new_state]=current_count_collaboration_conflicts;
						mCountCollisionConflicts[new_state]=new_count_collision_conflicts;
						mCountMoveActions[new_state]=current_count_move_actions;
						mFValue[new_state]= new_cost;
						mPrev[new_state]=current_state;
						pq.insert(new_state, new_cost);						
					}
				}	
			}

			

			std::vector<Vertex> neighbors = getNeighbors(mGraphs[agent_id],current_vertex);
			
			// std::cout<<"No. of neighbors :"<<neighbors.size()<<std::endl;

			for (auto &successor : neighbors) 
			{
				Edge uv_edge = boost::edge(current_vertex, successor, mGraphs[agent_id]).first;

				auto start2 = high_resolution_clock::now();
				bool col = false;
				for( CollisionConstraint c: collision_constraints)
				{
					if( (c.constraint_type == 1 && successor == c.v 
						&& current_tasks_completed == c.tasks_completed && current_in_delivery == c.in_delivery
					 	&& c.timestep == current_timestep + 1) 
						|| (c.constraint_type == 2 && uv_edge == c.e 
							&& current_tasks_completed == c.tasks_completed && current_in_delivery == c.in_delivery
					 		&& c.timestep == current_timestep + 1) )
					{
						// std::cout<<"CollisionConstraint Encountered! "<<std::endl;
						col =true;
						break;
					}
				}
				auto stop2 = high_resolution_clock::now();
				mCCTime += (stop2 - start2);

				if(!col)
				{       
					SearchState new_state = SearchState(successor, current_timestep+1, current_tasks_completed, current_in_delivery);
					
					int new_count_collision_conflicts = current_count_collision_conflicts + countCollisionConflicts(agent_id,current_state,new_state,shortestPaths,consider_agents);
					std::vector<int> new_cost = getHeuristics(agent_id, new_state, current_gvalue+1, current_makespan, current_count_collaboration_conflicts, new_count_collision_conflicts, current_count_move_actions+1);
					
					if(mFValue.count(new_state)==0 || new_cost < mFValue[new_state])
					{
						mGValue[new_state]= current_gvalue+1;
						mCountCollaborationConflicts[new_state]=current_count_collaboration_conflicts;
						mCountCollisionConflicts[new_state]=new_count_collision_conflicts;
						mCountMoveActions[new_state]=current_count_move_actions+1;
						mFValue[new_state]= new_cost;
						mPrev[new_state]=current_state;
						pq.insert(new_state, new_cost);
					}
				}
			}
		}

		if(costOut == INF)
		{
			// PRINT<<"no path!"<<std::endl;
			auto stop1 = high_resolution_clock::now();
			mCSPTime += (stop1 - start1);
			return std::vector<SearchState>();
		}

		// std::cout<<"Goal Time: "<<goal_timestep<<std::endl;
		std::vector<SearchState> finalPath;
		SearchState node = goal_state;

		// std::cout<<"timesteps: ";
		while(!(node == start_state))
		{
			// std::cin.get();
			// std::cout<<"INF LOOP LOL!";
			// std::cout<<goal_timestep<<" ";
			finalPath.push_back(node);
			node=mPrev[node];
		}
		// std::cout<<std::endl;
		finalPath.push_back(start_state);
		std::reverse(finalPath.begin(), finalPath.end());

		// std::cout<<"ST: "<<initial_timestep<<" GT: "<<final_timestep<<std::endl;

		// std::cout<<"Path: ";
		// for(int i=0; i<finalPath.size(); i++)
		// 	std::cout<<" ("<<int( (graph[finalPath[i].vertex].state[0]+0.001)/mUnitEdgeLength)<<","<<int( (graph[finalPath[i].vertex].state[1]+0.001)/mUnitEdgeLength)<<") "<<finalPath[i].timestep<<" "<<finalPath[i].tasks_completed<<" "<<finalPath[i].in_delivery<<std::endl;
		// std::cout<<std::endl;

		auto stop1 = high_resolution_clock::now();
		mCSPTime += (stop1 - start1);

		return finalPath;
	}

	void preprocess_graph(Graph &g)
	{
		auto start1 = high_resolution_clock::now();
		int V = boost::num_vertices(g);

		VertexIter vi_1, viend_1;
		VertexIter vi_2, viend_2;
		for (boost::tie(vi_1, viend_1) = vertices(g); vi_1 != viend_1; ++vi_1) 
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
					mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_2)]=1;
				else
					mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_2)]=INF;
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
			if (mAllPairsShortestPathMap[std::make_pair(vertex_2,vertex_3)] + 0.00001 > (mAllPairsShortestPathMap[std::make_pair(vertex_2,vertex_1)] + mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_3)])
				&& (mAllPairsShortestPathMap[std::make_pair(vertex_2,vertex_1)] != INF
				&& mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_3)] != INF))
				mAllPairsShortestPathMap[std::make_pair(vertex_2,vertex_3)] = mAllPairsShortestPathMap[std::make_pair(vertex_2,vertex_1)] + mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_3)];
		}

		// std::cout<<"All pairs shortest paths: "<<std::endl;
		// for (boost::tie(vi_1, viend_1) = vertices(g); vi_1 != viend_1; ++vi_1) 
		// for (boost::tie(vi_2, viend_2) = vertices(g); vi_2 != viend_2; ++vi_2) 
		// {
		// 	Vertex vertex_1 = *vi_1;
		// 	Vertex vertex_2 = *vi_2;
		// 	std::cout<<vertex_1<<" "<<vertex_2<<" "<<mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_2)]<<std::endl;
		// }
		// std::cin.get();
		auto stop1 = high_resolution_clock::now();
		mPreprocessTime += (stop1 - start1);
	}

};


} // namespace PCCBS

#endif 
