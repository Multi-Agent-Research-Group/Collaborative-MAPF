
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

#define INF std::numeric_limits<double>::infinity()

namespace CMAPF {

using namespace BGL_DEFINITIONS;

class CBS
{

public:

	std::chrono::duration<double, std::micro> mCSPTime;
	std::chrono::duration<double, std::micro> mGNTime;
	std::chrono::duration<double, std::micro> mQOTime;
	std::chrono::duration<double, std::micro> mCCTime;

	/// Environment
	cv::Mat mImage;

	std::vector<std::vector<std::pair<int,std::pair<Vertex,Vertex>>>> mTasksList;
	std::vector<std::unordered_map<Vertex,bool>> mSpecialPosition;

	/// The fixed graphs denoting individual environment of corresponding agents
	std::vector<Graph> mGraphs;

	/// Number of agents
	int mNumAgents; 

	/// Path to the roadmap files.
	std::vector<std::string> mRoadmapFileNames;

	/// Source vertex.
	std::vector<Eigen::VectorXd> mStartConfig;
	std::vector<Vertex> mStartVertex;

	std::vector<int> mStartTimestep;
	std::vector<int> mGoalTimestep;

	boost::unordered_map<std::pair<Vertex,Vertex>,double> mAllPairsShortestPathMap;

	double mUnitEdgeLength = 0.0625;

	CBS(cv::Mat img, int numAgents, std::vector<std::string> roadmapFileNames, Eigen::VectorXd _start_config, 
		std::vector<std::vector<std::pair<int,std::pair<Eigen::VectorXd,Eigen::VectorXd>>>> _tasks_list)
		: mImage(img)
		, mNumAgents(numAgents)
		, mRoadmapFileNames(roadmapFileNames)
	{
		auto t1 = std::chrono::high_resolution_clock::now();
	    auto t2 = std::chrono::high_resolution_clock::now();
		mCSPTime = t2-t1;
		mGNTime = t2-t1;
		mQOTime = t2-t1;
		mCCTime = t2-t1;

		for(int i=0; i<mNumAgents;i++)
		{
			Eigen::VectorXd start_config(2);
			for (int ui = i*2; ui < i*2+2; ui++)
				start_config[ui-i*2] = _start_config[ui];
			mStartConfig.push_back(start_config);
		}

		for(int i=0; i<_tasks_list.size(); i++)
		{
			std::vector<std::pair<int,std::pair<Vertex,Vertex>>> agent_tasks_list;
			for(int j=0; j<_tasks_list[i].size(); j++)
				agent_tasks_list.push_back(std::make_pair(_tasks_list[i][j].first,std::make_pair(0,0)));
			mTasksList.push_back(agent_tasks_list);
		}

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			Graph graph;
			Vertex start_vertex;

			create_vertices(graph,get(&VProp::state,graph),mRoadmapFileNames[agent_id],2,get(&EProp::prior,graph));
			create_edges(graph,get(&EProp::length,graph));

			VertexIter ind_vi, ind_vi_end;
			int i=0;
			for (boost::tie(ind_vi, ind_vi_end) = vertices(graph); ind_vi != ind_vi_end; ++ind_vi,++i)
			{
				put(&VProp::vertex_index,graph,*ind_vi,i);
				if(mStartConfig[agent_id].isApprox(graph[*ind_vi].state))
					start_vertex = *ind_vi;
				for(int i=0; i<_tasks_list[agent_id].size(); i++)
					if(_tasks_list[agent_id][i].second.first.isApprox(graph[*ind_vi].state))
						mTasksList[agent_id][i].second.first = *ind_vi;
					else if(_tasks_list[agent_id][i].second.second.isApprox(graph[*ind_vi].state))
						mTasksList[agent_id][i].second.second = *ind_vi;
			}

			mGraphs.push_back(graph);
			mStartVertex.push_back(start_vertex);
		}

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

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			preprocess_graph(mGraphs[agent_id]);
	}

	void printStats()
	{
		std::cout<<"computeShortestPath time: "<<mCSPTime.count()/1000000.0<<std::endl;
		std::cout<<"Queue Operations time: "<<mQOTime.count()/1000000.0<<std::endl;
		std::cout<<"Get Neighbors time: "<<mGNTime.count()/1000000.0<<std::endl;
		std::cout<<"Constraints time: "<<mCCTime.count()/1000000.0<<std::endl;
	}


	std::vector< std::vector<SearchState> > computeDecoupledPaths(std::vector<std::vector<CollisionConstraint>> collision_constraints, 
		std::vector<std::vector<CollaborationConstraint>> collaboration_constraints, std::vector<std::vector<CollaborationConstraint>> non_collaboration_constraints, 
		std::vector<double> &costs)
	{
		std::vector<std::vector<SearchState> > shortestPaths;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			double ind_cost;
			std::vector <SearchState> path = computeShortestPath(agent_id, collision_constraints[agent_id], collaboration_constraints[agent_id], 
				non_collaboration_constraints[agent_id], ind_cost);
			shortestPaths.push_back(path);
			costs.push_back(ind_cost);
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

	bool getCollaborationConstraints(std::vector<std::vector<SearchState>> &paths, bool &is_collaboration,
		std::vector<int> &collaborating_agent_ids, CollaborationConstraint &constraint_c)
	{
		return false;
	}

	bool getCollisionConstraints(std::vector<std::vector<SearchState>> &paths, std::vector<int> &agent_id_1, CollisionConstraint &constraint_1, std::vector<int> &agent_id_2, CollisionConstraint &constraint_2)
	{
		int current_timestep = 0;
		int maximum_timestep = 0;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			maximum_timestep = std::max(maximum_timestep, paths[agent_id].at(paths[agent_id].size()-1).timestep);
		// std::cout<<"MT: "<<maximum_timestep<<std::endl;
		std::vector<int> current_path_id(mNumAgents, 0);
		while(current_timestep < maximum_timestep)
		{
			// std::cout<<"\nCT:"<<current_timestep<<std::endl;
			std::vector<int> source_task_ids(mNumAgents, -1);
			std::vector<Vertex> source_vertices(mNumAgents); // all source vertices
			std::vector<int> source_tasks_completed(mNumAgents, 0);
			std::vector<bool> source_in_delivery(mNumAgents, false);

			std::vector<int> target_task_ids(mNumAgents, -1);
			std::vector<Vertex> target_vertices(mNumAgents); // target vertex for collab agents
			std::vector<int> target_tasks_completed(mNumAgents, 0);
			std::vector<bool> target_in_delivery(mNumAgents, false);


			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			{
				if(current_path_id[agent_id] == paths[agent_id].size())
				{
					source_vertices[agent_id] = paths[agent_id].at(paths[agent_id].size()-1).vertex;
					source_tasks_completed[agent_id] = paths[agent_id].at(paths[agent_id].size()-1).tasks_completed;
					source_in_delivery[agent_id] = paths[agent_id].at(paths[agent_id].size()-1).in_delivery;
				}
				else
				{
					source_vertices[agent_id] = paths[agent_id].at(current_path_id[agent_id]).vertex;
					while(current_path_id[agent_id] < paths[agent_id].size()
						&& paths[agent_id].at(current_path_id[agent_id]).timestep == current_timestep)
					{
						if(paths[agent_id].at(current_path_id[agent_id]).in_delivery == true)
							source_task_ids[agent_id] = mTasksList[agent_id][paths[agent_id].at(current_path_id[agent_id]).tasks_completed].first;
						source_tasks_completed[agent_id] = paths[agent_id].at(current_path_id[agent_id]).tasks_completed;
						source_in_delivery[agent_id] = paths[agent_id].at(current_path_id[agent_id]).in_delivery;
						current_path_id[agent_id]++;
					}
				}

				if(current_path_id[agent_id] == paths[agent_id].size())
				{
					target_vertices[agent_id] = paths[agent_id].at(paths[agent_id].size()-1).vertex;
					target_tasks_completed[agent_id] = paths[agent_id].at(paths[agent_id].size()-1).tasks_completed;
					target_in_delivery[agent_id] = paths[agent_id].at(paths[agent_id].size()-1).in_delivery;
				}
				else
				{
					int target_path_id = current_path_id[agent_id];
					target_vertices[agent_id] = paths[agent_id].at(target_path_id).vertex;
					target_tasks_completed[agent_id] = paths[agent_id].at(target_path_id).tasks_completed;
					target_in_delivery[agent_id] = paths[agent_id].at(target_path_id).in_delivery;

					while(target_path_id < paths[agent_id].size()
						&& paths[agent_id].at(target_path_id).timestep == current_timestep+1)
					{
						if(paths[agent_id].at(target_path_id).in_delivery == true)
							target_task_ids[agent_id] = mTasksList[agent_id][paths[agent_id].at(target_path_id).tasks_completed].first;
						target_path_id++;
					}
				}
			}

			// for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			// {
			// 	std::cout<<"SV: "<<source_vertices[agent_id]<<" "<<" TV: "<<target_vertices[agent_id]<<std::endl;
			// 	// std::cout<<source_task_ids[agent_id]
			// }

			// for(int i=0; i<agent_ids.size(); i++)
			//  std::cout<<agent_ids[i]<<" ";
			// std::cout<<std::endl;
			// std::cin.get();

			for(int i=0; i<mNumAgents; i++)
			for(int j=i+1; j<mNumAgents; j++)
			{
				if(target_task_ids[i]==-1 || target_task_ids[i]!=target_task_ids[j])
				{
					if(getVerticesCollisionStatus(mGraphs[i][target_vertices[i]].state, mGraphs[j][target_vertices[j]].state))
					{
						if(target_task_ids[i]==-1)
							agent_id_1.push_back(i);
						else
							for(int k=0; k<mNumAgents; k++)
								if(target_task_ids[k] == target_task_ids[i])
									agent_id_1.push_back(k);

						if(target_task_ids[j]==-1)
							agent_id_2.push_back(j);
						else
							for(int k=0; k<mNumAgents; k++)
								if(target_task_ids[k] == target_task_ids[j])
									agent_id_2.push_back(k);

						constraint_1 = CollisionConstraint(target_vertices[i], target_tasks_completed[i], target_in_delivery[i], current_timestep+1);
						constraint_2 = CollisionConstraint(target_vertices[j], target_tasks_completed[j], target_in_delivery[j], current_timestep+1);

						return true;
					}
				}
				
				if(getEdgesCollisionStatus(mGraphs[i][source_vertices[i]].state, mGraphs[i][target_vertices[i]].state, mGraphs[j][source_vertices[j]].state, mGraphs[j][target_vertices[j]].state))
				{
					if(source_task_ids[i]==-1)
							agent_id_1.push_back(i);
					else
						for(int k=0; k<mNumAgents; k++)
							if(source_task_ids[k] == source_task_ids[i])
								agent_id_1.push_back(k);

					if(source_task_ids[j]==-1)
						agent_id_2.push_back(j);
					else
						for(int k=0; k<mNumAgents; k++)
							if(source_task_ids[k] == source_task_ids[j])
								agent_id_2.push_back(k);

					Edge edge_1 = boost::edge(source_vertices[i],target_vertices[i],mGraphs[i]).first;
					constraint_1 = CollisionConstraint(edge_1, target_tasks_completed[i], target_in_delivery[i], current_timestep+1);

					Edge edge_2 = boost::edge(source_vertices[j],target_vertices[j],mGraphs[j]).first;
					constraint_2 = CollisionConstraint(edge_2, target_tasks_completed[j], target_in_delivery[j], current_timestep+1);

					return true;
				}
			}
			
			current_timestep++;
		}
		return false;
	}

	std::vector<std::vector<Eigen::VectorXd>> solve()
	{
		CBSPriorityQueue PQ(mNumAgents);

		std::vector<std::vector<CollisionConstraint>> collision_constraints(mNumAgents, std::vector<CollisionConstraint>());
		std::vector<std::vector<CollaborationConstraint>> collaboration_constraints(mNumAgents, std::vector<CollaborationConstraint>());
		std::vector<std::vector<CollaborationConstraint>> non_collaboration_constraints(mNumAgents, std::vector<CollaborationConstraint>());
		std::vector<double> start_costs;
		std::vector< std::vector<SearchState> > start_shortestPaths = computeDecoupledPaths(collision_constraints, collaboration_constraints, non_collaboration_constraints, start_costs);

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			if(start_shortestPaths.at(agent_id).size()==0)
			{
				// std::cout<<"No Path exists for index "<<agent_id<<"! Press [ENTER] to exit: ";
				// std::cin.get();
				return std::vector<std::vector<Eigen::VectorXd>>(mNumAgents,std::vector<Eigen::VectorXd>());
			}
		}

		
		
		PQ.insert(start_costs, collision_constraints, collaboration_constraints, non_collaboration_constraints, start_shortestPaths);

		
		int numSearches = 0;
		while(PQ.PQsize()!=0)
		{
			numSearches++;

			auto start = high_resolution_clock::now();
			Element p = PQ.pop();
			auto stop = high_resolution_clock::now();
			mQOTime += (stop - start);
			

			// 

			double total_cost = 0;
			for(int i=0; i<p.costs.size(); i++)
				total_cost = std::max(total_cost,p.costs[i]);

			// if(numSearches%2 == 0)
			// {
			// 	// std::cout<<PQ.PQsize()<<std::endl;
			// 	std::cout<<"\n-\nCBS numSearches: "<<numSearches<<" Cost: "<<int((total_cost+0.0001)/0.0625)<<std::endl;
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
			// 			std::cout<<p.shortestPaths[agent_id][i].vertex<<" - ("<<int( (mGraphs[agent_id][p.shortestPaths[agent_id][i].vertex].state[0]+0.001)/0.0625)<<","<<int( (mGraphs[agent_id][p.shortestPaths[agent_id][i].vertex].state[1]+0.001)/0.0625)<<") "<<p.shortestPaths[agent_id][i].timestep
			// 				<<" "<<p.shortestPaths[agent_id][i].tasks_completed<<" "<<p.shortestPaths[agent_id][i].in_delivery<<std::endl;
			// 		std::cout<<std::endl;
			// 	}
			// 	std::cin.get();
			// 	// break;
			// }

			// if(numSearches == 5000)
			// 	break;

			bool is_collaboration = true;
			std::vector<int> collaborating_agent_ids;
			CollaborationConstraint constraint_c;

			if(getCollaborationConstraints(p.shortestPaths, is_collaboration, collaborating_agent_ids, constraint_c))
			{
				std::vector<std::vector<CollaborationConstraint>> increase_constraints_c;
				if(is_collaboration)
					increase_constraints_c = p.collaboration_constraints;
				else
					increase_constraints_c = p.non_collaboration_constraints;

				std::vector< double> costs_c = p.costs;
				std::vector< std::vector<SearchState> > shortestPaths_c = p.shortestPaths;

				bool all_paths_exist = true;

				for(int i=0; i<collaborating_agent_ids.size(); i++)
				{
					increase_constraints_c[collaborating_agent_ids[i]].push_back(constraint_c);
					double cost_c;
					if(is_collaboration)
						shortestPaths_c[collaborating_agent_ids[i]] = computeShortestPath(collaborating_agent_ids[i], p.collision_constraints[collaborating_agent_ids[i]], 
							increase_constraints_c[collaborating_agent_ids[i]], p.non_collaboration_constraints[collaborating_agent_ids[i]], cost_c);
					else
						shortestPaths_c[collaborating_agent_ids[i]] = computeShortestPath(collaborating_agent_ids[i], p.collision_constraints[collaborating_agent_ids[i]], 
							p.collaboration_constraints[collaborating_agent_ids[i]], increase_constraints_c[collaborating_agent_ids[i]], cost_c);
					costs_c[collaborating_agent_ids[i]] = cost_c;
					if(cost_c == INF)
					{
						all_paths_exist = false;
						break;
					}
				}

				if(all_paths_exist)
				{
					// std::cout<<"inserting left!"<<std::endl;
					if(is_collaboration)
						PQ.insert(costs_c, p.collision_constraints, increase_constraints_c,
							p.non_collaboration_constraints, shortestPaths_c);
					else
						PQ.insert(costs_c, p.collision_constraints, p.collaboration_constraints,
							increase_constraints_c, shortestPaths_c);
				}
				continue;
			}

			std::vector<int> agent_id_1;
			CollisionConstraint constraint_1;

			std::vector<int> agent_id_2;
			CollisionConstraint constraint_2;

			// 

			// std::cout<<"Calling CollaborationConstraint!"<<std::endl;

			if(getCollisionConstraints(p.shortestPaths, agent_id_1, constraint_1, agent_id_2, constraint_2))
			{
				//agent_id_1

				// std::cout<<"In CollaborationConstraint!"<<std::endl;

				std::vector<std::vector<CollisionConstraint>> increase_constraints_agent_id_1 = p.collision_constraints;
				std::vector< double> costs_agent_id_1 = p.costs;
				std::vector< std::vector<SearchState> > shortestPaths_agent_id_1 = p.shortestPaths;

				bool all_paths_exist = true;

				for(int i=0; i<agent_id_1.size(); i++)
				{
					increase_constraints_agent_id_1[agent_id_1[i]].push_back(constraint_1);
					double cost_agent_id_1;
					shortestPaths_agent_id_1[agent_id_1[i]] = computeShortestPath(agent_id_1[i], increase_constraints_agent_id_1[agent_id_1[i]], 
						p.collaboration_constraints[agent_id_1[i]], p.non_collaboration_constraints[agent_id_1[i]], cost_agent_id_1);
					costs_agent_id_1[agent_id_1[i]] = cost_agent_id_1;

					if(cost_agent_id_1 == INF)
					{
						all_paths_exist = false;
						break;
					}
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
				std::vector< double> costs_agent_id_2 = p.costs;
				std::vector< std::vector<SearchState> > shortestPaths_agent_id_2 = p.shortestPaths;

				all_paths_exist = true;

				for(int i=0; i<agent_id_2.size(); i++)
				{
					increase_constraints_agent_id_2[agent_id_2[i]].push_back(constraint_2);
					double cost_agent_id_2;
					shortestPaths_agent_id_2[agent_id_2[i]] = computeShortestPath(agent_id_2[i], increase_constraints_agent_id_2[agent_id_2[i]], 
						p.collaboration_constraints[agent_id_2[i]], p.non_collaboration_constraints[agent_id_2[i]], cost_agent_id_2);
					costs_agent_id_2[agent_id_2[i]] = cost_agent_id_2;
					
					if(cost_agent_id_2 == INF)
					{
						all_paths_exist = false;
						break;
					}
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
			// 	std::cout<<"\n-\nCBS numSearches: "<<numSearches<<" Cost: "<<int((total_cost+0.0001)/0.0625)<<std::endl;
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
			// 			std::cout<<p.shortestPaths[agent_id][i].vertex<<" - ("<<int( (mGraphs[agent_id][p.shortestPaths[agent_id][i].vertex].state[0]+0.001)/0.0625)<<","<<int( (mGraphs[agent_id][p.shortestPaths[agent_id][i].vertex].state[1]+0.001)/0.0625)<<") "<<p.shortestPaths[agent_id][i].timestep
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
					}
					else
					{
						collision_free_path[agent_id].push_back(mGraphs[agent_id][paths[agent_id].at(current_path_id[agent_id]).vertex].state);
						while(current_path_id[agent_id] < paths[agent_id].size()
							&& paths[agent_id].at(current_path_id[agent_id]).timestep == current_timestep)
							current_path_id[agent_id]++;
					}

				}
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

			std::cout<<"Press [ENTER] to display path: ";
			std::cin.get();
			displayPath(path_configs);

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
		cv::Mat image;
		cv::cvtColor(mImage, image, CV_GRAY2BGR);
		// cv::Mat image = cv::merge(mImage,mImage,mImage);  // going from one to 3 channel
		int numberOfRows = image.rows;
		int numberOfColumns = image.cols;

		std::vector<cv::Mat4b> number_images(4);
		for(int i=0; i<number_images.size(); i++)
		{
			std::stringstream ss;
			ss << "./src/CMAPF/data/viz/";
			if(i==1)
				ss << 3;
			else if (i==2)
				ss << 2;
			else if(i==3)
				ss<< 1;
			else 
				ss << 4;
			ss << ".png";
			number_images[i] = imread(ss.str(), cv::IMREAD_UNCHANGED);
			double scale = 0.037;
			if(i<3)
				scale = 0.025;
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
				cv::line(image, source_Point, target_Point, cv::Scalar(0, 255, 255), 2);
			}

			VertexIter vi, vi_end;
			for (boost::tie(vi, vi_end) = vertices(mGraphs[agent_id]); vi != vi_end; ++vi)
			{
				double x_point = mGraphs[agent_id][*vi].state[0]*numberOfColumns;
				double y_point = (1 - mGraphs[agent_id][*vi].state[1])*numberOfRows;
				cv::Point centre_Point((int)x_point, (int)y_point);
				cv::circle(image, centre_Point, 4,  cv::Scalar(0, 150, 0), -1);
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
				cv::line(image, uPoint, vPoint, cv::Scalar(0, 140, 255), 2);
			}   
		}

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			VertexIter vi, vi_end;
			for (boost::tie(vi, vi_end) = vertices(mGraphs[agent_id]); vi != vi_end; ++vi)
			{
				double x_point = mGraphs[agent_id][*vi].state[0]*numberOfColumns;
				double y_point = (1 - mGraphs[agent_id][*vi].state[1])*numberOfRows;
				cv::Point centre_Point((int)x_point, (int)y_point);
				cv::circle(image, centre_Point, 4,  cv::Scalar(0, 150, 0), -1);
			}
		} 

		std::vector< std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>> > tasks;

		std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>> agent_tasks_4;
		agent_tasks_4.push_back(std::make_pair(std::make_pair(5,2),std::make_pair(6,9)));
		agent_tasks_4.push_back(std::make_pair(std::make_pair(6,10),std::make_pair(6,11)));
		tasks.push_back(agent_tasks_4);

		std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>> agent_tasks_3;
		agent_tasks_3.push_back(std::make_pair(std::make_pair(8,1),std::make_pair(6,7)));
		agent_tasks_3.push_back(std::make_pair(std::make_pair(6,8),std::make_pair(6,12)));
		tasks.push_back(agent_tasks_3);

		std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>> agent_tasks_2;
		agent_tasks_2.push_back(std::make_pair(std::make_pair(5,1),std::make_pair(6,5)));
		agent_tasks_2.push_back(std::make_pair(std::make_pair(6,6),std::make_pair(6,13)));
		tasks.push_back(agent_tasks_2);

		std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>> agent_tasks_1;
		agent_tasks_1.push_back(std::make_pair(std::make_pair(8,4),std::make_pair(6,3)));
		agent_tasks_1.push_back(std::make_pair(std::make_pair(6,4),std::make_pair(6,14)));
		tasks.push_back(agent_tasks_1);

		for(int i=0; i<tasks.size(); i++)
		{
			cv::Scalar col;
			if(i==0) 
				col = cv::Scalar(255,0,0);
			else if(i==1) 
				col = cv::Scalar(0,255,255);
			else if(i==2) 
				col = cv::Scalar(255,255,255);
			else
				col = cv::Scalar(0,0,255);
			for(int j=0; j<tasks[i].size(); j++)
			{
				{
					cv::Point uPoint((int)(tasks[i][j].first.first*0.0625*numberOfColumns), (int)((1 - tasks[i][j].first.second*0.0625)*numberOfRows)); 
					std::string text = std::to_string(4-i) + ((j==0)?"A":"B");
					cv::circle(image, uPoint, 7,  col, -1);
					cv::circle(image, uPoint, 8,  cv::Scalar(0,0,0), 1);
					cv::putText(image, text, cv::Point(uPoint.x - 6,uPoint.y+3), cv::FONT_HERSHEY_PLAIN, 0.6, cvScalar(0,0,0), 1, 4);
				}

				{
					cv::Point uPoint((int)(tasks[i][j].second.first*0.0625*numberOfColumns), (int)((1 - tasks[i][j].second.second*0.0625)*numberOfRows)); 
					std::string text = std::to_string(4-i) + ((j==0)?"A":"B");
					cv::circle(image, uPoint, 7, col, -1);
					cv::circle(image, uPoint, 8,  cv::Scalar(0,0,0), 1);
					cv::putText(image, text, cv::Point(uPoint.x - 6,uPoint.y+3), cv::FONT_HERSHEY_PLAIN, 0.6, cvScalar(0,0,0), 1, 4);
				}
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
				for(int agent_id = 0; agent_id<mNumAgents; agent_id++)
				{
					Eigen::VectorXd intermediate_config(2);
					if(i < nStates[agent_id] - 1 && !source_configs[agent_id].isApprox(target_configs[agent_id]))
						intermediate_config <<  source_configs[agent_id] + (resolution*i/edge_lengths[agent_id])*(target_configs[agent_id]-source_configs[agent_id]);
					else
						intermediate_config << target_configs[agent_id];

					double x_point = intermediate_config[0]*numberOfColumns;
					double y_point = (1 - intermediate_config[1])*numberOfRows;
					cv::Point _Point((int)x_point, (int)y_point);
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
				for(int agent_id=0; agent_id<mNumAgents; agent_id++)
				{   
					double x_point = v[agent_id*2]*numberOfColumns;
					double y_point = (1 - v[agent_id*2+1])*numberOfRows;
					cv::Point _Point((int)x_point, (int)y_point);
					// cv::circle(new_image, _Point, 6,  cv::Scalar(0,255,0), -1);
					// cv::circle(new_image, _Point, 8,  cv::Scalar(0,255,0), 1);
					int x = x_point - number_images[agent_id].cols/2;
					int y = y_point - number_images[agent_id].rows/2;
					double alpha = 1.0; // alpha in [0,1]

					// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
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
				
				cv::namedWindow("Agents",cv::WINDOW_NORMAL);
				cv::imshow("Agents", new_image);
				cv::waitKey(100);
			}
		}
		cv::namedWindow("Graph Visualization",cv::WINDOW_NORMAL);
		cv::imshow("Graph Visualization", image);
		cv::waitKey(0);
	}

	double getHeuristic(int &agent_id, SearchState &state)
	{
		// return 0;
		double heuristic=0;
		if(state.in_delivery == true)
			heuristic += mAllPairsShortestPathMap[std::make_pair(state.vertex, mTasksList[agent_id][state.tasks_completed].second.second)];
		else
		{
			heuristic += mAllPairsShortestPathMap[std::make_pair(state.vertex, mTasksList[agent_id][state.tasks_completed].second.first)];
			heuristic += mAllPairsShortestPathMap[std::make_pair(mTasksList[agent_id][state.tasks_completed].second.first, mTasksList[agent_id][state.tasks_completed].second.second)];
		}
		for(int i=state.tasks_completed+1; i<mTasksList[agent_id].size(); i++)
		{
			heuristic += mAllPairsShortestPathMap[std::make_pair(mTasksList[agent_id][i-1].second.second,mTasksList[agent_id][i].second.first)];
			heuristic += mAllPairsShortestPathMap[std::make_pair(mTasksList[agent_id][i].second.first,mTasksList[agent_id][i].second.second)];
		}
		return heuristic;
	}

	std::vector<SearchState> computeShortestPath(int &agent_id, std::vector<CollisionConstraint> &collision_constraints,
		std::vector<CollaborationConstraint> &collaboration_constraints, std::vector<CollaborationConstraint> &non_collaboration_constraints, double& costOut)
	{
		// std::cout<<"Tasks assigned: "<<std::endl;
		// for(int i=0; i<mTasksList[agent_id].size(); i++)
		// 	std::cout<<mTasksList[agent_id][i].first<<" - ("<<int( (mGraphs[agent_id][mTasksList[agent_id][i].second.first].state[0]+0.001)/0.0625)<<", "
		// 		<<int( (mGraphs[agent_id][mTasksList[agent_id][i].second.first].state[1]+0.001)/0.0625)<<")"
		// 		<<" "<<" ("<<int( (mGraphs[agent_id][mTasksList[agent_id][i].second.second].state[0]+0.001)/0.0625)<<", "
		// 			<<int( (mGraphs[agent_id][mTasksList[agent_id][i].second.second].state[1]+0.001)/0.0625)<<") "<<std::endl;
		// std::cout<<std::endl;

		// std::cout<<"Start position: ("<<int( (mGraphs[agent_id][mStartVertex[agent_id]].state[0]+0.001)/0.0625)<<", "
		// 	<<int( (mGraphs[agent_id][mStartVertex[agent_id]].state[1]+0.001)/0.0625)<<")"<<std::endl;

		// std::cout<<"Special Positions: ";
		// for(auto &node: mSpecialPosition[agent_id])
		// {
		// 	std::cout<<" ("<<int( (mGraphs[agent_id][node.first].state[0]+0.001)/0.0625)<<", "
		// 	<<int( (mGraphs[agent_id][node.first].state[1]+0.001)/0.0625)<<") ";
		// }
		// std::cin.get();
		// Graph graph = mGraphs[agent_id];

		auto start1 = high_resolution_clock::now();

		int min_goal_timestep = 0;

		for( CollisionConstraint &c: collision_constraints)
		{
			min_goal_timestep = std::max(min_goal_timestep, c.timestep);
		}

		Vertex start = mStartVertex[agent_id];

		std::unordered_map<int, SearchState> collabMap;
		for( CollaborationConstraint &c: collaboration_constraints)
		{
			SearchState state = SearchState(c.v,c.timestep,c.tasks_completed,c.in_delivery);
			collabMap[c.timestep] = state;
		}

		timePriorityQueue pq;
		boost::unordered_map<SearchState, double, state_hash> mDistance;
		boost::unordered_map<SearchState , SearchState, state_hash > mPrev;

		SearchState start_state = SearchState(start,0,0,0);
		pq.insert(start_state,getHeuristic(agent_id, start_state),0.0);
		mDistance[start_state]=0;

		int numSearches = 0;
		int maximum_timestep = 10000;

		int goal_timestep = -1;

		SearchState goal_state = SearchState();

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

			if(numSearches%1000 == 0)
			{
				std::cout<<"numSearches: "<<numSearches<<std::endl;
				std::cout<<" ("<<int( (mGraphs[agent_id][current_vertex].state[0]+0.001)/0.0625)<<", "
			<<int( (mGraphs[agent_id][current_vertex].state[1]+0.001)/0.0625)<<")"<<std::endl;
				std::cout<<current_timestep<<" "<<current_tasks_completed<<" "<<current_in_delivery<<std::endl;
			}

			

			if(current_tasks_completed == mTasksList[agent_id].size() && current_timestep>= min_goal_timestep)
			{
				// std::cout<<"Timestep goal was found: "<<final_timestep<<std::endl;
				costOut = mDistance[current_state];
				goal_state = current_state;
				break;
			}

			if(current_tasks_completed == mTasksList[agent_id].size())
			{
				continue;
			}

			
			if(collaboration_constraints.size()!=0)
			{
				if(collabMap.count(current_timestep) != 0)
				{
					SearchState collaboration_state = collabMap[current_timestep];

					if(current_vertex == collaboration_state.vertex 
						&& current_tasks_completed == collaboration_state.tasks_completed)
					{
						if(collaboration_state.in_delivery == current_in_delivery) // pickup state is fine, let go into generate neighbors
						{

						}
						else // generate pickup state/deliver state
						{
							double new_cost = mDistance[current_state];

							SearchState new_state;
							if(collaboration_state.in_delivery)
								new_state = SearchState(current_vertex, current_timestep, current_tasks_completed, true);
							else
								new_state = SearchState(current_vertex, current_timestep, current_tasks_completed+1, false);
							
							if(mDistance.count(new_state)==0 || new_cost < mDistance[new_state])
							{
								mDistance[new_state]= new_cost;
								double priority = new_cost + getHeuristic(agent_id, new_state);
								pq.insert(new_state,priority,0.0);
								mPrev[new_state]=current_state;
							}
							continue;
						}
					}
					else
					{
						continue; // useless state
					}
				}
			}

			// std::cout<<"K1";std::cin.get();

			if(mSpecialPosition[agent_id].count(current_vertex)!= 0)
			{
				if(!current_in_delivery && mTasksList[agent_id][current_tasks_completed].second.first == current_vertex) //pickup point
				{
					bool allowed = true;
					for( CollaborationConstraint &c: non_collaboration_constraints)
					{
						if( current_vertex == c.v && current_tasks_completed == c.tasks_completed
							&& c.in_delivery==true && c.timestep == current_timestep) //pickup object is not allowed at this timestep
						{
							// std::cout<<"Non collaboration Constraint Encountered! "<<std::endl;
							allowed = false;
							break;
						}
					}
					if(allowed)
					{
						for( CollisionConstraint &c: collision_constraints)
						{
							if( c.constraint_type == 1 && current_vertex == c.v 
								&& current_tasks_completed == c.tasks_completed && true == c.in_delivery
							 	&& c.timestep == current_timestep)
							{
								// std::cout<<"CollisionConstraint Encountered! "<<std::endl;
								allowed = false;
								break;
							}
						}
					}
					if(allowed)
					{
						double new_cost = mDistance[current_state];
						SearchState new_state = SearchState(current_vertex, current_timestep, current_tasks_completed, true);
						if(mDistance.count(new_state)==0 || new_cost < mDistance[new_state])
						{
							mDistance[new_state]= new_cost;
							double priority = new_cost + getHeuristic(agent_id, new_state);
							pq.insert(new_state,priority,0.0);
							mPrev[new_state]=current_state;
						}
					}
				}
				if(current_in_delivery && mTasksList[agent_id][current_tasks_completed].second.second == current_vertex) //delivery point
				{
					bool allowed = true;
					for( CollaborationConstraint &c: non_collaboration_constraints)
					{
						if( current_vertex == c.v && current_tasks_completed+1 == c.tasks_completed
							&& c.in_delivery==false && c.timestep == current_timestep) //pickup object is not allowed at this timestep
						{
							// std::cout<<"Non collaboration Constraint Encountered! "<<std::endl;
							allowed = false;
							break;
						}
					}
					if(allowed)
					{
						for( CollisionConstraint &c: collision_constraints)
						{
							if( c.constraint_type == 1 && current_vertex == c.v 
								&& current_tasks_completed+1 == c.tasks_completed && false == c.in_delivery
							 	&& c.timestep == current_timestep)
							{
								// std::cout<<"CollisionConstraint Encountered! "<<std::endl;
								allowed = false;
								break;
							}
						}
					}
					if(allowed)
					{
						double new_cost = mDistance[current_state];
						SearchState new_state= SearchState(current_vertex, current_timestep, current_tasks_completed+1, false);
						
						if(mDistance.count(new_state)==0 || new_cost < mDistance[new_state])
						{
							mDistance[new_state]= new_cost;
							double priority = new_cost + getHeuristic(agent_id, new_state);
							pq.insert(new_state,priority,0.0);
							mPrev[new_state]=current_state;
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
					double new_cost = mDistance[current_state] + mUnitEdgeLength;

					SearchState new_state = SearchState(current_vertex, current_timestep+1, current_tasks_completed, current_in_delivery);
					
					if(mDistance.count(new_state)==0 || new_cost < mDistance[new_state])
					{
						mDistance[new_state]= new_cost;
						double priority = new_cost + getHeuristic(agent_id, new_state);
						pq.insert(new_state,priority,0.0);
						mPrev[new_state]=current_state;
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
					double new_cost = mDistance[current_state] + mUnitEdgeLength;

					SearchState new_state = SearchState(successor, current_timestep+1, current_tasks_completed, current_in_delivery);
					
					if(mDistance.count(new_state)==0 || new_cost < mDistance[new_state])
					{
						mDistance[new_state]= new_cost;
						double priority = new_cost + getHeuristic(agent_id, new_state);
						pq.insert(new_state,priority,0.0);
						mPrev[new_state]=current_state;
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
		// 	std::cout<<" ("<<int( (graph[finalPath[i].vertex].state[0]+0.001)/0.0625)<<","<<int( (graph[finalPath[i].vertex].state[1]+0.001)/0.0625)<<") "<<finalPath[i].timestep<<" "<<finalPath[i].tasks_completed<<" "<<finalPath[i].in_delivery<<std::endl;
		// std::cout<<std::endl;

		auto stop1 = high_resolution_clock::now();
		mCSPTime += (stop1 - start1);

		return finalPath;
	}

	void preprocess_graph(Graph &g)
	{
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
					mAllPairsShortestPathMap[std::make_pair(vertex_1,vertex_2)]=mUnitEdgeLength;
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
	}

};


} // namespace CMAPF

#endif 
