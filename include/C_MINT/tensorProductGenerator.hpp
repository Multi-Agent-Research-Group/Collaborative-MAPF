#ifndef TPG_GENERATOR_
#define TPG_GENERATOR_

#include<bits/stdc++.h>

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <stdlib.h>

#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <Eigen/Dense>

#include "BGLDefinitions.hpp"

#define make_foursome(a,b,c,d) std::make_pair(a,std::make_pair(b,std::make_pair(c,d)));
#define foursome std::pair<size_t, std::pair < Vertex, std::pair < size_t, int > > >
#define foursome_first(t) t.first
#define foursome_second(t) t.second.first
#define foursome_third(t) t.second.second.first
#define foursome_fourth(t) t.second.second.second

namespace C_MINT {

using namespace BGL_DEFINITIONS;

class TensorPG
{
public: //Change to private once this is working

	boost::unordered_set<CompositeVertex> neighborsAddedSet;

	// graph number -> vertex -> multiple -> delta f map -> neigbors (vector of vertex)
	boost::unordered_map< std::pair<size_t,Vertex>, boost::unordered_map<int, std::vector<Vertex>> > OSF;

	/// vector of source_vertex, target_vertex, t_value
	boost::unordered_map< std::vector< std::pair<std::pair<Vertex,Vertex>, size_t > > , CompositeVertex> indToCompositeMap;

	size_t mNumAgents;

public:

	TensorPG()
	{
	}

	void populateMaps(CompositeGraph &composite_map, CompositeVertex &start, CompositeVertex &goal, std::vector<Graph> &maps)
	{
		// std::cout<<"P HERE"<<std::endl;
		mNumAgents = composite_map[start].composite_vertex.size();  
		indToCompositeMap[composite_map[start].composite_vertex] = start;
		indToCompositeMap[composite_map[goal].composite_vertex] = goal;

		for(size_t index = 0; index <maps.size(); index ++)
		{
			VertexIter vi, viend;
			for (boost::tie(vi, viend) = vertices(maps[index]); vi != viend; ++vi) 
			{
				Vertex node = *vi;
				// std::cout<<"Node: "<<node<<std::endl;
				// int node_f_val = (maps[index][node].distance + maps[index][node].heuristic + 0.000001)/epsilon_value;
				// std::cout<<maps[index][node].distance<<std::endl;
				// std::cout<<maps[index][node].heuristic<<std::endl;
				OutEdgeIter ei, ei_end;
				for (boost::tie(ei, ei_end) = out_edges(node, maps[index]); ei != ei_end; ++ei) 
				{
					Edge e = *ei;
					if(!maps[index][e].isEvaluated)
					{
						std::cout<<"ERROR: Edge not evaluated! "<<node<<" - "<<target(*ei, maps[index]);
						std::cin.get();
					}
					if(maps[index][e].status == CollisionStatus::FREE)
					{
						Vertex curSucc = target(*ei, maps[index]);
						// int neighbor_f_val = (maps[index][node].distance + maps[index][e].length + maps[index][curSucc].heuristic + 0.000001)/epsilon_value;
						int delta_f = (maps[index][e].length + maps[index][curSucc].heuristic - maps[index][node].heuristic + 0.000001)/epsilon_value;
						if(delta_f < -0.0001 )
						{
							std::cout<<"Negative delta!"<<delta_f;
							// std::cin.get();
						}
						std::pair<size_t,Vertex> key = std::make_pair(index,node);
						if(OSF.find(key) == OSF.end())
							OSF[key] = boost::unordered_map<int, std::vector<Vertex>>();
						if(OSF[key].find(delta_f) == OSF[key].end())
							OSF[key][delta_f] = std::vector<Vertex>();
						OSF[key][delta_f].push_back(curSucc);
						// std::cout<<" Delta: "<<delta_f<<" Succ: "<<curSucc<<std::endl;
					}
				}
				// std::cin.get();	
			}
		}
	}

	Edge getEdge(Graph &map, Vertex u, Vertex v)
	{
		Edge uv;
		bool edgeExists;
		boost::tie(uv, edgeExists) = edge(u, v, map);
		if (edgeExists == false)
		{
			std::cout<<"Edge does not exist. Press [ENTER] to get segmentation fault :( :"<<std::endl;
			std::cin.get();
		}

		return uv;
	}

	void getMinRejectedDelta(int current_agent_id, std::vector <std::vector<int>> &considered_deltas, std::vector<int> &delta_values, int &min_rejected_delta, int &delta_f, std::vector <bool> &at_goal)
	{
		if(current_agent_id == mNumAgents)
		{
			bool allWait = true;
			for(int agent_id=0; agent_id<mNumAgents;agent_id++)
				if(delta_values[agent_id] != -1)
				{
					allWait = false;
					break;
				}
			if(allWait)
				return;
			int delta_sum = 0;
			for(int agent_id=0; agent_id<mNumAgents;agent_id++)
			{
				if(delta_values[agent_id] == -1)
				{
					if(!at_goal[agent_id])
						delta_sum += 1;	// waiting converted to +1 at non-goal point
				}
				else
					delta_sum += std::abs(delta_values[agent_id]); 
			}
			// std::cout<<"Delta sum: "<<delta_sum<<std::endl;
			if(delta_sum > delta_f && delta_sum < min_rejected_delta)
				min_rejected_delta = delta_sum;
			return;
		}
		for(int i=0; i<considered_deltas[current_agent_id].size(); i++)
		{
			delta_values[current_agent_id] = considered_deltas[current_agent_id][i];
			getMinRejectedDelta(current_agent_id + 1, considered_deltas, delta_values, min_rejected_delta, delta_f,at_goal);
		}
	}

	void nestedLoops(int current_agent_id, std::vector<CompositeVertex> &neighbors, std::vector <Vertex> &goal_vertices, std::vector<int> &selected_target_indices, std::vector <Vertex> &source_vertices, std::vector <std::vector<Vertex>> &target_vertices, std::vector <size_t> &t_values, 
		CompositeVertex &node, CompositeGraph &composite_map, std::vector<Graph> &maps, CompositeVertex &goal, std::vector<int> &delta_values, int &min_rejected_delta)
	{
		if( current_agent_id == mNumAgents)
		{
			bool allWait = true;
			for(int agent_id=0; agent_id<mNumAgents;agent_id++)
				if(selected_target_indices[agent_id] != -1)
				{
					allWait = false;
					break;
				}
			if(allWait)
				return;
					
			size_t min_steps = std::numeric_limits<size_t>::max();

			std::vector<size_t> steps_vec(mNumAgents);

			for(int agent_id=0;agent_id<mNumAgents;agent_id++)
			{
				if(selected_target_indices[agent_id] == -1) // wait for one time_step
					min_steps = 1;
				else
				{
					Vertex source_vertex = source_vertices[agent_id];
					Vertex target_vertex = target_vertices[agent_id][selected_target_indices[agent_id]];
					size_t steps = getSteps(maps[agent_id][source_vertex].state, maps[agent_id][target_vertex].state) - t_values[agent_id];
					steps_vec[agent_id] = steps;
					min_steps = std::min(min_steps,steps);
				}
			}

			int total_steps = 0;
			double heuristic = 0.0;

			// std::cout<<"MIN STEPS: "<<min_steps<<std::endl;

			std::vector< std::pair<std::pair<Vertex,Vertex>, size_t > > composite_vertex(mNumAgents);
			for(int agent_id=0;agent_id<mNumAgents;agent_id++)
			{
				Vertex source_vertex = source_vertices[agent_id];

				if(selected_target_indices[agent_id] == -1) // wait for one time_step
				{
					if(source_vertex != goal_vertices[agent_id])
						total_steps += 1;
					// else
					// 	std::cout<<"waiting at goal!!";
					composite_vertex[agent_id] = make_threeple(source_vertex,source_vertex,0);

					heuristic += maps[agent_id][source_vertex].heuristic;
				}
				else
				{
					Vertex target_vertex = target_vertices[agent_id][selected_target_indices[agent_id]];
					size_t steps = steps_vec[agent_id];

					if(steps == min_steps)
						composite_vertex[agent_id] = make_threeple(target_vertex,target_vertex,0);
					else
						composite_vertex[agent_id] = make_threeple(source_vertex,target_vertex,t_values[agent_id] + min_steps);

					total_steps += min_steps;
					size_t steps_left = steps - min_steps;
					heuristic += maps[agent_id][target_vertex].heuristic + steps_left*epsilon_value;
				}	
			}

			if(indToCompositeMap.count(composite_vertex))
			{
				CompositeVertex new_node = indToCompositeMap[composite_vertex];
				// std::cout<<"Node fetched from cache: "<<new_node<<std::endl;
				bool edge_status = edge(node, new_node, composite_map).second;
				if(!edge_status)
				{
					std::pair<CompositeEdge,bool> curEdge = boost::add_edge(node, new_node, composite_map);
					composite_map[curEdge.first].length = total_steps*epsilon_value;
					composite_map[curEdge.first].prior = 1.0;
				}
				neighbors.push_back(new_node);
			}
			else
			{
				CompositeVertex new_node;
				new_node = add_vertex(composite_map);
				composite_map[new_node].composite_vertex = composite_vertex;
				composite_map[new_node].vertex_index = boost::num_vertices(composite_map) - 1;
				composite_map[new_node].heuristic = heuristic;

				indToCompositeMap[composite_vertex]=new_node;

				// Edge curEdge;
				std::pair<CompositeEdge,bool> curEdge = boost::add_edge(node, new_node, composite_map);
				composite_map[curEdge.first].length = total_steps*epsilon_value;
				composite_map[curEdge.first].prior = 1.0;

				neighbors.push_back(new_node);
			}

			return;
		}

		if(t_values[current_agent_id] == 0) // waiting 
		{
			// std::cout<<"considering waiting for agent: "<<current_agent_id<<std::endl;
			selected_target_indices[current_agent_id] = -1;
			nestedLoops(current_agent_id+1,neighbors,goal_vertices,selected_target_indices,source_vertices,target_vertices,t_values,node,composite_map,maps,goal,delta_values,min_rejected_delta);
		}   
		for(int i=0; i<target_vertices[current_agent_id].size(); i++)
		{
			selected_target_indices[current_agent_id] = i;
			nestedLoops(current_agent_id+1,neighbors,goal_vertices,selected_target_indices,source_vertices,target_vertices,t_values,node,composite_map,maps,goal,delta_values,min_rejected_delta);      
		}
	}

	std::vector<CompositeVertex> getNeighborsImplicitTPG(CompositeVertex &node, CompositeGraph &composite_map, std::vector<Graph> &maps, CompositeVertex &goal, int &delta_f, int &min_rejected_delta)
	{
		std::vector<CompositeVertex> neighbors;

		if(neighborsAddedSet.find(node) != neighborsAddedSet.end() )
		{
			CompositeOutEdgeIter out_i, out_end;
			for (boost::tie(out_i, out_end) = out_edges(node, composite_map); out_i != out_end; ++out_i) 
			{
				neighbors.push_back(target(*out_i,composite_map));
			}
			return neighbors;
		}

		neighborsAddedSet.insert(node);

		std::vector <Vertex> goal_vertices(mNumAgents);

		for(int agent_id=0;agent_id<mNumAgents; agent_id++)
			goal_vertices[agent_id] = threeple_first(composite_map[goal].composite_vertex[agent_id]);

		std::vector <Vertex> source_vertices(mNumAgents);
		std::vector <std::vector<Vertex>> target_vertices(mNumAgents,std::vector<Vertex>());
		std::vector <size_t> t_values(mNumAgents);

		for(int agent_id=0;agent_id<mNumAgents; agent_id++)
		{
			source_vertices[agent_id] = threeple_first(composite_map[node].composite_vertex[agent_id]);

			if(threeple_third(composite_map[node].composite_vertex[agent_id]) == 0)
			{
				NeighborIter ai, ai_end;
				for (boost::tie(ai, ai_end) = adjacent_vertices(source_vertices[agent_id], maps[agent_id]); ai != ai_end; ++ai) 
				{   
					Edge ind_edge = getEdge(maps[agent_id],source_vertices[agent_id],*ai);
					if( maps[agent_id][ind_edge].status == CollisionStatus::FREE)
						target_vertices[agent_id].push_back(*ai);
				}
				t_values[agent_id] = 0;
			}
			else
			{
				target_vertices[agent_id].push_back(threeple_second(composite_map[node].composite_vertex[agent_id]));
				t_values[agent_id] = threeple_third(composite_map[node].composite_vertex[agent_id]);
			}
		}

		std::vector<int> selected_target_indices(mNumAgents);
		std::vector<int> delta_values(mNumAgents);
		nestedLoops(0,neighbors,goal_vertices,selected_target_indices,source_vertices,target_vertices,t_values,node,composite_map,maps,goal,delta_values,min_rejected_delta);       

		return neighbors;
	}

	std::vector<CompositeVertex> getNeighborsDeltaFImplicitTPG(CompositeVertex &node, CompositeGraph &composite_map, std::vector<Graph> &maps, CompositeVertex &goal, int &delta_f, int &min_rejected_delta)
	{
		std::vector<CompositeVertex> neighbors;

		std::vector <Vertex> goal_vertices(mNumAgents);

		for(int agent_id=0;agent_id<mNumAgents; agent_id++)
			goal_vertices[agent_id] = threeple_first(composite_map[goal].composite_vertex[agent_id]);

		std::vector <Vertex> source_vertices(mNumAgents);
		std::vector <std::vector<Vertex>> target_vertices(mNumAgents,std::vector<Vertex>());
		std::vector <std::vector<int>> considered_deltas(mNumAgents,std::vector<int>());
		std::vector <size_t> t_values(mNumAgents);
		std::vector <bool> at_goal(mNumAgents,false);
		for(int agent_id=0;agent_id<mNumAgents; agent_id++)
		{
			source_vertices[agent_id] = threeple_first(composite_map[node].composite_vertex[agent_id]);

			if(threeple_third(composite_map[node].composite_vertex[agent_id]) == 0)
			{
				
				if(source_vertices[agent_id] == goal_vertices[agent_id])
				{
					at_goal[agent_id] = true;
					// std::cout<<" Goal-Vertex ";
				}
				// else 
					// std::cout<<" Vertex ";
				NeighborIter ai, ai_end;
				std::pair<size_t,Vertex> key = std::make_pair(agent_id,source_vertices[agent_id]);
				auto curMap = OSF[key];
				std::vector<int> temp_deltas;
				for(auto i = curMap.begin(); i!=curMap.end();i++)
					temp_deltas.push_back(i->first);
				sort(temp_deltas.begin(),temp_deltas.end());
				for(int i=0; i<temp_deltas.size(); i++)
				{
					considered_deltas[agent_id].push_back(temp_deltas[i]);
					if(temp_deltas[i] > delta_f)
						break;
				}
				considered_deltas[agent_id].push_back(-1); // can wait also
				for(int i=0; i<=delta_f; i++)
					if(OSF[key].find(i) != OSF[key].end())
						for(auto succ: OSF[key][i])
							target_vertices[agent_id].push_back(succ);

				t_values[agent_id] = 0;
			}
			else
			{
				// std::cout<<" Edge ";
				target_vertices[agent_id].push_back(threeple_second(composite_map[node].composite_vertex[agent_id]));
				t_values[agent_id] = threeple_third(composite_map[node].composite_vertex[agent_id]);
				considered_deltas[agent_id].push_back(0);
			}
		}
		// std::cout<<"\n";

		std::vector<int> selected_target_indices(mNumAgents);
		std::vector<int> delta_values(mNumAgents);
		nestedLoops(0,neighbors,goal_vertices,selected_target_indices,source_vertices,target_vertices,t_values,node,composite_map,maps,goal,delta_values,min_rejected_delta);       

		getMinRejectedDelta(0,considered_deltas,delta_values,min_rejected_delta,delta_f,at_goal);

		return neighbors;
	}

	size_t getSteps(Eigen::VectorXd config_one, Eigen::VectorXd config_two) 
	{
		return std::ceil((config_one-config_two).norm()*(1/epsilon_value)-0.000000001); 
	}
};

}  // namespace C_MINT

#endif