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

#define bind_all_values(a,b,c,d,e) std::make_pair(std::make_pair(std::make_pair(a,b),std::make_pair(c,d)),e)

#define alltype std::pair< std::pair< std::pair<Vertex,Vertex>, std::pair<Vertex,Vertex> >, size_t >

namespace C_MINT_2 {

using namespace BGL_DEFINITIONS;

class TensorPG
{
public: //Change to private once this is working

	boost::unordered_set<CompositeVertex> neighborsAddedSet;

	/// left_vertex, left_target_vertex, right_vertex, right_target_vertex, t_value
	boost::unordered_map< alltype, CompositeVertex> indToCompositeMap;

public:

	TensorPG()
	{
		// Do Nothing
	}

	void populateMaps(CompositeGraph &composite_map, CompositeVertex &start, CompositeVertex &goal)
	{
		// std::cout<<"P HERE"<<std::endl;
		indToCompositeMap[bind_all_values(composite_map[start].left_vertex, composite_map[start].left_vertex,
			composite_map[start].right_vertex, composite_map[start].right_vertex, 0)] = start;
		indToCompositeMap[bind_all_values(composite_map[goal].left_vertex,composite_map[goal].left_vertex,
			composite_map[goal].right_vertex,composite_map[goal].right_vertex, 0)] = goal;
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

	std::vector<CompositeVertex> getNeighborsImplicitTPG(CompositeVertex &node, CompositeGraph &composite_map, Graph &left_map, Graph &right_map, CompositeVertex &goal)
	{
		// std::cout<<"HERE"<<std::endl;
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

		Vertex left_goal_vertex = composite_map[goal].left_vertex;
		Vertex right_goal_vertex = composite_map[goal].right_vertex;

		NeighborIter ai_l, aiend_l, ai_r, aiend_r ;

		if(composite_map[node].state_type == 1) //vertex vertex
		{
			Vertex left_vertex = composite_map[node].left_vertex;
			Vertex right_vertex = composite_map[node].right_vertex;

			//tensor edges for type 1
			for (boost::tie(ai_l, aiend_l) = adjacent_vertices(left_vertex, left_map); ai_l != aiend_l; ++ai_l) 
			{
				for(boost::tie(ai_r,aiend_r) = adjacent_vertices(right_vertex,right_map);ai_r!=aiend_r; ++ai_r)
				{
					Edge left_edge = getEdge(left_map,left_vertex,*ai_l);
					Edge right_edge = getEdge(right_map,right_vertex,*ai_r);

					if( left_map[left_edge].status == CollisionStatus::FREE && right_map[right_edge].status == CollisionStatus::FREE)
					{	
						size_t left_steps = getSteps(left_map[left_vertex].state,left_map[*ai_l].state);
						size_t right_steps = getSteps(right_map[right_vertex].state, right_map[*ai_r].state);
						if(left_steps == right_steps)
						{
							addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
								1,*ai_l,*ai_l,*ai_r,*ai_r,0,false,false,left_steps); // = right_steps 
						}
						else if(left_steps < right_steps)
						{
							addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
								2,*ai_l,*ai_l,right_vertex,*ai_r,left_steps,false,false,left_steps);
						}
						else // left_steps > right_steps
						{
							addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
								3,left_vertex,*ai_l,*ai_r,*ai_r,right_steps,false,false,right_steps);
						}
					}
				}
			}

			// cartesian edges for type 1

			for (boost::tie(ai_l, aiend_l) = adjacent_vertices(left_vertex, left_map); ai_l != aiend_l; ++ai_l) 
			{
				Edge left_edge = getEdge(left_map,left_vertex,*ai_l);
				
				if( left_map[left_edge].status == CollisionStatus::FREE)
				{	
					size_t left_steps = getSteps(left_map[left_vertex].state,left_map[*ai_l].state);
					if (left_steps == 1)
					{
						addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
							1,*ai_l,*ai_l,right_vertex,right_vertex,0,false,true,1);				
					}
					else
					{
						addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
							3,left_vertex,*ai_l,right_vertex,right_vertex,1,false,true,1);	
					}
				}
			}


			for (boost::tie(ai_r, aiend_r) = adjacent_vertices(right_vertex, right_map); ai_r != aiend_r; ++ai_r) 
			{
				Edge right_edge = getEdge(right_map,right_vertex,*ai_r);

				if(right_map[right_edge].status == CollisionStatus::FREE)
				{	
					size_t right_steps = getSteps(right_map[right_vertex].state,right_map[*ai_r].state);
					if(right_steps == 1)
					{
						addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
							1,left_vertex,left_vertex,*ai_r,*ai_r,0,true,false,1);	
					}
					else
					{
						addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
							2,left_vertex,left_vertex,right_vertex,*ai_r,1,true,false,1);
					}
				}
			}
		}
		else if(composite_map[node].state_type == 2) //vertex edge
		{
			Vertex left_vertex = composite_map[node].left_vertex;
			
			Vertex right_source_vertex = composite_map[node].right_vertex;
			Vertex right_target_vertex = composite_map[node].right_target_vertex;

			size_t right_steps = getSteps(right_map[right_source_vertex].state,
				right_map[right_target_vertex].state);

			size_t right_steps_taken = composite_map[node].t_value;
			size_t right_steps_left = right_steps - right_steps_taken;

			for (boost::tie(ai_l, aiend_l) = adjacent_vertices(left_vertex, left_map); ai_l != aiend_l; ++ai_l) 
			{
				Edge left_edge = getEdge(left_map,left_vertex,*ai_l);

				if( left_map[left_edge].status == CollisionStatus::FREE)
				{	
					size_t left_steps = getSteps(left_map[left_vertex].state,left_map[*ai_l].state);
					if(left_steps == right_steps_left)
					{
						addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
							1,*ai_l,*ai_l,right_target_vertex,right_target_vertex,0,false,false,left_steps); // = right_steps_left
					}
					else if(left_steps < right_steps_left)
					{
						addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
							2,*ai_l,*ai_l,right_source_vertex,right_target_vertex,right_steps_taken + left_steps,false,false,left_steps);
					}
					else // left_edge_length < right_edge_length_left
					{
						addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
							3,left_vertex,*ai_l,right_target_vertex,right_target_vertex,right_steps_left,false,false,right_steps_left);					
					}
				}
			}		

			// cartesian movement of right edge

			if(right_steps_left == 1)
			{
				addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
					1,left_vertex,left_vertex,right_target_vertex,right_target_vertex,0,true,false,1);				
			}	
			else
			{
				addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
					2,left_vertex,left_vertex,right_source_vertex,right_target_vertex,right_steps_taken+1,true,false,1);
			}	
		}
		else // edge, vertex
		{
			Vertex left_source_vertex = composite_map[node].left_vertex;
			Vertex left_target_vertex = composite_map[node].left_target_vertex;

			Vertex right_vertex = composite_map[node].right_vertex;

			size_t left_steps = getSteps(left_map[left_source_vertex].state,
				left_map[left_target_vertex].state);

			size_t left_steps_taken = composite_map[node].t_value;
			size_t left_steps_left = left_steps - left_steps_taken;

			for (boost::tie(ai_r, aiend_r) = adjacent_vertices(right_vertex, right_map); ai_r != aiend_r; ++ai_r) 
			{
				Edge right_edge = getEdge(right_map,right_vertex,*ai_r);

				if(right_map[right_edge].status == CollisionStatus::FREE)
				{	
					double right_steps = getSteps(right_map[right_vertex].state, right_map[*ai_r].state);
					if(left_steps_left == right_steps)
					{
						addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
							1,left_target_vertex,left_target_vertex,*ai_r,*ai_r,0,false,false,right_steps); // = left_steps_left
					}
					else if(left_steps_left < right_steps)
					{
						addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
							2,left_target_vertex,left_target_vertex,right_vertex,*ai_r,left_steps_left,false,false,left_steps_left);
					}
					else // left_steps_left > right_steps
					{
						addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
							3,left_source_vertex,left_target_vertex,*ai_r,*ai_r,left_steps_taken+right_steps,false,false,right_steps);
					}
				}
			}	

			// cartesian movement of left edge
			if(left_steps_left == 1)
			{
				addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
						1,left_target_vertex,left_target_vertex,right_vertex,right_vertex,0,false,true,1);
			}
			else
			{
				addtoNeighbors(neighbors,node,composite_map,left_map,right_map,goal, 
						3,left_source_vertex,left_target_vertex,right_vertex,right_vertex,left_steps_taken+1,false,true,1);
			}	
		}

		return neighbors;
	}

	void addtoNeighbors(std::vector <CompositeVertex> &neighbors,
		CompositeVertex &node, CompositeGraph &composite_map, Graph &left_map, Graph &right_map, CompositeVertex &goal, 
		size_t state_type, 
		Vertex left_vertex, Vertex left_target_vertex, 
		Vertex right_vertex, Vertex right_target_vertex,
		size_t t_value,
		bool left_waiting, bool right_waiting,
		size_t steps_taken)
	{

		Vertex left_goal_vertex = composite_map[goal].left_vertex;
		Vertex right_goal_vertex = composite_map[goal].right_vertex;

		alltype map_entry = bind_all_values(left_vertex,left_target_vertex,right_vertex,right_target_vertex,t_value);
		
		if(indToCompositeMap.count(map_entry))
		{
			CompositeVertex new_node = indToCompositeMap[map_entry];
			bool edge_status = edge(node, new_node, composite_map).second;
			if(!edge_status)
			{
				std::pair<CompositeEdge,bool> curEdge = boost::add_edge(node, new_node, composite_map);
				if((left_waiting && left_vertex == left_goal_vertex) || 
					(right_waiting && right_vertex == right_goal_vertex))
				{
					composite_map[curEdge.first].length = epsilon_value;
				}
				else
					composite_map[curEdge.first].length = steps_taken*epsilon_value*2;
				composite_map[curEdge.first].prior = 1.0;
			}
			neighbors.push_back(new_node);
		}
		else
		{
			CompositeVertex new_node;
			new_node = add_vertex(composite_map);
			composite_map[new_node].state_type = state_type;
			composite_map[new_node].left_vertex = left_vertex;
			composite_map[new_node].left_target_vertex = left_target_vertex;
			composite_map[new_node].right_vertex = right_vertex;
			composite_map[new_node].right_target_vertex = right_target_vertex;
			composite_map[new_node].t_value = t_value;
			composite_map[new_node].vertex_index = boost::num_vertices(composite_map) - 1;

			if(state_type == 1)
			{
				composite_map[new_node].heuristic = left_map[left_vertex].heuristic + right_map[right_vertex].heuristic;
			}
			else if(state_type == 2)
			{
				double right_edge_length = epsilon_value * getSteps(right_map[right_vertex].state,
					right_map[right_target_vertex].state);
				composite_map[new_node].heuristic = left_map[left_vertex].heuristic+
					(right_edge_length - t_value*epsilon_value) + right_map[right_target_vertex].heuristic;
			}
			else // state = 3
			{
				double left_edge_length = epsilon_value * getSteps(left_map[left_vertex].state,
					left_map[left_target_vertex].state);
				composite_map[new_node].heuristic = (left_edge_length - t_value*epsilon_value) +
				left_map[left_target_vertex].heuristic + right_map[right_vertex].heuristic;
			}

			indToCompositeMap[map_entry]=new_node;

			// Edge curEdge;
			std::pair<CompositeEdge,bool> curEdge = boost::add_edge(node, new_node, composite_map);
			if((left_waiting && left_vertex == left_goal_vertex) || 
				(right_waiting && right_vertex == right_goal_vertex))
			{
				composite_map[curEdge.first].length = epsilon_value;
			}
			else
				composite_map[curEdge.first].length = steps_taken*epsilon_value*2;
			composite_map[curEdge.first].prior = 1.0;

			neighbors.push_back(new_node);
		}
	}

	size_t getSteps(Eigen::VectorXd config_one, Eigen::VectorXd config_two) 
	{
		return std::ceil((config_one-config_two).norm()*(1/epsilon_value)-0.000000001);	
	}
};

}  // namespace C_MINT_2

#endif