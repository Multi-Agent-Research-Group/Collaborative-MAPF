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


namespace DENSIFY {

using namespace BGL_DEFINITIONS;

class TensorPG
{
public: //Change to private once this is working

	boost::unordered_set<CompositeVertex> neighborsAddedSet;

	boost::unordered_map< std::pair<DensifiedVertex,DensifiedVertex> , CompositeVertex> indToCompositeMap;
	// std::unordered_map< std::pair<DensifiedVertex,DensifiedVertex>,CompositeVertex, boost::hash< std::pair<DensifiedVertex,DensifiedVertex> > > indToCompositeMap;

public:

	TensorPG()
	{
		// Do Nothing
	}

	void populateMaps(CompositeGraph &composite_map, CompositeVertex &start, CompositeVertex &goal)
	{
		// std::cout<<"P HERE"<<std::endl;
		indToCompositeMap[std::make_pair(composite_map[start].left_vertex,
			composite_map[start].right_vertex)] = start;
		indToCompositeMap[std::make_pair(composite_map[goal].left_vertex,
			composite_map[goal].right_vertex)] = goal;
	}

	std::vector<CompositeVertex> getNeighborsImplicitTPG(CompositeVertex &node, CompositeGraph &composite_map, DensifiedGraph &left_map, DensifiedGraph &right_map, CompositeVertex &goal)
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

		DensifiedOutEdgeIter outi_l, outiend_l, outi_r, outiend_r;

		DensifiedVertex left_vertex = composite_map[node].left_vertex;
		DensifiedVertex right_vertex = composite_map[node].right_vertex;

		DensifiedVertex left_goal_vertex = composite_map[goal].left_vertex;
		DensifiedVertex right_goal_vertex = composite_map[goal].right_vertex;

		// if(right_vertex == right_goal_vertex)
		// {
		// 	std::cout<<std::endl<<std::endl<<"right at goal"<<std::endl<<std::endl;
		// 	std::cin.get();
		// }

		for(boost::tie(outi_l, outiend_l) = out_edges(left_vertex, left_map); outi_l != outiend_l; ++outi_l) 
		{
			for(boost::tie(outi_r, outiend_r)= out_edges(right_vertex, right_map); outi_r!=outiend_r; ++outi_r)
			{
				std::pair<DensifiedVertex,DensifiedVertex> adjacent_vertex = 
					std::make_pair(target(*outi_l,left_map),target(*outi_r,right_map));

				if(indToCompositeMap.count(adjacent_vertex))
				{
					CompositeVertex new_node = indToCompositeMap[adjacent_vertex];
					bool edge_status = edge(node, new_node, composite_map).second;
					if(!edge_status)
					{
						std::pair<CompositeEdge,bool> curEdge = boost::add_edge(node, new_node, composite_map);
						composite_map[curEdge.first].length = epsilon_value*2;
						composite_map[curEdge.first].prior = 1.0;
					}
					neighbors.push_back(new_node);
					continue;
				}

				CompositeVertex new_node;
				new_node = add_vertex(composite_map);
				composite_map[new_node].left_vertex = adjacent_vertex.first;
				composite_map[new_node].right_vertex = adjacent_vertex.second;
				composite_map[new_node].vertex_index = boost::num_vertices(composite_map) - 1;
				composite_map[new_node].heuristic = left_map[adjacent_vertex.first].heuristic + 
					right_map[adjacent_vertex.second].heuristic;				

				indToCompositeMap[adjacent_vertex]=new_node;

				// Edge curEdge;
				std::pair<CompositeEdge,bool> curEdge = boost::add_edge(node, new_node, composite_map);
				composite_map[curEdge.first].length = epsilon_value*2;
				composite_map[curEdge.first].prior = 1.0;

				neighbors.push_back(new_node);
			}
		}

		if(right_map[composite_map[node].right_vertex].isOriginal)
		{
			for (boost::tie(outi_l, outiend_l) = out_edges(left_vertex, left_map); outi_l != outiend_l; ++outi_l) 
			{
				std::pair<DensifiedVertex,DensifiedVertex> adjacent_vertex = 
					std::make_pair(target(*outi_l,left_map),right_vertex);

				if(indToCompositeMap.count(adjacent_vertex))
				{
					CompositeVertex new_node = indToCompositeMap[adjacent_vertex];
					bool edge_status = edge(node, new_node, composite_map).second;
					if(!edge_status)
					{
						std::pair<CompositeEdge,bool> curEdge = boost::add_edge(node, new_node, composite_map);
						if(right_vertex == right_goal_vertex)
							composite_map[curEdge.first].length = epsilon_value;
						else
							composite_map[curEdge.first].length = epsilon_value*2;
						composite_map[curEdge.first].prior = 1.0;
					}
					neighbors.push_back(new_node);
					continue;
				}

				CompositeVertex new_node;
				new_node = add_vertex(composite_map);
				composite_map[new_node].left_vertex = adjacent_vertex.first;
				composite_map[new_node].right_vertex = adjacent_vertex.second;
				composite_map[new_node].vertex_index = boost::num_vertices(composite_map) - 1;
				composite_map[new_node].heuristic = left_map[adjacent_vertex.first].heuristic + 
					right_map[adjacent_vertex.second].heuristic;

				indToCompositeMap[adjacent_vertex]=new_node;

				std::pair<CompositeEdge,bool> curEdge = boost::add_edge(node, new_node, composite_map);
				composite_map[curEdge.first].prior = 1.0;
				if(right_vertex == right_goal_vertex)
					composite_map[curEdge.first].length = epsilon_value;
				else
					composite_map[curEdge.first].length = epsilon_value*2;

				neighbors.push_back(new_node);
			}
		}

		if(left_map[composite_map[node].left_vertex].isOriginal)
		{
			for (boost::tie(outi_r, outiend_r) = out_edges(right_vertex, right_map); outi_r != outiend_r; ++outi_r) 
			{
				std::pair<DensifiedVertex,DensifiedVertex> adjacent_vertex = 
					std::make_pair(left_vertex,target(*outi_r,right_map));

				if(indToCompositeMap.count(adjacent_vertex))
				{
					CompositeVertex new_node = indToCompositeMap[adjacent_vertex];
					bool edge_status = edge(node, new_node, composite_map).second;
					if(!edge_status)
					{
						std::pair<CompositeEdge,bool> curEdge = boost::add_edge(node, new_node, composite_map);
						if(left_vertex == left_goal_vertex)
							composite_map[curEdge.first].length = epsilon_value;
						else
							composite_map[curEdge.first].length = epsilon_value*2;
						composite_map[curEdge.first].prior = 1.0;
					}
					neighbors.push_back(new_node);
					continue;
				}

				CompositeVertex new_node;
				new_node = add_vertex(composite_map);
				composite_map[new_node].left_vertex = adjacent_vertex.first;
				composite_map[new_node].right_vertex = adjacent_vertex.second;
				composite_map[new_node].vertex_index = boost::num_vertices(composite_map) - 1;
				composite_map[new_node].heuristic = left_map[adjacent_vertex.first].heuristic + 
					right_map[adjacent_vertex.second].heuristic;
				
				indToCompositeMap[adjacent_vertex]=new_node;

				std::pair<CompositeEdge,bool> curEdge = boost::add_edge(node, new_node, composite_map);
				composite_map[curEdge.first].prior = 1.0;
				if(left_vertex == left_goal_vertex)
					composite_map[curEdge.first].length = epsilon_value;
				else
					composite_map[curEdge.first].length = epsilon_value*2;

				neighbors.push_back(new_node);
			}
		}

		return neighbors;

	}
};

}  // namespace DENSIFY

#endif