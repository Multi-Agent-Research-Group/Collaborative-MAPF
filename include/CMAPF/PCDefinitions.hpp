#ifndef PC_DEFINITIONS_
#define PC_DEFINITIONS_
#include <bits/stdc++.h>
#include "BGLDefinitions.hpp"
#include "LoadGraphfromFile.hpp"

namespace CMAPF {

using namespace boost;

	struct meta_data_t 
	{
		typedef vertex_property_tag kind;
	};

	// int count = 0;

	struct meta_data
	{
		std::pair <float, float> start;
		std::pair <float, float> goal;

		std::vector <int> agent_list;

		int task_id;

		meta_data() {}
		meta_data(std::pair <float, float> _start, std::pair <float, float> _goal, std::vector <int> _agent_list, int _task_id) : start(_start), goal(_goal), agent_list(_agent_list), task_id(_task_id) {}
	};

	typedef property<meta_data_t, meta_data> MetaData;
	typedef adjacency_list<vecS, vecS, directedS, MetaData> PrecedenceConstraintGraph;

	typedef std::pair<int,int> Pair;

	typedef boost::graph_traits<PrecedenceConstraintGraph>::vertex_descriptor PCVertex;
	typedef std::vector< PCVertex > container;

	/// Boost graph out edge iterator
	typedef boost::graph_traits<PrecedenceConstraintGraph>::out_edge_iterator PCOutEdgeIter;
	typedef boost::graph_traits<PrecedenceConstraintGraph>::vertex_iterator PCVertexIter;

} // namespace CMAPF

#endif