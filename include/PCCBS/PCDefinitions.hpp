#ifndef PC_DEFINITIONS_
#define PC_DEFINITIONS_
#include <bits/stdc++.h>
#include "BGLDefinitions.hpp"
#include "LoadGraphfromFile.hpp"

namespace PCCBS {

using namespace boost;

	struct meta_data_t 
	{
		typedef vertex_property_tag kind;
	};

	// int count = 0;

	struct meta_data
	{
		std::pair <double, double> start;
		std::pair <double, double> goal;

		std::vector <int> agent_list;

		meta_data() {}
		meta_data(std::pair <double, double> _start, std::pair <double, double> _goal, 
			std::vector <int> _agent_list) : start(_start), goal(_goal), agent_list(_agent_list) {}
	};

	typedef property<meta_data_t, meta_data> MetaData;
	typedef adjacency_list<vecS, vecS, directedS, MetaData> PrecedenceConstraintGraph;

	typedef std::pair<int,int> Pair;

	typedef boost::graph_traits<PrecedenceConstraintGraph>::vertex_descriptor PCVertex;
	typedef std::vector< PCVertex > container;

	/// Boost graph out edge iterator
	typedef boost::graph_traits<PrecedenceConstraintGraph>::out_edge_iterator PCOutEdgeIter;
	typedef boost::graph_traits<PrecedenceConstraintGraph>::vertex_iterator PCVertexIter;

} // namespace PCCBS

#endif