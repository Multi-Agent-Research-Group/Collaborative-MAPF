#ifndef LOAD_GRAPH_FROM_FILE_
#define LOAD_GRAPH_FROM_FILE_

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

namespace PCICTS{

using namespace BGL_DEFINITIONS;

class RoadmapFromFilePutStateMap
{
public:
	typedef boost::property_traits<VPStateMap>::key_type key_type; 
	typedef std::string value_type;
	typedef boost::writable_property_map_tag category;
	typedef std::string reference;

	const VPStateMap mVPStateMap;
	const size_t mDim;

	RoadmapFromFilePutStateMap(VPStateMap _state_map, size_t _dimensions)
	: mVPStateMap {_state_map}
	, mDim { _dimensions}
	{
	}
};

inline std::string get(const RoadmapFromFilePutStateMap& map, const RoadmapFromFilePutStateMap::key_type &k)
{
	// std::cout<<"IN ABORTED"<<std::endl;
	// abort();
	Eigen::VectorXd stateConfig(map.mDim);
	stateConfig << map.mVPStateMap[k];
	std::ostringstream ss;
	for( size_t i=0; i<map.mDim;i++)
	{
		ss<<stateConfig[i];
		ss<<" ";
	}  
    
    std::string config_string = ss.str();

    return config_string; 

}

inline void put(const RoadmapFromFilePutStateMap& map, const RoadmapFromFilePutStateMap::key_type& k, const std::string representation)
{
	Eigen::VectorXd stateConfig(map.mDim);
	std::stringstream ss(representation);
	for( size_t i=0; i<map.mDim;i++)
	{
		ss>>stateConfig[i];
	}

	put(map.mVPStateMap, k , stateConfig);
}

void create_vertices(Graph& g, VPStateMap state_map, std::string filename, unsigned int dimensions, EPPriorMap priorMap)
{
	std::ifstream fp;
	fp.open(filename.c_str());

	boost::dynamic_properties dp;
	dp.property("state",RoadmapFromFilePutStateMap(state_map,dimensions));
	dp.property("prior",priorMap);

	boost::read_graphml(fp, g, dp);

	fp.close();
}

double getWeight(const Graph& g,const Vertex& u, const Vertex& v)
{
	// double epsilon_value = 0.01;
	double weight = (g[u].state - g[v].state).norm();
	return weight;
}
void create_edges(Graph& g, EPLengthMap weight_map)
{
	EdgeIter ei, ei_end;
	for(boost::tie(ei,ei_end)=edges(g);ei!=ei_end;++ei)
	{
		Vertex u=source(*ei,g);
		Vertex v=target(*ei,g);

		put(weight_map, *ei, getWeight(g,u,v));
	}
}
// void display_graph(Graph herb_map)
// {
// 	VertexIter vi, vend;
// 	VPIndexMap index_map = get(&VProp::vertex_index, herb_map);

//   	for (boost::tie(vi, vend) = vertices(herb_map); vi != vend; ++vi) {
//   		std::cout << index_map[(*vi)]<<std::endl;
//   		// std::cout<<" "<<"State: "<<herb_map[*vi].state<<std::endl ;
//   	}

//   	EdgeIter ei, ei_end;
// 	for(boost::tie(ei,ei_end)=edges(herb_map);ei!=ei_end;++ei) {
// 		std::cout<<"("<<index_map[source(*ei,herb_map)]<<","<<index_map[target(*ei,herb_map)]<<")"<<std::endl;
// 	}
// }

// void write_graph(Graph &g, std::string filename, unsigned int dimensions)
// {
// 	std::ofstream fp;
// 	fp.open(filename.c_str());

// 	boost::dynamic_properties dp;
// 	dp.property("state", RoadmapFromFilePutStateMap(get(&VProp::state,g),dimensions));
    
//     write_graphml(fp, g,get(&VProp::vertex_index, g), dp);


// 	fp.close();
// }

} // namespace PCICTS

#endif