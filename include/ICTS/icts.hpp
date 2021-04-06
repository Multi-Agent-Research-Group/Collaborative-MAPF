/* Authors: Kushal Kedia, Rajat Kumar Jenamani */

#ifndef ICTS_HPP_
#define ICTS_HPP_

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

#include "BGLDefinitions.hpp"
#include "LoadGraphfromFile.hpp"
#include "tensorProductGenerator.hpp"

#define INF std::numeric_limits<double>::infinity()

namespace ICTS {

using namespace BGL_DEFINITIONS;

class ICTS
{
public:

	ICTS(int num_agents);

	~ICTS(void);

	///////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////

	/// Get the distance between two Eigens
	double getDistance(Eigen::VectorXd config_one, Eigen::VectorXd config_two); 

	std::vector<Eigen::VectorXd> solve();

	/// Environment
	cv::Mat mImage;

	/// The fixed left_graph denoting the left agent
	Graph left_graph;

	/// The fixed right_graph denoting the right agent
	Graph right_graph;

	/// The fixed roadmap over which the search is done.
	CompositeGraph graph;

	//// Used to populate the CompositeGraph
	TensorPG mTPG;

	/// Path to the left roadmap.
	std::string mLeftRoadmapFileName;

	/// Path to the right roadmap.
	std::string mRightRoadmapFileName;

	///////////////////////////////////////////////////////////////////

	/// Source vertex.
	CompositeVertex mStartVertex;

	/// Goal vertex.
	CompositeVertex mGoalVertex;

	bool evaluateEdge(const CompositeEdge& e);
	bool evaluateConfig(Eigen::VectorXd config);

	bool evaluateLeftEdge(const Edge& e);
	bool evaluateRightEdge(const Edge& e);
	bool evaluateIndividualConfig(Eigen::VectorXd config);

	CompositeEdge getEdge(CompositeVertex u, CompositeVertex v) const;

	double estimateCostToCome(CompositeVertex vertex) const;

	double heuristicFunction(CompositeVertex vertex) const;

	double estimateTotalCost(CompositeVertex vertex) const;

	std::vector<CompositeVertex> getNeighbors(CompositeVertex &v);

	/// A Star
	std::vector<CompositeVertex> AStar();

	/// Populate heuristic function 
	void preprocess_graph(Graph &g, Vertex & _goal, bool use_dijkstra);

	void displayPath(std::vector<Eigen::VectorXd> path);

}; // class ICTS

} // namespace ICTS

namespace ICTS
{

ICTS::ICTS()
{
	struct meta_data_t {
		typedef vertex_property_tag kind;
	};

	struct meta_data{
	std::pair <int, int> start;
	std::pair <int, int> goal;

	std::vector <int> agent_list;

	int  start_time;
	int end_time;

	int slack = 0;
	};

	typedef property<meta_data_t, meta_data> MetaData;
	typedef adjacency_list<vecS, vecS, directedS, 
	                     MetaData> PrecedenceConstraintGraph;

	typedef std::pair<int,int> Pair;

	typedef boost::graph_traits<PrecedenceConstraintGraph>::vertex_descriptor Vertex;
	typedef std::vector< Vertex > container;

	Pair edge_array[11] = { Pair(0,1), Pair(1,2), Pair(2,5), 
	                        Pair(3,5), Pair(4,6), Pair(5,7), 
	                        Pair(5,8), Pair(6,9), Pair(7,10), 
	                        Pair(8,11), Pair(9,11) };
	  
	PrecedenceConstraintGraph G(12);

	for (int i = 0; i < 11; ++i)
	  add_edge(edge_array[i].first, edge_array[i].second, G);

	property_map<PrecedenceConstraintGraph, meta_data_t>::type
	name = get(meta_data_t(), G);
	  
	meta_data random = {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0, 1}, 0, 5, 10,};

	name[0] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0}, 0, 5, 0,};
	name[1] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0}, 6, 15, 0,};
	name[2] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0}, 16, 20, 0,};
	name[3] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {1}, 0, 10, 10,};
	name[4] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {2}, 0, 10, 0,};
	name[5] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0, 1}, 21, 30, 0,};
	name[6] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {2}, 10, 15, 0,};
	name[7] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0}, 31, 40, 0,};
	name[8] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {1}, 31, 35, 0,};
	name[9] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {2}, 16, 25, 10,};
	name[10] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0, 1}, 41, 50, 0,};
	name[11] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0, 1}, 36, 45, 5,};

	container c;
	topological_sort(G, std::back_inserter(c));

	std::cout << "A topological ordering: ";
	for ( container::reverse_iterator ii=c.rbegin(); ii!=c.rend(); ++ii)
	{
	  std::cout << std::endl;
	  meta_data vertex = get(name, *ii);
	  std::cout << vertex.start_time << " ";
	  std::cout << vertex.end_time << " ";   
	  for(auto agent: vertex.agent_list){
	    std::cout << agent << " ";   
	  }
	}
	std::cout <<std::endl;
}

ICTS::~ICTS()
{
	// Do nothing.
}

} // namespace ICTS

struct Node{
	vector <int> cost;
	vector <vector <GraphVertex>> paths;
	vector <Node> children;
};

std::vector <vector <GraphVertex>> ICTS::ICTS_run(std::vector<Agent> agentList, std::list<Node> queue) {
    Node currentNode = queue.front();
    queue.pop_front();
    vector<int> optimalCostList = currentNode.cost;

    cout << "Costs: ";
    printTree(&currentNode, 1);

    int maxCost = 0;
    for(int i : optimalCostList)
        if(i > maxCost)
            maxCost = i;

    CombinedGraph cg = CombinedGraph(maxCost); 
    if(PathsCollisionFree(currentNode.paths))
    {
        return currentNode.paths;
    }
    else {
        // give error message
        cout << "Paths in Collision" << endl;

        // generate the next level of the tree, add those nodes to the queue and go to the next node
        generateNextLevel(&currentNode);

        for(auto child : currentNode.children) {
            queue.push_back(child);
        }

        ICTS_run(agentList, queue);
    }
}

#endif // ICTS_ICTS_HPP_