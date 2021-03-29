/* Authors: Kushal Kedia */

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

ICTS::ICTS(cv::Mat img, std::string& leftRoadmapFileName, std::string& rightRoadmapFileName,
		Eigen::VectorXd _start_config, Eigen::VectorXd _goal_config, bool insert_start_goal, bool use_dijkstra)
	: mImage(img)
	, mLeftRoadmapFileName(leftRoadmapFileName)
	, mRightRoadmapFileName(rightRoadmapFileName)
{

	Vertex left_start_vertex;
	Vertex left_goal_vertex;

	Vertex right_start_vertex;
	Vertex right_goal_vertex;

	Eigen::VectorXd left_start_config(2);
	for (size_t ui = 0; ui < 2; ui++)
	{
		left_start_config[ui] = _start_config[ui];
	}

	Eigen::VectorXd left_goal_config(2);
	for (size_t ui = 0; ui < 2; ui++)
	{
		left_goal_config[ui] = _goal_config[ui];
	}

	Eigen::VectorXd right_start_config(2);
	for (size_t ui = 2; ui < 4; ui++)
	{
		right_start_config[ui-2] = _start_config[ui];
	}

	Eigen::VectorXd right_goal_config(2);
	for (size_t ui = 2; ui < 4; ui++)
	{
		right_goal_config[ui-2] = _goal_config[ui];
	}

	if (insert_start_goal)
	{
		double mConnectionRadius = 0.2;

		create_vertices(left_graph,get(&VProp::state,left_graph),mLeftRoadmapFileName,2,get(&EProp::prior,left_graph));
		create_edges(left_graph,get(&EProp::length,left_graph));

		VertexIter ind_vi, ind_vi_end;
		size_t i=0;
		for (boost::tie(ind_vi, ind_vi_end) = vertices(left_graph); ind_vi != ind_vi_end; ++ind_vi,++i)
		{
			put(&VProp::vertex_index,left_graph,*ind_vi,i);
		}

		create_vertices(right_graph,get(&VProp::state,right_graph),mRightRoadmapFileName,2,get(&EProp::prior,right_graph));
		create_edges(right_graph,get(&EProp::length,right_graph));

		i=0;
		for (boost::tie(ind_vi, ind_vi_end) = vertices(right_graph); ind_vi != ind_vi_end; ++ind_vi,++i)
		{
			put(&VProp::vertex_index,right_graph,*ind_vi,i);
		}

		left_start_vertex = add_vertex(left_graph);
		left_graph[left_start_vertex].state = left_start_config;
		left_graph[left_start_vertex].vertex_index = boost::num_vertices(left_graph) - 1;

		left_goal_vertex = add_vertex(left_graph);
		left_graph[left_goal_vertex].state = left_goal_config;
		left_graph[left_goal_vertex].vertex_index = boost::num_vertices(left_graph) - 1;

		size_t startDegree = 0;
		size_t goalDegree = 0;

		for (boost::tie(ind_vi, ind_vi_end) = vertices(left_graph); ind_vi != ind_vi_end; ++ind_vi)
		{
			double startDist = getDistance(left_graph[left_start_vertex].state, left_graph[*ind_vi].state);
			double goalDist = getDistance(left_graph[left_goal_vertex].state, left_graph[*ind_vi].state);

			if (startDist < mConnectionRadius)
			{
				if(left_start_vertex == *ind_vi)
					continue;
				std::pair<Edge,bool> newEdge = boost::add_edge(left_start_vertex, *ind_vi, left_graph);
				left_graph[newEdge.first].length = startDist;
				left_graph[newEdge.first].prior = 1.0;
				left_graph[newEdge.first].isEvaluated = false;
				left_graph[newEdge.first].status = CollisionStatus::FREE;
				startDegree++;
			}

			if (goalDist < mConnectionRadius)
			{
				if(left_goal_vertex == *ind_vi)
					continue;
				std::pair<Edge,bool> newEdge = boost::add_edge(left_goal_vertex, *ind_vi, left_graph);
				left_graph[newEdge.first].length = goalDist;
				left_graph[newEdge.first].prior = 1.0;
				left_graph[newEdge.first].isEvaluated = false;
				left_graph[newEdge.first].status = CollisionStatus::FREE;
				goalDegree++;
			}
		}

		right_start_vertex = add_vertex(right_graph);
		right_graph[right_start_vertex].state = right_start_config;
		right_graph[right_start_vertex].vertex_index = boost::num_vertices(right_graph) - 1;


		right_goal_vertex = add_vertex(right_graph);
		right_graph[right_goal_vertex].state = right_goal_config;
		right_graph[right_goal_vertex].vertex_index = boost::num_vertices(right_graph) - 1;

		startDegree = 0;
		goalDegree = 0;

		// VertexIter ind_vi, ind_vi_end;
		for (boost::tie(ind_vi, ind_vi_end) = vertices(right_graph); ind_vi != ind_vi_end; ++ind_vi)
		{
			double startDist = getDistance(right_graph[right_start_vertex].state, right_graph[*ind_vi].state);
			double goalDist = getDistance(right_graph[right_goal_vertex].state, right_graph[*ind_vi].state);

			if (startDist < mConnectionRadius)
			{
				if(right_start_vertex == *ind_vi)
					continue;
				std::pair<Edge,bool> newEdge = boost::add_edge(right_start_vertex, *ind_vi, right_graph);
				right_graph[newEdge.first].length = startDist;
				right_graph[newEdge.first].prior = 1.0;
				right_graph[newEdge.first].isEvaluated = false;
				right_graph[newEdge.first].status = CollisionStatus::FREE;
				startDegree++;
			}

			if (goalDist < mConnectionRadius)
			{
				if(right_goal_vertex == *ind_vi)
					continue;
				std::pair<Edge,bool> newEdge = boost::add_edge(right_goal_vertex, *ind_vi, right_graph);
				right_graph[newEdge.first].length = goalDist;
				right_graph[newEdge.first].prior = 1.0;
				right_graph[newEdge.first].isEvaluated = false;
				right_graph[newEdge.first].status = CollisionStatus::FREE;
				goalDegree++;
			}
		}
	}
	else
	{
		create_vertices(left_graph,get(&VProp::state,left_graph),mLeftRoadmapFileName,2,get(&EProp::prior,left_graph));
		create_edges(left_graph,get(&EProp::length,left_graph));

		VertexIter ind_vi, ind_vi_end;
		size_t i=0;
		for (boost::tie(ind_vi, ind_vi_end) = vertices(left_graph); ind_vi != ind_vi_end; ++ind_vi,++i)
		{
			put(&VProp::vertex_index,left_graph,*ind_vi,i);
			if(left_start_config.isApprox(left_graph[*ind_vi].state))
				left_start_vertex = *ind_vi;
			if(left_goal_config.isApprox(left_graph[*ind_vi].state))
				left_goal_vertex = *ind_vi;	
		}

		create_vertices(right_graph,get(&VProp::state,right_graph),mRightRoadmapFileName,2,get(&EProp::prior,right_graph));
		create_edges(right_graph,get(&EProp::length,right_graph));

		i=0;
		for (boost::tie(ind_vi, ind_vi_end) = vertices(right_graph); ind_vi != ind_vi_end; ++ind_vi,++i)
		{
			put(&VProp::vertex_index,right_graph,*ind_vi,i);
			if(right_start_config.isApprox(right_graph[*ind_vi].state))
				right_start_vertex = *ind_vi;
			if(right_goal_config.isApprox(right_graph[*ind_vi].state))
				right_goal_vertex = *ind_vi;
		}
	}

	// std::cout<<"Left Graph: #Vertices: "<<boost::num_vertices(left_graph)<<" #Edges: "<<boost::num_edges(left_graph)<<std::endl;
	// std::cout<<"Right Graph: #Vertices: "<<boost::num_vertices(right_graph)<<" #Edges: "<<boost::num_edges(right_graph)<<std::endl;

	// std::cout<<"Press [ENTER] to preprocess graphs: ";
	// std::cin.get();

	preprocess_graph(left_graph, left_goal_vertex, use_dijkstra);
	preprocess_graph(right_graph, right_goal_vertex, use_dijkstra);

	// Add start and goal vertices to the graph
	mStartVertex = boost::add_vertex(graph); 
	graph[mStartVertex].state_type = 1;
	graph[mStartVertex].left_vertex = left_start_vertex;
	graph[mStartVertex].right_vertex = right_start_vertex;
	graph[mStartVertex].vertex_index = boost::num_vertices(graph) - 1;

	mGoalVertex = boost::add_vertex(graph);
	graph[mGoalVertex].state_type = 1;
	graph[mGoalVertex].left_vertex = left_goal_vertex;
	graph[mGoalVertex].right_vertex = right_goal_vertex;
	graph[mGoalVertex].vertex_index = boost::num_vertices(graph) - 1;

	// Assign default values
	graph[mStartVertex].distance = 0;
	graph[mStartVertex].heuristic = left_graph[left_start_vertex].heuristic + right_graph[right_start_vertex].heuristic;
	graph[mStartVertex].parent = -1;
	graph[mStartVertex].visited = false;
	graph[mStartVertex].status = CollisionStatus::FREE;

	graph[mGoalVertex].distance = std::numeric_limits<double>::infinity();
	graph[mGoalVertex].heuristic = 0;
	graph[mGoalVertex].parent = -1;
	graph[mGoalVertex].visited = false;
	graph[mGoalVertex].status = CollisionStatus::FREE;

	// Populate the tensor product generator class
	mTPG.populateMaps(graph,mStartVertex,mGoalVertex);
	// displayPath(std::vector<Eigen::VectorXd>());
}

ICTS::~ICTS()
{
	// Do nothing.
}

} // namespace ICTS

std::vector<CompositeVertex> ICTS::printPaths(vector<vector<int>> paths) {
    cout << "Paths:" << endl;

    for(auto path : paths) {
        for(int node : path) {
            cout << node << " ";
        }
        cout << endl;
    }
}

std::vector<CompositeVertex> C_MINT_2::ICTS_run(vector<Agent> agentList, list<Node> queue) {
    Node currentNode = queue.front();
    queue.pop_front();
    vector<int> optimalCostList = currentNode.data;

    cout << "Costs: ";
    printTree(&currentNode, 1);

    int maxCost = 0;
    for(int i : optimalCostList)
        if(i > maxCost)
            maxCost = i;

    CombinedGraph cg = CombinedGraph(maxCost); 
    if(getAtLeastOnePathPerAgentWithoutConflict(agentList, optimalCostList, &cg, maxCost)) 
    {
        // announce the happy news
        cout << "Ladies and Gentleman, we've got a solution!!" << endl;

        vector<vector<int>> pathsA, pathsB;

        getPathsFromCombinedGraph(maxCost, &cg, &pathsA, &pathsB);

        bool getOnlyTheFirstPath = true;
        
        // get Only the first path, remove all others
        int nrOfPaths = pathsA.size();
        for (int i = 0; i < nrOfPaths - 1; i++)
        {
            pathsA.pop_back();
            pathsB.pop_back();
        }
        
        printPaths(pathsA);
        printPaths(pathsB);

        vector<vector<int>> finalPaths;
        finalPaths.push_back(pathsA[0]);
        finalPaths.push_back(pathsB[0]);
    }
    else {
        // give error message
        cout << "No paths found" << endl;

        // generate the next level of the tree, add those nodes to the queue and go to the next node
        generateNextLevel(&currentNode);
        for(auto child : currentNode.children) {
            queue.push_back(child);
        }

        ICTS(agentList, queue);
    }
}

#endif // ICTS_ICTS_HPP_