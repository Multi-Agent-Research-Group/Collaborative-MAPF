/* Authors: Rajat Kumar Jenamani */

#ifndef C_MINT_HPP_
#define C_MINT_HPP_

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

namespace C_MINT_2 {

using namespace BGL_DEFINITIONS;

class C_MINT_2
{
public:

	C_MINT_2(cv::Mat img, std::string& leftRoadmapFileName, std::string& rightRoadmapFileName,
		Eigen::VectorXd _start_config, Eigen::VectorXd _goal_config, bool insert_start_goal, bool use_dijkstra);

	~C_MINT_2(void);

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

}; // class C_MINT_2

} // namespace C_MINT_2


namespace C_MINT_2
{

C_MINT_2::C_MINT_2(cv::Mat img, std::string& leftRoadmapFileName, std::string& rightRoadmapFileName,
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

C_MINT_2::~C_MINT_2()
{
	// Do nothing.
}

void C_MINT_2::displayPath(std::vector<Eigen::VectorXd> path)
{
	cv::Mat image;
	cv::cvtColor(mImage, image, CV_GRAY2BGR);
	// cv::Mat image = cv::merge(mImage,mImage,mImage);  // going from one to 3 channel
	int numberOfRows = image.rows;
	int numberOfColumns = image.cols;

	EdgeIter ei, ei_end;
	for(boost::tie(ei,ei_end) = edges(left_graph); ei!=ei_end;++ei)
	{
		cv::Point source_Point((int)(left_graph[source(*ei,left_graph)].state[0]*numberOfColumns), 
			(int)((1-left_graph[source(*ei,left_graph)].state[1])*numberOfColumns));
		cv::Point target_Point((int)(left_graph[target(*ei,left_graph)].state[0]*numberOfColumns), 
			(int)((1-left_graph[target(*ei,left_graph)].state[1])*numberOfColumns));
		cv::line(image, source_Point, target_Point, cv::Scalar(0, 255, 255), 2);
	}

	VertexIter vi, vi_end;
	for (boost::tie(vi, vi_end) = vertices(left_graph); vi != vi_end; ++vi)
	{
		double x_point = left_graph[*vi].state[0]*numberOfColumns;
		double y_point = (1 - left_graph[*vi].state[1])*numberOfRows;
		cv::Point centre_Point((int)x_point, (int)y_point);
		cv::circle(image, centre_Point, 4,  cv::Scalar(0, 150, 0), -1);
	}

	for(boost::tie(ei,ei_end) = edges(right_graph); ei!=ei_end;++ei)
	{
		cv::Point source_Point((int)(right_graph[source(*ei,right_graph)].state[0]*numberOfColumns), 
			(int)((1-right_graph[source(*ei,right_graph)].state[1])*numberOfColumns));
		cv::Point target_Point((int)(right_graph[target(*ei,right_graph)].state[0]*numberOfColumns), 
			(int)((1-right_graph[target(*ei,right_graph)].state[1])*numberOfColumns));
		cv::line(image, source_Point, target_Point, cv::Scalar(0, 255, 255), 2);
	}

	for (boost::tie(vi, vi_end) = vertices(right_graph); vi != vi_end; ++vi)
	{
		double x_point = right_graph[*vi].state[0]*numberOfColumns;
		double y_point = (1 - right_graph[*vi].state[1])*numberOfRows;
		cv::Point centre_Point((int)x_point, (int)y_point);
		cv::circle(image, centre_Point, 4,  cv::Scalar(0, 150, 0), -1);
	}

	// Get state count
	int pathSize = path.size();

	for (int i = 0; i < pathSize - 1; ++i)
	{
		Eigen::VectorXd u = path[i];
		Eigen::VectorXd v = path[i+1];

		cv::Point left_uPoint((int)(u[0]*numberOfColumns), (int)((1 - u[1])*numberOfRows));
		cv::Point left_vPoint((int)(v[0]*numberOfColumns), (int)((1 - v[1])*numberOfRows));	

		cv::line(image, left_uPoint, left_vPoint, cv::Scalar(0, 140, 255), 2);

		cv::Point right_uPoint((int)(u[2]*numberOfColumns), (int)((1 - u[3])*numberOfRows));
		cv::Point right_vPoint((int)(v[2]*numberOfColumns), (int)((1 - v[3])*numberOfRows));

		cv::line(image, right_uPoint, right_vPoint, cv::Scalar(0, 140, 255), 2);

		if(i==0)
		{
			cv::circle(image, left_uPoint, 6,  cv::Scalar(255,0,0), -1);
			cv::circle(image, right_uPoint, 6,  cv::Scalar(0,0,255), -1);
		}
		if(i==pathSize-2)
		{
			cv::circle(image, left_vPoint, 6,  cv::Scalar(255,0,0), -1);
			cv::circle(image, right_vPoint, 6,  cv::Scalar(0,0,255), -1);			
		}
	}

	cv::Mat new_image;
	for (int i = 0; i < pathSize - 1; ++i)
	{
		Eigen::VectorXd u = path[i];
		Eigen::VectorXd v = path[i+1];

		Eigen::VectorXd left_source_config = u.segment(0,2);
		Eigen::VectorXd left_target_config = v.segment(0,2);

		Eigen::VectorXd right_source_config = u.segment(2,2);
		Eigen::VectorXd right_target_config = v.segment(2,2);

		double resolution = 0.01;

		double left_edge_length = (left_source_config - left_target_config).norm();
		double right_edge_length = (right_source_config - right_target_config).norm();

		unsigned int left_nStates = std::ceil(left_edge_length / resolution)+1;
		unsigned int right_nStates = std::ceil( right_edge_length/ resolution)+1;

		if(left_nStates < 2u)
			left_nStates = 2u;

		if(right_nStates < 2u)
			right_nStates = 2u;

		// std::cout<<left_nStates<<" "<<right_nStates<<std::endl;
	
		for (unsigned int i = 0; i < std::max(left_nStates,right_nStates)-1; i++)
		{
			Eigen::VectorXd left_intermediate_config(2);
			Eigen::VectorXd right_intermediate_config(2);

			if(i < left_nStates - 1)
			{
				left_intermediate_config <<  left_source_config + 
				(resolution*i/left_edge_length)*(left_target_config-left_source_config);
			}
			else
				left_intermediate_config << left_target_config;

			if(i < right_nStates - 1)
				right_intermediate_config <<  right_source_config + 
				(resolution*i/right_edge_length)*(right_target_config-right_source_config);
			else
				right_intermediate_config << right_target_config;

			new_image = image.clone();

			double left_x_point = left_intermediate_config[0]*numberOfColumns;
			double left_y_point = (1 - left_intermediate_config[1])*numberOfRows;
			cv::Point left_Point((int)left_x_point, (int)left_y_point);

			double right_x_point = right_intermediate_config[0]*numberOfColumns;
			double right_y_point = (1 - right_intermediate_config[1])*numberOfRows;
			cv::Point right_Point((int)right_x_point, (int)right_y_point);

			// double distance_between_points = (left_x_point - right_x_point)*(left_x_point - right_x_point)
			// 	+ (left_y_point - right_y_point)*(left_y_point - right_y_point);
			// std::cout<<"Distance between agents: "<<distance_between_points<<std::endl;

			cv::circle(new_image, left_Point, 6,  cv::Scalar(255,0,0), -1);
			cv::circle(new_image, left_Point, 8,  cv::Scalar(255,0,0), 1);
			cv::circle(new_image, right_Point, 6,  cv::Scalar(0,0,255), -1);
			cv::circle(new_image, right_Point, 8,  cv::Scalar(0,0,255), 1);
			cv::namedWindow("Agents",cv::WINDOW_NORMAL);
			cv::imshow("Agents", new_image);
			cv::waitKey(100);
		}
		{
			new_image = image.clone();
			double left_x_point = v[0]*numberOfColumns;
			double left_y_point = (1 - v[1])*numberOfRows;
			cv::Point left_Point((int)left_x_point, (int)left_y_point);

			double right_x_point = v[2]*numberOfColumns;
			double right_y_point = (1 - v[3])*numberOfRows;
			cv::Point right_Point((int)right_x_point, (int)right_y_point);

			// double distance_between_points = (left_x_point - right_x_point)*(left_x_point - right_x_point)
			// 	+ (left_y_point - right_y_point)*(left_y_point - right_y_point);
			// std::cout<<"Distance between agents: "<<distance_between_points<<std::endl;

			cv::circle(new_image, left_Point, 6,  cv::Scalar(255,0,0), -1);
			cv::circle(new_image, left_Point, 8,  cv::Scalar(255,0,0), 1);
			cv::circle(new_image, right_Point, 6,  cv::Scalar(0,0,255), -1);
			cv::circle(new_image, right_Point, 8,  cv::Scalar(0,0,255), 1);
			
			cv::namedWindow("Agents",cv::WINDOW_NORMAL);
			cv::imshow("Agents", new_image);
			cv::waitKey(100);
		}
	}
	cv::namedWindow("Graph Visualization",cv::WINDOW_NORMAL);
	cv::imshow("Graph Visualization", image);
	cv::waitKey(0);
}

void C_MINT_2::preprocess_graph(Graph &g, Vertex & _goal, bool use_dijkstra)
{
	if(use_dijkstra)
	{
		boost::unordered_map<Vertex,bool> sptSet; 
		boost::unordered_map<Vertex,Vertex> parentMap;
		boost::unordered_map<Vertex,double> distanceMap;

		Vertex goal_vertex=_goal;

		VertexIter vi, viend;
		
		int numVertices=0;
	  	for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi) 
	  	{
	  		numVertices++;
	  		distanceMap[*vi]=std::numeric_limits<double>::infinity();
	  	}

	  	parentMap.clear();
	  	sptSet.clear();
	  	distanceMap[goal_vertex]=0;
	  	int totalVertices=numVertices+1;
	  	
	  	while(numVertices>1)
	  	{
	  		// std::cout<< "[INFO]: Preprocess Search "<<totalVertices-numVertices<<std::endl;

	  		// std::cout<<numVertices<<std::endl;
	  		// mindDistance
			double min_dist= std::numeric_limits<double>::infinity();
			Vertex min_vertex;
			for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi) 
		  	{
		  		if(!sptSet.count(*vi) && distanceMap[*vi]<min_dist)
		  		{
		  			min_dist = distanceMap[*vi];
		  			min_vertex = *vi;
		  		}
		  	}

	  		Vertex node = min_vertex;
	  		sptSet[node]=1;

			std::vector<Vertex> successors;
			OutEdgeIter ei, ei_end;

			for (boost::tie(ei, ei_end) = out_edges(node, g); ei != ei_end; ++ei) 
			{
	    		Vertex curSucc = target(*ei, g);
	    		successors.push_back(curSucc);
			}

	  		for(Vertex &successor : successors )
	  		{
	  			Edge uv;
				bool edgeExists;
				boost::tie(uv, edgeExists) = edge(node, successor, g);

	  			if( !sptSet.count(successor) && (distanceMap[successor] > distanceMap[node] + g[uv].length) )
	  			{
	  				distanceMap[successor] = distanceMap[node] + g[uv].length;
	  				parentMap[successor]=node;
	  			}
	  		}
	  		numVertices--;
	  	}

	  	for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi) 
	  	{
	  		g[*vi].heuristic = distanceMap[*vi];
	  		// std::cout<<"Index:"<<g[*vi].vertex_index<<" heuristic:"<<g[*vi].heuristic<<std::endl;
	  	}	
	}
	else
	{
		Vertex goal_vertex=_goal;
		VertexIter vi, viend;
	  	for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi) 
	  	{
	  		g[*vi].heuristic = std::abs(g[*vi].state[0] - g[goal_vertex].state[0])
	  			+ std::abs(g[*vi].state[1] - g[goal_vertex].state[1]);
	  		// std::cout<<"Index:"<<g[*vi].vertex_index<<" heuristic:"<<g[*vi].heuristic<<std::endl;
	  	}
	}
  	
  	// std::cout<<"Preprocessing Done. Press [ENTER] to continue:"<<std::endl;
  	// std::cin.get();
}

std::vector<CompositeVertex> C_MINT_2::getNeighbors(CompositeVertex &v)
{
	std::vector <CompositeVertex> neighbors;

	Vertex left_vertex = graph[v].left_vertex;
	Vertex right_vertex = graph[v].right_vertex;

	OutEdgeIter out_i, out_end;
	for (boost::tie(out_i, out_end) = out_edges(left_vertex, left_graph); out_i != out_end; ++out_i) 
	{
		Edge e = *out_i;
		if(!left_graph[e].isEvaluated)
			evaluateLeftEdge(e);
	}

	for (boost::tie(out_i, out_end) = out_edges(right_vertex, right_graph); out_i != out_end; ++out_i) 
	{
		Edge e = *out_i;
		if(!right_graph[e].isEvaluated)
			evaluateRightEdge(e);
	}

	neighbors = mTPG.getNeighborsImplicitTPG(v, graph, left_graph, right_graph, mGoalVertex);

	return neighbors;
}

std::vector<CompositeVertex> C_MINT_2::AStar()
{

	CompositeVertexIter vi, vi_end;
	for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
	{
	  graph[*vi].distance = std::numeric_limits<double>::infinity();
	  graph[*vi].visited = false;
	  graph[*vi].status = CollisionStatus::FREE;
	}

	// Priority Function: f-value
	auto cmpFValue = [&](const CompositeVertex& left, const CompositeVertex& right)
	{
		double estimateLeft = estimateTotalCost(left);
		double estimateRight = estimateTotalCost(right);

		if (estimateRight - estimateLeft > 0.00001)
			return true;
		if (estimateLeft - estimateRight > 0.00001)
			return false;
		if (left < right)
			return true;
		else
			return false;
	};

	std::set<CompositeVertex, decltype(cmpFValue)> qUseful(cmpFValue);

	bool solutionFound = false;

	graph[mStartVertex].distance = 0;
	graph[mStartVertex].parent = -1;
	graph[mStartVertex].visited = true;
	qUseful.insert(mStartVertex);


	size_t iteration=0;
	while(qUseful.size()!=0)
	{
		iteration++;
		CompositeVertex vTop = *qUseful.begin();
		qUseful.erase(qUseful.begin());
		if(vTop == mGoalVertex)
		{
			solutionFound = true;
			break;      
		}

		std::vector <CompositeVertex> neighbors = getNeighbors(vTop);

		for (auto ai = neighbors.begin() ; ai != neighbors.end(); ++ai) 
		{
			// displayGraph(graph);
			CompositeVertex successor = *ai; 
			CompositeEdge uv = getEdge(vTop,successor);
			if(!graph[uv].isEvaluated)
				evaluateEdge(uv);
			if(graph[uv].status == CollisionStatus::FREE)
			{
				double edgeLength = graph[uv].length;
				double new_cost = graph[vTop].distance + edgeLength;
				if(new_cost < graph[successor].distance)
				{
					qUseful.erase(successor);
					graph[successor].distance = new_cost;
					qUseful.insert(successor);
					graph[successor].parent= vTop;
				}	
			}	
		}
	}
	if (!solutionFound)
		return std::vector<CompositeVertex>();

	std::vector<CompositeVertex> path;
	
	CompositeVertex node = mGoalVertex;
	
	while(node!=mStartVertex)
	{
		path.push_back(node);
		node=graph[node].parent;
	}

	path.push_back(mStartVertex);
	std::reverse(path.begin(), path.end());
	return path;
}

// ===========================================================================================
std::vector<Eigen::VectorXd> C_MINT_2::solve()
{
	std::cout<<"IN SOLVE:!"<<std::endl;

	std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
  	
  	// std::cout<<"Press [ENTER] to start search:"<<std::endl;
  	// std::cin.get();

	start_time = std::chrono::system_clock::now();

	std::chrono::duration<double> elapsed_seconds;

	std::vector<CompositeVertex> path = AStar();

	if(path.size()>0)
	{
		std::vector<Eigen::VectorXd> path_configs;

		end_time = std::chrono::system_clock::now(); 

		std::cout<<"PATH: ";
		for(CompositeVertex &node: path )
		{
			Eigen::VectorXd path_config(4);

			if(graph[node].state_type == 1)
			{
				Vertex left_vertex = graph[node].left_vertex;
				Vertex right_vertex = graph[node].right_vertex;
				path_config << left_graph[left_vertex].state, right_graph[right_vertex].state;
			}
			else if(graph[node].state_type == 2)
			{
				Vertex left_vertex = graph[node].left_vertex;

				Vertex right_source_vertex = graph[node].right_vertex;
				Vertex right_target_vertex = graph[node].right_target_vertex;

				double edge_traversed = (graph[node].t_value*epsilon_value)/
					(right_graph[right_target_vertex].state - right_graph[right_source_vertex].state).norm();

				path_config << left_graph[left_vertex].state, right_graph[right_source_vertex].state +
					edge_traversed*(right_graph[right_target_vertex].state - right_graph[right_source_vertex].state);
			}
			else
			{
				Vertex left_source_vertex = graph[node].left_vertex;
				Vertex left_target_vertex = graph[node].left_target_vertex;

				Vertex right_vertex = graph[node].right_vertex;

				double edge_traversed = (graph[node].t_value*epsilon_value)/
					(left_graph[left_target_vertex].state - left_graph[left_source_vertex].state).norm();

				path_config << left_graph[left_source_vertex].state + 
					edge_traversed*(left_graph[left_target_vertex].state - left_graph[left_source_vertex].state),
				right_graph[right_vertex].state;				
			}
			
			std::cout<<graph[node].vertex_index<<" ";
			path_configs.push_back(path_config);

			// std::cout<<path_config[0]<<" "<<path_config[1]<<" "<<path_config[2]<<" "<<path_config[3]<<std::endl;
		}

		// for(auto config: path_configs)
		// 	std::cout<<config[0]<<" "<<config[1]<<" "<<config[2]<<" "<<config[3]<<std::endl;

		std::cout<<std::endl<<std::endl;

		std::cout<<"Path Length: "<<graph[mGoalVertex].distance<<std::endl;

		elapsed_seconds = (end_time - start_time);
		std::cout<< "Search Data: time, number of vertices, number of edges: "<<std::endl;
		std::cout<<elapsed_seconds.count()<<" "<<num_vertices(graph)<<" "<<num_edges(graph)<<std::endl;

		std::ofstream outfile;
		outfile.open("/home/rajat/melodic_ws/src/C-MINT/data/results/e_roadmaps.txt", std::ios_base::app);
		std::ostringstream ss;
		ss<<"C_MINT_2,";
		ss<<graph[mGoalVertex].distance<<",";
		ss<<elapsed_seconds.count()<<",";
		ss<<num_vertices(graph)<<","; //num_vertices
		ss<<num_edges(graph)<<"\n"; //num_edges
		std::string data_string = ss.str();
		outfile << data_string;
		outfile.close();
			
		// std::cout<<"Press [ENTER] to display path: ";
		// std::cin.get();
		// displayPath(path_configs);
		return path_configs;
	}

	else
	{
		std::cout<<"Solution NOT Found"<<std::endl;
		std::ofstream outfile;
		outfile.open("/home/rajat/melodic_ws/src/C-MINT/data/results/e_roadmaps.txt", std::ios_base::app);
		std::ostringstream ss;
		ss<<"C_MINT_2,";
		ss<<" SOLUTION NOT FOUND!! "; //sol not found
		ss<<"\n";
		std::string data_string = ss.str();
		outfile << data_string;
		outfile.close();
		return std::vector<Eigen::VectorXd>();
	}
}

bool C_MINT_2::evaluateIndividualConfig(Eigen::VectorXd config)
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

bool C_MINT_2::evaluateLeftEdge(const Edge& e) // returns false if in collision
{

	left_graph[e].isEvaluated = true;

	Vertex source_vertex = source(e, left_graph);
	Vertex target_vertex = target(e, left_graph);

	Eigen::VectorXd sourceState(2);
	sourceState << left_graph[source_vertex].state;

	Eigen::VectorXd targetState(2);
	targetState << left_graph[target_vertex].state;

	double resolution = 0.005;
	unsigned int nStates = std::ceil(left_graph[e].length / resolution-0.000000001)+1;

	// Just start and goal
	if(nStates < 2u)
	{
		nStates = 2u;
	}
	// std::cout<<"nStates:"<<nStates<<std::endl;

	bool checkResult = true;
	
	if (checkResult && !evaluateIndividualConfig(sourceState))
	{
		left_graph[source_vertex].status = CollisionStatus::BLOCKED;
		left_graph[e].status = CollisionStatus::BLOCKED;
		left_graph[e].length = INF;
		checkResult = false;
	}

	if (checkResult && !evaluateIndividualConfig(targetState))
	{
		left_graph[target_vertex].status = CollisionStatus::BLOCKED;
		left_graph[e].status = CollisionStatus::BLOCKED;
		left_graph[e].length = INF;
		checkResult = false;
	}

	if (checkResult)
	{
		// Evaluate the States in between
		for (unsigned int i = 1; i < nStates-1; i++)
		{

			if(!evaluateIndividualConfig(sourceState + (resolution*i/left_graph[e].length)*(targetState-sourceState) ))
			{
				left_graph[e].status = CollisionStatus::BLOCKED;
				left_graph[e].length = INF;
				checkResult = false;
				break;
			}
		}
	}

	return checkResult;
}

bool C_MINT_2::evaluateRightEdge(const Edge& e) // returns false if in collision
{
	right_graph[e].isEvaluated = true;

	Vertex source_vertex = source(e, right_graph);
	Vertex target_vertex = target(e, right_graph);

	Eigen::VectorXd sourceState(2);
	sourceState << right_graph[source_vertex].state;

	Eigen::VectorXd targetState(2);
	targetState << right_graph[target_vertex].state;

	double resolution = 0.005;
	unsigned int nStates = std::ceil(right_graph[e].length / resolution-0.000000001)+1;

	// Just start and goal
	if(nStates < 2u)
	{
		nStates = 2u;
	}
	// std::cout<<"nStates:"<<nStates<<std::endl;

	bool checkResult = true;
	
	if (checkResult && !evaluateIndividualConfig(sourceState))
	{
		right_graph[source_vertex].status = CollisionStatus::BLOCKED;
		right_graph[e].status = CollisionStatus::BLOCKED;
		right_graph[e].length = INF;
		checkResult = false;
	}

	if (checkResult && !evaluateIndividualConfig(targetState))
	{
		right_graph[target_vertex].status = CollisionStatus::BLOCKED;
		right_graph[e].status = CollisionStatus::BLOCKED;
		right_graph[e].length = INF;
		checkResult = false;
	}

	if (checkResult)
	{
		// Evaluate the States in between
		for (unsigned int i = 1; i < nStates-1; i++)
		{

			if(!evaluateIndividualConfig(sourceState + (resolution*i/right_graph[e].length)*(targetState-sourceState) ))
			{
				right_graph[e].status = CollisionStatus::BLOCKED;
				right_graph[e].length = INF;
				checkResult = false;
				break;
			}
		}
	}

	return checkResult;
}

bool C_MINT_2::evaluateConfig(Eigen::VectorXd config) // collision checking b/w left and right agent; both already checked with environment
{
	int numberOfRows = mImage.rows;
	int numberOfColumns = mImage.cols;

	//left agent
	double left_x_point = config[0]*numberOfColumns + 0.000001;
	double left_y_point = (1 - config[1])*numberOfRows + 0.000001;
	cv::Point left_point((int)left_x_point, (int)left_y_point);

	//right agent
	double right_x_point = config[2]*numberOfColumns + 0.000001;
	double right_y_point = (1 - config[3])*numberOfRows + 0.000001;
	cv::Point right_point((int)right_x_point, (int)right_y_point);

	// Collision check of left-agent and right-agent with each other
	if((right_x_point-left_x_point)*(right_x_point-left_x_point) + 
	(right_y_point-left_y_point)*(right_y_point-left_y_point) < 50 )
		return false;
	return true;
}

// ===========================================================================================
bool C_MINT_2::evaluateEdge(const CompositeEdge& e) // returns false if in collision
{
	graph[e].isEvaluated = true;

	CompositeVertex startVertex = source(e,graph);
	CompositeVertex endVertex   = target(e,graph);
	
	Eigen::VectorXd startState(4);

	if(graph[startVertex].state_type == 1)
	{
		Vertex left_vertex = graph[startVertex].left_vertex;
		Vertex right_vertex = graph[startVertex].right_vertex;
		startState << left_graph[left_vertex].state, right_graph[right_vertex].state;
	}
	else if(graph[startVertex].state_type == 2)
	{
		Vertex left_vertex = graph[startVertex].left_vertex;

		Vertex right_source_vertex = graph[startVertex].right_vertex;
		Vertex right_target_vertex = graph[startVertex].right_target_vertex;

		double edge_traversed = (graph[startVertex].t_value*epsilon_value)/
					getDistance(right_graph[right_target_vertex].state,right_graph[right_source_vertex].state);

		startState << left_graph[left_vertex].state, right_graph[right_source_vertex].state +
			edge_traversed*(right_graph[right_target_vertex].state - right_graph[right_source_vertex].state);
	}
	else
	{
		Vertex left_source_vertex = graph[startVertex].left_vertex;
		Vertex left_target_vertex = graph[startVertex].left_target_vertex;

		Vertex right_vertex = graph[startVertex].right_vertex;

		double edge_traversed = (graph[startVertex].t_value*epsilon_value)/
					getDistance(left_graph[left_target_vertex].state,left_graph[left_source_vertex].state);

		startState << left_graph[left_source_vertex].state + 
			edge_traversed*(left_graph[left_target_vertex].state - left_graph[left_source_vertex].state),
		right_graph[right_vertex].state;				
	}

	Eigen::VectorXd endState(4);
	
	if(graph[endVertex].state_type == 1)
	{
		Vertex left_vertex = graph[endVertex].left_vertex;
		Vertex right_vertex = graph[endVertex].right_vertex;
		endState << left_graph[left_vertex].state, right_graph[right_vertex].state;
	}
	else if(graph[endVertex].state_type == 2)
	{
		Vertex left_vertex = graph[endVertex].left_vertex;

		Vertex right_source_vertex = graph[endVertex].right_vertex;
		Vertex right_target_vertex = graph[endVertex].right_target_vertex;

		double edge_traversed = (graph[endVertex].t_value*epsilon_value)/
					getDistance(right_graph[right_target_vertex].state,right_graph[right_source_vertex].state);

		endState << left_graph[left_vertex].state, right_graph[right_source_vertex].state +
			edge_traversed*(right_graph[right_target_vertex].state - right_graph[right_source_vertex].state);
	}
	else
	{
		Vertex left_source_vertex = graph[endVertex].left_vertex;
		Vertex left_target_vertex = graph[endVertex].left_target_vertex;

		Vertex right_vertex = graph[endVertex].right_vertex;

		double edge_traversed = (graph[endVertex].t_value*epsilon_value)/
					getDistance(left_graph[left_target_vertex].state,left_graph[left_source_vertex].state);

		endState << left_graph[left_source_vertex].state + 
			edge_traversed*(left_graph[left_target_vertex].state - left_graph[left_source_vertex].state),
		right_graph[right_vertex].state;				
	}

	double resolution = 0.005;
	unsigned int nStates = std::ceil(graph[e].length / resolution-0.000000001)+1;

	// Just start and goal
	if(nStates < 2u)
	{
		nStates = 2u;
	}
	// std::cout<<"nStates:"<<nStates<<std::endl;

	bool checkResult = true;
	
	if (checkResult && !evaluateConfig(startState))
	{
		graph[startVertex].status = CollisionStatus::BLOCKED;
		graph[e].status = CollisionStatus::BLOCKED;
		graph[e].length = INF;
		checkResult = false;
	}

	if (checkResult && !evaluateConfig(endState))
	{
		graph[endVertex].status = CollisionStatus::BLOCKED;
		graph[e].status = CollisionStatus::BLOCKED;
		graph[e].length = INF;
		checkResult = false;
	}

	if (checkResult)
	{
		// Evaluate the States in between
		for (unsigned int i = 1; i < nStates-1; i++)
		{

			if(!evaluateConfig(startState + (resolution*i/graph[e].length)*(endState-startState) ))
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

// ===========================================================================================
CompositeEdge C_MINT_2::getEdge(CompositeVertex u, CompositeVertex v) const
{
	CompositeEdge uv;
	bool edgeExists;
	boost::tie(uv, edgeExists) = edge(u, v, graph);
	if (edgeExists == false)
	{
		std::cout<<"Edge does not exist. Press [ENTER] to get segmentation fault :( :"<<std::endl;
  		std::cin.get();
  	}

	return uv;
}

// ===========================================================================================
double C_MINT_2::estimateCostToCome(CompositeVertex v) const
{
	// return graph[v].distance + graph[v].lazyCostToCome;
	return graph[v].distance;
}

double C_MINT_2::heuristicFunction(CompositeVertex v) const
{
	return graph[v].heuristic;
}

double C_MINT_2::estimateTotalCost(CompositeVertex v) const
{
	return estimateCostToCome(v) + heuristicFunction(v);
}

double C_MINT_2::getDistance(Eigen::VectorXd config_one, Eigen::VectorXd config_two) 
{
	// double epsilon_value = 0.01;
	return epsilon_value*std::ceil((config_one-config_two).norm()*(1/epsilon_value)-0.000000001);
}

} // namespace C_MINT_2

#endif // C_MINT_C_MINT_HPP_
