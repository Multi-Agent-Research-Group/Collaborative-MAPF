 
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

namespace C_MINT {

using namespace BGL_DEFINITIONS;

class C_MINT
{
public:

	C_MINT(cv::Mat img, size_t numAgents, std::vector<std::string> roadmapFileNames, Eigen::VectorXd _start_config, Eigen::VectorXd _goal_config, bool insert_start_goal, bool use_dijkstra);

	~C_MINT(void);

	///////////////////////////////////////////////////////////////////

	/// Get the distance between two Eigens
	double getDistance(Eigen::VectorXd config_one, Eigen::VectorXd config_two); 

	std::vector<Eigen::VectorXd> solve();

	/// Environment
	cv::Mat mImage;

	/// The fixed graphs denoting individual environment of corresponding agents
	std::vector<Graph> mGraphs;

	/// Number of agents
	size_t mNumAgents; 

	/// The fixed roadmap over which the search is done.
	CompositeGraph mCompositeGraph;

	//// Used to populate the CompositeGraph
	TensorPG mTPG;

	/// Path to the roadmap files.
	std::vector<std::string> mRoadmapFileNames;

	/// Source vertex.
	CompositeVertex mStartVertex;

	/// Goal vertex.
	CompositeVertex mGoalVertex;

	double queue_operations_time = 0.0;
	double get_neighbors_time = 0.0;
	double iterate_neighbors_time = 0.0;
	double collision_checking_time = 0.0;
	double pre_processing_time = 0.0;
	long long int number_expansions = 0;
	long long int queue_max_size = 0;
	long long int neighbors_iterated = 0;
	long long int neighbors_added_in_queue = 0;

	int mPlanningStatus; // 0-> exited cleanly 1-> break due to time overflow 2-> break due to memory overflow

	bool evaluateEdge(const CompositeEdge& e);
	bool evaluateConfig(Eigen::VectorXd config);

	bool evaluateIndividualEdge(Graph &graph, Edge& e);
	bool evaluateIndividualConfig(Eigen::VectorXd config);

	CompositeEdge getEdge(CompositeVertex u, CompositeVertex v) const;

	double estimateCostToCome(CompositeVertex vertex) const;

	double heuristicFunction(CompositeVertex vertex) const;

	double estimateTotalCost(CompositeVertex vertex) const;

	std::vector<CompositeVertex> getNeighbors(CompositeVertex &v, int &delta_f, int &min_rejected_delta);
	std::vector<CompositeVertex> getNeighborsDeltaF(CompositeVertex &v, int &delta_f, int &min_rejected_delta);

	/// A Star
	std::vector<CompositeVertex> PEAStar();

	std::vector<CompositeVertex> EPEAStar();

	/// Populate heuristic function 
	void preprocess_graph(Graph &g, Vertex & _goal, bool use_dijkstra);

	void displayPath(std::vector<Eigen::VectorXd> path);

}; // class C_MINT

} // namespace C_MINT


namespace C_MINT
{

C_MINT::C_MINT(cv::Mat img, size_t numAgents, std::vector<std::string> roadmapFileNames, Eigen::VectorXd _start_config, Eigen::VectorXd _goal_config, bool insert_start_goal, bool use_dijkstra)
	: mImage(img)
	, mNumAgents(numAgents)
	, mRoadmapFileNames(roadmapFileNames)
{

	std::vector<Vertex> start_vertices;
	std::vector<Vertex> goal_vertices;

	std::vector<Eigen::VectorXd> start_configs;
	std::vector<Eigen::VectorXd> goal_configs;

	for(int i=0; i<mNumAgents;i++)
	{
		Eigen::VectorXd start_config(2);
		for (size_t ui = i*2; ui < i*2+2; ui++)
			start_config[ui-i*2] = _start_config[ui];
		start_configs.push_back(start_config);
	}

	for(int i=0; i<mNumAgents;i++)
	{
		Eigen::VectorXd goal_config(2);
		for (size_t ui = i*2; ui < i*2+2; ui++)
			goal_config[ui-i*2] = _goal_config[ui];
		goal_configs.push_back(goal_config);
	}

	if (insert_start_goal)
	{
		double mConnectionRadius = 0.2;

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			Graph graph;
			create_vertices(graph,get(&VProp::state,graph),mRoadmapFileNames[agent_id],2,get(&EProp::prior,graph));
			create_edges(graph,get(&EProp::length,graph));

			VertexIter ind_vi, ind_vi_end;
			size_t i=0;
			for (boost::tie(ind_vi, ind_vi_end) = vertices(graph); ind_vi != ind_vi_end; ++ind_vi,++i)
			{
				put(&VProp::vertex_index,graph,*ind_vi,i);
			}

			Vertex start_vertex = add_vertex(graph);
			graph[start_vertex].state = start_configs[agent_id];
			graph[start_vertex].vertex_index = boost::num_vertices(graph) - 1;

			Vertex goal_vertex = add_vertex(graph);
			graph[goal_vertex].state = goal_configs[agent_id];
			graph[goal_vertex].vertex_index = boost::num_vertices(graph) - 1;

			size_t startDegree = 0;
			size_t goalDegree = 0;

			for (boost::tie(ind_vi, ind_vi_end) = vertices(graph); ind_vi != ind_vi_end; ++ind_vi)
			{
				double startDist = getDistance(graph[start_vertex].state, graph[*ind_vi].state);
				double goalDist = getDistance(graph[goal_vertex].state, graph[*ind_vi].state);

				if (startDist < mConnectionRadius)
				{
					if(start_vertex == *ind_vi)
						continue;
					std::pair<Edge,bool> newEdge = boost::add_edge(start_vertex, *ind_vi, graph);
					graph[newEdge.first].length = startDist;
					graph[newEdge.first].prior = 1.0;
					graph[newEdge.first].isEvaluated = false;
					graph[newEdge.first].status = CollisionStatus::FREE;
					startDegree++;
				}

				if (goalDist < mConnectionRadius)
				{
					if(goal_vertex == *ind_vi)
						continue;
					std::pair<Edge,bool> newEdge = boost::add_edge(goal_vertex, *ind_vi, graph);
					graph[newEdge.first].length = goalDist;
					graph[newEdge.first].prior = 1.0;
					graph[newEdge.first].isEvaluated = false;
					graph[newEdge.first].status = CollisionStatus::FREE;
					goalDegree++;
				}
			}

			mGraphs.push_back(graph);
			start_vertices.push_back(start_vertex);
			goal_vertices.push_back(goal_vertex);
		}
	}
	else
	{
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			Graph graph;
			Vertex start_vertex;
			Vertex goal_vertex;

			create_vertices(graph,get(&VProp::state,graph),mRoadmapFileNames[agent_id],2,get(&EProp::prior,graph));
			create_edges(graph,get(&EProp::length,graph));

			VertexIter ind_vi, ind_vi_end;
			size_t i=0;
			for (boost::tie(ind_vi, ind_vi_end) = vertices(graph); ind_vi != ind_vi_end; ++ind_vi,++i)
			{
				put(&VProp::vertex_index,graph,*ind_vi,i);
				if(start_configs[agent_id].isApprox(graph[*ind_vi].state))
					start_vertex = *ind_vi;
				if(goal_configs[agent_id].isApprox(graph[*ind_vi].state))
					goal_vertex = *ind_vi;  
			}

			mGraphs.push_back(graph);
			start_vertices.push_back(start_vertex);
			goal_vertices.push_back(goal_vertex);
		}
	}

	for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		preprocess_graph(mGraphs[agent_id], goal_vertices[agent_id], use_dijkstra);

	// Add start and goal vertices to the graph
	mStartVertex = boost::add_vertex(mCompositeGraph); 
	for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		mCompositeGraph[mStartVertex].composite_vertex.push_back(make_threeple(start_vertices[agent_id],start_vertices[agent_id],0));
	mCompositeGraph[mStartVertex].vertex_index = boost::num_vertices(mCompositeGraph) - 1;

	mGoalVertex = boost::add_vertex(mCompositeGraph);
	for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		mCompositeGraph[mGoalVertex].composite_vertex.push_back(make_threeple(goal_vertices[agent_id],goal_vertices[agent_id],0));
	mCompositeGraph[mGoalVertex].vertex_index = boost::num_vertices(mCompositeGraph) - 1;

	// Assign default values
	mCompositeGraph[mStartVertex].distance = 0;
	mCompositeGraph[mStartVertex].heuristic = 0;
	for(int agent_id=0; agent_id<mNumAgents; agent_id++) 
		mCompositeGraph[mStartVertex].heuristic += mGraphs[agent_id][start_vertices[agent_id]].heuristic;
	mCompositeGraph[mStartVertex].parent = -1;
	mCompositeGraph[mStartVertex].visited = false;
	mCompositeGraph[mStartVertex].status = CollisionStatus::FREE;

	mCompositeGraph[mGoalVertex].distance = std::numeric_limits<double>::infinity();
	mCompositeGraph[mGoalVertex].heuristic = 0;
	mCompositeGraph[mGoalVertex].parent = -1;
	mCompositeGraph[mGoalVertex].visited = false;
	mCompositeGraph[mGoalVertex].status = CollisionStatus::FREE;

	// Populate the tensor product generator class
	mTPG.populateMaps(mCompositeGraph,mStartVertex,mGoalVertex,mGraphs);	
}

C_MINT::~C_MINT()
{
	// Do nothing.
}

void C_MINT::displayPath(std::vector<Eigen::VectorXd> path)
{
	cv::Mat image;
	cv::cvtColor(mImage, image, CV_GRAY2BGR);
	// cv::Mat image = cv::merge(mImage,mImage,mImage);  // going from one to 3 channel
	int numberOfRows = image.rows;
	int numberOfColumns = image.cols;

    std::vector<cv::Mat4b> number_images(10);
    for(int i=0; i<number_images.size(); i++)
    {
    	std::stringstream ss;
		ss << "/home/rajat/Downloads/numbers_BTP/";
		ss << (i+1);
		ss << ".png";
    	number_images[i] = imread(ss.str(), cv::IMREAD_UNCHANGED);
    	double scale = 0.037;
    	if(i>0)
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

	for (int i = 0; i < pathSize - 1; ++i)
	{
		Eigen::VectorXd u = path[i];
		Eigen::VectorXd v = path[i+1];

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			cv::Point uPoint((int)(u[2*agent_id]*numberOfColumns), (int)((1 - u[2*agent_id+1])*numberOfRows));
			cv::Point vPoint((int)(v[2*agent_id]*numberOfColumns), (int)((1 - v[2*agent_id+1])*numberOfRows));  
	
			if(i==0)
			{
				std::string text = "S" + std::to_string(agent_id+1);
				cv::circle(image, uPoint, 7,  cv::Scalar(255,255,255), -1);
				cv::circle(image, uPoint, 8,  cv::Scalar(0,0,0), 1);
				cv::putText(image, text, cv::Point(uPoint.x - 6,uPoint.y+3), cv::FONT_HERSHEY_PLAIN, 0.6, cvScalar(0,0,0), 1, 4);
			}
			if(i==pathSize-2)
			{
				std::string text = "G" + std::to_string(agent_id+1);
				cv::circle(image, vPoint, 7,  cv::Scalar(255,255,255), -1);
				cv::circle(image, vPoint, 8,  cv::Scalar(0,0,0), 1);
				cv::putText(image, text, cv::Point(vPoint.x - 6,vPoint.y+3), cv::FONT_HERSHEY_PLAIN, 0.6, cvScalar(0,0,0), 1, 4);
			}
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
				// cv::circle(new_image, _Point, 8,  cv::Scalar(0,255,0), 1);
				int x = x_point - number_images[agent_id].cols/2;
				int y = y_point - number_images[agent_id].rows/2;
				double alpha = 0.9; // alpha in [0,1]

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
				double alpha = 0.9; // alpha in [0,1]

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

void C_MINT::preprocess_graph(Graph &g, Vertex & _goal, bool use_dijkstra)
{
	auto begin = std::chrono::high_resolution_clock::now();
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
		
		while(numVertices>0)
		{
			// std::cout<< "[INFO]: Preprocess Search "<<totalVertices-numVertices<<std::endl;

			// std::cout<<numVertices<<std::endl;
			// mindDistance
			double min_dist= std::numeric_limits<double>::infinity();
			Vertex min_vertex;
			for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi) 
			{
				if(!sptSet.count(*vi) && distanceMap[*vi]<=min_dist)
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
				Edge e = *ei;
				if(!g[e].isEvaluated)
					evaluateIndividualEdge(g,e);
				if(g[e].status == CollisionStatus::FREE)
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
	pre_processing_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();

	// std::cout<<"Preprocessing Done. Press [ENTER] to continue:"<<std::endl;
	// std::cin.get();
}

std::vector<CompositeVertex> C_MINT::getNeighbors(CompositeVertex &v, int &delta_f, int &min_rejected_delta)
{
	std::vector <CompositeVertex> neighbors;

	for(int agent_id=0; agent_id <mNumAgents; agent_id++)
	{
		Vertex vertex = threeple_first(mCompositeGraph[v].composite_vertex[agent_id]);
		OutEdgeIter out_i, out_end;
		for (boost::tie(out_i, out_end) = out_edges(vertex, mGraphs[agent_id]); out_i != out_end; ++out_i) 
		{
			Edge e = *out_i;
			if(!mGraphs[agent_id][e].isEvaluated)
				evaluateIndividualEdge(mGraphs[agent_id],e);
		}
	}   
	neighbors = mTPG.getNeighborsImplicitTPG(v, mCompositeGraph, mGraphs, mGoalVertex, delta_f, min_rejected_delta);
	return neighbors;
}

std::vector<CompositeVertex> C_MINT::getNeighborsDeltaF(CompositeVertex &v, int &delta_f, int &min_rejected_delta)
{
	std::vector <CompositeVertex> neighbors;

	for(int agent_id=0; agent_id <mNumAgents; agent_id++)
	{
		Vertex vertex = threeple_first(mCompositeGraph[v].composite_vertex[agent_id]);
		OutEdgeIter out_i, out_end;
		for (boost::tie(out_i, out_end) = out_edges(vertex, mGraphs[agent_id]); out_i != out_end; ++out_i) 
		{
			Edge e = *out_i;
			if(!mGraphs[agent_id][e].isEvaluated)
				evaluateIndividualEdge(mGraphs[agent_id],e);
		}
	}   
	neighbors = mTPG.getNeighborsDeltaFImplicitTPG(v, mCompositeGraph, mGraphs, mGoalVertex, delta_f, min_rejected_delta);
	return neighbors;
}


std::vector<CompositeVertex> C_MINT::PEAStar()
{
	mPlanningStatus = 0;

	CompositeVertexIter vi, vi_end;
	for (boost::tie(vi, vi_end) = vertices(mCompositeGraph); vi != vi_end; ++vi)
	{
	  mCompositeGraph[*vi].distance = std::numeric_limits<double>::infinity();
	  mCompositeGraph[*vi].visited = false;
	  mCompositeGraph[*vi].status = CollisionStatus::FREE;
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

	mCompositeGraph[mStartVertex].distance = 0;
	mCompositeGraph[mStartVertex].parent = -1;
	mCompositeGraph[mStartVertex].visited = true;
	qUseful.insert(mStartVertex);

	std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
	start_time = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds;

	size_t iteration=0;
	while(qUseful.size()!=0)
	{
		end_time = std::chrono::system_clock::now(); 
		elapsed_seconds = (end_time - start_time);

		if(elapsed_seconds.count() > 100)
		{
			mPlanningStatus = 1;
			return std::vector<CompositeVertex>();
		}
		else if( num_vertices(mCompositeGraph) > 5000000 || num_edges(mCompositeGraph) > 50000000)
		{
			mPlanningStatus = 2;
			return std::vector<CompositeVertex>();
		}

		auto begin = std::chrono::high_resolution_clock::now();
		iteration++;
		CompositeVertex vTop = *qUseful.begin();
		qUseful.erase(qUseful.begin());
		// std::cout<<"Top Node: "<<vTop<<std::endl;
		queue_operations_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();

		number_expansions++;

		int delta_f = 0;
		double vTop_fval = mCompositeGraph[vTop].distance + mCompositeGraph[vTop].heuristic;
		if(mCompositeGraph[vTop].overestimate == true)
		{
			// std::cout<<"Re-expansion! ";
			// std::cout<<" - "<<mCompositeGraph[vTop].over_fval - vTop_fval<<std::endl;
			delta_f = (mCompositeGraph[vTop].over_fval - vTop_fval + 0.000001)/epsilon_value;
			// if(delta_f > 3)
			// 	std::cout<<"Re-expansion delta: "<<delta_f<<" Node:"<<vTop<<std::endl;
				// std::cin.get();
			vTop_fval = mCompositeGraph[vTop].over_fval;
		}
		// std::cout<<std::endl;
		if(vTop == mGoalVertex)
		{
			solutionFound = true;
			break;      
		}

		begin = std::chrono::high_resolution_clock::now();
		int min_rejected_delta = INT_MAX;
		std::vector <CompositeVertex> neighbors = getNeighbors(vTop,delta_f,min_rejected_delta);
		// std::cout<<"Neighbors Size: "<<neighbors.size()<<std::endl;
		// std::cout<<"Neighbors: ";
		// std::sort(neighbors.begin(), neighbors.end());
		// for(auto succ: neighbors)
		// 	std::cout<<succ<<" ";
		// std::cout<<"\n";
		// std::cout<<"min_rejected_delta: "<<min_rejected_delta<<std::endl;
		// std::cin.get();
		get_neighbors_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();

		// std::cout<<"vTop_fval - "<<vTop_fval<<std::endl;

		double min_rejected_fval = INF;
		auto iter_begin = std::chrono::high_resolution_clock::now();
		int neighbors_iterated_size = 0;
		int neighbors_queued_size = 0;
		int neighbors_collision_free_size = 0;
		for (auto ai = neighbors.begin() ; ai != neighbors.end(); ++ai) 
		{
			// displayGraph(graph);
			CompositeVertex successor = *ai; 
			CompositeEdge uv = getEdge(vTop,successor);

			double lazy_edgeLength = mCompositeGraph[uv].length;
			double lazy_new_cost = mCompositeGraph[vTop].distance + lazy_edgeLength + mCompositeGraph[successor].heuristic;

			// std::cout<<lazy_new_cost<<" "<<mCompositeGraph[successor].distance + mCompositeGraph[successor].heuristic<<" - "
			// 	<<mCompositeGraph[successor].distance<<" + "<<mCompositeGraph[successor].heuristic<<std::endl;

			// std::cout<<"successor dis: "<<mCompositeGraph[successor].distance<<"\n";

			double lazy_f_value = std::min(lazy_new_cost,mCompositeGraph[successor].distance + mCompositeGraph[successor].heuristic);
			// std::cout<<"L - "<<lazy_f_value<<"\n";

			if(lazy_f_value < vTop_fval + 0.0000001)
			{
				neighbors_iterated_size++;
				neighbors_iterated++;
				begin = std::chrono::high_resolution_clock::now();
				if(!mCompositeGraph[uv].isEvaluated)
					evaluateEdge(uv);
				collision_checking_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();

				if(mCompositeGraph[uv].status == CollisionStatus::FREE)
				{
					neighbors_collision_free_size++;
					double edgeLength = mCompositeGraph[uv].length;
					double new_cost = mCompositeGraph[vTop].distance + edgeLength;
					if(new_cost < mCompositeGraph[successor].distance)
					{
						neighbors_queued_size++;
						neighbors_added_in_queue++;
						begin = std::chrono::high_resolution_clock::now();
						qUseful.erase(successor);
						mCompositeGraph[successor].distance = new_cost;
						qUseful.insert(successor);
						mCompositeGraph[successor].parent= vTop;
						queue_operations_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();
					}   
				} 
			}
			else // reject
			{
				min_rejected_fval = std::min(min_rejected_fval,lazy_f_value);
				// std::cout<<"Rejecting! ";
			}
		}
		iterate_neighbors_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - iter_begin).count();

		// std::cout<<"Neighbors Iterated Size: "<<neighbors_iterated_size<<std::endl;
		// std::cout<<"Neighbors Collision Free Size: "<<neighbors_collision_free_size<<std::endl;
		// std::cout<<"Neighbors Queued Size: "<<neighbors_queued_size<<std::endl;
		// if(min_rejected_delta != INT_MAX)
		// {
		// 	begin = std::chrono::high_resolution_clock::now();
		// 	mCompositeGraph[vTop].overestimate = true;
		// 	mCompositeGraph[vTop].over_fval = mCompositeGraph[vTop].distance + mCompositeGraph[vTop].heuristic + min_rejected_delta*epsilon_value;
		// 	qUseful.insert(vTop);
		// 	queue_operations_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();
		// }
		if(min_rejected_fval != INF)
		{
			begin = std::chrono::high_resolution_clock::now();
			mCompositeGraph[vTop].overestimate = true;
			mCompositeGraph[vTop].over_fval = min_rejected_fval;
			qUseful.insert(vTop);
			queue_operations_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();
		}
		// std::cin.get();
		if(qUseful.size() > queue_max_size)
		{
			queue_max_size = qUseful.size();
			// check max unique elements
		}
	}
	if (!solutionFound)
		return std::vector<CompositeVertex>();

	std::cout<<"Iterations: "<<iteration<<std::endl;

	std::vector<CompositeVertex> path;
	
	CompositeVertex node = mGoalVertex;
	
	while(node!=mStartVertex)
	{
		path.push_back(node);
		node=mCompositeGraph[node].parent;
	}

	path.push_back(mStartVertex);
	std::reverse(path.begin(), path.end());
	return path;
}

std::vector<CompositeVertex> C_MINT::EPEAStar()
{

	mPlanningStatus = 0;

	CompositeVertexIter vi, vi_end;
	for (boost::tie(vi, vi_end) = vertices(mCompositeGraph); vi != vi_end; ++vi)
	{
	  mCompositeGraph[*vi].distance = std::numeric_limits<double>::infinity();
	  mCompositeGraph[*vi].visited = false;
	  mCompositeGraph[*vi].status = CollisionStatus::FREE;
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

	mCompositeGraph[mStartVertex].distance = 0;
	mCompositeGraph[mStartVertex].parent = -1;
	mCompositeGraph[mStartVertex].visited = true;
	qUseful.insert(mStartVertex);

	std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
	start_time = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds;

	size_t iteration=0;

	size_t re_expansions = 0;
	while(qUseful.size()!=0)
	{
		end_time = std::chrono::system_clock::now(); 
		elapsed_seconds = (end_time - start_time);

		if(elapsed_seconds.count() > 100)
		{
			mPlanningStatus = 1;
			return std::vector<CompositeVertex>();
		}
		else if( num_vertices(mCompositeGraph) > 5000000 || num_edges(mCompositeGraph) > 50000000)
		{
			mPlanningStatus = 2;
			return std::vector<CompositeVertex>();
		}

		auto begin = std::chrono::high_resolution_clock::now();
		iteration++;
		CompositeVertex vTop = *qUseful.begin();
		qUseful.erase(qUseful.begin());
		// std::cout<<"Top Node: "<<vTop<<std::endl;
		queue_operations_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();

		number_expansions++;

		int delta_f = 0;
		double vTop_fval = mCompositeGraph[vTop].distance + mCompositeGraph[vTop].heuristic;
		if(mCompositeGraph[vTop].overestimate == true)
		{
			re_expansions++;
			// std::cout<<"Re-expansion! ";
			// std::cout<<" - "<<mCompositeGraph[vTop].over_fval - vTop_fval<<std::endl;
			delta_f = (mCompositeGraph[vTop].over_fval - vTop_fval + 0.000001)/epsilon_value;
			// if(delta_f > 3)
			// 	std::cout<<"Re-expansion delta: "<<delta_f<<" Node:"<<vTop<<std::endl;
				// std::cin.get();
			vTop_fval = mCompositeGraph[vTop].over_fval;
			// std::cout<<std::endl;
		}
		if(vTop == mGoalVertex)
		{
			solutionFound = true;
			break;      
		}

		begin = std::chrono::high_resolution_clock::now();
		int min_rejected_delta = INT_MAX;
		// std::vector <CompositeVertex> neighbors = getNeighbors(vTop,delta_f,min_rejected_delta);
		std::vector <CompositeVertex> neighbors = getNeighborsDeltaF(vTop,delta_f,min_rejected_delta);
		// for(auto sn: some_neighbors)
			// if(std::find(neighbors.begin(), neighbors.end(), sn) == neighbors.end())
				// std::cout<<"Neighbor not found!!\n";
		// std::cout<<"delta_f: "<<delta_f<<"\n";
		// std::cout<<"#Vertices: "<<num_vertices(mCompositeGraph)<<" #Edges: "<<num_edges(mCompositeGraph)<<std::endl;
		// std::cout<<"Neighbors Size: "<<neighbors.size()<<std::endl;
		// std::cout<<"Neighbors: ";
		// std::sort(neighbors.begin(), neighbors.end());
		// for(auto succ: neighbors)
			// std::cout<<succ<<" ";
		// std::cout<<"\n";
		// std::cout<<"min_rejected_delta: "<<min_rejected_delta<<std::endl;
		// std::cin.get();
		get_neighbors_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();

		// std::cout<<"vTop_fval - "<<vTop_fval<<std::endl;

		double min_rejected_fval = INF;
		auto iter_begin = std::chrono::high_resolution_clock::now();
		int neighbors_iterated_size = 0;
		int neighbors_queued_size = 0;
		int neighbors_collision_free_size = 0;
		for (auto ai = neighbors.begin() ; ai != neighbors.end(); ++ai) 
		{
			// displayGraph(graph);
			CompositeVertex successor = *ai; 
			CompositeEdge uv = getEdge(vTop,successor);

			double lazy_edgeLength = mCompositeGraph[uv].length;
			double lazy_new_cost = mCompositeGraph[vTop].distance + lazy_edgeLength + mCompositeGraph[successor].heuristic;

			// std::cout<<lazy_new_cost<<" "<<mCompositeGraph[successor].distance + mCompositeGraph[successor].heuristic<<" - "
			// 	<<mCompositeGraph[successor].distance<<" + "<<mCompositeGraph[successor].heuristic<<std::endl;

			// std::cout<<"successor dis: "<<mCompositeGraph[successor].distance<<"\n";

			// double lazy_f_value = std::min(lazy_new_cost,mCompositeGraph[successor].distance + mCompositeGraph[successor].heuristic);
			double lazy_f_value = lazy_new_cost;
			// std::cout<<"L - "<<lazy_f_value<<"\n";

			// neighbors_iterated_size++;
			// neighbors_iterated++;

			double edgeLength = mCompositeGraph[uv].length;
			double new_cost = mCompositeGraph[vTop].distance + edgeLength;
			// std::cout<<"Succ: "<<successor<<" val: "<<lazy_f_value - mCompositeGraph[vTop].distance - mCompositeGraph[vTop].heuristic<<" lazy f val: "<<lazy_f_value<<" vTop_F: "<<vTop_fval<<std::endl;
			if(lazy_f_value < vTop_fval + 0.0000001)
			{
				if(new_cost < mCompositeGraph[successor].distance)
				{
					begin = std::chrono::high_resolution_clock::now();
					if(!mCompositeGraph[uv].isEvaluated)
						evaluateEdge(uv);
					collision_checking_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();

					if(mCompositeGraph[uv].status == CollisionStatus::FREE)
					{
						neighbors_collision_free_size++;
						// if(std::find(neighborsDeltaF.begin(),neighborsDeltaF.end(),successor) == neighborsDeltaF.end())
						// {
						// 	std::cout<<"Node f val: "<<mCompositeGraph[vTop].distance + mCompositeGraph[vTop].heuristic;
						// 	std::cout<<"\nNode overflow val: "<<mCompositeGraph[vTop].over_fval;
						// 	std::cout<<"\n\nSuccessor old f val: "<<mCompositeGraph[successor].distance + mCompositeGraph[successor].heuristic;
						// 	std::cout<<"\nSuccessor new f val: "<<mCompositeGraph[vTop].distance + lazy_edgeLength + mCompositeGraph[successor].heuristic;
						// 	std::cout<<"\nsuccessor f val: "<<lazy_f_value;
						// 	int val_delta = (lazy_edgeLength + mCompositeGraph[successor].heuristic - mCompositeGraph[vTop].heuristic + 0.000001)/epsilon_value;
						// 	std::cout<<"\n\nSet delta_f: "<<delta_f<<" value: "<<val_delta<<"\n";	
						// 	std::cout<<"WTF! "<<successor;
						// 	std::cout<<"\nneighborsDeltaF: ";
						// 	for(int i=0; i<neighborsDeltaF.size(); i++)
						// 		std::cout<<neighborsDeltaF[i]<<" ";
						// 	std::cout<<"\nneighbors: ";
						// 	for(int i=0; i<neighbors.size(); i++)
						// 		std::cout<<neighbors[i]<<" ";
						// 	std::cin.get();
						// }
						neighbors_queued_size++;
						neighbors_added_in_queue++;
						begin = std::chrono::high_resolution_clock::now();
						qUseful.erase(successor);
						mCompositeGraph[successor].distance = new_cost;
						qUseful.insert(successor);
						mCompositeGraph[successor].parent= vTop;
						queue_operations_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();
					} 
				} 
			} 
			else // reject
			{
				// std::cout<<"Succ: "<<successor<<" val: "<<lazy_f_value - mCompositeGraph[vTop].distance - mCompositeGraph[vTop].heuristic<<" lazy f val: "<<lazy_f_value<<" vTop_F: "<<vTop_fval<<std::endl;
				min_rejected_fval = std::min(min_rejected_fval,lazy_f_value);
				// std::cout<<"Rejecting! ";
			} 
		}
		iterate_neighbors_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - iter_begin).count();

		// std::cout<<"Neighbors Iterated Size: "<<neighbors_iterated_size<<std::endl;
		// std::cout<<"Neighbors Collision Free Size: "<<neighbors_collision_free_size<<std::endl;
		// std::cout<<"Neighbors Queued Size: "<<neighbors_queued_size<<std::endl;
		if(min_rejected_delta != INT_MAX)
		{
			begin = std::chrono::high_resolution_clock::now();
			mCompositeGraph[vTop].overestimate = true;
			mCompositeGraph[vTop].over_fval = mCompositeGraph[vTop].distance + mCompositeGraph[vTop].heuristic + min_rejected_delta*epsilon_value;
			qUseful.insert(vTop);
			queue_operations_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();
		}
		// if(min_rejected_fval != INF)
		// {
		// 	// if(int((min_rejected_fval+0.00001)/epsilon_value) != int(min_rejected_delta + (mCompositeGraph[vTop].distance + mCompositeGraph[vTop].heuristic + 0.000001)/epsilon_value))
		// 	// {
		// 	// 	std::cout<<"\nmin_rejected_fval: "<<int((min_rejected_fval+0.00001 - mCompositeGraph[vTop].distance - mCompositeGraph[vTop].heuristic)/epsilon_value);
		// 	// 	std::cout<<"\nmin_rejected_delta: "<<min_rejected_delta;
		// 	// 	std::cout<<"\nWTF!";
		// 	// 	std::cout<<"\nNeighbors: \n";
		// 	// 	for(auto succ: neighbors)
		// 	// 	{
		// 	// 		CompositeEdge uv = getEdge(vTop,succ);
		// 	// 		double edgeLength = mCompositeGraph[uv].length;

		// 	// 		std::cout<<"\nsucc: "<<succ<<"\n";
		// 	// 		std::cout<<edgeLength<<" "<<mCompositeGraph[succ].heuristic<<" "<<mCompositeGraph[vTop].heuristic<<"\n";
		// 	// 		int delta_val = (edgeLength + mCompositeGraph[succ].heuristic - mCompositeGraph[vTop].heuristic + 0.000001)/epsilon_value;
		// 	// 		std::cout<<"Delta val: "<<delta_val;
		// 	// 	}
		// 	// 	std::cout<<"\nDelta Neighbors: ";
		// 	// 	for(auto succ: neighborsDeltaF)
		// 	// 	{
		// 	// 		CompositeEdge uv = getEdge(vTop,succ);
		// 	// 		double edgeLength = mCompositeGraph[uv].length;
		// 	// 		int delta_val = (edgeLength + mCompositeGraph[succ].heuristic - mCompositeGraph[vTop].heuristic + 0.000001)/epsilon_value;
		// 	// 		std::cout<<succ<<" - "<<delta_val<<" | ";
		// 	// 	}
		// 	// 	std::cin.get();
		// 	// }
		// 	begin = std::chrono::high_resolution_clock::now();
		// 	mCompositeGraph[vTop].overestimate = true;
		// 	mCompositeGraph[vTop].over_fval = min_rejected_fval;
		// 	qUseful.insert(vTop);
		// 	queue_operations_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();
		// }
		// else
		// {
			// std::cout<<"NODE CLOSED!";
			// std::cin.get();
		// }
		// std::cin.get();
		if(qUseful.size() > queue_max_size)
		{
			queue_max_size = qUseful.size();
			// check max unique elements
		}
	}
	if (!solutionFound)
		return std::vector<CompositeVertex>();

	std::cout<<"iterations: "<<iteration<<" re_expansions: "<<re_expansions<<"\n";

	std::vector<CompositeVertex> path;
	
	CompositeVertex node = mGoalVertex;
	
	while(node!=mStartVertex)
	{
		path.push_back(node);
		node=mCompositeGraph[node].parent;
	}

	path.push_back(mStartVertex);
	std::reverse(path.begin(), path.end());
	return path;
}

// ===========================================================================================
std::vector<Eigen::VectorXd> C_MINT::solve()
{
	std::cout<<"IN SOLVE:!"<<std::endl;

	std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
	
	// std::cout<<"Press [ENTER] to start search:"<<std::endl;
	// std::cin.get();

	start_time = std::chrono::system_clock::now();

	std::chrono::duration<double> elapsed_seconds;

	std::vector<CompositeVertex> path = EPEAStar();

	end_time = std::chrono::system_clock::now(); 
	elapsed_seconds = (end_time - start_time);
	std::cout<< "Search Data: time, number of vertices, number of edges: "<<std::endl;
	std::cout<<elapsed_seconds.count() + pre_processing_time<<" "<<num_vertices(mCompositeGraph)<<" "<<num_edges(mCompositeGraph)<<std::endl;

	if(path.size()>0)
	{
		std::vector<Eigen::VectorXd> path_configs;

		std::cout<<"PATH: ";
		for(CompositeVertex &node: path )
		{
			Eigen::VectorXd path_config(mNumAgents*2);
			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			{
				Vertex source_vertex = threeple_first(mCompositeGraph[node].composite_vertex[agent_id]);
				Vertex target_vertex = threeple_second(mCompositeGraph[node].composite_vertex[agent_id]);
				size_t t_value = threeple_third(mCompositeGraph[node].composite_vertex[agent_id]);
			
				if(t_value == 0)
				{   
					path_config[agent_id*2] = mGraphs[agent_id][source_vertex].state[0];
					path_config[agent_id*2+1] = mGraphs[agent_id][source_vertex].state[1];
				}
				else
				{
					double edge_traversed = (t_value*epsilon_value)/(mGraphs[agent_id][target_vertex].state - mGraphs[agent_id][source_vertex].state).norm();
					Eigen::VectorXd config = mGraphs[agent_id][source_vertex].state + edge_traversed*(mGraphs[agent_id][target_vertex].state - mGraphs[agent_id][source_vertex].state);
					path_config[agent_id*2] = config[0];
					path_config[agent_id*2+1] =  config[1];
				}

			}
			
			std::cout<<mCompositeGraph[node].vertex_index<<" ";
			path_configs.push_back(path_config);

			// std::cout<<path_config[0]<<" "<<path_config[1]<<" "<<path_config[2]<<" "<<path_config[3]<<std::endl;
		}

		std::cout<<std::endl<<std::endl;

		std::cout<<"Path Length: "<<mCompositeGraph[mGoalVertex].distance<<std::endl;

		std::ofstream outfile;
		outfile.open("/home/rajat/melodic_ws/src/C-MINT/data/sparse_problems/dense_e_roadmaps.txt", std::ios_base::app);
		std::ostringstream ss;
		ss<<"C_MINT,";
		ss<<mCompositeGraph[mGoalVertex].distance<<",";
		ss<<elapsed_seconds.count() + pre_processing_time<<",";
		ss<<num_vertices(mCompositeGraph)<<","; //num_vertices
		ss<<num_edges(mCompositeGraph)<<","; //num_edges
		ss<<pre_processing_time<<",";
		ss<<number_expansions<<",";
		ss<<collision_checking_time<<",";
		ss<<get_neighbors_time<<",";
		ss<<iterate_neighbors_time<<",";
		ss<<queue_max_size<<",";
		ss<<queue_operations_time<<",";
		ss<<neighbors_iterated<<",";
		ss<<neighbors_added_in_queue<<"\n";
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
		outfile.open("/home/rajat/melodic_ws/src/C-MINT/data/sparse_problems/dense_e_roadmaps.txt", std::ios_base::app);
		std::ostringstream ss;
		ss<<"C_MINT,";
		if(mPlanningStatus == 0 )
			ss<<" SOLUTION NOT FOUND!! - No solution exists "; //sol not found
		else if( mPlanningStatus == 1)
			ss<<" SOLUTION NOT FOUND!! - Time Overflow "; //sol not found
		else
			ss<<" SOLUTION NOT FOUND!! - Space Overflow "; //sol not found"
		ss<<elapsed_seconds.count()<<",";
		ss<<num_vertices(mCompositeGraph)<<","; //num_vertices
		ss<<num_edges(mCompositeGraph)<<"\n"; //num_edges
		std::string data_string = ss.str();
		outfile << data_string;
		outfile.close();
		return std::vector<Eigen::VectorXd>();
	}
}

bool C_MINT::evaluateIndividualConfig(Eigen::VectorXd config)
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

bool C_MINT::evaluateIndividualEdge(Graph &graph, Edge& e) // returns false if in collision
{
	graph[e].isEvaluated = true;

	Vertex source_vertex = source(e, graph);
	Vertex target_vertex = target(e, graph);

	Eigen::VectorXd sourceState(2);
	sourceState << graph[source_vertex].state;

	Eigen::VectorXd targetState(2);
	targetState << graph[target_vertex].state;

	double resolution = 0.005;
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

bool C_MINT::evaluateConfig(Eigen::VectorXd config) // collision checking b/w left and right agent; both already checked with environment
{
	int numberOfRows = mImage.rows;
	int numberOfColumns = mImage.cols;

	std::vector<std::pair<double,double>> points;

	for(int agent_id_1=0; agent_id_1<mNumAgents; agent_id_1++)
	for(int agent_id_2=agent_id_1+1; agent_id_2<mNumAgents; agent_id_2++)
	{
		// double x_point_1 = config[agent_id_1*2]*numberOfColumns + 0.000001;
		// double y_point_1 = (1 - config[agent_id_1*2+1])*numberOfRows + 0.000001;

		// double x_point_2 = config[agent_id_2*2]*numberOfColumns + 0.000001;
		// double y_point_2 = (1 - config[agent_id_2*2+1])*numberOfRows + 0.000001;

		// // Collision check agent_id_1 and agent_id_2 with each other
		// if((x_point_1-x_point_2)*(x_point_1-x_point_2) + (y_point_1-y_point_2)*(y_point_1-y_point_2) < 50.000001 )
		// 	return false;

		double r = 0.02;

		if((config[agent_id_1*2]-config[agent_id_2*2])*(config[agent_id_1*2] - config[agent_id_2*2])
			+ (config[agent_id_1*2+1]-config[agent_id_2*2+1])*(config[agent_id_1*2+1] - config[agent_id_2*2+1]) < (4*r*r + 0.00001))
			return false;
	}

	return true;
}

bool C_MINT::evaluateEdge(const CompositeEdge& e) // returns false if in collision
{
	mCompositeGraph[e].isEvaluated = true;

	CompositeVertex startVertex = source(e,mCompositeGraph);
	CompositeVertex endVertex   = target(e,mCompositeGraph);

	Eigen::VectorXd startState(mNumAgents*2);
	for(int agent_id=0; agent_id<mNumAgents; agent_id++)
	{
		Vertex source_vertex = threeple_first(mCompositeGraph[startVertex].composite_vertex[agent_id]);
		Vertex target_vertex = threeple_second(mCompositeGraph[startVertex].composite_vertex[agent_id]);
		size_t t_value = threeple_third(mCompositeGraph[startVertex].composite_vertex[agent_id]);
	
		if(t_value == 0)
		{   
			startState[agent_id*2] = mGraphs[agent_id][source_vertex].state[0];
			startState[agent_id*2+1] = mGraphs[agent_id][source_vertex].state[1];
		}
		else
		{
			double edge_traversed = (t_value*epsilon_value)/(mGraphs[agent_id][target_vertex].state - mGraphs[agent_id][source_vertex].state).norm();
			Eigen::VectorXd config = mGraphs[agent_id][source_vertex].state + edge_traversed*(mGraphs[agent_id][target_vertex].state - mGraphs[agent_id][source_vertex].state);
			startState[agent_id*2] = config[0];
			startState[agent_id*2+1] =  config[1];
		}
	}
	
	Eigen::VectorXd endState(mNumAgents*2);
	for(int agent_id=0; agent_id<mNumAgents; agent_id++)
	{
		Vertex source_vertex = threeple_first(mCompositeGraph[endVertex].composite_vertex[agent_id]);
		Vertex target_vertex = threeple_second(mCompositeGraph[endVertex].composite_vertex[agent_id]);
		size_t t_value = threeple_third(mCompositeGraph[endVertex].composite_vertex[agent_id]);
	
		if(t_value == 0)
		{   
			endState[agent_id*2] = mGraphs[agent_id][source_vertex].state[0];
			endState[agent_id*2+1] = mGraphs[agent_id][source_vertex].state[1];
		}
		else
		{
			double edge_traversed = (t_value*epsilon_value)/(mGraphs[agent_id][target_vertex].state - mGraphs[agent_id][source_vertex].state).norm();
			Eigen::VectorXd config = mGraphs[agent_id][source_vertex].state + edge_traversed*(mGraphs[agent_id][target_vertex].state - mGraphs[agent_id][source_vertex].state);
			endState[agent_id*2] = config[0];
			endState[agent_id*2+1] =  config[1];
		}
	}

	double resolution = 0.005;
	unsigned int nStates = std::ceil(mCompositeGraph[e].length / resolution-0.000000001)+1;

	// Just start and goal
	if(nStates < 2u)
	{
		nStates = 2u;
	}
	// std::cout<<"nStates:"<<nStates<<std::endl;

	bool checkResult = true;
	
	if (checkResult && !evaluateConfig(startState))
	{
		mCompositeGraph[startVertex].status = CollisionStatus::BLOCKED;
		mCompositeGraph[e].status = CollisionStatus::BLOCKED;
		mCompositeGraph[e].length = INF;
		checkResult = false;
	}

	if (checkResult && !evaluateConfig(endState))
	{
		mCompositeGraph[endVertex].status = CollisionStatus::BLOCKED;
		mCompositeGraph[e].status = CollisionStatus::BLOCKED;
		mCompositeGraph[e].length = INF;
		checkResult = false;
	}

	if (checkResult)
	{
		// Evaluate the States in between
		for (unsigned int i = 1; i < nStates-1; i++)
		{
			if(mCompositeGraph[e].length < 0.0000001)
			{
				std::cout<<"WTF!";
				std::cin.get();
			}
			if(!evaluateConfig(startState + (resolution*i/mCompositeGraph[e].length)*(endState-startState) ))
			{
				mCompositeGraph[e].status = CollisionStatus::BLOCKED;
				mCompositeGraph[e].length = INF;
				checkResult = false;
				break;
			}
		}
	}

	return checkResult;
}

// ===========================================================================================
CompositeEdge C_MINT::getEdge(CompositeVertex u, CompositeVertex v) const
{
	CompositeEdge uv;
	bool edgeExists;
	boost::tie(uv, edgeExists) = edge(u, v, mCompositeGraph);
	if (edgeExists == false)
	{
		std::cout<<"Edge does not exist. Press [ENTER] to get segmentation fault :( :"<<std::endl;
		std::cin.get();
	}

	return uv;
}

// ===========================================================================================
double C_MINT::estimateCostToCome(CompositeVertex v) const
{
	return mCompositeGraph[v].distance;
}

double C_MINT::heuristicFunction(CompositeVertex v) const
{
	return mCompositeGraph[v].heuristic;
}

double C_MINT::estimateTotalCost(CompositeVertex v) const
{
	if(mCompositeGraph[v].overestimate == true)
	{
		return mCompositeGraph[v].over_fval;
	}
	return estimateCostToCome(v) + heuristicFunction(v);
}

double C_MINT::getDistance(Eigen::VectorXd config_one, Eigen::VectorXd config_two) 
{
	// double epsilon_value = 0.01;
	return epsilon_value*std::ceil((config_one-config_two).norm()*(1/epsilon_value)-0.000000001);
}

} // namespace C_MINT

#endif // C_MINT_C_MINT_HPP_