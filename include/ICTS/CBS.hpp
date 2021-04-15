 
/* Authors: Rajat Kumar Jenamani */

#ifndef _CBS_HPP
#define _CBS_HPP

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
#include "time_priority_queue.h"
#include "CBSDefinitions.h"
#include "LoadGraphfromFile.hpp"

#define INF std::numeric_limits<double>::infinity()

namespace CBS {

using namespace BGL_DEFINITIONS;

class CBS
{

public:

	/// Environment
	cv::Mat mImage;

	/// The fixed graphs denoting individual environment of corresponding agents
	std::vector<Graph> mGraphs;

	/// Number of agents
	size_t mNumAgents; 

	/// Path to the roadmap files.
	std::vector<std::string> mRoadmapFileNames;

	/// Source vertex.
	std::vector<Eigen::VectorXd> mStartConfig;
	std::vector<Vertex> mStartVertex;

	/// Goal vertex.
	std::vector<Eigen::VectorXd> mGoalConfig;
	std::vector<Vertex> mGoalVertex;


	CBS(cv::Mat img, size_t numAgents, std::vector<std::string> roadmapFileNames, Eigen::VectorXd _start_config, Eigen::VectorXd _goal_config)
		: mImage(img)
		, mNumAgents(numAgents)
		, mRoadmapFileNames(roadmapFileNames)
	{
		for(int i=0; i<mNumAgents;i++)
		{
			Eigen::VectorXd start_config(2);
			for (size_t ui = i*2; ui < i*2+2; ui++)
				start_config[ui-i*2] = _start_config[ui];
			mStartConfig.push_back(start_config);
		}

		for(int i=0; i<mNumAgents;i++)
		{
			Eigen::VectorXd goal_config(2);
			for (size_t ui = i*2; ui < i*2+2; ui++)
				goal_config[ui-i*2] = _goal_config[ui];
			mGoalConfig.push_back(goal_config);
		}

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
			mStartVertex.push_back(start_vertex);
			mGoalVertex.push_back(goal_vertex);
		}

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			preprocess_graph(mGraphs[agent_id], mGoalVertex[agent_id]);
	}


	std::vector< std::vector<Vertex> > computeDecoupledPaths(std::vector<std::vector<Constraint>> constraints, std::vector<double> &costs)
	{
		std::vector<std::vector<Vertex> > shortestPaths;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			double ind_cost;
			std::vector <Vertex> path = computeShortestPath(mGraphs[agent_id], mStartVertex[agent_id], mGoalVertex[agent_id], constraints[agent_id], ind_cost);
			shortestPaths.push_back(path);
			costs.push_back(ind_cost);
		}

		return shortestPaths;
	}

	bool getVerticesCollisionStatus(Eigen::VectorXd left, Eigen::VectorXd right)
	{
		double distance = (left - right).norm();
		if(distance < 0.0141) // tune threshold!!
			return true;
		return false;
	}

	bool getEdgesCollisionStatus(Eigen::VectorXd left_source, Eigen::VectorXd left_target, Eigen::VectorXd right_source, Eigen::VectorXd right_target)
	{
		double resolution = 0.0025;
		double length = (left_source - left_target).norm(); // assuming length is same for both edges (grid edge)
		unsigned int nStates = std::ceil(length / resolution-0.000000001)+1;

		// Just start and goal
		if(nStates < 2u)
		{
			nStates = 2u;
		}

		// Evaluate the States in between
		for (unsigned int i = 1; i < nStates-1; i++)
		{
			if(getVerticesCollisionStatus(left_source + (resolution*i/length)*(left_target-left_source),
						right_source + (resolution*i/length)*(right_target-right_source)))
				return true;
		}

		return false;
	}

	bool checkCoupling(std::vector<std::vector<Vertex>> &paths, int &agent_id_1, Constraint &constraint_1, int &agent_id_2, Constraint &constraint_2)
	{
		size_t timeStep = 0;
		while(true)
		{
			std::vector<Vertex> source_vertices;
			std::vector<Vertex> target_vertices;

			int left = 0;
			
			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			{
				if(paths[agent_id].size() >= timeStep+1)
				{
					left++;
					source_vertices.push_back(paths[agent_id].at(timeStep));
					target_vertices.push_back(paths[agent_id].at(timeStep+1));
				}
				else
				{
					source_vertices.push_back(paths[agent_id].at(paths[agent_id].size()-1));
					target_vertices.push_back(paths[agent_id].at(paths[agent_id].size()-1));
				}
			}

			if(left == 0)
				break;

			bool col_type = -1; // 1 -> source vertices, 2 -> target vertices, 3 -> edges
			int agent_id_1 = -1;
			int agent_id_2 = -1;
			for(size_t i=0; i<mNumAgents; i++)
			for(size_t j=i+1; j<mNumAgents; j++)
			{
				if(getVerticesCollisionStatus(mGraphs[i][source_vertices[i]].state, mGraphs[j][source_vertices[j]].state))
				{
					agent_id_1 = i;
					agent_id_2 = j;

					Constraint con_1(source_vertices[i],timeStep);
					constraint_1 = con_1;

					Constraint con_2(source_vertices[j],timeStep);
					constraint_2 = con_2;

					return true;	
				}

				if(getVerticesCollisionStatus(mGraphs[i][target_vertices[i]].state, mGraphs[j][target_vertices[j]].state))
				{
					agent_id_1 = i;
					agent_id_2 = j;

					Constraint con_1(target_vertices[i],timeStep+1);
					constraint_1 = con_1;

					Constraint con_2(target_vertices[j],timeStep+1);
					constraint_2 = con_2;

					return true;
				}
				
				if(getEdgesCollisionStatus(mGraphs[i][source_vertices[i]].state, mGraphs[i][target_vertices[i]].state, mGraphs[j][source_vertices[j]].state, mGraphs[j][target_vertices[j]].state))
				{
					agent_id_1 = i;
					agent_id_2 = j;

					Edge edge_1 = boost::edge(source_vertices[i],target_vertices[i],mGraphs[i]).first;
					Constraint con_1(edge_1,timeStep+1);
					constraint_1 = con_1;

					Edge edge_2 = boost::edge(source_vertices[j],target_vertices[j],mGraphs[i]).first;
					Constraint con_2(edge_2,timeStep+1);
					constraint_2 = con_2;

					return true;
				}
			}
			
			timeStep++;
		}
		return false;
	}

	std::vector<std::vector<Eigen::VectorXd>> findPath()
	{
		CBSPriorityQueue PQ(mNumAgents);

		std::vector<std::vector<Constraint>> constraints(mNumAgents, std::vector<Constraint>());
		std::vector<double> start_costs;
		std::vector< std::vector<Vertex> > start_shortestPaths = computeDecoupledPaths(constraints, start_costs);

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			if(start_shortestPaths.at(agent_id).size()==0)
			{
				std::cout<<"No Path exists for index "<<agent_id<<"! Press [ENTER] to exit: ";
				std::cin.get();
			}
		}
		
		PQ.insert(start_costs, constraints, start_shortestPaths);

		while(PQ.PQsize()!=0)
		{
			Element p = PQ.pop();

			double total_cost = 0;
			for(int i=0; i<p.costs.size(); i++)
				total_cost += p.costs[i];

			int agent_id_1 = -1;
			Constraint constraint_1;

			int agent_id_2 = -1;
			Constraint constraint_2;

			if(!checkCoupling(p.shortestPath, agent_id_1, constraint_1, agent_id_2, constraint_2))
			{
				std::vector<std::vector<Eigen::VectorXd>> collision_free_path(mNumAgents, std::vector<Eigen::VectorXd>());

				std::cout<<" Path Cost: "<<total_cost<<std::endl;
				for(int agent_id=0; agent_id<mNumAgents; agent_id++)
				{
					std::cout<<"Shortest Path Cost for index - "<<agent_id<<" : "<<p.costs[agent_id]<<std::endl;
					std::cout<<"Shortest Path for index - "<<agent_id<<" : ";
					for(Vertex &nodes: p.shortestPath[agent_id])
					{
						std::cout<<mGraph[agent_id][nodes].vertex_index<<" ";
						collision_free_path[agent_id].push_back(mGraph[agent_id][nodes].state);
					}
				}

				return collision_free_path;

			} 

			//agent_id_1

			std::vector<std::vector<Constraint>> increase_constraints_agent_id_1 = p.constraints;
			increase_constraints_agent_id_1[agent_id_1].push_back(constraint_1);

			double cost_agent_id_1;

			std::vector< double> costs_agent_id_1 = p.costs;
			std::vector< std::vector<Vertex> > shortestPaths_agent_id_1 = p.shortestPath;
			
			shortestPaths_agent_id_1[agent_id_1] = computeShortestPath(mGraphs[agent_id_1], mStartVertex[agent_id_1], mGoalVertex[agent_id_1], increase_constraints_agent_id_1[agent_id_1], cost_agent_id_1);
			costs_agent_id_1[agent_id_1] = cost_agent_id_1;

			if(costs_agent_id_1[agent_id_1] != INF_VAL)
				PQ.insert(costs_agent_id_1,increase_constraints_agent_id_1,shortestPaths_agent_id_1);
			
			
			//agent_id_2

			std::vector<std::vector<Constraint>> increase_constraints_agent_id_2 = p.constraints;
			increase_constraints_agent_id_2[agent_id_2] = constraint_2;

			double cost_agent_id_2;

			std::vector< double> costs_agent_id_2 = p.costs;
			std::vector< std::vector<Vertex> > shortestPaths_agent_id_2 = p.shortestPath;
			
			shortestPaths_agent_id_2[agent_id_2] = computeShortestPath(mGraphs[agent_id_2], mStartVertex[agent_id_2], mGoalVertex[agent_id_2], increase_constraints_agent_id_2[agent_id_2], cost_agent_id_2);
			costs_agent_id_2[agent_id_2] = cost_agent_id_2;

			if(costs_agent_id_2[agent_id_2] != INF_VAL)
				PQ.insert(costs_agent_id_2,increase_constraints_agent_id_2,shortestPaths_agent_id_2);
		}

		return std::vector(mNumAgents,std::vector<VectorXd>());
	}

	bool evaluateIndividualConfig(Eigen::VectorXd config)
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

	bool evaluateIndividualEdge(Graph &graph, Edge& e) // returns false if in collision
	{
		graph[e].isEvaluated = true;

		Vertex source_vertex = source(e, graph);
		Vertex target_vertex = target(e, graph);

		Eigen::VectorXd sourceState(2);
		sourceState << graph[source_vertex].state;

		Eigen::VectorXd targetState(2);
		targetState << graph[target_vertex].state;

		double resolution = 0.0025;
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

	std::vector<Vertex> getNeighbors(Graph &herb_map, Vertex &v)
	{
		std::vector<Vertex> neighbors;
		OutEdgeIter ei, ei_end;

		for (boost::tie(ei, ei_end) = out_edges(node, g); ei != ei_end; ++ei) 
		{
			Vertex curSucc = target(*ei, g);
			Edge e = *ei;
			if(!g[e].isEvaluated)
				evaluateIndividualEdge(g,e);
			if(g[e].status == CollisionStatus::FREE)
				neighbors.push_back(curSucc);
		}
		return neighbors;
	}

	std::vector<Vertex> computeShortestPath(Graph &graph, Vertex &start, Vertex &goal, std::vector<Constraint> &constraints, double& costOut)
	{
		timePriorityQueue pq;
		std::unordered_map<std::pair<Vertex, size_t>, double, pair_hash> mDistance;
		std::unordered_map<std::pair<Vertex, size_t> , std::pair<Vertex, size_t>, pair_hash > mPrev;
		std::unordered_map<size_t , Vertex> nodeMap;

		pq.insert(graph[start].vertex_index,0,graph[start].heuristic,0.0);
		nodeMap[graph[start].vertex_index]=start;

		VertexIter vi, viend;
		mDistance[std::make_pair(start,0)]=0;

		size_t numSearches = 0;
		size_t maximum_timestep = 10000;

		int goal_timestep = -1;
		while(pq.PQsize()!=0)
		{
			numSearches++;
			std::cout<<"Queue pop no: "<<numSearches<<std::endl;
			std::pair<int,size_t> top_element = pq.pop();
			size_t index = top_element.first;
			size_t timestep = top_element.second;
			if(timestep > maximum_timestep)
				break;
			if(index == graph[goal].vertex_index)
			{
				goal_timestep = timestep;
				costOut = mDistance[std::make_pair(goal,goal_timestep)]
				break;
			}
			Vertex curr_node = nodeMap[index];
			std::vector<Vertex> neighbors = getNeighbors(curr_node);
			neighbors.push_back(curr_node);
			// std::cout<<"No. of neighbors :"<<neighbors.size()<<std::endl;

			for (auto successor : neighbors) 
			{
				if(successor == curr_node)
				{
					bool col = false;
					for( Constraint c: constraints)
					{
						if( successor == c.v && c.t == timestep + 1)
						{
							// std::cout<<"Constraint Encountered! "<<std::endl;
							col =true;
							break;
						}
					}

					if(!col)
					{
						double new_cost = mDistance[std::make_pair(curr_node,timestep)] + mUnitEdgeLength;
						
						if(mDistance.count(std::make_pair(successor,timestep+1))==0 || 
							new_cost < mDistance[std::make_pair(successor,timestep+1)])
						{
							mDistance[std::make_pair(successor,timestep+1)]= new_cost;
							double priority;
							// std::cout<<"FOUND FREE EDGE!!"<<std::endl;
							priority = new_cost + graph[successor].heuristic;
							pq.insert(graph[successor].vertex_index,timestep+1,priority,0.0);
							if(nodeMap.count(graph[successor].vertex_index)==0)
								nodeMap[graph[successor].vertex_index]=successor;
							mPrev[std::make_pair(successor,timestep+1)]=std::make_pair(curr_node,timestep);
						}
					}
				}
				else
				{
					Edge uv_edge = boost::edge(curr_node, successor, graph).first;

					bool col = false;
					for( Constraint c: constraints)
					{
						if( (successor == c.v && c.t == timestep + 1) || (uv_edge == c.e && c.t == timestep + 1) )
						{
							// std::cout<<"Constraint Encountered! "<<std::endl;
							col =true;
							break;
						}
					}

					if(!col)
					{					
						double new_cost = mDistance[std::make_pair(curr_node,timestep)] + mUnitEdgeLength;
						if(mDistance.count(std::make_pair(successor,timestep+1))==0 || new_cost < mDistance[std::make_pair(successor,timestep+1)])
						{
							mDistance[std::make_pair(successor,timestep+1)]= new_cost;
							double priority;
							priority = new_cost + graph[successor].heuristic;
							pq.insert(graph[successor].vertex_index,timestep+1,priority,0.0);
							if(nodeMap.count(graph[successor].vertex_index)==0)
								nodeMap[graph[successor].vertex_index]=successor;
							mPrev[std::make_pair(successor,timestep+1)]=std::make_pair(curr_node,timestep);
						}
					}
				}
			}
		}

		if(goal_timestep == -1)
		{
			std::cout<<"ALL_COL!"<<std::endl;
			costOut = INF_VAL;
			return std::vector<Vertex>();
		}

		std::cout<<"Goal Time: "<<goal_timestep<<std::endl;
		std::vector<Vertex> finalPath;
		Vertex node = goal;

		while(node!=mStartNode)
		{
			// std::cin.get();
			// std::cout<<"INF LOOP LOL!";
			finalPath.push_back(node);
			Vertex temp_node = node;
			size_t temp_timestep = goal_timestep;
			node=mPrev[std::make_pair(temp_node,temp_timestep)].first;
			goal_timestep=mPrev[std::make_pair(temp_node,temp_timestep)].second;
		}
		finalPath.push_back(mStartNode);
		std::reverse(finalPath.begin(), finalPath.end());
		return finalPath;
	}

	void preprocess_graph(Graph &g, Vertex & _goal)
	{
		auto begin = std::chrono::high_resolution_clock::now();

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
		}   
	}

};


} // namespace CBS

#endif 