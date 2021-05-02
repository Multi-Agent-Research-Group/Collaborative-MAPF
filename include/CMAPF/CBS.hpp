 
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
#include "time_priority_queue.hpp"
#include "CBSDefinitions.hpp"
#include "LoadGraphfromFile.hpp"

#include <chrono>
using namespace std::chrono;

#define INF std::numeric_limits<double>::infinity()

namespace CMAPF {

using namespace BGL_DEFINITIONS;

class CBS
{

public:

	/// Environment
	cv::Mat mImage;

	/// The fixed graphs denoting individual environment of corresponding agents
	std::vector<Graph> mGraphs;

	/// Number of agents
	int mNumAgents; 

	/// Path to the roadmap files.
	std::vector<std::string> mRoadmapFileNames;

	/// Source vertex.
	std::vector<Eigen::VectorXd> mStartConfig;
	std::vector<Vertex> mStartVertex;

	/// Goal vertex.
	std::vector<Eigen::VectorXd> mGoalConfig;
	std::vector<Vertex> mGoalVertex;

	std::vector<int> mStartTimestep;
	std::vector<int> mGoalTimestep;

	std::vector<std::pair<Eigen::VectorXd,std::pair<int,int>>> mStationaryAgents;

	int mHashUsed = 0;
	int mNotHashUsed = 0;
	int total_time = 0;

	double mUnitEdgeLength = 0.04;

	CBS(cv::Mat img, int numAgents, std::vector<std::string> roadmapFileNames, std::vector<Eigen::VectorXd> startConfig, std::vector<Eigen::VectorXd> goalConfig, 
		std::vector<int> startTimesteps, std::vector<int> goalTimesteps, std::vector<Graph> graphs, std::vector<Vertex> startVertex, std::vector<Vertex> goalVertex,
		std::vector<std::pair<Eigen::VectorXd,std::pair<int,int>>>& stationaryAgents)
		: mImage(img)
		, mNumAgents(numAgents)
		, mRoadmapFileNames(roadmapFileNames)
		, mStartTimestep(startTimesteps)
		, mGoalTimestep(goalTimesteps)
		, mStartConfig(startConfig)
		, mGoalConfig(goalConfig)
		, mGraphs(graphs)
		, mStartVertex(startVertex)
		, mGoalVertex(goalVertex) 
		, mStationaryAgents(stationaryAgents){}

	std::vector< std::vector<Vertex> > computeDecoupledPaths(std::vector<std::vector<Constraint>> constraints, std::vector<double> &costs)
	{
		std::vector<std::vector<Vertex> > shortestPaths;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			double ind_cost;
			std::vector <Vertex> path = computeShortestPath(mGraphs[agent_id], mStartVertex[agent_id], mGoalVertex[agent_id], constraints[agent_id], mStartTimestep[agent_id], mGoalTimestep[agent_id], ind_cost);
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
		if ( (left_source - right_target).norm() < 0.0141 &&  (right_source - left_target).norm() < 0.0141)
			return true;
		return false;
	}

	bool checkCoupling(std::vector<std::vector<Vertex>> &paths, int &agent_id_1, Constraint &constraint_1, int &agent_id_2, Constraint &constraint_2)
	{
		int timeStep = 0;
		int maximum_timestep = 0;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			maximum_timestep = std::max(maximum_timestep, mGoalTimestep[agent_id]);
		// std::cout<<"MT: "<<maximum_timestep<<std::endl;std::cin.get();
		while(timeStep < maximum_timestep)
		{
			std::vector<Vertex> source_vertices;
			std::vector<Vertex> target_vertices;
			std::vector<int> agent_ids;

			// std::cout<<mStartTimestep.size()<<" "<<mGoalTimestep.size()<<std::endl;
			// std::cin.get();
			
			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			{
				if( timeStep - mStartTimestep[agent_id] >= 0 &&  mGoalTimestep[agent_id] - timeStep >= 1)
				{
					agent_ids.push_back(agent_id);
					if(timeStep - mStartTimestep[agent_id]+1 > paths[agent_id].size())
					{
						std::cout<<"Outside memory!"; std::cin.get();
					}
					// std::cout<<"Index: "<<timeStep - mStartTimestep[agent_id];std::cin.get();

					// if(timeStep - mStartTimestep[agent_id] < paths[agent_id].size() - 1)
					// {
						source_vertices.push_back(paths[agent_id].at(timeStep - mStartTimestep[agent_id]));
						target_vertices.push_back(paths[agent_id].at(timeStep - mStartTimestep[agent_id]+1));
					// }
					// else
					// {
					//  source_vertices.push_back(paths[agent_id].at(paths[agent_id].size()-1));
					//  target_vertices.push_back(paths[agent_id].at(paths[agent_id].size()-1));
					// }
				}
			}

			// for(int i=0; i<agent_ids.size(); i++)
			//  std::cout<<agent_ids[i]<<" ";
			// std::cout<<std::endl;
			// std::cin.get();

			for(int i=0; i<agent_ids.size(); i++)
			for(int j=i+1; j<agent_ids.size(); j++)
			{
				// if(getVerticesCollisionStatus(mGraphs[agent_ids[i]][source_vertices[i]].state, mGraphs[agent_ids[j]][source_vertices[j]].state))
				// {
				//  agent_id_1 = agent_ids[i];
				//  agent_id_2 = agent_ids[j];

				//  bool agent_1_at_goal = paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == source_vertices[i];
				//  bool agent_2_at_goal = paths[agent_id_2].at(mGoalTimestep[agent_id_2]-mStartTimestep[agent_id_2]) == source_vertices[j];

				//  if(  (agent_1_at_goal && agent_2_at_goal )  && (target_vertices[i]!=source_vertices[i] || target_vertices[j]!=source_vertices[j])   ){
				// 		Edge edge_1 = boost::edge(source_vertices[i],target_vertices[i],mGraphs[agent_ids[i]]).first;
				// 		constraint_1 = Constraint(edge_1,timeStep+1);

				// 		Edge edge_2 = boost::edge(source_vertices[j],target_vertices[j],mGraphs[agent_ids[j]]).first;
				// 		constraint_2 = Constraint(edge_2,timeStep+1);
				// 		return true;
				// 	} 
				// }

				if(getVerticesCollisionStatus(mGraphs[agent_ids[i]][target_vertices[i]].state, mGraphs[agent_ids[j]][target_vertices[j]].state))
				{
					agent_id_1 = agent_ids[i];
					agent_id_2 = agent_ids[j];

					bool agent_1_safe = ((paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == target_vertices[i])
						|| (paths[agent_id_1].at(0) == target_vertices[i]));
					bool agent_2_safe = ((paths[agent_id_2].at(mGoalTimestep[agent_id_2]-mStartTimestep[agent_id_2]) == target_vertices[j])
						|| (paths[agent_id_2].at(0) == target_vertices[j]));

					if( !agent_1_safe || !agent_2_safe )
					{
						constraint_1 = Constraint(target_vertices[i],timeStep+1);
						constraint_2 = Constraint(target_vertices[j],timeStep+1);
						return true;
					}
					// else{
					// 	std::cout << "same goal!\n";
					// 	continue;
					// }
				}
				
				if(getEdgesCollisionStatus(mGraphs[agent_ids[i]][source_vertices[i]].state, mGraphs[agent_ids[i]][target_vertices[i]].state, mGraphs[agent_ids[j]][source_vertices[j]].state, mGraphs[agent_ids[j]][target_vertices[j]].state))
				{
					agent_id_1 = agent_ids[i];
					agent_id_2 = agent_ids[j];

					bool agent_1_safe = ((source_vertices[i] == target_vertices[i])
						&& ((source_vertices[i] == paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]))
							|| source_vertices[i] == paths[agent_id_1].at(0)));

					bool agent_2_safe = ((source_vertices[j] == target_vertices[j])
						&& ((source_vertices[j] == paths[agent_id_2].at(mGoalTimestep[agent_id_2]-mStartTimestep[agent_id_2]))
							|| source_vertices[j] == paths[agent_id_2].at(0)));

					// bool agent_1_target_goal = paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == target_vertices[i];
					// bool agent_1_source_goal = paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == target_vertices[i];
					

					if(!agent_1_safe || !agent_2_safe)
					{
						Edge edge_1 = boost::edge(source_vertices[i],target_vertices[i],mGraphs[agent_ids[i]]).first;
						constraint_1 = Constraint(edge_1,timeStep+1);

						Edge edge_2 = boost::edge(source_vertices[j],target_vertices[j],mGraphs[agent_ids[j]]).first;
						constraint_2 = Constraint(edge_2,timeStep+1);
						return true;
					}
				}
			}
			
			timeStep++;
		}
		return false;
	}

	bool checkStationaryCoupling(std::vector<std::vector<Vertex>> &paths, int &agent_id_1, Constraint &constraint_1)
	{
		int timeStep = 0;
		int maximum_timestep = 0;
		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			maximum_timestep = std::max(maximum_timestep, mGoalTimestep[agent_id]);
		// std::cout<<"MT: "<<maximum_timestep<<std::endl;std::cin.get();
		while(timeStep < maximum_timestep)
		{
			std::vector<Vertex> source_vertices;
			std::vector<Vertex> target_vertices;
			std::vector<int> agent_ids;

			std::vector<int> s_ids;

			// std::cout<<mStartTimestep.size()<<" "<<mGoalTimestep.size()<<std::endl;
			// std::cin.get();
			
			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			{
				if( timeStep - mStartTimestep[agent_id] >= 0 &&  mGoalTimestep[agent_id] - timeStep >= 1)
				{
					agent_ids.push_back(agent_id);
					if(timeStep - mStartTimestep[agent_id]+1 > paths[agent_id].size())
					{
						std::cout<<"Outside memory!"; std::cin.get();
					}
					source_vertices.push_back(paths[agent_id].at(timeStep - mStartTimestep[agent_id]));
					target_vertices.push_back(paths[agent_id].at(timeStep - mStartTimestep[agent_id]+1));
				}
			}
			for(int s_id=0; s_id<mStationaryAgents.size(); s_id++)
			{
				if( timeStep - mStationaryAgents[s_id].second.first >= 0 &&  mStationaryAgents[s_id].second.second - timeStep >= 1)
				{
					s_ids.push_back(s_id);
				}
			}

			// for(int i=0; i<agent_ids.size(); i++)
			//  std::cout<<agent_ids[i]<<" ";
			// std::cout<<std::endl;
			// std::cin.get();

			for(int i=0; i<agent_ids.size(); i++)
			for(int j=0; j<s_ids.size(); j++)
			{
				if(getVerticesCollisionStatus(mGraphs[agent_ids[i]][target_vertices[i]].state, mStationaryAgents[s_ids[j]].first))
				{
					agent_id_1 = agent_ids[i];
					int agent_id_2 = s_ids[j];

					bool agent_1_safe = ((paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == target_vertices[i])
						|| (paths[agent_id_1].at(0) == target_vertices[i]));

					if( !agent_1_safe)
					{
						constraint_1 = Constraint(target_vertices[i],timeStep+1);
						return true;
					}
				}
				
				if(getEdgesCollisionStatus(mGraphs[agent_ids[i]][source_vertices[i]].state, mGraphs[agent_ids[i]][target_vertices[i]].state, mStationaryAgents[s_ids[j]].first, mStationaryAgents[s_ids[j]].first))
				{
					agent_id_1 = agent_ids[i];
					int agent_id_2 = s_ids[j];

					bool agent_1_safe = ((source_vertices[i] == target_vertices[i])
						&& ((source_vertices[i] == paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]))
							|| source_vertices[i] == paths[agent_id_1].at(0)));

					// bool agent_1_target_goal = paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == target_vertices[i];
					// bool agent_1_source_goal = paths[agent_id_1].at(mGoalTimestep[agent_id_1]-mStartTimestep[agent_id_1]) == target_vertices[i];
					

					if(!agent_1_safe)
					{
						Edge edge_1 = boost::edge(source_vertices[i],target_vertices[i],mGraphs[agent_ids[i]]).first;
						constraint_1 = Constraint(edge_1,timeStep+1);
						return true;
					}
				}
			}
			
			timeStep++;
		}
		return false;
	}

	std::vector<std::vector<Eigen::VectorXd>> solve()
	{
		CBSPriorityQueue PQ(mNumAgents);

		std::vector<std::vector<Constraint>> constraints(mNumAgents, std::vector<Constraint>());
		std::vector<double> start_costs;
		std::vector< std::vector<Vertex> > start_shortestPaths = computeDecoupledPaths(constraints, start_costs);

		for(int agent_id=0; agent_id<mNumAgents; agent_id++)
		{
			if(start_shortestPaths.at(agent_id).size()==0)
			{
				// std::cout<<"No Path exists for index "<<agent_id<<"! Press [ENTER] to exit: ";
				// std::cin.get();
				return std::vector<std::vector<Eigen::VectorXd>>(mNumAgents,std::vector<Eigen::VectorXd>());
			}
		}

		// std::cout<<"K";std::cin.get();

		PQ.insert(start_costs, constraints, start_shortestPaths);

		int numSearches = 0;
		while(PQ.PQsize()!=0)
		{
			numSearches++;
			// if(numSearches == 100)
			// {
			// 	// std::cout<<"numSearches: "<<numSearches<<std::endl;
			// 	break;
			// }
			Element p = PQ.pop();

			// std::cout<<"K";std::cin.get();

			double total_cost = 0;
			for(int i=0; i<p.costs.size(); i++)
				total_cost += p.costs[i];

			bool cost_increased = false;

			for(int i=0; i<p.costs.size(); i++)
				if(std::abs(start_costs[i] - p.costs[i]) > 0.0001)
				{
					cost_increased = true;
					break;
				}

			if(cost_increased)
				break;

			int agent_id_1 = -1;
			Constraint constraint_1;

			int agent_id_2 = -1;
			Constraint constraint_2;

			// std::cout<<"K";std::cin.get();

			if(!checkCoupling(p.shortestPaths, agent_id_1, constraint_1, agent_id_2, constraint_2))
			{
				if(!checkStationaryCoupling(p.shortestPaths, agent_id_1, constraint_1))
				{
					std::vector<std::vector<Eigen::VectorXd>> collision_free_path(mNumAgents, std::vector<Eigen::VectorXd>());

					// std::cout<<" Path Cost: "<<total_cost<<std::endl;
					for(int agent_id=0; agent_id<mNumAgents; agent_id++)
					{
						// std::cout<<"Shortest Path Cost for index - "<<agent_id<<" : "<<p.costs[agent_id]<<std::endl;
						// std::cout<<"Shortest Path for index - "<<agent_id<<" : ";
						for(Vertex &nodes: p.shortestPaths[agent_id])
						{
							// std::cout<<mGraphs[agent_id][nodes].vertex_index<<"_"<<mGraphs[agent_id][nodes].state<<" ";
							collision_free_path[agent_id].push_back(mGraphs[agent_id][nodes].state);
						}
						// std::cout<<std::endl;
					}
					// std::cerr<<"returning!"<<std::endl;

					return collision_free_path;
				}

				std::vector<std::vector<Constraint>> increase_constraints_agent_id_1 = p.constraints;

				// std::cout<<increase_constraints_agent_id_1.size()<<" "<<agent_id_1<<std::endl;
				increase_constraints_agent_id_1[agent_id_1].push_back(constraint_1);

				double cost_agent_id_1;

				std::vector< double> costs_agent_id_1 = p.costs;
				std::vector< std::vector<Vertex> > shortestPaths_agent_id_1 = p.shortestPaths;
				
				shortestPaths_agent_id_1[agent_id_1] = computeShortestPath(mGraphs[agent_id_1], mStartVertex[agent_id_1], mGoalVertex[agent_id_1], increase_constraints_agent_id_1[agent_id_1], mStartTimestep[agent_id_1], mGoalTimestep[agent_id_1], cost_agent_id_1);
				costs_agent_id_1[agent_id_1] = cost_agent_id_1;

				if(costs_agent_id_1[agent_id_1] == p.costs[agent_id_1])
				{
					// std::cout<<"inserting left!"<<std::endl;
					PQ.insert(costs_agent_id_1,increase_constraints_agent_id_1,shortestPaths_agent_id_1);
				}
				continue;

			} 

			if(numSearches%100 == 0)
			{
				// std::cout<<"numSearches"<<numSearches<<std::endl;
				// if(constraint_1.constraint_type == 1)
				// 	std::cout<<"Vertex Constraint: ("<<int( (mGraphs[agent_id_1][constraint_1.v].state[0]+0.001)/0.0625)<<","
				// 		<<int( (mGraphs[agent_id_1][constraint_1.v].state[1]+0.001)/0.0625)<<") at "<<constraint_1.t<<std::endl;
				// else
				// {
				// 	std::cout<<"Edge Constraint: ("<<int( (mGraphs[agent_id_1][source(constraint_1.e, mGraphs[agent_id_1])].state[0]+0.001)/0.0625)<<","
				// 		<<int( (mGraphs[agent_id_1][source(constraint_1.e, mGraphs[agent_id_1])].state[1]+0.001)/0.0625)<<") , ("<<int( (mGraphs[agent_id_1][target(constraint_1.e, mGraphs[agent_id_1])].state[0]+0.001)/0.0625)<<","
				// 		<<int( (mGraphs[agent_id_1][target(constraint_1.e, mGraphs[agent_id_1])].state[1]+0.001)/0.0625)<<") at "<<constraint_1.t<<std::endl;
				// }
			}

			// std::cout<<"K";std::cin.get();

			//agent_id_1

			std::vector<std::vector<Constraint>> increase_constraints_agent_id_1 = p.constraints;

			// std::cout<<increase_constraints_agent_id_1.size()<<" "<<agent_id_1<<std::endl;
			increase_constraints_agent_id_1[agent_id_1].push_back(constraint_1);

			double cost_agent_id_1;

			std::vector< double> costs_agent_id_1 = p.costs;
			std::vector< std::vector<Vertex> > shortestPaths_agent_id_1 = p.shortestPaths;
			
			shortestPaths_agent_id_1[agent_id_1] = computeShortestPath(mGraphs[agent_id_1], mStartVertex[agent_id_1], mGoalVertex[agent_id_1], increase_constraints_agent_id_1[agent_id_1], mStartTimestep[agent_id_1], mGoalTimestep[agent_id_1], cost_agent_id_1);
			costs_agent_id_1[agent_id_1] = cost_agent_id_1;

			// std::cout<<"K";std::cin.get();

			// std::cout<<"Agent id 1: "<<agent_id_1<<std::endl;

			if(costs_agent_id_1[agent_id_1] == p.costs[agent_id_1])
			{
				// std::cout<<"inserting left!"<<std::endl;
				PQ.insert(costs_agent_id_1,increase_constraints_agent_id_1,shortestPaths_agent_id_1);
			}

			// std::cout<<"K";std::cin.get();
			
			//agent_id_2

			std::vector<std::vector<Constraint>> increase_constraints_agent_id_2 = p.constraints;
			increase_constraints_agent_id_2[agent_id_2].push_back(constraint_2);

			double cost_agent_id_2;

			std::vector< double> costs_agent_id_2 = p.costs;
			std::vector< std::vector<Vertex> > shortestPaths_agent_id_2 = p.shortestPaths;
			
			shortestPaths_agent_id_2[agent_id_2] = computeShortestPath(mGraphs[agent_id_2], mStartVertex[agent_id_2], mGoalVertex[agent_id_2], increase_constraints_agent_id_2[agent_id_2],  mStartTimestep[agent_id_2], mGoalTimestep[agent_id_2], cost_agent_id_2);
			costs_agent_id_2[agent_id_2] = cost_agent_id_2;

			// std::cout<<"Agent id 2: "<<agent_id_2<<std::endl;
			if(costs_agent_id_2[agent_id_2] == p.costs[agent_id_2])
			{
				// std::cout<<"inserting right!"<<std::endl;
				PQ.insert(costs_agent_id_2,increase_constraints_agent_id_2,shortestPaths_agent_id_2);
			}
		}

		return std::vector<std::vector<Eigen::VectorXd>>(mNumAgents,std::vector<Eigen::VectorXd>());
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

	std::vector<Vertex> getNeighbors(Graph &graph, Vertex &v)
	{
		std::vector<Vertex> neighbors;
		OutEdgeIter ei, ei_end;

		for (boost::tie(ei, ei_end) = out_edges(v, graph); ei != ei_end; ++ei) 
		{
			Vertex curSucc = target(*ei, graph);
			Edge e = *ei;
			if(!graph[e].isEvaluated)
				evaluateIndividualEdge(graph,e);
			if(graph[e].status == CollisionStatus::FREE)
				neighbors.push_back(curSucc);
		}

		// std::cout<<"neighbors size: "<<neighbors.size()<<std::endl;
		return neighbors;
	}

	void displayPath(std::vector<Eigen::VectorXd> path)
	{
		cv::Mat image;
		cv::cvtColor(mImage, image, CV_GRAY2BGR);
		// cv::Mat image = cv::merge(mImage,mImage,mImage);  // going from one to 3 channel
		int numberOfRows = image.rows;
		int numberOfColumns = image.cols;

		std::vector<cv::Mat4b> number_images(mNumAgents);
		for(int i=0; i<number_images.size(); i++)
		{
			std::stringstream ss;
			ss << "./src/CMAPF/data/viz/";
			ss << i+1;
			ss << ".png";
			number_images[i] = imread(ss.str(), cv::IMREAD_UNCHANGED);
			double scale = 0.037;
			if(i!=0)
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

		std::vector< std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>> > tasks;

		std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>> agent_tasks_4;
		agent_tasks_4.push_back(std::make_pair(std::make_pair(5,2),std::make_pair(6,9)));
		agent_tasks_4.push_back(std::make_pair(std::make_pair(6,10),std::make_pair(6,11)));
		tasks.push_back(agent_tasks_4);

		std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>> agent_tasks_3;
		agent_tasks_3.push_back(std::make_pair(std::make_pair(8,1),std::make_pair(6,7)));
		agent_tasks_3.push_back(std::make_pair(std::make_pair(6,8),std::make_pair(6,12)));
		tasks.push_back(agent_tasks_3);

		std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>> agent_tasks_2;
		agent_tasks_2.push_back(std::make_pair(std::make_pair(5,1),std::make_pair(6,5)));
		agent_tasks_2.push_back(std::make_pair(std::make_pair(6,6),std::make_pair(6,13)));
		tasks.push_back(agent_tasks_2);

		std::vector<std::pair<std::pair<int,int>, std::pair<int,int>>> agent_tasks_1;
		agent_tasks_1.push_back(std::make_pair(std::make_pair(8,4),std::make_pair(6,3)));
		agent_tasks_1.push_back(std::make_pair(std::make_pair(6,4),std::make_pair(6,14)));
		tasks.push_back(agent_tasks_1);

		for(int i=0; i<tasks.size(); i++)
		{
			cv::Scalar col;
			if(i==0) 
				col = cv::Scalar(255,0,0);
			else if(i==1) 
				col = cv::Scalar(0,255,255);
			else if(i==2) 
				col = cv::Scalar(255,255,255);
			else
				col = cv::Scalar(0,0,255);
			for(int j=0; j<tasks[i].size(); j++)
			{
				{
					cv::Point uPoint((int)(tasks[i][j].first.first*0.0625*numberOfColumns), (int)((1 - tasks[i][j].first.second*0.0625)*numberOfRows)); 
					std::string text = std::to_string(4-i) + ((j==0)?"A":"B");
					cv::circle(image, uPoint, 7,  col, -1);
					cv::circle(image, uPoint, 8,  cv::Scalar(0,0,0), 1);
					cv::putText(image, text, cv::Point(uPoint.x - 6,uPoint.y+3), cv::FONT_HERSHEY_PLAIN, 0.6, cvScalar(0,0,0), 1, 4);
				}

				{
					cv::Point uPoint((int)(tasks[i][j].second.first*0.0625*numberOfColumns), (int)((1 - tasks[i][j].second.second*0.0625)*numberOfRows)); 
					std::string text = std::to_string(4-i) + ((j==0)?"A":"B");
					cv::circle(image, uPoint, 7, col, -1);
					cv::circle(image, uPoint, 8,  cv::Scalar(0,0,0), 1);
					cv::putText(image, text, cv::Point(uPoint.x - 6,uPoint.y+3), cv::FONT_HERSHEY_PLAIN, 0.6, cvScalar(0,0,0), 1, 4);
				}
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
		
				// if(i==0)
				// {
				// 	std::string text = "S" + std::to_string(agent_id+1);
				// 	cv::circle(image, uPoint, 7,  cv::Scalar(255,255,255), -1);
				// 	cv::circle(image, uPoint, 8,  cv::Scalar(0,0,0), 1);
				// 	cv::putText(image, text, cv::Point(uPoint.x - 6,uPoint.y+3), cv::FONT_HERSHEY_PLAIN, 0.6, cvScalar(0,0,0), 1, 4);
				// }
				// if(i==pathSize-2)
				// {
				// 	std::string text = "G" + std::to_string(agent_id+1);
				// 	cv::circle(image, vPoint, 7,  cv::Scalar(255,255,255), -1);
				// 	cv::circle(image, vPoint, 8,  cv::Scalar(0,0,0), 1);
				// 	cv::putText(image, text, cv::Point(vPoint.x - 6,vPoint.y+3), cv::FONT_HERSHEY_PLAIN, 0.6, cvScalar(0,0,0), 1, 4);
				// }
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
					// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
					int x = x_point - number_images[agent_id].cols/2;
					int y = y_point - number_images[agent_id].rows/2;
					double alpha = 1.0; // alpha in [0,1]

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
					double alpha = 1.0; // alpha in [0,1]

					// cv::circle(new_image, _Point, 8,  cv::Scalar(0,0,0), 2);
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

	std::vector<Vertex> computeShortestPath(Graph &graph, Vertex &start, Vertex &goal, std::vector<Constraint> &constraints, int initial_timestep, int final_timestep, double& costOut)
	{
		timePriorityQueue pq;
		boost::unordered_map<std::pair<Vertex, int>, double, pair_hash> mDistance;
		boost::unordered_map<std::pair<Vertex, int> , std::pair<Vertex, int>, pair_hash > mPrev;
		boost::unordered_map<int , Vertex> nodeMap;

		pq.insert(graph[start].vertex_index,initial_timestep,graph[start].heuristic,0.0);
		nodeMap[graph[start].vertex_index]=start;

		VertexIter vi, viend;
		mDistance[std::make_pair(start,initial_timestep)]=0;

		int numSearches = 0;
		int maximum_timestep = 10000;

		int goal_timestep = -1;
		while(pq.PQsize()!=0)
		{
			numSearches++;
			// std::cout<<"Queue pop no: "<<numSearches<<std::endl;
			std::pair<int,int> top_element = pq.pop();
			int index = top_element.first;
			int timeStep = top_element.second;
			if(timeStep > final_timestep)
				continue;
			if(index == graph[goal].vertex_index && timeStep == final_timestep)
			{
				// std::cout<<"Timestep goal was found: "<<final_timestep<<std::endl;
				goal_timestep = timeStep;
				costOut = mDistance[std::make_pair(goal,goal_timestep)];
				break;
			}
			Vertex curr_node = nodeMap[index];
			std::vector<Vertex> neighbors = getNeighbors(graph,curr_node);
			neighbors.push_back(curr_node);
			// std::cout<<"No. of neighbors :"<<neighbors.size()<<std::endl;

			for (auto successor : neighbors) 
			{
				if(successor == curr_node)
				{
					bool col = false;
					for( Constraint c: constraints)
					{
						if( c.constraint_type == 1 && successor == c.v && c.t == timeStep + 1)
						{
							// std::cout<<"Constraint Encountered! "<<std::endl;
							col =true;
							break;
						}
					}

					if(!col)
					{
						double new_cost = mDistance[std::make_pair(curr_node,timeStep)] + mUnitEdgeLength;
						
						if(mDistance.count(std::make_pair(successor,timeStep+1))==0 || 
							new_cost < mDistance[std::make_pair(successor,timeStep+1)])
						{
							mDistance[std::make_pair(successor,timeStep+1)]= new_cost;
							double priority;
							// std::cout<<"FOUND FREE EDGE!!"<<std::endl;
							priority = new_cost + graph[successor].heuristic;
							pq.insert(graph[successor].vertex_index,timeStep+1,priority,0.0);
							if(nodeMap.count(graph[successor].vertex_index)==0)
								nodeMap[graph[successor].vertex_index]=successor;
							mPrev[std::make_pair(successor,timeStep+1)]=std::make_pair(curr_node,timeStep);
						}
					}
				}
				else
				{
					Edge uv_edge = boost::edge(curr_node, successor, graph).first;

					bool col = false;
					for( Constraint c: constraints)
					{
						if( (c.constraint_type == 1 && successor == c.v && c.t == timeStep + 1) || (c.constraint_type == 2 && uv_edge == c.e && c.t == timeStep + 1) )
						{
							// std::cout<<"Constraint Encountered! "<<std::endl;
							col =true;
							break;
						}
					}

					if(!col)
					{                   
						double new_cost = mDistance[std::make_pair(curr_node,timeStep)] + mUnitEdgeLength;
						if(mDistance.count(std::make_pair(successor,timeStep+1))==0 || new_cost < mDistance[std::make_pair(successor,timeStep+1)])
						{
							mDistance[std::make_pair(successor,timeStep+1)]= new_cost;
							double priority;
							priority = new_cost + graph[successor].heuristic;
							pq.insert(graph[successor].vertex_index,timeStep+1,priority,0.0);
							if(nodeMap.count(graph[successor].vertex_index)==0)
								nodeMap[graph[successor].vertex_index]=successor;
							mPrev[std::make_pair(successor,timeStep+1)]=std::make_pair(curr_node,timeStep);
						}
					}
				}
			}
		}

		if(goal_timestep == -1)
		{
			costOut = INF;
			return std::vector<Vertex>();
		}

		// std::cout<<"Goal Time: "<<goal_timestep<<std::endl;
		std::vector<Vertex> finalPath;
		Vertex node = goal;

		// std::cout<<"timesteps: ";
		while(!(node == start && goal_timestep == initial_timestep))
		{
			// std::cin.get();
			// std::cout<<"INF LOOP LOL!";
			// std::cout<<goal_timestep<<" ";
			finalPath.push_back(node);
			Vertex temp_node = node;
			int temp_timestep = goal_timestep;
			node=mPrev[std::make_pair(temp_node,temp_timestep)].first;
			goal_timestep=mPrev[std::make_pair(temp_node,temp_timestep)].second;
		}
		// std::cout<<std::endl;
		finalPath.push_back(start);
		std::reverse(finalPath.begin(), finalPath.end());
		return finalPath;
	}
};


} // namespace CMAPF

#endif 
