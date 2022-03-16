// Standard C++ libraries
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>
#include <random>

// Boost libraries
// #include <boost/shared_ptr.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/function.hpp>
#include <boost/program_options.hpp>

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Custom header files
#include "../include/utils/graphFileUtils.hpp"

int INF = 100000;

namespace po = boost::program_options;

// using namespace PCCBS;
// using namespace BGL_DEFINITIONS;
using namespace cv;

int mUnitEdgeLength = 1;

std::vector<std::vector<bool>> map;

bool evaluateConfig(Eigen::VectorXd config)
{
	if(map[(int)config[0]][(int)config[1]])
		return true;
	return false;
}

std::vector<std::vector<bool>> extract_map(const std::string map_fname)
{
    std::vector<std::vector<bool>> extracted_map;
    std::ifstream ifs(map_fname);
    std::string line;
    int height, width;
    
    std::getline(ifs,line); // Assuming map type is octile, so skip
    
    std::getline(ifs,line);
    int i = 0;
    for (; i < line.length(); i++) if (std::isdigit(line[i])) break;
    line = line.substr(i,line.length() - i);
    height = std::atoi(line.c_str());
    std::cout << "Height: " << height;

    std::getline(ifs,line);    
    for (i = 0; i < line.length(); i++) if (std::isdigit(line[i])) break;
    line = line.substr(i,line.length() - i);
    width = std::atoi(line.c_str());
    std::cout << " Width: " << width << std::endl;

    std::getline(ifs,line); // This is the line that says "map"

    for (i = 0; i < height; i++)
    {
        std::vector<bool> map_line;
        std::getline(ifs,line);
        for (int j = 0; j < width; j++)
        {
            map_line.push_back(line[j] == '.');
            // std::cout << map_line.back() << " ";
        }
        // std::cout << std::endl;

        extracted_map.push_back(map_line);
    }

    return extracted_map;
}

std::pair <int, int> findCoordinate(Graph graph, Vertex v){
	int x = (int)graph[v].state[0];
	int y = (int)graph[v].state[1];
	return std::make_pair(x, y);
}

std::vector < std::vector < int>> preprocess_graph(std::string weights_file, int mNumVertices)
{
	std::vector < std::vector < int>> edgeWeights(mNumVertices, std::vector <int> (mNumVertices, INF));
	std::ifstream ifs(weights_file);
	for(int i=0; i<mNumVertices; i++){
		for(int j=0; j<mNumVertices; j++){
			int v1; ifs >> v1;
			int v2; ifs >> v2;
			std::string weight; ifs >> weight;
			edgeWeights[v1][v2] = std::stoi(weight);
			edgeWeights[v2][v1] = std::stoi(weight);
		}
	}
	return edgeWeights;
}

int uniform_int_random(int min, int max)
{
	int val = (rand() % (max + 1 - min)) + min;
	return val;
}

bool isCyclicUtil(std::vector <std::vector <int>> adj, int v, bool visited[], bool *recStack)
{
    if(visited[v] == false)
    {
        // Mark the current node as visited and part of recursion stack
        visited[v] = true;
        recStack[v] = true;
 
        // Recur for all the vertices adjacent to this vertex
        // list<int>::iterator i;
        for(auto i: adj[v]){
            if ( !visited[i] && isCyclicUtil(adj, i, visited, recStack) )
                return true;
            else if (recStack[i])
                return true;
        }
 
    }
    recStack[v] = false;  // remove the vertex from recursion stack
    return false;
}
 
// Returns true if the graph contains a cycle, else false.
// This function is a variation of DFS() in https://www.geeksforgeeks.org/archives/18212
bool isCyclic(std::vector <std::vector <int>> adj)
{
	int V = adj.size();
    bool *visited = new bool[V];
    bool *recStack = new bool[V];
    for(int i = 0; i < V; i++)
    {
        visited[i] = false;
        recStack[i] = false;
    }
 
    // Call the recursive helper function to detect cycle in different
    // DFS trees
    for(int i = 0; i < V; i++)
        if ( !visited[i] && isCyclicUtil(adj, i, visited, recStack))
            return true;
 
    return false;
}

int main(int argc, char *argv[])
{
	srand(time(NULL));
	po::options_description desc("Planning Problems Generation Options");
	desc.add_options()
			("help,h", "produce help message")
			("map_file,m",po::value<std::string>()->required(), "path to map file")
			("graph_file,g",po::value<std::string>()->required(), "path to map file")
			("weights_file,w",po::value<std::string>()->required(), "path to map file")
			("output_folder,o",po::value<std::string>()->required(), " path to graphml file")
			("num_agents,n", po::value<int>()->default_value(10), "Number of Agents")
			("single_tasks,s", po::value<int>()->default_value(40), "Number of Tasks")
			("double_tasks,d", po::value<int>()->default_value(10), "Number of Agents")
			("triple_tasks,t", po::value<int>()->default_value(0), "Number of Tasks")
			("precedence_constraints,p", po::value<int>()->default_value(10), "Number of Tasks")
	;

	// Read arguments
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help"))
	{
		std::cout << desc << std::endl;
		return 1;
	}

	std::string graph_file_name(vm["graph_file"].as<std::string>());
	std::string map_file(vm["map_file"].as<std::string>());
	std::string weights_file(vm["weights_file"].as<std::string>());
	std::string output_folder(vm["output_folder"].as<std::string>());
	int numAgents = vm["num_agents"].as<int>();
	int singleTasks = vm["single_tasks"].as<int>();
	int doubleTasks = vm["double_tasks"].as<int>();
	int precedenceConstraints = vm["precedence_constraints"].as<int>();
	// int doubleTasks = 

	map = extract_map(map_file);
	Graph graph;
	create_vertices(graph,get(&VProp::state,graph),graph_file_name,2,get(&EProp::prior,graph));
	create_edges(graph,get(&EProp::length,graph));
	int mNumVertices = num_vertices(graph);
	std::vector < std::vector < int>> edgeWeights = preprocess_graph(weights_file, mNumVertices);

	int count = 0;
	srand(unsigned(time(0)));
	unsigned seed = 0;
	while (count < 20)
	{
		std::cout << "Start making the problem - " + std::to_string(count) + "\n";
		// std::cin.get();
		count++;

		std::vector<Vertex> vertex_list;
		VertexIter vi, vi_end;
			for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
				if(evaluateConfig(graph[*vi].state))
					vertex_list.push_back(*vi);

		// set starting points for each agent
		random_shuffle(vertex_list.begin(), vertex_list.end());
		std::vector<Vertex> agent_inits;
		int cur_vertex = 0;
		for(int agent_id=0; agent_id<numAgents; agent_id++)
		{
			Vertex begin_vertex = vertex_list[cur_vertex];
			agent_inits.push_back(begin_vertex);
			cur_vertex++;
		}

		
		std::vector<int> taskStarts;
		std::vector<int> taskGoals;
		std::vector<int> taskDegrees;

		// set start and goal points for each task
		for(int j=0; j<singleTasks; j++){
			Vertex start_vertex = vertex_list[cur_vertex];
			Vertex goal_vertex = vertex_list[cur_vertex+1];
			taskStarts.push_back(start_vertex);
			taskGoals.push_back(goal_vertex);
			taskDegrees.push_back(1);
			cur_vertex+=2;
		}

		for(int j=0; j<doubleTasks; j++){
			Vertex start_vertex = vertex_list[cur_vertex];
			Vertex goal_vertex = vertex_list[cur_vertex+1];
			taskStarts.push_back(start_vertex);
			taskGoals.push_back(goal_vertex);
			taskDegrees.push_back(2);
			cur_vertex+=2;
		}

		int numTasks = singleTasks+doubleTasks;

		// std::unordered_map <int, int> predMap;
		std::vector <std::vector <int>> predMap (numTasks);
		//make the PC Graph

		// for(int i=0; i<numTasks; i++){
		// 	predMap[i] = -1;
		// }

		int i=0;
		for(int i=0; i<numTasks; i++){
			predMap[i] = std::vector <int> {};
		}

		while(i<precedenceConstraints){
			int task_1 = uniform_int_random(0, taskStarts.size()-1);
			int task_2 = uniform_int_random(0, taskStarts.size()-1);
			if(std::find(predMap[task_2].begin(), predMap[task_2].end(), task_1) == predMap[task_2].end()){
				predMap[task_2].push_back(task_1);
				i++;
			}
			// if(task_1==task_2) std::cout << "same" << std::endl;
		}

		// std::cout << "stuck after graph construction\n";
		//check for cycles
		if(isCyclic(predMap)){
			// std::cout << "Cycle Found in Graph\n";
			count -= 1;
			continue;
		}
		// std::cout << "stuck after cycle check \n";
		// continue;
		std::map <int, std::vector <int>> tasksToAgents;
		std::map <int, std::pair <int,int>> taskTimes;
		std::map <int, std::vector <int>> taskToAgentsLastTask;

		// set start time as 0 and starting position as initial posiition
		std::map <int, std::vector <Vertex>> agentPositions;
		std::map <int, std::vector <int>> agentLastTimes;
		std::map <int, int> agentLastTask;


		for (int i=0; i<numAgents; i++){
			agentPositions[i] = std::vector <Vertex> {agent_inits[i]};
			agentLastTimes[i] = std::vector <int> {0};
			agentLastTask[i] = -1;
		}

		std::vector <std::pair <int,int>> CBS_task_edges;
		std::vector <std::pair <int,int>> ICTS_task_edges;
		std::vector <std::pair <Vertex,Vertex>> ICTS_tasks (numTasks);
		std::vector <std::vector<int>> ICTS_task_agents (numTasks);
		std::map <int, int> ICTSLastTask;
		// greedily assign agents to tasks
		while(true){
			// std::cerr << "Start Iteration" << std::endl;
			//find not assigned tasks
			std::vector <int> not_assigned_tasks;
			for (int i=0; i<numTasks; i++){
				if(tasksToAgents.find(i) == tasksToAgents.end()){
					not_assigned_tasks.push_back(i);
				}
			}
			// std::cout << not_assigned_tasks.size() << std::endl;
			//exit if all tasks assigned
			if(not_assigned_tasks.size() == 0) break;

			//find times to start node of each task
			std::map <int, std::vector <std::pair <int,int>>> taskTimeToStart;
			for (auto task: not_assigned_tasks){
				bool allowed = true;
				for(auto pred_task: predMap[task]){
					if(tasksToAgents.find(pred_task) == tasksToAgents.end()){
						allowed = false;
						continue;
					}
				}
				if(!allowed) continue;
				taskTimeToStart[task] = std::vector <std::pair <int,int>> {};
				Vertex goVertex = taskStarts[task];

				for (int agent=0; agent<numAgents; agent++){

					Vertex curVertex = agentPositions[agent].back();
					int curTime = agentLastTimes[agent].back();

					// std::vector<Vertex> go_path = AStar(image, graph, curVertex, goVertex);
					int go_time = edgeWeights[curVertex][goVertex];
					
					taskTimeToStart[task].push_back(std::make_pair(curTime + go_time, agent));
				}
			}

			//find task which can be completed earliest
			int best_time = 10000000;
			int best_task = -1;
			std::vector <int> best_agents;
			int best_start;

			for(auto task: not_assigned_tasks){
				// int pred_task = predMap[task];
				int collabDegree = taskDegrees[task];
				bool allowed = true;
				int minStartTime = 0;
				for(auto pred_task: predMap[task]){
					if(tasksToAgents.find(pred_task) == tasksToAgents.end()){
						allowed = false;
						continue;
					}
					minStartTime = std::max(minStartTime, taskTimes[pred_task].second);
				}
				if(!allowed) continue;
				//find time to complete Task
				Vertex startVertex = taskStarts[task];
				Vertex goalVertex = taskGoals[task];
				// std::vector<Vertex> task_path = AStar(image, graph, startVertex, goalVertex);
				int task_time = edgeWeights[startVertex][goalVertex];

				//find the closest Agents
				std::sort(taskTimeToStart[task].begin(), taskTimeToStart[task].end());
				
				minStartTime = std::max(minStartTime,
					taskTimeToStart[task][collabDegree-1].first);
				// std::cin.get();
				int end_time = minStartTime + task_time;

				if(end_time < best_time){
					best_time = end_time;
					best_task = task;
					best_start = taskTimeToStart[task][collabDegree-1].first;
					best_agents.clear();

					for(int i=0; i<collabDegree; i++){
						int agent = taskTimeToStart[task][i].second;
						best_agents.push_back(agent);
					}
				}
			}

			// std::cerr << "Best Task = " << best_task << std::endl;
			// std::cerr << "Best Time = " << best_time << std::endl;
			//assign task to agents
			tasksToAgents[best_task] = best_agents;
			// std::cerr << "assign task to agents" << std::endl;
			
			ICTS_tasks[best_task] = (std::make_pair(taskStarts[best_task], taskGoals[best_task]));
			ICTS_task_agents[best_task] = (best_agents);
			int cur_task = ICTS_tasks.size()-1;
			for(auto agent: best_agents){
				if(ICTSLastTask.find(agent) != ICTSLastTask.end()){
					ICTS_tasks.push_back(std::make_pair(taskGoals[agentLastTask[agent]], taskStarts[best_task]));
					ICTS_task_agents.push_back(std::vector <int> {agent});
					ICTS_task_edges.push_back(std::make_pair(ICTS_tasks.size()-1, best_task));
					ICTS_task_edges.push_back(std::make_pair(ICTSLastTask[agent], ICTS_tasks.size()-1));
				}
				else{
					ICTS_tasks.push_back(std::make_pair(agent_inits[agent], taskStarts[best_task]));
					ICTS_task_agents.push_back(std::vector <int> {agent});
					ICTS_task_edges.push_back(std::make_pair(ICTS_tasks.size()-1, best_task));
				}

				ICTSLastTask[agent] = best_task;
			}

			// std::cerr << "ICTS shit" << std::endl;

			for(auto agent: best_agents){				
				if(agentLastTask[agent] != -1){
					CBS_task_edges.push_back(std::make_pair(agentLastTask[agent], best_task));
				}
				agentLastTask[agent] = best_task;
				agentPositions[agent].push_back(taskGoals[best_task]);
				agentLastTimes[agent].push_back(best_time);
			}

			// std::cerr << "CBS shit" << std::endl;
			taskTimes[best_task] = std::make_pair(best_start, best_time);
		}

		bool not_possible_combo = false;
		for(int agent = 0; agent  < numAgents; agent++){
			if(agentLastTask[agent] == -1){
				not_possible_combo = true;
			}
		}
		if(not_possible_combo){
			std::cout << "Not possible bubba\n";
			std::cin.get();
			count -= 1;
			continue;
		}
		int print_time = 0;
		for(auto time: taskTimes){
			print_time = std::max(print_time, time.second.second);
		}
		// std::cout << "Max time = " << print_time << std::endl;
		// std::cout << "possible bubba\n";
		// if(predMap.size()!=precedenceConstraints){
		// 	std::cerr << predMap.size() << std::endl;
		// 	std::cin.get();
		// }
		
		std::ofstream file_stream;
		std::cout << output_folder + "/" + std::to_string(count) + ".txt" << std::endl;
		file_stream.open(output_folder + "/" + std::to_string(count) + ".txt");

		file_stream << std::to_string(ICTS_tasks.size()) + " ";
		file_stream << std::to_string(ICTS_task_edges.size()+precedenceConstraints) + "\n";
		file_stream << std::to_string(numAgents) + "\n";

		file_stream <<"\n";

		for(int i=0; i<ICTS_task_edges.size(); i++){
			file_stream<<std::to_string(ICTS_task_edges[i].first) + " " + 
				std::to_string(ICTS_task_edges[i].second) + "\n";
		}

		// for(auto it: predMap){
		// 	if(it.second==-1) 
		// 		std::cout << "why" << std::endl;
		// 	file_stream<<std::to_string(it.second) + " " + 
		// 		std::to_string(it.first) + "\n";
		// }
		for(int task = 0; task < numTasks; task++){
			for(auto pred_task: predMap[task]){
				file_stream<<std::to_string(pred_task) + " " + 
				std::to_string(task) + "\n";
			}
		}
		file_stream <<"\n";
		for(int i=0; i<ICTS_tasks.size(); i++)
		{
			std::pair <int, int> init = findCoordinate(graph, ICTS_tasks[i].first);
			file_stream<<std::to_string(init.first) + " " + 
				std::to_string(init.second) + " ";

			init = findCoordinate(graph, ICTS_tasks[i].second);
			file_stream<<std::to_string(init.first) + " " + 
				std::to_string(init.second) + "\n";

			file_stream << ICTS_task_agents[i].size() <<" ";
			for(int j=0; j<ICTS_task_agents[i].size(); j++)
				file_stream<< ICTS_task_agents[i][j]<<" ";
			file_stream <<"\n\n";
			// file_stream <<"0 0 0\n\n";
		}
		file_stream.close();
	}
	return 0;
}