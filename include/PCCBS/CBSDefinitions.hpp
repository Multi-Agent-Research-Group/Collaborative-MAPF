#ifndef CBS_DEFINITIONS_
#define CBS_DEFINITIONS_
#include <bits/stdc++.h>
#include "BGLDefinitions.hpp"
#include "LoadGraphfromFile.hpp"

namespace PCCBS {

using namespace BGL_DEFINITIONS;

using namespace std;

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2;  
    }
}; 

struct CollisionConstraint
{
	int constraint_type;  // 1 -> vertex collision_constraint 2 -> edge collision_constraint
	Vertex v;
	Edge e;	//edge
	int tasks_completed;
	bool in_delivery;
	int timestep; //time . for edge, time is time of target vertex

	CollisionConstraint() : constraint_type(1) {}
	CollisionConstraint(Vertex _v, int _tasks_completed, bool _in_delivery, int _t) 
		: v(_v), tasks_completed(_tasks_completed), in_delivery(_in_delivery), timestep(_t), constraint_type(1) {}
	CollisionConstraint(Edge _e, int _tasks_completed, bool _in_delivery, int _t) 
		: e(_e), tasks_completed(_tasks_completed), in_delivery(_in_delivery), timestep(_t), constraint_type(2) {}
};

struct CollaborationConstraint
{
	Vertex v;
	int task_id;
	bool is_pickup; // false implies delivery
	int timestep;

	CollaborationConstraint() {}
	CollaborationConstraint(Vertex _v, int _task_id, bool _is_pickup, int _t) 
		: v(_v), task_id(_task_id), is_pickup(_is_pickup), timestep(_t){}
};

struct Element
{
	std::vector<int> costs;
	std::vector<std::vector< CollisionConstraint >> collision_constraints;
	std::vector<std::vector< CollaborationConstraint >> collaboration_constraints;
	std::vector<std::vector< CollaborationConstraint >> non_collaboration_constraints;
	std::vector<std::vector<SearchState>> shortestPaths;

	Element(std::vector<int> _costs, std::vector<std::vector<CollisionConstraint>> _collision_constraints,
		std::vector<std::vector<CollaborationConstraint>> _collaboration_constraints, std::vector<std::vector<CollaborationConstraint>> _non_collaboration_constraints,
		std::vector<std::vector<SearchState>> _shortestPaths): 
		costs(_costs), collision_constraints(_collision_constraints), collaboration_constraints(_collaboration_constraints), non_collaboration_constraints(_non_collaboration_constraints), shortestPaths(_shortestPaths) 
		{}

	inline bool operator < (const Element &b) const 
	{
		int cost = 0;
		for(int i=0; i<costs.size(); i++)
			cost = std::max(cost,costs[i]);

		int b_cost = 0;
		for(int i=0; i<b.costs.size(); i++)
			b_cost = std::max(b_cost,b.costs[i]);

		// std::cout<<"Costs: "<<cost<<" "<<b_cost<<std::endl;

    	if(cost<b_cost)
			return true;
		else
			return false;
	}
};

class CBSPriorityQueue
{
private:
	vector <Element> PQ;
	int mNumAgents;

	void min_heapify(int x)
	{
		int l=2*x;
		int r=2*x+1;
		int smallest = x;
		if(l<=(PQ.size()-1) && PQ[l]<PQ[x])
			smallest = l;
		if(r<=(PQ.size()-1) && PQ[r]<PQ[smallest])
			smallest = r;
		if(smallest!=x)
		{
			swap(PQ[smallest],PQ[x]);
			min_heapify(smallest);
		}
	}

public:
	CBSPriorityQueue(int numAgents)
	{ 
		mNumAgents = numAgents;
		Element a(std::vector<int>(mNumAgents, -1),std::vector<std::vector<CollisionConstraint>>(mNumAgents, std::vector<CollisionConstraint>()),
			std::vector<std::vector<CollaborationConstraint>>(mNumAgents, std::vector<CollaborationConstraint>()), std::vector<std::vector<CollaborationConstraint>>(mNumAgents, std::vector<CollaborationConstraint>()),
			std::vector<std::vector<SearchState>>(mNumAgents,std::vector<SearchState>()));
		PQ.push_back(a);
	}
	void reset()
	{
		PQ.clear();
		Element a(std::vector<int>(mNumAgents, -1),std::vector<std::vector<CollisionConstraint>>(mNumAgents, std::vector<CollisionConstraint>()),
			std::vector<std::vector<CollaborationConstraint>>(mNumAgents, std::vector<CollaborationConstraint>()), std::vector<std::vector<CollaborationConstraint>>(mNumAgents, std::vector<CollaborationConstraint>()),
			std::vector<std::vector<SearchState>>(mNumAgents,std::vector<SearchState>()));
		PQ.push_back(a);
	}
	int PQsize()
	{
		return PQ.size()-1;
	}
	int topKey()
	{
		int cost = 0;
		for(int i=0; i<PQ[1].costs.size(); i++)
			cost = std::max(cost,PQ[1].costs[i]);
		return cost;
	}
	Element pop()
	{
		Element temp=PQ[1];
		PQ[1]=PQ[PQ.size()-1];
		PQ.erase(PQ.end()-1);
		min_heapify(1);
		return temp;
	}
	void insert(std::vector<int> _costs, std::vector<std::vector<CollisionConstraint>> _collision_constraints,
		std::vector<std::vector<CollaborationConstraint>> _collaboration_constraints, std::vector<std::vector<CollaborationConstraint>> _non_collaboration_constraints,
		std::vector<std::vector<SearchState>> _shortestPaths)

	{
		Element a(_costs, _collision_constraints, _collaboration_constraints, _non_collaboration_constraints,  _shortestPaths);
		PQ.push_back(a);
		// printPQ();
		int i=PQ.size()-1;
		while((i/2)>0)
		{
			if(PQ[i/2]<PQ[i])
				break;
			else
			{
				swap(PQ[i/2],PQ[i]);
				i=i/2;
			}
		}

	}
	void printPQ(int numAgents)
	{
		cout<<"Elements: "<<endl;
		for(int i=1;i<PQ.size();i++)
		{
			cout<<"Cost: ";
			for(int j=0; j<PQ[i].costs.size(); j++)
				cout<<PQ[i].costs[j]<<" ";
			cout<<endl;

			cout<<"Constraints: "<<endl;
			//print collision_constraints
			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			{
				std::cout<<" Agent: "<<agent_id<<" - ";

				std::cout<<" Collision Constraints - ";
				for(auto it=PQ[i].collision_constraints[agent_id].begin(); it != PQ[i].collision_constraints[agent_id].end(); it++)
				{
					if((*it).constraint_type == 1)
						cout<<"Vertex: "<<(*it).v<<" Time: "<<(*it).timestep<<endl;
					else
						cout<<"Edge: "<<(*it).e<<" Time: "<<(*it).timestep<<endl;
				}

				std::cout<<" Collaboration Constraints - ";
				for(auto it=PQ[i].collaboration_constraints[agent_id].begin(); it != PQ[i].collaboration_constraints[agent_id].end(); it++)
						cout<<"Vertex: "<<(*it).v<<" Time: "<<(*it).timestep<<endl;

				std::cout<<" Non-Collaboration Constraints - ";
				for(auto it=PQ[i].non_collaboration_constraints[agent_id].begin(); it != PQ[i].non_collaboration_constraints[agent_id].end(); it++)
						cout<<"Vertex: "<<(*it).v<<" Time: "<<(*it).timestep<<endl;
			}

			// cout<<"shortestPaths: "<<endl;
			// not print paths as indexmap needed
			cout<<endl;
		}
		cout<<endl;
	}
};

} // namespace PCCBS

#endif