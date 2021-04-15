#ifndef CBS_DEFINITIONS_
#define CBS_DEFINITIONS_
#include <bits/stdc++.h>
#include "BGLDefinitions.h"
#include "LoadGraphfromFile.h"

namespace CMAPF {

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

struct Constraint
{
	int contraint_type = 1;  // 1 -> vertex constraint 2 -> edge constraint
	Vertex v;
	Edge e;	//edge
	size_t t; //time . for edge, time is time of target vertex

	Constraint(Vertex _v, size_t _t) : v(_v), t(_t), contraint_type(1) {}
	Constraint(Edge _e, size_t _t) : e(_e), t(_t), constraint_type(2) {}
};

struct Element
{
	std::vector<double> costs;
	std::vector<std::vector< Constraint >> constraints;
	std::vector<std::vector<Vertex>> shortestPaths;

	Element(std::vector<double> _costs, std::vector<std::vector<Constraint>> _constraints, std::vector<std::vector<Vertex>> _shortestPaths): 
		costs(_costs), constraints(_constraints), shortestPaths(_shortestPaths) 
		{}

	inline bool operator < (const Element &b) const 
	{
		double cost = 0;
		for(int i=0; i<costs.size(); i++)
			cost += costs[i];

		double b_cost = 0;
		for(int i=0; i<b.costs.size(); i++)
			b_cost += b.costs[i];

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
	size_t mNumAgents;

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
	CBSPriorityQueue(size_t numAgents)
	{ 
		mNumAgents = numAgents;
		Element a(std::vector(mNumAgents, -1),std::vector(mNumAgents, std::vector<Constraint>()),std::vector(mNumAgents,std::vector<Vertex>()));
		PQ.push_back(a);
	}
	void reset()
	{
		PQ.clear();
		Element a(std::vector(mNumAgents, -1),std::vector(mNumAgents, std::vector<Constraint>()),std::vector(mNumAgents,std::vector<Vertex>()));
		PQ.push_back(a);
	}
	int PQsize()
	{
		return PQ.size()-1;
	}
	double topKey()
	{
		double cost = 0;
		for(int i=0; i<costs.size(); i++)
			cost += PQ[1].costs[i];
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
	void insert(std::vector<double> _cost, std::vector<std::vector<Constraint>> _constraints, std::vector<Vertex> _shortestPaths)
	{
		Element a(_cost, _constraints, _shortestPaths);
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
	void printPQ(size_t numAgents)
	{
		cout<<"Elements: "<<endl;
		for(int i=1;i<PQ.size();i++)
		{
			cout<<"Cost: ";
			for(int i=0; i<costs.size(); i++)
				cout<<PQ[i].costs[i]<<" ";
			cout<<endl;

			cout<<"Constraints: "<<endl;
			//print constraints
			for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			{
				std::cout<<" Agent: "<<agent_id<<" - ";
				for(auto it=PQ[i].constraints[agent_id].begin(); it != PQ[i].constraints[agent_id].end(); it++)
				{
					if((*it).contraint_type == 1)
						cout<<"Vertex: "<<(*it).v<<" Time: "<<(*it).t<<endl;
					else
						cout<<"Edge: "<<(*it).e<<" Time: "<<(*it).t<<endl;
				}
			}

			// cout<<"shortestPaths: "<<endl;
			// not print paths as indexmap needed
			cout<<endl;
		}
		cout<<endl;
	}
};

} // namespace CMAPF

#endif