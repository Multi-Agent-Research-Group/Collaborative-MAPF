#ifndef ISPS_DEFINITIONS_
#define ISPS_DEFINITIONS_
#include <bits/stdc++.h>
#include "BGLDefinitions.hpp"
#include "LoadGraphfromFile.hpp"
#include <boost/functional/hash.hpp>

namespace HCBS {

using namespace BGL_DEFINITIONS;

using namespace std;

struct slackElement
{
	size_t agent_id;
	size_t slack;

	slackElement(size_t _agent_id, size_t _slack): 
		agent_id(_agent_id), slack(_slack)
		{}

	inline bool operator < (const slackElement &b) const 
	{
    	if(slack<b.slack)
			return true;
		else
			return false;
	}
};

class ISPSPriorityQueue
{
private:
	vector <slackElement> PQ;

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
	ISPSPriorityQueue()
	{ 
		slackElement a(0, 1000);
		PQ.push_back(a);
	}
	void reset()
	{
		PQ.clear();
		slackElement a(0, 1000);
		PQ.push_back(a);
	}
	int PQsize()
	{
		return PQ.size()-1;
	}
	double topKey()
	{
		return PQ[1].slack;
	}
	slackElement pop()
	{
		slackElement temp=PQ[1];
		PQ[1]=PQ[PQ.size()-1];
		PQ.erase(PQ.end()-1);
		min_heapify(1);
		return temp;
	}
	void remove(size_t v)
	{
		int i;
		for(i=1;i<PQ.size();i++)
			if(PQ[i].agent_id==v)
				break;
		if(i<PQ.size())
		{
			swap(PQ[i],PQ[PQ.size()-1]);
			PQ.erase(PQ.end()-1);
			// printPQ();
			min_heapify(i);
			// printPQ();
		}
	}
	void insert(size_t agent_id, size_t slack)
	{
		slackElement a(agent_id, slack);
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

	void print()
	{
		cout<<"\nPQ Elements: ";
		for(int i=1;i<PQ.size();i++)
			cout<<PQ[i].agent_id<<":"<<PQ[i].slack<<" ";
		cout<<endl;
	}
};

} // namespace HCBS

#endif