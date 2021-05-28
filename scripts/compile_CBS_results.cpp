#include <bits/stdc++.h>
using namespace std;

int main() {
	// your code goes here
	for(int k=0; k<6; k++)
	{
		double count=0;
		double total=0;
		for(int i=0; i<100; i++)
		{
			int a,c;
			double b;
			cin>>a>>b>>c;
			if(a!=0 && b<10)
			{
				total+=b;
				count++;
			}
		}
		std::cout<<total*1.0/count<<std::endl;
	}
	return 0;
}