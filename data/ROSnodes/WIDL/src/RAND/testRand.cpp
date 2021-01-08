#include "rand.h"
#include "histogram.h"
#include "Mat.h"

int main(int argc, char* argv[])
{
	long seed = 180;
	if(argc>=2)
	{
		seed = (long)atoi(argv[1]);
		cout << " SEED = " << seed << endl;
	}
		
	NormalRand instance((double)150,(double)100, seed);
	Mat<double> val(1000,1000);
	for(int i=val.getLine();i--;)
	{
		for(int j=val.getColumn();j--;)
			val.set( instance.dev(), i+1,j+1);
	}
	
	(BHT<double>::histogram(val)).afficher();


}
