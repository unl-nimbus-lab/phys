#ifndef RAND_H
#define RAND_H

#include <iostream>
#include "Mat.h"

class Rand
{
	private :

	long u;
	long v;
	long w;
	
	public :
	
	Rand(long j)
	{
		v = (long)4101842887655102017LL;
		w = (long)1 ;
		u = j ^ v; 
		int64();
		
		v = u; 
		int64();
		
		w = v; 
		int64();
	}
	
	inline long int64()
	{
	
		u = u * 2862933555777941757LL + 7046029254386353087LL;
		v ^= v >> 17; 
		v ^= v << 31; 
		v ^= v >> 8;

		w = 4294957665U*(w & 0xffffffff) + (w >> 32);
		long x = u ^ (u << 21); 
		x ^= x >> 35; 
		x ^= x << 4;
		
		return (x + v) ^ w;
	}
	
	inline double doub()	{ return 5.42101086242752217E-20 * int64(); }
	
	inline int int32()	{ return (int)int64(); }
	
};


class Ranq1
{
	//Recommended generator for everyday use. The period is 1:8 10 19 . Calling conventions same as Ran, above.

	private :
	
	long v;

	public :
	
	Ranq1(long j) : v(4101842887655102017LL) 
	{
		v ^= j;
		v = int64();

	}

	inline long int64() 
	{
		v ^= v >> 21; v ^= v << 35; v ^= v >> 4;
		return v * 2685821657736338717LL;
	}
	
	inline double doub() { return 5.42101086242752217E-20 * int64(); }
	
	inline int int32() { return (int)int64(); }

};



class Ranfib
{
	private :
	
	double dtab[55], dd;
	int inext, inextp;
	
	public :
	
	Ranfib(long j) : inext(0), inextp(31) 
	{
		//Constructor. Call with any integer seed. Uses Ranq1 to initialize.
		Ranq1 init(j);
		
		for (int k=0; k<55; k++) dtab[k] = init.doub();
	}
	
	double doub() 
	{
		//Returns random double-precision floating value between 0. and 1.
		if (++inext == 55) inext = 0;
		if (++inextp == 55) inextp = 0;
		dd = dtab[inext] - dtab[inextp];
		if (dd < 0) dd += 1.0;
		return (dtab[inext] = dd);
	}
	
	inline unsigned long int32()
	//Returns random 32-bit integer. Recommended only for testing purposes.
	{ return (unsigned long)(doub() * 4294967295.0);}
	
};


class NormalRand : public Ranfib //Ranq1
{
	private :
	
	double mu;
	double sig;
	double storedval;
	
	public :
	
	NormalRand(double mmu, double ssig, long seed) : Ranfib(seed), mu(mmu) ,  sig(ssig), storedval(0.)
	{
		srand(time(NULL));
	
	}
	
	double dev()
	{
	
		double v1,v2,rsq,factor;
		
		if(storedval == 0.)
		{
			do
			{
				v1 = 2.0*doub()-1.0;
				v2 = 2.0*doub()-1.0;
				//v1 = 2.0*((double)(rand()%10000000))/10000000-1.0;
				//v1 = 2.0*((double)(rand()%10000000))/10000000-1.0;
				rsq = v1*v1+v2*v2;
				//cout << " Rsq = " << rsq << endl;
			}while( rsq >= 1.0 || rsq == 0.0);
			
			factor = sqrt( -2.0*log(rsq)/rsq);
			storedval = v1*factor;
			
			return mu+sig*v2*factor;
		}
		else
		{
			factor = storedval;
			storedval = 0.0;
			
			return mu+sig*factor;
		}
		
	}
	
};


#endif


		
