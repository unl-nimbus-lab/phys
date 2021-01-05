#include <iostream>

#define GCoccupied 1
#define GCfree 0

class GridCell
{
	private :
	
	int state;
	// occupied : 1;
	// free : 0;
	float occP;
	//Occupancy probability.
	float occL;
	//Occupancy log-odds.
	
	public :
	
	float xb;
	float yb;
	float x;
	float y;
	float r;
	float theta;
	
	GridCell() : xb(0), yb(0), x(0.0f), y(0.0f), r(0.0f), theta(0.0f)
	{
		state = GCfree;
		occP = 0.5f;
		occL = log( occP/(1.0f-occP) );
	}
	
	GridCell(float xb_, float yb_) : xb(xb_), yb(yb_), x((float)xb_), y((float)yb_), r( sqrt(xb_*xb_+yb_*yb_) ), theta( atan2(yb_,xb_) )
	{
		state = GCfree;
		occP = 0.5f;
		occL = log( occP/(1.0f-occP) );
	}
	
	GridCell(int state_, float xb_, float yb_) : state(state_), xb(xb_), yb(yb_), x((float)xb_), y((float)yb_), r( sqrt(xb_*xb_+yb_*yb_) ), theta( atan2(yb_,xb_) )
	{
		if(state_ == GCoccupied)
		{
			occP = 0.9f;
			occL = log( occP/(1.0f-occP) );
		}
		else
		{
			occP = 0.1f;
			occL = log( occP/(1.0f-occP) );
		}
	}
	
	~GridCell()
	{
	
	}
	
	//-------------------------
	//		GETTERS
	//-------------------------
	
	int getState()
	{
		return state;
	}
	
	float getOccP()
	{
		return occP;
	}
	
	float getOccL()
	{
		return occL;
	}
	
	float getXb()
	{
		return xb;
	}
	
	float getYb()
	{
		return yb;
	}
	
	//-------------------------
	//		SETTERS
	//-------------------------

	void setState(int state_)
	{
		if(state_ == 0 || state_ == 1)
			state = state_;
	}
	
	void setOccP(float occP_)
	{
		if( occP_ >= 0.0f && occP_ <= 1.0f)
		{
			occP = occP_; 
			occL = log( occP/(1.0f-occP) );
		}
		
		if(occP > 0.5f)
			state = GCoccupied;
	}
	
	void setOccL(float occL_)
	{
		float tempOccP = 1.0f - 1.0f/(1.0f+exp(occL_));
		if(tempOccP >= 0.0f && tempOccP <= 1.0f)
		{
			occP = tempOccP;
			occL = occL_;
		}
		
		if(occP > 0.5f)
			state = GCoccupied;
	}

};
