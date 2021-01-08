#ifndef HOUGH_T
#define HOUGH_T

class HoughT
{
	private :
	
	int nbrPoints;
	vector<Mat<float> > pointsCart;
	
	float Rmax;			//max value of the normal distance to the origin
	float R_inc;		//increment between each cell, in normal distance R.
	float angle_inc;	//increment between each cell, in normal angle Phi.
	
	int heightR;		//nbr of line for the discretization table.
	int widthA;			//nbr of column for the discretization table.
	
	//Each cell is a vector containing the indexes of the points which belongs to the colinear subset :
	// ------------------------- ? which comes along with its defined parameters, thus the need for float coded values.
	vector<int >** grid;
	Mat<int>* sizeMat;
	Mat<int>* idxCellMat;
	
	vector<Mat<float> > bestMatch;
	
	public :
	
	HoughT(const vector<Mat<float> >& pointsCart_, float Rmax_ = 4.0f, float R_precision = 0.05f, float angle_precision = (float)PI/100) : Rmax(Rmax_), R_inc(R_precision), angle_inc(angle_precision)
	{
		pointsCart = pointsCart_;
		nbrPoints = pointsCart.size();
		
		heightR = (int)((float)2*Rmax / R_inc);
		widthA = (int)((float)PI / angle_inc);
		
		if(heightR != 0 && widthA != 0)
		{
			grid = new vector<int >*[heightR];
			
			for(int i=heightR;i--;)	
			{
				grid[i] = new vector<int >[widthA];	
			}
		}
		else
		{
			cout << "WRONG PARAMETERS... Exiting." << endl;
		}
		
		sizeMat = new Mat<int>(heightR,widthA);
		idxCellMat = new Mat<int>( nbrPoints, heightR*widthA);
		for(int i=nbrPoints;i--;)
			for(int j=heightR*widthA;j--;)
				idxCellMat->set( 0, i+1,j+1);
		
		this->proceed();
		
	}
	
	~HoughT()
	{
		for(int i=heightR;i--;)	delete[] grid[i];
		delete[] grid;
		
		delete sizeMat;
		delete idxCellMat;
		
	}
	
	void proceed()
	{
		//let's go over all the points :
		for(int k=nbrPoints;k--;)
		{
			//for each points we have to go over all the possible discretized parameters :
			float angle;
			float computedR;
			
			float rangeAnglemax;
			float rangeAnglemin;
			float rangeRmax;
			float rangeRmin;
			
			for(int idxA=0;idxA<widthA;idxA++)
			{
				//this loop will stop as soon as the rangeAnglemin will become negative.
				//so it will have cover all the interval within {0,PI}.
				rangeAnglemin = 0 + idxA*angle_inc;
				rangeAnglemax = rangeAnglemin + angle_inc;
				angle = rangeAnglemin + angle_inc/2;
				
				//computeR = pointsCart[k].get(1,1)*cos(angle) + pointsCart[k].get(2,1)*sin(angle);
				computedR = pointsCart[k].get(1,1)*cos(angle) + pointsCart[k].get(2,1)*sin(angle);
				
				for(int idxR=0;idxR<heightR;idxR++)
				{
					rangeRmin = -Rmax + idxR*R_inc;
					rangeRmax = rangeRmin + R_inc;
					
					if(computedR >= rangeRmin && computedR < rangeRmax)
					{
						//then this point belong to this peculiar subset :
						grid[idxR][idxA].push_back( k);
						
						//let's add it to be able to precompute more effectively the table which identify the maximum sized cell to each point :
						idxCellMat->set( 1, k+1, idxR*widthA+idxA+1); 
						
					}
				}
				
			}
		
		}
		
		//let's pinpoint the potential peaks in our grid:
		/*
		for(int i=heightR;i--;)
		{
			for(int j=widthA;j--;)
			{
				sizeMat->set( grid[i][j].size(), i+1,j+1);
			}
		}
		*/
		
		//let's compute a table to identify a maximum sized cell to each point :
		for(int k=nbrPoints;k--;)
		{
			int maxSize = 1;
			int i_idxmax = 0;
			int j_idxmax = 0;
			
			for(int i=heightR;i--;)
			{
				for(int j=widthA;j--;)
				{
					//------------------------------------------------
					//let's pinpoint the potential peaks in out grid :
					sizeMat->set( grid[i][j].size(), i+1,j+1);
					//------------------------------------------------
					
					int tempSize = idxCellMat->get( k+1, i*widthA+j+1);
					//does this point belong to the current cell ?
					if( tempSize == 1)
					{
						//then it might be the maximum that we are seeking :
						tempSize = grid[i][j].size();
						if( tempSize >= maxSize)
						{
							maxSize = tempSize;
							i_idxmax = i;
							j_idxmax = j;
						}
						
						
						//whether it is the current point's maximum-sized cell or not, we register its size.
						tempSize = grid[i][j].size();
						idxCellMat->set( tempSize, k, i*widthA+j+1);
					}
				}
			}
			
			//we currently have the maximum and its position. Let's register it :
			//it is computed in reverse order so we put everything at the beginning of our container :
			float tempR = (-Rmax) + i_idxmax*R_inc;
			float tempAngle = 0 + j_idxmax*angle_inc;
			
			Mat<float> tempLine(2,1);
			tempLine.set( tempR, 1,1);
			tempLine.set( tempAngle, 2,1);
			
			bestMatch.insert(bestMatch.begin(), tempLine); 
		}	
	}
	
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	//---------------------------------------------------------------
	
	
	Mat<int> getSizeMat()
	{
		return *sizeMat;
	}
	
	
	int getHeight()
	{
		return heightR;
	}
	
	int getWidth()
	{
		return widthA;
	}
	
	vector<int>** getGrid()
	{
		return grid;
	}
	

	vector<int> getCell()
	{
		//TODO
		return grid[0][0];
	}
	
	float getIncR()
	{
		return R_inc;
	}
	
	float getIncA()
	{
		return angle_inc;
	}
	
	Mat<float> getLine4Idx(int idx)
	{
		return bestMatch[idx];
	}
	
	vector<Mat<float> > getLines()
	{
		return bestMatch;
	}
	

};

#endif
