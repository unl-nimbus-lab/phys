#include "GridCell.h"
#include "MATv2/Mat.h"
#include <vector>

#define debug_lvl1

class Grid
{
	private :
	float precX;
	// X precision --> size of a cell in x in meters;
	float precY;
	// Y precision --> size of a cell in y in meters;
	
	int posX0Cell;
	int posY0Cell;
	//record the position in the vector vector map of the origin.
	
	int maxYCellNumber;
	//record the maximum number of cell that has been inserted in a column of the map, so far.
	vector< vector<GridCell> > grid;
	
	public :
	
	Grid() : precX(0.1f),precY(0.1f)
	{
		grid.insert(grid.begin(), vector<GridCell>());
		grid[0].insert(grid[0].begin(), GridCell((float)0,(float)0) );
		grid[0][0].setState(GCfree);
		//necessarily the current cell occupied by the robot is free.
		posX0Cell = 0;
		posY0Cell = 0;
		//record the position in the vector vector map of the origin.
		maxYCellNumber = 1;
	}
	
	Grid(float precX_, float precY_) : precX(precX_),precY(precY_)
	{
		grid.insert(grid.begin(), vector<GridCell>());
		grid[0].insert(grid[0].begin(), GridCell((float)0,(float)0) );
		grid[0][0].setState(GCfree);
		//necessarily the current cell occupied by the robot is free.
		posX0Cell = 0;
		posY0Cell = 0;
		//record the position in the vector vector map of the origin.
		maxYCellNumber = 1;
	}
	
	~Grid()
	{
	
	}
	
	void addGridCell(int state, float r, float theta)	//r in meters, theta in radians.
	{
		float x = r*cos(theta);
		float y = r*sin(theta);
		
		int xbi = floor( (fabs_(x)+precX/2) / precX ); 
		int ybi = floor( (fabs_(y)+precY/2) / precY );
		// example : x = 0.26 : precX = 0.1 : xbi = 3 ; indeed : Cell0 : -0.5--0.5;Cell1:0.5--1.5;Cell2:1.5--2.5; 	Cell3:2.5--3.5;
		
		if(x>0.0f)
		{
		
		
			int nbrCellXPositive = grid.size()-posX0Cell-1;
			float coverXMax = (nbrCellXPositive)*precX+precX/2;
			int nbrFill = 0;
			
			if(  coverXMax < x)
			{
				//then we must fill in until coverXMax > x.
				
				while( coverXMax + nbrFill*precX < x)
					nbrFill++;
					
				//now we know how many cells are to be inserted in x. Let's insert those :
				for(int i=0;i<nbrFill;i++)
				{
					float xb = (i+1)*precX+((grid.size()-1)-posX0Cell)*precX;
					grid.insert(grid.end(), vector<GridCell>());
					//we insert those at the end since its for x positive.
					
					//and we have to put at least the number of rows that is necessary ^^":
					for(int j=0;j<maxYCellNumber;j++)
					{
						float yb = (j-posY0Cell)*precY;
						
						//--------------------------------------
						//What about the state of that new Cell ? for now, let's try to put them free by default...
						int state = GCfree;
						//--------------------------------------
						
						grid[grid.size()-1].insert( grid[grid.size()-1].end(), GridCell(state,xb,yb) );
						//we insert those at the end since its for y positive.
						// keep in mind that we will change the state of that last one that is inserted 
						//since it is the one which has been scanned as a hurdle and that is why we can add it to the grid.
					}	
				}
				
				//no need to update the position of the origin, it hasn't moved.
				//What about the rows now ? see below.
				
			}
#ifdef	debug_lvl1
			cout << " X POSITIVE : nbrFil = " << nbrFill << endl;
#endif						
			//no need to insert any new column.
			//What about the rows ? see below.
			//What about this new cell ? see at the whole end.
			
		}
		else
		{
			// this new cell in of a negative x.
			int nbrCellXNegative = posX0Cell;
			float coverXMin = -(nbrCellXNegative)*precX-precX/2;
			int nbrFill = 0;
			
			if(  coverXMin > x)
			{
				//then we must fill in until coverXMin < x.
				
				while( coverXMin - nbrFill*precX > x)
					nbrFill++;
					
				//now we know how many cells are to be inserted in x. Let's insert those :
				for(int i=0;i<nbrFill;i++)
				{
					float xb = -(i+1)*precX-posX0Cell*precX;
					//float yb = -(j+1)*precY-precY*(posY0Cell);
					//we insert those at the beginning since its for x negative.
					grid.insert(grid.begin(), vector<GridCell>());
					
					//and we have to put at least the number of rows that is necessary ^^":
					for(int j=0;j<maxYCellNumber;j++)
					{
						float yb = (j-posY0Cell)*precY;
						
						//--------------------------------------
						//What about the state of that new Cell ? for now, let's try to put them free by default...
						int state = GCfree;
						//--------------------------------------
						
						grid[grid.size()-1].insert( grid[grid.size()-1].end(), GridCell(state,xb,yb) );
						//we insert those at the end since its for y positive.
						// keep in mind that we will change the state of that last one that is inserted 
						//since it is the one which has been scanned as a hurdle and that is why we can add it to the grid.
					}
				}
				
				//Let's update the position of the origin :
				posX0Cell += nbrFill;
				
				//What about the rows now ? see below.
				
			}
#ifdef	debug_lvl1
			cout << " X NEGATIVE : nbrFil = " << nbrFill << endl;
#endif				
			//no need to insert any new column.
			//What about the rows ? see below.
			//What about this new cell ? see at the whole end.
		
		}
		
		//About the rows :
		if(y>0.0f)
		{
			//maxYCellNumber is assumed to be the size of every column vector.
			int nbrCellYPositive = maxYCellNumber-posY0Cell-1;
			float coverYMax = (nbrCellYPositive+1)*precY+precY/2;
			int nbrFill = 0;
			
			if(  coverYMax < y)
			{
				//then we must fill in until coverYMax > y.
				
				while( coverYMax + nbrFill*precY < y)
					nbrFill++;
					
				//now we know how many cells are to be inserted in every column vector. Let's insert those :
				//but first, let's update the position of the origin in order to use it accordingly : here : nothing to add since we insert at the end.
				
				for(int i=grid.size();i--;)
				{
					float xb = (i-posX0Cell)*precX;
					//again, it is assumed that every column has a size of maxYCellNumber.
					for(int j=0;j<nbrFill;j++)
					{
						float yb = (j+maxYCellNumber)*precY;
						
						//--------------------------------------
						//What about the state of that new Cell ? for now, let's try to put them free by default...
						int state = GCfree;
						//--------------------------------------
						
						grid[i].insert(grid[i].end(), GridCell(state,xb,yb) );
						//we insert those at the end since its for y positive.
						// keep in mind that we will change the state of that last one that is inserted 
						//since it is the one which has been scanned as a hurdle and that is why we can add it to the grid.
					}
				}
				//let's update the number of cells then :
				maxYCellNumber += nbrFill;
				
			}
			
#ifdef	debug_lvl1
			cout << " Y POSITIVE : nbrFil = " << nbrFill << endl;
#endif			
			//What about this new cell ? see at the whole end.
			
		}
		else
		{
			//maxYCellNumber is assumed to be the size of every column vector.
			int nbrCellYNegative = posY0Cell;
			float coverYMin = -(nbrCellYNegative)*precY-precY/2;
			int nbrFill = 0;
			
			if(  coverYMin > y)
			{
				//then we must fill in until coverYMin < y.
				
				while( coverYMin - nbrFill*precY > y)
					nbrFill++;
					
				//now we know how many cells are to be inserted in every column vector. Let's insert those :
				//but first, let's update the position of the origin in order to use it accordingly : 
				posY0Cell += nbrFill;
				
				for(int i=grid.size();i--;)
				{
					float xb = (i-posX0Cell)*precX;
					//again, it is assumed that every column has a size of maxYCellNumber.
					for(int j=0;j<nbrFill;j++)
					{
						float yb = -(j+1)*precY-precY*(posY0Cell);
						
						//--------------------------------------
						//What about the state of that new Cell ? for now, let's try to put them free by default...
						int state = GCfree;
						//--------------------------------------
						
						grid[i].insert(grid[i].begin(), GridCell(state,xb,yb) );
						//we insert those at the beginning since its for y negative.
						// keep in mind that we will change the state of that last one that is inserted 
						// since it is the one which has been scanned as a hurdle and that is why we can add it to the grid.
					}
				}
#ifdef	debug_lvl1
			cout << " Y NEGATIVE : nbrFil = " << nbrFill << endl;
#endif
				
				//let's update the number of cells then :
				maxYCellNumber += nbrFill;
				
			}
			
			//What about this new cell ? see at the whole end.
		}

		//--------------------------------------------------------------		
		//--------------------------------------------------------------		
		//--------------------------------------------------------------
		//SO, let us tackle, at last, this new cell,
		// now that we know that it is mapped within the grid :
		if(x >= 0.0f)
		{
			//we know that we have to access the positive xbi-th column above the origin:
			if(y>=0.0f)
			{
				cout << " x = " << x << " ; y = " << y << " ; xb = " << xbi << " ; yb = " << ybi << endl;
				cout << " i = " << xbi+posX0Cell << " / " << grid.size() << " ; j = " << ybi+posY0Cell << " / " << maxYCellNumber 	<< endl;				
				//we know that we have to access the ybi-th rows above the origin:
				grid[xbi+posX0Cell][ybi+posY0Cell].setState(state);
			}
			else
			{
				cout << " x = " << x << " ; y = " << y << " ; xb = " << xbi << " ; yb = " << ybi << endl;
				cout << " i = " << xbi+posX0Cell << " / " << grid.size() << " ; j = " << -(ybi-1)+posY0Cell << " / " << maxYCellNumber 	<< endl;				
				//we know that we have to access the ybi-th rows below the origin:
				grid[xbi+posX0Cell][-(ybi-1)+posY0Cell].setState(state);
			}
		}
		else
		{
			//we know that we have to access the positive xbi-th column below the origin:
			if(y>=0.0f)
			{
				cout << " x = " << x << " ; y = " << y << " ; xb = " << xbi << " ; yb = " << ybi << endl;
				cout << " i = " << -(xbi-1)+posX0Cell << " / " << grid.size() << " ; j = " << ybi+posY0Cell << " / " << maxYCellNumber 	<< endl;				
				//we know that we have to access the ybi-th rows above the origin:
				grid[-(xbi-1)+posX0Cell][ybi+posY0Cell].setState(state);
			}
			else
			{
				cout << " x = " << x << " ; y = " << y << " ; xb = " << xbi << " ; yb = " << ybi << endl;
				cout << " i = " << -(xbi-1)+posX0Cell << " / " << grid.size() << " ; j = " << -(ybi-1)+posY0Cell << " / " << maxYCellNumber 	<< endl;				
				//we know that we have to access the ybi-th rows below the origin:
				grid[-(xbi-1)+posX0Cell][-(ybi-1)+posY0Cell].setState(state);
			}
		}
		
		//--------------------------------------------------------------		
		//--------------------------------------------------------------		
		//--------------------------------------------------------------
	}
	
	void setGridCellStatePolar(int state, float r, float theta)
	{
		addGridCell(state,r,theta);
	}
	
	void setGridCellStateCart(int state, float x, float y)
	{
		addGridCell(state, sqrt(x*x+y*y), atan2(y,x));
	}
	
	void setGridCellPPolar( float occP, float r, float theta)
	{
		setGridCellPCart( occP, r*cos(theta), r*sin(theta));
	}
	
	void setGridCellPCart( float occP, float x, float y)
	{
		int state = GCfree;
		addGridCell(state, sqrt(x*x+y*y), atan2(y,x));
		
		
		int xbi = floor( (fabs_(x)+precX/2)/precX);
		int ybi = floor( (fabs_(y)+precY/2)/precY);
		
		if(x>0.0f)
		{
			//we know that we have to access the positive xbi-th column above the origin:
			if(y>0.0f)
			{
				//we know that we have to access the ybi-th rows above the origin:
				grid[xbi+posX0Cell][ybi+posY0Cell].setOccP(occP);
			}
			else
			{
				//we know that we have to access the ybi-th rows below the origin:
				grid[xbi+posX0Cell][-ybi+posY0Cell].setOccP(occP);
			}
		}
		else
		{
			//we know that we have to access the positive xbi-th column below the origin:
			if(y>0.0f)
			{
				//we know that we have to access the ybi-th rows above the origin:
				grid[-xbi+posX0Cell][ybi+posY0Cell].setOccP(occP);
			}
			else
			{
				//we know that we have to access the ybi-th rows below the origin:
				grid[-xbi+posX0Cell][-ybi+posY0Cell].setOccP(occP);
			}
		}
	}
	
	
	void setGridCellLPolar( float occL, float r, float theta)
	{
		setGridCellLCart( occL, r*cos(theta), r*sin(theta));
	}
	
	void setGridCellLCart( float occL, float x, float y)
	{
		int state = GCfree;
		addGridCell(state, sqrt(x*x+y*y), atan2(y,x));
		
		int xbi = floor( (fabs_(x)+precX/2)/precX);
		int ybi = floor( (fabs_(y)+precY/2)/precY);
		
		if(x>0.0f)
		{
			//we know that we have to access the positive xbi-th column above the origin:
			if(y>0.0f)
			{
				//we know that we have to access the ybi-th rows above the origin:
				grid[xbi+posX0Cell][ybi+posY0Cell].setOccL(occL);
			}
			else
			{
				//we know that we have to access the ybi-th rows below the origin:
				grid[xbi+posX0Cell][-ybi+posY0Cell].setOccL(occL);
			}
		}
		else
		{
			//we know that we have to access the positive xbi-th column below the origin:
			if(y>0.0f)
			{
				//we know that we have to access the ybi-th rows above the origin:
				grid[-xbi+posX0Cell][ybi+posY0Cell].setOccL(occL);
			}
			else
			{
				//we know that we have to access the ybi-th rows below the origin:
				grid[-xbi+posX0Cell][-ybi+posY0Cell].setOccL(occL);
			}
		}
	}

	inline GridCell* getCell( float x, float y)
	{
		//TODO : incorporate a verification of the data ^^""
		
		
		int xbi = floor( (fabs_(x)+precX/2)/precX);
		int ybi = floor( (fabs_(y)+precY/2)/precY);
		
		if(x>0.0f)
		{
		
			//we know that we have to access the positive xbi-th column above the origin:
			if(y>0.0f)
			{
				if(xbi+posX0Cell > grid.size() || ybi+posY0Cell > maxYCellNumber)
				{
					//then this cell doesn't exist yet :
					int state = GCfree;
					addGridCell(state, sqrt(x*x+y*y), atan2(y,x));
				}
				
				//we know that we have to access the ybi-th rows above the origin:
				return &grid[xbi+posX0Cell][ybi+posY0Cell];
			}
			else
			{
				if(xbi+posX0Cell > grid.size() || ybi+posY0Cell < 0)
				{
					//then this cell doesn't exist yet :
					int state = GCfree;
					addGridCell(state, sqrt(x*x+y*y), atan2(y,x));
				}
				//we know that we have to access the ybi-th rows below the origin:
				return &grid[xbi+posX0Cell][-ybi+posY0Cell];
			}
		}
		else
		{
			//we know that we have to access the positive xbi-th column below the origin:
			if(y>0.0f)
			{
				if(xbi+posX0Cell < 0 || ybi+posY0Cell > maxYCellNumber)
				{
					//then this cell doesn't exist yet :
					int state = GCfree;
					addGridCell(state, sqrt(x*x+y*y), atan2(y,x));
				}
				//we know that we have to access the ybi-th rows above the origin:
				return &grid[-xbi+posX0Cell][ybi+posY0Cell];
			}
			else
			{
				if( xbi+posX0Cell < 0 || ybi+posY0Cell < 0)
				{
					//then this cell doesn't exist yet :
					int state = GCfree;
					addGridCell(state, sqrt(x*x+y*y), atan2(y,x));
				}
				//we know that we have to access the ybi-th rows below the origin:
				return &grid[-xbi+posX0Cell][-ybi+posY0Cell];
			}
		}
	}

	float inverse_sensor_model(const Mat<float>& tempGlobalPose, const Mat<float>& EKFPose, const Mat<float>& EKFPoseCovar, float l0 = 0.5f)
	{
		//TODO : implementation ...
		return 0.8f;
	}
	
	
	void update( /*state*/const Mat<float>& EKFPose, /*CovarianceMatrix*/const Mat<float>&EKFPoseCovar, /*observations*/const vector<Mat<float> >& pointsCartesianPose, const vector<Mat<float> >& pointsPolarPose, float beta = (float)180.0f/600.0f, float alpha = (float)0.1f)
	// those observations are the ones given in the current frame.
	{
	 	int nbrPoints = pointsCartesianPose.size();
	 	Mat<float> tempGlobalPose(2,1);
	 	Mat<float> offset( extract(EKFPose, 1,1, 2,1) );
	 	Mat<float> R2Global(rot2D( -EKFPose.get(3,1))); 
	 	
	 	float tempL;
	 	float l0 = 0.5f;
	 	
	 	for(int k=nbrPoints;k--;)
	 	{
	 		//for each points :
	 		tempGlobalPose = R2Global * (pointsCartesianPose[k]-offset);
	 		
	 		//---------------------
	 		//	L computation :
	 		//tempL = getCell(tempGlobalPose.get(1,1), tempGlobalPose.get(2,1))->getOccL() + inverse_sensor_model(tempGlobalPose, EKFPose, EKFPoseCovar, l0) - l0;
	 		tempL = getCell(tempGlobalPose.get(1,1), tempGlobalPose.get(2,1))->getOccL() + inverse_sensor_model(tempGlobalPose, EKFPose, EKFPoseCovar, l0) - l0;
	 		//---------------------
	 		
	 		setGridCellLCart( tempL,tempGlobalPose.get(1,1), tempGlobalPose.get(2,1) );
	 	} 
	 	
	 	
#ifdef debug_lvl1	
				//Gestion ecriture dans un fichier :
				string filepathgrid("/home/kevidena/ROS/sandbox/WIDL/src/logGRID.txt");
				FILE* loggrid = fopen(filepathgrid.c_str(), "w+");
				if(loggrid == NULL)
				{
					cout << "ERROR : cannot open the file LOGGRID." << endl;
					exit(1);
				}
				else
					cout << "File opened LOGGRID." << endl;
			
				//------------------------------------------
				//------------------------------------------
			
				//Ecriture  :
				for(int i=0;i<grid.size();i++)	
				{
					for(int j=0;j<grid[i].size();j++)
					{
						if( grid[i][j].getState() == GCoccupied)
						{
							stringstream s;
							s << grid[i][j].getXb() << " | " << grid[i][j].getYb() << " | " << i << " | " << j << " | " << grid[i][j].getOccP() << " | " << grid[i][j].getOccL() ; 
							s << endl;
							fputs( s.str().c_str(), loggrid);
						}
					}
				}
				//------------------------------------
				//Fermeture du fichier :
				if(fclose(loggrid) == EOF)
				{
					cout << "ERROR : cannot close the file LOGGRID." << endl;
					exit(1);
				}
				else
					cout << "File closed." << endl;
			
				//--------------------------------
				//--------------------------------
#endif	
	}
	
	Mat<float> rot2D(float angle)
	{
		Mat<float> r(2,2);
		r.set(cos(angle), 1,1);
		r.set(cos(angle), 2,2);
		r.set(-sin(angle), 1,2);
		r.set(sin(angle), 2,1);
		return r;
	}

};
