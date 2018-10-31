#include "image_properties.h"
#include "videofeed/lane.h"
#include "videofeed/multi_lane.h"
#include "videofeed/calib.h"
#include "videofeed/multi_calib.h"
#include "math.h"

using namespace cv;
using namespace std;

int degree = 3;

ros::Subscriber sub_interpolate;
ros::Publisher pub_pix;

videofeed::calib Lane_data;
videofeed::multi_calib Interpolated_Lane_data;

std::vector<std::vector<Point> > Lane_points;

Mat src;

double poly_angle(std::vector<Point> poly, int idx)
{
	//cout<<"poly_angle() called \n";
	double angle;
	double m1, m2;
	if (idx == 0)
	{
		//Points [size-1], [0], [1]
		m1 = atan2((poly[0].y - poly[poly.size() - 1].y),(poly[0].x- poly[poly.size() - 1].x));
		m2 = atan2((poly[1].y - poly[0].y),(poly[1].x - poly[0].x));
	}
	else if (idx == poly.size()-1)
	{
		//Points [size-2], [size-1], [0]
		m1 = atan2((poly[poly.size() - 1].y - poly[poly.size() - 2].y),(poly[poly.size() - 1].x- poly[poly.size() - 2].x));
		m2 = atan2((poly[0].y - poly[poly.size() - 1].y),(poly[0].x - poly[poly.size() - 1].x));
	}
	else
	{
		//Points [idx-1], [idx], [idx+1]
		m1 = atan2((poly[idx].y - poly[idx - 1].y),(poly[idx].x- poly[idx - 1].x));
		m2 = atan2((poly[idx + 1].y - poly[idx].y),(poly[idx + 1].x - poly[idx].x));
	}

	angle = abs(m1) - abs(m2);
	//cout<<"poly_angle() ends \n";
	return angle;
}


Point deBoor(int k, int degree, int i, float x, std::vector<float> knots, std::vector<Point> ctrlPoints)
{
	
	if(i < 0)
	return ctrlPoints[0];
	
	if(k == 0)
	{
		//cout<<"k = "<< k <<" i = "<< i <<endl;
		//cout<<"ctrlPoints["<< i <<"] = ("<< ctrlPoints[i].x <<" , "<< ctrlPoints[i].y <<") \n";
		return ctrlPoints[i];
	}
	else
	{
		float alpha;
		if(abs(knots[i+degree+1-k] - knots[i]) < 0.1)
		alpha = 0;
		else
		alpha = (x - knots[i])/(knots[i+degree+1-k] - knots[i]);
		
		Point pt = deBoor(k-1, degree, i, x, knots, ctrlPoints)*alpha + deBoor(k-1, degree, i-1, x, knots, ctrlPoints)*(1 - alpha);
		//cout<<"pt = ("<< pt.x <<" , "<< pt.y <<")  alpha = "<< alpha <<"  "; 
		//cout<<"k = "<< k <<" i = "<< i <<endl;
		return pt; 
	}
}

int insert(float x, std::vector<float> knot)
{
	int index;
	
	for(int i=0; i<knot.size(); i++)
	if((knot[i] <= x) && (x < knot[i+1]))
	{
		index = i;
		break;
	}
	
	//cout<<"Index = "<< index <<endl;
	
	return index;
}

std::vector<Point> Get_Spline(std::vector<Point> control_points)
{
	std::vector<Point> spline_points;
	std::vector<float> knot;

	knot.resize(control_points.size() + degree);

	for (int i = 0; i < knot.size(); ++i)
	knot[i] = i*10;

	for (int i = 0; i < degree; ++i)
	knot[knot.size() - i - 1] = knot.size()*10;
	
	//for(float x=knot[0]; x<knot[knot.size()-1]; x=x+5)
	for(float x=knot[0]; x<knot[knot.size()-1]; x=x+(knot[knot.size()-1]-knot[0])/40)
	{
		//cout<<"x = "<< x <<"  ";
		Point pt = deBoor(degree, degree, insert(x, knot), x, knot, control_points);
		spline_points.push_back(pt);
	}

	return spline_points;
}


void Interpolate(std::vector<std::vector<Point> > &Interpolated_Lane_points)
{
	std::vector<std::vector<Point> > Lane_points;
	Lane_points = Interpolated_Lane_points;
	Interpolated_Lane_points.clear();

	std::vector<float> thresh_forward;
	std::vector<float> dist_forward;
	std::vector<int> index_forward;

	std::vector<float> thresh_backward;
	std::vector<float> dist_backward;
	std::vector<int> index_backward;

	int i = 0;
	while(Lane_points.size() != 0)
	{
		Point pt1 = Lane_points[i][0];
		Point pt2 = Lane_points[i][Lane_points[i].size() - 1];

		float m, c;
		m = 0;
		
		if (pt1.x != pt2.x)
		m = (float)(pt1.y - pt2.y)/(float)(pt1.x - pt2.x);

		c = pt1.y - m*pt1.x;

		Point pt;
		if (pt1.x < pt2.x)
		pt.x = pt2.x + (int)sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
		else
		pt.x = pt2.x - (int)sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));

		pt.y =(int)(m*pt.x + c);
		Lane_points[i].push_back(pt);

		std::vector<Point> spline_points;
		spline_points = Get_Spline(Lane_points[i]);

		for (int j = 0; j < Lane_points.size(); ++j)
		{
			if (i != j)
			{
				Point pt = Lane_points[j][0];
				Point pt1 = spline_points[spline_points.size() - 1];
				Point pt2 = spline_points[spline_points.size() - 6];

				float m, c;
				m = 0;

				if (pt1.x != pt2.x)
				m = (float)(pt1.y - pt2.y)/(float)(pt1.x - pt2.x);
				c = pt1.y - m*pt1.x;

				float dist = abs(m*pt.x + c - pt.y)/(pow((1 + m*m), 0.5));
				thresh_forward.push_back(dist);
				index_forward.push_back(j);
			}
		}

		for (int j = 0; j < Lane_points.size(); ++j)
		{
			if (i != j)
			{
				Point pt = Lane_points[j][Lane_points[j].size() - 1];
				Point pt1 = spline_points[0];
				Point pt2 = spline_points[5];

				float m, c;
				m = 0;

				if (pt1.x != pt2.x)
				m = (float)(pt1.y - pt2.y)/(float)(pt1.x - pt2.x);
				c = pt1.y - m*pt1.x;

				float dist = abs(m*pt.x + c - pt.y)/(pow((1 + m*m), 0.5));
				thresh_backward.push_back(dist);
				index_backward.push_back(j);
			}
		}

		Lane_points[i].erase(Lane_points[i].begin() + Lane_points[i].size() - 1);

		if (Lane_points.size() == 1)
		{
			std::vector<Point> temp = Lane_points[i];
			Interpolated_Lane_points.push_back(Lane_points[i]);
			Lane_points.erase(Lane_points.begin() + i);
			i--;

			i++;
			thresh_forward.clear();
			thresh_backward.clear();
			index_forward.clear();
			index_backward.clear();
			continue;
		}

		float min_dist_forward = thresh_forward[0];
		int idx_forward = 0;
		for (int j = 0; j < thresh_forward.size(); ++j)
		{
			if (min_dist_forward > thresh_forward[j])
			{
				min_dist_forward = thresh_forward[j];
				idx_forward = j;
			}
		}

		dist_forward.push_back(min_dist_forward);

		float minimum_dist_forward = dist_forward[0];
		for (int j = 0; j < dist_forward.size(); ++j)
		{
			if (minimum_dist_forward > dist_forward[j])
			minimum_dist_forward = dist_forward[j];
		}

		float min_dist_backward = thresh_backward[0];
		int idx_backward = 0;
		for (int j = 0; j < thresh_backward.size(); ++j)
		{
			if (min_dist_backward > thresh_backward[j])
			{
				min_dist_backward = thresh_backward[j];
				idx_backward = j;
			}
		}

		dist_backward.push_back(min_dist_backward);

		float minimum_dist_backward = dist_backward[0];
		for (int j = 0; j < dist_backward.size(); ++j)
		{
			if (minimum_dist_backward > dist_backward[j])
			minimum_dist_backward = dist_backward[j];
		}

		bool chk_forward1 = (Lane_points[index_forward[idx_forward]][0].y < Lane_points[i][Lane_points[i].size() - 1].y);
		bool chk_backward1 = (Lane_points[index_backward[idx_backward]][Lane_points[index_backward[idx_backward]].size() - 1].y > Lane_points[i][0].y);
		bool chk_forward2, chk_backward2;

		double avg_angle_forward = 0;
		for (int j = 1; j < Lane_points[i].size() - 1; ++j)
		{
			double angle = 180 - poly_angle(Lane_points[i], j);

			avg_angle_forward = avg_angle_forward + angle;
		}
		avg_angle_forward = avg_angle_forward/(Lane_points[i].size() - 1);

		double std_dev_forward = 0;
		for (int j = 1; j < Lane_points[i].size(); ++j)
		{
			std_dev_forward = std_dev_forward + pow((avg_angle_forward - 180 - poly_angle(Lane_points[i], j)), 2);
		}
		std_dev_forward = std_dev_forward/(Lane_points[i].size() - 2);

		Lane_points[i].push_back(Lane_points[index_forward[idx_forward]][0]);
		double angle_forward = 180 - poly_angle(Lane_points[i], Lane_points[i].size() - 2);
		Lane_points[i].erase(Lane_points[i].begin() + Lane_points[i].size() - 1);
		chk_forward2 = (angle_forward > (avg_angle_forward - 2*std_dev_forward)) && (angle_forward < (avg_angle_forward + 2*std_dev_forward));

		double avg_angle_backward = 0;
		for (int j = 1; j < Lane_points[index_backward[idx_backward]].size() - 1; ++j)
		{
			double angle = 180 - poly_angle(Lane_points[index_backward[idx_backward]], j);

			avg_angle_backward = avg_angle_backward + angle;
		}
		avg_angle_backward = avg_angle_backward/(Lane_points[index_backward[idx_backward]].size() - 1);

		double std_dev_backward = 0;
		for (int j = 1; j < Lane_points[index_backward[idx_backward]].size(); ++j)
		{
			std_dev_backward = std_dev_backward + pow((avg_angle_backward - 180 - poly_angle(Lane_points[index_backward[idx_backward]], j)), 2);
		}
		std_dev_backward = std_dev_backward/(Lane_points[index_backward[idx_backward]].size() - 2);

		Lane_points[index_backward[idx_backward]].push_back(Lane_points[i][0]);
		double angle_backward = 180 - poly_angle(Lane_points[index_backward[idx_backward]], Lane_points[index_backward[idx_backward]].size() - 2);
		Lane_points[index_backward[idx_backward]].erase(Lane_points[index_backward[idx_backward]].begin() + Lane_points[index_backward[idx_backward]].size() - 1);
		chk_backward2 = (angle_backward > (avg_angle_backward - 2*std_dev_backward)) && (angle_backward < (avg_angle_backward + 2*std_dev_backward));

		if ((min_dist_forward < min_dist_backward) && (min_dist_forward < 5*minimum_dist_forward) && chk_forward1 && chk_forward2)
		{
			for (int j = 0; j < Lane_points[index_forward[idx_forward]].size(); ++j)
			{
				Point pt = Lane_points[index_forward[idx_forward]][j];
				Lane_points[i].push_back(pt);
			}
			Lane_points.erase(Lane_points.begin() + index_forward[idx_forward]);
			i--;
		}

		else if ((min_dist_forward > min_dist_backward) && (min_dist_backward < 5*minimum_dist_backward) && chk_backward1 && chk_backward2)
		{
			for (int j = 0; j < Lane_points[i].size(); ++j)
			{
				Point pt = Lane_points[i][j];
				Lane_points[index_backward[idx_backward]].push_back(pt);
			}
			Lane_points.erase(Lane_points.begin() + i);
			i--;
		}

		else
		{
			std::vector<Point> temp = Lane_points[i];
			Interpolated_Lane_points.push_back(Lane_points[i]);
			Lane_points.erase(Lane_points.begin() + i);
			i--;
		}
		thresh_forward.clear();
		thresh_backward.clear();
		index_forward.clear();
		index_backward.clear();

		i++;
	};

	for (int i = 0; i < Interpolated_Lane_points.size(); ++i)
	{
		Interpolated_Lane_points[i] = Get_Spline(Interpolated_Lane_points[i]);

		for (int j = 0; j < Interpolated_Lane_points[i].size(); ++j)
		src.at<uchar>(Interpolated_Lane_points[i][j].y, Interpolated_Lane_points[i][j].x) = 150;
	}
}


void interpolate(const videofeed::multi_calib& message)
{
	Interpolated_Lane_data.num_of_lanes = 0;
	
	src = Mat::zeros(Size(image_width, image_height), CV_8UC1);

	for (int i = 0; i < message.num_of_lanes; ++i)
	{
		std::vector<Point> temp;

		for (int j = 0; j < message.Lanes[i].number; ++j)
		{
			temp.push_back(Point(message.Lanes[i].x[j], message.Lanes[i].y[j]));
			circle(src, temp[temp.size() - 1], 2, Scalar(255, 255, 255));
		}

		assert(temp.size() > 0);

		Lane_points.push_back(temp);
		temp.clear();	
	}

	Interpolate(Lane_points);

	for (int i = 0; i < Lane_points.size(); ++i)
	{
		Lane_data.number = 0;
		
		for (int j = 0; j < Lane_points[i].size(); ++j)
		{
			Lane_data.x.push_back(Lane_points[i][j].x);
			Lane_data.y.push_back(Lane_points[i][j].y);
			Lane_data.number++;
		}

		Interpolated_Lane_data.Lanes.push_back(Lane_data);

		Lane_data.x.clear();
		Lane_data.y.clear();

		Interpolated_Lane_data.num_of_lanes++;
	}

	pub_pix.publish(Interpolated_Lane_data);

	for (int i = 0; i < Lane_points.size(); ++i)
		Lane_points[i].clear();

	Lane_points.clear();

	waitKey(1);

	imshow("src", src);

	Interpolated_Lane_data.num_of_lanes = 0;
	Interpolated_Lane_data.Lanes.clear();

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Interpolation");

	ros::NodeHandle nh;

	namedWindow("src", CV_WINDOW_AUTOSIZE);

	sub_interpolate = nh.subscribe("/Interpolater", 1, interpolate);
	pub_pix = nh.advertise<videofeed::multi_calib>("/caliberation", 100);

	ros::Rate loop_rate(5);

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_INFO("videofeed::interpolater.cpp::No error.");
}