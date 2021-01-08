#include "ros/ros.h"            //ROS
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/Image.h> 
#include <sensor_msgs/image_encodings.h>    //Converting ROS to OpenCV images
#include <cv_bridge/cv_bridge.h>            //Converting ROS to OpenCV images
#include <image_transport/image_transport.h>//Publishing and subscribing to images in ROS
#include <opencv2/imgproc/imgproc.hpp>      //Converting ROS to OpenCV images
#include "opencv2/core/core.hpp"            //OpenCV Stitching
#include "opencv2/features2d/features2d.hpp"//OpenCV Stitching
#include "opencv2/highgui/highgui.hpp"      //OpenCV Stitching 
#include "std_msgs/Int16MultiArray.h"
#include "math.h"
#include <sensor_msgs/LaserScan.h>

#define PI 3.14159265

using namespace std;
using namespace cv;

bool j1Recieved, j2Recieved = false;
int points;
Point pt1, pt2;
int sf =2;
float* lineArrayL = new float[180];
float* lineArrayR = new float[180];
float* lidar = new float[180];
int endL, endR;
bool empty = true;

bool check(int x, int y){

	if(x<17){
		return false;
	}else if(x<28){
		if(y<73)
			return false;
	}else if(x<38){
		if(y<57)
			return false;
	}else if(x<45){
		if(y<45)
			return false;
	}else if(x<51){
		if(y<35)
			return false;
	}else if(x<57){
		if(y<25)
			return false;
	}else if(x<62){
		if(y<17)
			return false;
	}


	if(y>117){
		return false;
	}else if(y>111){
		if(x<122)
			return false;
	}else if(y>105){
		if(x<93)
			return false;
	}else if(y>101){
		if(x<70)
			return false;
	}else if(y>96){
		if(x<50)
			return false;
	}else if(y>92){
		if(x<32)
			return false;
	}else if(y>90){
		if(x<18)
			return false;
	}

	if(y<7){
		return false;
	}else if(y<9){
		if(x<137)
			return false;
	}else if(y<11){
		if(x<120)
			return false;
	}else if(y<12){
		if(x<104)
			return false;
	}else if(y<14){
		if(x<89)
			return false;
	}else if(y<16){
		if(x<76)
			return false;
	}else if(y<17){
		if(x<63)
			return false;
	}

	if(x>292){
		return false;
	}else if(x>279){
		if(y<67)
			return false;
	}else if(x>273){
		if(y<57)
			return false;
	}else if(x>266){
		if(y<45)
			return false;
	}else if(x>258){
		if(y<35)
			return false;
	}else if(x>252){
		if(y<25)
			return false;
	}else if(x>247){
		if(y<17)
			return false;
	}


	if(y>117){
		return false;
	}else if(y>109){
		if(x>195)
			return false;
	}else if(y>103){
		if(x>220)
			return false;
	}else if(y>97){
		if(x>242)
			return false;
	}else if(y>93){
		if(x>263)
			return false;
	}else if(y>90){
		if(x>277)
			return false;
	}else if(y>87){
		if(x>292)
			return false;
	}





	return true;
}

void arrayCallbackJ1(const std_msgs::Int16MultiArray::ConstPtr& array){
	cout<<"ENTER1"<<endl;
	j1Recieved = true;
	int size = (array->data[0])*4;
	int real_x,real_y,r_val,angle;
	cout<<"Size: "<<size<<endl;
  //int pointCount = 1;

	int left_x, left_y, right_x, right_y, x_val, y_val, y_offset, y_incrementor;	
	for(int i = 1; i< size; i+=4){
		if(array->data[i] < array->data[i+2]){
			left_x = array->data[i+0];
			left_y = array->data[i+1];
			right_x = array->data[i+2];
			right_y = array->data[i+3];                           
		}else{
			left_x = array->data[i+2];
			left_y = array->data[i+3];
			right_x = array->data[i+0];
			right_y = array->data[i+1];
		}


		y_offset = 0;
		int z;                        
		float slope_m; 

		if((right_y != left_y)&&(right_x!=left_x)){
			z = sqrt((right_x-left_x)*(right_x-left_x) + (right_y-left_y)*(right_y-left_y));                        
			slope_m = (float)(right_y - left_y) / (float)(right_x - left_x); 
			y_offset = 0;
			y_incrementor = (right_y-left_y) / abs(right_y-left_y);
		}else if(right_y == left_y){
			z = abs(right_x - left_x);                        
		}else if(right_x == left_x){
			z = abs(right_y - left_y);                        
		}

		for(int i=0; i<z; i++){
			if(right_y == left_y){
				y_val = left_y;
				x_val = left_x + i;   
			}else if(right_x == left_x){
				y_val = left_y+i;
				x_val = left_x;
			}else if(right_y != left_y){
				if(abs(right_y - left_y)>abs(left_x-right_x)){
					y_val = left_y + y_offset;
					x_val = left_x + abs(y_offset/slope_m);
					y_offset += y_incrementor;
					if(x_val>=right_x)
						i=z+1;
				}else{
					x_val = left_x+i;
					y_val = (i*slope_m)+left_y;
					if(x_val>=right_x)
						i=z+1;
				}
			}

			Point pt = Point(x_val,y_val);
			if(check(x_val, y_val)){
				int region_id=0;


				if(pt.x <= 62){
					if(pt.y<=24){
						region_id=1;
					}else if(pt.y<=34){
						region_id=7;
					}else if(pt.y<=45){
						region_id=13;
					}else if(pt.y<=58){
						if(pt.x <=55){
							region_id=19;
						}else{
							region_id=20;
						}
					}else if(pt.y<=74){
						if(pt.x <=47){
							region_id=25;
						}else{
							region_id=26;
						}
					}else if(pt.y<=94){
						if(pt.x <=38){
							region_id=31;
						}else if(pt.x<=55){
							region_id=32;
						}else{
							region_id=33;
						}
					}
				}else if(pt.x <= 75){
					if(pt.y<=23){
						region_id=1;
					}else if(pt.y<=34){
						if(pt.x <=68){
							region_id=7;
						}else{
							region_id=8;
						}
					}else if(pt.y<=45){
						region_id=14;
					}else if(pt.y<=58){
						if(pt.x <=71){
							region_id=20;
						}else{
							region_id=21;
						}
					}else if(pt.y<=77){
						if(pt.x <=66){
							region_id=26;
						}else{
							region_id=27;
						}
					}else if(pt.y<=100){
						if(pt.x <=38){
							region_id=31;
						}else if(pt.x<=92){
							region_id=33;
						}else{
							region_id=34;
						}
					}
				}else if(pt.x <= 89){
					if(pt.y<=23){
						region_id=2;
					}else if(pt.y<=33){
						if(pt.x <=83){
							region_id=8;
						}else{
							region_id=9;
						}
					}else if(pt.y<=45){
						if(pt.x <=79){
							region_id=14;
						}else{
							region_id=15;
						}
					}else if(pt.y<=60){
						if(pt.x <=87){
							region_id=21;
						}else{
							region_id=22;
						}
					}else if(pt.y<=78){
						if(pt.x <=82){
							region_id=27;
						}else{
							region_id=28;
						}
					}else if(pt.y<=103){
						region_id=34;
					}
				}else if(pt.x <= 104){
					if(pt.y<=21){
						region_id=3;
					}else if(pt.y<=33){
						if(pt.x <=98){
							region_id=9;
						}else{
							region_id=10;
						}
					}else if(pt.y<=45){
						if(pt.x <=94){
							region_id=15;
						}else{
							region_id=16;
						}
					}else if(pt.y<=60){
						if(pt.x <=88){
							region_id=21;
						}else{
							region_id=22;
						}
					}else if(pt.y<=79){
						if(pt.x <=101){
							region_id=28;
						}else{
							region_id=29;
						}
					}else if(pt.y<=103){
						if(pt.x <=96){
							region_id=34;
						}else{
							region_id=35;
						}
					}
				}else if(pt.x <= 119){
					if(pt.y<=21){
						region_id=4;
					}else if(pt.y<=32){
						if(pt.x <=115){
							region_id=10;
						}else{
							region_id=11;
						}
					}else if(pt.y<=45){
						if(pt.x <=112){
							region_id=16;
						}else{
							region_id=17;
						}
					}else if(pt.y<=60){
						if(pt.x <=107){
							region_id=22;
						}else{
							region_id=23;
						}
					}else if(pt.y<=82){
						region_id=29;
					}else if(pt.y<=108){
						if(pt.x <=96){
							region_id=34;
						}else{
							region_id=34;
						}

					} 
				}else if(pt.x <= 137){
					if(pt.y<=20){
						region_id=5;
					}else if(pt.y<=31){
						if(pt.x <=135){
							region_id=11;
						}else{
							region_id=12;
						}
					}else if(pt.y<=45){
						if(pt.x <=133){
							region_id=17;
						}else{
							region_id=18;
						}
					}else if(pt.y<=63){
						if(pt.x <=130){
							region_id=23;
						}else{
							region_id=24;
						}
					}else if(pt.y<=83){
						if(pt.x <=128){
							region_id=29;
						}else{
							region_id=30;
						}
					}else if(pt.y<=112){
						if(pt.x <=124){
							region_id=35;
						}else{
							region_id=36;
						}
					}
				}else if(pt.x <= 157){
					if(pt.y<=17){
						region_id=6;
					}else if(pt.y<=30){
						region_id=12;
					}else if(pt.y<=45){
						region_id=18;
					}else if(pt.y<=64){
						region_id=24;
					}else if(pt.y<=84){
						region_id=30;
					}else if(pt.y<=117){
						region_id=36;    
					}
				}else if(pt.x <= 172){
					if(pt.y<=18){
						region_id=37;
					}else if(pt.y<=30){
						region_id=43;
					}else if(pt.y<=45){
						region_id=49;
					}else if(pt.y<=65){
						region_id=55;
					}else if(pt.y<=84){
						region_id=61;
					}else if(pt.y<=117){
						region_id=67;    
					}
				}else if(pt.x <= 192){
					if(pt.y<=20){
						region_id=38;
					}else if(pt.y<=31){
						if(pt.x <=176){
							region_id=43;
						}else{
							region_id=44;
						}
					}else if(pt.y<=45){
						if(pt.x <=178){
							region_id=49;
						}else{
							region_id=50;
						}
					}else if(pt.y<=64){
						if(pt.x <=182){
							region_id=55;
						}else{
							region_id=56;
						}
					}else if(pt.y<=83){
						if(pt.x <=186){
							region_id=61;
						}else{
							region_id=62;
						}
					}else if(pt.y<=113){
						if(pt.x <=190){
							region_id=67;
						}else{
							region_id=68;
						}
					}
				}else if(pt.x <= 208){
					if(pt.y<=20){
						region_id=39;
					}else if(pt.y<=32){
						if(pt.x <=196){
							region_id=44;
						}else{
							region_id=45;
						}
					}else if(pt.y<=45){
						if(pt.x <=200){
							region_id=50;
						}else{
							region_id=51;
						}
					}else if(pt.y<=63){
						if(pt.x <=204){
							region_id=56;
						}else{
							region_id=57;
						}
					}else if(pt.y<=80){
						region_id=62;
					}else if(pt.y<=109){
						if(pt.x <=193){
							region_id=67;
						}else{
							region_id=68;
						}
					}
				}else if(pt.x <= 221){
					if(pt.y<=22){
						if(pt.x <= 210){
							region_id=39;
						}else{
							region_id=40;
						}
					}else if(pt.y<=32){
						if(pt.x <=213){
							region_id=45;
						}else{
							region_id=46;
						}
					}else if(pt.y<=45){
						if(pt.x <=218){
							region_id=51;
						}else{
							region_id=52;
						}
					}else if(pt.y<=62){
						region_id=57;
					}else if(pt.y<=78){
						if(pt.x<=210){
							region_id=62;  
						}else{
							region_id=63;
						}
					}else if(pt.y<=106){
						if(pt.x <=215){
							region_id=68;
						}else{
							region_id=69;
						}
					}
				}else if(pt.x <= 236){
					if(pt.y<=23){
						if(pt.x <= 223){
							region_id=40;
						}else{
							region_id=41;
						}
					}else if(pt.y<=34){
						if(pt.x <=228){
							region_id=46;
						}else{
							region_id=47;
						}
					}else if(pt.y<=45){
						if(pt.x <=232){
							region_id=52;
						}else{
							region_id=53;
						}
					}else if(pt.y<=61){
						if(pt.x<=224){
							region_id=57;
						}else{
							region_id=58;
						}
					}else if(pt.y<=76){
						if(pt.x<=234){
							region_id=63;  
						}else{
							region_id=64;
						}
					}else if(pt.y<=103){
						if(pt.x <=237){
							region_id=69;
						}else{
							region_id=70;
						}
					}
				}else if(pt.x <= 247){
					if(pt.y<=24){
						if(pt.x <= 238){
							region_id=41;
						}else{
							region_id=42;
						}
					}else if(pt.y<=35){
						if(pt.x <=241){
							region_id=47;
						}else{
							region_id=48;
						}
					}else if(pt.y<=45){
						region_id=53;
					}else if(pt.y<=60){
						if(pt.x<=240){
							region_id=58;
						}else{
							region_id=59;
						}
					}else if(pt.y<=74){
						region_id=64;
					}else if(pt.y<=99){
						region_id=70;
					}
				}else{
					if(pt.y<=24){
						region_id=42;
					}else if(pt.y<=36){
						region_id=48;
					}else if(pt.y<=45){
						if(pt.x<=249){
							region_id=53;
						}else{
							region_id=54;
						}
					}else if(pt.y<=58){
						if(pt.x<=254){
							region_id=59;
						}else{
							region_id=60;
						}
					}else if(pt.y<=72){
						if(pt.x<=250){
							region_id=54;
						}else if(pt.x<=262){
							region_id=65;
						}else{
							region_id=66;
						}
					}else if(pt.y<=99){
						if(pt.x<=256){
							region_id=70;
						}else if(pt.x<=270){
							region_id=71;
						}else{
							region_id=72;
						}
					}
				}

				switch(region_id){

					case(1):
					real_x = ((((74-pt.x)/16.0)*12.0)+60);
					real_y = ((((23-pt.y)/8.0)*12.0)+60);
					break;
					case(2):
					real_x = ((((89-pt.x)/16.0)*12.0)+48);
					real_y = ((((23-pt.y)/8.0)*12.0)+60);
					break;
					case(3):
					real_x = ((((103-pt.x)/16.0)*12.0)+36);
					real_y = ((((22-pt.y)/9.0)*12.0)+60);
					break;
					case(4):
					real_x = ((((118-pt.x)/16.0)*12.0)+24);
					real_y = ((((20-pt.y)/10.0)*12.0)+60);
					break;
					case(5):
					real_x = ((((137-pt.x)/18.0)*12.0)+12);
					real_y = ((((21-pt.y)/10.0)*12.0)+60);
					break;
					case(6):
					real_x = ((((157-pt.x)/20.0)*12.0)+0);
					real_y = ((((18-pt.y)/11.0)*12.0)+60);
					break;

					case(7):
					real_x = ((((71-pt.x)/9.0)*12.0)+60);
					real_y = ((((34-pt.y)/10.0)*12.0)+48);
					break;
					case(8):
					real_x = ((((85-pt.x)/19.0)*12.0)+48);
					real_y = ((((34-pt.y)/9.0)*12.0)+48);
					break;
					case(9):
					real_x = ((((101-pt.x)/19.0)*12.0)+36);
					real_y = ((((33-pt.y)/11.0)*12.0)+48);
					break;
					case(10):
					real_x = ((((117-pt.x)/19.0)*12.0)+24);
					real_y = ((((33-pt.y)/12.0)*12.0)+48);
					break;
					case(11):
					real_x = ((((135-pt.x)/20.0)*12.0)+12);
					real_y = ((((31-pt.y)/13.0)*12.0)+48);
					break;
					case(12):
					real_x = ((((157-pt.x)/22.0)*12.0)+0);
					real_y = ((((31-pt.y)/13.0)*12.0)+48);
					break;

					case(13):
					real_x = ((((65-pt.x)/19.0)*12.0)+60);
					real_y = ((((45-pt.y)/9.0)*12.0)+36);
					break;
					case(14):
					real_x = ((((80-pt.x)/20.0)*12.0)+48);
					real_y = ((((45-pt.y)/10.0)*12.0)+36);
					break;
					case(15):
					real_x = ((((96-pt.x)/21.0)*12.0)+36);
					real_y = ((((45-pt.y)/11.0)*12.0)+36);
					break;
					case(16):
					real_x = ((((113-pt.x)/20.0)*12.0)+24);
					real_y = ((((45-pt.y)/13.0)*12.0)+36);
					break;
					case(17):
					real_x = ((((134-pt.x)/23.0)*12.0)+12);
					real_y = ((((45-pt.y)/14.0)*12.0)+36);
					break;
					case(18):
					real_x = ((((157-pt.x)/25.0)*12.0)+0);
					real_y = ((((45-pt.y)/15.0)*12.0)+36);
					break;

					case(19):
					real_x = ((((59-pt.x)/20.0)*12.0)+60);
					real_y = ((((58-pt.y)/13.0)*12.0)+24);
					break;
					case(20):
					real_x = ((((74-pt.x)/22.0)*12.0)+48);
					real_y = ((((60-pt.y)/15.0)*12.0)+24);
					break;
					case(21):
					real_x = ((((92-pt.x)/24.0)*12.0)+36);
					real_y = ((((61-pt.y)/16.0)*12.0)+24);
					break;  
					case(22):
					real_x = ((((110-pt.x)/24.0)*12.0)+24);
					real_y = ((((62-pt.y)/17.0)*12.0)+24);
					break;  
					case(23):
					real_x = ((((132-pt.x)/26.0)*12.0)+12);
					real_y = ((((64-pt.y)/19.0)*12.0)+24);
					break;  
					case(24):
					real_x = ((((157-pt.x)/28.0)*12.0)+0);
					real_y = ((((66-pt.y)/21.0)*12.0)+24);
					break;

					case(25):
					real_x = ((((51-pt.x)/22.0)*12.0)+60);
					real_y = ((((58-pt.y)/17.0)*12.0)+12);
					break;  
					case(26):
					real_x = ((((68-pt.x)/25.0)*12.0)+48);
					real_y = ((((60-pt.y)/18.0)*12.0)+12);
					break;
					case(27):
					real_x = ((((85-pt.x)/25.0)*12.0)+36);
					real_y = ((((77-pt.y)/18.0)*12.0)+12);
					break;  
					case(28):
					real_x = ((((105-pt.x)/25.0)*12.0)+24);
					real_y = ((((80-pt.y)/20.0)*12.0)+12);
					break;  
					case(29):
					real_x = ((((129-pt.x)/29.0)*12.0)+12);
					real_y = ((((83-pt.y)/21.0)*12.0)+12);
					break;  
					case(30):
					real_x = ((((157-pt.x)/29.0)*12.0)+0);
					real_y = ((((86-pt.y)/23.0)*12.0)+12);
					break;          

					case(31):
					real_x = ((((42-pt.x)/24.0)*12.0)+60);
					real_y = ((((92-pt.y)/19.0)*12.0)+0);
					break;  
					case(32):
					real_x = ((((59-pt.x)/26.0)*12.0)+48);
					real_y = ((((96-pt.y)/21.0)*12.0)+0);
					break;
					case(33):
					real_x = ((((79-pt.x)/30.0)*12.0)+36);
					real_y = ((((100-pt.y)/24.0)*12.0)+0);
					break;  
					case(34):
					real_x = ((((105-pt.x)/29.0)*12.0)+24);
					real_y = ((((103-pt.y)/26.0)*12.0)+0);
					break;  
					case(35):
					real_x = ((((126-pt.x)/34.0)*12.0)+12);
					real_y = ((((110-pt.y)/30.0)*12.0)+0);
					break;  
					case(36):
					real_x = ((((157-pt.x)/34.0)*12.0)+0);
					real_y = ((((118-pt.y)/32.0)*12.0)+0);
					break;

		  /*RIGHT CAMERA*/

					case(37):
					real_x = ((((pt.x-157)/16.0)*12.0)+0);
					real_y = ((((19-pt.y)/11.0)*12.0)+60);
					break;  
					case(38):
					real_x = ((((pt.x-173)/18.0)*12.0)+12);
					real_y = ((((20-pt.y)/11.0)*12.0)+60);
					break;
					case(39):
					real_x = ((((pt.x-193)/16.0)*12.0)+24);
					real_y = ((((21-pt.y)/9.0)*12.0)+60);
					break;  
					case(40):
					real_x = ((((pt.x-209)/13.0)*12.0)+36);
					real_y = ((((22-pt.y)/9.0)*12.0)+60);
					break;  
					case(41):
					real_x = ((((pt.x-222)/13.0)*12.0)+48);
					real_y = ((((24-pt.y)/9.0)*12.0)+60);
					break;  
					case(42):
					real_x = ((((pt.x-237)/34.0)*12.0)+60);
					real_y = ((((24-pt.y)/8.0)*12.0)+60);
					break; 

					case(43):
					real_x = ((((pt.x-157)/18.0)*12.0)+0);
					real_y = ((((31-pt.y)/12.0)*12.0)+48);
					break;  
					case(44):
					real_x = ((((pt.x-175)/22.0)*12.0)+12);
					real_y = ((((32-pt.y)/12.0)*12.0)+48);
					break;
					case(45):
					real_x = ((((pt.x-197)/16.0)*12.0)+24);
					real_y = ((((33-pt.y)/12.0)*12.0)+48);
					break;  
					case(46):
					real_x = ((((pt.x-213)/14.0)*12.0)+36);
					real_y = ((((34-pt.y)/12.0)*12.0)+48);
					break;  
					case(47):
					real_x = ((((pt.x-227)/14.0)*12.0)+48);
					real_y = ((((34-pt.y)/10.0)*12.0)+48);
					break;  
					case(48):
					real_x = ((((pt.x-241)/14.0)*12.0)+60);
					real_y = ((((34-pt.y)/10.0)*12.0)+48);
					break; 


					case(49):
					real_x = ((((pt.x-157)/21.0)*12.0)+0);
					real_y = ((((45-pt.y)/14.0)*12.0)+36);
					break;  
					case(50):
					real_x = ((((pt.x-178)/22.0)*12.0)+12);
					real_y = ((((45-pt.y)/13.0)*12.0)+36);
					break;
					case(51):
					real_x = ((((pt.x-200)/18.0)*12.0)+24);
					real_y = ((((45-pt.y)/12.0)*12.0)+36);
					break;  
					case(52):
					real_x = ((((pt.x-218)/15.0)*12.0)+36);
					real_y = ((((45-pt.y)/11.0)*12.0)+36);
					break;  
					case(53):
					real_x = ((((pt.x-233)/15.0)*12.0)+48);
					real_y = ((((45-pt.y)/11.0)*12.0)+36);
					break;  
					case(54):
					real_x = ((((pt.x-248)/13.0)*12.0)+60);
					real_y = ((((45-pt.y)/10.0)*12.0)+36);
					break; 


					case(55):
					real_x = ((((pt.x-157)/25.0)*12.0)+0);
					real_y = ((((64-pt.y)/18.0)*12.0)+24);
					break;  
					case(56):
					real_x = ((((pt.x-182)/23.0)*12.0)+12);
					real_y = ((((62-pt.y)/17.0)*12.0)+24);
					break;
					case(57):
					real_x = ((((pt.x-205)/18.0)*12.0)+24);
					real_y = ((((60-pt.y)/15.0)*12.0)+24);
					break;  
					case(58):
					real_x = ((((pt.x-223)/18.0)*12.0)+36);
					real_y = ((((59-pt.y)/14.0)*12.0)+24);
					break;  
					case(59):
					real_x = ((((pt.x-241)/13.0)*12.0)+48);
					real_y = ((((58-pt.y)/13.0)*12.0)+24);
					break;  
					case(60):
					real_x = ((((pt.x-254)/15.0)*12.0)+60);
					real_y = ((((57-pt.y)/12.0)*12.0)+24);
					break; 


					case(61):
					real_x = ((((pt.x-157)/29.0)*12.0)+0);
					real_y = ((((83-pt.y)/19.0)*12.0)+12);
					break;  
					case(62):
					real_x = ((((pt.x-186)/24.0)*12.0)+12);
					real_y = ((((62-pt.y)/18.0)*12.0)+12);
					break;
					case(63):
					real_x = ((((pt.x-210)/20.0)*12.0)+24);
					real_y = ((((76-pt.y)/16.0)*12.0)+12);
					break;  
					case(64):
					real_x = ((((pt.x-230)/18.0)*12.0)+36);
					real_y = ((((73-pt.y)/14.0)*12.0)+12);
					break;  
					case(65):
					real_x = ((((pt.x-248)/14.0)*12.0)+48);
					real_y = ((((71-pt.y)/13.0)*12.0)+12);
					break;  
					case(66):
					real_x = ((((pt.x-262)/15.0)*12.0)+60);
					real_y = ((((69-pt.y)/12.0)*12.0)+12);
					break; 


					case(67):
					real_x = ((((pt.x-157)/34.0)*12.0)+0);
					real_y = ((((113-pt.y)/30.0)*12.0)+0);
					break;  
					case(68):
					real_x = ((((pt.x-191)/25.0)*12.0)+12);
					real_y = ((((107-pt.y)/27.0)*12.0)+0);
					break;
					case(69):
					real_x = ((((pt.x-216)/21.0)*12.0)+24);
					real_y = ((((101-pt.y)/25.0)*12.0)+0);
					break;  
					case(70):
					real_x = ((((pt.x-237)/19.0)*12.0)+36);
					real_y = ((((96-pt.y)/23.0)*12.0)+0);
					break;  
					case(71):
					real_x = ((((pt.x-256)/15.0)*12.0)+48);
					real_y = ((((93-pt.y)/22.0)*12.0)+0);
					break;  
					case(72):
					real_x = ((((pt.x-271)/15.0)*12.0)+60);
					real_y = ((((89-pt.y)/20.0)*12.0)+0);
					break; 

					default:
					break;

				}


				r_val = sqrt(1.0*(real_x*real_x)+(1.0*(real_y+11)*(1.0*real_y+11)));
				if(pt.x<=157){
					angle =  180 - atan(1.0*(real_y+11)/(real_x))*180.0/3.14159;
				}else{
					angle =  90-(atan(1.0*(real_x)/(real_y+11))*180.0/3.14159) ;
				}
				lineArrayL[angle] = r_val*0.0254;

				if(i==0)
					endL = angle;






	  //cout<<"J1Point "<<pointCount<<": ["<<left_x<<","<<left_y<<"]"<<endl;
	  //cout<<"J1Point "<<pointCount<<": ["<<right_x<<","<<right_y<<"]"<<endl;
	  //pointCount+=2;
			}
		}
	}
}

void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan){
	int count = 0;
	for(int i = 89; i>=14; i--){
		lidar[i] = scan->ranges[count];
		count++;
	}
	count = 285;

	for(int i = 164; i>=90; i--){
		lidar[i] = scan->ranges[count];
		count++;
	}



}

void arrayCallbackJ2(const std_msgs::Int16MultiArray::ConstPtr& array){
	cout<<"ENTER2"<<endl;
	j2Recieved = true;
	int size = (array->data[0])*4;
	int real_x,real_y,r_val,angle;
	cout<<"Size: "<<size<<endl;
  //int pointCount = 1;

	int left_x, left_y, right_x, right_y, x_val, y_val, y_offset, y_incrementor;  
	for(int i = 1; i< size; i+=4){
	//cout<<array->data[i]<<endl;
		if(array->data[i] < array->data[i+2]){
			left_x = array->data[i+0]+157;
			left_y = array->data[i+1];
			right_x = array->data[i+2]+157;
			right_y = array->data[i+3];                           
		}else{
			left_x = array->data[i+2]+157;
			left_y = array->data[i+3];
			right_x = array->data[i+0]+157;
			right_y = array->data[i+1];
		}

		y_offset = 0;
		int z;                        
		float slope_m; 

		if((right_y != left_y)&&(right_x!=left_x)){
			z = sqrt((right_x-left_x)*(right_x-left_x) + (right_y-left_y)*(right_y-left_y));                        
			slope_m = (float)(right_y - left_y) / (float)(right_x - left_x); 
			y_offset = 0;
			y_incrementor = (right_y-left_y) / abs(right_y-left_y);
		}else if(right_y == left_y){
			z = abs(right_x - left_x);                        
		}else if(right_x == left_x){
			z = abs(right_y - left_y);                        
		}

		for(int i=0; i<z; i++){
			if(right_y == left_y){
				y_val = left_y;
				x_val = left_x + i;   
			}else if(right_x == left_x){
				y_val = left_y+i;
				x_val = left_x;
			}else if(right_y != left_y){
				if(abs(right_y - left_y)>abs(left_x-right_x)){
					y_val = left_y + y_offset;
					x_val = left_x + abs(y_offset/slope_m);
					y_offset += y_incrementor;
					if(x_val>=right_x)
						i=z+1;
				}else{
					x_val = left_x+i;
					y_val = (i*slope_m)+left_y;
					if(x_val>=right_x)
						i=z+1;
				}
			}

			Point pt = Point(x_val,y_val);
			if(check(x_val, y_val)){
				int region_id=0;


				if(pt.x <= 62){
					if(pt.y<=24){
						region_id=1;
					}else if(pt.y<=34){
						region_id=7;
					}else if(pt.y<=45){
						region_id=13;
					}else if(pt.y<=58){
						if(pt.x <=55){
							region_id=19;
						}else{
							region_id=20;
						}
					}else if(pt.y<=74){
						if(pt.x <=47){
							region_id=25;
						}else{
							region_id=26;
						}
					}else if(pt.y<=94){
						if(pt.x <=38){
							region_id=31;
						}else if(pt.x<=55){
							region_id=32;
						}else{
							region_id=33;
						}
					}
				}else if(pt.x <= 75){
					if(pt.y<=23){
						region_id=1;
					}else if(pt.y<=34){
						if(pt.x <=68){
							region_id=7;
						}else{
							region_id=8;
						}
					}else if(pt.y<=45){
						region_id=14;
					}else if(pt.y<=58){
						if(pt.x <=71){
							region_id=20;
						}else{
							region_id=21;
						}
					}else if(pt.y<=77){
						if(pt.x <=66){
							region_id=26;
						}else{
							region_id=27;
						}
					}else if(pt.y<=100){
						if(pt.x <=38){
							region_id=31;
						}else if(pt.x<=92){
							region_id=33;
						}else{
							region_id=34;
						}
					}
				}else if(pt.x <= 89){
					if(pt.y<=23){
						region_id=2;
					}else if(pt.y<=33){
						if(pt.x <=83){
							region_id=8;
						}else{
							region_id=9;
						}
					}else if(pt.y<=45){
						if(pt.x <=79){
							region_id=14;
						}else{
							region_id=15;
						}
					}else if(pt.y<=60){
						if(pt.x <=87){
							region_id=21;
						}else{
							region_id=22;
						}
					}else if(pt.y<=78){
						if(pt.x <=82){
							region_id=27;
						}else{
							region_id=28;
						}
					}else if(pt.y<=103){
						region_id=34;
					}
				}else if(pt.x <= 104){
					if(pt.y<=21){
						region_id=3;
					}else if(pt.y<=33){
						if(pt.x <=98){
							region_id=9;
						}else{
							region_id=10;
						}
					}else if(pt.y<=45){
						if(pt.x <=94){
							region_id=15;
						}else{
							region_id=16;
						}
					}else if(pt.y<=60){
						if(pt.x <=88){
							region_id=21;
						}else{
							region_id=22;
						}
					}else if(pt.y<=79){
						if(pt.x <=101){
							region_id=28;
						}else{
							region_id=29;
						}
					}else if(pt.y<=103){
						if(pt.x <=96){
							region_id=34;
						}else{
							region_id=35;
						}
					}
				}else if(pt.x <= 119){
					if(pt.y<=21){
						region_id=4;
					}else if(pt.y<=32){
						if(pt.x <=115){
							region_id=10;
						}else{
							region_id=11;
						}
					}else if(pt.y<=45){
						if(pt.x <=112){
							region_id=16;
						}else{
							region_id=17;
						}
					}else if(pt.y<=60){
						if(pt.x <=107){
							region_id=22;
						}else{
							region_id=23;
						}
					}else if(pt.y<=82){
						region_id=29;
					}else if(pt.y<=108){
						if(pt.x <=96){
							region_id=34;
						}else{
							region_id=34;
						}

					} 
				}else if(pt.x <= 137){
					if(pt.y<=20){
						region_id=5;
					}else if(pt.y<=31){
						if(pt.x <=135){
							region_id=11;
						}else{
							region_id=12;
						}
					}else if(pt.y<=45){
						if(pt.x <=133){
							region_id=17;
						}else{
							region_id=18;
						}
					}else if(pt.y<=63){
						if(pt.x <=130){
							region_id=23;
						}else{
							region_id=24;
						}
					}else if(pt.y<=83){
						if(pt.x <=128){
							region_id=29;
						}else{
							region_id=30;
						}
					}else if(pt.y<=112){
						if(pt.x <=124){
							region_id=35;
						}else{
							region_id=36;
						}
					}
				}else if(pt.x <= 157){
					if(pt.y<=17){
						region_id=6;
					}else if(pt.y<=30){
						region_id=12;
					}else if(pt.y<=45){
						region_id=18;
					}else if(pt.y<=64){
						region_id=24;
					}else if(pt.y<=84){
						region_id=30;
					}else if(pt.y<=117){
						region_id=36;    
					}
				}else if(pt.x <= 172){
					if(pt.y<=18){
						region_id=37;
					}else if(pt.y<=30){
						region_id=43;
					}else if(pt.y<=45){
						region_id=49;
					}else if(pt.y<=65){
						region_id=55;
					}else if(pt.y<=84){
						region_id=61;
					}else if(pt.y<=117){
						region_id=67;    
					}
				}else if(pt.x <= 192){
					if(pt.y<=20){
						region_id=38;
					}else if(pt.y<=31){
						if(pt.x <=176){
							region_id=43;
						}else{
							region_id=44;
						}
					}else if(pt.y<=45){
						if(pt.x <=178){
							region_id=49;
						}else{
							region_id=50;
						}
					}else if(pt.y<=64){
						if(pt.x <=182){
							region_id=55;
						}else{
							region_id=56;
						}
					}else if(pt.y<=83){
						if(pt.x <=186){
							region_id=61;
						}else{
							region_id=62;
						}
					}else if(pt.y<=113){
						if(pt.x <=190){
							region_id=67;
						}else{
							region_id=68;
						}
					}
				}else if(pt.x <= 208){
					if(pt.y<=20){
						region_id=39;
					}else if(pt.y<=32){
						if(pt.x <=196){
							region_id=44;
						}else{
							region_id=45;
						}
					}else if(pt.y<=45){
						if(pt.x <=200){
							region_id=50;
						}else{
							region_id=51;
						}
					}else if(pt.y<=63){
						if(pt.x <=204){
							region_id=56;
						}else{
							region_id=57;
						}
					}else if(pt.y<=80){
						region_id=62;
					}else if(pt.y<=109){
						if(pt.x <=193){
							region_id=67;
						}else{
							region_id=68;
						}
					}
				}else if(pt.x <= 221){
					if(pt.y<=22){
						if(pt.x <= 210){
							region_id=39;
						}else{
							region_id=40;
						}
					}else if(pt.y<=32){
						if(pt.x <=213){
							region_id=45;
						}else{
							region_id=46;
						}
					}else if(pt.y<=45){
						if(pt.x <=218){
							region_id=51;
						}else{
							region_id=52;
						}
					}else if(pt.y<=62){
						region_id=57;
					}else if(pt.y<=78){
						if(pt.x<=210){
							region_id=62;  
						}else{
							region_id=63;
						}
					}else if(pt.y<=106){
						if(pt.x <=215){
							region_id=68;
						}else{
							region_id=69;
						}
					}
				}else if(pt.x <= 236){
					if(pt.y<=23){
						if(pt.x <= 223){
							region_id=40;
						}else{
							region_id=41;
						}
					}else if(pt.y<=34){
						if(pt.x <=228){
							region_id=46;
						}else{
							region_id=47;
						}
					}else if(pt.y<=45){
						if(pt.x <=232){
							region_id=52;
						}else{
							region_id=53;
						}
					}else if(pt.y<=61){
						if(pt.x<=224){
							region_id=57;
						}else{
							region_id=58;
						}
					}else if(pt.y<=76){
						if(pt.x<=234){
							region_id=63;  
						}else{
							region_id=64;
						}
					}else if(pt.y<=103){
						if(pt.x <=237){
							region_id=69;
						}else{
							region_id=70;
						}
					}
				}else if(pt.x <= 247){
					if(pt.y<=24){
						if(pt.x <= 238){
							region_id=41;
						}else{
							region_id=42;
						}
					}else if(pt.y<=35){
						if(pt.x <=241){
							region_id=47;
						}else{
							region_id=48;
						}
					}else if(pt.y<=45){
						region_id=53;
					}else if(pt.y<=60){
						if(pt.x<=240){
							region_id=58;
						}else{
							region_id=59;
						}
					}else if(pt.y<=74){
						region_id=64;
					}else if(pt.y<=99){
						region_id=70;
					}
				}else{
					if(pt.y<=24){
						region_id=42;
					}else if(pt.y<=36){
						region_id=48;
					}else if(pt.y<=45){
						if(pt.x<=249){
							region_id=53;
						}else{
							region_id=54;
						}
					}else if(pt.y<=58){
						if(pt.x<=254){
							region_id=59;
						}else{
							region_id=60;
						}
					}else if(pt.y<=72){
						if(pt.x<=250){
							region_id=54;
						}else if(pt.x<=262){
							region_id=65;
						}else{
							region_id=66;
						}
					}else if(pt.y<=99){
						if(pt.x<=256){
							region_id=70;
						}else if(pt.x<=270){
							region_id=71;
						}else{
							region_id=72;
						}
					}
				}

				switch(region_id){

					case(1):
					real_x = ((((74-pt.x)/16.0)*12.0)+60);
					real_y = ((((23-pt.y)/8.0)*12.0)+60);
					break;
					case(2):
					real_x = ((((89-pt.x)/16.0)*12.0)+48);
					real_y = ((((23-pt.y)/8.0)*12.0)+60);
					break;
					case(3):
					real_x = ((((103-pt.x)/16.0)*12.0)+36);
					real_y = ((((22-pt.y)/9.0)*12.0)+60);
					break;
					case(4):
					real_x = ((((118-pt.x)/16.0)*12.0)+24);
					real_y = ((((20-pt.y)/10.0)*12.0)+60);
					break;
					case(5):
					real_x = ((((137-pt.x)/18.0)*12.0)+12);
					real_y = ((((21-pt.y)/10.0)*12.0)+60);
					break;
					case(6):
					real_x = ((((157-pt.x)/20.0)*12.0)+0);
					real_y = ((((18-pt.y)/11.0)*12.0)+60);
					break;

					case(7):
					real_x = ((((71-pt.x)/9.0)*12.0)+60);
					real_y = ((((34-pt.y)/10.0)*12.0)+48);
					break;
					case(8):
					real_x = ((((85-pt.x)/19.0)*12.0)+48);
					real_y = ((((34-pt.y)/9.0)*12.0)+48);
					break;
					case(9):
					real_x = ((((101-pt.x)/19.0)*12.0)+36);
					real_y = ((((33-pt.y)/11.0)*12.0)+48);
					break;
					case(10):
					real_x = ((((117-pt.x)/19.0)*12.0)+24);
					real_y = ((((33-pt.y)/12.0)*12.0)+48);
					break;
					case(11):
					real_x = ((((135-pt.x)/20.0)*12.0)+12);
					real_y = ((((31-pt.y)/13.0)*12.0)+48);
					break;
					case(12):
					real_x = ((((157-pt.x)/22.0)*12.0)+0);
					real_y = ((((31-pt.y)/13.0)*12.0)+48);
					break;

					case(13):
					real_x = ((((65-pt.x)/19.0)*12.0)+60);
					real_y = ((((45-pt.y)/9.0)*12.0)+36);
					break;
					case(14):
					real_x = ((((80-pt.x)/20.0)*12.0)+48);
					real_y = ((((45-pt.y)/10.0)*12.0)+36);
					break;
					case(15):
					real_x = ((((96-pt.x)/21.0)*12.0)+36);
					real_y = ((((45-pt.y)/11.0)*12.0)+36);
					break;
					case(16):
					real_x = ((((113-pt.x)/20.0)*12.0)+24);
					real_y = ((((45-pt.y)/13.0)*12.0)+36);
					break;
					case(17):
					real_x = ((((134-pt.x)/23.0)*12.0)+12);
					real_y = ((((45-pt.y)/14.0)*12.0)+36);
					break;
					case(18):
					real_x = ((((157-pt.x)/25.0)*12.0)+0);
					real_y = ((((45-pt.y)/15.0)*12.0)+36);
					break;

					case(19):
					real_x = ((((59-pt.x)/20.0)*12.0)+60);
					real_y = ((((58-pt.y)/13.0)*12.0)+24);
					break;
					case(20):
					real_x = ((((74-pt.x)/22.0)*12.0)+48);
					real_y = ((((60-pt.y)/15.0)*12.0)+24);
					break;
					case(21):
					real_x = ((((92-pt.x)/24.0)*12.0)+36);
					real_y = ((((61-pt.y)/16.0)*12.0)+24);
					break;  
					case(22):
					real_x = ((((110-pt.x)/24.0)*12.0)+24);
					real_y = ((((62-pt.y)/17.0)*12.0)+24);
					break;  
					case(23):
					real_x = ((((132-pt.x)/26.0)*12.0)+12);
					real_y = ((((64-pt.y)/19.0)*12.0)+24);
					break;  
					case(24):
					real_x = ((((157-pt.x)/28.0)*12.0)+0);
					real_y = ((((66-pt.y)/21.0)*12.0)+24);
					break;

					case(25):
					real_x = ((((51-pt.x)/22.0)*12.0)+60);
					real_y = ((((58-pt.y)/17.0)*12.0)+12);
					break;  
					case(26):
					real_x = ((((68-pt.x)/25.0)*12.0)+48);
					real_y = ((((60-pt.y)/18.0)*12.0)+12);
					break;
					case(27):
					real_x = ((((85-pt.x)/25.0)*12.0)+36);
					real_y = ((((77-pt.y)/18.0)*12.0)+12);
					break;  
					case(28):
					real_x = ((((105-pt.x)/25.0)*12.0)+24);
					real_y = ((((80-pt.y)/20.0)*12.0)+12);
					break;  
					case(29):
					real_x = ((((129-pt.x)/29.0)*12.0)+12);
					real_y = ((((83-pt.y)/21.0)*12.0)+12);
					break;  
					case(30):
					real_x = ((((157-pt.x)/29.0)*12.0)+0);
					real_y = ((((86-pt.y)/23.0)*12.0)+12);
					break;          

					case(31):
					real_x = ((((42-pt.x)/24.0)*12.0)+60);
					real_y = ((((92-pt.y)/19.0)*12.0)+0);
					break;  
					case(32):
					real_x = ((((59-pt.x)/26.0)*12.0)+48);
					real_y = ((((96-pt.y)/21.0)*12.0)+0);
					break;
					case(33):
					real_x = ((((79-pt.x)/30.0)*12.0)+36);
					real_y = ((((100-pt.y)/24.0)*12.0)+0);
					break;  
					case(34):
					real_x = ((((105-pt.x)/29.0)*12.0)+24);
					real_y = ((((103-pt.y)/26.0)*12.0)+0);
					break;  
					case(35):
					real_x = ((((126-pt.x)/34.0)*12.0)+12);
					real_y = ((((110-pt.y)/30.0)*12.0)+0);
					break;  
					case(36):
					real_x = ((((157-pt.x)/34.0)*12.0)+0);
					real_y = ((((118-pt.y)/32.0)*12.0)+0);
					break;

		  /*RIGHT CAMERA*/

					case(37):
					real_x = ((((pt.x-157)/16.0)*12.0)+0);
					real_y = ((((19-pt.y)/11.0)*12.0)+60);
					break;  
					case(38):
					real_x = ((((pt.x-173)/18.0)*12.0)+12);
					real_y = ((((20-pt.y)/11.0)*12.0)+60);
					break;
					case(39):
					real_x = ((((pt.x-193)/16.0)*12.0)+24);
					real_y = ((((21-pt.y)/9.0)*12.0)+60);
					break;  
					case(40):
					real_x = ((((pt.x-209)/13.0)*12.0)+36);
					real_y = ((((22-pt.y)/9.0)*12.0)+60);
					break;  
					case(41):
					real_x = ((((pt.x-222)/13.0)*12.0)+48);
					real_y = ((((24-pt.y)/9.0)*12.0)+60);
					break;  
					case(42):
					real_x = ((((pt.x-237)/34.0)*12.0)+60);
					real_y = ((((24-pt.y)/8.0)*12.0)+60);
					break; 

					case(43):
					real_x = ((((pt.x-157)/18.0)*12.0)+0);
					real_y = ((((31-pt.y)/12.0)*12.0)+48);
					break;  
					case(44):
					real_x = ((((pt.x-175)/22.0)*12.0)+12);
					real_y = ((((32-pt.y)/12.0)*12.0)+48);
					break;
					case(45):
					real_x = ((((pt.x-197)/16.0)*12.0)+24);
					real_y = ((((33-pt.y)/12.0)*12.0)+48);
					break;  
					case(46):
					real_x = ((((pt.x-213)/14.0)*12.0)+36);
					real_y = ((((34-pt.y)/12.0)*12.0)+48);
					break;  
					case(47):
					real_x = ((((pt.x-227)/14.0)*12.0)+48);
					real_y = ((((34-pt.y)/10.0)*12.0)+48);
					break;  
					case(48):
					real_x = ((((pt.x-241)/14.0)*12.0)+60);
					real_y = ((((34-pt.y)/10.0)*12.0)+48);
					break; 


					case(49):
					real_x = ((((pt.x-157)/21.0)*12.0)+0);
					real_y = ((((45-pt.y)/14.0)*12.0)+36);
					break;  
					case(50):
					real_x = ((((pt.x-178)/22.0)*12.0)+12);
					real_y = ((((45-pt.y)/13.0)*12.0)+36);
					break;
					case(51):
					real_x = ((((pt.x-200)/18.0)*12.0)+24);
					real_y = ((((45-pt.y)/12.0)*12.0)+36);
					break;  
					case(52):
					real_x = ((((pt.x-218)/15.0)*12.0)+36);
					real_y = ((((45-pt.y)/11.0)*12.0)+36);
					break;  
					case(53):
					real_x = ((((pt.x-233)/15.0)*12.0)+48);
					real_y = ((((45-pt.y)/11.0)*12.0)+36);
					break;  
					case(54):
					real_x = ((((pt.x-248)/13.0)*12.0)+60);
					real_y = ((((45-pt.y)/10.0)*12.0)+36);
					break; 


					case(55):
					real_x = ((((pt.x-157)/25.0)*12.0)+0);
					real_y = ((((64-pt.y)/18.0)*12.0)+24);
					break;  
					case(56):
					real_x = ((((pt.x-182)/23.0)*12.0)+12);
					real_y = ((((62-pt.y)/17.0)*12.0)+24);
					break;
					case(57):
					real_x = ((((pt.x-205)/18.0)*12.0)+24);
					real_y = ((((60-pt.y)/15.0)*12.0)+24);
					break;  
					case(58):
					real_x = ((((pt.x-223)/18.0)*12.0)+36);
					real_y = ((((59-pt.y)/14.0)*12.0)+24);
					break;  
					case(59):
					real_x = ((((pt.x-241)/13.0)*12.0)+48);
					real_y = ((((58-pt.y)/13.0)*12.0)+24);
					break;  
					case(60):
					real_x = ((((pt.x-254)/15.0)*12.0)+60);
					real_y = ((((57-pt.y)/12.0)*12.0)+24);
					break; 


					case(61):
					real_x = ((((pt.x-157)/29.0)*12.0)+0);
					real_y = ((((83-pt.y)/19.0)*12.0)+12);
					break;  
					case(62):
					real_x = ((((pt.x-186)/24.0)*12.0)+12);
					real_y = ((((62-pt.y)/18.0)*12.0)+12);
					break;
					case(63):
					real_x = ((((pt.x-210)/20.0)*12.0)+24);
					real_y = ((((76-pt.y)/16.0)*12.0)+12);
					break;  
					case(64):
					real_x = ((((pt.x-230)/18.0)*12.0)+36);
					real_y = ((((73-pt.y)/14.0)*12.0)+12);
					break;  
					case(65):
					real_x = ((((pt.x-248)/14.0)*12.0)+48);
					real_y = ((((71-pt.y)/13.0)*12.0)+12);
					break;  
					case(66):
					real_x = ((((pt.x-262)/15.0)*12.0)+60);
					real_y = ((((69-pt.y)/12.0)*12.0)+12);
					break; 


					case(67):
					real_x = ((((pt.x-157)/34.0)*12.0)+0);
					real_y = ((((113-pt.y)/30.0)*12.0)+0);
					break;  
					case(68):
					real_x = ((((pt.x-191)/25.0)*12.0)+12);
					real_y = ((((107-pt.y)/27.0)*12.0)+0);
					break;
					case(69):
					real_x = ((((pt.x-216)/21.0)*12.0)+24);
					real_y = ((((101-pt.y)/25.0)*12.0)+0);
					break;  
					case(70):
					real_x = ((((pt.x-237)/19.0)*12.0)+36);
					real_y = ((((96-pt.y)/23.0)*12.0)+0);
					break;  
					case(71):
					real_x = ((((pt.x-256)/15.0)*12.0)+48);
					real_y = ((((93-pt.y)/22.0)*12.0)+0);
					break;  
					case(72):
					real_x = ((((pt.x-271)/15.0)*12.0)+60);
					real_y = ((((89-pt.y)/20.0)*12.0)+0);
					break; 

					default:
					cout<<"INVALID REGION"<<endl;     
					break;

				}


				r_val = sqrt(1.0*(real_x*real_x)+(1.0*(real_y+11)*(1.0*real_y+11)));
				if(pt.x<=157){
					angle =  180 - atan(1.0*(real_y+11)/(real_x))*180.0/3.14159;
				}else{
					angle =  90-(atan(1.0*(real_x)/(real_y+11))*180.0/3.14159) ;
				}
				lineArrayR[angle] = r_val*0.0254;

				if(i==0)
					endR = angle;






	  //cout<<"J1Point "<<pointCount<<": ["<<left_x<<","<<left_y<<"]"<<endl;
	  //cout<<"J1Point "<<pointCount<<": ["<<right_x<<","<<right_y<<"]"<<endl;
	  //pointCount+=2;
			}
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "laser_scan_publisher");
	cout<<"START"<<endl;

	ros::NodeHandle n;
	ros::NodeHandle nh_array;
	ros::Subscriber lidar_data = n.subscribe("lidar_scan", 1, lidar_cb);
	ros::Subscriber sub_arrayJ1 = nh_array.subscribe("PointsJ1", 1, arrayCallbackJ1);
	ros::Subscriber sub_arrayJ2 = nh_array.subscribe("PointsJ2", 1, arrayCallbackJ2);
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);


	unsigned int num_readings = 180;
	double laser_frequency = 40;
	double ranges[num_readings];
	double intensities[num_readings];

	int count = 0;
	ros::Rate r(1000);
	int scan_number =0;

	while(n.ok()){
		cout<<"in the loop"<<endl;
		cout<<"J1: "<<j1Recieved<<endl;
		cout<<"J2: "<<j2Recieved<<endl<<endl;
		if((j1Recieved==true)&&(j2Recieved==true)){
	//generate some fake data for our laser scan
			cout<<"Scan: "<<scan_number<<endl;
			ros::Time scan_time = ros::Time::now();

	//populate the LaserScan message
			sensor_msgs::LaserScan scan;
			scan.header.stamp = scan_time;
			scan.header.frame_id = "laser_frame";
			scan.angle_min = 0;
			scan.angle_max = 3.14;
			scan.angle_increment = 3.14 / num_readings;
			scan.time_increment = (1 / laser_frequency) / (num_readings);
			scan.range_min = 0.0;
			scan.range_max = 100.0;
			scan.ranges.resize(num_readings);
			scan.intensities.resize(num_readings);


			for(unsigned int i = 90; i < 180; ++i){

				if((lineArrayL[i]==0)&&(i<endL)){
					lineArrayL[i] = lineArrayL[i-1];
				}
				scan.ranges[i] = lineArrayL[i];
				scan.intensities[i] = 1;
				cout<<"Angle: "<<i<<" | Range: "<<scan.ranges[i]<<endl;
			}

			for(unsigned int i = 1; i < 90; ++i){

				if((lineArrayR[i]==0)&&(i<endR)){
					lineArrayR[i] = lineArrayR[i-1];
				}
				scan.ranges[i] = lineArrayR[i];
				scan.intensities[i] = 1;
				cout<<"Angle: "<<i<<" | Range: "<<scan.ranges[i]<<endl;
			}

			for(int i = 0; i<180; i++){
				if(scan.ranges[i] == 0){
					scan.ranges[i] = lidar[i];
				}else{
					if(lidar[i] < scan.ranges[i]){
						scan.ranges[i] = lidar[i];
					}
				}

			}
			cout<<endl<<endl;

			/*r(int i = 0; i <90; i++){
				scan.ranges[i+269] = scan.ranges[i];
				scan.ranges[i] = 0;
			}

			for(int i = 90; i <180; i++){
				scan.ranges[i-89] = scan.ranges[i];
				scan.ranges[i] = 0;
			}*/

			scan_pub.publish(scan);
			empty = true;
		}
   //waitKey(); 
		for(unsigned int i = 0; i < num_readings; ++i){
			lineArrayL[i] = 0;
			lineArrayR[i] = 0;
		}
		empty = true;


		cout<<"sleep"<<endl;
		ros::Rate loop_rate(1);
		ros::spinOnce();
		//loop_rate.sleep();
	}

	ros::shutdown;
}


