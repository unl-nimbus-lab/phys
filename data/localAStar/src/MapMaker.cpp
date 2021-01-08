#include "MapMaker.h"
#include "MakeMarker.h"

#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>



//2 - nieznane
//1 - zajęte
//0 - wolne
//ulepszenie zgodne z moim pomysłem
//200 - nieznane
//201 - zajęte
//0 - 199 - wolne
void FillOccupancyMap(myMap *&map, sensor_msgs::PointCloud* cloud, float bucketSize, float maxDist){
    int a(maxDist/bucketSize);
    if(map!=NULL)
        delete map;
    map = new myMap(a, bucketSize);
    //std::cout<<"wymiary mapy to: "<<a+1<<" X "<<2*a+1<<std::endl;
    ///fill the grid with '2'
    for (int i = 0; i < map->sizeX; ++i) {
        for (int j = 0; j < map->sizeY; ++j) {
            map->SetMap(i, j, 200);
        }
    }
    for(int  i = 0;i<cloud->points.size(); i++){
        map->SetMap(cloud->points[i].x, cloud->points[i].y, 201);
    }
    bool continue_step = true;

    //map->SetMap((float)0.0,(float)0.0,1);
    //map->SetMap(0, a, 2);
    std::vector<Position<int> > possible_steps, nextSteps;
    possible_steps.push_back(Position<int>(0, a-1));
    possible_steps.push_back(Position<int>(1, a));
    possible_steps.push_back(Position<int>(0, a+1));
    int step = 0, maxStep = a-1;

    while(continue_step){
        step++;
        int nextMaxSize = 2*(step+1)+1;
        nextSteps.reserve(nextMaxSize);

        for(int i = 0;i<possible_steps.size();i++){
            if(map->GetMap(possible_steps[i])!=201){
                map->SetMap(possible_steps[i], step);
            }
        }
       //  std::cout<<step<<" to był step, teraz nextMaxSize "<<nextMaxSize<<std::endl;
        for(int i = 0; i < nextMaxSize; i++){
            Position<int> pos_next(step+1 - std::abs(-step-1+i), a-step-1+i);
            //dwóch sąsiadów o poprzednim kroku lub 1 sąsiad, ale którakolwiek współrzędna równa 0 - w tej chwili uwzględniane w funkcji
            //std::cout<<step+1 - std::abs(-step-1+i)<<" sadfas "<<a-step-1+i<<std::endl;
            int bonus = 0;
            if(pos_next.y == a){
                bonus++;
            }
            if((numberNeig(map, pos_next, step)>1-bonus)){ //|| ((pos_next.x == 0||pos_next.y == 0)&&numberNeig(map, pos_next, step)>0)){
                nextSteps.push_back(pos_next);
            }
        }
        possible_steps.swap(nextSteps);
        if(nextSteps.size()==0||step==maxStep){
            continue_step = false;
        }
    }
}
int numberNeig(myMap *map, Position<int> here, int step){
    int num = 0;
    if(here.x<map->sizeX-1){
        if(map->GetMap(here.x+1, here.y)==step){
            num++;
        }
    }else{
        num++;
    }

    if(here.x>1){
        if(map->GetMap(here.x-1, here.y)==step){
            num++;
        }
    }else{
        num++;
    }
    if(here.y>1){
        if(map->GetMap(here.x, here.y-1)==step){
            num++;
        }
    }else{
     num++;
    }
    if(here.y<map->sizeY-1){
        if(map->GetMap(here.x, here.y+1)==step){
            num++;
        }
    }else{
        num++;
    }
    return num;
}

void FillOccupancyMap2(myMap *&map, sensor_msgs::PointCloud* cloud, float bucketSize, float maxDist){
    int a(maxDist/bucketSize);
    if(map!=NULL)
        delete map;
    map = new myMap(a, bucketSize);
    //std::cout<<"wymiary mapy to: "<<a+1<<" X "<<2*a+1<<std::endl;
    ///fill the grid with '2'
    for (int i = 0; i < map->sizeX; ++i) {
        for (int j = 0; j < map->sizeY; ++j) {
            map->SetMap(i, j, 200);//test1
        }
    }
    for(int  i = 0;i<cloud->points.size(); i++){
        map->SetMap(cloud->points[i].x, cloud->points[i].y, 201);//test0
    }
    bool continue_step = true;
    int tilt = 0, tiltMax = map->sizeX*2+map->sizeY;

    while(continue_step){
        if(tilt<map->sizeX){
            for(int i = 0;i<a;i++){
                int x = (int)::round((float)tilt*(float)((float)i/(float)a));
                int y = a+i;
                //std::cout<<"tilt1 "<<tilt<<" w "<<x<<", "<<y<<std::endl;
                if(map->GetMap(x, y)!=201){
                    map->SetMap(x, y, 2);//test2
                }else{
                    break;
                }
            }
        }else{
            if(tilt<map->sizeX+map->sizeY){
                for(int i = 0;i<a;i++){
                    int x = i;
                    int y = a+((int)::round((float)(tilt-map->sizeY)*(float)((float)i/(float)a)));//to zaczyna od pionu i leci w lewo a+((int)::round((float)(tilt-map->sizeX)*(float)((float)i/(float)a)));
                    //std::cout<<"tilt2 "<<tilt<<" w "<<x<<", "<<y<<std::endl;
                    if(map->GetMap(x, y)!=201){
                        map->SetMap(x, y, 2);//test3
                    }else{
                        break;
                    }
                }
            }else{
                for(int i = 0;i<a;i++){
                    int x = (int)::round((float)(tilt-map->sizeX-map->sizeY)*(float)((float)i/(float)a));
                    int y = a-i;
                    //std::cout<<"tilt3 "<<tilt<<" w "<<x<<", "<<y<<std::endl;
                    if(map->GetMap(x, y)!=201){
                        map->SetMap(x, y, 2);//test4
                    }else{
                        break;
                    }
                }
            }
        }
        tilt++;
        if(tilt==tiltMax){
            continue_step = false;
        }
    }
}
void drawMap(myMap * map, ros::Publisher pub){

    if(map!=NULL)
    {
        for(int i = 0; i<map->Length; i++){
            Position<float> p = map->getPos(i);
            switch(map->GetMap(i)){
            case 200:
                makeMarker(pub, i, p.x, p.y, (float)1, (float)1, (float)1, (float)0.0);
                break;
            case 201:
                makeMarker(pub, i, p.x, p.y, (float)0, (float)0, (float)0, visualization_msgs::Marker::CUBE);
                break;
            default:
                makeMarker(pub, i, p.x, p.y, (float)0.0, (float)0.5, (float)0.0, (float)0.5);
                break;
            }
            if(i==map->GetXSize()*(map->GetXSize()-1)){
                makeMarker(pub, i, p.x, p.y, (float)1.0, (float)1.0, (float)1.0);
            }
            //wzór na krawędzie
            /*if(i%map->sizeX<1||i%map->sizeX>map->sizeX-2||i<map->sizeX||i>(map->sizeX)*(map->sizeY-1)){
                makeMarker(pub, i, p.x, p.y, (float)0.0, (float)0.0, (float)1.0, (float)1.0);
            }*/
        }

    }
}
myMap* mask(myMap *map, int size){
    myMap *newOne = new myMap(*map);
    for(int i = 0;i<newOne->Length;i++){

        if(map->GetMap(i)==201){
            newOne->SetMap(i, map->GetMap(i));
        }else{
            bool allFree = true;
            bool wall = false;
            for(int j = -size; j<size+1; j++){
                for(int k = -size; k<size+1; k++){
                    int l = i+k+map->sizeX*j;

                    if((l>=0&&l<map->Length)&&i%map->sizeX>2)
                    {
                        if(map->GetMap(l)>=200){
                            allFree = false;
                        }
                        if(map->GetMap(l)>200){
                            wall=true;
                            break;
                            break;
                        }
                    }
                }
            }
            if(allFree){
                newOne->SetMap(i, 2);
            }else{
                if(wall){
                    newOne->SetMap(i, 201);
                }
                else{
                    newOne->SetMap(i, 200);
                }
            }
        }
    }
    return newOne;
}
