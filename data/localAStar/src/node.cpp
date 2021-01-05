#include "node.h"
#include "MakeMarker.h"
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <cmath>
#include <ctime>

using namespace std;

//directions
static int dx[dir] = {1, 1, 0, -1, -1, -1, 0, 1};
int        dy[dir] = {0, 1, 1, 1, 0, -1, -1, -1};


string pathFind(int xStart, int yStart, int xFinish, int yFinish, myMap* map)
{
    priority_queue<Node> pq[2]; // list of open (not-yet-tried) Nodes
    int pqi; // pq index
    Node* n0;
    Node* m0;
    int i = 0, j = 0, x = 0, y = 0, xdx = 0, ydy = 0;
    char c;

    int xSize = map->GetXSize();
    int ySize = map->GetYSize();
    int closed_nodes_map[xSize][ySize];
    int open_nodes_map[xSize][ySize];
    int dir_map[xSize][ySize];
    int dir = 8;

    pqi=0;

    // reset the Node maps
    for (y = 0; y < ySize; y++)
    {
        for (x = 0; x < xSize; x++)
        {
            closed_nodes_map[x][y] = 0;
            open_nodes_map[x][y] = 0;
        }
    }

    // create the start Node and push into list of open Nodes
    n0 = new Node(xStart, yStart, 0, 0);
    n0->UpdatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);

    open_nodes_map[x][y] = n0->GetPriority(); // mark it on the open Nodes map

    // A* search
    while (!pq[pqi].empty())
    {
        // Get the current Node w/ the highest priority
        // from the list of open Nodes
        n0 = new Node( pq[pqi].top().GetxPos(), pq[pqi].top().GetyPos(),
                     pq[pqi].top().GetLevel(), pq[pqi].top().GetPriority());

        x = n0->GetxPos(); y=n0->GetyPos();

        pq[pqi].pop(); // remove the Node from the open list
        open_nodes_map[x][y] = 0;
        // mark it on the closed Nodes map
        closed_nodes_map[x][y] = 1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if (x == xFinish && y == yFinish)
        {
           /// printf("Path found\n");

            // generate the path from finish to start
            // by following the directions
            string path = "";
            int lastUnknownOccurence = -1;
            int iterator = 0;

            while (!(x == xStart && y == yStart))
            {
                j = dir_map[x][y];
                c = '0'+ (j + dir / 2) % dir;

                path = c + path;

                if (map->GetMap(x, y) == 200) {
                    lastUnknownOccurence = iterator;
                }

                ++iterator;
                x += dx[j];
                y += dy[j];
            }

            for (int i = 0; i < lastUnknownOccurence; ++i) {
                path.pop_back();
            }

            printf("To be deleted: %d\n", lastUnknownOccurence);

            // garbage collection
            delete n0;
            // empty the leftover Nodes
            while(!pq[pqi].empty()) pq[pqi].pop();

            std::cout << "PATH FOUND " << path << '\n' << std::endl;
            return path;
        }

        // generate moves (child Nodes) in all possible directions
        for (i = 0; i < dir; i++)
        {
            xdx = x + dx[i]; ydy = y + dy[i];

            if (!(xdx < 0 || xdx > xSize - 1 || ydy < 0 || ydy > ySize - 1 || map->GetMap(xdx, ydy) > 200
                || closed_nodes_map[xdx][ydy] == 1))
            {
                // generate a child Node
                m0 = new Node( xdx, ydy, n0->GetLevel(), n0->GetPriority());
                m0->NextLevel(i);
                m0->UpdatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if (open_nodes_map[xdx][ydy] == 0)
                {
                    open_nodes_map[xdx][ydy] = m0->GetPriority();
                    pq[pqi].push(*m0);
                    // mark its parent Node direction
                    dir_map[xdx][ydy] = (i + dir / 2) % dir;
                }
                else if (open_nodes_map[xdx][ydy] > m0->GetPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy] = m0->GetPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy] = (i + dir / 2) % dir;

                    // replace the Node
                    // by emptying one pq to the other one
                    // except the Node to be replaced will be ignored
                    // and the new Node will be pushed in instead
                    while (!(pq[pqi].top().GetxPos() == xdx &&
                           pq[pqi].top().GetyPos() == ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted Node

                    // empty the larger size pq to the smaller one
                    if (pq[pqi].size() > pq[1-pqi].size()) pqi = 1-pqi;
                    while (!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi= 1 - pqi;
                    pq[pqi].push(*m0); // add the better Node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    printf("No route found! \n");
    return ""; // no route found
}

// Determine priority (in the priority queue)
bool operator<(const Node &a, const Node &b)
{
    return a.GetPriority() > b.GetPriority();
}

void VisualizePath(std::string route, myMap *map, ros::Publisher pub){
    // follow the route on the map and display it
    if(route.length()>0)
    {
        int j; char c;

        ///start point
        int sizeX = map->GetXSize();
        int x = 0;
        int y = sizeX + 1;

        for (int i = 0; i < route.length(); i++)
        {
            c = route.at(i);
            j = atoi(&c);
            x = x + dx[j];
            y = y + dy[j];

            Position<float> p = map->getPos(sizeX * y + x);
            makeMarker(pub, i + 10000, p.x, p.y, (float)0, (float)1, (float)1, (float)1);
        }
    }
}
//my magic:D
std::vector <geometry_msgs::Point> string2Vectors(std::string path, myMap *map){
    std::vector <geometry_msgs::Point> simple_path;
    int j; char c;
    int sizeX = map->GetXSize();
    int x = 0;
    int y = sizeX - 1;
    geometry_msgs::Point nextPos;
    Position<float> p = map->getPos(sizeX * y + x);
    nextPos.x = p.x;
    nextPos.y = p.y;
    simple_path.push_back(nextPos);
    for (int i = 0; i < path.length(); i++)
    {
        c = path.at(i);
        j = atoi(&c);
        x = x + dx[j];
        y = y + dy[j];

        p = map->getPos(sizeX * y + x);
        nextPos.x = p.x;
        nextPos.y = p.y;
        simple_path.push_back(nextPos);
    }
    return simple_path;
}

//chcę skrócić drogę... w tym celu zakładam sobie vector(bo lubię vectory) punktów, który skrócę do przejezdnego minimum
//zakładamy, że punkt 0, to punkt w którym jest robot
bool IsRunnable(geometry_msgs::Point point1, geometry_msgs::Point point2, sensor_msgs::PointCloud* obstacles, float robotWidth);
std::vector <geometry_msgs::Point> PathShortener(std::vector <geometry_msgs::Point> path, sensor_msgs::PointCloud* obstacles, float robotWidth){
    std::vector <geometry_msgs::Point> simple_path;
    if(path.size()>0)
    {
        simple_path.push_back(path[0]);
        for(int i = 0; i < path.size()-2; i++)
        {
            for(int j = i+2; j<path.size();j++){
                //std::cout<<"i: "<<i<<" j: "<<j<<std::endl;
                if(IsRunnable(path[i], path[j], obstacles, robotWidth)){

                }else{
                    simple_path.push_back(path[j-1]);
                    i=j-1;
                    break;
                }
            }
        }
        simple_path.push_back(path[path.size()-1]);
    }
    std::cout<<"Shortened from "<<path.size()<<" to "<<simple_path.size()<<" nodes"<<std::endl;
    return simple_path;
}
geometry_msgs::Point operator-(geometry_msgs::Point p1, geometry_msgs::Point p2){
    geometry_msgs::Point substract;
    substract.x = p1.x-p2.x;
    substract.y = p1.y-p2.y;
    substract.z = p1.z-p2.z;
    return substract;
}
geometry_msgs::Point operator+(geometry_msgs::Point p1, geometry_msgs::Point p2){
    geometry_msgs::Point sum;
    sum.x = p1.x+p2.x;
    sum.y = p1.y+p2.y;
    sum.z = p1.z+p2.z;
    return sum;
}
geometry_msgs::Point operator/(geometry_msgs::Point p1, double x){
    geometry_msgs::Point divided;
    divided.x = p1.x/x;
    divided.y = p1.y/x;
    divided.z = p1.z/x;
    return divided;
}
double dotProductWithoutZ(geometry_msgs::Point A, geometry_msgs::Point B);
bool IsRunnable(geometry_msgs::Point point1, geometry_msgs::Point point2, sensor_msgs::PointCloud* obstacles, float robotWidth){
    //todo niechaj działa
    //szukanie punktu środkowego
    //punkty i wektory: substraction - wektor między P1 i P2, median - punkt po środku między P1 i P2, A,B,C - punkty prostokąta

    geometry_msgs::Point substraction=point1-point2, median=(point1+point2)/2, A, B, C, AB, BC, AP, BP;
    float angle = atan2(substraction.y, substraction.x), maxCircle = (M_SQRT2*robotWidth+std::sqrt(substraction.x*substraction.x+substraction.y*substraction.y))/2;
    maxCircle*=maxCircle;
    std::vector <geometry_msgs::Point> suspected;

    for(int i = 0; i< obstacles->points.size(); i++){
        if(obstacles->points[i].x>0)
        {
            //czy punkt mieści się w wielkim kole, jak tak, to na listę podejrzanych
            if(std::pow(obstacles->points[i].x - median.x, 2)+ std::pow(obstacles->points[i].y - median.y, 2)<(double)maxCircle){
                geometry_msgs::Point point;
                point.x = obstacles->points[i].x;
                point.y = obstacles->points[i].y;
                point.z = 0;
                suspected.push_back(point);
            }
        }
    }
    if(suspected.size()==0){
        return true;
    }
    //obliczamy resztę prostokąta
    A.x = point1.x+std::cos(angle+M_PI_2+M_PI_4); A.y = point1.y+std::sin(angle+M_PI_2+M_PI_4); A.z = 0;
    B.x = point1.x+std::cos(angle-M_PI_2-M_PI_4); B.y = point1.y+std::sin(angle-M_PI_2-M_PI_4); B.z = 0;
    C.x = point2.x+std::cos(angle-M_PI_4); C.y = point2.y+std::sin(angle-M_PI_4); C.z = 0;
    AB = B-A;
    BC = C-B;
    for(int i = 0; i<suspected.size(); i++){
        AP = suspected[i]-A;
        BP = suspected[i]-B;
        double dotAP = dotProductWithoutZ(AB, AP), dotBP = dotProductWithoutZ(BC, BP);
        //test iloczynów skalarnych
        if(0<dotAP && dotAP<dotProductWithoutZ(AB, AB) && 0<dotBP && dotBP<dotProductWithoutZ(BC, BC)){
            return false;
        }
    }
    return true;
}
double dotProductWithoutZ(geometry_msgs::Point A, geometry_msgs::Point B){
    return (A.x*B.x+A.y*B.y);
}
