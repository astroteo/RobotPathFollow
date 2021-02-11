#include <stdio.h>
#include <iostream>
#include <string> // string library
#include <vector>
#include <utility>
#include <cmath>

using namespace::std;

/* The pastebin path is imported here as raw-sring*/
string raw_path =R"({0.00359, -0.0013},   {0.00608, -0.00281},  {0.00756, -0.0027},
      {0.00842, -0.00307},  {0.00849, -0.0037},   {0.00846, -0.00387},
      {0.00829, -0.00379},  {0.0084, -0.00388},   {0.00846, -0.00409},
      {0.0138, -0.00347},   {0.0698, -0.00098},   {0.11151, -0.00745},
      {0.167, -0.01404},    {0.32572, -0.05356},  {0.41797, -0.07953},
      {0.52867, -0.11505},  {0.61002, -0.13945},  {0.63633, -0.14954},
      {0.70933, -0.18835},  {0.7191, -0.19822},   {0.72701, -0.20117},
      {0.731, -0.20424},    {0.73371, -0.20805},  {0.77746, -0.2621},
      {0.86029, -0.34734},  {0.88373, -0.37565},  {0.90413, -0.40655},
      {0.92189, -0.43795},  {0.93867, -0.47125},  {0.95337, -0.50479},
      {0.96615, -0.54003},  {0.97729, -0.57518},  {0.98669, -0.60948},
      {0.9944, -0.64442},   {0.99963, -0.67999},  {1.00244, -0.71709},
      {1.00327, -0.75302},  {0.99907, -0.78939},  {0.99464, -0.8237},
      {0.98722, -0.86223},  {0.97558, -0.90511},  {0.96147, -0.94947},
      {0.94402, -0.99336},  {0.92286, -1.03964},  {0.89779, -1.08594},
      {0.8698, -1.13096},   {0.73009, -1.33175},  {0.59053, -1.5304},
      {0.46166, -1.7128},   {0.30239, -1.93285},  {0.25147, -1.99011},
      {0.19826, -2.04254},  {0.14275, -2.09163},  {0.08663, -2.13425},
      {0.03116, -2.17209},  {-0.0238, -2.20402},  {-0.07864, -2.23286},
      {-0.1318, -2.25636},  {-0.1825, -2.27552},  {-0.23171, -2.29113},
      {-0.27795, -2.30206}, {-0.32673, -2.31054}, {-0.37225, -2.31536},
      {-0.41574, -2.31996}, {-0.45496, -2.32042}, {-0.48902, -2.31757},
      {-0.52496, -2.3164},  {-0.55811, -2.31102}, {-0.77049, -2.25292},
      {-0.99, -2.19669},    {-1.19266, -2.14085}, {-1.23428, -2.12438},
      {-1.27377, -2.10614}, {-1.31327, -2.08351}, {-1.39679, -2.03016},
      {-1.48345, -1.95929}, {-1.52353, -1.91628}, {-1.66757, -1.77012},
      {-1.83468, -1.60606}, {-2.01648, -1.41688}, {-2.18845, -1.20596},
      {-2.35403, -0.99207}, {-2.44666, -0.84068}, {-2.48383, -0.76261},
      {-2.51504, -0.68854}, {-2.53995, -0.61543}, {-2.56026, -0.54313},
      {-2.57583, -0.47095}, {-2.58632, -0.40214}, {-2.5929, -0.33388},
      {-2.59584, -0.2669},  {-2.5965, -0.20323},  {-2.59088, -0.13817},
      {-2.58415, -0.07689}, {-2.57404, -0.0163},  {-2.55813, 0.04199},
      {-2.5374, 0.10109},   {-2.51245, 0.15825},  {-2.48738, 0.21222},
      {-2.45803, 0.26488},  {-2.42471, 0.314},    {-2.38647, 0.36297},
      {-2.3471, 0.40819},   {-2.30357, 0.45124},  {-2.1598, 0.59651},
      {-1.99623, 0.75884},  {-1.84116, 0.91525},  {-1.68546, 1.07255},
      {-1.57778, 1.17373})";

struct Point
{
 Point(){}
 Point(double x_, double y_)
 {
   x = x_;
   y = y_;
 }
public:
  double x;
  double y;
};

pair<double,double> UP(0,1);

class Robot{

  public:
    /*
      Two initialization assumptions:
      1. By default robot starts from the origin [x_i = 0, y_i = 0]
        => if the the robot initila conditions are modified the robot traks the
           first point of the path.
           Another option could be to reach the closest point of the path as
           starting point [NOT APPLIED HERE]
      2. Path can be provided only a string formatted accordingly to the one
         provided in the example.
    */

    ///constructor
    Robot(string raw_path,double step_grid_=0.001, double x_i=0, double y_i=0 )
    {
      x = x_i;
      y = y_i;
      step_grid = step_grid_;
      loadPath(raw_path);
    }
    /// Function loading the path into a vector<WayPoint>
    void loadPath(string raw_path)
    {
      while (raw_path.length())
      {
        unsigned first = raw_path.find("{");
        unsigned last = raw_path.find("}");
        string sub = raw_path.substr(first+1,last-(first+1));
        raw_path = raw_path.substr(last+1,raw_path.length());

        unsigned del_p = sub.find(",");
        Point wp;// WayPoint = path's point to be followed/tracked.
        wp.x = stod(sub.substr(0,del_p));
        wp.y = stod(sub.substr(del_p+1, sub.length()));
        path.push_back(wp);
      }

      cout << "...path loaded..." << endl;
    }

    /// method leading  the robot to the next waypoint
    void nextPoint(Point wp, int wp_idx)
    {
      while(abs(wp.x - x) > step_grid || abs(wp.y - y ) > step_grid)
      {
        /*
          It is assued here that the robot can only move up/down or left/right.
          the squared footprint cannot be applied on a diagonal.
        */

        /*Move right/left
          => lateral motion evaluated first*/
        if ( wp.x >= x + step_grid)
        {
           x += step_grid;
           area += pow(step_grid,2);
        }
        else if (wp.x <= x-step_grid)
        {
          x -= step_grid;
          area += pow(step_grid,2);
        }

        /* Move up/down */
        if ( wp.y >= y + step_grid)
        {
           y += step_grid;
           area += pow(step_grid,2);
        }
        else if (wp.y <= y -step_grid)
        {
          y -= step_grid;
          area += pow(step_grid,2);
        }

         Point tp(x,y);// trajectory point
         trajectory.push_back(tp);
      }

      cout << "reached waypoint "  << wp_idx <<"/" << path.size()
           <<" at position {" <<wp.x << "," <<wp.y << "}" << endl;

    }

    /// method commandig the robot to follow the path
    void followPath()
    {
      int cnt =0;// index counting for waypoints
      for(auto wp : path)
      {
          nextPoint(wp, cnt+1);
          cnt++;
      }

    }

    /// method printing the results: area, final position, etc..
     void getResults()
    {
      cout << "total area covered by the robot: "<< area <<endl;
      cout << "final position => {"<< trajectory.back().x<<" , "<<trajectory.back().y<<"}"<<endl;
      cout << "last way-point in path => {"<< path.back().x << " , " << path.back().y << "}"<<endl;
    }


  private:
    /*
      Attributes/Members:
      step_grid |-> the assumed  side of the "square-footprint" of the robot
      (it is assumed that the side of the "square-footprint" of the robot is
      minor than the distance between two consecutive path's points)

      x,y, |-> the current Robot's position

      path | -> the path (WayPoints) followed by the Robot

      trajectory |-> the actual robot's trajectory.

      area | -> the area is the sum of all the square footprints of the robots
                along its trajectory
       */
    double step_grid, x, y;
    vector<Point> path,trajectory;
    double area = 0;


};

int main(int argc, const char * argv[])
{
    Robot rbt(raw_path);
    rbt.followPath();
    rbt.getResults();
}
