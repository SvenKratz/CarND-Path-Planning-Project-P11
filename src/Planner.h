#ifndef Planner_h
#define Planner_h

#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"



using namespace std;


class Planner
{
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  int currentWayPoint;

  // stores the path generated from the last update
  vector<vector<double>> mPath;
  int mLane;
  // refernece velocity
  double ref_vel;

public:

  Planner(vector<double> & map_wp_x,
          vector<double> & map_wp_y,
          vector<double> & map_wp_s,
          vector<double> & map_wp_dx,
          vector<double> & map_wp_dy);

  void update(double car_x,
              double car_y,
              double car_s,
              double car_d,
              double car_yaw,
              double car_speed,
              nlohmann::json &prev_x,
              nlohmann::json &prev_y,
              double end_s,
              double end_d,
              nlohmann::json &fusion);

  vector<vector<double>>& getPath();
};

#endif
