#include "Planner.h"
#include "Utils.hpp"
#include "spline.h"

const double K_LANE_WIDTH = 4.0; // m
const double K_DIST_INC = 0.45; // units
const int K_PATH_POINTS = 50;
const double K_SLOW_REF_VEL = 29.5;

Planner::Planner(vector<double> & map_wp_x,
        vector<double> & map_wp_y,
        vector<double> & map_wp_s,
        vector<double> & map_wp_dx,
        vector<double> & map_wp_dy)
{
  map_waypoints_x = map_wp_x;
  map_waypoints_y = map_wp_y;
  map_waypoints_s = map_wp_s;
  map_waypoints_dx = map_wp_dx;
  map_waypoints_dy = map_wp_dy;

  // starting lane is 1
  mLane = 1;
}

void Planner::update(double car_x,
            double car_y,
            double car_s,
            double car_d,
            double car_yaw,
            double car_speed,
            nlohmann::json &prev_x,
            nlohmann::json &prev_y,
            double end_s,
            double end_d,
            nlohmann::json &sensor_fusion)
{
  // prepare spline

  const int prev_size = prev_x.size();

  // take sensor fusion data into account
  if (prev_size > 0)
  {
    car_s = end_s;
  }

  bool too_close = false;

  for (int i = 0; i < sensor_fusion.size(); i++)
  {
    // lane of other car
    float d = sensor_fusion[i][6];
    if ((d < 4+4*mLane) && (d > 4 * mLane))
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];
      check_car_s += ((double) prev_size*.02*check_speed);

      // are we close to the other car?
      if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
      {
        ref_vel = K_SLOW_REF_VEL;
      }
    }
  }

  // widely-spread spline points
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);


  // use car as starting reference if previous size is empty
  if (prev_size < 2)
   {
     // two points tangent to the car
     double prev_car_x = car_x - cos(car_yaw);
     double prev_car_y = car_y - sin(car_yaw);

     ptsx.push_back(prev_car_x);
     ptsx.push_back(car_x);

     ptsy.push_back(prev_car_y);
     ptsy.push_back(car_y);
   }
   else
   {
     ref_x = prev_x[prev_size-1];
     ref_y = prev_y[prev_size-1];

     double ref_x_prev = prev_x[prev_size-2];
     double ref_y_prev = prev_y[prev_size-2];
     ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

     // make path tangent
     ptsx.push_back(ref_x_prev);
     ptsx.push_back(ref_x);

     ptsy.push_back(ref_y_prev);
     ptsy.push_back(ref_y);
   }

   // add evenly spaced points in Frenet
   vector<double> next_wp0 = getXY(car_s+30,
                                   (2+4*mLane),
                                   map_waypoints_s,
                                   map_waypoints_x,
                                   map_waypoints_y);
  vector<double> next_wp1 = getXY(  car_s+60,
                                    (2+4*mLane),
                                    map_waypoints_s,
                                    map_waypoints_x,
                                    map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90,
                                   (2+4*mLane),
                                    map_waypoints_s,
                                    map_waypoints_x,
                                    map_waypoints_y);
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);



 // shift points to local reference frame
  for (int i = 0; i < ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  // create spline

  //cout << "x size " << ptsx.size() << " y size " << ptsy.size() << endl;

  tk::spline s;

  // set the spaced-out points to define the spline
  s.set_points(ptsx, ptsy);


  // path planning points
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // start with previous path points
  for (int i = 0; i < prev_x.size(); i++)
  {
    next_x_vals.push_back(prev_x[i]);
    next_y_vals.push_back(prev_y[i]);
  }

  // break up spline points to travel at desired target velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x+target_y*target_y);

  double x_add_on = 0;

  // fill up rest of path planner with points from spline

  for(int i= 1; i <= 50-prev_x.size(); i++)
  {
    // car visits each points every .02 s
    double N = target_dist / (.02 * ref_vel/2.24);
    double x_point = x_add_on+(target_x) / N;
    double y_point = s(x_point);
    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to world coordinates
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }



  // for (int i = 0; i < K_PATH_POINTS; i++)
  // {
  //   double next_s = car_s + (i+1) * K_DIST_INC;
  //   double next_d = 6;
  //   vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  //   next_x_vals.push_back(xy[0]);
  //   next_y_vals.push_back(xy[1]);
  //   //next_x_vals.push_back(car_x + (K_DIST_INC*i) * cos(deg2rad(car_yaw)));
  //   //next_y_vals.push_back(car_y + (K_DIST_INC *i) * sin(deg2rad(car_yaw)));
  // }

  // save the generated path
  mPath.clear();
  mPath.push_back(next_x_vals);
  mPath.push_back(next_y_vals);
}

/**
* gets current path. Return by reference to minimize memcopying
*/
vector<vector<double>>& Planner::getPath()
{
  return mPath;
}
