#include "../include/path_planner.h"
#include <math.h>
#include "../include/helpers.h"
#include "../include/spline.h"
#include <iostream>

using namespace path_planner;

PathPlanner::PathPlanner() : current_state{State::FollowLane} {};

void PathPlanner::Step()
{
    CalculateNewState();
    CalculatePathPoints();
}

PathPoints PathPlanner::GetPathPoints() const
{
    return current_path;
}

void PathPlanner::CalculateNewState()
{
    const int critical_distance = 30;

    switch (current_state)
    {
    case State::FollowLane:
    {
        auto closest_vehicle = GetClosestCarInCurrentLane();
        if (closest_vehicle != nullptr)
        {
            auto distance_to_next_vehicle = closest_vehicle->s - car_dynamics.s;
            std::cerr << "Distance to next vehicle: " << distance_to_next_vehicle << std::endl;
            if (distance_to_next_vehicle < critical_distance && closest_vehicle->V() < car_dynamics.speed)
            {
                current_state = State::FollowLeadingVehicle;
            }
        }
        break;
    }
    case State::FollowLeadingVehicle:
    {
        if (IsChangeToLanePossible(GetLeftLane())) //Try to overtake from the left
        {
            current_state = State::ChangeLaneToLeft;
            current_target_lane = GetLeftLane();
            break;
        }

        if (IsChangeToLanePossible(GetRightLane())) //Try to overtake from the right
        {
            current_state = State::ChangeLaneToRight;
            current_target_lane = GetRightLane();
            break;
        }

        auto closest_vehicle = GetClosestCarInCurrentLane();
        if (closest_vehicle == nullptr)
        {
            current_state = State::FollowLane;
        }
        else
        {
            auto distance_to_next_vehicle = closest_vehicle->s - car_dynamics.s;
            std::cerr << "Distance to next vehicle: " << distance_to_next_vehicle << std::endl;
            if (distance_to_next_vehicle > critical_distance)
            {
                std::cerr << "Follow Vehicle Cancelled due to distance" << std::endl;
                current_state = State::FollowLane;
            }
            if (closest_vehicle->V() > car_dynamics.speed + 2)
            {
                std::cerr << "Follow Vehicle Cancelled due to speed" << std::endl;
                current_state = State::FollowLane;
            }
        }
        break;
    }
    case State::ChangeLaneToLeft:
        if (current_target_lane == GetCurrentLane())
        {
            std::cerr << "Lane change to left complete" << std::endl;
            current_state = State::FollowLane;
        }
        break;
    case State::ChangeLaneToRight:
        if (current_target_lane == GetCurrentLane())
        {
            std::cerr << "Lane change to right complete" << std::endl;
            current_state = State::FollowLane;
        }
        break;
    default:
        break;
    }
}

void PathPlanner::CalculatePathPoints()
{
    switch (current_state)
    {
    case State::FollowLane:
        std::cerr << "CurrentState: FollowLane" << std::endl;
        CalculateFollowLanePathPoints();
        break;
    case State::FollowLeadingVehicle:
        std::cerr << "CurrentState: FollowLeadingVehicle" << std::endl;
        CalculateFollowLeadingVehiclePathPoints();
        break;
    case State::ChangeLaneToLeft:
        std::cerr << "CurrentState: ChangeLaneToLeft" << std::endl;
        CalculateChangeLaneToLeftPathPoints();
        break;
    case State::ChangeLaneToRight:
        std::cerr << "CurrentState: ChangeLaneToRight" << std::endl;
        CalculateChangeLaneToLeftRightPoints();
        break;
    }
}

void PathPlanner::CalculateFollowLanePathPoints()
{
    const double kMaxTargetVelocity{49.5 - kMaxAcceleration};

    if (target_ref_velocity < kMaxTargetVelocity)
    {
        target_ref_velocity += kMaxAcceleration;
    }

    GenerateTrajectory(target_ref_velocity);
}

void PathPlanner::CalculateFollowLeadingVehiclePathPoints()
{
    double target_velocity{0.0};
    auto closest_vehicle = GetClosestCarInCurrentLane();
    auto distance{100.0};
    if (closest_vehicle != nullptr)
    {
        target_velocity = closest_vehicle->V();
        distance = closest_vehicle->s - car_dynamics.s;
    }

    std::cerr << "Speed of leading vehicle: " << target_velocity << std::endl;

    constexpr auto mphtokph = 2.24;
    target_velocity *= mphtokph;
    const auto safe_distance = 20.0;

    if (target_ref_velocity > target_velocity || distance < safe_distance)
    {
        target_ref_velocity -= kMaxDeceleration;
    }

    if (target_ref_velocity < target_velocity)
    {
        target_ref_velocity += kMaxAcceleration;
    }

    std::cerr << "Target Speed: " << target_velocity << std::endl;
    std::cerr << "Speed: " << target_ref_velocity << std::endl;

    GenerateTrajectory(target_ref_velocity);
}

void PathPlanner::CalculateChangeLaneToLeftPathPoints()
{
    GenerateTrajectory(target_ref_velocity, true);
}

void PathPlanner::CalculateChangeLaneToLeftRightPoints()
{
    GenerateTrajectory(target_ref_velocity, true);
}

void PathPlanner::UpdateCarDynamics(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed)
{
    car_dynamics.x = car_x;
    car_dynamics.y = car_y;
    car_dynamics.s = car_s;
    car_dynamics.d = car_d;
    car_dynamics.yaw = car_yaw;
    car_dynamics.speed = car_speed;
}

void PathPlanner::UpdateMapWayPoints(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &s,
                                     const std::vector<double> &dx, const std::vector<double> &dy)
{
    map_way_points.x = x;
    map_way_points.y = y;
    map_way_points.s = s;
    map_way_points.dx = dx;
    map_way_points.dy = dy;
}

void PathPlanner::UpdatePreviousPathPoints(const std::vector<double> &x, const std::vector<double> &y)
{
    previous_path_points.x = x;
    previous_path_points.y = y;
}

void PathPlanner::UpdateTrafficVehicleList(const std::vector<TrafficVehicle> &traffic_vehicles)
{
    traffic_vehicle_list = traffic_vehicles;
}

void PathPlanner::GenerateTrajectory(const double target_ref_velocity, const bool lane_change_trajectory)
{
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    vector<double> way_pts_x;
    vector<double> way_pts_y;

    // Use the previous path end points as the starting reference

    double ref_x{car_dynamics.x};
    double ref_y{car_dynamics.y};
    double ref_yaw{deg2rad(car_dynamics.yaw)};

    // If previous size is almost empty, then use the position as starting reference points
    if (previous_path_points.x.size() < 2)
    {
        // Create points tangent to the angle of the car, one for current pos'n and other previous angle position
        const auto prev_car_x{car_dynamics.x - cos(car_dynamics.yaw)};
        const auto prev_car_y{car_dynamics.y - sin(car_dynamics.yaw)};

        // we got two points to place 30 m apart
        way_pts_x.push_back(prev_car_x);
        way_pts_x.push_back(ref_x);

        way_pts_y.push_back(prev_car_y);
        way_pts_y.push_back(ref_y);
    }
    else
    {
        // Use the previous path end points as the starting reference
        ref_x = previous_path_points.x[previous_path_points.x.size() - 1];
        ref_y = previous_path_points.y[previous_path_points.x.size() - 1];

        const double ref_x_prev = previous_path_points.x[previous_path_points.x.size() - 2];
        const double ref_y_prev = previous_path_points.y[previous_path_points.x.size() - 2];

        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // we got two points to place 30 m apart
        way_pts_x.push_back(ref_x_prev);
        way_pts_x.push_back(ref_x);

        way_pts_y.push_back(ref_y_prev);
        way_pts_y.push_back(ref_y);
    }

    int lane_id = GetHostLaneId();

    // In frenet, add evenly 30 m spaced points ahead of the starting reference
    auto x = lane_change_trajectory ? 2 : 1;

    vector<double> way_points_at_30m{getXY(
        car_dynamics.s + (30 * x), 2 + (lane_id * lane_width),
        map_way_points.s, map_way_points.x, map_way_points.y)};

    vector<double> way_points_at_60m{getXY(
        car_dynamics.s + (60 * x), 2 + (lane_id * lane_width),
        map_way_points.s, map_way_points.x, map_way_points.y)};

    vector<double> way_points_at_90m{getXY(
        car_dynamics.s + (90 * x), 2 + (lane_id * lane_width),
        map_way_points.s, map_way_points.x, map_way_points.y)};

    // add these new waypoints to the reference points
    way_pts_x.push_back(way_points_at_30m[0]);
    way_pts_x.push_back(way_points_at_60m[0]);
    way_pts_x.push_back(way_points_at_90m[0]);

    way_pts_y.push_back(way_points_at_30m[1]);
    way_pts_y.push_back(way_points_at_60m[1]);
    way_pts_y.push_back(way_points_at_90m[1]);

    // Till here we have in total 5 reference way points, 2 from previous one and 3 spaced at 30, 60 and 90 m

    for (unsigned int itr = 0; itr < way_pts_x.size(); itr++)
    {
        // Shift the way points reference angle to 0 deg such that they are in car co-ordinate system
        // i.e. the previous points may be pointing to an angle, we need to make them as 0 deg

        const auto shift_x{way_pts_x[itr] - ref_x};
        const auto shift_y{way_pts_y[itr] - ref_y};

        way_pts_x[itr] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        way_pts_y[itr] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    // create a spline
    tk::spline spline;

    // set (x, y) points to the spline
    spline.set_points(way_pts_x, way_pts_y);

    // start from all previous points from the last time
    for (unsigned itr = 0; itr < previous_path_points.x.size(); itr++)
    {
        // This is needed in order to reduce the sudden jerk
        next_x_vals.push_back(previous_path_points.x[itr]);
        next_y_vals.push_back(previous_path_points.y[itr]);
    }

    // compute how to break up the spline points so that we travel at our desired ref velocity
    const auto target_x{30.0};             // goal is to reach 30m
    const auto target_y{spline(target_x)}; // get y point for given x
    const auto target_distance{sqrt(
        (target_x * target_x) + (target_y * target_y))};

    // used to get next x for given N
    auto x_add_on{0.};

    // fill up the rest of the path planner after filling it with previous points if any
    for (unsigned itr = 1; itr < 50 - previous_path_points.x.size(); itr++)
    {
        constexpr auto time_gap = 0.02;
        constexpr auto mphtokph = 2.24;

        const double number_of_points{target_distance / (time_gap * target_ref_velocity / mphtokph)};

        double next_x_point{x_add_on + (target_distance / number_of_points)};
        double next_y_point{spline(next_x_point)};

        x_add_on = next_x_point;

        // convert back to the global coordinate
        double x_ref{next_x_point};
        double y_ref{next_y_point};

        next_x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        next_y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        next_x_point += ref_x;
        next_y_point += ref_y;

        next_x_vals.push_back(next_x_point);
        next_y_vals.push_back(next_y_point);
    }

    current_path = PathPoints(next_x_vals, next_y_vals);
}

int PathPlanner::GetHostLaneId() const
{
    switch (current_target_lane)
    {
    case VehicleLane::Left:
        return 0;
    case VehicleLane::Middle:
        return 1;
    case VehicleLane::Right:
        return 2;
    default:
        return -1;
    }
}

const TrafficVehicle *PathPlanner::GetClosestCarInCurrentLane() const
{
    const TrafficVehicle *closest_traffic_vehicle = nullptr;
    auto current_shortest_distance = 0;
    for (const auto &traffic_vehicle : traffic_vehicle_list)
    {
        if (traffic_vehicle.Lane() == current_target_lane && traffic_vehicle.s > car_dynamics.s)
        {
            if (closest_traffic_vehicle == nullptr || current_shortest_distance > (traffic_vehicle.s - car_dynamics.s))
            {
                current_shortest_distance = (traffic_vehicle.s - car_dynamics.s);
                closest_traffic_vehicle = &traffic_vehicle;
            }
        }
    }
    return closest_traffic_vehicle;
}

bool PathPlanner::IsChangeToLanePossible(const VehicleLane &vehicle_lane) const
{
    const double min_lane_change_speed = 30.0;

    if (target_ref_velocity < min_lane_change_speed)
    {
        return false;
    }

    if (vehicle_lane == VehicleLane::Invalid)
    {
        return false;
    }

    const double safety_distance_front{35.0};
    const double safety_distance_rear{15.0};

    for (const auto &traffic_vehicle : traffic_vehicle_list)
    {
        if ((traffic_vehicle.Lane() == vehicle_lane) &&
            (traffic_vehicle.s < car_dynamics.s + safety_distance_front) &&
            (traffic_vehicle.s > car_dynamics.s - safety_distance_rear))
        {
            return false;
        }
    }
    return true;
}

VehicleLane PathPlanner::GetLeftLane() const
{
    switch (current_target_lane)
    {
    case VehicleLane::Left:
        return VehicleLane::Invalid;
    case VehicleLane::Middle:
        return VehicleLane::Left;
    case VehicleLane::Right:
        return VehicleLane::Middle;
    default:
        return VehicleLane::Invalid;
    }
}

VehicleLane PathPlanner::GetRightLane() const
{
    switch (current_target_lane)
    {
    case VehicleLane::Left:
        return VehicleLane::Middle;
    case VehicleLane::Middle:
        return VehicleLane::Right;
    case VehicleLane::Right:
        return VehicleLane::Invalid;
    default:
        return VehicleLane::Invalid;
    }
}

VehicleLane PathPlanner::GetCurrentLane() const
{
    return DToLane(car_dynamics.d);
}