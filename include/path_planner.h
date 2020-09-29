#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <tuple>
#include <vector>
#include "../include/json.hpp"
#include "../include/helpers.h"
#include <iostream>

namespace path_planner
{

    using PathPoints = std::tuple<std::vector<double>, std::vector<double>>;

    enum class State
    {
        FollowLane,
        FollowLeadingVehicle,
        ChangeLaneToLeft,
        ChangeLaneToRight
    };

    struct CarDynamics
    {
        CarDynamics() : x{0.0}, y{0.0}, s{0.0}, d{0.0}, yaw{0.0}, speed{0.0} {};
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
    };

    struct MapWayPoints
    {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> s;
        std::vector<double> dx;
        std::vector<double> dy;
    };

    struct PreviousPathPoints
    {
        std::vector<double> x;
        std::vector<double> y;
    };

    struct TrafficVehicle
    {
        TrafficVehicle(nlohmann::json node) : vx(node[3]), vy(node[4]), s(node[5]), d(node[6]){};
        double vx;
        double vy;
        double s;
        double d;
        double V() const { return sqrt(vx * vx + vy * vy); }

        VehicleLane Lane() const
        {
            return DToLane(d);
        }
    };

    class PathPlanner
    {
    public:
        PathPlanner();
        void Step();
        PathPoints GetPathPoints() const;
        void UpdateCarDynamics(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);
        void UpdateMapWayPoints(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &s,
                                const std::vector<double> &dx, const std::vector<double> &dy);
        void UpdatePreviousPathPoints(const std::vector<double> &x, const std::vector<double> &y);
        void UpdateTrafficVehicleList(const std::vector<TrafficVehicle> &traffic_vehicle_list);

    private:
        void CalculateNewState();
        void CalculatePathPoints();
        void CalculateFollowLanePathPoints();
        void CalculateFollowLeadingVehiclePathPoints();
        void CalculateChangeLaneToLeftPathPoints();
        void CalculateChangeLaneToLeftRightPoints();

        void GenerateTrajectory(const double target_ref_velocity, const bool lane_change_trajectory = false);
        int GetHostLaneId() const;
        const TrafficVehicle *GetClosestCarInCurrentLane() const;
        bool IsChangeToLanePossible(const VehicleLane &vehicle_lane) const;
        VehicleLane GetLeftLane() const;
        VehicleLane GetRightLane() const;
        VehicleLane GetCurrentLane() const;

        State current_state;
        PathPoints current_path;
        CarDynamics car_dynamics;
        MapWayPoints map_way_points;
        PreviousPathPoints previous_path_points;
        std::vector<TrafficVehicle> traffic_vehicle_list;
        double target_ref_velocity{0.0};
        VehicleLane current_target_lane{VehicleLane::Middle};

        const double lane_width{k_lane_width};
        const double kMaxAcceleration{0.9};
        const double kMaxDeceleration{0.95};
    };

} // namespace path_planner

#endif //PATH_PLANNER_H