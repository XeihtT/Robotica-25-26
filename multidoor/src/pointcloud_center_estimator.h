//
// Created by pbustos on 21/11/25.
//

#ifndef ROBUST_ROOM_CENTER_ESTIMATOR_H
#define ROBUST_ROOM_CENTER_ESTIMATOR_H

#include <Eigen/Dense>
#include <vector>
#include <optional>
#include <Lidar3D.h>

namespace rc
{
    class PointcloudCenterEstimator
    {
    public:
        struct Config
        {
            int num_sectors = 360;              // 10° sectors
            double max_range = 30000.0;           // meters
            double min_range = 200;           // Remove robot hardware
            double outlier_std_threshold = 2500; // Statistical outlier removal
            double obb_fit_tolerance = 100;    // Bounding box fit tolerance
            size_t min_valid_points = 20;      // Minimum points required

            Config(){}
        };

        explicit PointcloudCenterEstimator(const Config &config = Config{});

        std::optional<Eigen::Vector2f> estimate(const std::vector<Eigen::Vector2f>& points);
        std::optional<Eigen::Vector2f> estimate(const RoboCompLidar3D::TPoints& points);

    private:
        Config config_;

        std::vector<Eigen::Vector2f> filterPoints(const std::vector<Eigen::Vector2f>& points);
        std::vector<Eigen::Vector2f> extractBoundaryPoints(const std::vector<Eigen::Vector2f>& points);
        bool isLocalMaximum(const Eigen::Vector2f& candidate,
                           const std::vector<Eigen::Vector2f>& neighbors,
                           double threshold);
        std::vector<Eigen::Vector2f> removeStatisticalOutliers(const std::vector<Eigen::Vector2f>& points);
        Eigen::Vector2f calculateRobustCentroid(const std::vector<Eigen::Vector2f>& points);
        std::vector<Eigen::Vector2f> computeConvexHull(const std::vector<Eigen::Vector2f>& points);

        struct OBB
        {
            Eigen::Vector2f center{};
            double width = 0.0;
            double height = 0.0;
            double rotation = 0.0; // radians
        };

        OBB computeOBB(const std::vector<Eigen::Vector2f>& hull);
    };


};
#endif // ROBUST_ROOM_CENTER_ESTIMATOR_H