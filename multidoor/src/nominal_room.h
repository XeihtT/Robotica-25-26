#pragma once
#include <QPointF>
#include <QRectF>
#include <Eigen/Dense>
#include <vector>
#include "src/common_types.h"

  struct NominalRoom
        {
            float width; //  mm
            float length;
            Walls my_walls;
            Doors my_doors;
            explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners  corners_ = {}) :
                width(width_), length(length_)
            {};
            [[nodiscard]] Corners corners() const
            {
                // compute corners from width and length
                return {
                    {QPointF{-width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, length/2.f}, 0.f, 0.f},
                    {QPointF{-width/2.f, length/2.f}, 0.f, 0.f}
                };
            }
            [[nodiscard]] QRectF rect() const
            {
                return QRectF{-width/2.f, -length/2.f, width, length};
            }
            [[nodiscard]] Corners transform_corners_to(const Eigen::Affine2d &transform) const  // for room to robot pass the inverse of robot_pose
            {
                Corners transformed_corners;
                for(const auto &[p, _, __] : corners())
                {
                    auto ep = Eigen::Vector2d{p.x(), p.y()};
                    Eigen::Vector2d tp = transform * ep;
                    transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
                }
                return transformed_corners;
            }
            [[nodiscard]] Walls walls(RoboCompLidar3D::TPoints data, rc::Room_Detector rd, QGraphicsScene* scene) {
                auto corners = this->corners();
                corners.push_back(corners[0]);
                Walls toReturn;
                for (const auto& [i, c]: corners | iter::sliding_window(2)|iter::enumerate) {
                    const auto& [c1, _,__]=c[0];
                    const auto& [c2, ___,____]=c[1];

                    const auto& r = Eigen::ParametrizedLine<float, 2>::Through(Eigen::Vector2f{c1.x(), c1.y()}, Eigen::Vector2f{c2.x(), c2.y()});
                    toReturn.emplace_back(r, i, c[0], c[1]);
                }

                my_walls = toReturn;
                return toReturn;

            }
            [[nodiscard]] Wall point_to_wall(const Eigen::Vector2f& p) {
                auto m = std::ranges::min_element(my_walls, [p](const auto& w1, const auto& w2) {
                    const auto& [r1, _, __, ___]=w1;
                    const auto& [r2, a, b, c]=w2;
                    return r1.distance(p) < r2.distance(p);
                });

                return *m;
            }
        };