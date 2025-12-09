//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"

#include <expected>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>


Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    if(points.empty()) return {};

    // get the peaks
    Peaks peaks;
    for (const auto &p : iter::sliding_window(points, 2))
    {
        const auto &p1 = p[0]; const auto &p2 = p[1];
        const float d1 = p1.distance2d; const float d2 = p2.distance2d;
        if (const float dd_da1 = abs(d2 - d1); dd_da1 > 1000.f)
        {
            const auto m = std::ranges::min_element(p, [](auto &pa, auto &pb){return pa.distance2d < pb.distance2d;});
            peaks.emplace_back(Eigen::Vector2f(m->x, m->y), m->phi);
        }
    }

    // non-maximum suppression of peaks: remove peaks closer than 500mm
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
    {
        const bool too_close = std::ranges::any_of(nms_peaks, [&p](const auto &p2) { return (p - std::get<0>(p2)).norm() < 500.f; });
        if (not too_close)
            nms_peaks.emplace_back(p, a);
    }
    peaks = nms_peaks;

    // find doors as pairs of peaks separated by a gap < 1200mm and > 800mm
    Doors doors;
    for(const auto &p : peaks | iter::combinations(2))
    {
        const auto &[p0,a0] = p[0]; const auto &[p1, a1] = p[1];
        const float gap = (p1-p0).norm();
        //qInfo() << "Gap: " << gap;
        if(gap < 1300.f and gap > 800.f)
            doors.emplace_back(p0, a0, p1, a1);
    }
    //qInfo() << __FUNCTION__ << "Peaks found: " << peaks.size() << "Doors found: " << doors.size();

    // draw peaks in viewer
    static std::vector<QGraphicsItem *> doors_draw;
    if(scene)
    {
        for (const auto &p : doors_draw)
        { scene->removeItem(p); delete p;}
        doors_draw.clear();
        const QColor color("blue");
        const QPen pen(color);
        const QBrush brush(color);
        for (const auto &d : doors)
        //for (const auto &[peak, angle] : peaks)
        {
            const auto peak_A_draw = scene->addEllipse(-100, -100, 200, 200, pen, brush);
            const auto peak_B_draw = scene->addEllipse(-100, -100, 200, 200, pen, brush);
            peak_A_draw->setPos(d.p1.x(), d.p1.y());
            peak_B_draw->setPos(d.p2.x(), d.p2.y());
            const auto line_draw = scene->addLine(d.p1.x(), d.p1.y(), d.p2.x(), d.p2.y(), QPen(Qt::cyan, 30));
            doors_draw.push_back(peak_A_draw);
            doors_draw.push_back(peak_B_draw);
            doors_draw.push_back(line_draw);
        }
    }
    doors_cache = doors;
    return doors;
}

RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty())
        return points;

    RoboCompLidar3D::TPoints filtered;
    filtered.reserve(points.size());

    for(const auto &p : points)
    {
        bool discard = false;

        for(const auto &d : doors)
        {
            const float dist_to_door = d.center().norm();
            const bool angle_wraps = d.p2_angle < d.p1_angle;

            // Check angular range
            bool in_range =
                angle_wraps ?
                    ((p.phi + 0.18 > d.p1_angle) || (p.phi - 0.18 < d.p2_angle)) :
                    ((p.phi + 0.18 > d.p1_angle) && (p.phi - 0.18 < d.p2_angle));

            // If the point is "through" the door → throw the point away
            if(in_range && p.distance2d >= dist_to_door)
            {
                discard = true;
                break;
            }
        }

        if(!discard)
            filtered.emplace_back(p);   // ← solo se mete UNA vez
    }

    return filtered;
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points1(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty()) return points;

    // for each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    for(const auto &d : doors)
    {
        const float dist_to_door = d.center().norm();
        // Check if the angular range wraps around the -π/+π boundary
        const bool angle_wraps = d.p2_angle < d.p1_angle;
        for(const auto &p : points)
        {
            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi+0.18 > d.p1_angle) or (p.phi-0.18 < d.p2_angle);
            }
            else
            {
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi+0.18 > d.p1_angle) and (p.phi-0.18 < d.p2_angle);
            }

            // Filter out points that are through the door (in angular range and farther than door)
            if(point_in_angular_range and p.distance2d >= dist_to_door)
                continue;

            //qInfo() << __FUNCTION__ << "Point angle: " << p.phi << " Door angles: " << d.p1_angle << ", " << d.p2_angle << " Point distance: " << p.distance2d << " Door distance: " << dist_to_door;
            filtered.emplace_back(p);
        }
    }
    //qDebug()<<"han sido eliminados: "<<points.size()-filtered.size()<<" puntos"; //de vez en cuando se le cuelan 1 o 2 puntos no se si de ruido o de que
    return filtered;
}

std::expected<Door, std::string> DoorDetector::get_current_door() const
{
    if (doors_cache.empty())
        return std::unexpected<std::string>{"No doors detected"};
    return doors_cache[0];
}

std::expected<Door, std::string> DoorDetector::get_door(const int door) const
{
    if (doors_cache.empty())
        return std::unexpected<std::string>{"No doors detected"};
    qDebug()<<"Hay: "<<doors_cache.size()<<" puertas y estás intentando acceder a la: "<<door;
    return doors_cache.at(door);
}