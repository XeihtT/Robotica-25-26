/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#ifdef emit
#  undef emit
#endif
#include <execution>
#include <expected>

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <ranges>
// #include <webots/Robot.hpp>
#include "/usr/local/webots/include/controller/cpp/webots/Robot.hpp"
#include "cppitertools/itertools.hpp"
#include <random>
#include <math.h>
#include<cppitertools/enumerate.hpp>
#include <Eigen/Dense>
#include "rapplication/rapplication.h"
#include "room_detector.h"
#include "hungarian.h"
#include <cppitertools/enumerate.hpp>
#include <cppitertools/zip.hpp>


/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */

enum class State { FORWARD, TURN_FORWARD, TURN_FOLLOW, SPIRAL, FOLLOW_WALL};

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();



	struct NominalRoom {
		float width;   // mm
		float length;  // mm
		Corners corners;
		QRectF rect = QRectF(-5000, -2500, 10000, 5000);
		explicit NominalRoom(
			const float width_ = 10000.f,
			const float length_ = 5000.f,
			Corners corners_ = {}
		) : width(width_), length(length_), corners(std::move(corners_)) {}

		// Transforma las esquinas con una matriz de transformación
		// Para pasar de habitación a robot, usa el inverso de robot_pose
		Corners transform_corners_to(const Eigen::Affine2d &transform) const {
			Corners transformed_corners;
			for (const auto &[p, _, __] : corners) {
				Eigen::Vector2d ep(p.x(), p.y());
				Eigen::Vector2d tp = transform * ep;
				transformed_corners.emplace_back(
					QPointF(static_cast<float>(tp.x()), static_cast<float>(tp.y())),
					0.f,
					0.f
				);
			}
			return transformed_corners;
		}
	};




public slots:

	//Chocachoca
	void new_target_slot(QPointF);

	//Chocachoca
	void draw_lidar(const RoboCompLidar3D::TPoints& filter_data, QGraphicsScene* scene);

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();



private:

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;
	State state = State::SPIRAL;

	//Random nums:
	std::random_device rd;
	std::mt19937 gen;
	std::uniform_real_distribution<float> rand;
	std::uniform_int_distribution<int> rand_turn_way;

	// graphics

	//room
	rc::Room_Detector room_detector;
	NominalRoom room{
		10000.f, 5000.f,
		{
	        {QPointF{-5000.f, 2500.f}, 0.f, 0.f},
			{QPointF{ 5000.f, 2500.f}, 0.f, 0.f},
			{QPointF{ 5000.f,  -2500.f}, 0.f, 0.f},
			{QPointF{-5000.f,  -2500.f}, 0.f, 0.f}
		}
	};


	//robot
	Eigen::Affine2d robot_pose;
	//match
	rc::Hungarian hungarian;

	//Chocachoca todo lo de abajo:
	QRectF dimensions;

	AbstractGraphicViewer *viewer1, *viewer2;
	const int ROBOT_LENGTH = 400;
	QGraphicsPolygonItem *robot_polygon;

	std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points);
	RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);
	void update_robot_position();

	std::tuple<float, float> update_robot_state(const RoboCompLidar3D::TPoints& points);

	std::tuple<State, float, float> forward_method(const RoboCompLidar3D::TPoints& points);
	std::tuple<State, float, float> follow_wall_method(const RoboCompLidar3D::TPoints& points);
	std::tuple<State, float, float> spiral_method(const RoboCompLidar3D::TPoints& points);

	std::tuple<State, float, float> turn_forward_method(const RoboCompLidar3D::TPoints& filter_data);
	std::tuple<State, float, float> turn_follow_method(const RoboCompLidar3D::TPoints& points);
	RoboCompLidar3D::TPoints filtro_datos();

	std::expected<int, std::string> closest_lidar_index_to_given_angle(const auto &points, float angle);



signals:
	//void customSignal();
};

#endif
