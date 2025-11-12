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
#include "time_series_plotter.h"
#include "nominal_room.h"
#include "door_detector.h"
#include "image_processor.h"


/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */




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

	void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);




	struct NominalRoom {
		float width;   // mm
		float length;  // mm
		Corners corners;
		QRectF rect = QRectF(-5000, -2500, 10000, 5000);
		NominalRoom(
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
	void draw_lidar(const RoboCompLidar3D::TPoints &filtered_points, std::optional<Eigen::Vector2d> center, QGraphicsScene *scene);
	//void draw_lidar(const RoboCompLidar3D::TPoints &filtered_points, QGraphicsScene *scene);

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

	//Random nums:
	std::random_device rd;
	std::mt19937 gen;
	std::uniform_real_distribution<float> rand;
	std::uniform_int_distribution<int> rand_turn_way;

	//params
	struct Params
	{
		float ROBOT_WIDTH = 460;  // mm
		float ROBOT_LENGTH = 480;  // mm
		float MAX_ADV_SPEED = 1000; // mm/s
		float MAX_ROT_SPEED = 1; // rad/s
		float MAX_SIDE_SPEED = 50; // mm/s
		float MAX_TRANSLATION = 500; // mm/s
		float MAX_ROTATION = 0.2;
		float STOP_THRESHOLD = 700; // mm
		float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm
		float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
		// wall
		float LIDAR_RIGHT_SIDE_SECTION = M_PI/3; // rads, 90 degrees
		float LIDAR_LEFT_SIDE_SECTION = -M_PI/3; // rads, 90 degrees
		float WALL_MIN_DISTANCE = ROBOT_WIDTH*1.2;
		// match error correction
		float MATCH_ERROR_SIGMA = 150.f; // mm
		float DOOR_REACHED_DIST = 300.f;
		std::string LIDAR_NAME_LOW = "bpearl";
		std::string LIDAR_NAME_HIGH = "helios";
		QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

		// relocalization
		float RELOCAL_CENTER_EPS = 300.f;    // mm: stop when |mean| < eps
		float RELOCAL_KP = 0.002f;           // gain to convert mean (mm) -> speed (magnitude)
		float RELOCAL_MAX_ADV = 300.f;       // mm/s cap while re-centering
		float RELOCAL_MAX_SIDE = 300.f;      // mm/s cap while re-centering
		float RELOCAL_ROT_SPEED = 0.3f;     // rad/s while aligning
		float RELOCAL_DELTA = 5.0f * M_PI/180.f; // small probe angle in radians
		float RELOCAL_MATCH_MAX_DIST = 2000.f;   // mm for Hungarian gating
		float RELOCAL_DONE_COST = 500.f;
		float RELOCAL_DONE_MATCH_MAX_ERROR = 1000.f;
	};
	Params params;

	// graphics

	//room
	rc::Room_Detector room_detector;
	NominalRoom rooms[2] = {
		{
			10000.f, 5000.f,
			{
	            {QPointF{-5000.f,  2500.f}, 0.f, 0.f},
				{QPointF{ 5000.f,  2500.f}, 0.f, 0.f},
				{QPointF{ 5000.f, -2500.f}, 0.f, 0.f},
				{QPointF{-5000.f, -2500.f}, 0.f, 0.f}
			}
		},
		{} // segundo NominalRoom vacío (constructor por defecto)
	};


	//robot
	Eigen::Affine2d robot_pose;
	//match
	rc::Hungarian hungarian;


	// plotter
	std::unique_ptr<TimeSeriesPlotter> time_series_plotter;
	int match_error_graph; // To store the index of the speed graph


	// doors
	DoorDetector door_detector;

	// image processor
	rc::ImageProcessor image_processor;

	// timing
	std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();

	// relocalization
	bool relocal_centered = false;
	bool localised = false;

	bool update_robot_pose(const Corners &corners, const Match &match);
	void move_robot(float adv, float rot, float max_match_error);
	Eigen::Vector3d solve_pose(const Corners &corners, const Match &match);
	void predict_robot_pose();
	std::tuple<float, float> robot_controller(const Eigen::Vector2f &target);


	//states
	// state machine
	enum class STATE {GOTO_DOOR, ORIENT_TO_DOOR, LOCALISE, GOTO_ROOM_CENTER, TURN, IDLE, CROSS_DOOR};
	inline const char* to_string(const STATE s) const
	{
		switch(s) {
		case STATE::IDLE:               return "IDLE";
		case STATE::LOCALISE:           return "LOCALISE";
		case STATE::GOTO_DOOR:          return "GOTO_DOOR";
		case STATE::TURN:               return "TURN";
		case STATE::ORIENT_TO_DOOR:     return "ORIENT_TO_DOOR";
		case STATE::GOTO_ROOM_CENTER:   return "GOTO_ROOM_CENTER";
		case STATE::CROSS_DOOR:         return "CROSS_DOOR";
		default:                        return "UNKNOWN";
		}
	}
	STATE state = STATE::LOCALISE;
	using RetVal = std::tuple<STATE, float, float>;
	RetVal goto_door(const RoboCompLidar3D::TPoints &points);
	RetVal orient_to_door(const RoboCompLidar3D::TPoints &points);
	RetVal cross_door(const RoboCompLidar3D::TPoints &points);
	RetVal localise(const Match &match);
	RetVal goto_room_center(const RoboCompLidar3D::TPoints &points);
	RetVal update_pose(const Corners &corners, const Match &match);
	RetVal turn(const Corners &corners);
	RetVal process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, AbstractGraphicViewer *viewer);

	// viewer
	AbstractGraphicViewer *viewer, *viewer_room;
	QGraphicsPolygonItem *robot_draw, *robot_room_draw;

	//Chocachoca todo lo de abajo:
	QRectF dimensions;

	enum class State {
		FORWARD, TURN_FORWARD, TURN_FOLLOW, SPIRAL, FOLLOW_WALL
	};

	State state_chocachoca = State::SPIRAL;

	AbstractGraphicViewer *viewer1, *viewer2;
	const int ROBOT_LENGTH = 400;
	QGraphicsPolygonItem *robot_polygon;

	std::vector<QPointF> filter_close_corners(const Corners& corners, float min_dist);

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
