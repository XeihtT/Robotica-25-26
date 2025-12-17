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
#include "specificworker.h"
//TODO: Hacer que no se choque DEFINITIVO, SACAR CAPTURAS DE PANTALLA Y AÑADIR A DOCU, LIMPIAR Y QUITAR COMENTARIOS Y PONER COMENTARIOS, HACER COMMIT FINAL,
float MIN_TO_WALL_FORWARD = 1100.0f; //Distancia minima que el robot tendra a una pared antes de que este empiece a girar
float MIN_TO_WALL_FOLLOW = 900;
int turn_way = 1;
std::vector<QGraphicsItem*> room_items;
std::vector<QGraphicsItem*> door_items;

chrono::time_point<chrono::steady_clock, chrono::steady_clock::duration> first_time;

//Ahora mismo esta OK pero se sigue chocando un pelin (muy poco, mas bien roce)
//float dist_threshold=1900;
//Usamos sintaxis de inicializacion de lista en el constructor para inicializar los valores aleatorios
SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx), gen(rd()), rand(1700,3600), rand_turn_way(1, 2)
{ //igual deberia ponerle rand (MIN_TO_WALL, 2200) o mas de 2200
	this->startup_check_flag = startup_check;
	//inicializamos los atributos de generacion de numeros aleatorios


	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::initialize()
{
	std::cout << "Initialize worker" << std::endl;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		///////////// Your code ////////
		// Viewer
		viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
		auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_draw = r;
		//viewer->show();


		viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
		auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_room_draw = rr;
		// draw room in viewer_room
		room_items.push_back(viewer_room->scene.addRect(rooms[0].rect(), QPen(Qt::black, 30)));
		//viewer_room->show();
		show();


		// initialise robot pose
		robot_pose.setIdentity();
		robot_pose.translate(Eigen::Vector2f(0.0,0.0));


		// time series plotter for match error
		TimeSeriesPlotter::Config plotConfig;
		plotConfig.title = "Maximum Match Error Over Time";
		plotConfig.yAxisLabel = "Error (mm)";
		plotConfig.timeWindowSeconds = 15.0; // Show a 15-second window
		plotConfig.autoScaleY = false;       // We will set a fixed range
		plotConfig.yMin = 0;
		plotConfig.yMax = 1000;
		time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
		match_error_graph = time_series_plotter->addGraph("", Qt::blue);


		// stop robot
		//move_robot(0, 0, 0);
	}
}



void SpecificWorker::compute()
{
    RoboCompLidar3D::TPoints data = filtro_datos();
    data = door_detector.filter_points(data, &viewer->scene);

    // compute corners
    const auto &[corners, lines] = room_detector.compute_corners(data, &viewer->scene);
    //const auto center_opt = room_detector.estimate_center_from_walls(lines);
    const auto center_opt = centro.estimate(data);
    draw_lidar(data, center_opt, &viewer->scene);

    // Predict robot pose using odometry
    //predict_robot_pose();

    // update robot pose
    float max_match_error = 99999.f;
    Match match;
    if (localised)
    {
        // match corners  transforming first nominal corners to robot's frame
        match = hungarian.match(corners, rooms[room].transform_corners_to(robot_pose.inverse()));

        // compute max of  match error
        if (not match.empty())
        {
            const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
                { return std::get<2>(a) < std::get<2>(b); });
            max_match_error = static_cast<float>(std::get<2>(*max_error_iter));
            time_series_plotter->addDataPoint(match_error_graph,max_match_error);
            //print_match(match, max_match_error); //debugging
        }
        update_robot_pose(corners, match);
    }

    // Process state machine
    RetVal ret_val = process_state(data, corners, match, &viewer->scene, &viewer_room->scene);
    auto [st, adv, rot] = ret_val;
    state = st;

	//qDebug()<<"El state es: "<<to_string(st);
	label_state->setText(QString::fromStdString(to_string(st)));

	std::string location = localised ? "Localised" : "Not localised";

	label_localised->setText(QString::fromStdString(location));



    // Send movements commands to the robot constrained by the match_error
    //qInfo() << __FUNCTION__ << "Adv: " << adv << " Rot: " << rot;
	try{ omnirobot_proxy->setSpeedBase(0, adv, rot);}
	catch (const Ice::Exception &e){ std::cout << e << " " << "Conexión con Laser" << std::endl; return;}
	const double angle = qRadiansToDegrees(std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0)));

    // draw robot in viewer
    robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
    robot_room_draw->setRotation(angle);

    // update GUI
    time_series_plotter->update();
    lcdNumber_adv->display(adv);
    lcdNumber_rot->display(rot);
    lcdNumber_x->display(robot_pose.translation().x());
    lcdNumber_y->display(robot_pose.translation().y());
    lcdNumber_room->display(room); 
    lcdNumber_angle->display(angle);
    last_time = std::chrono::high_resolution_clock::now();;
}
std::tuple<STATE, float, float> SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, QGraphicsScene *scene1, QGraphicsScene *scene2){

	std::tuple<STATE, float, float> result;
	switch (this->state) {
		case STATE::IDLE:
			break; //no hacer nada
		case STATE::GOTO_ROOM_CENTER:
			result = goto_room_center(data);
			break;
		case STATE::TURN:
			result = turn(corners);
			break;
		case STATE::GOTO_DOOR:
			result = goto_door(data, &viewer_room->scene);
			break;
		case STATE::ORIENT_TO_DOOR:
			result = orient_to_door(data);
			break;
		case STATE::CROSS_DOOR:
			result = cross_door(data);
			break;
			/*
		case STATE::LOCALISE: //nose que es este estado xd
			result = localise(match);
			break;
			*/
		default:
			break;
	}

	this->state=std::get<STATE>(result);
	return result;
}
std::vector<QPointF> SpecificWorker::filter_close_corners(const Corners& corners, float min_dist)
{
	std::vector<QPointF> filtered;
	for (auto &[c, _, __] : corners)
	{
		bool too_close = false;
		for (const auto &f : filtered)
		{
			float dist = std::hypot(c.x() - f.x(), c.y() - f.y());
			if (dist < min_dist)
			{
				too_close = true;
				break;
			}
		}
		if (!too_close)
			filtered.push_back(c);
	}
	return filtered;
}
std::expected<int, std::string> SpecificWorker::closest_lidar_index_to_given_angle(const auto &points, float angle)
{
	// search for the point in points whose phi value is closest to angle
	//auto res = std::ranges::find_if(points, [angle](auto &a){ return a.phi > angle;});
	auto res = std::ranges::min_element(points, [angle](auto &a, auto &b) {
	return std::abs(a.phi - angle) < std::abs(b.phi - angle);
	});
	if(res != std::end(points))
		return std::distance(std::begin(points), res);
	else
		return std::unexpected("No closest value found in method <closest_lidar_index_to_given_angle>");
}
void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints& filtered_points,std::optional<Eigen::Vector2f> center ,QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

    // remove all items drawn in the previous iteration
    for(auto i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    auto color = QColor(Qt::green);
    auto brush = QBrush(QColor(Qt::green));
    for(const auto &p : filtered_points)
    {
        auto item = scene->addRect(-50, -50, 100, 100, color, brush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }

	//Adicional de la actividad 3 multiroom para dibujar el centro de la sala
	if(center.has_value())
	{
		const double x = center->x();
		const double y = center->y();

		double radius = 150;   // tamaño del punto
		auto centro = scene->addEllipse(x - radius/2,
						  y - radius/2,
						  radius,
						  radius,
						  QPen(Qt::red),
						  QBrush(Qt::red));

		items.push_back(centro);
	}

    // compute and draw minimum distance point in frontal range
    auto offset_begin = closest_lidar_index_to_given_angle(filtered_points, -0.05); //params.LIDAR_FRONT_SECTION = -10
    auto offset_end = closest_lidar_index_to_given_angle(filtered_points, 0.05); //params.LIDAR_FRONT_SECTION = +10

	if(not offset_begin or not offset_end)
	{ std::cout << offset_begin.error() << " " << offset_end.error() << std::endl; return ;}    // abandon the ship



	if (offset_begin.value() >= filtered_points.size() ||
		offset_end.value() > filtered_points.size() ||
		offset_begin.value() >= offset_end.value()) {
		//qDebug()<< "Offsets fuera de rango";
		return;
		}

    auto min_point = std::min_element(std::begin(filtered_points) + offset_begin.value(), std::begin(filtered_points) + offset_end.value(), [](auto &a, auto &b)
    { return a.distance2d < b.distance2d; });

    QColor dcolor;
    if(min_point->distance2d < 800) //800 de momento = params.STOP_THRESHOLD
        dcolor = QColor(Qt::red);
    else
        dcolor = QColor(Qt::magenta);
    auto ditem = scene->addRect(-100, -100, 200, 200, dcolor, QBrush(dcolor));
    ditem->setPos(min_point->x, min_point->y);
    items.push_back(ditem);

    // compute and draw minimum distance point to wall
    auto wall_res_right = closest_lidar_index_to_given_angle(filtered_points, M_PI/2); //params.LIDAR_RIGHT_SIDE_SECTION = -pi/2
    auto wall_res_left = closest_lidar_index_to_given_angle(filtered_points, -M_PI/2); //params.LIDAR_LEFT_SIDE_SECTION =
    if(not wall_res_right or not wall_res_left)   // abandon the ship
    {
        qWarning() << "No valid lateral readings" << QString::fromStdString(wall_res_right.error()) << QString::fromStdString(wall_res_left.error());
        return;
    }

    auto right_point = filtered_points[wall_res_right.value()];
    auto left_point = filtered_points[wall_res_left.value()];
    // compare both to get the one with minimum distance
    auto min_obj = (right_point.distance2d < left_point.distance2d) ? right_point : left_point;
    auto item = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::orange), QBrush(QColor(QColorConstants::Svg::orange)));
    item->setPos(min_obj.x, min_obj.y);
    items.push_back(item);

    // draw a line from the robot to the minimum distance point
    auto item_line = scene->addLine(QLineF(QPointF(0.f, 0.f), QPointF(min_obj.x, min_obj.y)), QPen(QColorConstants::Svg::orange, 10));
    items.push_back(item_line);
    // Draw two lines coming out from the robot at angles given by params.LIDAR_OFFSET
    // Calculate the end points of the lines
	auto res_right = closest_lidar_index_to_given_angle(filtered_points, 0.005); //params.LIDAR_FRONT_SECTION = 0.005
	auto res_left = closest_lidar_index_to_given_angle(filtered_points, -0.005); //params.LIDAR_FRONT_SECTION = -0.005
    if(not res_right or not res_left)
    { std::cout << res_right.error() << " " << res_left.error() << std::endl; return ;}

    // draw two lines at the edges of the range
    float right_line_length = filtered_points[res_right.value()].distance2d;
    float left_line_length = filtered_points[res_left.value()].distance2d;
    float angle1 = filtered_points[res_left.value()].phi;
    float angle2 = filtered_points[res_right.value()].phi;

    QLineF line_left{QPointF(0.f, 0.f),
                     robot_draw->mapToScene(left_line_length * sin(angle1), left_line_length * cos(angle1))};
    QLineF line_right{QPointF(0.f, 0.f),
                      robot_draw->mapToScene(right_line_length * sin(angle2), right_line_length * cos(angle2))};

    QPen left_pen(Qt::blue, 10); // Blue color pen with thickness 3
    QPen right_pen(Qt::red, 10); // Blue color pen with thickness 3
    auto line1 = scene->addLine(line_left, left_pen);
    auto line2 = scene->addLine(line_right, right_pen);
    items.push_back(line1);
    items.push_back(line2);
}

void SpecificWorker::draw_lidar2(QGraphicsScene *scene, int i)
{
	// remove all items drawn in the previous iteration
	for(auto i: room_items)
	{
		scene->removeItem(i);
		delete i;
	}
	room_items.clear();
	room_items.push_back(scene->addRect(rooms[i].rect(), QPen(Qt::black, 30)));

}

void SpecificWorker::draw_nominal_doors(QGraphicsScene *scene, int room, int current_door) {

	for (auto i: door_items) {
		scene->removeItem(i);
		delete i;
	}
	door_items.clear();
	for (auto d: rooms[room].doors) {
		qDebug()<<d.p1_global.x()<<"///"<<d.p1_global.y();
		qDebug()<<d.p2_global.x()<<"///"<<d.p2_global.y();
		room_items.emplace_back(viewer_room->scene.addLine(d.p1_global.x(), d.p1_global.y(), d.p2_global.x(), d.p2_global.y(), QPen(Qt::red, 90)));
	}
}

void SpecificWorker::new_target_slot(QPointF p)
{
	std::cout << "Nuevo target recibido en: ("
			  << p.x() << ", " << p.y() << ")" << std::endl;
}
RoboCompLidar3D::TPoints SpecificWorker::filtro_datos()
{
	std::optional<RoboCompLidar3D::TPoints> filter_data;
	RoboCompLidar3D::TPoints  p_filter;
	try
	{
		auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 2); //para mayor precision (puedo comparar ejemplos de ejecucion entre este y 0.1f round en la docu)
		//qInfo() << "Size: "<<data.points.size();
		if (data.points.empty()){qDebug()<<"No points"; return p_filter;}

		/*
		std::ranges::copy_if(data.points, std::back_inserter(p_filter),
												   [](auto  &a){ return a.z < 500 and a.distance2d > 200;});
		*/

		//Esto siguiente es opcional
		p_filter = filter_isolated_points(data.points, 200);
		if (p_filter.empty())
			return {};

		return p_filter;
	}
	catch (const Ice::Exception &e){ std::cout<<e.what()<<std::endl; return p_filter;}
}
std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points) {

	if (points.empty())
		return {};

	RoboCompLidar3D::TPoints result; result.reserve(points.size());

	for (auto&& [angle, group]: iter::groupby(points, [](const auto& p)
	{ float multiplier=std::pow(10.0f, 2); return std::floor(p.phi*multiplier)/multiplier;})) {
		auto min=std::min_element(std::begin(group), std::end(group), [](const auto& a, const auto& b){return a.r<b.r;});
		result.emplace_back(*min);
	}
	return result;
}
RoboCompLidar3D::TPoints SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d)
{
	if (points.empty()) return {};

	const float d_squared = d * d;  // Avoid sqrt by comparing squared distances
	std::vector<bool> hasNeighbor(points.size(), false);

	// Create index vector for parallel iteration
	std::vector<size_t> indices(points.size());
	std::iota(indices.begin(), indices.end(), size_t{0});

	// Parallelize outer loop - each thread checks one point
	std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
		{
			const auto& p1 = points[i];
			// Sequential inner loop (avoid nested parallelism)
			for (auto &&[j,p2] : iter::enumerate(points))
			{
				if (i == j) continue;
				const float dx = p1.x - p2.x;
				const float dy = p1.y - p2.y;
				if (dx * dx + dy * dy <= d_squared)
				{
					hasNeighbor[i] = true;
					break;
				}
			}
	});

	// Collect results
	std::vector<RoboCompLidar3D::TPoint> result;
	result.reserve(points.size());
	for (auto &&[i, p] : iter::enumerate(points))
		if (hasNeighbor[i])
			result.push_back(points[i]);
	return result;
}
void SpecificWorker::update_robot_position() {
	try {
		RoboCompGenericBase::TBaseState bState;
		omnirobot_proxy->getBaseState(bState);
		robot_polygon->setRotation(bState.alpha*100/M_PI);
		robot_polygon->setPos(bState.x, bState.z);
		std::cout<<bState.alpha<< " "<<bState.x<<" "<<bState.z<<std::endl;

	}
	catch (const Ice::Exception& e){std::cout<<e.what();}
}
SpecificWorker::RetVal SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints& points)
{
	auto center = centro.estimate(points);

	if (!center.has_value())
		return{STATE::GOTO_ROOM_CENTER, 0, 0}; //devuelvo el mismo estado pero sin cambiar nada para que en la proxima iteracion si pille el centro
		//igualmente es necesario upgradear el estimador del centro

	if (center->norm() < 100.0f) //el problema esta aqui
		return {STATE::TURN, 0, 0};


	// 2. Convertir Vector2d → Vector2f
	Eigen::Vector2f center_f = center.value().cast<float>();

	// 3. Llamar al controlador
	auto [v, w] = robot_controller(center_f);

	// 4. Devolver estado, avance y rotación
	return {STATE::GOTO_ROOM_CENTER, v, w};

}


std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f &target)
{
	float new_theta, rot;
	float sigma = M_PI / 4;
	float kp = 0.5f;
	float k = 10; //10, termino medio
	float d_stop = 600.0f; //le pongo 600 a la distancia de freno

	const double x = target.x();
	const double y = target.y();

	new_theta = std::atan2(x, y);
	rot = (kp * new_theta); //+ (kd * inc_theta);

	float d = std::sqrt(x*x + y*y);
	float f_zero = std::exp(- (new_theta*new_theta)/(2*sigma*sigma));
	float f_d = 1/ (1+std::exp(k/(0.01f+d-d_stop))); //la alternativa con el signo - y multiplicando funciona peor

	//vmax = 800
	float v = 800 * f_zero * f_d;

	//qDebug()<<"D es: "<<d;
	//qDebug()<<"f_zero es: "<<f_zero;
	//qDebug()<<"f_d es: "<<f_d;
	//qDebug()<<"v es: "<<v;
	//qDebug()<<"El resultado del exp de la formula de f_d es: "<<std::exp(k/(0.01f+d-d_stop));
	//debe devolver v y rot en vez de 0, 0
	return {v, rot};
}

SpecificWorker::RetVal SpecificWorker::turn(const Corners &corners)
{
    const auto &[success, room_index, left_right] = image_processor.check_colour_patch_in_image(camera360rgb_proxy, this->label_img);
    if (success)
    {
        room = room_index;
        const auto m = hungarian.match(corners,rooms[room].corners() );
        if (m.empty())
        {
            qInfo() << __FUNCTION__ << "empty match";
        };
        if (m.size() < 3)
        {
            qInfo() << __FUNCTION__ << "m size < 3";
            return{STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};
        }
        const auto max_error_iter = std::ranges::max_element(m, [](const auto &a, const auto &b)
                                { return std::get<2>(a) < std::get<2>(b); });
        if (const auto max_match_error = std::get<2>(*max_error_iter); max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR)
        {
            qInfo() << __FUNCTION__ << "match error > " << params.RELOCAL_DONE_MATCH_MAX_ERROR;
            return{STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};
        }
        // update robot pose to have a fresh value
        auto res= update_robot_pose(corners, m);
    	/* //TODO: VER QUE ESTO FUNCIONE
		if (not res) {
			qDebug()<<""
			return{STATE::TURN, 0.0f, 0.5*left_right*params.RELOCAL_ROT_SPEED};
		}
		*/
        ///////////////////////////////////////////////////////////////////////
        // save doors to nominal_room
        auto doors = door_detector.doors();
        if (doors.empty()) { qWarning() << __FUNCTION__ << "empty doors"; return{STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};}
        for (auto &d : doors)
        {
            d.p1_global = rooms[room].get_projection_of_point_on_closest_wall(robot_pose * d.p1);
            d.p2_global = rooms[room].get_projection_of_point_on_closest_wall(robot_pose * d.p2);
        }
        rooms[room].doors = doors;
    	num_door = (num_door + 1) % door_detector.doors().size();
    	int current_door = 0; // TODO: more sophisticated choice (le faltaba un int)
    	draw_lidar2(&viewer_room->scene, room);
    	draw_nominal_doors(&viewer_room->scene, room, current_door);
    	localised = true;

        // choose door to go
    	/*
        // we need to match the current selected nominal door to the successive local doors detected during the approach
        // select the local door closest to the selected nominal door
        const auto dn = rooms[room].doors[current_door];
        const auto ds = door_detector.doors();
        const auto sd = std::ranges::min_element(ds, [dn, this](const auto &a, const auto &b)
                {  return (a.center() - robot_pose.inverse() * dn.center_global()).norm() <
                          (b.center() - robot_pose.inverse() * dn.center_global()).norm(); });
        // sd is the closest local door to the selected nominal door. Update nominal door with local values
        rooms[room].doors[current_door].p1 = sd->p1;
        rooms[room].doors[current_door].p2 = sd->p2;
		*/

        return {STATE::GOTO_DOOR, 0.0f, 0.0f};  // SUCCESS
    }
    // continue turning
    return {STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};
}

SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    Doors doors;
    // Exit conditions
    if ( doors = door_detector.doors(); doors.empty())
    {
        qInfo() << __FUNCTION__ << "No doors detected, switching to UPDATE_POSE";
        return {STATE::GOTO_DOOR, 0.f, 0.f};  // TODO: keep moving for a while?
    }
    // select from doors, the one closest to the nominal door
    Door target_door;
    if (localised)
    {
        qInfo() << __FUNCTION__ << "Localised, selecting door closest to nominal door";
        const auto dn = rooms[room].doors[num_door];
        const auto sd = std::ranges::min_element(doors, [dn, this](const auto &a, const auto &b)
               {  return (a.center() - robot_pose.inverse() * dn.center_global()).norm() <
                         (b.center() - robot_pose.inverse() * dn.center_global()).norm(); });
        target_door = *sd;
    }
    else  // select the one closest to the robot's heading direction
    {
        qInfo() << __FUNCTION__ << "Not localised, selecting door closest to robot heading";
        const auto sd = std::ranges::min_element(doors, [](const auto &a, const auto &b)
               {  return abs(a.p1_angle) < abs(b.p1_angle); });
        target_door = *sd;
    }
    qInfo() << target_door.p1.x() << target_door.p1.y();

    // distance to target is less than threshold, stop and switch to ORIENT_TO_DOOR
    constexpr float offset = 600.f;
    const auto target = target_door.center_before(robot_pose.translation(), offset);
    const auto dist_to_door = target.norm();

    // draw target
    static QGraphicsItem *door_target_draw = nullptr;
    if (door_target_draw != nullptr)
        scene->removeItem(door_target_draw);
    door_target_draw = scene->addEllipse(-50, -50, 100, 100, QPen(Qt::magenta), QBrush(Qt::magenta));
    door_target_draw->setPos(target.x(), target.y());

   // Exit condition
    if (dist_to_door < params.DOOR_REACHED_DIST)
    {
        qInfo() << __FUNCTION__ << "Door reached at distance " << dist_to_door << ", switching to ORIENT_TO_DOOR";
        return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};
    }

    qInfo() << __FUNCTION__ << "moving to door at " << target.x() << "," << target.y() << " dist: " << dist_to_door;
    const auto &[adv, rot] = robot_controller(target); // go to first detected door
    return {STATE::GOTO_DOOR, adv, rot};
}


std::tuple<STATE, float, float> SpecificWorker::orient_to_door(const RoboCompLidar3D::TPoints &points) {

	Door door = rooms[room].doors.at(num_door);
	float new_theta, rot;
	float kp = 0.5f;

	Doors doors;

	// Exit conditions
	if ( doors = door_detector.doors(); doors.empty())
	{
		qInfo() << __FUNCTION__ << "No doors detected, switching to UPDATE_POSE";
		return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};  // TODO: keep moving for a while?
	}
	Door target_door;
	if (localised)
	{
		qInfo() << __FUNCTION__ << "Localised, selecting door closest to nominal door";
		const auto dn = rooms[room].doors[num_door];
		const auto sd = std::ranges::min_element(doors, [dn, this](const auto &a, const auto &b)
			   {  return (a.center() - robot_pose.inverse() * dn.center_global()).norm() <
						 (b.center() - robot_pose.inverse() * dn.center_global()).norm(); });
		target_door=*sd;
	}
	else  // select the one closest to the robot's heading direction
	{
		qInfo() << __FUNCTION__ << "Not localised, selecting door closest to robot heading";
		const auto sd = std::ranges::min_element(doors, [](const auto &a, const auto &b)
			   {  return abs(a.p1_angle) < abs(b.p1_angle); });
		target_door=*sd;
	}
	Eigen::Vector2f middle_point=target_door.center();
	const double x = middle_point.x();
	const double y = middle_point.y();

	new_theta = std::atan2(x, y);
	rot = (kp * new_theta); //+ (kd * inc_theta);

	if (std::abs(rot) >= 0.01   && std::abs(rot) <= 0.1) { //mucho mas?
		first_time = std::chrono::steady_clock::now(); //empezamos a contar
		return{STATE::CROSS_DOOR, 1000.0f, 0.0f};
	}
	else {
		return {STATE::ORIENT_TO_DOOR, 0.0f, rot};
	}

}


std::tuple<STATE, float, float> SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points) {
	auto elapsed = std::chrono::duration_cast<std::chrono::seconds>( //para ver cuánto tiempo ha pasado desde la primera llamada
				   std::chrono::steady_clock::now() - first_time)
				   .count();


	float v = elapsed < 2 ? 1000.0f : 0;
	STATE s;

	if (elapsed < 2)
		s = STATE::CROSS_DOOR;
	else
	{
		room = (room + 1) % rooms.size();
		draw_lidar2(&viewer_room->scene, room);
		s = STATE::GOTO_ROOM_CENTER;

	}

	return std::make_tuple(s, v, 0);
}

bool SpecificWorker::update_robot_pose(const Corners& corners, const Match& match)
{
	Eigen::MatrixXd W(match.size() * 2, 3);
	Eigen::VectorXd b(match.size() * 2);
	for (auto &&[i,m]: match | iter::enumerate )
	{
		auto &[meas_c, nom_c, _] = m;
		auto &[p_meas, __, ___] = meas_c;
		auto &[p_nom, ____, _____] = nom_c;

		b(2 * i)     = p_nom.x() - p_meas.x();

		b(2 * i + 1) = p_nom.y() - p_meas.y();
		W.block<1, 3>(2 * i, 0)     << 1.0, 0.0, -p_meas.y();
		W.block<1, 3>(2 * i + 1, 0) << 0.0, 1.0, p_meas.x();
	}

	// estimate new pose with pseudoinverse
	const Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;

	//qInfo() << "--------------------";

	if (r.array().isNaN().any()) {
		//qDebug()<<"No dibujo porque hay NaN";
		return {};
	}

	robot_pose.translate(Eigen::Vector2f(r(0), r(1)));
	robot_pose.rotate(r[2]);
	//qDebug()<<"Voy a dibujarlo en las coords: "<<robot_pose.translation().x()<<"/////"<<robot_pose.translation().y();
	robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
	const double angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
	robot_room_draw->setRotation(qRadiansToDegrees(angle));

	return false; //TODO: Cambiar mas adelante
}

















/**************************************/
void SpecificWorker::emergency()
{
	std::cout << "Emergency worker" << std::endl;
	//emergencyCODE
	//
	//if (SUCCESSFUL) //The componet is safe for continue
	//  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
	std::cout << "Restore worker" << std::endl;
	//restoreCODE
	//Restore emergency component

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}



// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData