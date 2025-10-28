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
//Ahora mismo esta OK pero se sigue chocando un pelin (muy poco, mas bien roce) //TODO: Mirarlo mañana con obstaculos
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
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

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
    std::cout << "initialize worker" << std::endl;

    //initializeCODE


	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	this->resize(900,450);
	viewer->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);
	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);


    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

}



void SpecificWorker::compute()
{

	RoboCompLidar3D::TPoints filter_data = filtro_datos();
	std::tuple<float, float> velocidades = update_robot_state(filter_data);
	try {
		omnirobot_proxy->setSpeedBase(0.0, std::get<0>(velocidades), std::get<1>(velocidades)); //le hago el setSpeedBase
	}catch (const Ice::Exception &e){std::cout<<e.what()<<std::endl; return;}


	//cuando la diferencia de z es menos de 120
}
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



void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints& filtered_points, QGraphicsScene *scene)
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

    // compute and draw minimum distance point in frontal range
    auto offset_begin = closest_lidar_index_to_given_angle(filtered_points, -0.05); //params.LIDAR_FRONT_SECTION = -10
    auto offset_end = closest_lidar_index_to_given_angle(filtered_points, 0.05); //params.LIDAR_FRONT_SECTION = +10

	if(not offset_begin or not offset_end)
	{ std::cout << offset_begin.error() << " " << offset_end.error() << std::endl; return ;}    // abandon the ship

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
                     robot_polygon->mapToScene(left_line_length * sin(angle1), left_line_length * cos(angle1))};
    QLineF line_right{QPointF(0.f, 0.f),
                      robot_polygon->mapToScene(right_line_length * sin(angle2), right_line_length * cos(angle2))};
    QPen left_pen(Qt::blue, 10); // Blue color pen with thickness 3
    QPen right_pen(Qt::red, 10); // Blue color pen with thickness 3
    auto line1 = scene->addLine(line_left, left_pen);
    auto line2 = scene->addLine(line_right, right_pen);
    items.push_back(line1);
    items.push_back(line2);
}



void SpecificWorker::new_target_slot(QPointF p)
{
	std::cout << "Nuevo target recibido en: ("
			  << p.x() << ", " << p.y() << ")" << std::endl;
}
RoboCompLidar3D::TPoints SpecificWorker::filtro_datos() {

	std::optional<RoboCompLidar3D::TPoints> filter_data;
	RoboCompLidar3D::TPoints  p_filter;
	try {
		auto data = lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1); //para mayor precision (puedo comparar ejemplos de ejecucion entre este y 0.1f round en la docu)
		//qInfo() << "Size: "<<data.points.size();
		if (data.points.empty()){qDebug()<<"No points"; return p_filter;}

		/*
		std::ranges::copy_if(data.points, std::back_inserter(p_filter),
												   [](auto  &a){ return a.z < 500 and a.distance2d > 200;});
		*/

		//Esto siguiente es opcional
		p_filter = filter_isolated_points(data.points, 100);

		if (!p_filter.empty()) {
			draw_lidar(p_filter, &viewer->scene);
		}
		else {
			return p_filter; //nos aseguramos de que vamos a llamar a update_robot_state() con valores validos
		}

	}catch (const Ice::Exception &e){ std::cout<<e.what()<<std::endl; return p_filter;}

	return p_filter;
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


std::tuple<float, float> SpecificWorker::update_robot_state(const RoboCompLidar3D::TPoints& points) {
	std::tuple<State, float, float> result;
	switch (this->state) {
		default:
			break;
		case State::FORWARD:
			result=forward_method(points);
			break;
		case State::TURN_FOLLOW:
			result=turn_follow_method(points);
			break;
		case State::TURN_FORWARD:
			result=turn_forward_method(points);
			break;
		case State::SPIRAL:
			result=spiral_method(points);
			break;
		case State::FOLLOW_WALL:
			result=follow_wall_method(points);
			break;
	}
	this->state=std::get<State>(result);
	return {std::get<1> (result), std::get<2>(result)};
}


std::tuple<State, float, float> SpecificWorker::forward_method(const RoboCompLidar3D::TPoints& filter_data) {
	//Obtenemos el índice de filter data correspondiente a la pared más cercana por el frente del robot
	auto frente_begin = closest_lidar_index_to_given_angle(filter_data, -0.2);
	auto frente_end = closest_lidar_index_to_given_angle(filter_data, 0.2);
	auto front_min=std::min_element(filter_data.begin()+frente_begin.value(), filter_data.begin()+frente_end.value(), [](const auto& a, const auto& b){return a.r<b.r;});

	//Obtenemos el índice de filter data correspondiente a la pared más cercana por la izquierda del robot
	auto left_begin = closest_lidar_index_to_given_angle(filter_data, -M_PI/2 -0.01); //params.LIDAR_FRONT_SECTION = -10
	auto left_end = closest_lidar_index_to_given_angle(filter_data, -M_PI/2  + 0.01); //params.LIDAR_FRONT_SECTION = +10
	auto left_min=std::min_element(filter_data.begin()+left_begin.value(), filter_data.begin()+left_end.value(), [](const auto& a, const auto& b){return a.distance2d<b.distance2d;});

	//Obtenemos el índice de filter data correspondiente a la pared más cercana por la derecha del robot
	auto right_begin = closest_lidar_index_to_given_angle(filter_data, M_PI/2 -0.01); //params.LIDAR_FRONT_SECTION = -10
	auto right_end = closest_lidar_index_to_given_angle(filter_data, M_PI/2  + 0.01); //params.LIDAR_FRONT_SECTION = +10
	auto right_min=std::min_element(filter_data.begin()+right_begin.value(), filter_data.begin()+right_end.value(), [](const auto& a, const auto& b){return a.distance2d<b.distance2d;});

	//Decidimos el sentido del giro dependiendo de qué pared tengo más cerca. P ej, si tengo una pared cerca a la derecha, si giro hacia allí tendré que volver a girar pronto, así que giro a la izquierda.
	turn_way = right_min-> distance2d < left_min->distance2d ? -1.0f: 1.0f; //almaceno el sentido del giro en una variable global para que TURN_FORWARD siga haciendo lo mismo
	if (front_min->distance2d<MIN_TO_WALL_FORWARD) {
		return{State::TURN_FORWARD, 0.0, turn_way};
	}
	return {State::FORWARD, 1000.0, 0}; //por defecto sigo hacia adelante
}

std::tuple<State, float, float> SpecificWorker::turn_forward_method(const RoboCompLidar3D::TPoints& filter_data) {
	//Obtenemos el mínimo por delante del robot
	auto frente_begin = closest_lidar_index_to_given_angle(filter_data, -0.2);
	auto frente_end = closest_lidar_index_to_given_angle(filter_data, 0.2);
	auto front_min=std::min_element(filter_data.begin()+frente_begin.value(), filter_data.begin()+frente_end.value(),
		[](const auto& a, const auto& b){return a.distance2d<b.distance2d;});

	//Generamos el margen de giro de forma aleatoria, de ese modo al girar para evitar chocarse, unas veces se abrirá más y otras menos, lo que hará que haga recorridos más variados
	float dist_threshold = rand(gen); //genero una distancia de apertura aleatoria, para generar recorridos más diversos
	if (front_min->distance2d > dist_threshold) { //el problema es este if
		return {State::FORWARD, 1000.0, 0};
	}
	//Si no tengo margen suficiente sigo girando
	return {State::TURN_FORWARD, 0.0, turn_way};
}

std::tuple<State, float, float> SpecificWorker::turn_follow_method(const RoboCompLidar3D::TPoints& filter_data) { //en un principio esta bien, hacer que cuando elapsed pase a forward
	//Miro el frente del robot
	auto frente_begin = closest_lidar_index_to_given_angle(filter_data, -0.2);
	auto frente_end = closest_lidar_index_to_given_angle(filter_data, 0.2);
	auto front_min=std::min_element(filter_data.begin()+frente_begin.value(), filter_data.begin()+frente_end.value(),
		[](const auto& a, const auto& b){return a.distance2d<b.distance2d;});

	//Miro a su izquierda
	auto left_begin = closest_lidar_index_to_given_angle(filter_data, -M_PI/2 -0.02); //params.LIDAR_FRONT_SECTION = -10
	auto left_end = closest_lidar_index_to_given_angle(filter_data, -M_PI/2  + 0.02); //params.LIDAR_FRONT_SECTION = +10
	auto left_min=std::min_element(filter_data.begin()+left_begin.value(), filter_data.begin()+left_end.value(), [](const auto& a, const auto& b){return a.distance2d<b.distance2d;});

	float left_threshold = left_end.value() - left_begin.value() < 5 ? 450 : 350;
	float diff = left_min -> distance2d - left_threshold;
	float rotacion = diff < 15 ? 0.08 : -0.2;

	if (front_min -> distance2d > MIN_TO_WALL_FOLLOW-90) { //
		return {State::FOLLOW_WALL, 1000.0, rotacion}; //es poquisimo xd
	}
	return {State::TURN_FOLLOW, 0.0, 1.0f}; //por defecto sigo girando
}

std::tuple<State, float, float> SpecificWorker::follow_wall_method(const RoboCompLidar3D::TPoints& filter_data) {
	//Empiezo a medir el tiempo desde que se llama al método por primera vez
	static auto start_time = std::chrono::steady_clock::now();
	auto left_begin = closest_lidar_index_to_given_angle(filter_data, -M_PI/2 -0.02); //params.LIDAR_FRONT_SECTION = -10
	auto left_end = closest_lidar_index_to_given_angle(filter_data, -M_PI/2  + 0.02); //params.LIDAR_FRONT_SECTION = +10
	auto left_min=std::min_element(filter_data.begin()+left_begin.value(), filter_data.begin()+left_end.value(), [](const auto& a, const auto& b){return a.distance2d<b.distance2d;});

	auto front_begin = closest_lidar_index_to_given_angle(filter_data, -0.1);
	auto front_end = closest_lidar_index_to_given_angle(filter_data, 0.1);
	auto front_min = std::min_element(filter_data.begin()+front_begin.value(), filter_data.begin()+front_end.value(), [](const auto& a, const auto& b){return a.distance2d<b.distance2d;});


	auto elapsed = std::chrono::duration_cast<std::chrono::seconds>( //para ver cuánto tiempo ha pasado desde la primera llamada a follow_wall_method()
				   std::chrono::steady_clock::now() - start_time)
				   .count();

	State which_turn = elapsed <= 38 ? State::TURN_FOLLOW : State::TURN_FORWARD; //Si ha pasado el tiempo, pasaré a TURN_FORWARD

	if (front_min->distance2d < MIN_TO_WALL_FOLLOW) { //Para no chocarme de frente
		return {which_turn, 0.0f, 1.0f};//para no chocarme en el follow wall
	}
	float left_threshold = left_end.value() - left_begin.value() < 5 ? 450 : 350; //Hay veces que si está muy pegado a la pared, coge menos puntos porque no llega a leer bien la pared.
	//Si eso ocurre, es porque estoy cerca la pared así que aumento ligeramente el umbral para no chocar
	float diff = left_min->distance2d - left_threshold;// calcularé el giro en función de esta variable

	if (diff >= 15.0 && diff< 70.0) { //esta es la zona muerta -> sigo hacia delante en línea recta
		return {State::FOLLOW_WALL, 1000.0, 0.0f}; //Si no necesito hacer giros puedo seguir en FOLLOW_WALL
	}
	float signo = diff < 15 ? 0.08 : -0.20;
	return {which_turn, 1000.0, signo}; //En caso de no estar en zona muerta, calculo qué giro debo hacer para no chocar por la izquierda y lo hago
}

std::tuple<State, float, float> SpecificWorker::spiral_method(const RoboCompLidar3D::TPoints& filter_data) {
	//Miro el frente del robot para no chocarme
	auto front_begin = closest_lidar_index_to_given_angle(filter_data, -0.1);
	auto front_end = closest_lidar_index_to_given_angle(filter_data, 0.1);
	auto front_min = std::min_element(filter_data.begin()+front_begin.value(), filter_data.begin()+front_end.value(), [](const auto& a, const auto& b){return a.distance2d<b.distance2d;});

	//Variables de control de la espiral.
	static float down = 1.0f;
	static float up = 0.f;
	const float inc_up = 5.0f; //antes 5
	const float dec_down = 0.001f; //antes 0.001

	if (front_min->distance2d < MIN_TO_WALL_FORWARD)
	{
		return {State::FOLLOW_WALL, 1000.0f, 0}; //al acabar la espiral entramos del tirón en FOLLOW_WALL
	}
	else
	{
		down -=dec_down;
		up +=inc_up;
		down = std::clamp(down, 0.f, 1.f); //a pesar de los cambios en las variables, no nos pasamos de los límites
		up = std::clamp(up, 0.f, 1000.0f);
		return {State::SPIRAL, up, down}; //seguimos en espiral en caso de que no
	}

}
/**************************************/
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

