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

const float MIN_TO_WALL = 760.0f; //Distancia minima que el robot tendra a una pared antes de que este empiece a girar
bool new_turn = false;
int turn_way = 1;
State last_state = State::SPIRAL;
float follow_wall_time = 50; //cuantos segundos está en follow wall antes de pasar a forward

//Usamos sintaxis de inicializacion de lista en el constructor para inicializar los valores aleatorios
SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx), gen(rd()), rand(600,2200), rand_turn_way(1, 2)
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

	/*
	auto left_begin = closest_lidar_index_to_given_angle(filter_data, -M_PI/2 -0.01); //params.LIDAR_FRONT_SECTION = -10
	auto left_end = closest_lidar_index_to_given_angle(filter_data, -M_PI/2  + 0.01); //params.LIDAR_FRONT_SECTION = +10
	auto left_min=std::min_element(filter_data.begin()+left_begin.value(), filter_data.begin()+left_end.value(), [](const auto& a, const auto& b){return a.distance2d<b.distance2d;});
	float left_threshold = left_end.value()-left_begin.value() < 10 ? 600 : 450;
	//auto view = std::ranges::subrange(filter_data.begin()+left_begin.value(), filter_data.begin()+left_end.value());
	qDebug()<<left_min->distance2d << "---" <<left_end.value() - left_begin.value() << left_threshold;
	*/
	/*
	auto front_begin = closest_lidar_index_to_given_angle(filter_data, -0.1);
	auto front_end = closest_lidar_index_to_given_angle(filter_data, 0.1);
	auto front_min = std::min_element(filter_data.begin()+front_begin.value(), filter_data.begin()+front_end.value(), [](const auto& a, const auto& b){return a.distance2d<b.distance2d;});
	qDebug()<<"Izq:"<<left_min->x<<"- "<<left_min->y<<" - "<<left_min->z<<" - "<<left_min->phi<<" - "<<left_min->theta;
	qDebug()<<"Fron:"<<front_min->x<<"- "<<front_min->y<<" - "<<front_min->z<<" - "<<front_min->phi<<" - "<<front_min->theta; //parece que pertenecen a misma pared cuando theta parecida
	*/


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
		p_filter = filter_isolated_points(data.points, 400);

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
	auto frente_begin = closest_lidar_index_to_given_angle(filter_data, -0.2); //pillo más puntos para no chocar
	auto frente_end = closest_lidar_index_to_given_angle(filter_data, 0.2);
	auto front_min=std::min_element(filter_data.begin()+frente_begin.value(), filter_data.begin()+frente_end.value(), [](const auto& a, const auto& b){return a.r<b.r;});
	if (front_min->distance2d<MIN_TO_WALL) {
		turn_way = rand_turn_way(gen) % 2 == 0 ? 1:-1;
		return{State::TURN_FORWARD, 0.0, (3.0f)*turn_way};
	}
	//con esto no se choca
	return {State::FORWARD, 10000.0, 0}; //por defecto sigo haciendo lo mismo


}

std::tuple<State, float, float> SpecificWorker::turn_forward_method(const RoboCompLidar3D::TPoints& filter_data) {
	auto frente_begin = closest_lidar_index_to_given_angle(filter_data, -0.2); //cojo más puntos para no chocarme
	auto frente_end = closest_lidar_index_to_given_angle(filter_data, 0.2); //cojo más puntos para no chocarme
	auto front_min=std::min_element(filter_data.begin()+frente_begin.value(), filter_data.begin()+frente_end.value(),
		[](const auto& a, const auto& b){return a.distance2d<b.distance2d;});

	last_state = State::TURN_FORWARD;
	float dist_threshold = rand(gen); //genero una distancia aleatoria
	if (front_min->distance2d > dist_threshold) {
		return {State::FORWARD, 3000.0, 0};
	}

	//Si no tengo margen suficiente sigo girando

	if(turn_way == 1)
		qDebug()<<"Giro a derecha";
	else
		qDebug()<<"Giro a izquierda";
	return {State::TURN_FORWARD, 0.0, (3.0f)*turn_way};



}

std::tuple<State, float, float> SpecificWorker::turn_follow_method(const RoboCompLidar3D::TPoints& filter_data) { //en un principio esta bien, hacer que cuando elapsed pase a forward
	static auto start_time = std::chrono::steady_clock::now();
	auto frente_begin = closest_lidar_index_to_given_angle(filter_data, -0.1); //params.LIDAR_FRONT_SECTION = -10
	auto frente_end = closest_lidar_index_to_given_angle(filter_data, 0.1); //params.LIDAR_FRONT_SECTION = +10
	auto front_min=std::min_element(filter_data.begin()+frente_begin.value(), filter_data.begin()+frente_end.value(),
		[](const auto& a, const auto& b){return a.distance2d<b.distance2d;});

	auto left_begin = closest_lidar_index_to_given_angle(filter_data, -M_PI/2 -0.01); //params.LIDAR_FRONT_SECTION = -10
	auto left_end = closest_lidar_index_to_given_angle(filter_data, -M_PI/2  + 0.01); //params.LIDAR_FRONT_SECTION = +10
	auto left_min=std::min_element(filter_data.begin()+left_begin.value(), filter_data.begin()+left_end.value(), [](const auto& a, const auto& b){return a.distance2d<b.distance2d;});

	auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
					   std::chrono::steady_clock::now() - start_time)
					   .count();
	float rot = left_min->distance2d < 420 ? 0.5:0; //ha de ser 0 sino es terrible -> sigue ocurriendo lo de stuckearse en un muro (deberia funcionar como esta, pero produce lo de la ultima captura de pantalla)
	//probar a poner a 1 en vez de a 1.5
	if (front_min -> distance2d > MIN_TO_WALL) { //MIN_TO_WALL -> 600 (asi funciona decente) //TODO APLICAR ESTE CAMBIO
		return {State::FOLLOW_WALL, 1500.0, rot}; //si la distancia por la izquierda es tambien pequeña, gira a la derecha un poco
	}

	State s = elapsed <= 45 ? State::TURN_FOLLOW : State::TURN_FORWARD; //igual le tengo que dar más tiempo?
	qDebug()<<"ultimo return";
	return {s, 0.0, 2.0f}; //con 1 es lento y con 3 demasiado rapido

}

std::tuple<State, float, float> SpecificWorker::follow_wall_method(const RoboCompLidar3D::TPoints& filter_data) {
	//TODO: MEJORAR PARA EVITAR CICLOS EN OBSTACULOS
	static auto start_time = std::chrono::steady_clock::now();

	auto left_begin = closest_lidar_index_to_given_angle(filter_data, -M_PI/2 -0.01); //params.LIDAR_FRONT_SECTION = -10
	auto left_end = closest_lidar_index_to_given_angle(filter_data, -M_PI/2  + 0.01); //params.LIDAR_FRONT_SECTION = +10
	auto left_min=std::min_element(filter_data.begin()+left_begin.value(), filter_data.begin()+left_end.value(), [](const auto& a, const auto& b){return a.distance2d<b.distance2d;});

	auto front_begin = closest_lidar_index_to_given_angle(filter_data, -0.1);
	auto front_end = closest_lidar_index_to_given_angle(filter_data, 0.1);
	auto front_min = std::min_element(filter_data.begin()+front_begin.value(), filter_data.begin()+front_end.value(), [](const auto& a, const auto& b){return a.distance2d<b.distance2d;});
	//qDebug()<<front_begin.value()<<"----"<<front_end.value();
	const float min_dist = MIN_TO_WALL;     // umbral de seguridad frontal (va aumentando para recorrer más)
	static float max_extra_dist= 400.0f; //maximo que puede aumentar la distancia a pared //nose si era 240 o 340 el 7

	auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
					   std::chrono::steady_clock::now() - start_time)
					   .count();
	float period = 5.0f; //cada cuantos s hace un ciclo completo
	float max_dist = 3000.0f;
	float extra = std::clamp(max_extra_dist * ::sinf(M_PI*2*elapsed / period), 20.0f, 400.0f); //A pesar de que sinf devuelve algo en el rango
	//[-1, 1], es necesario hacer el clamp porque en ocasiones sumada (o restaba) demasiado extra debido a problemas de desbordamiento, porque la funcion sin (antes de poner sinf)
	//devuelve un double, y al tener tanta precision e intentar meterla en un float, desbordaba el signo y generaba valores muy grandes. Para evitar eso, he decidido usar std::clamp
	//para limitar el valor y sinf para que devuelva la misma precisión y así no tener problemas de desbordamiento de signos
	float desired_dist = min_dist + extra;
	//bool esquina = front_min->distance2d < 1500 && left_min->distance2d < desired_dist;
	//qDebug() <<front_min->x << "-" << front_min->y << "////"<<left_min->x<<","<<left_min->y;

	Eigen::Vector2f P_left(left_min->x, left_min->y);
	Eigen::Vector2f P_front(front_min->x, front_min->y);
	float dist_corner = (P_left - P_front).norm();
	bool esquina = (dist_corner < 2000) && !((std::abs(left_min->z - front_min->z) < 120));
	last_state=State::FOLLOW_WALL;
	if (!(front_min->distance2d < MIN_TO_WALL) && !(left_min->distance2d < 600) && dist_corner < 2000) { //MIN_TO_WALL -> 650 (asi funciona decente)
		return {State::FOLLOW_WALL, 1500.0, -1.5f}; //sigo palante pero girando para cubrir la esquina
	}
	float left_threshold = left_end.value()-left_begin.value() < 10 ? 600 : 450; //esto no se si es lo propio
	if (left_min->distance2d > left_threshold && !(front_min->distance2d < MIN_TO_WALL)) { //si es menor o igual giro hacia la derecha
		return {State::TURN_FOLLOW, 1500.0f, -0.75f};//antes -0.75
	}
	qDebug()<<"sigo devolviendo el otro return----"<<left_min->distance2d<<"//////"<<left_threshold<<"////////"<<front_min->distance2d; //todo: ultima captura de pantalla relativa a estos datos y alchoque, arreglar
	return {State::TURN_FOLLOW, 1500.0, 0.5f}; //giro suave para no tener que volver a corregir la trayectoria pronto
}

std::tuple<State, float, float> SpecificWorker::spiral_method(const RoboCompLidar3D::TPoints& filter_data) {
	//FALLA EN NO SALIR BIEN LA ESPIRAL (TIENE QUE ACABAR POR ARRIBA) //TODO QUIZA METERLE -5 A LA B
	const float min_distance = 600.0f;  // mm (detección obstáculo) //mejor usar MIN_TO_WALL
	const float a = 50.0f;               // mm (radio inicial)
	const float max_r = 5000.0f;        // mm (radio máximo)
	const float dt = 0.01f;             // periodo de compute -> 1/100Hz

	// --- Medir obstáculo frontal ---
	auto front_begin = closest_lidar_index_to_given_angle(filter_data, -0.1);
	auto front_end = closest_lidar_index_to_given_angle(filter_data, 0.1);
	auto front_min = std::min_element(filter_data.begin()+front_begin.value(), filter_data.begin()+front_end.value(), [](const auto& a, const auto& b){return a.distance2d<b.distance2d;});

	if(front_min->distance2d < MIN_TO_WALL)
	{
		static float theta = 0.0f;
		theta = 0.0f;  // reinicia la espiral
		return {State::TURN_FOLLOW, 0.0f, 3.0f};
	}

	// --- Variables persistentes ---
	static float theta = 0.001f;  // rad
	static float r = a;         // mm
	static float v = 1150.0f;        // mm/s (velocidad lineal)
	static float b = 305.0f;              // mm/rad (separación entre vueltas) //bueno 250


	r = a + b * theta;

	if(r > 1000.0f){
		qDebug() << r;
		v = v + 0.85f;
		b = 355.0f; //bueno 300
	}
	float w = v / std::sqrt(b*b + r*r);  // rad/s


	theta += w * dt;

	// --- Fin de la espiral ---
	if(r >= max_r)
	{
		theta = 0.0f;
		return {State::TURN_FOLLOW, 0.0f, 3.0f};
	}

	// --- Resultado ---
	return {State::SPIRAL, v, w};

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

