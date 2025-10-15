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
//Usamos sintaxis de inicializacion de lista en el constructor para inicializar los valores aleatorios
SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx), gen(rd()), rand(0,1)
{
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

	RoboCompLidar3D::TPoints filtrados = filtro_datos();
	/*
	auto view = filtrados | std::ranges::view(filtrados.size()/2-20, filtrados.size()/2+20);
	for (const auto& p: filtrados) {
		qDebug()<<p.r;
	}
	*/
	std::tuple<float, float> velocidades = update_robot_state(filtrados);
	try {
		omnirobot_proxy->setSpeedBase(0.0, std::get<0>(velocidades), std::get<1>(velocidades)); //le hago el setSpeedBase
	}catch (const Ice::Exception &e){std::cout<<e.what()<<std::endl; return;}

}

RoboCompLidar3D::TPoints SpecificWorker::filtro_datos() {

	std::optional<RoboCompLidar3D::TPoints> filter_data;
	try {
		auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1); //para mayor precision (puedo comparar ejemplos de ejecucion entre este y 0.1f round en la docu)
		//qInfo() << "Size: "<<data.points.size();
		if (data.points.empty()){qWarning()<<"No points"; return filter_data.value();}
		filter_data=filter_min_distance_cppitertools(data.points);
		auto data2=filter_data.value();
		if (filter_data.has_value())
			draw_lidar(filter_data.value(), &viewer->scene);
		else
			return filter_data.value(); //nos aseguramos de que vamos a llamar a update_robot_state() con valores validos

	}catch (const Ice::Exception &e){ std::cout<<e.what()<<std::endl; return filter_data.value();}

	return filter_data.value();
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
void SpecificWorker::draw_lidar(const std::vector<RoboCompLidar3D::TPoint>& points, QGraphicsScene* scene)
{
	static std::vector<QGraphicsItem*> draw_points;
	for (const auto &p : draw_points)
	{
		scene->removeItem(p);
		delete p;
	}
	draw_points.clear();

	const QColor color("LightGreen");
	const QPen pen(color, 10);
	//const QBrush brush(color, Qt::SolidPattern);
	for (const auto &p : points)
	{
		const auto dp = scene->addRect(-25, -25, 50, 50, pen);
		dp->setPos(p.x, p.y);
		draw_points.push_back(dp);   // add to the list of points to be deleted next time
	}
}

void SpecificWorker::new_target_slot(QPointF p)
{
	std::cout << "Nuevo target recibido en: ("
			  << p.x() << ", " << p.y() << ")" << std::endl;
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
		case State::TURN:
			result=turn_method(points);
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
	std::size_t start= filter_data.size()/2 - 20;
	std::size_t end   = filter_data.size()/2 + 20; //necesario porque en trayectorias casi paralelas a la pared va rozando

	float min_threshold=980;
	auto front_min=std::min_element(filter_data.begin()+start, filter_data.begin()+end, [](const auto& a, const auto& b){return a.r<b.r;});
	if (front_min->r<min_threshold) {
		//qDebug()<<"Cambio a giro";
		return{State::TURN, 0.0, 2};
	}
	else{
		return {State::FORWARD, 10000.0, 0}; //por defecto sigo haciendo lo mismo
	}

}

std::tuple<State, float, float> SpecificWorker::turn_method(const RoboCompLidar3D::TPoints& filter_data) {
	std::size_t start= filter_data.size()/2 - 20;
	std::size_t end   = filter_data.size()/2 + 20; //necesario porque en trayectorias casi paralelas a la pared va rozando
	static auto start_time = std::chrono::steady_clock::now();
	State possible_states[2] = {State::FORWARD, State::FOLLOW_WALL};
	int aux;
	float min_threshold=950;
	auto front_min=std::min_element(filter_data.begin()+start, filter_data.begin()+end, [](const auto& a, const auto& b){return a.r<b.r;});

	auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
					   std::chrono::steady_clock::now() - start_time)
					   .count();

	if (front_min->r > 80+min_threshold) { //tengo margen para avanzar -> tengo que hacerlo para que quede paralelo
		//qDebug()<<"Cambio a avance";
		aux = rand(gen);
		//qDebug()<<aux;
		//return {possible_states[rand(gen)], 10000.0, 0.0};

		if (elapsed <= 90 ) { //por probar, con 90 me sale un 7,45
			//qDebug()<<"Como ha pasado menos de 1 min y medio, sigo a la pared";
			return {State::FOLLOW_WALL, 10000.0, 0.0};
		}
		else {
			//qDebug()<<"Como ha pasado más de un min y medio, voy hacia delante";
			return {State::FORWARD, 10000.0, 0};
		}
	}
	else {
		return{State::TURN, 300.0, 2.0}; //por defecto sigo haciendo lo mismo
	}
}

std::tuple<State, float, float> SpecificWorker::follow_wall_method(const RoboCompLidar3D::TPoints& filter_data) {
	static auto start_time = std::chrono::steady_clock::now();

	std::size_t left_start= filter_data.size()/4-15; //test para ver si asi se choca (no deberia)
	std::size_t left_end   = filter_data.size()/4 + 15;
	auto left_view=std::ranges::subrange(filter_data.begin()+ left_start, filter_data.begin()+left_end); //si son los de la izquierda
	auto left_min=std::min_element(filter_data.begin()+left_start, filter_data.begin()+left_end, [](const auto& a, const auto& b){return a.r<b.r;});

	const float min_dist = 890.0f;     // umbral de seguridad frontal (va aumentando para recorrer más)
	static float max_extra_dist= 240.0f; //maximo que puede aumentar la distancia a pared //nose si era 240 o 340 el 7
	const float Kp_rot = 0.003f;             // ganancia proporcional de giro
	const float velZ = 4000.0f;              // avance constante
	const float rot_max=2.0f;


	auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
					   std::chrono::steady_clock::now() - start_time)
					   .count();
	float period = 15.0f; //cada cuantos s hace un ciclo completo

	float desired_dist = min_dist + (max_extra_dist * std::sin(M_PI*2*elapsed / period));
	float rot = -Kp_rot * std::abs((min_dist - left_min->r));

	//qDebug()<<rot;
	if (left_min->r > desired_dist) { //perdio la pared
		return {State::TURN, 300.0, rot};
	}
	if (left_min->r < desired_dist)             // se está chocando
		return {State::TURN, 300.0f, 2.0f};

	// --- Comportamiento normal: seguir pared con zig-zag ---
	return {State::FOLLOW_WALL, velZ, rot};



	/*
	if (left_min->r<desired_dist) { //min_dist
		return{State::TURN, 300.0, 0.75}; //me alejo de la pared girando
	}
	if (left_min->r > desired_dist) { //min_dist + 80
		rot = -Kp_rot * std::abs((min_dist - left_min->r)); //en vez de min_front_dist, ponia desired dist
		return{State::TURN, 300.0, rot}; //me acerco a la pared
	}
	//TODO ACABAR

	// Devuelve estado por defecto
	return {State::FOLLOW_WALL, velZ, rot};
	*/

}

std::tuple<State, float, float> SpecificWorker::spiral_method(const RoboCompLidar3D::TPoints& filter_data) {
	//TODO
	std::size_t start= filter_data.size()/2 - 20;
	std::size_t end   = filter_data.size()/2 + 20; //necesario porque en trayectorias casi paralelas a la pared va rozando


	float min_threshold=580;
	auto front_min=std::min_element(filter_data.begin()+start, filter_data.begin()+end, [](const auto& a, const auto& b){return a.r<b.r;});
	// variables estáticas o de clase para mantener el estado entre llamadas
	static float v = 200.0f;   // velocidad lineal inicial
	static float w = 1.5f;   // velocidad angular inicial (rad/s)

	// incrementos/decrementos
	const float dv = 10.0f;  // cuánto aumenta la velocidad lineal por ciclo
	const float dw = 0.005f;  // cuánto disminuye la velocidad angular por ciclo

	// actualizar velocidades
	v += dv;
	w -= dw;

	// evitar que w se vuelva negativa o cero
	if (w < 0.25f) w = 1.0f;

	if (front_min->r<min_threshold) {
		return{State::TURN, 0.0, 1.5};
	}else {
		return{State::SPIRAL, v,w};
	}





	/*
			qDebug()<<"Estoy en espiral";
			if (front_min->r<min_threshold) {
				qDebug()<<"Cambio a giro";
				omnirobot_proxy->setSpeedBase(0.0, 0.0, 1.5);
				state = State::TURN;
			}
			else {
				inc_velZ++; dec_velRot-=0.1;
				omnirobot_proxy->setSpeedBase(0.0, inc_velZ, dec_velRot);
			}
			*/
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

