// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <fstream>
#include<sstream>
#include "main.h"

using namespace std;

using namespace msr::airlib;

using msr::airlib::Pose;


float distancia(const Pose &a, const Pose &b) {


	return sqrt((pow((b.position[0] - a.position[0]),2)+ pow((b.position[1] - a.position[1]),2)));
}

void printCarPose(const msr::airlib::Pose &pose, float speed)
{
	std::cout << "x=" << pose.position[0] << " y=" << pose.position[1] << " v=" << speed << std::endl;

}

void saveCarPose(ofstream &Valores,const msr::airlib::Pose &pose, float speed)
{
	Valores << pose.position[0] << ";" << pose.position[1] << ";" << speed << std::endl;

}

void moveForwardAndBackward(msr::airlib::CarRpcLibClient &client)
{
	client.enableApiControl(true);
	CarApiBase::CarControls controls;

	std::cout << "Pressione Enter andar pra frente." << std::endl; std::cin.get();
	controls.throttle = 0.5f;
	controls.steering = 0.0f;
	client.setCarControls(controls);

	std::cout << "Pressione Enter para puxar o freio de m�o." << std::endl; std::cin.get();
	controls.handbrake = true;
	client.setCarControls(controls);

	std::cout << "Pressione Enter para fazer uma manobra." << std::endl; std::cin.get();
	controls.handbrake = false;
	controls.throttle = -0.5;
	controls.steering = 1;
	controls.is_manual_gear = true;
	controls.manual_gear = -1;
	client.setCarControls(controls);

	std::cout << "Pressione Enter para parar." << std::endl; std::cin.get();
	client.setCarControls(CarApiBase::CarControls());
}

bool chegada(const msr::airlib::Pose &pose) {
	if (pose.position[0] > -5 && pose.position[0] <5) {
		if (pose.position[1] > 0.1 && pose.position[1] < 1.2) {
			return true;
		}	
	}
	return false;
}


void manual1(msr::airlib::CarRpcLibClient &simulador)
{
	ofstream waypointS1;

	waypointS1.open("waypoint_1m.txt");

	bool completouAvolta = false;

	auto poseAnterior = simulador.getCarState().kinematics_estimated.pose;

	while (!completouAvolta) {
		auto car_state = simulador.getCarState();
		auto poseAtual = car_state.kinematics_estimated.pose;
		auto velocidade = car_state.speed;
		
		if(distancia(poseAnterior, poseAtual) >1){
			 saveCarPose(waypointS1, poseAtual, velocidade);
			 poseAnterior = poseAtual;
		}

		if (chegada(poseAtual)) {
			completouAvolta = true;
		}
	}

	waypointS1.close();
	
}
void manual5(msr::airlib::CarRpcLibClient &simulador)
{
	ofstream waypointS5;

	waypointS5.open("waypoint_5m.txt");

	bool completouAvolta = false;

	auto poseAnterior = simulador.getCarState().kinematics_estimated.pose;

	while (!completouAvolta) {
		auto car_state = simulador.getCarState();
		auto poseAtual = car_state.kinematics_estimated.pose;
		auto velocidade = car_state.speed;

		if (distancia(poseAnterior, poseAtual) >5) {
			saveCarPose(waypointS5, poseAtual, velocidade);
			poseAnterior = poseAtual;
		}

		if (chegada(poseAtual)) {
			completouAvolta = true;
		}
	}

	waypointS5.close();

}

void automatico(msr::airlib::CarRpcLibClient &simulador)
{
	
	ifstream waypointE;

	waypointE.open("waypoint_1m.csv");

	while (!waypointE.eof()) {

		string linha, coluna;
		std::getline(waypointE, linha);
		std::istringstream colunas(linha);

		while (!colunas.eof()) {
			std::getline(colunas, coluna, ',');
			std::cout << coluna << std::endl;

		}
		
	}
	
	waypointE.close();

}

int main()
{
	
	msr::airlib::CarRpcLibClient client;

	try {
		client.confirmConnection();
		client.reset();

		std::cout << "Favor digite a opcao de controle:\n";
		std::cout << "Opcao 1-Manual\nOpcao 2-Automatico\n";

		int opcao;
		std::cin >> opcao;

		switch (opcao) 
		{
		case 1:
			std::cout << "Opcao 1 escolhida. " << std::endl;
			manual1(client);
		break;

		case 2:
			automatico(client);
		break;

		}
	}

		
	catch (rpc::rpc_error& e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Verifique a exce��o lan�ada pela API do AirSim." << std::endl << msg << std::endl; std::cin.get();
	}

	return 0;
}
