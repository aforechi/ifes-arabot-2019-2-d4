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
	return sqrt( (b.position[0] - a.position[0])*(b.position[0] - a.position[0])+
		(b.position[1] - a.position[1])*(b.position[1] - a.position[1]));
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

	std::cout << "Pressione Enter para puxar o freio de mão." << std::endl; std::cin.get();
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
	return (pose.position[1] < 1 && pose.position[1]>0) && (pose.position[0] > -5 && pose.position[0] < 5);
}


void manual(msr::airlib::CarRpcLibClient &simulador)
{
	ofstream waypointS;

	waypointS.open("waypoint.csv");

	bool completouAvolta = false;

	auto poseAnterior = simulador.getCarState().kinematics_estimated.pose;

	while (!completouAvolta) {
		auto car_state = simulador.getCarState();
		auto poseAtual = car_state.kinematics_estimated.pose;
		auto velocidade = car_state.speed;
		
		if(distancia(poseAnterior, poseAtual) > 1){
			 saveCarPose(waypointS, poseAtual, velocidade);
			 poseAnterior = poseAtual;
		}

		if (chegada(poseAtual)) {
			completouAvolta = true;
		}
	}

	waypointS.close();
	
}

void automatico()
{
	
	ifstream waypointE;

	waypointE.open("waypoint.csv");

	while (!waypointE.eof()) {

		//string linha, txt;
		//arq.eof()
		//getline(arq, linha);
		//istringstream valores(linha)

		//cout << x << " " << y << "" << v << std::endl;
	}
	waypointE.close();


}

int main()
{
	std::cout << "Verifique se o arquivo Documentos\\AirSim\\settings.json " <<
		"está configurado para simulador de carros \"SimMode\"=\"Car\". " <<
		"Pressione Enter para continuar." << std::endl;
	std::cin.get();

	msr::airlib::CarRpcLibClient client;
	try {
		client.confirmConnection();
		client.reset();

		cout << "Favor digite a opção de controle:\n";
		cout << "Opção 0-Manual\n Opção1-Automatico";

		int opcao;
		cin >> opcao;

		switch (opcao) {
		case 0:
			manual(client);
		break;

		case 1:
			automatico();
		break;
		}
	}

		
	catch (rpc::rpc_error&  e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Verifique a exceção lançada pela API do AirSim." << std::endl << msg << std::endl; std::cin.get();
	}

	return 0;
}
