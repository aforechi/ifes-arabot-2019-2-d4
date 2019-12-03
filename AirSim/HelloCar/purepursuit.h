#pragma once


class purepursuit {

	purepursuit(float, float, float, float);
	float update(float);
	void finished();


private:
	float baseline;
	float steermax;
	float gain;
	float lookahead_distance;
	float steering;
	float velocity;
	int waypoint[];


};
purepursuit::purepursuit(float baseline, float steermax, float gain, float lokkhead_distance) //construtor
{

	this->baseline = baseline;
	this->steermax = steermax;
	this->gain = gain;
	this->lookahead_distance = lookahead_distance;
	float steering(0);
	float velocity(0);
	//int waypoint[];

}
float purepursuit::update(float x) //1ºfunção-membro
{




}
void purepursuit::finished() //2ºfunção-membro
{


}