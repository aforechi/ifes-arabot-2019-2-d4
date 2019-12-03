#pragma once
#include<string>
#include<fstream>
#include<vector>

class waypoints
{
public:
	waypoints();
	~waypoints();
	void load(std::string);
	void save(std::string);
private:
	std::vector<float> x;
	std::vector<float> y;
	std::vector<float> v;
};

waypoints::waypoints()
{
}

waypoints::~waypoints()
{
}

void waypoints::load(std::string) {

}

void waypoints::save(std::string) {

}
