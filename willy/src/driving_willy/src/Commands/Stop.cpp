#include "../include.h"

using namespace std;

void Stop::Execute()
{
	std::cout << "Stopping\n";
}

Stop::Stop(int Time) 
{
	_time = Time;
}	

std::string Stop::toString() {
	return "Stop";
}