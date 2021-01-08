#ifndef LIB_H
#define LIB_H
#include <iostream>
#include <string>
#include <ctime>

namespace Jeffsan{

class Timer
{
public:
	Timer()
	{
		tic();
	}
	double end()
	{
		duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
		start = std::clock();
		return duration;
	}
	void tic()
	{
		start = std::clock();
	}
	void toc(std::string section = " ")
	{
		end();
		printf("%s T: %3.5f FPS: %.2fHz \r\n", section.c_str(), duration, 1/duration);
	}
private:
	std::clock_t start;
	double duration;
};

}
#endif 