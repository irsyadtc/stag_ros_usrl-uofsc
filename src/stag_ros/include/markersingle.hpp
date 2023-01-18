#ifndef MARKERSINGLE_HPP_
#define MARKERSINGLE_HPP_

#include <cmath>

#define NO_OF_MARKERS 1


class markersingle
{
	
private:
    struct markersdb
	{
		std::string name;
		double posx, posy, posz;
		double roll, pitch, yaw;
	};
    
    markersdb MarkersDb[NO_OF_MARKERS] {
	{"STag_tag_7", 1.4, 0.153, 1, -1*M_PI_2, 0, 0},
	};
	

public:
	markersingle();
	~markersingle();

	unsigned int getSize() { return NO_OF_MARKERS; }
	std::string getName(int a) { return this->MarkersDb[a].name; }
	double getPosx(unsigned int a) { return this->MarkersDb[a].posx; }
	double getPosy(unsigned int a)  { return this->MarkersDb[a].posy; }
	double getPosz(unsigned int a)  { return this->MarkersDb[a].posz; }
	double getRoll(unsigned int a)  { return this->MarkersDb[a].roll; }
	double getPitch(unsigned int a)  { return this->MarkersDb[a].pitch; }
	double getYaw(unsigned int a)  { return this->MarkersDb[a].yaw; }
	
};



#endif
