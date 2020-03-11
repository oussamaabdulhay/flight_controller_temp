#pragma once 

#include "FilterSettings.hpp"
#include <cmath>

class DataFilter
{	
public:

	virtual void resetFilter(void) = 0;
	virtual void setFilterSettings(FilterSettings*) = 0;
	virtual float getFilteredData(float) = 0;
	virtual float getFilteredData(float, float) = 0;
};