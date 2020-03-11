#pragma once 

enum class filter_type {COMPLEMENTARY};

class FilterSettings
{
	public:

		virtual filter_type getType() = 0;
};