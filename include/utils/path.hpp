#pragma once

#include <list>
#include "point.hpp"

namespace Pronounce {
    class Path {
	private:
		std::list<Point> path;
	public:
		Path(/* args */);
		~Path();
	};
	
	Path::Path(/* args */)
	{
	}
	
	Path::~Path()
	{
	}
	
} // namespace Pronounce
