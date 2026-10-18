#pragma once
#include "../WickedEngine/wiCSG.h"
#include <cmath>

// Grid snapping for CSG brush, vertex and face drags. Kept free of editor state so it can be tested directly.
namespace CSGSnap
{
	// Snaps the grabbed world point (start + move) to absolute grid positions on each axis not marked fixed.
	inline void Move(wi::csg::Point start, float move[3], double snap, const bool fixed[3])
	{
		const double startAxes[3] = {start.x,start.y,start.z};
		for (int axis = 0; axis < 3; ++axis)
		{
			if (fixed[axis]) continue;
			move[axis] = float(std::round((startAxes[axis]+move[axis])/snap)*snap - startAxes[axis]);
		}
	}

	// Snaps a face drag distance to grid steps. For axis-aligned faces the face plane itself lands on the grid.
	// point is a world point on the face, normal its world normal (any length), and scale the world distance
	// moved per unit of local face-drag distance.
	inline double FaceDistance(double distance, double snap, wi::csg::Point point, wi::csg::Point normal, double scale)
	{
		distance = std::round(distance/snap)*snap;
		const double length = wi::csg::Length(normal);
		if (length <= 0 || scale <= 0) return distance;
		normal = normal * (1/length);
		const double nAxes[3] = {normal.x,normal.y,normal.z}, pAxes[3] = {point.x,point.y,point.z};
		for (int axis = 0; axis < 3; ++axis)
		{
			if (std::abs(nAxes[axis]) < 0.999) continue;
			const double sign = nAxes[axis] > 0 ? 1 : -1;
			const double target = std::round((pAxes[axis] + sign*distance*scale)/snap)*snap;
			distance = (target - pAxes[axis]) * sign / scale;
		}
		return distance;
	}
}
