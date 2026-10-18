#include "../../Editor/CSGSnap.h"
#include <cmath>
#include <iostream>

namespace
{
	int failures = 0;
	void Check(bool ok, const char* message)
	{
		if (!ok) { std::cerr << message << '\n'; ++failures; }
	}
	bool Near(double a, double b) { return std::abs(a - b) <= 1e-5; }
	bool OnGrid(double value, double snap) { return Near(value, std::round(value/snap)*snap); }
}

int main()
{
	using namespace wi::csg;
	const bool none[3] = {false,false,false};
	{
		// The grabbed point lands on the grid; the move itself is not just rounded.
		float move[3] = {1.0f,0.4f,0};
		CSGSnap::Move({0.3,1.7,-2.2},move,1,none);
		Check(Near(move[0],0.7) && Near(move[1],0.3) && Near(move[2],0.2), "Move places grabbed point on unit grid");
	}
	{
		float move[3] = {0.1f,-0.3f,0.6f};
		CSGSnap::Move({0.3,1.7,-2.2},move,0.25,none);
		Check(Near(0.3+move[0],0.5) && Near(1.7+move[1],1.5) && Near(-2.2+move[2],-1.5), "Move snaps to fractional grid");
	}
	{
		// Orthographic panes keep their depth axis exactly as dragged.
		const bool topDepth[3] = {false,true,false};
		float move[3] = {1.0f,0.4f,0};
		CSGSnap::Move({0.3,1.7,-2.2},move,1,topDepth);
		Check(Near(move[0],0.7) && move[1] == 0.4f && Near(move[2],0.2), "Move leaves fixed depth axis untouched");
	}
	{
		float move[3] = {-3.4f,0,0};
		CSGSnap::Move({-0.3,0,0},move,1,none);
		Check(Near(-0.3+move[0],-4), "Move snaps negative coordinates");
	}

	// +X face at x=2.3 dragged outward 1.4 lands on x=3.
	Check(Near(CSGSnap::FaceDistance(1.4,1,{2.3,0,0},{2,0,0},1),0.7), "Axis-aligned face plane lands on grid");
	// -X face at x=-0.6 dragged outward 1.2 lands on x=-2.
	Check(Near(CSGSnap::FaceDistance(1.2,1,{-0.6,5,5},{-1,0,0},1),1.4), "Negative-facing plane lands on grid");
	// Brush scaled by 2: local distance 0.6 rounds to one step and lands the world plane on x=3.
	Check(Near(CSGSnap::FaceDistance(0.6,1,{1,0,0},{1,0,0},2),1), "Face snap accounts for brush scale");
	Check(Near(CSGSnap::FaceDistance(-0.6,0.5,{0,0,0.2},{0,0,1},1),-0.7), "Face snap moves inward to fractional grid");
	Check(Near(CSGSnap::FaceDistance(1.4,1,{0.3,0,0},{1,1,0},1),1), "Oblique faces only round the drag distance");
	Check(Near(CSGSnap::FaceDistance(1.4,1,{0.3,0,0},{0,0,0},1),1), "Degenerate faces only round the drag distance");
	Check(Near(CSGSnap::FaceDistance(1.4,1,{0.3,0,0},{1,0,0},0),1), "Zero scale only rounds the drag distance");

	{
		// End to end: a snapped drag on an off-grid cube face puts that face on the grid.
		Brush cube;
		std::string error;
		Check(MakeCube({2.6,2,2},1,cube,error), "Create cube");
		Check(MoveVertices(cube,{0,1,2,3,4,5,6,7},{0.1,0,0},error), "Offset cube off grid");
		for (const auto& face : cube.faces)
		{
			const Point a = cube.vertices[face.vertices[0]];
			const Point n = Cross(cube.vertices[face.vertices[1]]-a,cube.vertices[face.vertices[2]]-a);
			if (n.x <= 0 || std::abs(n.y) > 1e-9 || std::abs(n.z) > 1e-9) continue;
			Check(Near(a.x,1.4), "Cube +X face starts off grid");
			Brush dragged = cube;
			const double distance = CSGSnap::FaceDistance(0.9,1,a,n,1);
			Check(DragFace(dragged,face.id,distance,error), "Drag snapped face");
			bool moved = false;
			for (const auto& p : dragged.vertices)
				if (p.x > 1.5) { moved = true; Check(Near(p.x,2) && OnGrid(p.x,1), "Dragged face vertices lie on grid plane"); }
			Check(moved, "Snapped drag moves the face");
		}
	}

	if (failures) return 1;
	std::cout << "CSG grid snapping contracts passed\n";
}
