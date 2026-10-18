#include "../../WickedEngine/wiCSG.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace
{
	int failures = 0;
	void Check(bool ok, const char* message)
	{
		if (!ok) { std::cerr << message << '\n'; ++failures; }
	}
	bool Near(double a, double b) { return std::abs(a - b) <= std::max(1.0, std::abs(b)) * 1e-8; }
	double Volume(const std::vector<wi::csg::Triangle>& mesh)
	{
		double volume = 0;
		for (const auto& t : mesh) volume += wi::csg::Dot(t.points[0], wi::csg::Cross(t.points[1], t.points[2])) / 6;
		return volume;
	}
	bool Occupied(const std::vector<wi::csg::Triangle>& mesh, wi::csg::Point point)
	{
		using namespace wi::csg;
		double angle = 0;
		for (const auto& triangle : mesh)
		{
			const Point a = triangle.points[0] - point, b = triangle.points[1] - point, c = triangle.points[2] - point;
			const double x = Length(a), y = Length(b), z = Length(c);
			angle += 2 * std::atan2(Dot(a,Cross(b,c)), x*y*z + Dot(a,b)*z + Dot(b,c)*x + Dot(c,a)*y);
		}
		return std::abs(angle) > 6.28318530717958647692;
	}
	bool Occupied(const std::vector<wi::csg::Brush>& brushes, wi::csg::Point point)
	{
		using namespace wi::csg;
		bool result = false;
		for (const auto& brush : brushes)
		{
			if (!brush.enabled) continue;
			bool inside = true;
			for (const auto& face : brush.faces)
			{
				const Point a = brush.vertices[face.vertices[0]];
				const Point n = Cross(brush.vertices[face.vertices[1]]-a,brush.vertices[face.vertices[2]]-a);
				if (Dot(n,point-a) > 0) { inside = false; break; }
			}
			result = brush.operation == Operation::Add ? result || inside : result && !inside;
		}
		return result;
	}
}
int main()
{
	using namespace wi::csg;
	std::string error;
	Brush cube;
	std::vector<Polygon> polygons;
	std::vector<Triangle> triangles;
	Check(MakeCube({2,4,6}, 42, cube, error), "Create cube");
	Check(cube.vertices.size() == 8 && cube.faces.size() == 6 && error.empty(), "Cube topology");
	Check(Evaluate({cube}, polygons, error) && Triangulate(polygons, triangles, error), "Triangulate cube");
	Check(triangles.size() == 12 && Near(Volume(triangles), 48), "Outward cube volume");
	for (const auto& t : triangles)
		Check(t.brushID == 42 && t.faceID > 0 && t.faceID <= 6 && Near(Length(t.normal), 1), "Face provenance and unit normals");
	for (uint32_t sides : {3u,4u,16u,256u})
	{
		Brush cone;
		Check(MakeCone(3, 8, sides, 7, cone, error), "Create cone");
		Check(Evaluate({cone}, polygons, error) && Triangulate(polygons, triangles, error), "Triangulate cone");
		Check(triangles.size() == 2 * sides - 2 && Near(Volume(triangles),
			sides * 9 * std::sin(6.28318530717958647692 / sides) * 8 / 6), "Polygonal cone volume");
	}
	for (double scale : {0.001, 1.0, 100000.0})
	{
		Brush scaled;
		Check(MakeCube({scale,scale,scale}, 1, scaled, error), "Scaled cube");
		for (auto& p : scaled.vertices) p = p + Point{500000,-500000,500000};
		Check(Validate(scaled, error), "Translated cube validation");
	}
	auto Reject = [&](Brush invalid, const char* reason) {
		Check(!Validate(invalid, error) && !error.empty(), reason);
		const size_t previous = polygons.size();
		Check(!Evaluate({invalid}, polygons, error) && polygons.size() == previous && !error.empty(), "Failure must preserve output");
	};
	auto bad = cube;
	bad.id = 0; Reject(bad, "Reject zero ID");
	bad = cube; bad.faces[1].id = bad.faces[0].id; Reject(bad, "Reject duplicate face ID");
	bad = cube; bad.faces.pop_back(); Reject(bad, "Reject open solid");
	bad = cube; std::reverse(bad.faces[0].vertices.begin(), bad.faces[0].vertices.end()); Reject(bad, "Reject inverted face");
	bad = cube; bad.faces.push_back({99,cube.faces[0].vertices}); Reject(bad, "Reject nonmanifold face");
	bad = cube; bad.faces[0].vertices[0] = 100; Reject(bad, "Reject invalid index");
	bad = cube; bad.faces[0].vertices[0] = bad.faces[0].vertices[1]; Reject(bad, "Reject repeated index");
	bad = cube; std::swap(bad.faces[0].vertices[1], bad.faces[0].vertices[2]); Reject(bad, "Reject crossed loop");
	bad = cube; bad.vertices[0].x += 0.25; Reject(bad, "Reject nonplanar face");
	bad = cube; bad.vertices[1] = bad.vertices[0]; Reject(bad, "Reject coincident vertices");
	bad = cube; bad.vertices.push_back({0,0,0}); Reject(bad, "Reject unused vertex");
	bad = cube; bad.vertices[0].x = std::numeric_limits<double>::infinity(); Reject(bad, "Reject infinite point");
	bad.vertices[0].x = std::numeric_limits<double>::quiet_NaN(); Reject(bad, "Reject NaN");
	bad.vertices[0].x = MaxCoordinate + 1; Reject(bad, "Reject coordinate overflow");
	bad = cube; for (auto& p : bad.vertices) p.y = 0; Reject(bad, "Reject zero volume");
	bad = cube; bad.vertices.resize(MaxVertices + 1); Reject(bad, "Vertex limit");
	bad = cube; bad.faces.resize(MaxFaces + 1); Reject(bad, "Face limit");
	bad = cube; bad.faces[0].vertices.resize(MaxFaceVertices + 1); Reject(bad, "Face vertex limit");
	for (uint32_t sides : {0u,2u,257u,UINT32_MAX})
		Check(!MakeCone(1,1,sides,9,cube,error) && cube.id == 42, "Invalid sides preserve brush");
	Check(!MakeCube({1,0,1},9,cube,error) && cube.id == 42, "Invalid dimensions preserve brush");
	Check(!MakeCube({1,1,1},0,cube,error) && cube.id == 42, "Invalid ID preserves brush");
	Check(!MakeCone(1,std::numeric_limits<double>::infinity(),16,9,cube,error), "Invalid cone height");
	Check(Validate(cube,error) && error.empty(), "Success clears stale errors");

	Brush a, b;
	MakeCube({4,4,4},1,a,error);
	MakeCube({2,2,2},2,b,error);
	auto Expect = [&](std::vector<Brush> brushes, double volume, const char* message) {
		bool ok = Evaluate(brushes,polygons,error) && Triangulate(polygons,triangles,error);
		if (!ok) std::cerr << error << '\n';
		Check(ok && Near(Volume(triangles),volume),message);
		bool occupancy = ok;
		for (int x = -5; x <= 5 && occupancy; ++x)
			for (int y = -5; y <= 5 && occupancy; ++y)
				for (int z = -5; z <= 5 && occupancy; ++z)
				{
					const Point p = {0.173+0.61*x,0.271+0.59*y,0.139+0.67*z};
					occupancy = Occupied(brushes,p) == Occupied(triangles,p);
				}
		Check(occupancy,"Evaluated triangle winding agrees with ordered source occupancy");
	};
	Expect({a,b},64,"Contained union");
	b.operation = Operation::Subtract;
	Expect({b},0,"Subtract from empty");
	Expect({a,b},56,"Enclosed room difference");
	Check(std::any_of(polygons.begin(),polygons.end(),[](const Polygon& p){ return p.brushID == 2; }),"Cuts retain subtractive face provenance");
	b = a; b.id = 2;
	Expect({a,b},64,"Identical union");
	b.operation = Operation::Subtract;
	Expect({a,b},0,"Identical difference");
	MakeCube({4,4,4},2,b,error);
	for (auto& p : b.vertices) p.x += 2;
	Expect({a,b},96,"Overlapping union");
	b.operation = Operation::Subtract;
	Expect({a,b},32,"Overlapping difference");
	for (auto& p : b.vertices) p.x += 2;
	b.operation = Operation::Add;
	Expect({a,b},128,"Face-touching union");
	b.operation = Operation::Subtract;
	Expect({a,b},64,"Face-touching subtraction");
	for (auto& p : b.vertices) p.x += 1;
	b.operation = Operation::Add;
	Expect({a,b},128,"Disjoint union");
	b.operation = Operation::Subtract;
	Expect({a,b},64,"Disjoint subtraction");
	b.enabled = false;
	Expect({a,b},64,"Disabled brush");
	MakeCube({4,4,4},2,b,error);
	for (auto& p : b.vertices) p = p + Point{4,4,0};
	Expect({a,b},128,"Edge-touching union");
	for (auto& p : b.vertices) p.z += 4;
	Expect({a,b},128,"Point-touching union");
	MakeCube({8,8,8},2,b,error);
	b.operation = Operation::Subtract;
	Expect({a,b},0,"Enclosing subtraction");
	MakeCube({2,2,2},2,b,error);
	b.operation = Operation::Subtract;
	auto repeatedCut = b;
	repeatedCut.id = 3;
	Expect({a,b,repeatedCut},56,"Repeated identical room cuts");
	MakeCube({0.02,6,6},2,b,error);
	b.operation = Operation::Subtract;
	Expect({a,b},63.68,"Narrow through-opening");
	Check(!Occupied(triangles,{0,0,0}) && Occupied(triangles,{0.02,0,0}),"Narrow opening is actually clear");
	Brush crossCut;
	MakeCube({6,6,0.02},3,crossCut,error);
	crossCut.operation = Operation::Subtract;
	Expect({a,b,crossCut},63.3616,"Intersecting narrow cuts");
	Brush cone;
	MakeCone(1,2,12,4,cone,error);
	cone.operation = Operation::Subtract;
	cone.faces[1].offsetU = 3.25;
	Expect({a,cone},62,"Polygonal cone subtraction");
	Check(std::any_of(triangles.begin(),triangles.end(),[](const Triangle& t) {
		return t.brushID == 4 && t.faceID == 2 && t.offsetU == 3.25;
	}),"Split/cut triangles preserve source UV mappings");
	const auto previous = triangles;
	Check(Evaluate({a,cone},polygons,error) && Triangulate(polygons,triangles,error) &&
		previous.size() == triangles.size(),"Repeated evaluation preserves triangle count");
	for (size_t i = 0; i < std::min(previous.size(),triangles.size()); ++i)
	{
		const auto& x = previous[i];
		const auto& y = triangles[i];
		Check(x.brushID == y.brushID && x.faceID == y.faceID &&
			Length(x.normal-y.normal) == 0 && Length(x.u-y.u) == 0 && Length(x.v-y.v) == 0 &&
			x.offsetU == y.offsetU && x.offsetV == y.offsetV &&
			Length(x.points[0]-y.points[0]) == 0 && Length(x.points[1]-y.points[1]) == 0 &&
			Length(x.points[2]-y.points[2]) == 0,"Deterministic triangle order, positions and provenance");
	}
	Check(!Evaluate({a,a},polygons,error),"Duplicate brush identities");
	Limits limited;
	limited.work = 1;
	Check(!Evaluate({a,b},polygons,error,limited) && !error.empty(),"Work budget");
	limited = {};
	limited.polygons = 1;
	Check(!Evaluate({a},polygons,error,limited),"Polygon budget on first brush");
	limited = {};
	limited.brushes = 1;
	Check(!Evaluate({a,b},polygons,error,limited),"Brush budget");

	Brush edited = a;
	Check(!MoveVertices(edited,{0},{2,2,2},error) && edited.vertices[0].x == a.vertices[0].x,"Concave vertex move is atomic");
	Brush raised = a;
	Check(MoveVertices(raised,{6},{0,1,0},error) && Validate(raised,error),"Single raised corner splits warped faces");
	Check(raised.faces.size() == 7 && raised.faces[5].id == 6 && raised.faces[6].id == 7,"Split faces keep source IDs and append new IDs");
	for (const auto& f : raised.faces)
		Check(f.id <= 6 || SourceFace(a,raised,f.id) == 6,"Raised corner pieces descend from the top face");
	Expect({raised},64+16.0/3,"Raised corner adds its pyramid");
	Brush lowered = a;
	Check(MoveVertices(lowered,{6},{0,-1,0},error) && lowered.faces.size() == 7,"Lowered corner splits only the top face");
	Check(SourceFace(a,lowered,7) == 6,"Lowered corner split descends from the top face");
	Expect({lowered},64-8.0/3,"Lowered corner removes its tetrahedron");
	Check(MoveVertices(edited,{2,3},{0,1,0},error) && edited.faces.size() == 6,"Planar group edit makes a ramp");
	Brush corner = a;
	Check(MoveVertices(corner,{6},{1,1,1},error) && corner.faces.size() == 9,"Diagonal corner pull splits its three faces");
	Expect({corner},80,"Diagonal corner pull adds three side pyramids");
	Check(DragFace(a,4,2,error),"Plane-based face drag");
	Expect({a},96,"Face drag changes volume");
	Check(!DragFace(a,4,-10,error),"Collapsed/inverted face drag fails");
	Expect({a},96,"Failed face drag preserves brush");
	Check(!DragFace(a,99,1,error),"Invalid face selection");
	if (failures == 0) std::cout << "CSG contracts passed\n";
	return failures == 0 ? 0 : 1;
}
