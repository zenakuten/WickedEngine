#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace wi::csg
{
	struct Point
	{
		double x = 0, y = 0, z = 0;
		Point operator+(Point b) const { return { x + b.x, y + b.y, z + b.z }; }
		Point operator-(Point b) const { return { x - b.x, y - b.y, z - b.z }; }
		Point operator*(double s) const { return { x * s, y * s, z * s }; }
	};
	double Dot(Point a, Point b);
	Point Cross(Point a, Point b);
	double Length(Point a);

	struct Face
	{
		uint64_t id = 0;
		std::vector<uint32_t> vertices;
		Point u = { 1, 0, 0 }, v = { 0, 0, 1 };
		double offsetU = 0, offsetV = 0;
	};
	enum class Operation : uint32_t { Add, Subtract };
	struct Brush
	{
		uint64_t id = 0;
		Operation operation = Operation::Add;
		bool enabled = true;
		std::vector<Point> vertices;
		std::vector<Face> faces;
	};
	struct Polygon
	{
		uint64_t brushID = 0, faceID = 0;
		std::vector<Point> vertices;
		Point normal, u, v;
		double offsetU = 0, offsetV = 0;
	};
	struct Triangle
	{
		Point points[3];
		Point normal, u, v;
		double offsetU = 0, offsetV = 0;
		uint64_t brushID = 0, faceID = 0;
	};
	struct Limits
	{
		uint32_t brushes = 256;
		uint32_t polygons = 65536;
		uint32_t depth = 256;
		uint64_t work = 16000000;
	};
	constexpr uint32_t MaxVertices = 1024;
	constexpr uint32_t MaxFaces = 512;
	constexpr uint32_t MaxFaceVertices = 256;
	constexpr double MaxCoordinate = 1000000;

	// Source faces use outward right-hand winding. Render/cook adapters reverse it.
	// Failed operations preserve output and return a diagnostic; success clears it.
	bool Validate(const Brush& brush, std::string& error);
	bool MakeCube(Point dimensions, uint64_t id, Brush& output, std::string& error);
	bool MakeCone(double radius, double height, uint32_t sides, uint64_t id, Brush& output, std::string& error);
	bool Evaluate(const std::vector<Brush>& brushes, std::vector<Polygon>& output, std::string& error, Limits limits = {});
	bool Triangulate(const std::vector<Polygon>& polygons, std::vector<Triangle>& output, std::string& error);
	// Moves vertices; warped faces are split into planar convex pieces, but the brush must stay convex.
	bool MoveVertices(Brush& brush, const std::vector<uint32_t>& indices, Point delta, std::string& error);
	// Face of `before` that a face of an edited `after` brush descends from, or 0.
	uint64_t SourceFace(const Brush& before, const Brush& after, uint64_t faceID);
	bool DragFace(Brush& brush, uint64_t faceID, double distance, std::string& error);
}
