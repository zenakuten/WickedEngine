#include "wiCSG.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <utility>

namespace wi::csg
{
	double Dot(Point a, Point b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
	Point Cross(Point a, Point b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
	double Length(Point a) { return std::sqrt(Dot(a, a)); }
	namespace
	{
		bool Finite(Point p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
		bool Fail(std::string& error, const char* message) { error = message; return false; }
		Point Normal(const Brush& b, const Face& f)
		{
			Point n = Cross(b.vertices[f.vertices[1]] - b.vertices[f.vertices[0]],
				b.vertices[f.vertices[2]] - b.vertices[f.vertices[0]]);
			return n * (1 / Length(n));
		}
		void SetUV(Brush& b)
		{
			for (auto& f : b.faces)
			{
				Point n = Normal(b, f);
				Point axis = std::abs(n.y) > 0.9 ? Point{ 1, 0, 0 } : Point{ 0, 1, 0 };
				f.u = Cross(axis, n);
				f.u = f.u * (1 / Length(f.u));
				f.v = Cross(n, f.u);
			}
		}
		double Tolerance(const Brush& b)
		{
			Point low = b.vertices.front(), high = low;
			for (Point p : b.vertices)
			{
				low = { std::min(low.x, p.x), std::min(low.y, p.y), std::min(low.z, p.z) };
				high = { std::max(high.x, p.x), std::max(high.y, p.y), std::max(high.z, p.z) };
			}
			return std::max(1e-8, Length(high - low) * 1e-9);
		}
	}
	bool Validate(const Brush& b, std::string& error)
	{
		if (b.id == 0 || (b.operation != Operation::Add && b.operation != Operation::Subtract))
			return Fail(error, "CSG brush has an invalid ID or operation");
		if (b.vertices.size() < 4 || b.vertices.size() > MaxVertices || b.faces.size() < 4 || b.faces.size() > MaxFaces)
			return Fail(error, "CSG brush vertex/face count is outside supported limits");
		for (Point p : b.vertices)
			if (!Finite(p) || std::abs(p.x) > MaxCoordinate || std::abs(p.y) > MaxCoordinate || std::abs(p.z) > MaxCoordinate)
				return Fail(error, "CSG brush has nonfinite or out-of-range coordinates");
		const double epsilon = Tolerance(b);
		for (size_t i = 0; i < b.vertices.size(); ++i)
			for (size_t j = 0; j < i; ++j)
				if (Length(b.vertices[i] - b.vertices[j]) <= epsilon)
					return Fail(error, "CSG brush has duplicate or indistinguishable vertices");
		std::set<uint64_t> ids;
		struct Edge { uint32_t count = 0; int direction = 0; };
		std::map<std::pair<uint32_t, uint32_t>, Edge> edges;
		std::vector<bool> used(b.vertices.size(), false);
		double volume = 0;
		for (const Face& f : b.faces)
		{
			if (f.id == 0 || !ids.insert(f.id).second)
				return Fail(error, "CSG source-face IDs must be nonzero and unique within a brush");
			if (f.vertices.size() < 3 || f.vertices.size() > MaxFaceVertices ||
				!Finite(f.u) || !Finite(f.v) || !std::isfinite(f.offsetU) || !std::isfinite(f.offsetV))
				return Fail(error, "CSG face has invalid vertex count or UV mapping");
			std::set<uint32_t> vertices;
			for (uint32_t i : f.vertices)
			{
				if (i >= b.vertices.size() || !vertices.insert(i).second)
					return Fail(error, "CSG face has an invalid or repeated vertex index");
				used[i] = true;
			}
			Point origin = b.vertices[f.vertices[0]];
			Point a = b.vertices[f.vertices[1]] - origin, c = b.vertices[f.vertices[2]] - origin;
			if (Length(Cross(a, c)) <= epsilon * (Length(a) + Length(c)))
				return Fail(error, "CSG face has a degenerate initial triangle");
			Point n = Normal(b, f);
			for (uint32_t i : f.vertices)
				if (std::abs(Dot(n, b.vertices[i] - origin)) > epsilon)
					return Fail(error, "CSG face is not planar");
			for (Point p : b.vertices)
				if (Dot(n, p - origin) > epsilon)
					return Fail(error, "CSG brush is nonconvex or has inward-facing surfaces");
			for (size_t i = 0; i < f.vertices.size(); ++i)
			{
				uint32_t x = f.vertices[i], y = f.vertices[(i + 1) % f.vertices.size()], z = f.vertices[(i + 2) % f.vertices.size()];
				Point edge = b.vertices[y] - b.vertices[x], next = b.vertices[z] - b.vertices[y];
				if (Dot(Cross(edge, next), n) <= epsilon * (Length(edge) + Length(next)))
					return Fail(error, "CSG face has a collinear or nonconvex corner");
				for (uint32_t k : f.vertices)
					if (Dot(Cross(edge, b.vertices[k] - b.vertices[x]), n) < -epsilon * Length(edge))
						return Fail(error, "CSG face has a crossed or nonconvex boundary");
				auto& e = edges[std::minmax(x, y)];
				++e.count;
				e.direction += x < y ? 1 : -1;
			}
			for (size_t i = 1; i + 1 < f.vertices.size(); ++i)
				volume += Dot(origin - b.vertices[0], Cross(b.vertices[f.vertices[i]] - b.vertices[0],
					b.vertices[f.vertices[i + 1]] - b.vertices[0])) / 6;
		}
		for (const auto& e : edges)
			if (e.second.count != 2 || e.second.direction != 0)
				return Fail(error, "CSG brush is not a closed, consistently wound two-manifold");
		if (std::find(used.begin(), used.end(), false) != used.end() || !std::isfinite(volume) || volume <= epsilon * epsilon * epsilon)
			return Fail(error, "CSG brush has unused vertices or no positive solid volume");
		error.clear();
		return true;
	}
	bool MakeCube(Point size, uint64_t id, Brush& output, std::string& error)
	{
		if (!Finite(size) || size.x <= 0 || size.y <= 0 || size.z <= 0 ||
			size.x > 2 * MaxCoordinate || size.y > 2 * MaxCoordinate || size.z > 2 * MaxCoordinate)
			return Fail(error, "Cube dimensions must be finite, positive and within coordinate limits");
		const double x = size.x / 2, y = size.y / 2, z = size.z / 2;
		Brush b;
		b.id = id;
		b.vertices = { {-x,-y,-z}, {x,-y,-z}, {x,y,-z}, {-x,y,-z}, {-x,-y,z}, {x,-y,z}, {x,y,z}, {-x,y,z} };
		b.faces = { {1,{0,3,2,1}}, {2,{4,5,6,7}}, {3,{0,4,7,3}}, {4,{1,2,6,5}}, {5,{0,1,5,4}}, {6,{3,7,6,2}} };
		SetUV(b);
		if (!Validate(b, error)) return false;
		output = std::move(b);
		return true;
	}
	bool MakeCone(double radius, double height, uint32_t sides, uint64_t id, Brush& output, std::string& error)
	{
		if (!std::isfinite(radius) || !std::isfinite(height) || radius <= 0 || height <= 0 ||
			radius > MaxCoordinate || height > 2 * MaxCoordinate || sides < 3 || sides > MaxFaceVertices)
			return Fail(error, "Cone requires positive finite dimensions and 3 to 256 sides within coordinate limits");
		Brush b;
		b.id = id;
		Face base;
		base.id = 1;
		for (uint32_t i = 0; i < sides; ++i)
		{
			double a = 6.28318530717958647692 * i / sides;
			b.vertices.push_back({ radius * std::cos(a), -height / 2, radius * std::sin(a) });
			base.vertices.push_back(i);
		}
		b.vertices.push_back({ 0, height / 2, 0 });
		b.faces.push_back(std::move(base));
		for (uint32_t i = 0; i < sides; ++i)
			b.faces.push_back({ uint64_t(i) + 2, {i, sides, (i + 1) % sides} });
		SetUV(b);
		if (!Validate(b, error)) return false;
		output = std::move(b);
		return true;
	}

	namespace
	{
		struct Budget
		{
			Limits limits;
			uint64_t work = 0, fragments = 0;
			double epsilon = 1e-8;
			std::string& error;
			bool Spend(uint64_t n = 1)
			{
				work += n;
				return work <= limits.work || Fail(error, "CSG evaluation work limit exceeded");
			}
			bool Fragment()
			{
				return ++fragments <= uint64_t(limits.polygons) * 8 || Fail(error, "CSG evaluation fragment limit exceeded");
			}
		};
		void Flip(Polygon& p)
		{
			std::reverse(p.vertices.begin(), p.vertices.end());
			p.normal = p.normal * -1;
		}
		bool Clean(Polygon& p, double epsilon)
		{
			bool changed = true;
			while (changed && p.vertices.size() >= 3)
			{
				changed = false;
				for (size_t i = 0; i < p.vertices.size(); ++i)
				{
					Point a = p.vertices[i], b = p.vertices[(i + 1) % p.vertices.size()], c = p.vertices[(i + 2) % p.vertices.size()];
					if (Length(b - a) <= epsilon || Length(Cross(b - a, c - b)) <= epsilon * (Length(b - a) + Length(c - b)))
					{
						p.vertices.erase(p.vertices.begin() + (i + 1) % p.vertices.size());
						changed = true;
						break;
					}
				}
			}
			return p.vertices.size() >= 3;
		}
		struct Plane
		{
			Point n;
			double d = 0;
			explicit Plane(const Polygon& p) : n(p.normal), d(Dot(n, p.vertices[0])) {}
			bool Split(const Polygon& p, std::vector<Polygon>& coplanarFront, std::vector<Polygon>& coplanarBack,
				std::vector<Polygon>& front, std::vector<Polygon>& back, Budget& budget) const
			{
				if (!budget.Spend(p.vertices.size())) return false;
				std::vector<int> types;
				int type = 0;
				for (Point v : p.vertices)
				{
					double distance = Dot(n, v) - d;
					int side = distance > budget.epsilon ? 1 : distance < -budget.epsilon ? 2 : 0;
					types.push_back(side);
					type |= side;
				}
				if (type == 0)
					(Dot(n, p.normal) >= 0 ? coplanarFront : coplanarBack).push_back(p);
				else if (type == 1) front.push_back(p);
				else if (type == 2) back.push_back(p);
				else
				{
					if (!budget.Fragment()) return false;
					Polygon f = p, b = p;
					f.vertices.clear();
					b.vertices.clear();
					for (size_t i = 0; i < p.vertices.size(); ++i)
					{
						size_t j = (i + 1) % p.vertices.size();
						Point a = p.vertices[i], v = p.vertices[j];
						if (types[i] != 2) f.vertices.push_back(a);
						if (types[i] != 1) b.vertices.push_back(a);
						if ((types[i] | types[j]) == 3)
						{
							double t = (d - Dot(n, a)) / Dot(n, v - a);
							Point intersection = a + (v - a) * std::clamp(t, 0.0, 1.0);
							f.vertices.push_back(intersection);
							b.vertices.push_back(intersection);
						}
					}
					if (Clean(f, budget.epsilon)) front.push_back(std::move(f));
					if (Clean(b, budget.epsilon)) back.push_back(std::move(b));
				}
				return true;
			}
		};
		struct Node
		{
			std::unique_ptr<Plane> plane;
			std::vector<Polygon> polygons;
			std::unique_ptr<Node> front, back;
			bool Build(const std::vector<Polygon>& input, Budget& budget, uint32_t depth = 0)
			{
				if (input.empty()) return true;
				if (depth > budget.limits.depth) return Fail(budget.error, "CSG partition depth limit exceeded");
				if (!budget.Spend(input.size())) return false;
				if (!plane) plane = std::make_unique<Plane>(input.front());
				std::vector<Polygon> f, b;
				for (const auto& p : input)
					if (!plane->Split(p, polygons, polygons, f, b, budget)) return false;
				if (!f.empty())
				{
					if (!front) front = std::make_unique<Node>();
					if (!front->Build(f, budget, depth + 1)) return false;
				}
				if (!b.empty())
				{
					if (!back) back = std::make_unique<Node>();
					if (!back->Build(b, budget, depth + 1)) return false;
				}
				return true;
			}
			void Invert()
			{
				for (auto& p : polygons) Flip(p);
				if (plane) { plane->n = plane->n * -1; plane->d = -plane->d; }
				if (front) front->Invert();
				if (back) back->Invert();
				std::swap(front, back);
			}
			bool Clip(std::vector<Polygon>& input, Budget& budget) const
			{
				if (!plane || input.empty()) return true;
				std::vector<Polygon> f, b;
				for (const auto& p : input)
					if (!plane->Split(p, f, b, f, b, budget)) return false;
				if (front && !front->Clip(f, budget)) return false;
				if (back)
				{
					if (!back->Clip(b, budget)) return false;
				}
				else b.clear();
				f.insert(f.end(), std::make_move_iterator(b.begin()), std::make_move_iterator(b.end()));
				if (f.size() > budget.limits.polygons) return Fail(budget.error, "CSG polygon limit exceeded");
				input = std::move(f);
				return true;
			}
			bool ClipTo(const Node& other, Budget& budget)
			{
				return other.Clip(polygons, budget) && (!front || front->ClipTo(other, budget)) && (!back || back->ClipTo(other, budget));
			}
			void Gather(std::vector<Polygon>& result) const
			{
				result.insert(result.end(), polygons.begin(), polygons.end());
				if (front) front->Gather(result);
				if (back) back->Gather(result);
			}
		};
	}
	bool Evaluate(const std::vector<Brush>& brushes, std::vector<Polygon>& output, std::string& error, Limits limits)
	{
		if (brushes.size() > limits.brushes || limits.depth == 0 || limits.depth > 512 || limits.polygons == 0)
			return Fail(error, "CSG brush count or evaluation limits are invalid");
		Budget budget{ limits, 0, 0, 1e-8, error };
		std::set<uint64_t> ids;
		for (const auto& b : brushes)
		{
			if (!Validate(b, error)) return false;
			if (!budget.Spend(b.vertices.size() + b.faces.size())) return false;
			if (!ids.insert(b.id).second) return Fail(error, "CSG brush IDs must be unique");
			budget.epsilon = std::max(budget.epsilon, Tolerance(b));
		}
		std::vector<Polygon> result;
		for (const auto& brush : brushes)
		{
			if (!brush.enabled) continue;
			std::vector<Polygon> incoming;
			for (const auto& face : brush.faces)
			{
				Polygon p;
				p.brushID = brush.id; p.faceID = face.id;
				p.normal = Normal(brush, face); p.u = face.u; p.v = face.v;
				p.offsetU = face.offsetU; p.offsetV = face.offsetV;
				for (uint32_t index : face.vertices) p.vertices.push_back(brush.vertices[index]);
				incoming.push_back(std::move(p));
			}
			if (result.empty())
			{
				if (brush.operation == Operation::Add) result = std::move(incoming);
				continue;
			}
			Node a, b;
			if (!a.Build(result, budget) || !b.Build(incoming, budget)) return false;
			const bool subtract = brush.operation == Operation::Subtract;
			if (subtract) a.Invert();
			if (!a.ClipTo(b, budget) || !b.ClipTo(a, budget)) return false;
			b.Invert();
			if (!b.ClipTo(a, budget)) return false;
			b.Invert();
			incoming.clear();
			b.Gather(incoming);
			if (!a.Build(incoming, budget)) return false;
			if (subtract) a.Invert();
			result.clear();
			a.Gather(result);
			if (result.size() > limits.polygons) return Fail(error, "CSG output polygon limit exceeded");
		}
		if (result.size() > limits.polygons) return Fail(error, "CSG output polygon limit exceeded");
		output = std::move(result);
		error.clear();
		return true;
	}
	bool Triangulate(const std::vector<Polygon>& polygons, std::vector<Triangle>& output, std::string& error)
	{
		std::vector<Triangle> result;
		if (polygons.size() > 65536) return Fail(error, "CSG triangulation polygon limit exceeded");
		for (const auto& p : polygons)
		{
			if (p.vertices.size() < 3 || p.vertices.size() > 4096 || !Finite(p.normal) || !Finite(p.u) || !Finite(p.v) ||
				std::abs(Length(p.normal) - 1) > 1e-6 ||
				!std::isfinite(p.offsetU) || !std::isfinite(p.offsetV) || p.brushID == 0 || p.faceID == 0)
				return Fail(error, "Invalid CSG output polygon");
			for (Point v : p.vertices)
				if (!Finite(v) || std::abs(v.x) > MaxCoordinate || std::abs(v.y) > MaxCoordinate || std::abs(v.z) > MaxCoordinate)
					return Fail(error, "Invalid CSG output coordinate");
			for (size_t i = 1; i + 1 < p.vertices.size(); ++i)
			{
				Point cross = Cross(p.vertices[i] - p.vertices[0], p.vertices[i + 1] - p.vertices[0]);
				if (Dot(cross, p.normal) <= 0) return Fail(error, "Degenerate or inverted CSG output triangle");
				result.push_back({ {p.vertices[0], p.vertices[i], p.vertices[i + 1]}, p.normal, p.u, p.v,
					p.offsetU, p.offsetV, p.brushID, p.faceID });
				if (result.size() > 262144) return Fail(error, "CSG output triangle limit exceeded");
			}
		}
		output = std::move(result);
		error.clear();
		return true;
	}
	namespace
	{
		// Rebuilds faces as the convex hull of the moved vertices, splitting warped faces into planar
		// polygons. Every vertex must remain a hull corner, so the vertex indices stay meaningful.
		bool RebuildHull(const Brush& original, Brush& next, std::string& error)
		{
			const auto& p = next.vertices;
			const uint32_t count = uint32_t(p.size());
			const double epsilon = Tolerance(next);
			struct Triangle { uint32_t v[3]; Point n; double d; bool alive; };
			std::vector<Triangle> triangles;
			auto add = [&](uint32_t a, uint32_t b, uint32_t c) {
				Point n = Cross(p[b] - p[a], p[c] - p[a]);
				const double length = Length(n);
				if (!(length > 0)) return false;
				n = n * (1 / length);
				triangles.push_back({ { a, b, c }, n, Dot(n, p[a]), true });
				return true;
			};
			uint32_t a = 0, b = 0, c = 0, d = 0;
			double best = 0;
			for (uint32_t i = 1; i < count; ++i)
				if (Length(p[i] - p[a]) > best) { best = Length(p[i] - p[a]); b = i; }
			best = 0;
			for (uint32_t i = 0; i < count; ++i)
			{
				const double area = Length(Cross(p[b] - p[a], p[i] - p[a]));
				if (area > best) { best = area; c = i; }
			}
			best = 0;
			const Point base = Cross(p[b] - p[a], p[c] - p[a]);
			for (uint32_t i = 0; i < count; ++i)
			{
				const double volume = std::abs(Dot(base, p[i] - p[a]));
				if (volume > best) { best = volume; d = i; }
			}
			if (b == a || c == a || c == b || d == a || d == b || d == c || !(best > epsilon * Length(base)))
				return Fail(error, "Vertex move would flatten the brush");
			const Point centroid = (p[a] + p[b] + p[c] + p[d]) * 0.25;
			const uint32_t simplex[4][3] = { { a, b, c }, { a, b, d }, { a, c, d }, { b, c, d } };
			for (const auto& f : simplex)
			{
				if (!add(f[0], f[1], f[2])) return Fail(error, "Vertex move would flatten the brush");
				if (Dot(triangles.back().n, centroid) - triangles.back().d > 0)
				{
					triangles.pop_back();
					add(f[0], f[2], f[1]);
				}
			}
			for (uint32_t i = 0; i < count; ++i)
			{
				if (i == a || i == b || i == c || i == d) continue;
				std::map<std::pair<uint32_t, uint32_t>, size_t> owner;
				std::vector<size_t> visible;
				for (size_t t = 0; t < triangles.size(); ++t)
				{
					if (!triangles[t].alive) continue;
					for (int e = 0; e < 3; ++e) owner[{ triangles[t].v[e], triangles[t].v[(e + 1) % 3] }] = t;
					if (Dot(triangles[t].n, p[i]) - triangles[t].d > epsilon) visible.push_back(t);
				}
				if (visible.empty()) continue;
				std::vector<std::pair<uint32_t, uint32_t>> horizon;
				for (size_t t : visible)
					for (int e = 0; e < 3; ++e)
					{
						const uint32_t u = triangles[t].v[e], v = triangles[t].v[(e + 1) % 3];
						const auto neighbor = owner.find({ v, u });
						if (neighbor == owner.end()) return Fail(error, "Vertex move produced an open hull");
						if (std::find(visible.begin(), visible.end(), neighbor->second) == visible.end()) horizon.push_back({ u, v });
					}
				for (size_t t : visible) triangles[t].alive = false;
				for (auto edge : horizon)
					if (!add(edge.first, edge.second, i)) return Fail(error, "Vertex move produced a degenerate face");
			}
			std::vector<Triangle> hull;
			for (const auto& t : triangles) if (t.alive) hull.push_back(t);
			std::vector<bool> used(count, false);
			for (const auto& t : hull) for (uint32_t v : t.v) used[v] = true;
			if (std::find(used.begin(), used.end(), false) != used.end())
				return Fail(error, "Vertex move would make the brush concave; move more vertices together");

			// Merge coplanar neighbors into planar convex faces.
			std::map<std::pair<uint32_t, uint32_t>, size_t> owner;
			for (size_t t = 0; t < hull.size(); ++t)
				for (int e = 0; e < 3; ++e) owner[{ hull[t].v[e], hull[t].v[(e + 1) % 3] }] = t;
			std::vector<size_t> group(hull.size());
			for (size_t t = 0; t < hull.size(); ++t) group[t] = t;
			std::function<size_t(size_t)> root = [&](size_t t) { return group[t] == t ? t : group[t] = root(group[t]); };
			for (size_t t = 0; t < hull.size(); ++t)
				for (int e = 0; e < 3; ++e)
				{
					const auto found = owner.find({ hull[t].v[(e + 1) % 3], hull[t].v[e] });
					if (found == owner.end()) return Fail(error, "Vertex move produced an open hull");
					const auto& s = hull[found->second];
					bool coplanar = Dot(hull[t].n, s.n) > 0;
					for (uint32_t v : s.v) coplanar = coplanar && std::abs(Dot(hull[t].n, p[v]) - hull[t].d) <= epsilon;
					if (coplanar) group[root(t)] = root(found->second);
				}
			std::map<size_t, std::map<uint32_t, uint32_t>> boundaries;
			for (size_t t = 0; t < hull.size(); ++t)
				for (int e = 0; e < 3; ++e)
				{
					const uint32_t u = hull[t].v[e], v = hull[t].v[(e + 1) % 3];
					if (root(owner.find({ v, u })->second) != root(t)) boundaries[root(t)][u] = v;
				}
			std::vector<Face> faces;
			for (const auto& boundary : boundaries)
			{
				Face face;
				uint32_t start = boundary.second.begin()->first, v = start;
				do
				{
					face.vertices.push_back(v);
					const auto next = boundary.second.find(v);
					if (next == boundary.second.end() || face.vertices.size() > boundary.second.size())
						return Fail(error, "Vertex move produced a nonmanifold face");
					v = next->second;
				} while (v != start);
				if (face.vertices.size() != boundary.second.size()) return Fail(error, "Vertex move produced a nonmanifold face");
				faces.push_back(std::move(face));
			}
			if (faces.size() > MaxFaces) return Fail(error, "Vertex move exceeds the brush face limit");

			// Keep the original face identity/UV mapping for the best-matching piece; extra pieces get new IDs.
			struct Match { size_t face, source; size_t shared; double alignment; };
			std::vector<Match> matches;
			for (size_t f = 0; f < faces.size(); ++f)
			{
				const Point n = Normal(next, faces[f]);
				for (size_t s = 0; s < original.faces.size(); ++s)
				{
					size_t shared = 0;
					for (uint32_t v : faces[f].vertices)
						shared += std::count(original.faces[s].vertices.begin(), original.faces[s].vertices.end(), v);
					matches.push_back({ f, s, shared, Dot(n, Normal(original, original.faces[s])) });
				}
			}
			std::stable_sort(matches.begin(), matches.end(), [](const Match& x, const Match& y) {
				return x.shared != y.shared ? x.shared > y.shared : x.alignment > y.alignment;
			});
			std::vector<bool> named(faces.size(), false), taken(original.faces.size(), false), mapped(faces.size(), false);
			uint64_t nextID = 1;
			for (const auto& f : original.faces) nextID = std::max(nextID, f.id + 1);
			for (const auto& m : matches)
			{
				Face& face = faces[m.face];
				const Face& source = original.faces[m.source];
				if (!mapped[m.face] && m.alignment > 0.5)
				{
					face.u = source.u; face.v = source.v;
					face.offsetU = source.offsetU; face.offsetV = source.offsetV;
					mapped[m.face] = true;
				}
				if (!named[m.face] && !taken[m.source] && m.alignment > 0.5)
				{
					face.id = source.id;
					named[m.face] = taken[m.source] = true;
				}
			}
			Brush candidate = next;
			candidate.faces = std::move(faces);
			for (size_t f = 0; f < candidate.faces.size(); ++f)
			{
				if (!named[f]) candidate.faces[f].id = nextID++;
				if (!mapped[f])
				{
					Brush single = candidate;
					single.faces = { candidate.faces[f] };
					SetUV(single);
					candidate.faces[f].u = single.faces[0].u;
					candidate.faces[f].v = single.faces[0].v;
				}
			}
			std::sort(candidate.faces.begin(), candidate.faces.end(), [](const Face& x, const Face& y) { return x.id < y.id; });
			if (!Validate(candidate, error)) return false;
			next = std::move(candidate);
			return true;
		}
	}
	bool MoveVertices(Brush& brush, const std::vector<uint32_t>& indices, Point delta, std::string& error)
	{
		if (!Finite(delta) || indices.empty()) return Fail(error, "Select vertices and specify a finite movement");
		Brush next = brush;
		std::set<uint32_t> unique;
		for (uint32_t index : indices)
		{
			if (index >= next.vertices.size() || !unique.insert(index).second) return Fail(error, "Invalid vertex selection");
			next.vertices[index] = next.vertices[index] + delta;
		}
		if (!Validate(next, error))
		{
			// Warped faces are split into planar pieces, as long as the brush stays convex.
			for (Point v : next.vertices)
				if (!Finite(v) || std::abs(v.x) > MaxCoordinate || std::abs(v.y) > MaxCoordinate || std::abs(v.z) > MaxCoordinate)
					return Fail(error, "CSG brush has nonfinite or out-of-range coordinates");
			if (!RebuildHull(brush, next, error)) return false;
		}
		brush = std::move(next);
		return true;
	}
	uint64_t SourceFace(const Brush& before, const Brush& after, uint64_t faceID)
	{
		const auto face = std::find_if(after.faces.begin(), after.faces.end(), [&](const Face& f) { return f.id == faceID; });
		if (face == after.faces.end() || after.vertices.size() != before.vertices.size()) return 0;
		for (const auto& f : before.faces) if (f.id == faceID) return faceID;
		const Point n = Normal(after, *face);
		uint64_t best = 0;
		size_t bestShared = 0;
		double bestAlignment = 0.5;
		for (const auto& f : before.faces)
		{
			size_t shared = 0;
			for (uint32_t v : face->vertices) shared += std::count(f.vertices.begin(), f.vertices.end(), v);
			const double alignment = Dot(n, Normal(before, f));
			if (alignment > 0.5 && (shared > bestShared || (shared == bestShared && alignment > bestAlignment)))
			{
				best = f.id; bestShared = shared; bestAlignment = alignment;
			}
		}
		return best;
	}
	bool DragFace(Brush& brush, uint64_t id, double distance, std::string& error)
	{
		if (!Validate(brush, error)) return false;
		if (!std::isfinite(distance)) return Fail(error, "Face displacement must be finite");
		struct FacePlane { Point n; double d; };
		std::vector<FacePlane> planes;
		bool found = false;
		for (const auto& face : brush.faces)
		{
			Point n = Normal(brush, face);
			planes.push_back({ n, Dot(n, brush.vertices[face.vertices[0]]) + (face.id == id ? distance : 0) });
			found |= face.id == id;
		}
		if (!found) return Fail(error, "Selected CSG face no longer exists");
		Brush next = brush;
		const double epsilon = Tolerance(brush);
		for (size_t vertex = 0; vertex < brush.vertices.size(); ++vertex)
		{
			std::vector<size_t> adjacent;
			for (size_t f = 0; f < brush.faces.size(); ++f)
				if (std::find(brush.faces[f].vertices.begin(), brush.faces[f].vertices.end(), uint32_t(vertex)) != brush.faces[f].vertices.end())
					adjacent.push_back(f);
			bool solved = false;
			Point point;
			for (size_t a = 0; a < adjacent.size() && !solved; ++a)
				for (size_t b = a + 1; b < adjacent.size() && !solved; ++b)
					for (size_t c = b + 1; c < adjacent.size() && !solved; ++c)
					{
						auto p = planes[adjacent[a]], q = planes[adjacent[b]], r = planes[adjacent[c]];
						double det = Dot(p.n, Cross(q.n, r.n));
						if (std::abs(det) < 1e-10) continue;
						point = (Cross(q.n, r.n) * p.d + Cross(r.n, p.n) * q.d + Cross(p.n, q.n) * r.d) * (1 / det);
						solved = true;
					}
			if (!solved) return Fail(error, "Face drag cannot resolve a vertex plane intersection");
			for (size_t f : adjacent)
				if (std::abs(Dot(planes[f].n, point) - planes[f].d) > epsilon)
					return Fail(error, "Face drag would change brush topology; use a different brush or vertex group");
			next.vertices[vertex] = point;
		}
		if (!Validate(next, error)) return false;
		brush = std::move(next);
		return true;
	}
}
