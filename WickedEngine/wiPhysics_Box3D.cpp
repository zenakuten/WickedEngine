#include "wiPhysics.h"
#include "wiBacklog.h"
#include "wiRenderer.h"
#include "Box3D/include/box3d/box3d.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wi::physics
{
	using namespace wi::scene;
	using namespace wi::ecs;
	using namespace DirectX;

	namespace
	{
		std::atomic<bool> enabled{ true }, simulation{ true }, interpolation{ true }, debugdraw{ false };
		std::atomic<int> accuracy{ 10 };
		std::atomic<float> timestep{ 1.0f / 60.0f }, constraint_size{ 0.1f }, debug_distance{ 1000 }, character_tolerance{ 0.02f };
		std::mutex world_lifecycle_mutex, cooking_mutex;

		b3Vec3 vec(const XMFLOAT3& v) { return { v.x, v.y, v.z }; }
		XMFLOAT3 vec(b3Vec3 v) { return { v.x, v.y, v.z }; }
		b3Quat quat(const XMFLOAT4& q) { return { { q.x, q.y, q.z }, q.w }; }
		XMFLOAT4 quat(b3Quat q) { return { q.v.x, q.v.y, q.v.z, q.s }; }
		bool finite(const XMFLOAT3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
		bool valid_rotation(const XMFLOAT4& q)
		{
			return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w)
				&& q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 1e-12f;
		}
		b3Quat normalized(const XMFLOAT4& q)
		{
			XMFLOAT4 result;
			XMStoreFloat4(&result, XMQuaternionNormalize(XMLoadFloat4(&q)));
			return quat(result);
		}
		uint64_t hash_bytes(uint64_t hash, const void* data, size_t count)
		{
			const auto* bytes = static_cast<const unsigned char*>(data);
			for (size_t i = 0; i < count; ++i)
				hash = (hash ^ bytes[i]) * 1099511628211ull;
			return hash;
		}
		template<typename T> void hash_value(uint64_t& hash, const T& value)
		{
			hash = hash_bytes(hash, &value, sizeof(value));
		}
		void unsupported(const char* operation, Feature feature)
		{
			wilog_error("Box3D: %s: %s", operation, GetUnsupportedReason(feature));
		}

		struct Geometry
		{
			RigidBodyPhysicsComponent::CollisionShape kind = RigidBodyPhysicsComponent::BOX;
			uint64_t key = 0;
			XMFLOAT3 scale = XMFLOAT3(1, 1, 1);
			b3BoxHull box{};
			b3HullData* hull = nullptr;
			b3MeshData* mesh = nullptr;
			b3HeightFieldData* height = nullptr;
			b3Sphere sphere{};
			b3Capsule capsule{};
			b3Vec3 origin = b3Vec3_zero;
			float planar_half_thickness = 0;
			~Geometry()
			{
				// The builders/destructors do not take a world and are conservatively serialized.
				std::lock_guard<std::mutex> lock(cooking_mutex);
				if (hull) b3DestroyHull(hull);
				if (mesh) b3DestroyMesh(mesh);
				if (height) b3DestroyHeightField(height);
			}
		};
		struct Body
		{
			b3BodyId id{};
			b3ShapeId shape{};
			Entity entity = INVALID_ENTITY;
			std::shared_ptr<Geometry> geometry;
			std::atomic<bool> released{ false };
			b3WorldTransform previous = b3WorldTransform_identity;
			b3WorldTransform current = b3WorldTransform_identity;
			b3WorldTransform initial = b3WorldTransform_identity;
			b3Vec3 offset = b3Vec3_zero;
			bool ghost = false;
			bool teleported = false;
			float authored_mass = -1;
		};
		struct Joint
		{
			b3JointId id{};
			b3BodyId a{}, b{};
			b3BodyId anchor_a{}, anchor_b{};
			std::atomic<bool> released{ false };
			bool broken = false;
			PhysicsConstraintComponent::Type type = PhysicsConstraintComponent::Type::Fixed;
			Entity entity_a = INVALID_ENTITY, entity_b = INVALID_ENTITY;
			b3Transform frame_a = b3Transform_identity, frame_b = b3Transform_identity;
			float bind_distance = 0;
			float break_distance = FLT_MAX;
		};
		struct World
		{
			b3WorldId id{};
			std::mutex gate;
			std::vector<std::shared_ptr<Body>> bodies;
			std::vector<std::shared_ptr<Joint>> joints;
			std::unordered_map<uint64_t, std::weak_ptr<Geometry>> geometry_cache;
			std::unordered_map<Entity, std::array<std::string, 7>> diagnostics;
			float accumulator = 0;
			bool activate_all = false;
			World()
			{
				b3WorldDef def = b3DefaultWorldDef();
				// Each scene has its own scheduler. Keep the budget deliberately small.
				def.workerCount = std::max(1u, std::min(2u, std::thread::hardware_concurrency() / 4));
				def.enableContinuous = true;
				std::lock_guard<std::mutex> lock(world_lifecycle_mutex);
				id = b3CreateWorld(&def);
			}
			~World()
			{
				std::lock_guard<std::mutex> lock(gate);
				std::lock_guard<std::mutex> global_lock(world_lifecycle_mutex);
				if (b3World_IsValid(id)) b3DestroyWorld(id);
				// Shape references disappear before the cooked data is freed.
				joints.clear();
				bodies.clear();
			}
			void Diagnose(Entity entity, uint64_t configuration, const char* reason, size_t category = 0, bool warning = false)
			{
				const std::string signature = std::to_string(configuration) + ":" + reason;
				auto& previous = diagnostics[entity][category];
				if (previous == signature) return;
				previous = signature;
				wilog_level("Box3D entity %llu: %s",
					warning ? wi::backlog::LogLevel::Warning : wi::backlog::LogLevel::Error,
					static_cast<unsigned long long>(entity), reason);
			}
			void DestroyJoint(Joint& joint)
			{
				if (b3Joint_IsValid(joint.id)) b3DestroyJoint(joint.id, true);
				joint.id = {};
				if (b3Body_IsValid(joint.anchor_a)) b3DestroyBody(joint.anchor_a);
				if (b3Body_IsValid(joint.anchor_b)) b3DestroyBody(joint.anchor_b);
				joint.anchor_a = joint.anchor_b = {};
			}
			void Drain()
			{
				if (!b3World_IsValid(id)) return;
				// Snapshot releases. A destructor may publish another release during this drain;
				// that record must remain alive until its handles are destroyed on the next drain.
				std::unordered_set<uint64_t> pending_bodies;
				for (const auto& body : bodies)
					if (body->released.load()) pending_bodies.insert(b3StoreBodyId(body->id));
				// Remove joints (including pick anchors) before body user data and geometry.
				for (auto it = joints.begin(); it != joints.end();)
				{
					auto& joint = **it;
					const bool released = joint.released.load();
					const bool orphaned = !b3Body_IsValid(joint.a) || !b3Body_IsValid(joint.b) ||
						pending_bodies.count(b3StoreBodyId(joint.a)) != 0 || pending_bodies.count(b3StoreBodyId(joint.b)) != 0;
					if (released || orphaned)
					{
						DestroyJoint(joint);
						joint.broken = true;
					}
					if (released) it = joints.erase(it);
					else ++it;
				}
				for (auto it = bodies.begin(); it != bodies.end();)
				{
					auto& body = **it;
					if (pending_bodies.count(b3StoreBodyId(body.id)) != 0)
					{
						if (b3Body_IsValid(body.id)) b3DestroyBody(body.id);
						body.id = {};
						it = bodies.erase(it);
					}
					else ++it;
				}
				for (auto it = geometry_cache.begin(); it != geometry_cache.end();)
				{
					if (it->second.expired()) it = geometry_cache.erase(it);
					else ++it;
				}
			}
		};
		struct SceneState { std::shared_ptr<World> world = std::make_shared<World>(); };
		struct BodyState
		{
			std::shared_ptr<World> world;
			std::shared_ptr<Body> body;
			std::shared_ptr<Geometry> prepared;
			~BodyState() { if (body) body->released.store(true); }
		};
		struct JointState
		{
			std::shared_ptr<World> world;
			std::shared_ptr<Joint> joint;
			~JointState() { if (joint) joint->released.store(true); }
		};
		struct DragState : JointState
		{
			float distance = 0;
			std::shared_ptr<Body> target;
		};
		std::shared_ptr<World> get_world(const Scene& scene)
		{
			if (!scene.physics_scene) return {};
			return static_cast<SceneState*>(scene.physics_scene.get())->world;
		}
		BodyState* state(const RigidBodyPhysicsComponent& component)
		{
			return static_cast<BodyState*>(component.physicsobject.get());
		}
		JointState* state(const PhysicsConstraintComponent& component)
		{
			return static_cast<JointState*>(component.physicsobject.get());
		}
		b3BodyType body_type(const RigidBodyPhysicsComponent& component)
		{
			return component.mass == 0 ? b3_staticBody : component.IsKinematic() ? b3_kinematicBody : b3_dynamicBody;
		}
		uint64_t geometry_key(const RigidBodyPhysicsComponent& component, const XMFLOAT3& scale, const MeshComponent* mesh)
		{
			uint64_t hash = 14695981039346656037ull;
			hash_value(hash, component.shape);
			hash_value(hash, scale);
			hash_value(hash, component.box.halfextents);
			hash_value(hash, component.sphere.radius);
			hash_value(hash, component.capsule.radius);
			hash_value(hash, component.capsule.height);
			hash_value(hash, component.mesh_lod);
			if (mesh && (component.shape == RigidBodyPhysicsComponent::CONVEX_HULL ||
				component.shape == RigidBodyPhysicsComponent::TRIANGLE_MESH || component.shape == RigidBodyPhysicsComponent::HEIGHTFIELD))
			{
				hash = hash_bytes(hash, mesh->vertex_positions.data(), mesh->vertex_positions.size() * sizeof(XMFLOAT3));
				hash = hash_bytes(hash, mesh->indices.data(), mesh->indices.size() * sizeof(uint32_t));
				for (const auto& subset : mesh->subsets)
				{
					hash_value(hash, subset.indexOffset);
					hash_value(hash, subset.indexCount);
				}
				hash_value(hash, mesh->subsets_per_lod);
			}
			return hash;
		}
		b3HullData* cook_planar_hull(const std::vector<b3Vec3>& points, float& half_thickness)
		{
			if (points.size() < 3)
				return nullptr;
			const b3Vec3 origin = points.front();
			auto difference = [&](const b3Vec3& point) {
				return std::array<double, 3>{ double(point.x) - origin.x, double(point.y) - origin.y, double(point.z) - origin.z };
			};
			auto length_squared = [](const std::array<double, 3>& v) {
				return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
			};
			std::array<double, 3> edge{};
			double extent_squared = 0;
			for (const auto& point : points)
			{
				const auto delta = difference(point);
				const double distance = length_squared(delta);
				if (distance > extent_squared)
				{
					extent_squared = distance;
					edge = delta;
				}
			}
			std::array<double, 3> normal{};
			double area_squared = 0;
			for (const auto& point : points)
			{
				const auto delta = difference(point);
				const std::array<double, 3> cross{
					edge[1] * delta[2] - edge[2] * delta[1],
					edge[2] * delta[0] - edge[0] * delta[2],
					edge[0] * delta[1] - edge[1] * delta[0]
				};
				const double area = length_squared(cross);
				if (area > area_squared)
				{
					area_squared = area;
					normal = cross;
				}
			}
			if (area_squared == 0)
				return nullptr;
			const double tolerance = std::sqrt(extent_squared) * 4 * FLT_EPSILON;
			const double normal_length = std::sqrt(area_squared);
			for (const auto& point : points)
			{
				const auto delta = difference(point);
				const double distance = std::abs(normal[0] * delta[0] + normal[1] * delta[1] + normal[2] * delta[2]);
				if (distance > tolerance * normal_length)
					return nullptr;
			}
			// Box3D requires volume, including for kinematic hulls. Keep this numerical
			// thickness symmetric about the authored plane and report it to the caller.
			half_thickness = float(std::max(0.001, std::sqrt(extent_squared) * 16 * FLT_EPSILON));
			const b3Vec3 offset{
				float(normal[0] / normal_length * half_thickness),
				float(normal[1] / normal_length * half_thickness),
				float(normal[2] / normal_length * half_thickness)
			};
			std::vector<b3Vec3> expanded;
			expanded.reserve(points.size() * 2);
			for (const auto& point : points)
			{
				const auto above = b3Add(point, offset);
				const auto below = b3Sub(point, offset);
				if (!finite(vec(above)) || !finite(vec(below)))
					return nullptr;
				expanded.push_back(above);
				expanded.push_back(below);
			}
			return b3CreateHull(expanded.data(), int(expanded.size()), B3_MAX_HULL_VERTICES);
		}

		std::shared_ptr<Geometry> cook(const RigidBodyPhysicsComponent& component, const XMFLOAT3& scale,
			const MeshComponent* mesh, std::string& error)
		{
			if (!finite(scale) || scale.x <= 0 || scale.y <= 0 || scale.z <= 0)
			{
				error = "Collision scale must be finite and positive; mirrored/sheared collision geometry is unsupported";
				return {};
			}
			if (const char* reason = GetUnsupportedReason(component.shape))
			{
				error = reason;
				return {};
			}
			auto geometry = std::make_shared<Geometry>();
			geometry->kind = component.shape;
			geometry->key = geometry_key(component, scale, mesh);
			geometry->scale = scale;
			// Declared after geometry so the gate is released before failure destroys it.
			std::lock_guard<std::mutex> lock(cooking_mutex);
			auto positive = [](float value) { return std::isfinite(value) && value > 0; };
			switch (component.shape)
			{
			case RigidBodyPhysicsComponent::BOX:
			{
				const XMFLOAT3& e = component.box.halfextents;
				if (!finite(e) || !positive(e.x * scale.x) || !positive(e.y * scale.y) || !positive(e.z * scale.z))
				{ error = "Box half extents must be finite and positive"; return {}; }
				geometry->box = b3MakeBoxHull(e.x * scale.x, e.y * scale.y, e.z * scale.z);
				break;
			}
			case RigidBodyPhysicsComponent::SPHERE:
				if (!positive(component.sphere.radius * scale.x))
				{ error = "Sphere radius must be finite and positive"; return {}; }
				// Preserve the engine's radius convention (the local X scale).
				geometry->sphere = { b3Vec3_zero, component.sphere.radius * scale.x };
				break;
			case RigidBodyPhysicsComponent::CAPSULE:
			case RigidBodyPhysicsComponent::CYLINDER:
			{
				float radius = component.capsule.radius * scale.x;
				float height = component.capsule.height * scale.y;
				if (!positive(radius) || !std::isfinite(height) || height < 0 ||
					(component.shape == RigidBodyPhysicsComponent::CYLINDER && height == 0))
				{ error = "Capsule/cylinder radius or half height is invalid"; return {}; }
				if (component.shape == RigidBodyPhysicsComponent::CAPSULE)
					geometry->capsule = { { 0, -height, 0 }, { 0, height, 0 }, radius };
				else
				{
					// 32-sided cylinder, aligned with the engine's local Y axis.
					b3Vec3 points[64];
					for (int i = 0; i < 32; ++i)
					{
						const float angle = XM_2PI * float(i) / 32.0f;
						points[i] = { radius * std::cos(angle), -height, radius * std::sin(angle) };
						points[i + 32] = { points[i].x, height, points[i].z };
					}
					geometry->hull = b3CreateHull(points, 64, 64);
					if (!geometry->hull) { error = "Cylinder hull cooking failed"; return {}; }
				}
				break;
			}
			case RigidBodyPhysicsComponent::CONVEX_HULL:
			case RigidBodyPhysicsComponent::TRIANGLE_MESH:
			case RigidBodyPhysicsComponent::HEIGHTFIELD:
			{
				if (!mesh || mesh->vertex_positions.empty())
				{ error = "Mesh collision requested without nonempty mesh geometry"; return {}; }
				std::vector<b3Vec3> points;
				points.reserve(mesh->vertex_positions.size());
				for (const XMFLOAT3& p : mesh->vertex_positions)
				{
					if (!finite(p)) { error = "Collision mesh contains nonfinite vertices"; return {}; }
					const b3Vec3 point = { p.x * scale.x, p.y * scale.y, p.z * scale.z };
					if (!finite(vec(point))) { error = "Scaled collision vertices overflow"; return {}; }
					points.push_back(point);
				}
				if (component.shape == RigidBodyPhysicsComponent::CONVEX_HULL)
				{
					geometry->hull = b3CreateHull(points.data(), int(points.size()), B3_MAX_HULL_VERTICES);
					if (!geometry->hull)
					{
						geometry->hull = cook_planar_hull(points, geometry->planar_half_thickness);
						if (!geometry->hull) { error = "Convex hull cooking failed (degenerate or unsupported geometry)"; return {}; }
					}
					// Reject the builder's reduced hull if it would discard exterior geometry.
					const auto* planes = b3GetHullPlanes(geometry->hull);
					const float tolerance = std::max(1e-4f, b3Length(b3Sub(geometry->hull->aabb.upperBound, geometry->hull->aabb.lowerBound)) * 1e-5f);
					for (const auto& point : points)
						for (int i = 0; i < geometry->hull->faceCount; ++i)
							if (b3Dot(planes[i].normal, point) - planes[i].offset > tolerance)
							{ error = "Convex hull exceeds Box3D complexity/precision limits; exterior geometry would be discarded"; return {}; }
				}
				else if (component.shape == RigidBodyPhysicsComponent::TRIANGLE_MESH)
				{
					std::vector<int32_t> indices;
					auto append = [&](uint32_t begin, uint32_t count)
					{
						if (size_t(begin) + count > mesh->indices.size() || count % 3 != 0) return false;
						for (uint32_t i = begin; i < begin + count; ++i)
						{
							if (mesh->indices[i] >= points.size()) return false;
							indices.push_back(int32_t(mesh->indices[i]));
						}
						return true;
					};
					if (mesh->subsets.empty())
					{
						if (!append(0, uint32_t(mesh->indices.size())))
						{ error = "Collision mesh indices are invalid"; return {}; }
					}
					else
					{
						uint32_t first = 0, last = 0;
						mesh->GetLODSubsetRange(component.mesh_lod, first, last);
						if (first > last || last > mesh->subsets.size())
						{ error = "Collision mesh LOD is invalid"; return {}; }
						for (uint32_t i = first; i < last; ++i)
							if (!append(mesh->subsets[i].indexOffset, mesh->subsets[i].indexCount))
							{ error = "Collision mesh subset indices are invalid"; return {}; }
					}
					if (indices.empty()) { error = "Collision mesh LOD contains no triangles"; return {}; }
					b3MeshDef def{};
					def.vertices = points.data();
					def.indices = indices.data();
					def.vertexCount = int(points.size());
					def.triangleCount = int(indices.size() / 3);
					def.clockWiseWinding = true; // Wicked mesh indices are clockwise.
					def.identifyEdges = true;
					geometry->mesh = b3CreateMesh(&def, nullptr, 0);
					if (!geometry->mesh) { error = "Triangle mesh cooking failed"; return {}; }
				}
				else
				{
					// Recover grid coordinates rather than assuming vertex storage order.
					std::vector<float> xs, zs;
					for (const auto& p : points) { xs.push_back(p.x); zs.push_back(p.z); }
					auto unique = [](std::vector<float>& values)
					{
						std::sort(values.begin(), values.end());
						values.erase(std::unique(values.begin(), values.end()), values.end());
					};
					unique(xs); unique(zs);
					if (xs.size() < 2 || zs.size() < 2 || xs.size() * zs.size() != points.size())
					{ error = "Heightfield requires a complete rectangular X/Z grid"; return {}; }
					const float dx = (xs.back() - xs.front()) / float(xs.size() - 1);
					const float dz = (zs.back() - zs.front()) / float(zs.size() - 1);
					if (!positive(dx) || !positive(dz))
					{ error = "Heightfield grid spacing overflow"; return {}; }
					for (size_t i = 0; i < xs.size(); ++i)
						if (std::abs(xs[i] - (xs.front() + dx * float(i))) > std::max(1e-4f, dx * 1e-4f))
						{ error = "Heightfield X spacing is not uniform"; return {}; }
					for (size_t i = 0; i < zs.size(); ++i)
						if (std::abs(zs[i] - (zs.front() + dz * float(i))) > std::max(1e-4f, dz * 1e-4f))
						{ error = "Heightfield Z spacing is not uniform"; return {}; }
					std::vector<float> heights(points.size());
					std::vector<bool> assigned(points.size(), false);
					float low = FLT_MAX, high = -FLT_MAX;
					for (const auto& p : points)
					{
						const size_t x = size_t(std::lower_bound(xs.begin(), xs.end(), p.x) - xs.begin());
						const size_t z = size_t(std::lower_bound(zs.begin(), zs.end(), p.z) - zs.begin());
						const size_t index = z * xs.size() + x;
						if (assigned[index]) { error = "Heightfield contains duplicate grid vertices"; return {}; }
						assigned[index] = true;
						heights[index] = p.y;
						low = std::min(low, p.y); high = std::max(high, p.y);
					}
					b3HeightFieldDef def{};
					def.heights = heights.data();
					def.scale = { dx, 1, dz };
					def.countX = int(xs.size()); def.countZ = int(zs.size());
					def.globalMinimumHeight = low;
					def.globalMaximumHeight = std::max(high, low + 0.001f);
					geometry->height = b3CreateHeightField(&def);
					geometry->origin = { xs.front(), 0, zs.front() };
					if (!geometry->height) { error = "Heightfield cooking failed"; return {}; }
				}
				break;
			}
			default: error = "Unknown collision shape"; return {};
			}
			return geometry;
		}
		b3WorldTransform engine_pose(const Body& body, b3WorldTransform pose)
		{
			pose.p = b3Sub(pose.p, b3RotateVector(pose.q, body.offset));
			return pose;
		}
		b3WorldTransform body_pose(const Body& body, b3WorldTransform pose)
		{
			pose.p = b3Add(pose.p, b3RotateVector(pose.q, body.offset));
			return pose;
		}
		bool teleport(Body& body, b3WorldTransform pose)
		{
			if (!finite(vec(pose.p)))
			{
				wilog_error("Box3D: teleport offset position overflow");
				return false;
			}
			b3Body_SetTransform(body.id, pose.p, pose.q);
			body.previous = body.current = pose;
			body.teleported = true;
			b3Body_SetAwake(body.id, true);
			return true;
		}
		size_t hierarchy_depth(const Scene& scene, Entity entity)
		{
			size_t depth = 0;
			while (const auto* hierarchy = scene.hierarchy.GetComponent(entity))
			{
				if (hierarchy->parentID == INVALID_ENTITY || depth > scene.hierarchy.GetCount()) break;
				entity = hierarchy->parentID;
				++depth;
			}
			return depth;
		}
		void properties(Body& body, const RigidBodyPhysicsComponent& component)
		{
			const b3BodyType type = body_type(component);
			const bool changed = type != b3Body_GetType(body.id);
			if (changed) b3Body_SetType(body.id, type);
			b3MotionLocks locks{};
			if (component.IsLocked2D()) { locks.linearZ = true; locks.angularX = true; locks.angularY = true; }
			locks.angularX |= component.IsRotationLockedX();
			locks.angularY |= component.IsRotationLockedY();
			locks.angularZ |= component.IsRotationLockedZ();
			const b3MotionLocks previous = b3Body_GetMotionLocks(body.id);
			const bool locks_changed = previous.linearZ != locks.linearZ ||
				previous.angularX != locks.angularX || previous.angularY != locks.angularY || previous.angularZ != locks.angularZ;
			b3Body_SetMotionLocks(body.id, locks);
			// Changing fixed rotation recomputes density-based mass in Box3D.
			// Apply the engine's authored mass after changing motion locks.
			if (changed || locks_changed || body.authored_mass != component.mass)
			{
				b3Body_ApplyMassFromShapes(body.id);
				if (type == b3_dynamicBody)
				{
					b3MassData mass = b3Body_GetMassData(body.id);
					if (mass.mass > 0)
					{
						mass.inertia = b3MulSM(component.mass / mass.mass, mass.inertia);
						mass.mass = component.mass;
						b3Body_SetMassData(body.id, mass);
					}
				}
				body.authored_mass = component.mass;
			}
			b3Shape_SetFriction(body.shape, component.friction);
			b3Shape_SetRestitution(body.shape, component.restitution);
			b3Body_SetLinearDamping(body.id, component.damping_linear);
			b3Body_SetAngularDamping(body.id, component.damping_angular);
			b3Body_EnableSleep(body.id, !component.IsDisableDeactivation());
		}
		bool valid_properties(const RigidBodyPhysicsComponent& component)
		{
			return std::isfinite(component.mass) && component.mass >= 0 &&
				std::isfinite(component.friction) && component.friction >= 0 &&
				std::isfinite(component.restitution) && component.restitution >= 0 &&
				std::isfinite(component.damping_linear) && component.damping_linear >= 0 &&
				std::isfinite(component.damping_angular) && component.damping_angular >= 0 && finite(component.local_offset);
		}
		template<typename F> void with_body(RigidBodyPhysicsComponent& component, const char* operation, F&& fn)
		{
			BodyState* s = state(component);
			if (!s || !s->world || !s->body)
			{
				if (component.IsCharacterPhysics()) { unsupported(operation, Feature::Characters); return; }
				if (component.IsVehicle()) { unsupported(operation, Feature::Vehicles); return; }
				wilog_error("Box3D: %s requires a registered rigid body", operation);
				return;
			}
			auto world = s->world;
			auto body = s->body;
			std::lock_guard<std::mutex> lock(world->gate);
			// Refresh flag writes share this gate with reads of the component's packed flags.
			if (component.IsCharacterPhysics()) { unsupported(operation, Feature::Characters); return; }
			if (component.IsVehicle()) { unsupported(operation, Feature::Vehicles); return; }
			world->Drain();
			if (body->released.load() || !b3Body_IsValid(body->id))
			{
				wilog_error("Box3D: %s rejected a stale rigid body", operation);
				return;
			}
			fn(*body);
		}
		float body_distance(b3BodyId a, b3BodyId b)
		{
			return b3Length(b3Sub(b3Body_GetWorldCenter(a), b3Body_GetWorldCenter(b)));
		}
		b3JointId create_joint(World& world, Joint& joint, const PhysicsConstraintComponent& component)
		{
			auto base = [&](b3JointDef& def)
			{
				def.bodyIdA = joint.a; def.bodyIdB = joint.b;
				def.localFrameA = joint.frame_a; def.localFrameB = joint.frame_b;
				def.collideConnected = !component.IsDisableSelfCollision();
				def.drawScale = constraint_size.load();
			};
			switch (component.type)
			{
			case PhysicsConstraintComponent::Type::Fixed:
			{
				auto def = b3DefaultWeldJointDef(); base(def.base);
				return b3CreateWeldJoint(world.id, &def);
			}
			case PhysicsConstraintComponent::Type::Distance:
			{
				if (component.distance_constraint.max_distance == 0)
				{
					auto def = b3DefaultSphericalJointDef(); base(def.base);
					def.enableConeLimit = false; def.enableTwistLimit = false;
					return b3CreateSphericalJoint(world.id, &def);
				}
				auto def = b3DefaultDistanceJointDef(); base(def.base);
				def.length = std::max(0.001f, component.distance_constraint.min_distance);
				def.minLength = component.distance_constraint.min_distance;
				def.maxLength = component.distance_constraint.max_distance;
				def.enableSpring = def.minLength != def.maxLength;
				def.enableLimit = def.enableSpring;
				def.hertz = 0;
				return b3CreateDistanceJoint(world.id, &def);
			}
			case PhysicsConstraintComponent::Type::Hinge:
			{
				auto def = b3DefaultRevoluteJointDef(); base(def.base);
				def.enableLimit = component.hinge_constraint.min_angle > -XM_PI || component.hinge_constraint.max_angle < XM_PI;
				def.lowerAngle = component.hinge_constraint.min_angle;
				def.upperAngle = component.hinge_constraint.max_angle;
				def.motorSpeed = component.hinge_constraint.target_angular_velocity;
				def.enableMotor = def.motorSpeed != 0;
				def.maxMotorTorque = FLT_MAX;
				return b3CreateRevoluteJoint(world.id, &def);
			}
			case PhysicsConstraintComponent::Type::Slider:
			{
				auto def = b3DefaultPrismaticJointDef(); base(def.base);
				def.enableLimit = component.slider_constraint.min_limit != -FLT_MAX || component.slider_constraint.max_limit != FLT_MAX;
				def.lowerTranslation = component.slider_constraint.min_limit;
				def.upperTranslation = component.slider_constraint.max_limit;
				def.motorSpeed = component.slider_constraint.target_velocity;
				def.enableMotor = def.motorSpeed != 0;
				def.maxMotorForce = component.slider_constraint.max_force;
				return b3CreatePrismaticJoint(world.id, &def);
			}
			case PhysicsConstraintComponent::Type::Point:
			case PhysicsConstraintComponent::Type::Cone:
			case PhysicsConstraintComponent::Type::SwingTwist:
			{
				auto def = b3DefaultSphericalJointDef(); base(def.base);
				def.enableConeLimit = component.type != PhysicsConstraintComponent::Type::Point;
				def.coneAngle = component.type == PhysicsConstraintComponent::Type::Cone ?
					component.cone_constraint.half_cone_angle : component.type == PhysicsConstraintComponent::Type::SwingTwist ?
					component.swing_twist.normal_half_cone_angle : 0;
				def.enableTwistLimit = component.type == PhysicsConstraintComponent::Type::SwingTwist;
				def.lowerTwistAngle = def.enableTwistLimit ? component.swing_twist.min_twist_angle : 0;
				def.upperTwistAngle = def.enableTwistLimit ? component.swing_twist.max_twist_angle : 0;
				if (def.coneAngle == XM_PI)
				{
					def.enableConeLimit = false;
					def.coneAngle = 0;
				}
				if (def.lowerTwistAngle == -XM_PI && def.upperTwistAngle == XM_PI)
					def.enableTwistLimit = false;
				return b3CreateSphericalJoint(world.id, &def);
			}
			default: return {};
			}
		}
		const char* constraint_error(const PhysicsConstraintComponent& component)
		{
			// The enum-level capability is conservative; circular SwingTwist is checked below.
			if (component.type != PhysicsConstraintComponent::Type::SwingTwist)
				if (const char* reason = GetUnsupportedReason(component.type)) return reason;
			if (!std::isfinite(component.break_distance) || component.break_distance < 0) return "Constraint break distance is invalid";
			auto range = [](float low, float high) { return std::isfinite(low) && std::isfinite(high) && low <= high; };
			switch (component.type)
			{
			case PhysicsConstraintComponent::Type::Distance:
				if (!range(component.distance_constraint.min_distance, component.distance_constraint.max_distance) ||
					component.distance_constraint.min_distance < 0) return "Distance limits are invalid";
				break;
			case PhysicsConstraintComponent::Type::Hinge:
				if (!range(component.hinge_constraint.min_angle, component.hinge_constraint.max_angle) ||
					component.hinge_constraint.min_angle < -XM_PI || component.hinge_constraint.max_angle > XM_PI ||
					!std::isfinite(component.hinge_constraint.target_angular_velocity)) return "Hinge limits/motor velocity are invalid";
				// Box3D cannot represent finite limits in the outer 1% of the circle.
				if ((component.hinge_constraint.min_angle != -XM_PI || component.hinge_constraint.max_angle != XM_PI) &&
					(component.hinge_constraint.min_angle < -0.99f * XM_PI || component.hinge_constraint.max_angle > 0.99f * XM_PI))
					return "Box3D finite hinge limits must lie in [-0.99*pi, 0.99*pi]";
				break;
			case PhysicsConstraintComponent::Type::Cone:
				if (!range(0, component.cone_constraint.half_cone_angle) || component.cone_constraint.half_cone_angle > XM_PI)
					return "Cone half angle must be in [0, pi]";
				if (component.cone_constraint.half_cone_angle > XM_PIDIV2 && component.cone_constraint.half_cone_angle != XM_PI)
					return "Box3D cone limits support half angles up to pi/2, or pi (unlimited), without approximation";
				break;
			case PhysicsConstraintComponent::Type::SwingTwist:
				if (component.swing_twist.normal_half_cone_angle != component.swing_twist.plane_half_cone_angle)
					return "Box3D SwingTwist requires exactly equal circular swing cone angles";
				if (!range(0, component.swing_twist.normal_half_cone_angle) || component.swing_twist.normal_half_cone_angle > XM_PI ||
					!range(component.swing_twist.min_twist_angle, component.swing_twist.max_twist_angle))
					return "SwingTwist angles are outside Box3D's supported ranges";
				if (component.swing_twist.normal_half_cone_angle > XM_PIDIV2 && component.swing_twist.normal_half_cone_angle != XM_PI)
					return "Box3D circular swing limits support half angles up to pi/2, or pi (unlimited)";
				if ((component.swing_twist.min_twist_angle != -XM_PI || component.swing_twist.max_twist_angle != XM_PI) &&
					(component.swing_twist.min_twist_angle < -0.99f * XM_PI || component.swing_twist.max_twist_angle > 0.99f * XM_PI))
					return "Box3D finite twist limits must lie in [-0.99*pi, 0.99*pi], or be fully unlimited";
				break;
			case PhysicsConstraintComponent::Type::Slider:
				if (!range(component.slider_constraint.min_limit, component.slider_constraint.max_limit) ||
					!std::isfinite(component.slider_constraint.target_velocity) ||
					!std::isfinite(component.slider_constraint.max_force) || component.slider_constraint.max_force < 0)
					return "Slider limits/motor parameters are invalid";
				break;
			default: break;
			}
			return nullptr;
		}
		uint64_t constraint_key(const PhysicsConstraintComponent& component)
		{
			uint64_t hash = 14695981039346656037ull;
			hash_value(hash, component.type);
			hash_value(hash, component.bodyA); hash_value(hash, component.bodyB);
			hash_value(hash, component._flags);
			hash_value(hash, component.distance_constraint); hash_value(hash, component.hinge_constraint);
			hash_value(hash, component.cone_constraint); hash_value(hash, component.swing_twist);
			hash_value(hash, component.slider_constraint); hash_value(hash, component.break_distance);
			return hash;
		}
		b3BodyId anchor(World& world, b3WorldTransform pose, bool kinematic = false)
		{
			auto def = b3DefaultBodyDef();
			def.type = kinematic ? b3_kinematicBody : b3_staticBody;
			def.position = pose.p; def.rotation = pose.q;
			return b3CreateBody(world.id, &def); // Shapeless: no contacts and no query hits.
		}
		b3Quat joint_rotation(PhysicsConstraintComponent::Type type, b3Quat rotation)
		{
			// Wicked's authored UP is Box3D's joint Z; slider RIGHT already matches X.
			if (type == PhysicsConstraintComponent::Type::Hinge || type == PhysicsConstraintComponent::Type::Cone ||
				type == PhysicsConstraintComponent::Type::SwingTwist)
			{
				const float s = std::sqrt(0.5f);
				return b3MulQuat(rotation, b3Quat{ { -s, 0, 0 }, s });
			}
			return rotation;
		}
		XMFLOAT4 debug_color(b3HexColor color)
		{
			const uint32_t rgb = uint32_t(color);
			return XMFLOAT4(float((rgb >> 16) & 255) / 255, float((rgb >> 8) & 255) / 255, float(rgb & 255) / 255, 1);
		}
		void debug_line(b3Pos a, b3Pos b, b3HexColor color, void*)
		{
			wi::renderer::RenderableLine line;
			line.start = vec(a); line.end = vec(b);
			line.color_start = line.color_end = debug_color(color);
			wi::renderer::DrawLine(line, true);
		}
		void debug_bounds(b3AABB bounds, b3HexColor color, void*)
		{
			wi::renderer::DrawBox(wi::primitive::AABB(vec(bounds.lowerBound), vec(bounds.upperBound)), debug_color(color));
		}
		void debug_point(b3Pos point, float size, b3HexColor color, void*)
		{
			wi::renderer::DrawSphere(wi::primitive::Sphere(vec(point), size * 0.001f), debug_color(color));
		}
		void draw_debug(World& world, const Scene& scene)
		{
			const b3Vec3 eye = vec(scene.camera.Eye);
			const float distance = debug_distance.load();
			for (const auto& body : world.bodies)
			{
				const auto pose = b3Body_GetTransform(body->id);
				if (b3Length(b3Sub(pose.p, eye)) > distance) continue;
				const auto& geometry = *body->geometry;
				const b3HexColor color = b3Body_IsAwake(body->id) ? b3_colorLime : b3_colorGray;
				if (geometry.kind == RigidBodyPhysicsComponent::SPHERE)
				{
					wi::renderer::DrawSphere(wi::primitive::Sphere(vec(pose.p), geometry.sphere.radius), debug_color(color));
					continue;
				}
				const b3HullData* hull = geometry.kind == RigidBodyPhysicsComponent::BOX ? &geometry.box.base : geometry.hull;
				if (hull)
				{
					const auto* points = b3GetHullPoints(hull);
					const auto* edges = b3GetHullEdges(hull);
					for (int i = 0; i < hull->edgeCount; ++i)
					{
						if (i > edges[i].twin) continue;
						debug_line(b3TransformPoint(pose, points[edges[i].origin]),
							b3TransformPoint(pose, points[edges[edges[i].twin].origin]), color, nullptr);
					}
				}
				else debug_bounds(b3Body_ComputeAABB(body->id), color, nullptr);
			}
			b3DebugDraw draw = b3DefaultDebugDraw();
			draw.drawShapes = false; draw.drawJoints = true;
			draw.drawJointExtras = true;
			draw.jointScale = constraint_size.load();
			draw.DrawSegmentFcn = debug_line;
			draw.DrawPointFcn = debug_point;
			draw.DrawBoundsFcn = debug_bounds;
			draw.drawingBounds = { b3Sub(eye, b3Vec3{ distance, distance, distance }), b3Add(eye, b3Vec3{ distance, distance, distance }) };
			b3World_Draw(world.id, &draw, B3_DEFAULT_MASK_BITS);
		}
		RayIntersectionResult cast_ray(World& world, wi::primitive::Ray ray, Entity ignore_entity = INVALID_ENTITY)
		{
			RayIntersectionResult result;
			if (!b3World_IsValid(world.id)) return result;
			if (!finite(ray.origin) || !finite(ray.direction) || !std::isfinite(ray.TMin) ||
				!std::isfinite(ray.TMax) || ray.TMin < 0 || ray.TMax <= ray.TMin)
			{
				wilog_error("Box3D: ray bounds/origin/direction are invalid");
				return result;
			}
			const b3Vec3 direction = vec(ray.direction);
			if (b3LengthSquared(direction) <= 1e-12f) return result;
			// FLT_MAX translations overflow BVH arithmetic. Bound infinite rays by world bounds.
			float end = ray.TMax;
			const b3Vec3 start = b3MulAdd(vec(ray.origin), ray.TMin, direction);
			if (!finite(vec(start))) { wilog_error("Box3D: ray starting position overflow"); return result; }
			if (end == FLT_MAX)
			{
				const b3AABB bounds = b3World_GetBounds(world.id);
				const float extent = std::max(b3Length(b3Sub(bounds.lowerBound, start)), b3Length(b3Sub(bounds.upperBound, start)));
				end = ray.TMin + (extent + 1) / b3Length(direction);
			}
			b3Vec3 translation = b3MulSV(end - ray.TMin, direction);
			if (!finite(vec(translation))) { wilog_error("Box3D: ray translation overflow"); return result; }
			struct Query
			{
				Entity ignore;
				b3RayResult hit{};
			} query{ ignore_entity };
			b3World_CastRay(world.id, start, translation, b3DefaultQueryFilter(),
				[](b3ShapeId shape, b3Pos point, b3Vec3 normal, float fraction, uint64_t, int, int, void* context) -> float {
					auto& query = *static_cast<Query*>(context);
					auto* body = static_cast<Body*>(b3Body_GetUserData(b3Shape_GetBody(shape)));
					if (!body || body->released.load() || (query.ignore != INVALID_ENTITY && body->entity == query.ignore))
						return -1;
					query.hit.hit = true;
					query.hit.shapeId = shape;
					query.hit.point = point;
					query.hit.normal = normal;
					query.hit.fraction = fraction;
					return fraction;
				}, &query);
			const b3RayResult& hit = query.hit;
			if (!hit.hit) return result;
			const b3BodyId id = b3Shape_GetBody(hit.shapeId);
			auto* body = static_cast<Body*>(b3Body_GetUserData(id));
			if (!body || body->released.load()) return result;
			result.entity = body->entity;
			result.position = vec(hit.point); result.normal = vec(hit.normal);
			// Public local force points, like Jolt, are relative to the center of mass.
			result.position_local = vec(b3InvRotateVector(b3Body_GetRotation(id), b3Sub(hit.point, b3Body_GetWorldCenter(id))));
			result.physicsobject = body;
			return result;
		}
	}

	const char* GetBackendName() { return "Box3D"; }
	const char* GetUnsupportedReason(Feature feature)
	{
		switch (feature)
		{
		case Feature::Characters: return "Box3D character controller integration is not implemented";
		case Feature::Ragdolls: return "Box3D humanoid ragdoll integration is not implemented";
		case Feature::Vehicles: return "Box3D vehicle controller integration is not implemented";
		case Feature::SoftBodies: return "Box3D does not provide the engine's soft body solver";
		case Feature::Buoyancy: return "Box3D buoyancy integration is not implemented";
		default: return "Unknown Box3D feature";
		}
	}
	const char* GetUnsupportedReason(RigidBodyPhysicsComponent::CollisionShape shape)
	{
		switch (shape)
		{
		case RigidBodyPhysicsComponent::BOX: case RigidBodyPhysicsComponent::SPHERE:
		case RigidBodyPhysicsComponent::CAPSULE: case RigidBodyPhysicsComponent::CYLINDER:
		case RigidBodyPhysicsComponent::CONVEX_HULL: case RigidBodyPhysicsComponent::TRIANGLE_MESH:
		case RigidBodyPhysicsComponent::HEIGHTFIELD: return nullptr;
		default: return "Unknown collision shape";
		}
	}
	const char* GetUnsupportedReason(PhysicsConstraintComponent::Type type)
	{
		switch (type)
		{
		case PhysicsConstraintComponent::Type::Fixed: case PhysicsConstraintComponent::Type::Point:
		case PhysicsConstraintComponent::Type::Distance: case PhysicsConstraintComponent::Type::Hinge:
		case PhysicsConstraintComponent::Type::Slider: case PhysicsConstraintComponent::Type::Cone: return nullptr;
		case PhysicsConstraintComponent::Type::SwingTwist:
			return "Box3D supports only exactly circular SwingTwist cones with supported angle ranges; general SwingTwist is unavailable";
		case PhysicsConstraintComponent::Type::SixDOF: return "Box3D SixDOF integration is not implemented";
		default: return "Unknown constraint type";
		}
	}
	void Initialize()
	{
		wilog("Physics backend: Box3D (a2e4816b4359b32408d5623784a7fe2d7259d3d6), serialized worlds, built-in scheduler");
	}
	void SetEnabled(bool value) { enabled.store(value); }
	bool IsEnabled() { return enabled.load(); }
	void SetSimulationEnabled(bool value) { simulation.store(value); }
	bool IsSimulationEnabled() { return enabled.load() && simulation.load(); }
	void SetInterpolationEnabled(bool value) { interpolation.store(value); }
	bool IsInterpolationEnabled() { return interpolation.load(); }
	void SetDebugDrawEnabled(bool value) { debugdraw.store(value); }
	bool IsDebugDrawEnabled() { return debugdraw.load(); }
	void SetConstraintDebugSize(float value)
	{
		if (!std::isfinite(value) || value <= 0) { wilog_error("Box3D: constraint debug size must be positive"); return; }
		constraint_size.store(value);
	}
	float GetConstraintDebugSize() { return constraint_size.load(); }
	void SetDebugDrawMaxDistance(float value)
	{
		if (!std::isfinite(value) || value <= 0) { wilog_error("Box3D: debug draw distance must be positive"); return; }
		debug_distance.store(value);
	}
	float GetDebugDrawMaxDistance() { return debug_distance.load(); }
	void SetAccuracy(int value)
	{
		if (value <= 0) { wilog_error("Box3D: maximum catch-up step count must be positive"); return; }
		accuracy.store(value);
	}
	int GetAccuracy() { return accuracy.load(); }
	void SetFrameRate(float value)
	{
		if (!std::isfinite(value) || value <= 0 || !std::isfinite(1.0f / value))
		{ wilog_error("Box3D: frame rate must be finite and positive"); return; }
		timestep.store(1.0f / value);
	}
	float GetFrameRate() { return 1.0f / timestep.load(); }
	void SetCharacterCollisionTolerance(float value)
	{
		unsupported("SetCharacterCollisionTolerance", Feature::Characters);
		(void)value;
	}
	float GetCharacterCollisionTolerance()
	{
		unsupported("GetCharacterCollisionTolerance", Feature::Characters);
		return character_tolerance.load();
	}

	void CreateRigidBodyShape(RigidBodyPhysicsComponent& component, const XMFLOAT3& scale, const MeshComponent* mesh)
	{
		{
			const auto* existing = state(component);
			const auto world = existing ? existing->world : std::shared_ptr<World>{};
			std::unique_lock<std::mutex> lock;
			if (world) lock = std::unique_lock<std::mutex>(world->gate);
			if (component.IsCharacterPhysics()) { unsupported("CreateRigidBodyShape", Feature::Characters); return; }
			if (component.IsVehicle()) { unsupported("CreateRigidBodyShape", Feature::Vehicles); return; }
		}
		std::string error;
		auto geometry = cook(component, scale, mesh, error);
		if (!geometry) { wilog_error("Box3D: CreateRigidBodyShape: %s", error.c_str()); return; }
		if (!component.physicsobject) component.physicsobject = wi::allocator::make_shared<BodyState>();
		auto* s = state(component);
		if (s->world)
		{
			std::lock_guard<std::mutex> lock(s->world->gate);
			s->prepared = std::move(geometry);
		}
		else s->prepared = std::move(geometry);
	}
	void FlushDeferredDeletes(Scene& scene)
	{
		auto world = get_world(scene);
		if (!world) return;
		std::lock_guard<std::mutex> lock(world->gate);
		world->Drain();
	}

	void RunPhysicsUpdateSystem(wi::jobsystem::context& ctx, Scene& scene, float dt)
	{
		// Never wait for engine jobs while holding a world gate.
		wi::jobsystem::Wait(ctx);
		if (!scene.physics_scene)
		{
			if (!IsEnabled()) return;
			scene.physics_scene = wi::allocator::make_shared_single<SceneState>();
		}
		auto world = get_world(scene);
		// Releasing a wrapper can destroy its old world. Do this before taking the new
		// world's gate, including when components were moved out of a cleared scene.
		for (size_t i = 0; i < scene.rigidbodies.GetCount(); ++i)
		{
			const auto* s = state(scene.rigidbodies[i]);
			if (s && s->world && s->world != world) scene.rigidbodies[i].physicsobject.reset();
		}
		for (size_t i = 0; i < scene.constraints.GetCount(); ++i)
		{
			const auto* s = state(scene.constraints[i]);
			if (s && s->world && s->world != world) scene.constraints[i].physicsobject.reset();
		}
		std::lock_guard<std::mutex> lock(world->gate);
		if (!b3World_IsValid(world->id)) { wilog_error("Box3D: world creation failed"); return; }
		world->Drain();
		if (!IsEnabled()) { world->accumulator = 0; return; }
		const float step = timestep.load();
		const bool running = IsSimulationEnabled() && std::isfinite(dt) && dt > 0;
		if (finite(scene.weather.gravity))
		{
			b3World_SetGravity(world->id, vec(scene.weather.gravity));
			world->diagnostics[INVALID_ENTITY][5].clear();
		}
		else world->Diagnose(INVALID_ENTITY, 0x47524156, "Scene gravity is nonfinite", 5);

		// Deterministic serialized commit order, independent of ECS storage order.
		std::vector<size_t> order(scene.rigidbodies.GetCount());
		for (size_t i = 0; i < order.size(); ++i) order[i] = i;
		std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return scene.rigidbodies.GetEntity(a) < scene.rigidbodies.GetEntity(b); });
		for (size_t index : order)
		{
			auto& component = scene.rigidbodies[index];
			const Entity entity = scene.rigidbodies.GetEntity(index);
			auto* transform = scene.transforms.GetComponent(entity);
			if (!transform)
			{
				component.physicsobject.reset();
				world->Diagnose(entity, 0, "Rigid body requires a TransformComponent");
				continue;
			}
			XMVECTOR scale_v, rotation_v, position_v;
			const XMMATRIX parent = scene.ComputeParentMatrixRecursive(entity);
			const XMMATRIX matrix = transform->GetLocalMatrix() * parent;
			XMFLOAT3 scale, position;
			XMFLOAT4 rotation;
			if (!XMMatrixDecompose(&scale_v, &rotation_v, &position_v, matrix))
			{
				component.physicsobject.reset();
				world->Diagnose(entity, 0, "World transform cannot be decomposed (shear or singular scale)");
				continue;
			}
			XMStoreFloat3(&scale, scale_v); XMStoreFloat3(&position, position_v); XMStoreFloat4(&rotation, rotation_v);
			if (const auto* existing = state(component))
			{
				const auto geometry = existing->body ? existing->body->geometry : existing->prepared;
				if (geometry)
				{
					auto near = [](float a, float b)
					{
						return std::abs(a - b) <= 1e-5f * std::max({ 1.0f, std::abs(a), std::abs(b) });
					};
					// Matrix decomposition introduces tiny scale drift as rotations change.
					// Do not recook/recreate bodies and dependent joints for that noise.
					if (near(scale.x, geometry->scale.x) && near(scale.y, geometry->scale.y) && near(scale.z, geometry->scale.z))
						scale = geometry->scale;
				}
			}
			const MeshComponent* mesh = scene.meshes.GetComponent(entity);
			if (const auto* object = scene.objects.GetComponent(entity)) mesh = scene.meshes.GetComponent(object->meshID);
			uint64_t key = geometry_key(component, scale, mesh);
			uint64_t configuration = key;
			hash_value(configuration, component.mass); hash_value(configuration, component._flags);
			hash_value(configuration, component.vehicle.type);
			hash_value(configuration, component.friction); hash_value(configuration, component.restitution);
			hash_value(configuration, component.damping_linear); hash_value(configuration, component.damping_angular);
			hash_value(configuration, component.local_offset);
			const char* reason = nullptr;
			if (component.IsCharacterPhysics()) reason = GetUnsupportedReason(Feature::Characters);
			else if (component.IsVehicle()) reason = GetUnsupportedReason(Feature::Vehicles);
			else if (!valid_properties(component)) reason = "Rigid body mass/material/damping/offset parameters are invalid";
			else if ((component.shape == RigidBodyPhysicsComponent::TRIANGLE_MESH || component.shape == RigidBodyPhysicsComponent::HEIGHTFIELD) &&
				body_type(component) != b3_staticBody) reason = "Triangle mesh and heightfield collision requires a static (mass zero) body";
			else if (!finite(position) || !valid_rotation(rotation)) reason = "Rigid body pose is invalid";
			if (reason)
			{
				component.physicsobject.reset();
				world->Diagnose(entity, configuration, reason);
				continue;
			}
			if (!component.physicsobject) component.physicsobject = wi::allocator::make_shared<BodyState>();
			auto* s = state(component);
			std::shared_ptr<Geometry> geometry;
			if (s->body && s->body->geometry->key == key) geometry = s->body->geometry;
			else if (s->prepared && s->prepared->key == key) geometry = s->prepared;
			else
			{
				auto found = world->geometry_cache.find(key);
				if (found != world->geometry_cache.end()) geometry = found->second.lock();
				if (!geometry)
				{
					std::string error;
					geometry = cook(component, scale, mesh, error);
					if (!geometry)
					{
						if (s->body) s->body->released.store(true);
						s->body.reset();
						world->Diagnose(entity, configuration, error.c_str());
						continue;
					}
					world->geometry_cache[key] = geometry;
				}
			}
			if (geometry->planar_half_thickness > 0)
			{
				char message[160];
				std::snprintf(message, sizeof(message),
					"Planar convex hull uses %.6g units of collision thickness; authored geometry is unchanged",
					double(geometry->planar_half_thickness * 2));
				world->Diagnose(entity, key, message, 6, true);
			}
			b3WorldTransform authored = { vec(position), normalized(rotation) };
			if (!s->body || !b3Body_IsValid(s->body->id) || s->body->geometry != geometry)
			{
				b3WorldTransform initial = authored;
				b3Vec3 linear_velocity = b3Vec3_zero, angular_velocity = b3Vec3_zero;
				bool awake = !component.IsStartDeactivated();
				bool ghost = false;
				if (s->body)
				{
					initial = s->body->initial;
					ghost = s->body->ghost;
					if (b3Body_IsValid(s->body->id))
					{
						linear_velocity = b3Body_GetLinearVelocity(s->body->id);
						angular_velocity = b3Body_GetAngularVelocity(s->body->id);
						awake = b3Body_IsAwake(s->body->id);
						if (body_type(component) == b3_dynamicBody && running)
							authored = engine_pose(*s->body, b3Body_GetTransform(s->body->id));
					}
					s->body->released.store(true);
					world->Drain();
				}
				auto body = std::make_shared<Body>();
				body->entity = entity; body->geometry = geometry; body->initial = initial; body->ghost = ghost;
				body->offset = b3Add(vec(component.local_offset), geometry->origin);
				body->previous = body->current = body_pose(*body, authored);
				if (!finite(vec(body->current.p)))
				{ world->Diagnose(entity, configuration, "Rigid body offset position overflow"); continue; }
				auto def = b3DefaultBodyDef();
				def.type = body_type(component);
				def.position = body->current.p; def.rotation = body->current.q;
				def.userData = body.get();
				def.enableSleep = !component.IsDisableDeactivation();
				def.isAwake = awake;
				def.linearVelocity = linear_velocity; def.angularVelocity = angular_velocity;
				body->id = b3CreateBody(world->id, &def);
				if (!b3Body_IsValid(body->id))
				{ world->Diagnose(entity, configuration, "Rigid body allocation failed"); continue; }
				auto shape_def = b3DefaultShapeDef();
				shape_def.baseMaterial.friction = component.friction;
				shape_def.baseMaterial.restitution = component.restitution;
				if (ghost) shape_def.filter.maskBits = 0;
				switch (geometry->kind)
				{
				case RigidBodyPhysicsComponent::BOX: body->shape = b3CreateHullShape(body->id, &shape_def, &geometry->box.base); break;
				case RigidBodyPhysicsComponent::SPHERE: body->shape = b3CreateSphereShape(body->id, &shape_def, &geometry->sphere); break;
				case RigidBodyPhysicsComponent::CAPSULE:
					if (component.capsule.height == 0)
					{
						const b3Sphere sphere = { b3Vec3_zero, geometry->capsule.radius };
						body->shape = b3CreateSphereShape(body->id, &shape_def, &sphere);
					}
					else body->shape = b3CreateCapsuleShape(body->id, &shape_def, &geometry->capsule);
					break;
				case RigidBodyPhysicsComponent::CYLINDER: case RigidBodyPhysicsComponent::CONVEX_HULL:
					body->shape = b3CreateHullShape(body->id, &shape_def, geometry->hull); break;
				case RigidBodyPhysicsComponent::TRIANGLE_MESH:
					body->shape = b3CreateMeshShape(body->id, &shape_def, geometry->mesh, b3Vec3_one); break;
				case RigidBodyPhysicsComponent::HEIGHTFIELD:
					body->shape = b3CreateHeightFieldShape(body->id, &shape_def, geometry->height); break;
				default: break;
				}
				if (!b3Shape_IsValid(body->shape))
				{
					b3DestroyBody(body->id);
					world->Diagnose(entity, configuration, "Rigid body shape allocation failed");
					continue;
				}
				s->world = world; s->body = body; s->prepared.reset();
				world->bodies.push_back(body);
			}
			auto& body = *s->body;
			properties(body, component);
			const b3Vec3 offset = b3Add(vec(component.local_offset), geometry->origin);
			if (offset.x != body.offset.x || offset.y != body.offset.y || offset.z != body.offset.z)
			{
				const auto pose = engine_pose(body, b3Body_GetTransform(body.id));
				const auto old_offset = body.offset;
				body.offset = offset;
				if (!teleport(body, body_pose(body, pose)))
				{
					body.offset = old_offset;
					component.physicsobject.reset();
					world->Diagnose(entity, configuration, "Rigid body offset position overflow");
					continue;
				}
			}
			if (world->activate_all) b3Body_SetAwake(body.id, true);
			const auto type = body_type(component);
			if (body.teleported) body.teleported = false;
			else if (!IsSimulationEnabled() || type == b3_staticBody)
			{
				const auto pose = body_pose(body, authored);
				if (!finite(vec(pose.p)))
				{
					component.physicsobject.reset();
					world->Diagnose(entity, configuration, "Rigid body authored offset position overflow");
					continue;
				}
				b3Body_SetTransform(body.id, pose.p, pose.q);
				body.previous = body.current = pose;
			}
			else if (type == b3_kinematicBody)
			{
				const float target_dt = std::max(step, std::min(world->accumulator + std::max(0.0f, dt), step * accuracy.load()));
				const auto pose = body_pose(body, authored);
				if (!finite(vec(pose.p)))
				{
					component.physicsobject.reset();
					world->Diagnose(entity, configuration, "Kinematic target offset position overflow");
					continue;
				}
				b3Body_SetTargetTransform(body.id, pose, target_dt, true);
			}
			component.SetRefreshParametersNeeded(false);
			world->diagnostics[entity][0].clear();
		}
		world->activate_all = false;
		world->Drain();

		order.resize(scene.constraints.GetCount());
		for (size_t i = 0; i < order.size(); ++i) order[i] = i;
		std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return scene.constraints.GetEntity(a) < scene.constraints.GetEntity(b); });
		for (size_t index : order)
		{
			auto& component = scene.constraints[index];
			const Entity entity = scene.constraints.GetEntity(index);
			const uint64_t key = constraint_key(component);
			if (const char* reason = constraint_error(component))
			{
				component.physicsobject.reset();
				world->Diagnose(entity, key, reason, 1);
				continue;
			}
			const auto* transform = scene.transforms.GetComponent(entity);
			if (!transform)
			{
				component.physicsobject.reset();
				world->Diagnose(entity, key, "Constraint requires a TransformComponent", 1);
				continue;
			}
			auto resolve = [&](Entity target) -> b3BodyId
			{
				const auto* rb = scene.rigidbodies.GetComponent(target);
				const auto* s = rb ? state(*rb) : nullptr;
				return s && s->world == world && s->body && !s->body->released.load() ? s->body->id : b3BodyId{};
			};
			b3BodyId a = resolve(component.bodyA), b = resolve(component.bodyB);
			if ((component.bodyA != INVALID_ENTITY && !b3Body_IsValid(a)) ||
				(component.bodyB != INVALID_ENTITY && !b3Body_IsValid(b)))
			{
				component.physicsobject.reset();
				world->Diagnose(entity, key, "Constraint references a missing or unsupported rigid body", 1);
				continue;
			}
			if ((!b3Body_IsValid(a) && !b3Body_IsValid(b)) || B3_ID_EQUALS(a, b))
			{
				component.physicsobject.reset();
				world->Diagnose(entity, key, "Constraint requires two distinct bodies and at least one registered body", 1);
				continue;
			}
			if ((!b3Body_IsValid(a) || b3Body_GetType(a) == b3_staticBody) &&
				(!b3Body_IsValid(b) || b3Body_GetType(b) == b3_staticBody))
			{
				component.physicsobject.reset();
				world->Diagnose(entity, key, "Box3D cannot join two static bodies", 1);
				continue;
			}
			auto* s = state(component);
			if (s && (s->world != world || s->joint->type != component.type ||
				s->joint->entity_a != component.bodyA || s->joint->entity_b != component.bodyB ||
				(B3_IS_NON_NULL(a) && !B3_ID_EQUALS(a, s->joint->a)) ||
				(B3_IS_NON_NULL(b) && !B3_ID_EQUALS(b, s->joint->b))))
			{
				component.physicsobject.reset();
				s = nullptr;
			}
			if (!s)
			{
				XMVECTOR sv, qv, pv;
				if (!XMMatrixDecompose(&sv, &qv, &pv, transform->GetLocalMatrix() * scene.ComputeParentMatrixRecursive(entity)))
				{ world->Diagnose(entity, key, "Constraint transform is invalid", 1); continue; }
				XMFLOAT3 p; XMFLOAT4 q; XMStoreFloat3(&p, pv); XMStoreFloat4(&q, qv);
				if (!finite(p) || !valid_rotation(q)) { world->Diagnose(entity, key, "Constraint pose is invalid", 1); continue; }
				b3WorldTransform frame = { vec(p), joint_rotation(component.type, normalized(q)) };
				auto joint = std::make_shared<Joint>();
				joint->type = component.type;
				joint->entity_a = component.bodyA; joint->entity_b = component.bodyB;
				if (!b3Body_IsValid(a)) a = joint->anchor_a = anchor(*world, frame);
				if (!b3Body_IsValid(b)) b = joint->anchor_b = anchor(*world, frame);
				if (!b3Body_IsValid(a) || !b3Body_IsValid(b))
				{
					world->DestroyJoint(*joint);
					world->Diagnose(entity, key, "Constraint anchor allocation failed", 1);
					continue;
				}
				joint->a = a; joint->b = b;
				joint->frame_a = b3InvMulWorldTransforms(b3Body_GetTransform(a), frame);
				joint->frame_b = b3InvMulWorldTransforms(b3Body_GetTransform(b), frame);
				joint->bind_distance = body_distance(a, b);
				joint->break_distance = component.break_distance;
				joint->id = create_joint(*world, *joint, component);
				if (!b3Joint_IsValid(joint->id))
				{
					world->DestroyJoint(*joint);
					world->Diagnose(entity, key, "Constraint allocation failed", 1);
					continue;
				}
				component.physicsobject = wi::allocator::make_shared<JointState>();
				s = state(component); s->world = world; s->joint = joint;
				world->joints.push_back(joint);
			}
			else if (component.IsRefreshParametersNeeded())
			{
				auto& joint = *s->joint;
				if (!joint.broken && b3Body_IsValid(joint.a) && b3Body_IsValid(joint.b))
				{
					if (b3Joint_IsValid(joint.id)) b3DestroyJoint(joint.id, true);
					joint.id = create_joint(*world, joint, component);
					if (!b3Joint_IsValid(joint.id)) { world->Diagnose(entity, key, "Constraint refresh failed", 1); continue; }
				}
			}
			s->joint->break_distance = component.break_distance;
			component.SetRefreshParametersNeeded(false);
			world->diagnostics[entity][1].clear();
		}
		for (size_t i = 0; i < scene.softbodies.GetCount(); ++i)
			world->Diagnose(scene.softbodies.GetEntity(i), 0x534f4654, GetUnsupportedReason(Feature::SoftBodies), 2);
		for (size_t i = 0; i < scene.humanoids.GetCount(); ++i)
			if (scene.humanoids[i].IsRagdollPhysicsEnabled() && !scene.humanoids[i].IsRagdollDisabled())
				world->Diagnose(scene.humanoids.GetEntity(i), 0x52414744, GetUnsupportedReason(Feature::Ragdolls), 3);
		// The default buoyancy value is not a registration error. Diagnose water scenes once.
		bool water = scene.weather.IsOceanEnabled();
		for (size_t i = 0; i < scene.materials.GetCount() && !water; ++i)
			water = scene.materials[i].shaderType == MaterialComponent::SHADERTYPE_WATER;
		if (water)
			world->Diagnose(INVALID_ENTITY, 0x57415445, GetUnsupportedReason(Feature::Buoyancy), 4);
		else world->diagnostics[INVALID_ENTITY][4].clear();
		world->Drain();
		auto check_breaks = [&]()
		{
			for (auto& joint : world->joints)
			{
				if (!joint->broken && b3Body_IsValid(joint->a) && b3Body_IsValid(joint->b) &&
					body_distance(joint->a, joint->b) - joint->bind_distance > joint->break_distance)
				{
					if (b3Joint_IsValid(joint->id)) b3DestroyJoint(joint->id, true);
					joint->id = {}; joint->broken = true;
				}
			}
		};
		check_breaks();
		if (running)
		{
			world->accumulator = std::min(world->accumulator + dt, step * accuracy.load());
			while (world->accumulator >= step)
			{
				for (auto& body : world->bodies) body->previous = b3Body_GetTransform(body->id);
				// Solver quality is independent of SetAccuracy's catch-up limit.
				b3World_Step(world->id, step, 4);
				for (auto& body : world->bodies) body->current = b3Body_GetTransform(body->id);
				check_breaks();
				world->accumulator -= step;
			}
		}
		else if (!IsSimulationEnabled()) world->accumulator = 0;
		const float alpha = IsInterpolationEnabled() ? std::clamp(world->accumulator / step, 0.0f, 1.0f) : 1;
		{
			order.resize(scene.rigidbodies.GetCount());
			for (size_t i = 0; i < order.size(); ++i) order[i] = i;
			std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b)
			{
				return hierarchy_depth(scene, scene.rigidbodies.GetEntity(a)) < hierarchy_depth(scene, scene.rigidbodies.GetEntity(b));
			});
			for (size_t i : order)
			{
				auto& component = scene.rigidbodies[i];
				auto* s = state(component);
				if (!s || s->world != world || !s->body || !b3Body_IsValid(s->body->id) || body_type(component) != b3_dynamicBody) continue;
				auto* transform = scene.transforms.GetComponent(scene.rigidbodies.GetEntity(i));
				if (!transform) continue;
				auto pose = engine_pose(*s->body, s->body->current);
				const auto previous = engine_pose(*s->body, s->body->previous);
				pose.p = b3Lerp(previous.p, pose.p, alpha);
				XMFLOAT4 qa = quat(previous.q), qb = quat(pose.q), q;
				XMStoreFloat4(&q, XMQuaternionSlerp(XMLoadFloat4(&qa), XMLoadFloat4(&qb), alpha));
				XMFLOAT3 p = vec(pose.p);
				// Publish in local space without disturbing the authored local scale.
				const XMMATRIX parent = scene.ComputeParentMatrixRecursive(scene.rigidbodies.GetEntity(i));
				const XMMATRIX local = XMMatrixRotationQuaternion(XMLoadFloat4(&q)) * XMMatrixTranslation(p.x, p.y, p.z) * XMMatrixInverse(nullptr, parent);
				XMVECTOR sv, qv, pv;
				if (XMMatrixDecompose(&sv, &qv, &pv, local))
				{
					XMStoreFloat3(&transform->translation_local, pv);
					XMStoreFloat4(&transform->rotation_local, qv);
					transform->SetDirty();
				}
			}
		}
		if (IsDebugDrawEnabled())
			draw_debug(*world, scene);
	}

	void SetPosition(RigidBodyPhysicsComponent& component, const XMFLOAT3& position)
	{
		if (!finite(position)) { wilog_error("Box3D: SetPosition requires finite coordinates"); return; }
		with_body(component, "SetPosition", [&](Body& body)
		{
			auto pose = engine_pose(body, b3Body_GetTransform(body.id));
			pose.p = vec(position); teleport(body, body_pose(body, pose));
		});
	}
	void SetPositionAndRotation(RigidBodyPhysicsComponent& component, const XMFLOAT3& position, const XMFLOAT4& rotation)
	{
		if (!finite(position) || !valid_rotation(rotation)) { wilog_error("Box3D: teleport pose is invalid"); return; }
		with_body(component, "SetPositionAndRotation", [&](Body& body)
		{ teleport(body, body_pose(body, b3WorldTransform{ vec(position), normalized(rotation) })); });
	}
	void SetLinearVelocity(RigidBodyPhysicsComponent& component, const XMFLOAT3& velocity)
	{
		if (!finite(velocity)) { wilog_error("Box3D: linear velocity is invalid"); return; }
		with_body(component, "SetLinearVelocity", [&](Body& body) { b3Body_SetLinearVelocity(body.id, vec(velocity)); b3Body_SetAwake(body.id, true); });
	}
	void SetAngularVelocity(RigidBodyPhysicsComponent& component, const XMFLOAT3& velocity)
	{
		if (!finite(velocity)) { wilog_error("Box3D: angular velocity is invalid"); return; }
		with_body(component, "SetAngularVelocity", [&](Body& body) { b3Body_SetAngularVelocity(body.id, vec(velocity)); b3Body_SetAwake(body.id, true); });
	}
	XMFLOAT3 GetVelocity(RigidBodyPhysicsComponent& component)
	{
		XMFLOAT3 result{};
		with_body(component, "GetVelocity", [&](Body& body) { result = vec(b3Body_GetLinearVelocity(body.id)); });
		return result;
	}
	XMFLOAT3 GetPosition(RigidBodyPhysicsComponent& component)
	{
		XMFLOAT3 result{};
		with_body(component, "GetPosition", [&](Body& body) { result = vec(b3Body_GetPosition(body.id)); });
		return result;
	}
	XMFLOAT4 GetRotation(RigidBodyPhysicsComponent& component)
	{
		XMFLOAT4 result(0, 0, 0, 1);
		with_body(component, "GetRotation", [&](Body& body) { result = quat(b3Body_GetRotation(body.id)); });
		return result;
	}
	void ApplyForce(RigidBodyPhysicsComponent& component, const XMFLOAT3& force)
	{
		if (!finite(force)) { wilog_error("Box3D: force is invalid"); return; }
		with_body(component, "ApplyForce", [&](Body& body) { b3Body_ApplyForceToCenter(body.id, vec(force), true); });
	}
	void ApplyImpulse(RigidBodyPhysicsComponent& component, const XMFLOAT3& impulse)
	{
		if (!finite(impulse)) { wilog_error("Box3D: impulse is invalid"); return; }
		with_body(component, "ApplyImpulse", [&](Body& body) { b3Body_ApplyLinearImpulseToCenter(body.id, vec(impulse), true); });
	}
	void ApplyForceAt(RigidBodyPhysicsComponent& component, const XMFLOAT3& force, const XMFLOAT3& at, bool at_local)
	{
		if (!finite(force) || !finite(at)) { wilog_error("Box3D: force/point is invalid"); return; }
		with_body(component, "ApplyForceAt", [&](Body& body)
		{
			const b3Vec3 point = at_local ? b3Add(b3Body_GetWorldCenter(body.id), b3RotateVector(b3Body_GetRotation(body.id), vec(at))) : vec(at);
			if (!finite(vec(point))) { wilog_error("Box3D: force point overflow"); return; }
			b3Body_ApplyForce(body.id, vec(force), point, true);
		});
	}
	void ApplyImpulseAt(RigidBodyPhysicsComponent& component, const XMFLOAT3& impulse, const XMFLOAT3& at, bool at_local)
	{
		if (!finite(impulse) || !finite(at)) { wilog_error("Box3D: impulse/point is invalid"); return; }
		with_body(component, "ApplyImpulseAt", [&](Body& body)
		{
			const b3Vec3 point = at_local ? b3Add(b3Body_GetWorldCenter(body.id), b3RotateVector(b3Body_GetRotation(body.id), vec(at))) : vec(at);
			if (!finite(vec(point))) { wilog_error("Box3D: impulse point overflow"); return; }
			b3Body_ApplyLinearImpulse(body.id, vec(impulse), point, true);
		});
	}
	void ApplyTorque(RigidBodyPhysicsComponent& component, const XMFLOAT3& torque)
	{
		if (!finite(torque)) { wilog_error("Box3D: torque is invalid"); return; }
		with_body(component, "ApplyTorque", [&](Body& body) { b3Body_ApplyTorque(body.id, vec(torque), true); });
	}
	void SetActivationState(RigidBodyPhysicsComponent& component, ActivationState activation)
	{
		with_body(component, "SetActivationState", [&](Body& body) { b3Body_SetAwake(body.id, activation == ActivationState::Active); });
	}
	void SetGhostMode(RigidBodyPhysicsComponent& component, bool value)
	{
		with_body(component, "SetGhostMode", [&](Body& body)
		{
			body.ghost = value;
			auto filter = b3Shape_GetFilter(body.shape);
			filter.maskBits = value ? 0 : B3_DEFAULT_MASK_BITS;
			b3Shape_SetFilter(body.shape, filter, true);
		});
	}
	void ActivateAllRigidBodies(Scene& scene)
	{
		auto world = get_world(scene);
		if (!world) return;
		std::lock_guard<std::mutex> lock(world->gate);
		world->Drain();
		for (auto& body : world->bodies) b3Body_SetAwake(body->id, true);
	}
	void OptimizeBroadPhase(Scene&)
	{
		wilog("Box3D: OptimizeBroadPhase is unsupported; Box3D maintains its broadphase automatically");
	}
	void ResetPhysicsObjects(Scene& scene)
	{
		auto world = get_world(scene);
		if (!world) return;
		std::lock_guard<std::mutex> lock(world->gate);
		world->Drain();
		world->accumulator = 0;
		auto bodies = world->bodies;
		std::stable_sort(bodies.begin(), bodies.end(), [&](const auto& a, const auto& b)
		{
			return hierarchy_depth(scene, a->entity) < hierarchy_depth(scene, b->entity);
		});
		for (auto& body : bodies)
		{
			if (!teleport(*body, body_pose(*body, body->initial))) continue;
			b3Body_SetLinearVelocity(body->id, b3Vec3_zero);
			b3Body_SetAngularVelocity(body->id, b3Vec3_zero);
			if (auto* component = scene.rigidbodies.GetComponent(body->entity))
				b3Body_SetAwake(body->id, !component->IsStartDeactivated());
			if (b3Body_GetType(body->id) == b3_dynamicBody)
			{
				if (auto* transform = scene.transforms.GetComponent(body->entity))
				{
					const XMFLOAT3 p = vec(body->initial.p);
					const XMFLOAT4 q = quat(body->initial.q);
					const XMMATRIX local = XMMatrixRotationQuaternion(XMLoadFloat4(&q)) *
						XMMatrixTranslation(p.x, p.y, p.z) * XMMatrixInverse(nullptr, scene.ComputeParentMatrixRecursive(body->entity));
					XMVECTOR sv, qv, pv;
					if (XMMatrixDecompose(&sv, &qv, &pv, local))
					{
						XMStoreFloat3(&transform->translation_local, pv);
						XMStoreFloat4(&transform->rotation_local, qv);
						transform->SetDirty();
					}
				}
			}
		}
	}
	bool IsConstraintBroken(const PhysicsConstraintComponent& component)
	{
		if (component.type == PhysicsConstraintComponent::Type::SixDOF)
		{ wilog_error("Box3D: IsConstraintBroken: %s", GetUnsupportedReason(component.type)); return false; }
		const auto* s = state(component);
		if (!s || !s->world || !s->joint) return false;
		std::lock_guard<std::mutex> lock(s->world->gate);
		s->world->Drain();
		return s->joint->broken || !b3Joint_IsValid(s->joint->id);
	}
	void SetConstraintBroken(PhysicsConstraintComponent& component, bool broken)
	{
		auto* s = state(component);
		if (!s || !s->world || !s->joint)
		{ wilog_error("Box3D: SetConstraintBroken requires a registered constraint"); return; }
		std::lock_guard<std::mutex> lock(s->world->gate);
		s->world->Drain();
		auto& joint = *s->joint;
		if (broken)
		{
			if (b3Joint_IsValid(joint.id)) b3DestroyJoint(joint.id, true);
			joint.id = {}; joint.broken = true;
		}
		else if (joint.broken)
		{
			if (const char* reason = constraint_error(component)) { wilog_error("Box3D: %s", reason); return; }
			if (!b3Body_IsValid(joint.a) || !b3Body_IsValid(joint.b))
			{ wilog_error("Box3D: Cannot repair a constraint whose body was deleted"); return; }
			if (b3Body_GetType(joint.a) == b3_staticBody && b3Body_GetType(joint.b) == b3_staticBody)
			{ wilog_error("Box3D: Cannot repair a constraint between two static bodies"); return; }
			joint.id = create_joint(*s->world, joint, component);
			joint.broken = !b3Joint_IsValid(joint.id);
		}
	}
	RayIntersectionResult Intersects(const Scene& scene, wi::primitive::Ray ray, Entity ignore_entity)
	{
		auto world = get_world(scene);
		if (!world) return {};
		std::lock_guard<std::mutex> lock(world->gate);
		world->Drain();
		return cast_ray(*world, ray, ignore_entity);
	}
	void PickDrag(const Scene& scene, wi::primitive::Ray ray, PickDragOperation& op, ConstraintType type, float break_distance)
	{
		auto world = get_world(scene);
		if (!world) { op.internal_state.reset(); return; }
		if (!finite(ray.origin) || !finite(ray.direction) || b3LengthSquared(vec(ray.direction)) <= 1e-12f ||
			!std::isfinite(break_distance) || break_distance < 0 ||
			(type != ConstraintType::Fixed && type != ConstraintType::Point))
		{ wilog_error("Box3D: PickDrag ray/break distance is invalid"); return; }
		if (op.internal_state && static_cast<DragState*>(op.internal_state.get())->world != world)
		{
			op.internal_state.reset();
			return;
		}
		std::lock_guard<std::mutex> lock(world->gate);
		world->Drain();
		if (op.internal_state)
		{
			auto* drag = static_cast<DragState*>(op.internal_state.get());
			if (drag->world != world || drag->target->released.load() || drag->joint->broken ||
				!b3Body_IsValid(drag->joint->anchor_a) || !b3Joint_IsValid(drag->joint->id))
			{
				op.internal_state.reset(); world->Drain(); return;
			}
			const b3Vec3 p = b3MulAdd(vec(ray.origin), drag->distance, vec(ray.direction));
			if (!finite(vec(p))) { wilog_error("Box3D: PickDrag target position overflow"); return; }
			b3Body_SetTransform(drag->joint->anchor_a, p, b3Body_GetRotation(drag->joint->anchor_a));
			b3Joint_WakeBodies(drag->joint->id);
			if (body_distance(drag->joint->a, drag->joint->b) - drag->joint->bind_distance > drag->joint->break_distance)
			{ op.internal_state.reset(); world->Drain(); }
			return;
		}
		auto hit = cast_ray(*world, ray);
		if (!hit.IsValid()) return;
		auto found = std::find_if(world->bodies.begin(), world->bodies.end(), [&](const auto& body) { return body.get() == hit.physicsobject; });
		if (found == world->bodies.end() || b3Body_GetType((*found)->id) != b3_dynamicBody) return;
		auto target = *found;
		auto joint = std::make_shared<Joint>();
		b3WorldTransform frame{ vec(hit.position), b3Body_GetRotation(target->id) };
		joint->anchor_a = anchor(*world, frame, true);
		if (!b3Body_IsValid(joint->anchor_a))
		{ wilog_error("Box3D: PickDrag anchor allocation failed"); return; }
		joint->a = joint->anchor_a; joint->b = target->id;
		joint->frame_a = b3Transform_identity;
		joint->frame_b = b3InvMulWorldTransforms(b3Body_GetTransform(target->id), frame);
		joint->bind_distance = body_distance(joint->a, joint->b);
		joint->break_distance = break_distance;
		PhysicsConstraintComponent component;
		component.type = type == ConstraintType::Fixed ? PhysicsConstraintComponent::Type::Fixed : PhysicsConstraintComponent::Type::Point;
		component.SetDisableSelfCollision(true);
		joint->type = component.type;
		joint->id = create_joint(*world, *joint, component);
		if (!b3Joint_IsValid(joint->id))
		{ world->DestroyJoint(*joint); wilog_error("Box3D: PickDrag joint allocation failed"); return; }
		op.internal_state = wi::allocator::make_shared<DragState>();
		auto* drag = static_cast<DragState*>(op.internal_state.get());
		drag->world = world; drag->joint = joint; drag->target = target;
		drag->distance = b3Length(b3Sub(vec(hit.position), vec(ray.origin))) / b3Length(vec(ray.direction));
		world->joints.push_back(joint);
		b3Joint_WakeBodies(joint->id);
	}

	XMFLOAT3 GetCharacterGroundPosition(RigidBodyPhysicsComponent&) { unsupported("GetCharacterGroundPosition", Feature::Characters); return {}; }
	XMFLOAT3 GetCharacterGroundNormal(RigidBodyPhysicsComponent&) { unsupported("GetCharacterGroundNormal", Feature::Characters); return {}; }
	XMFLOAT3 GetCharacterGroundVelocity(RigidBodyPhysicsComponent&) { unsupported("GetCharacterGroundVelocity", Feature::Characters); return {}; }
	bool IsCharacterGroundSupported(RigidBodyPhysicsComponent&) { unsupported("IsCharacterGroundSupported", Feature::Characters); return false; }
	CharacterGroundStates GetCharacterGroundState(RigidBodyPhysicsComponent&) { unsupported("GetCharacterGroundState", Feature::Characters); return CharacterGroundStates::InAir; }
	bool ChangeCharacterShape(RigidBodyPhysicsComponent&, const RigidBodyPhysicsComponent::CapsuleParams&) { unsupported("ChangeCharacterShape", Feature::Characters); return false; }
	void MoveCharacter(RigidBodyPhysicsComponent&, const XMFLOAT3&, float, float, bool) { unsupported("MoveCharacter", Feature::Characters); }
	void DriveVehicle(RigidBodyPhysicsComponent&, float, float, float, float) { unsupported("DriveVehicle", Feature::Vehicles); }
	float GetVehicleForwardVelocity(RigidBodyPhysicsComponent&) { unsupported("GetVehicleForwardVelocity", Feature::Vehicles); return 0; }
	void OverrideWehicleWheelTransforms(Scene& scene)
	{
		auto world = get_world(scene);
		if (!world) return; // Registration diagnoses unavailable scene features.
		std::lock_guard<std::mutex> lock(world->gate);
		for (size_t i = 0; i < scene.rigidbodies.GetCount(); ++i)
			if (scene.rigidbodies[i].IsVehicle())
			{
				const auto entity = scene.rigidbodies.GetEntity(i);
				if (world->diagnostics[entity][0].empty())
					world->Diagnose(entity, 0x56454849, GetUnsupportedReason(Feature::Vehicles));
			}
	}
	void ApplyImpulse(HumanoidComponent&, HumanoidComponent::HumanoidBone, const XMFLOAT3&) { unsupported("ApplyImpulse", Feature::Ragdolls); }
	void ApplyImpulseAt(HumanoidComponent&, HumanoidComponent::HumanoidBone, const XMFLOAT3&, const XMFLOAT3&, bool) { unsupported("ApplyImpulseAt", Feature::Ragdolls); }
	void SetGhostMode(HumanoidComponent&, bool) { unsupported("SetGhostMode", Feature::Ragdolls); }
	void SetRagdollGhostMode(HumanoidComponent&, bool) { unsupported("SetRagdollGhostMode", Feature::Ragdolls); }
	void SetActivationState(SoftBodyPhysicsComponent&, ActivationState) { unsupported("SetActivationState", Feature::SoftBodies); }
	XMFLOAT3 GetSoftBodyNodePosition(SoftBodyPhysicsComponent&, uint32_t) { unsupported("GetSoftBodyNodePosition", Feature::SoftBodies); return {}; }
}
