#include "WickedEngine.h"

#if defined(PLATFORM_LINUX)
#include "wiGraphicsDevice_Vulkan.h"
#endif

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>

namespace
{
	using namespace wi::scene;
	using wi::ecs::Entity;
	constexpr float step = 1.0f / 60.0f;

	bool Check(bool condition, const char* message)
	{
		if (!condition)
			std::cerr << wi::physics::GetBackendName() << " physics contract: " << message << '\n';
		return condition;
	}

	bool Near(float a, float b, float tolerance = 0.02f)
	{
		return std::isfinite(a) && std::abs(a - b) <= tolerance;
	}

	struct Fixture
	{
		Scene scene;
		wi::jobsystem::context context;

		Fixture()
		{
			scene.weather.gravity = XMFLOAT3(0, -9.81f, 0);
		}

		~Fixture()
		{
			wi::jobsystem::Wait(context);
			scene.Clear();
		}

		Entity Body(XMFLOAT3 position, float mass = 1,
			RigidBodyPhysicsComponent::CollisionShape shape = RigidBodyPhysicsComponent::BOX)
		{
			const Entity entity = wi::ecs::CreateEntity();
			auto& transform = scene.transforms.Create(entity);
			transform.Translate(position);
			transform.UpdateTransform();
			auto& body = scene.rigidbodies.Create(entity);
			body.shape = shape;
			body.mass = mass;
			body.box.halfextents = XMFLOAT3(0.5f, 0.5f, 0.5f);
			body.sphere.radius = 0.5f;
			body.buoyancy = 0;
			return entity;
		}

		void Tick(int count = 1)
		{
			for (int i = 0; i < count; ++i)
			{
				scene.dt = step;
				scene.RunTransformUpdateSystem(context);
				wi::jobsystem::Wait(context);
				wi::physics::RunPhysicsUpdateSystem(context, scene, step);
				wi::jobsystem::Wait(context);
			}
		}

		void AttachGeometry(Entity body, const wi::vector<XMFLOAT3>& positions,
			const wi::vector<uint32_t>& indices)
		{
			const Entity mesh = wi::ecs::CreateEntity();
			auto& geometry = scene.meshes.Create(mesh);
			geometry.vertex_positions = positions;
			geometry.indices = indices;
			MeshComponent::MeshSubset subset;
			subset.indexOffset = 0;
			subset.indexCount = (uint32_t)indices.size();
			geometry.subsets.push_back(subset);
			scene.objects.Create(body).meshID = mesh;
		}

		RigidBodyPhysicsComponent& Get(Entity entity)
		{
			return *scene.rigidbodies.GetComponent(entity);
		}
	};

	bool Simulation()
	{
		Fixture f;
		const Entity floor = f.Body(XMFLOAT3(0, -0.5f, 0), 0);
		f.Get(floor).box.halfextents = XMFLOAT3(20, 0.5f, 20);
		const Entity ball = f.Body(XMFLOAT3(0, 4, 0), 2, RigidBodyPhysicsComponent::SPHERE);
		f.Tick();
		if (!Check(f.Get(ball).physicsobject != nullptr, "dynamic sphere was not registered"))
			return false;
		f.Tick(30);
		if (!Check(wi::physics::GetPosition(f.Get(ball)).y < 3.5f, "gravity did not move the sphere"))
			return false;
		f.Tick(240);
		if (!Check(Near(wi::physics::GetPosition(f.Get(ball)).y, 0.5f, 0.08f), "sphere did not settle on the floor"))
			return false;
		wi::physics::ApplyImpulse(f.Get(ball), XMFLOAT3(0, 4, 0));
		if (!Check(wi::physics::GetVelocity(f.Get(ball)).y > 1.8f, "mass-scaled impulse did not wake the sphere"))
			return false;
		f.Tick(10);
		if (!Check(wi::physics::GetPosition(f.Get(ball)).y > 0.65f, "impulse had no simulation effect"))
			return false;
		wi::physics::SetPositionAndRotation(f.Get(ball), XMFLOAT3(2, 3, 0), XMFLOAT4(0, 0, 0, 1));
		wi::physics::SetLinearVelocity(f.Get(ball), XMFLOAT3(1, 2, 3));
		wi::physics::SetAngularVelocity(f.Get(ball), XMFLOAT3(0, 1, 0));
		if (!Check(Near(wi::physics::GetPosition(f.Get(ball)).x, 2), "teleport getter did not observe the write") ||
			!Check(Near(wi::physics::GetVelocity(f.Get(ball)).z, 3), "velocity getter did not observe the write") ||
			!Check(Near(wi::physics::GetRotation(f.Get(ball)).w, 1), "rotation getter changed identity"))
			return false;
		wi::physics::ResetPhysicsObjects(f.scene);
		return Check(Near(wi::physics::GetPosition(f.Get(ball)).y, 4), "reset did not restore the initial pose");
	}

	bool QueryAndLifetime()
	{
		Fixture f;
		f.scene.weather.gravity = XMFLOAT3(0, 0, 0);
		const Entity box = f.Body(XMFLOAT3(0, 0, 0), 0);
		f.Tick();
		const wi::primitive::Ray ray(XMFLOAT3(0, 3, 0), XMFLOAT3(0, -1, 0), 0, 10);
		auto hit = wi::physics::Intersects(f.scene, ray);
		if (!Check(hit.entity == box, "ray did not resolve the authored entity") ||
			!Check(Near(hit.position.y, 0.5f), "ray returned an incorrect hit position") ||
			!Check(hit.normal.y > 0.9f, "ray returned an incorrect surface normal"))
			return false;
		if (!Check(!wi::physics::Intersects(f.scene,
			wi::primitive::Ray(ray.origin, ray.direction, 0, 1)).IsValid(), "ray TMax was ignored") ||
			!Check(!wi::physics::Intersects(f.scene,
			wi::primitive::Ray(ray.origin, ray.direction, 4, 10)).IsValid(), "ray TMin/backface policy was ignored"))
			return false;
		wi::physics::SetSimulationEnabled(false);
		f.scene.rigidbodies.Remove(box);
		wi::physics::FlushDeferredDeletes(f.scene);
		const bool removed = !wi::physics::Intersects(f.scene, ray).IsValid();
		wi::physics::SetSimulationEnabled(true);
		if (!Check(removed, "paused body deletion left a queryable body"))
			return false;
		f.scene.Clear();
		wi::physics::SetEnabled(false);
		wi::physics::RunPhysicsUpdateSystem(f.context, f.scene, 0);
		wi::physics::SetEnabled(true);
		return true;
	}

	bool ForcesAndGhosts()
	{
		Fixture f;
		f.scene.weather.gravity = XMFLOAT3(0, 0, 0);
		const Entity a = f.Body(XMFLOAT3(0, 0, 0), 2);
		f.Get(a).damping_linear = 0;
		f.Tick();
		wi::physics::ApplyForce(f.Get(a), XMFLOAT3(120, 0, 0));
		f.Tick();
		if (!Check(Near(wi::physics::GetVelocity(f.Get(a)).x, 1, 0.15f), "force/mass integration is incorrect"))
			return false;
		wi::physics::ApplyImpulseAt(f.Get(a), XMFLOAT3(0, 0, 1), XMFLOAT3(0, 1, 0), true);
		wi::physics::ApplyTorque(f.Get(a), XMFLOAT3(0, 1, 0));
		f.Tick();
		const auto rotation = wi::physics::GetRotation(f.Get(a));
		if (!Check(std::abs(rotation.x) > 0.001f, "off-center impulse did not rotate the body"))
			return false;

		const Entity b = f.Body(XMFLOAT3(5, 0, 0), 0);
		f.Tick();
		wi::physics::SetGhostMode(f.Get(b), true);
		wi::physics::SetLinearVelocity(f.Get(a), XMFLOAT3(6, 0, 0));
		f.Tick(80);
		if (!Check(wi::physics::GetPosition(f.Get(a)).x > 7, "ghost body blocked another body"))
			return false;
		wi::physics::SetGhostMode(f.Get(b), false);
		return true;
	}

	bool Shapes()
	{
		Fixture f;
		f.scene.weather.gravity = XMFLOAT3(0, 0, 0);
		const RigidBodyPhysicsComponent::CollisionShape shapes[] = {
			RigidBodyPhysicsComponent::BOX, RigidBodyPhysicsComponent::SPHERE,
			RigidBodyPhysicsComponent::CAPSULE, RigidBodyPhysicsComponent::CYLINDER,
		};
		for (int i = 0; i < 4; ++i)
		{
			const Entity e = f.Body(XMFLOAT3(float(i * 4), 0, 0), 0, shapes[i]);
			f.Tick();
			if (!Check(f.Get(e).physicsobject != nullptr, "primitive shape registration failed") ||
				!Check(wi::physics::Intersects(f.scene, wi::primitive::Ray(
					XMFLOAT3(float(i * 4), 4, 0), XMFLOAT3(0, -1, 0), 0, 10)).entity == e,
					"primitive shape was not queryable"))
				return false;
		}
		return true;
	}

	bool Constraints()
	{
		using Type = PhysicsConstraintComponent::Type;
		const Type types[] = { Type::Fixed, Type::Point, Type::Distance, Type::Hinge, Type::Slider, Type::Cone };
		for (Type type : types)
		{
			Fixture f;
			const Entity anchor = f.Body(XMFLOAT3(0, 2, 0), 0);
			const Entity body = f.Body(XMFLOAT3(0, 1, 0));
			const Entity joint = wi::ecs::CreateEntity();
			auto& transform = f.scene.transforms.Create(joint);
			transform.Translate(XMFLOAT3(0, 2, 0));
			transform.UpdateTransform();
			auto& constraint = f.scene.constraints.Create(joint);
			constraint.type = type;
			constraint.bodyA = anchor;
			constraint.bodyB = body;
			constraint.distance_constraint.min_distance = 0.5f;
			constraint.distance_constraint.max_distance = 1.5f;
			constraint.slider_constraint.min_limit = -1;
			constraint.slider_constraint.max_limit = 1;
			constraint.cone_constraint.half_cone_angle = 0.4f;
			f.Tick();
			if (!Check(constraint.physicsobject != nullptr, "common constraint was not registered"))
				return false;
			f.Tick(120);
			if (!Check(wi::physics::GetPosition(f.Get(body)).y > -1, "joint did not restrain the falling body"))
				return false;
			wi::physics::SetConstraintBroken(constraint, true);
			if (!Check(wi::physics::IsConstraintBroken(constraint), "constraint broken state was not recorded"))
				return false;
			wi::physics::SetConstraintBroken(constraint, false);
			if (!Check(!wi::physics::IsConstraintBroken(constraint), "constraint could not be repaired"))
				return false;
		}
		return true;
	}

	bool CookedGeometry()
	{
		using Shape = RigidBodyPhysicsComponent;
		const Shape::CollisionShape shapes[] = { Shape::CONVEX_HULL, Shape::TRIANGLE_MESH, Shape::HEIGHTFIELD };
		for (auto shape : shapes)
		{
			if (wi::physics::GetUnsupportedReason(shape) != nullptr)
				continue;
			Fixture f;
			const Entity floor = f.Body(XMFLOAT3(0, 0, 0), 0, shape);
			if (shape == Shape::CONVEX_HULL)
			{
				f.AttachGeometry(floor, {
					XMFLOAT3(-2, -1, -2), XMFLOAT3(2, -1, -2),
					XMFLOAT3(-2, -1, 2), XMFLOAT3(2, -1, 2),
					XMFLOAT3(-2, 0, -2), XMFLOAT3(2, 0, -2),
					XMFLOAT3(-2, 0, 2), XMFLOAT3(2, 0, 2),
				}, {});
			}
			else if (shape == Shape::HEIGHTFIELD)
			{
				wi::vector<XMFLOAT3> positions;
				for (int z = 0; z < 8; ++z)
					for (int x = 0; x < 8; ++x)
						positions.push_back(XMFLOAT3(-2 + 4.0f * x / 7, 0, -2 + 4.0f * z / 7));
				f.AttachGeometry(floor, positions, {});
			}
			else
			{
				f.AttachGeometry(floor, {
					XMFLOAT3(-2, 0, -2), XMFLOAT3(2, 0, -2),
					XMFLOAT3(-2, 0, 2), XMFLOAT3(2, 0, 2),
				}, { 0, 1, 2, 1, 3, 2 });
			}
			const Entity ball = f.Body(XMFLOAT3(0, 2, 0), 1, Shape::SPHERE);
			f.Tick();
			if (!Check(f.Get(floor).physicsobject != nullptr, "cooked shape was not registered"))
				return false;
			const auto hit = wi::physics::Intersects(f.scene,
				wi::primitive::Ray(XMFLOAT3(1, 4, 0), XMFLOAT3(0, -1, 0), 0, 10));
			if (!Check(hit.entity == floor && hit.normal.y > 0.9f, "cooked shape winding or heightfield orientation is incorrect"))
				return false;
			f.Tick(240);
			if (!Check(Near(wi::physics::GetPosition(f.Get(ball)).y, 0.5f, 0.1f), "cooked geometry did not support a dynamic body"))
				return false;
		}
		return true;
	}

	bool OffsetsAndParenting()
	{
		Fixture f;
		f.scene.weather.gravity = XMFLOAT3(0, 0, 0);
		const Entity body = f.Body(XMFLOAT3(1, 2, 3));
		f.Get(body).local_offset = XMFLOAT3(0.25f, 0, 0);
		f.Tick();
		if (!Check(Near(wi::physics::GetPosition(f.Get(body)).x, 1.25f), "body getter lost the local offset"))
			return false;
		wi::physics::SetPosition(f.Get(body), XMFLOAT3(4, 2, 3));
		if (!Check(Near(wi::physics::GetPosition(f.Get(body)).x, 4.25f), "teleport local offset differs from registration"))
			return false;
		f.Tick();
		if (!Check(Near(f.scene.transforms.GetComponent(body)->translation_local.x, 4), "rendered pose did not remove local offset"))
			return false;
		const Entity parent = wi::ecs::CreateEntity();
		auto& parentTransform = f.scene.transforms.Create(parent);
		parentTransform.Translate(XMFLOAT3(10, 0, 0));
		parentTransform.UpdateTransform();
		f.scene.hierarchy.Create(body).parentID = parent;
		f.Get(body).physicsobject.reset();
		f.scene.StartBuildTopDownHierarchy();
		f.scene.WaitBuildTopDownHierarchy();
		f.Tick();
		if (!Check(Near(wi::physics::GetPosition(f.Get(body)).x, 14.25f), "parented body did not register in world space"))
			return false;
		wi::physics::SetLinearVelocity(f.Get(body), XMFLOAT3(1, 0, 0));
		f.Tick(60);
		const float x = f.scene.transforms.GetComponent(body)->translation_local.x;
		return Check(x > 4.5f && x < 5.1f, "simulation did not convert the world pose back to parent space");
	}

	bool PlanarConvexFloor()
	{
		using Shape = RigidBodyPhysicsComponent;
		Fixture f;
		const Entity floor = f.Body(XMFLOAT3(0, 0, 0), 0, Shape::CONVEX_HULL);
		f.AttachGeometry(floor, {
			XMFLOAT3(-20, 0, -20), XMFLOAT3(20, 0, -20),
			XMFLOAT3(20, 0, 20), XMFLOAT3(-20, 0, 20),
			XMFLOAT3(0, 0, 0), XMFLOAT3(-20, 0, -20),
		}, { 0, 1, 2, 0, 2, 3 });
		const Shape::CollisionShape shapes[] = { Shape::BOX, Shape::SPHERE, Shape::CAPSULE, Shape::CYLINDER };
		wi::vector<Entity> bodies;
		for (size_t i = 0; i < arraysize(shapes); ++i)
			bodies.push_back(f.Body(XMFLOAT3(-6 + float(i) * 4, 3, 0), 1, shapes[i]));
		const Entity triangle = f.Body(XMFLOAT3(12, 3, 0), 1, Shape::CONVEX_HULL);
		f.AttachGeometry(triangle, {
			XMFLOAT3(-0.5f, 0, -0.5f), XMFLOAT3(0.5f, 0, -0.5f), XMFLOAT3(0, 0, 0.5f),
		}, { 0, 1, 2 });
		f.Tick();
		if (!Check(f.Get(floor).physicsobject != nullptr, "planar convex floor was not registered"))
			return false;
		if (!Check(f.Get(triangle).physicsobject != nullptr, "three-point dynamic planar hull was not registered"))
			return false;
		const auto above = wi::physics::Intersects(f.scene,
			wi::primitive::Ray(XMFLOAT3(0, 4, 5), XMFLOAT3(0, -1, 0), 0, 10));
		const auto below = wi::physics::Intersects(f.scene,
			wi::primitive::Ray(XMFLOAT3(0, -4, 5), XMFLOAT3(0, 1, 0), 0, 10));
		if (!Check(above.entity == floor && below.entity == floor, "planar convex floor must be queryable from both sides"))
			return false;
		f.Tick(300);
		for (Entity body : bodies)
			if (!Check(wi::physics::GetPosition(f.Get(body)).y > 0.35f, "rigid body fell through the planar convex floor"))
				return false;
		if (!Check(wi::physics::GetPosition(f.Get(triangle)).y >= -0.01f, "dynamic planar hull fell through the planar convex floor"))
			return false;
		f.Get(floor).mass = 1;
		f.Get(floor).SetKinematic(true);
		f.Get(floor).SetRefreshParametersNeeded();
		for (int i = 0; i < 80; ++i)
		{
			f.scene.transforms.GetComponent(floor)->Translate(XMFLOAT3(0, 0.01f, 0));
			f.Tick();
		}
		for (Entity body : bodies)
			if (!Check(wi::physics::GetPosition(f.Get(body)).y > 1.15f, "moving planar kinematic floor did not support its rigid bodies"))
				return false;
		// This checks Box3D's new volumetric hull; Jolt retains a zero-volume planar hull.
		if (std::strcmp(wi::physics::GetBackendName(), "Box3D") == 0 &&
			!Check(wi::physics::GetPosition(f.Get(triangle)).y >= 0.79f, "moving planar floor did not support a dynamic planar hull"))
			return false;
		return true;
	}

	bool RefreshAndInterpolation()
	{
		Fixture f;
		f.scene.weather.gravity = XMFLOAT3(0, 0, 0);
		const Entity body = f.Body(XMFLOAT3(0, 0, 0));
		f.Tick();
		f.Get(body).mass = 2;
		f.Get(body).damping_linear = 0;
		f.Get(body).SetRefreshParametersNeeded();
		f.Tick();
		wi::physics::ApplyImpulse(f.Get(body), XMFLOAT3(2, 0, 0));
		if (!Check(Near(wi::physics::GetVelocity(f.Get(body)).x, 1), "mass refresh did not update inertia/mass"))
			return false;
		wi::physics::SetInterpolationEnabled(true);
		f.Tick(3);
		auto* transform = f.scene.transforms.GetComponent(body);
		const float physical = wi::physics::GetPosition(f.Get(body)).x;
		const float rendered = transform->translation_local.x;
		wi::physics::SetInterpolationEnabled(false);
		if (!Check(rendered <= physical + 0.01f && physical - rendered < 0.04f, "interpolated pose escaped its simulation endpoints"))
			return false;
		f.Get(body).SetLocked2D(true);
		f.Get(body).SetRefreshParametersNeeded();
		f.Tick();
		wi::physics::ApplyImpulse(f.Get(body), XMFLOAT3(0, 0, 2));
		f.Tick(10);
		return Check(Near(wi::physics::GetPosition(f.Get(body)).z, 0), "2D lock allowed out-of-plane movement");
	}

	bool Kinematics()
	{
		Fixture f;
		f.scene.weather.gravity = XMFLOAT3(0, 0, 0);
		const Entity body = f.Body(XMFLOAT3(0, 0, 0));
		f.Get(body).SetKinematic(true);
		f.Tick();
		for (int i = 0; i < 60; ++i)
		{
			f.scene.transforms.GetComponent(body)->Translate(XMFLOAT3(step, 0, 0));
			f.Tick();
		}
		if (!Check(Near(wi::physics::GetPosition(f.Get(body)).x, 1, 0.05f), "kinematic target did not follow the authored transform"))
			return false;
		f.Get(body).SetKinematic(false);
		f.Get(body).SetRefreshParametersNeeded();
		f.Tick();
		wi::physics::SetLinearVelocity(f.Get(body), XMFLOAT3(0, 0, 0));
		wi::physics::ApplyImpulse(f.Get(body), XMFLOAT3(1, 0, 0));
		return Check(wi::physics::GetVelocity(f.Get(body)).x > 0.9f, "kinematic-to-dynamic transition rejected an impulse");
	}

	bool RotationLocksAndFilteredQueries()
	{
		for (int axis = 0; axis < 3; ++axis)
		{
			Fixture f;
			f.scene.weather.gravity = XMFLOAT3(0, 0, 0);
			const Entity entity = f.Body(XMFLOAT3(0, 0, 0));
			auto& body = f.Get(entity);
			if (axis == 0) body.SetRotationLockedX(true);
			if (axis == 1) body.SetRotationLockedY(true);
			if (axis == 2) body.SetRotationLockedZ(true);
			f.Tick();
			XMFLOAT3 torque{};
			if (axis == 0) torque.x = 100;
			if (axis == 1) torque.y = 100;
			if (axis == 2) torque.z = 100;
			wi::physics::ApplyTorque(body, torque);
			f.Tick(5);
			const auto rotation = wi::physics::GetRotation(body);
			if (!Check(Near(rotation.x, 0, 0.001f) && Near(rotation.y, 0, 0.001f) && Near(rotation.z, 0, 0.001f),
				"individual angular axis lock did not constrain its axis"))
				return false;
			torque = axis == 0 ? XMFLOAT3(0, 100, 0) : XMFLOAT3(100, 0, 0);
			wi::physics::ApplyTorque(body, torque);
			f.Tick(5);
			const auto free_rotation = wi::physics::GetRotation(body);
			if (!Check(std::abs(free_rotation.x) + std::abs(free_rotation.y) > 0.01f,
				"individual angular lock constrained an unrelated axis"))
				return false;
		}
		for (uint64_t version : { uint64_t(0), uint64_t(6) })
		{
			RigidBodyPhysicsComponent authored;
			authored.SetRotationLockedX(true);
			authored.SetRotationLockedY(true);
			authored.SetRotationLockedZ(true);
			wi::ecs::EntitySerializer serializer;
			serializer.version = version;
			wi::Archive archive;
			authored.Serialize(archive, serializer);
			wi::vector<uint8_t> bytes;
			archive.WriteData(bytes);
			wi::Archive reader(bytes.data(), bytes.size());
			RigidBodyPhysicsComponent loaded;
			loaded.Serialize(reader, serializer);
			if (!Check(loaded.IsRotationLockedX() && loaded.IsRotationLockedY() && loaded.IsRotationLockedZ(),
				"rotation locks did not survive existing serialized flags layout"))
				return false;
		}
		Fixture f;
		const Entity floor = f.Body(XMFLOAT3(0, -0.5f, 0), 0);
		f.Get(floor).box.halfextents = XMFLOAT3(10, 0.5f, 10);
		const Entity capsule = f.Body(XMFLOAT3(0, 2, 0), 1, RigidBodyPhysicsComponent::CAPSULE);
		auto& body = f.Get(capsule);
		if (!Check(!body.IsRotationLockedX() && !body.IsRotationLockedY() && !body.IsRotationLockedZ(),
			"new rotation locks must default off"))
			return false;
		body.capsule.radius = 0.35f;
		body.capsule.height = 0.55f;
		body.friction = 0;
		body.damping_linear = 0;
		body.SetRotationLockedX(true);
		body.SetRotationLockedY(true);
		body.SetRotationLockedZ(true);
		f.Tick();
		const wi::primitive::Ray ray(XMFLOAT3(0, 5, 0), XMFLOAT3(0, -1, 0), 0, 10);
		if (!Check(wi::physics::Intersects(f.scene, ray).entity == capsule, "unfiltered query missed capsule") ||
			!Check(wi::physics::Intersects(f.scene, ray, capsule).entity == floor,
				"self-exclusion did not continue to floor behind capsule") ||
			!Check(wi::physics::Intersects(f.scene, ray, floor).entity == capsule, "filter excluded unrelated body"))
			return false;
		wi::physics::ApplyTorque(body, XMFLOAT3(200, 200, 200));
		wi::physics::ApplyImpulseAt(body, XMFLOAT3(1, 0, 0), XMFLOAT3(0, 0.5f, 0));
		if (!Check(Near(wi::physics::GetVelocity(body).x, 1), "angular locks changed authored mass"))
			return false;
		f.Tick(120);
		const XMFLOAT4 rotation = f.scene.transforms.GetComponent(capsule)->rotation_local;
		if (!Check(Near(rotation.x, 0, 0.001f) && Near(rotation.y, 0, 0.001f) && Near(rotation.z, 0, 0.001f),
			"solver rotation locks allowed capsule to tilt") ||
			!Check(wi::physics::GetPosition(body).x > 0.1f, "rotation locks also locked translation"))
			return false;
		body.SetRotationLockedZ(false);
		f.Tick();
		wi::physics::ApplyTorque(body, XMFLOAT3(0, 0, 200));
		f.Tick(5);
		return Check(std::abs(f.scene.transforms.GetComponent(capsule)->rotation_local.z) > 0.01f,
			"rotation lock refresh did not unlock angular motion");
	}

	bool LuaIdentity()
	{
		wi::lua::Initialize();
		const std::string script = std::string("assert(physics.GetBackendName() == '") +
			wi::physics::GetBackendName() + "')\n"
			"local body = RigidBodyPhysicsComponent()\n"
			"assert(not body.IsRotationLockedX() and not body.IsRotationLockedY() and not body.IsRotationLockedZ())\n"
			"body.SetRotationLockedX(true); body.SetRotationLockedY(true); body.SetRotationLockedZ(true)\n"
			"assert(body.IsRotationLockedX() and body.IsRotationLockedY() and body.IsRotationLockedZ())\n"
			"body.SetRotationLockedY(false); assert(not body.IsRotationLockedY())\n"
			"local scene = Scene()\n"
			"local entity = physics.Intersects(scene, Ray(), 12345)\n"
			"assert(entity == 0)\n";
		return Check(wi::lua::RunText(script), "Lua physics backend identity did not match the engine");
	}

	bool Picking()
	{
		Fixture f;
		f.scene.weather.gravity = XMFLOAT3(0, 0, 0);
		const Entity body = f.Body(XMFLOAT3(0, 0, 0));
		f.Tick();
		wi::physics::PickDragOperation drag;
		const wi::primitive::Ray ray(XMFLOAT3(0, 3, 0), XMFLOAT3(0, -1, 0), 0, 10);
		wi::physics::PickDrag(f.scene, ray, drag);
		if (!Check(drag.IsValid(), "picking did not create a drag operation"))
			return false;
		wi::physics::PickDrag(f.scene, wi::primitive::Ray(
			XMFLOAT3(1, 3, 0), XMFLOAT3(0, -1, 0), 0, 10), drag);
		f.Tick(30);
		if (!Check(wi::physics::GetPosition(f.Get(body)).x > 0.5f, "dragging did not move the body"))
			return false;
		f.scene.rigidbodies.Remove(body);
		wi::physics::FlushDeferredDeletes(f.scene);
		wi::physics::PickDrag(f.scene, ray, drag);
		drag.internal_state.reset();
		wi::physics::FlushDeferredDeletes(f.scene);
		return true;
	}

	bool Box3DSpecific()
	{
		if (std::strcmp(wi::physics::GetBackendName(), "Box3D") != 0)
			return true;
		using Feature = wi::physics::Feature;
		if (!Check(wi::physics::GetUnsupportedReason(Feature::SoftBodies) != nullptr, "soft bodies falsely advertised") ||
			!Check(wi::physics::GetUnsupportedReason(PhysicsConstraintComponent::Type::SixDOF) != nullptr,
				"SixDOF falsely advertised"))
			return false;
		Fixture f;
		f.scene.weather.gravity = XMFLOAT3(0, 0, 0);
		const Entity body = f.Body(XMFLOAT3(0, 0, 0));
		const Entity character = f.Body(XMFLOAT3(5, 0, 0));
		f.Get(character).SetCharacterPhysics(true);
		f.Tick();
		if (!Check(f.Get(character).physicsobject == nullptr, "unsupported character was registered") ||
			!Check(f.Get(character).IsCharacterPhysics(), "unsupported authored flag was erased"))
			return false;
		f.Get(character).SetCharacterPhysics(false);
		f.Get(character).SetRefreshParametersNeeded();
		f.Tick();
		if (!Check(f.Get(character).physicsobject != nullptr, "correcting an unsupported component did not retry registration"))
			return false;
		auto* component = &f.Get(body);
		std::atomic<bool> failed{ false };
		std::thread writer([&] {
			for (int i = 0; i < 100; ++i)
			{
				wi::physics::SetLinearVelocity(*component, XMFLOAT3(1, 0, 0));
				if (!std::isfinite(wi::physics::GetPosition(*component).x))
					failed = true;
			}
		});
		f.Tick(100);
		writer.join();
		if (!Check(!failed, "concurrent serialized operations returned invalid state"))
			return false;
		auto detached = std::move(component->physicsobject);
		std::thread destroyer([state = std::move(detached)]() mutable { state.reset(); });
		destroyer.join();
		wi::physics::SetEnabled(false);
		wi::physics::RunPhysicsUpdateSystem(f.context, f.scene, 0);
		wi::physics::SetEnabled(true);
		return true;
	}

	bool Box3DLifetimeAndCooking()
	{
		if (std::strcmp(wi::physics::GetBackendName(), "Box3D") != 0)
			return true;
		using Shape = RigidBodyPhysicsComponent::CollisionShape;
		Fixture source;
		Fixture target;
		source.scene.weather.gravity = target.scene.weather.gravity = XMFLOAT3(0, 0, 0);
		const Entity body = source.Body(XMFLOAT3(10, 0, 0));
		target.Body(XMFLOAT3(0, 0, 0));
		source.Tick();
		target.Tick();
		target.scene.Merge(source.scene);
		target.Tick();
		source.scene.Clear();
		const wi::primitive::Ray ray(XMFLOAT3(10, 3, 0), XMFLOAT3(0, -1, 0), 0, 10);
		if (!Check(wi::physics::Intersects(target.scene, ray).entity == body,
			"merging live scenes did not transfer the body to the destination world"))
			return false;

		const Entity meshBody = target.Body(XMFLOAT3(20, 0, 0), 1, Shape::TRIANGLE_MESH);
		target.AttachGeometry(meshBody, {
			XMFLOAT3(-2, 0, -2), XMFLOAT3(2, 0, -2),
			XMFLOAT3(-2, 0, 2), XMFLOAT3(2, 0, 2),
		}, { 0, 1, 2, 1, 3, 2 });
		target.Tick();
		if (!Check(target.Get(meshBody).physicsobject == nullptr, "dynamic triangle mesh was silently registered"))
			return false;
		target.Get(meshBody).mass = 0;
		target.Get(meshBody).SetRefreshParametersNeeded();
		target.Tick();
		if (!Check(target.Get(meshBody).physicsobject != nullptr, "fixing mesh configuration did not retry registration"))
			return false;
		auto* geometry = target.scene.meshes.GetComponent(target.scene.objects.GetComponent(meshBody)->meshID);
		for (auto& position : geometry->vertex_positions)
			position.y += 2;
		target.Get(meshBody).physicsobject.reset();
		target.Tick();
		const auto hit = wi::physics::Intersects(target.scene,
			wi::primitive::Ray(XMFLOAT3(20, 4, 0), XMFLOAT3(0, -1, 0), 0, 10));
		if (!Check(hit.entity == meshBody && Near(hit.position.y, 2), "recreated mesh reused stale cooked geometry"))
			return false;

		wi::vector<Entity> entities;
		wi::vector<wi::allocator::shared_ptr<void>> detached;
		for (int i = 0; i < 128; ++i)
			entities.push_back(target.Body(XMFLOAT3(30 + 2.0f * i, 0, 0)));
		target.Tick();
		for (Entity entity : entities)
		{
			detached.push_back(std::move(target.Get(entity).physicsobject));
			target.scene.rigidbodies.Remove(entity);
		}
		std::thread destroyer([states = std::move(detached)]() mutable {
			for (auto& state : states)
			{
				state.reset();
				std::this_thread::yield();
			}
		});
		target.Tick(100);
		destroyer.join();
		wi::physics::FlushDeferredDeletes(target.scene);
		return Check(wi::physics::Intersects(target.scene, ray).entity == body,
			"concurrent deferred teardown corrupted a surviving body");
	}

	bool Box3DParallelWorlds()
	{
		if (std::strcmp(wi::physics::GetBackendName(), "Box3D") != 0)
			return true;
		std::atomic<bool> failed{ false };
		wi::vector<std::thread> workers;
		for (int worker = 0; worker < 4; ++worker)
		{
			workers.emplace_back([&] {
				for (int iteration = 0; iteration < 12; ++iteration)
				{
					Fixture f;
					f.scene.weather.gravity = XMFLOAT3(0, 0, 0);
					const Entity body = f.Body(XMFLOAT3(0, 0, 0));
					f.Tick();
					wi::physics::SetLinearVelocity(f.Get(body), XMFLOAT3(1, 0, 0));
					f.Tick(10);
					const float x = wi::physics::GetPosition(f.Get(body)).x;
					if (!(x > 0.1f && x < 0.2f))
						failed = true;
				}
			});
		}
		for (auto& worker : workers)
			worker.join();
		return Check(!failed, "parallel world creation, stepping or destruction corrupted simulation");
	}

#ifdef WICKEDENGINE_BUILD_VULKAN
	bool PhysicsSampleScene(const char* filename)
	{
		Fixture f;
		wi::Archive archive(filename, true);
		if (!Check(archive.IsOpen(), "could not open the physics sample scene"))
			return false;
		f.scene.Serialize(archive);
		f.scene.StartBuildTopDownHierarchy();
		f.scene.WaitBuildTopDownHierarchy();
		f.Tick();
		wi::vector<Entity> dynamic;
		float floor_height = -FLT_MAX;
		float floor_min_x = FLT_MAX, floor_max_x = -FLT_MAX;
		float floor_min_z = FLT_MAX, floor_max_z = -FLT_MAX;
		for (size_t i = 0; i < f.scene.rigidbodies.GetCount(); ++i)
		{
			const Entity entity = f.scene.rigidbodies.GetEntity(i);
			const auto& body = f.Get(entity);
			const auto* name = f.scene.names.GetComponent(entity);
			std::cout << "Sample body: " << (name ? name->name : "<unnamed>")
				<< ", shape=" << int(body.shape) << ", mass=" << body.mass
				<< ", kinematic=" << body.IsKinematic()
				<< ", registered=" << bool(body.physicsobject) << '\n';
			if (!Check(body.physicsobject != nullptr, "physics sample contains an unregistered rigid body"))
				return false;
			if (body.mass > 0 && !body.IsKinematic())
			{
				dynamic.push_back(entity);
			}
			else
			{
				const auto* object = f.scene.objects.GetComponent(entity);
				const auto* mesh = object ? f.scene.meshes.GetComponent(object->meshID) : nullptr;
				if (!mesh)
					continue;
				const auto matrix = XMLoadFloat4x4(&f.scene.transforms.GetComponent(entity)->world);
				for (const auto& vertex : mesh->vertex_positions)
				{
					XMFLOAT3 position;
					XMStoreFloat3(&position, XMVector3TransformCoord(XMLoadFloat3(&vertex), matrix));
					floor_height = std::max(floor_height, position.y);
					floor_min_x = std::min(floor_min_x, position.x);
					floor_max_x = std::max(floor_max_x, position.x);
					floor_min_z = std::min(floor_min_z, position.z);
					floor_max_z = std::max(floor_max_z, position.z);
				}
			}
		}
		if (!Check(!dynamic.empty() && floor_height > -FLT_MAX, "physics sample is missing its floor or dynamic bodies"))
			return false;
		for (size_t i = 0; i < f.scene.softbodies.GetCount(); ++i)
		{
			const auto* name = f.scene.names.GetComponent(f.scene.softbodies.GetEntity(i));
			std::cout << "Sample soft body: " << (name ? name->name : "<unnamed>")
				<< ", supported=" << (wi::physics::GetUnsupportedReason(wi::physics::Feature::SoftBodies) == nullptr) << '\n';
		}
		wi::vector<float> previous_velocity;
		wi::vector<bool> bounced(dynamic.size(), false);
		for (Entity entity : dynamic)
			previous_velocity.push_back(wi::physics::GetVelocity(f.Get(entity)).y);
		for (int frame = 0; frame < 600; ++frame)
		{
			f.Tick();
			for (size_t i = 0; i < dynamic.size(); ++i)
			{
				auto& body = f.Get(dynamic[i]);
				const auto position = wi::physics::GetPosition(body);
				const float velocity = wi::physics::GetVelocity(body).y;
				// Bouncing objects may leave this finite floor; only test penetration
				// away from its edges, not their height after falling off the platform.
				const bool over_floor = position.x > floor_min_x + 1 && position.x < floor_max_x - 1 &&
					position.z > floor_min_z + 1 && position.z < floor_max_z - 1;
				if (over_floor && !Check(position.y >= floor_height - 0.1f, "physics sample rigid body fell through its floor"))
					return false;
				if (over_floor && position.y < floor_height + 3 && previous_velocity[i] < -0.2f && velocity >= 0)
					bounced[i] = true;
				previous_velocity[i] = velocity;
			}
		}
		const auto bounce_count = std::count(bounced.begin(), bounced.end(), true);
		std::cout << "Simulated " << dynamic.size() << " sample rigid bodies for 10 seconds; "
			<< bounce_count << " bounced near the floor\n";
		return Check(bounce_count > 0, "physics sample had no rigid-body bounces near its floor");
	}
#endif
}

int main(int argc, char** argv)
{
	const bool sample_scene = argc == 3 && std::strcmp(argv[1], "--sample-scene") == 0;
	if (argc != 1 && !sample_scene)
	{
		std::cerr << "Usage: PhysicsContractTest [--sample-scene path/to/physics_test.wiscene]\n";
		return 1;
	}
#ifdef WICKEDENGINE_BUILD_VULKAN
	std::unique_ptr<wi::graphics::GraphicsDevice_Vulkan> graphics_device;
	if (sample_scene)
	{
#if defined(SDL3)
		const bool initialized = SDL_Init(SDL_INIT_VIDEO) && SDL_Vulkan_LoadLibrary(nullptr);
#elif defined(SDL2)
		const bool initialized = SDL_Init(SDL_INIT_VIDEO) == 0 && SDL_Vulkan_LoadLibrary(nullptr) == 0;
#endif
#if defined(SDL3) || defined(SDL2)
		if (!initialized)
		{
			std::cerr << "Could not initialize SDL/Vulkan for the sample scene: " << SDL_GetError() << '\n';
			SDL_Quit();
			return 1;
		}
#endif
		graphics_device = std::make_unique<wi::graphics::GraphicsDevice_Vulkan>(nullptr);
		wi::graphics::GetDevice() = graphics_device.get();
	}
#else
	if (sample_scene)
	{
		std::cerr << "The physics sample scene check requires Vulkan on Linux.\n";
		return 1;
	}
#endif
	wi::jobsystem::Initialize(4);
	wi::physics::Initialize();
	wi::physics::SetFrameRate(60);
	wi::physics::SetAccuracy(8);
	wi::physics::SetInterpolationEnabled(false);
	bool passed;
#ifdef WICKEDENGINE_BUILD_VULKAN
	if (sample_scene)
	{
		passed = PhysicsSampleScene(argv[2]);
	}
	else
#endif
	{
		passed = Simulation() && QueryAndLifetime() && ForcesAndGhosts() &&
			Shapes() && Constraints() && CookedGeometry() && PlanarConvexFloor() && OffsetsAndParenting() &&
			RefreshAndInterpolation() && Kinematics() && Picking() && Box3DSpecific() &&
			Box3DLifetimeAndCooking() && Box3DParallelWorlds() && RotationLocksAndFilteredQueries() && LuaIdentity();
	}
	wi::jobsystem::ShutDown();
#ifdef WICKEDENGINE_BUILD_VULKAN
	if (graphics_device)
	{
		wi::resourcemanager::Clear();
		// Deserialization allocates GPU pages whose releases require frame advancement.
		for (uint32_t i = 0; i <= graphics_device->GetBufferCount(); ++i)
		{
			graphics_device->BeginCommandList();
			graphics_device->SubmitCommandLists();
			wi::renderer::UpdateGPUSuballocator();
		}
		graphics_device->WaitForGPU();
	}
#endif
	wi::graphics::GetDevice() = nullptr;
#ifdef WICKEDENGINE_BUILD_VULKAN
	graphics_device.reset();
#if defined(SDL3) || defined(SDL2)
	if (sample_scene)
	{
		SDL_Vulkan_UnloadLibrary();
		SDL_Quit();
	}
#endif
#endif
	if (passed)
		std::cout << wi::physics::GetBackendName() << " physics contract passed\n";
	return passed ? 0 : 1;
}
