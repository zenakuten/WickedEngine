#include "wiCSGScene.h"
#include <algorithm>
#include <map>

namespace wi::csg
{
	bool EvaluateScene(const wi::scene::Scene& scene, wi::ecs::Entity root, std::vector<Triangle>& triangles, std::string& error)
	{
		const auto* source = scene.csgs.GetComponent(root);
		if (!source) { error = "Select a CSG root"; return false; }
		std::vector<Polygon> polygons;
		if (!Evaluate(source->brushes, polygons, error) || !Triangulate(polygons, triangles, error)) return false;
		return true;
	}
	bool Rebuild(wi::scene::Scene& scene, wi::ecs::Entity root, std::string& error)
	{
		auto* source = scene.csgs.GetComponent(root);
		if (!source) { error = "Select a CSG root"; return false; }
		std::vector<Triangle> triangles;
		if (!EvaluateScene(scene, root, triangles, error)) return false;
		std::map<std::pair<uint64_t, uint64_t>, std::vector<const Triangle*>> groups;
		for (const auto& t : triangles) groups[{t.brushID, t.faceID}].push_back(&t);
		wi::scene::Scene staging;
		auto bindings = source->surfaces;
		for (auto& binding : bindings)
		{
			const auto* object = scene.objects.GetComponent(binding.object);
			const auto* mesh = object ? scene.meshes.GetComponent(object->meshID) : nullptr;
			if (mesh && !mesh->subsets.empty() && scene.materials.Contains(mesh->subsets[0].materialID))
				binding.material = mesh->subsets[0].materialID;
			binding.object = wi::ecs::INVALID_ENTITY;
		}
		for (const auto& group : groups)
		{
			const auto key = group.first;
			auto it = std::find_if(bindings.begin(), bindings.end(), [&](const auto& b) {
				return b.brushID == key.first && b.faceID == key.second;
			});
			if (it == bindings.end())
			{
				bindings.push_back({key.first, key.second});
				it = bindings.end() - 1;
			}
			const std::string name = "CSG " + std::to_string(key.first) + ":" + std::to_string(key.second);
			if (!scene.materials.Contains(it->material))
			{
				it->material = staging.Entity_CreateMaterial(name + " material");
				auto& material = *staging.materials.GetComponent(it->material);
				material.SetBaseColor(XMFLOAT4(0.8f, 0.8f, 0.8f, 1));
				material.SetRoughness(0.8f);
				staging.hierarchy.Create(it->material).parentID = root;
			}
			const auto object = staging.Entity_CreateObject(name);
			it->object = object;
			auto& mesh = staging.meshes.Create(object);
			staging.objects.GetComponent(object)->meshID = object;
			wi::primitive::AABB bounds;
			for (const auto* t : group.second)
				for (Point p : t->points) bounds.AddPoint(XMFLOAT3(float(p.x), float(p.y), float(p.z)));
			const auto center = bounds.getCenter();
			for (const auto* t : group.second)
			{
				for (int corner : {0, 2, 1})
				{
					const Point p = t->points[corner];
					mesh.indices.push_back(uint32_t(mesh.vertex_positions.size()));
					mesh.vertex_positions.emplace_back(float(p.x) - center.x, float(p.y) - center.y, float(p.z) - center.z);
					mesh.vertex_normals.emplace_back(float(t->normal.x), float(t->normal.y), float(t->normal.z));
					mesh.vertex_uvset_0.emplace_back(float(Dot(p,t->u) + t->offsetU), float(Dot(p,t->v) + t->offsetV));
				}
			}
			mesh.subsets.resize(1);
			mesh.subsets[0].indexCount = uint32_t(mesh.indices.size());
			mesh.subsets[0].materialID = it->material;
			auto& metadata = staging.metadatas.Create(object);
			metadata.HideIcon = true;
			metadata.string_values.set("csg.surface", name);
			staging.transforms.GetComponent(object)->Translate(center);
			staging.hierarchy.Create(object).parentID = root;
			if (wi::graphics::GetDevice()) mesh.CreateRenderData();
		}
		// No mutation of the authored scene occurs until all geometry is ready.
		for (const auto& binding : source->surfaces)
			if (binding.object != wi::ecs::INVALID_ENTITY) scene.Entity_Remove(binding.object);
		scene.Merge(staging);
		source = scene.csgs.GetComponent(root);
		source->surfaces = std::move(bindings);
		source->dirty = false;
		error.clear();
		return true;
	}
}
