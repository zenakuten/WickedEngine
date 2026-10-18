#include "wiCSGScene.h"
#include "wiJobSystem.h"
#include <iostream>

namespace
{
	int failures = 0;
	void Check(bool value, const char* message)
	{
		if (!value) { std::cerr << message << '\n'; ++failures; }
	}
}
int main()
{
	using namespace wi::scene;
	using namespace wi::ecs;
	using namespace wi::csg;
	wi::jobsystem::Initialize(1);
	{
		Scene scene;
		const auto root = scene.Entity_CreateTransform("CSG test");
		const auto prop = scene.Entity_CreateTransform("Unrelated prop");
		auto& source = scene.csgs.Create(root);
		Brush outer, room;
		std::string error;
		MakeCube({20,10,20},1,outer,error);
		MakeCube({16,8,16},2,room,error);
		room.operation = Operation::Subtract;
		source.brushes = {outer,room};
		source.nextBrushID = 3;
		Check(Rebuild(scene,root,error),"Rebuild original CSG scene");
		Check(scene.objects.GetCount() == 12 && scene.materials.GetCount() == 12 &&
			!scene.csgs.GetComponent(root)->dirty,"Independent outer and room surfaces");
		const auto binding = scene.csgs.GetComponent(root)->surfaces[0];
		scene.materials.GetComponent(binding.material)->texMulAdd = XMFLOAT4(2,3,0.25f,0.5f);
		Check(Rebuild(scene,root,error),"Rebuild with authored material");
		Check(scene.transforms.Contains(prop) && scene.materials.GetComponent(binding.material)->texMulAdd.x == 2,
			"Rebuild preserves unrelated entities and surface texture edits");
		for (size_t i = 0; i < scene.objects.GetCount(); ++i)
		{
			auto e = scene.objects.GetEntity(i);
			const auto* mesh = scene.meshes.GetComponent(scene.objects[i].meshID);
			Check(mesh && !mesh->vertex_uvset_0.empty() && mesh->vertex_atlas.empty() &&
				scene.metadatas.GetComponent(e)->HideIcon,"Generated surfaces preserve primary UVs and hide icons");
			wi::primitive::AABB bounds;
			for (auto p : mesh->vertex_positions) bounds.AddPoint(p);
			Check(wi::math::Distance(bounds.getCenter(),XMFLOAT3(0,0,0)) < 1e-4f,"Generated pivots centered");
			for (size_t t = 0; t < mesh->indices.size(); t += 3)
			{
				auto a = XMLoadFloat3(&mesh->vertex_positions[mesh->indices[t]]);
				auto b = XMLoadFloat3(&mesh->vertex_positions[mesh->indices[t+1]]);
				auto c = XMLoadFloat3(&mesh->vertex_positions[mesh->indices[t+2]]);
				auto n = XMLoadFloat3(&mesh->vertex_normals[mesh->indices[t]]);
				Check(XMVectorGetX(XMVector3Dot(XMVector3Cross(b-a,c-a),n)) < 0,"Wicked clockwise winding");
			}
		}
		auto* current = scene.csgs.GetComponent(root);
		auto saved = current->brushes;
		current->brushes[0].vertices[0].x += 1;
		const size_t objects = scene.objects.GetCount();
		Check(!Rebuild(scene,root,error) && !error.empty() && scene.objects.GetCount() == objects,
			"Invalid source cannot replace previous generated geometry");
		current->brushes = saved;
		wi::Archive archive;
		scene.Serialize(archive);
		archive.SetReadModeAndResetPos(true);
		Scene restored;
		restored.Serialize(archive);
		auto restoredRoot = restored.Entity_FindByName("CSG test");
		const auto* restoredSource = restored.csgs.GetComponent(restoredRoot);
		Check(restoredSource && restoredSource->brushes.size() == 2 && restoredSource->nextBrushID == 3,
			"Native scene serialization preserves editable source and IDs");
		if (restoredSource)
		{
			Check(restoredSource->surfaces.size() == 12,"Native scene preserves source bindings");
			const auto& s = restoredSource->surfaces[0];
			Check(restored.materials.Contains(s.material) && restored.objects.Contains(s.object) &&
				restored.materials.GetComponent(s.material)->texMulAdd.y == 3,"Source references remap on native reload");
		}
		Check(Rebuild(restored,restoredRoot,error),"Reloaded source can rebuild without editor");
		wi::Archive duplicate;
		EntitySerializer write;
		scene.Entity_Serialize(duplicate,write,root);
		duplicate.SetReadModeAndResetPos(true);
		EntitySerializer read;
		const auto copy = scene.Entity_Serialize(duplicate,read);
		Check(copy != root && scene.csgs.Contains(copy),"Recursive duplication preserves source");
		const auto* copied = scene.csgs.GetComponent(copy);
		if (copied)
			Check(copied->surfaces[0].material != binding.material &&
				scene.materials.Contains(copied->surfaces[0].material),"Duplicating CSG remaps material references");
		Check(Rebuild(scene,copy,error) && scene.transforms.Contains(prop),"Duplicated CSG can rebuild independently");
		current = scene.csgs.GetComponent(root);
		current->brushes.clear();
		Check(Rebuild(scene,root,error) && scene.csgs.GetComponent(root)->surfaces[0].object == INVALID_ENTITY &&
			scene.materials.Contains(binding.material),"Empty rebuild removes objects but retains face materials");
		current = scene.csgs.GetComponent(root);
		current->brushes = saved;
		Check(Rebuild(scene,root,error) && scene.materials.GetComponent(binding.material)->texMulAdd.x == 2,
			"Reappearing surfaces retain authored material mappings");
		auto capture = [&]() {
			wi::Archive snapshot;
			snapshot << scene.csgs.Contains(root);
			EntitySerializer serializer;
			serializer.allow_remap = false;
			scene.Entity_Serialize(snapshot,serializer,root);
			wi::vector<uint8_t> bytes;
			snapshot.WriteData(bytes);
			return bytes;
		};
		const auto before = capture();
		scene.csgs.GetComponent(root)->brushes.clear();
		Check(Rebuild(scene,root,error),"Build empty history result");
		const auto after = capture();
		wi::Archive history;
		history << int(123) << root << before << after;
		history.SetReadModeAndResetPos(true);
		int operation = 0;
		Entity historyRoot = INVALID_ENTITY;
		history >> operation >> historyRoot;
		Check(operation == 123 && historyRoot == root,"History prefix does not change snapshot offsets");
		for (bool undo : {true,false})
		{
			wi::vector<uint8_t> bytes;
			history >> bytes;
			wi::Archive stored(bytes.data(),bytes.size());
			bool exists = false;
			stored >> exists;
			Scene snapshot;
			EntitySerializer serializer;
			serializer.allow_remap = false;
			snapshot.Entity_Serialize(stored,serializer);
			Check(exists && snapshot.csgs.Contains(root) &&
				snapshot.csgs.GetComponent(root)->brushes.size() == (undo ? 2u : 0u) &&
				snapshot.objects.GetCount() == (undo ? 12u : 0u),
				"Length-delimited history restores source and generated entities independently");
		}
	}
	wi::jobsystem::ShutDown();
	if (!failures) std::cout << "CSG scene persistence and rebuild contracts passed\n";
	return failures ? 1 : 0;
}
