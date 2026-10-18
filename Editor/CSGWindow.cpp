#include "stdafx.h"
#include "Editor.h"
#include "CSGWindow.h"
#include "CSGSnap.h"
#include <numeric>
#include <set>

using namespace wi::scene;
using namespace wi::ecs;
using namespace wi::csg;
namespace
{
	XMFLOAT3 Float(Point p) { return {float(p.x),float(p.y),float(p.z)}; }
	Point Double(XMFLOAT3 p) { return {p.x,p.y,p.z}; }
	template<typename Source>
	auto Selected(Source& source, uint64_t id) -> decltype(source.brushes.data())
	{
		for (auto& brush : source.brushes) if (brush.id == id) return &brush;
		return nullptr;
	}
	XMFLOAT3 World(Point p, const Scene& scene, Entity root)
	{
		const auto* transform = scene.transforms.GetComponent(root);
		XMFLOAT3 result = Float(p);
		if (transform) XMStoreFloat3(&result, XMVector3TransformCoord(XMLoadFloat3(&result), transform->GetWorldMatrix()));
		return result;
	}
	// Faces split off by a vertex edit share their source face's material.
	void InheritSurfaces(CSGComponent& c, const Brush& before, const Brush& after)
	{
		for (const auto& face : after.faces)
		{
			const uint64_t source = SourceFace(before,after,face.id);
			if (source == 0 || source == face.id) continue;
			auto bound = [&](uint64_t id) {
				return std::find_if(c.surfaces.begin(),c.surfaces.end(),[&](const auto& s) { return s.brushID == after.id && s.faceID == id; });
			};
			const auto from = bound(source);
			if (from == c.surfaces.end() || bound(face.id) != c.surfaces.end()) continue;
			c.surfaces.push_back({after.id,face.id,from->material});
		}
	}
}

void CSGWindow::Create(EditorComponent* value)
{
	editor = value;
	wi::gui::Window::Create("CSG", wi::gui::Window::WindowControls::CLOSE | wi::gui::Window::WindowControls::RESIZE_RIGHT);
	SetText("CSG options");
	SetSize(XMFLOAT2(340,740));
	auto button = [&](wi::gui::Button& widget, const char* text, std::function<void()> action) {
		widget.Create(text);
		widget.OnClick([action](wi::gui::EventArgs) { action(); });
		AddWidget(&widget);
	};
	auto input = [&](wi::gui::TextInputField& widget, const char* text, double& number) {
		widget.Create(text);
		widget.SetDescription(text);
		widget.SetValue(float(number));
		widget.OnInputAccepted([&number](wi::gui::EventArgs args) { number = args.fValue; });
		AddWidget(&widget);
	};
	roots.Create("CSG level");
	roots.OnSelect([this](wi::gui::EventArgs args) {
		auto& state = editor->GetCurrentEditorScene().csg;
		state.root = roots.GetItemUserData(args.iValue);
		state.brush = 0; state.vertices.clear(); state.hasPreview = false;
	});
	AddWidget(&roots);
	brushes.Create("Source brush");
	brushes.OnSelect([this](wi::gui::EventArgs args) {
		auto& state = editor->GetCurrentEditorScene().csg;
		state.brush = brushes.GetItemUserData(args.iValue);
		state.face = 0; state.vertices.clear();
	});
	AddWidget(&brushes);
	mode.Create("Edit mode");
	for (auto text : {"Surface/material", "Brush move", "Vertex edit", "Face drag"}) mode.AddItem(text);
	mode.OnSelect([this](wi::gui::EventArgs args) {
		CancelDrag();
		editor->GetCurrentEditorScene().csg.mode = args.iValue;
		editor->ClearSelected();
	});
	AddWidget(&mode);
	operation.Create("Operation");
	operation.AddItem("Add"); operation.AddItem("Subtract");
	operation.OnSelect([this](wi::gui::EventArgs args) {
		auto& state = editor->GetCurrentEditorScene().csg;
		if (state.hasPreview) state.preview.operation = Operation(args.iValue);
		else if (state.brush) Apply([&](CSGComponent& c, std::string& error) {
			auto* b = Selected(c,state.brush);
			if (!b) { error = "Select a source brush"; return false; }
			b->operation = Operation(args.iValue); return true;
		});
	});
	AddWidget(&operation);
	input(width,"Cube X",dimensions.x); input(height,"Height Y",dimensions.y);
	input(depth,"Cube Z",dimensions.z); input(radius,"Cone radius",coneRadius);
	input(sides,"Cone sides",coneSides);
	input(posX,"Center X",position.x); input(posY,"Center Y",position.y); input(posZ,"Center Z",position.z);
	button(cube,"Cube brush",[this] { Preview(false); });
	button(cone,"Cone brush",[this] { Preview(true); });
	button(commit,"Commit brush",[this] {
		auto& state = editor->GetCurrentEditorScene().csg;
		if (!state.hasPreview) { Report("Create a builder preview first",true); return; }
		const Brush preview = state.preview;
		uint64_t newID = 0;
		if (Apply([&](CSGComponent& c, std::string& error) {
			if (c.brushes.size() >= Limits{}.brushes || c.nextBrushID == UINT64_MAX)
			{ error = "CSG brush/ID limit exceeded"; return false; }
			Brush b = preview; b.id = c.nextBrushID++;
			newID = b.id; c.brushes.push_back(std::move(b)); return true;
		}))
		{
			state.brush = newID;
			state.face = 0;
			state.vertices.clear();
			state.hasPreview = false;
		}
	});
	button(cancel,"Cancel preview",[this] { editor->GetCurrentEditorScene().csg.hasPreview = false; CancelDrag(); });
	button(remove,"Delete brush",[this] {
		auto& state = editor->GetCurrentEditorScene().csg;
		if (Apply([&](CSGComponent& c, std::string& error) {
			auto* b = Selected(c,state.brush);
			if (!b) { error = "Select a source brush"; return false; }
			c.brushes.erase(c.brushes.begin() + (b - c.brushes.data())); return true;
		}))
		{
			state.brush = 0; state.vertices.clear();
		}
	});
	button(up,"Earlier",[this] { ChangeOrder(-1); });
	button(down,"Later",[this] { ChangeOrder(1); });
	button(enabled,"Enable / disable",[this] {
		auto& state = editor->GetCurrentEditorScene().csg;
		Apply([&](CSGComponent& c, std::string& error) {
			auto* b = Selected(c,state.brush);
			if (!b) { error = "Select a source brush"; return false; }
			b->enabled = !b->enabled; return true;
		});
	});
	input(deltaX,"Move X",delta.x); input(deltaY,"Move Y",delta.y); input(deltaZ,"Move Z",delta.z);
	button(move,"Move selection",[this] {
		auto& state = editor->GetCurrentEditorScene().csg;
		Apply([&](CSGComponent& c, std::string& error) {
			auto* b = Selected(c,state.brush);
			if (!b) { error = "Select a source brush"; return false; }
			auto vertices = state.vertices;
			if (state.mode != 2) { vertices.resize(b->vertices.size()); std::iota(vertices.begin(),vertices.end(),0u); }
			const Brush before = *b;
			if (!MoveVertices(*b,vertices,delta,error)) return false;
			InheritSurfaces(c,before,*b);
			return true;
		});
	});
	input(scale,"Uniform scale",uniformScale);
	button(resize,"Scale brush",[this] {
		auto& state = editor->GetCurrentEditorScene().csg;
		Apply([&](CSGComponent& c, std::string& error) {
			auto* b = Selected(c,state.brush);
			if (!b || !std::isfinite(uniformScale) || uniformScale == 0)
			{ error = "Select a brush and use a finite nonzero scale"; return false; }
			Point center;
			for (auto p : b->vertices) center = center + p * (1.0 / b->vertices.size());
			for (auto& p : b->vertices) p = center + (p - center) * uniformScale;
			if (uniformScale < 0)
				for (auto& f : b->faces) std::reverse(f.vertices.begin(),f.vertices.end());
			return Validate(*b,error);
		});
	});
	rotationAxis.Create("Rotation axis");
	for (auto axis : {"X","Y","Z"}) rotationAxis.AddItem(axis);
	rotationAxis.SetSelectedWithoutCallback(1);
	AddWidget(&rotationAxis);
	input(angle,"Rotation degrees",rotationDegrees);
	button(rotate,"Rotate brush",[this] {
		auto& state = editor->GetCurrentEditorScene().csg;
		Apply([&](CSGComponent& c, std::string& error) {
			auto* b = Selected(c,state.brush);
			if (!b || !std::isfinite(rotationDegrees)) { error = "Select a brush and use a finite rotation"; return false; }
			Point center;
			for (auto p : b->vertices) center = center + p * (1.0 / b->vertices.size());
			double a = rotationDegrees * 3.14159265358979323846 / 180;
			auto rotation = [&](Point p) {
				if (rotationAxis.GetSelected() == 0) return Point{p.x,p.y*std::cos(a)-p.z*std::sin(a),p.y*std::sin(a)+p.z*std::cos(a)};
				if (rotationAxis.GetSelected() == 2) return Point{p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a),p.z};
				return Point{p.x*std::cos(a)+p.z*std::sin(a),p.y,-p.x*std::sin(a)+p.z*std::cos(a)};
			};
			for (auto& p : b->vertices) p = center + rotation(p-center);
			for (auto& f : b->faces)
			{
				auto u = rotation(f.u), v = rotation(f.v);
				f.offsetU += Dot(center,f.u-u); f.offsetV += Dot(center,f.v-v);
				f.u = u; f.v = v;
			}
			return Validate(*b,error);
		});
	});
	button(rebuild,"Rebuild CSG",[this] { Apply([](CSGComponent&,std::string&) { return true; },true); });
	status.Create("CSG status");
	status.SetText("Create a cube or cone preview. Rebuild explicitly after source edits.\nShift-click vertices for groups; Ctrl inverts grid snap; Esc cancels a drag.");
	status.SetSize(XMFLOAT2(300,90));
	AddWidget(&status);
	SetVisible(false);
}
void CSGWindow::ResizeLayout()
{
	wi::gui::Window::ResizeLayout();
	layout.add(roots); layout.add(brushes); layout.add(mode); layout.add(operation);
	layout.add(width); layout.add(height); layout.add(depth);
	layout.add(radius); layout.add(sides);
	layout.add(posX); layout.add(posY); layout.add(posZ);
	layout.add(cube,cone); layout.add(commit,cancel);
	layout.add_fullwidth(rebuild);
	layout.add(remove,enabled); layout.add(up,down);
	layout.add(deltaX); layout.add(deltaY); layout.add(deltaZ); layout.add_fullwidth(move);
	layout.add(scale); layout.add(resize); layout.add(rotationAxis); layout.add(angle); layout.add(rotate);
	layout.add_fullwidth(status);
}
void CSGWindow::Report(const std::string& message, bool error)
{
	editor->GetCurrentEditorScene().csg.status = message;
	if (error)
	{
		wi::backlog::post("CSG: " + message, wi::backlog::LogLevel::Error);
		editor->PostSaveText("CSG: " + message);
	}
}
wi::Archive CSGWindow::Snapshot(Entity root)
{
	wi::Archive archive;
	auto& scene = editor->GetCurrentScene();
	const bool exists = scene.csgs.Contains(root);
	archive << exists;
	if (exists)
	{
		EntitySerializer seri; seri.allow_remap = false;
		scene.Entity_Serialize(archive,seri,root);
	}
	return archive;
}
bool CSGWindow::Apply(const std::function<bool(CSGComponent&,std::string&)>& edit, bool evaluate)
{
	auto& state = editor->GetCurrentEditorScene().csg;
	auto& scene = editor->GetCurrentScene();
	const Entity oldRoot = state.root;
	auto before = Snapshot(oldRoot);
	CSGComponent candidate;
	if (auto* c = scene.csgs.GetComponent(oldRoot)) candidate = *c;
	std::string error;
	if (!edit(candidate,error)) { Report(error,true); return false; }
	std::vector<Polygon> checked;
	if (!Evaluate(candidate.brushes,checked,error)) { Report(error,true); return false; }
	const bool created = !scene.csgs.Contains(state.root);
	if (created)
	{
		state.root = scene.Entity_CreateTransform("CSG level");
		scene.csgs.Create(state.root);
	}
	auto& source = *scene.csgs.GetComponent(state.root);
	auto previous = source;
	source = std::move(candidate);
	source.dirty = true;
	if (evaluate && !Rebuild(scene,state.root,error))
	{
		if (created)
		{
			scene.Entity_Remove(state.root);
			state.root = oldRoot;
		}
		else source = std::move(previous);
		Report(error,true); return false;
	}
	auto after = Snapshot(state.root);
	auto& history = editor->AdvanceHistory();
	history << int(EditorComponent::HISTORYOP_CSG);
	history << state.root;
	wi::vector<uint8_t> beforeData, afterData;
	before.WriteData(beforeData);
	after.WriteData(afterData);
	history << beforeData << afterData;
	editor->componentsWnd.RefreshEntityTree();
	Report(evaluate ? "CSG rebuilt; changed surfaces have no baked lightmap." : "Source changed. Rebuild CSG to update surfaces.");
	return true;
}
void CSGWindow::RestoreHistory(wi::Archive& archive, bool undo)
{
	Entity root;
	archive >> root;
	Scene before, after;
	for (auto* snapshot : {&before,&after})
	{
		wi::vector<uint8_t> data;
		archive >> data;
		// Entity archives contain absolute jump offsets; decode each in its own address space.
		wi::Archive stored(data.data(),data.size());
		bool exists; stored >> exists;
		if (exists)
		{
			EntitySerializer seri; seri.allow_remap = false;
			snapshot->Entity_Serialize(stored,seri);
		}
	}
	auto& scene = editor->GetCurrentScene();
	scene.Entity_Remove(root);
	scene.Merge(undo ? before : after);
	auto& state = editor->GetCurrentEditorScene().csg;
	state.root = scene.csgs.Contains(root) ? root : INVALID_ENTITY;
	state.brush = 0; state.face = 0; state.vertices.clear(); CancelDrag();
	editor->ClearSelected();
	Report("CSG history restored");
}
void CSGWindow::Preview(bool isCone)
{
	Brush brush;
	std::string error;
	if (isCone && (!std::isfinite(coneSides) || coneSides < 3 || coneSides > 256 || std::floor(coneSides) != coneSides))
	{ Report("Cone sides must be an integer in [3,256]",true); return; }
	const bool ok = isCone ? MakeCone(coneRadius,dimensions.y,uint32_t(coneSides),1,brush,error) : MakeCube(dimensions,1,brush,error);
	if (!ok) { Report(error,true); return; }
	for (auto& p : brush.vertices) p = p + position;
	if (!Validate(brush,error)) { Report(error,true); return; }
	brush.operation = Operation(operation.GetSelected());
	auto& state = editor->GetCurrentEditorScene().csg;
	state.preview = std::move(brush); state.hasPreview = true;
	Report("Builder preview ready. Commit to add it to the source list.");
}
void CSGWindow::ChangeOrder(int direction)
{
	auto& state = editor->GetCurrentEditorScene().csg;
	Apply([&](CSGComponent& c,std::string& error) {
		auto* b = Selected(c,state.brush);
		if (!b) { error = "Select a source brush"; return false; }
		int index = int(b-c.brushes.data()), target = index + direction;
		if (target < 0 || target >= int(c.brushes.size())) { error = "Brush is already at the end of the list"; return false; }
		std::swap(c.brushes[index],c.brushes[target]); return true;
	});
}
bool CSGWindow::Editing() const
{
	return editor && !editor->scenes.empty() && IsVisible() && editor->GetCurrentEditorScene().csg.mode != 0;
}
void CSGWindow::CancelDrag()
{
	if (!editor || editor->scenes.empty()) return;
	auto& state = editor->GetCurrentEditorScene().csg;
	state.dragging = false; state.validDrag = false;
}
void CSGWindow::UpdateData()
{
	if (!IsVisible()) { CancelDrag(); return; }
	auto& state = editor->GetCurrentEditorScene().csg;
	const auto& scene = editor->GetCurrentScene();
	if (!scene.csgs.Contains(state.root) && scene.csgs.GetCount()) state.root = scene.csgs.GetEntity(0);
	roots.ClearItems();
	for (size_t i = 0; i < scene.csgs.GetCount(); ++i)
	{
		Entity e = scene.csgs.GetEntity(i);
		roots.AddItem("CSG level " + std::to_string(e),e);
	}
	roots.SetSelectedByUserdataWithoutCallback(state.root);
	brushes.ClearItems();
	if (const auto* c = scene.csgs.GetComponent(state.root))
	{
		for (const auto& b : c->brushes)
			brushes.AddItem(std::to_string(b.id) + (b.operation == Operation::Add ? " Add" : " Subtract") + (b.enabled ? "" : " [off]"),b.id);
		if (!state.hasPreview)
			if (const auto* b = Selected(*c,state.brush))
				operation.SetSelectedWithoutCallback(int(b->operation));
		status.SetText(std::string(c->dirty ? "NEEDS REBUILD\n" : "Built\n") + state.status);
	}
	else status.SetText(state.status);
	brushes.SetSelectedWithoutCallback(-1);
	brushes.SetSelectedByUserdataWithoutCallback(state.brush);
	mode.SetSelectedWithoutCallback(state.mode);
}

void CSGWindow::UpdateInput()
{
	if (!Editing() || wi::backlog::isActive()) return;
	auto& state = editor->GetCurrentEditorScene().csg;
	auto& scene = editor->GetCurrentScene();
	auto* source = scene.csgs.GetComponent(state.root);
	if (!source) return;
	if (wi::input::Press(wi::input::KEYBOARD_BUTTON_ESCAPE)) { CancelDrag(); return; }
	if (editor->GetGUI().HasFocus() && !state.dragging) return;
	const auto pointer = wi::input::GetPointer();
	const auto& pane = editor->viewportLayout.Panes[editor->activeView];
	const auto& canvas = editor->GetInteractionCanvas();
	const auto& camera = editor->GetInteractionCamera();
	XMFLOAT2 mouse = {pointer.x-editor->PhysicalToLogical(pane.top_left_x),pointer.y-editor->PhysicalToLogical(pane.top_left_y)};
	auto ray = wi::renderer::GetPickRay(long(mouse.x),long(mouse.y),canvas,camera);
	auto local = [&](XMFLOAT3 p, bool direction) {
		auto* t = scene.transforms.GetComponent(state.root);
		if (!t) return Double(p);
		XMMATRIX inverse = XMMatrixInverse(nullptr,t->GetWorldMatrix());
		XMStoreFloat3(&p,direction ? XMVector3TransformNormal(XMLoadFloat3(&p),inverse) : XMVector3TransformCoord(XMLoadFloat3(&p),inverse));
		return Double(p);
	};
	Point origin = local(ray.origin,false), direction = local(ray.direction,true);
	if (wi::input::Press(wi::input::MOUSE_BUTTON_LEFT) && editor->GetSceneViewportAtPointer() == editor->activeView)
	{
		double nearest = 1e30;
		Brush* selected = nullptr;
		uint64_t faceID = 0;
		uint32_t pickedVertex = UINT32_MAX;
		for (auto& b : source->brushes)
		{
			if (state.brush && b.id != state.brush && !wi::input::Down(wi::input::KEYBOARD_BUTTON_ALT))
				continue;
			for (const auto& f : b.faces)
			{
				Point p = b.vertices[f.vertices[0]], n = Cross(b.vertices[f.vertices[1]]-p,b.vertices[f.vertices[2]]-p);
				n = n * (1 / Length(n));
				double denom = Dot(n,direction);
				if (std::abs(denom) < 1e-12) continue;
				double distance = Dot(n,p-origin) / denom;
				if (distance < 0 || distance >= nearest) continue;
				Point hit = origin + direction * distance;
				bool inside = true;
				for (size_t i = 0; i < f.vertices.size(); ++i)
				{
					Point a = b.vertices[f.vertices[i]], bpoint = b.vertices[f.vertices[(i+1)%f.vertices.size()]];
					if (Dot(Cross(bpoint-a,hit-a),n) < -1e-6) inside = false;
				}
				if (inside) { nearest = distance; selected = &b; faceID = f.id; }
			}
		}
		if (state.mode == 2)
		{
			selected = Selected(*source,state.brush);
			if (!selected) { Report("Choose a brush from the source list before editing vertices"); return; }
			double best = 100;
			uint32_t vertex = UINT32_MAX;
			for (uint32_t i = 0; i < selected->vertices.size(); ++i)
			{
				auto world = World(selected->vertices[i],scene,state.root);
				XMFLOAT3 p; XMStoreFloat3(&p,XMVector3TransformCoord(XMLoadFloat3(&world),camera.GetViewProjection()));
				if (p.z < 0 || p.z > 1) continue;
				double x = (p.x+1)*0.5*canvas.GetLogicalWidth()-mouse.x, y = (1-p.y)*0.5*canvas.GetLogicalHeight()-mouse.y;
				if (x*x+y*y < best) { best = x*x+y*y; vertex = i; }
			}
			if (vertex == UINT32_MAX) return;
			pickedVertex = vertex;
			std::vector<uint32_t> picked = {vertex};
			if (editor->activeView != 0)
			{
				auto world = World(selected->vertices[vertex],scene,state.root);
				XMFLOAT3 first; XMStoreFloat3(&first,XMVector3TransformCoord(XMLoadFloat3(&world),camera.GetViewProjection()));
				for (uint32_t i = 0; i < selected->vertices.size(); ++i)
				{
					if (i == vertex) continue;
					world = World(selected->vertices[i],scene,state.root);
					XMFLOAT3 p; XMStoreFloat3(&p,XMVector3TransformCoord(XMLoadFloat3(&world),camera.GetViewProjection()));
					if (std::abs(p.x-first.x)*canvas.GetLogicalWidth() < 2 &&
						std::abs(p.y-first.y)*canvas.GetLogicalHeight() < 2)
						picked.push_back(i);
				}
			}
			if (wi::input::Down(wi::input::KEYBOARD_BUTTON_LSHIFT))
			{
				for (uint32_t i : picked)
				{
					auto found = std::find(state.vertices.begin(),state.vertices.end(),i);
					if (found == state.vertices.end()) state.vertices.push_back(i); else state.vertices.erase(found);
				}
				return;
			}
			// Keep a multi-selection only when the click grabs it; an ortho click selects its whole projected group.
			const bool grabsSelection = std::all_of(picked.begin(),picked.end(),[&](uint32_t i) {
				return std::find(state.vertices.begin(),state.vertices.end(),i) != state.vertices.end();
			});
			if (!grabsSelection) state.vertices = picked;
		}
		else
		{
			if (!selected) return;
			state.brush = selected->id; state.face = faceID;
			state.vertices.clear();
		}
		state.dragStart = *selected; state.dragCandidate = *selected;
		state.dragging = true; state.validDrag = false;
		state.pointerStart = mouse;
		state.anchor = {};
		if (state.mode == 2)
			for (uint32_t i : state.vertices) state.anchor = state.anchor + selected->vertices[i] * (1.0/state.vertices.size());
		else
			for (auto p : selected->vertices) state.anchor = state.anchor + p * (1.0/selected->vertices.size());
		// Grid snapping places the grabbed vertex, or the brush's minimum bounds corner, on grid positions.
		state.snapAnchor = selected->vertices.empty() ? Point{} : selected->vertices[0];
		for (auto p : selected->vertices)
			state.snapAnchor = {std::min(state.snapAnchor.x,p.x),std::min(state.snapAnchor.y,p.y),std::min(state.snapAnchor.z,p.z)};
		if (state.mode == 2 && !state.vertices.empty())
		{
			const auto clicked = std::find(state.vertices.begin(),state.vertices.end(),pickedVertex);
			state.snapAnchor = selected->vertices[clicked != state.vertices.end() ? *clicked : state.vertices.front()];
		}
	}
	if (!state.dragging) return;
	if (!wi::input::Down(wi::input::MOUSE_BUTTON_LEFT))
	{
		state.dragging = false;
		if (state.validDrag)
		{
			const Brush candidate = state.dragCandidate;
			Apply([&](CSGComponent& c,std::string& error) {
				auto* b = Selected(c,candidate.id);
				if (!b) { error = "Dragged brush no longer exists"; return false; }
				const Brush before = *b;
				*b = candidate;
				InheritSurfaces(c,before,*b);
				return true;
			});
		}
		return;
	}
	Brush candidate = state.dragStart;
	Point movement;
	auto worldAnchor = World(state.anchor,scene,state.root);
	float units = editor->activeView == 0 ? wi::math::Distance(camera.Eye,worldAnchor) * 1.2f / canvas.GetLogicalHeight() :
		camera.ortho_vertical_size / canvas.GetLogicalHeight();
	XMFLOAT3 worldMove;
	XMStoreFloat3(&worldMove,camera.GetRight() * ((mouse.x-state.pointerStart.x)*units) -
		camera.GetUp() * ((mouse.y-state.pointerStart.y)*units));
	// Drop float noise from camera axes so axis-aligned panes move exactly along their plane.
	const float moveLength = std::sqrt(worldMove.x*worldMove.x+worldMove.y*worldMove.y+worldMove.z*worldMove.z);
	for (float* axis : {&worldMove.x,&worldMove.y,&worldMove.z})
		if (std::abs(*axis) < moveLength*1e-5f) *axis = 0;
	const double snap = editor->translator.translate_snap;
	const bool snapping = editor->translator.IsGridSnapping();
	const auto world = [&](Point p) { return Double(World(p,scene,state.root)); };
	if (snapping)
	{
		// Snap the grabbed point to absolute world grid positions along the axes this drag moves.
		// Orthographic panes leave their depth axis alone.
		const XMFLOAT3 forward = camera.At;
		const bool ortho = editor->activeView != 0;
		const bool fixed[3] = {ortho && std::abs(forward.x) > 0.5f,ortho && std::abs(forward.y) > 0.5f,ortho && std::abs(forward.z) > 0.5f};
		float move[3] = {worldMove.x,worldMove.y,worldMove.z};
		CSGSnap::Move(world(state.snapAnchor),move,snap,fixed);
		worldMove = {move[0],move[1],move[2]};
	}
	movement = local(worldMove,true);
	std::string error;
	bool valid = false;
	if (state.mode == 3)
	{
		double distance = -(mouse.y-state.pointerStart.y)*units;
		if (snapping)
		{
			double scale = 0;
			Point a, n;
			for (const auto& f : state.dragStart.faces)
			{
				if (f.id != state.face || f.vertices.size() < 3) continue;
				a = world(state.dragStart.vertices[f.vertices[0]]);
				n = Cross(world(state.dragStart.vertices[f.vertices[1]])-a,world(state.dragStart.vertices[f.vertices[2]])-a);
				const Point lp = state.dragStart.vertices[f.vertices[0]];
				const Point ln = Cross(state.dragStart.vertices[f.vertices[1]]-lp,state.dragStart.vertices[f.vertices[2]]-lp);
				const double localLength = Length(ln);
				// World distance moved per unit of local face-drag distance.
				if (localLength > 0) scale = Length(world(lp + ln*(1/localLength)) - a);
				break;
			}
			distance = CSGSnap::FaceDistance(distance,snap,a,n,scale);
		}
		valid = DragFace(candidate,state.face,distance,error);
	}
	else
	{
		auto vertices = state.vertices;
		if (state.mode == 1) { vertices.resize(candidate.vertices.size()); std::iota(vertices.begin(),vertices.end(),0u); }
		valid = MoveVertices(candidate,vertices,movement,error);
	}
	if (valid)
	{
		state.dragCandidate = std::move(candidate);
		state.validDrag = Length(movement) > 1e-8 || std::abs(mouse.y-state.pointerStart.y) > 0.1f;
		state.status = "Drag preview; release commits, Esc cancels. Ctrl inverts grid snap.";
	}
	else
	{
		state.validDrag = false;
		state.status = error;
	}
}

void CSGWindow::DrawOverlay(wi::graphics::CommandList cmd) const
{
	if (!editor || editor->scenes.empty() || !IsVisible()) return;
	const auto& state = editor->GetCurrentEditorScene().csg;
	const auto& scene = editor->GetCurrentScene();
	const auto* source = scene.csgs.GetComponent(state.root);
	wi::image::SetCanvas(*editor);
	wi::font::SetCanvas(*editor);
	for (int view = 0; view < 4; ++view)
	{
		const auto& pane = editor->viewportLayout.Panes[view];
		if (pane.width == 0) continue;
		const auto& camera = view == 0 ? editor->GetCurrentEditorScene().camera : editor->GetCurrentEditorScene().orthographic[view-1].Camera;
		auto project = [&](Point p, XMFLOAT2& result) {
			auto world = World(p,scene,state.root);
			XMFLOAT3 ndc; XMStoreFloat3(&ndc,XMVector3TransformCoord(XMLoadFloat3(&world),camera.GetViewProjection()));
			result = {editor->PhysicalToLogical(pane.top_left_x+(ndc.x+1)*0.5f*pane.width),
				editor->PhysicalToLogical(pane.top_left_y+(1-ndc.y)*0.5f*pane.height)};
			return ndc.z >= 0 && ndc.z <= 1 && std::abs(ndc.x) <= 1 && std::abs(ndc.y) <= 1;
		};
		// Clip edges to the view volume so edges with an off-pane endpoint stay visible.
		auto segment = [&](Point p0, Point p1, XMFLOAT2& a, XMFLOAT2& b) {
			XMFLOAT4 c[2];
			for (int i = 0; i < 2; ++i)
			{
				auto world = World(i == 0 ? p0 : p1,scene,state.root);
				XMStoreFloat4(&c[i],XMVector4Transform(XMVectorSetW(XMLoadFloat3(&world),1),camera.GetViewProjection()));
			}
			float t0 = 0, t1 = 1;
			auto inside = [](const XMFLOAT4& v, int plane) {
				switch (plane) { case 0: return v.w+v.x; case 1: return v.w-v.x; case 2: return v.w+v.y; case 3: return v.w-v.y; case 4: return v.z; default: return v.w-v.z; }
			};
			for (int plane = 0; plane < 6; ++plane)
			{
				const float f0 = inside(c[0],plane), f1 = inside(c[1],plane);
				if (f0 < 0 && f1 < 0) return false;
				if (f0 < 0) t0 = std::max(t0,f0/(f0-f1));
				else if (f1 < 0) t1 = std::min(t1,f0/(f0-f1));
			}
			if (t0 > t1) return false;
			auto screen = [&](float t, XMFLOAT2& result) {
				const float x = c[0].x+(c[1].x-c[0].x)*t, y = c[0].y+(c[1].y-c[0].y)*t, w = c[0].w+(c[1].w-c[0].w)*t;
				result = {editor->PhysicalToLogical(pane.top_left_x+(x/w+1)*0.5f*pane.width),
					editor->PhysicalToLogical(pane.top_left_y+(1-y/w)*0.5f*pane.height)};
			};
			screen(t0,a); screen(t1,b);
			return true;
		};
		auto draw = [&](const Brush& brush, bool preview) {
			XMFLOAT4 color = preview ? XMFLOAT4(0.2f,1,0.3f,1) : brush.id == state.brush ?
				XMFLOAT4(1,0.7f,0.1f,1) : brush.operation == Operation::Subtract ? XMFLOAT4(1,0.3f,0.3f,0.6f) : XMFLOAT4(0.3f,0.6f,1,0.5f);
			for (const auto& face : brush.faces)
				for (size_t i = 0; i < face.vertices.size(); ++i)
				{
					XMFLOAT2 a,b;
					if (!segment(brush.vertices[face.vertices[i]],brush.vertices[face.vertices[(i+1)%face.vertices.size()]],a,b)) continue;
					wi::image::Params line;
					line.pos = {a.x,a.y,0}; line.siz = {wi::math::Distance(a,b),1};
					line.rotation = std::atan2(b.y-a.y,b.x-a.x);
					line.color = state.mode == 3 && brush.id == state.brush && face.id == state.face ? XMFLOAT4(0,1,1,1) : color;
					wi::image::Draw(nullptr,line,cmd);
				}
			if (state.mode == 2 && brush.id == state.brush)
				for (uint32_t i = 0; i < brush.vertices.size(); ++i)
				{
					XMFLOAT2 p;
					if (!project(brush.vertices[i],p)) continue;
					wi::font::Params text;
					text.position = {p.x,p.y,0}; text.size = 16;
					text.h_align = wi::font::WIFALIGN_CENTER; text.v_align = wi::font::WIFALIGN_CENTER;
					text.color = std::find(state.vertices.begin(),state.vertices.end(),i) != state.vertices.end() ? wi::Color::Yellow() : wi::Color::White();
					wi::font::Draw("+",text,cmd);
				}
		};
		if (source)
			for (const auto& brush : source->brushes)
				draw(state.dragging && brush.id == state.brush ? state.dragCandidate : brush,false);
		if (state.hasPreview) draw(state.preview,true);
	}
}
