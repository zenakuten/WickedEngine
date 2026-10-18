#include "WickedEngine.h"
#include "../../Editor/SceneViewport.h"
#include "../../Editor/Translator.h"
#include <iostream>
#include <limits>

namespace
{
int failures = 0;
void Check(bool condition, const char* message)
{
	if (!condition)
	{
		std::cerr << message << '\n';
		++failures;
	}
}
bool Near(float a, float b)
{
	return std::abs(a - b) < 0.001f;
}
}

int main()
{
	SceneViewportLayout layout;
	wi::graphics::Viewport area;
	area.top_left_x = 213;
	area.top_left_y = 67;
	area.width = 1001;
	area.height = 701;
	layout.Update(area, true, -1);
	Check(layout.Hit(214, 68) == 1, "Top must be upper left");
	Check(layout.Hit(714, 68) == 2, "Front must be upper right");
	Check(layout.Hit(214, 418) == 0, "Perspective must be lower left");
	Check(layout.Hit(714, 418) == 3, "Side must be lower right");
	Check(layout.Hit(1214, 200) == -1 && layout.Hit(200, 200) == -1, "Outside panes must not capture input");
	for (int y = 67; y < 768; ++y)
		for (int x = 213; x < 1214; ++x)
			Check(layout.Hit(float(x), float(y)) >= 0, "Odd-sized layout must not leave gaps");
	for (int pane = 0; pane < 4; ++pane)
	{
		layout.Update(area, true, pane);
		Check(layout.Hit(214, 68) == pane && layout.Hit(1213, 767) == pane, "Maximize must cover available area");
	}
	layout.Update(area, false, -1);
	Check(layout.Hit(214, 68) == 0 && layout.Hit(1213, 767) == 0, "Single-view fallback must cover area");

	OrthographicView top;
	top.Forward = {0, -1, 0};
	top.Up = {0, 0, 1};
	top.Center = {7, 3, -4};
	top.Update(1000, 500);
	auto project = [&](XMFLOAT3 point) {
		XMFLOAT3 result;
		XMStoreFloat3(&result, XMVector3TransformCoord(XMLoadFloat3(&point), top.Camera.GetViewProjection()));
		return result;
	};
	auto center = project(top.Center);
	Check(Near(center.x, 0) && Near(center.y, 0), "Top camera must center on navigation target");
	auto x = project({8, 3, -4});
	auto z = project({7, 3, -3});
	Check(x.x > 0 && Near(x.y, 0) && Near(z.x, 0) && z.y > 0, "Top axes must map +X right and +Z up");
	const auto originalForward = top.Forward;
	top.Pan(100, 50, 500);
	Check(Near(top.Center.x, -1) && Near(top.Center.y, 3) && Near(top.Center.z, 0), "Pan must move in fixed camera plane");
	top.Zoom(1);
	Check(top.Size < 40 && top.Forward.x == originalForward.x && top.Forward.y == originalForward.y,
		"Zoom must preserve direction");
	for (int i = 0; i < 1000; ++i)
		top.Zoom(100);
	Check(Near(top.Size, 0.01f), "Zoom-in must be bounded");
	for (int i = 0; i < 1000; ++i)
		top.Zoom(-100);
	Check(Near(top.Size, 200000), "Zoom-out must be bounded");

	const XMFLOAT3 directions[] = {{0, -1, 0}, {0, 0, 1}, {-1, 0, 0}};
	for (int i = 0; i < 3; ++i)
	{
		OrthographicView view;
		view.Forward = directions[i];
		if (i == 0)
			view.Up = {0, 0, 1};
		view.Update(1000, 500);
		wi::Canvas canvas;
		canvas.init(1000, 500, 192);
		const auto ray = wi::renderer::GetPickRay(250, 125, canvas, view.Camera);
		Check(Near(ray.direction.x, directions[i].x) && Near(ray.direction.y, directions[i].y) &&
			Near(ray.direction.z, directions[i].z), "DPI-aware center ray must follow pane axis");
		Check(std::abs(XMVectorGetX(XMVector3Dot(XMLoadFloat3(&ray.origin), view.Camera.GetRight()))) < 0.1f &&
			std::abs(XMVectorGetX(XMVector3Dot(XMLoadFloat3(&ray.origin), view.Camera.GetUp()))) < 0.1f,
			"DPI-aware center ray must originate at pane center");
		Check(view.Camera.IsOrtho() && Near(view.Camera.ortho_vertical_size, 40), "Pane must remain orthographic");
	}
	wi::input::Initialize();
	{
		Translator translator;
		Check(translator.grid_snap && translator.translate_snap == 1 && translator.IsGridSnapping(),
			"Grid snapping must default on with a unit grid");
		translator.grid_snap = false;
		Check(!translator.IsGridSnapping(), "Disabled grid snapping must not snap without Ctrl");
		translator.grid_snap = true;
		for (float invalid : {0.0f, -1.0f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()})
		{
			translator.translate_snap = invalid;
			Check(!translator.IsGridSnapping(), "Non-positive or non-finite grid sizes must disable snapping");
		}
	}
	// Grid settings: snapping off, unit grid, fractional grid and an invalid grid that must not snap.
	struct SnapCase { bool enabled; float size; float near; float far; };
	const SnapCase snapCases[] = {
		{false, 1, 5.72f, 57.72f},
		{true, 1, 6, 58},
		{true, 0.25f, 5.75f, 57.75f},
		{true, 0, 5.72f, 57.72f},
	};
	for (const auto& snapCase : snapCases)
	for (int pane = 0; pane < 3; ++pane)
		for (float dpi : {96.0f, 192.0f})
		{
			wi::scene::Scene scene;
			auto entity = scene.Entity_CreateTransform("Viewport gizmo probe");
			auto* transform = scene.transforms.GetComponent(entity);
			const XMFLOAT3 initial = pane == 0 ? XMFLOAT3(3, 3, 0) : XMFLOAT3(3, 0, 3);
			transform->Translate(initial);
			transform->UpdateTransform();
			Translator translator;
			translator.scene = &scene;
			translator.grid_snap = snapCase.enabled;
			translator.translate_snap = snapCase.size;
			wi::scene::PickResult pick;
			pick.entity = entity;
			translator.selected.push_back(pick);
			OrthographicView view;
			view.Forward = directions[pane];
			if (pane == 0)
				view.Up = {0, 0, 1};
			view.Update(1000, 500);
			wi::Canvas canvas;
			canvas.init(1000, 500, dpi);
			auto frame = [&](float x, bool pressed, bool enabled = true) {
				const float scale = canvas.GetDPIScaling();
				wi::input::Pen pen;
				pen.position = {x / scale, 250 / scale};
				pen.pressure = pressed ? 1 : 0;
				wi::input::SetPen(pen);
				wi::input::Update(nullptr, canvas);
				translator.Update(view.Camera, {pen.position.x, pen.position.y, 0, 0}, canvas, enabled);
				wi::input::ClearForNextFrame();
			};
			auto checkPosition = [&](float expected) {
				const auto position = transform->GetPosition();
				const bool matches = Near(position.y, initial.y) &&
					(pane == 2 ? Near(position.x, initial.x) && Near(position.z, expected) :
						Near(position.x, expected) && Near(position.z, initial.z));
				if (!matches)
					std::cerr << "Snap " << snapCase.enabled << " grid " << snapCase.size << " pane " << pane << " DPI " << dpi << " position " << position.x << ','
						<< position.y << ',' << position.z << " expected axis value " << expected << '\n';
				Check(matches, "Orthographic gizmo drag must move only along the selected world axis");
			};
			frame(0, false, false);
			frame(516, false);
			if (translator.state != (pane == 2 ? Translator::TRANSLATOR_Z : Translator::TRANSLATOR_X))
				std::cerr << "Pane " << pane << " DPI " << dpi << " hover state " << translator.state << '\n';
			Check(translator.state == (pane == 2 ? Translator::TRANSLATOR_Z : Translator::TRANSLATOR_X),
				"Each orthographic view must pick its mirrored horizontal-axis gizmo");
			frame(516, false, false);
			Check(translator.state == Translator::TRANSLATOR_IDLE, "Inactive pane must not retain hover input");
			frame(516, true);
			Check(translator.IsDragging() && translator.IsDragStarted(), "Orthographic view must start axis drag");
			frame(550, true);
			checkPosition(snapCase.near);
			frame(1200, true, false);
			checkPosition(snapCase.far);
			Check(translator.IsDragging(), "Captured drag must continue outside its original pane");
			frame(1200, false, false);
			Check(!translator.IsDragging() && translator.IsDragEnded(), "Captured release must end drag");
		}
	if (failures)
		return 1;
	std::cout << "Editor viewport layout, fixed axes, pan/zoom, maximize, DPI picking and captured gizmo drags with grid snapping passed\n";
}
