#include "stdafx.h"
#include "Editor.h"

using namespace wi::graphics;
using namespace wi::scene;

CameraComponent& EditorComponent::GetInteractionCamera()
{
	return QuadViewsEnabled() && activeView != 0 ?
		GetCurrentEditorScene().orthographic[activeView - 1].Camera : GetCurrentEditorScene().camera;
}

const wi::Canvas& EditorComponent::GetInteractionCanvas() const
{
	return activeView == 0 ? static_cast<const wi::Canvas&>(*renderPath) : viewportCanvases[activeView];
}

int EditorComponent::GetSceneViewportAtPointer() const
{
	const auto pointer = wi::input::GetPointer();
	return viewportLayout.Hit(pointer.x * GetDPIScaling(), pointer.y * GetDPIScaling());
}

void EditorComponent::ResizeSceneViewports(const Viewport& area)
{
	if (!QuadViewsEnabled())
		activeView = 0;
	viewportLayout.Update(area, QuadViewsEnabled(), maximizedView);
	viewport3D = viewportLayout.Panes[0];
	if (viewport3D.width == 0)
	{
		viewport3D.width = 64;
		viewport3D.height = 64;
	}
	if (scenes.empty())
		return;
	auto& editorScene = GetCurrentEditorScene();
	if (!editorScene.orthographicInitialized)
	{
		auto& views = editorScene.orthographic;
		views[0].Forward = {0, -1, 0};
		views[0].Up = {0, 0, 1};
		views[1].Forward = {0, 0, 1};
		views[2].Forward = {-1, 0, 0};
		editorScene.orthographicInitialized = true;
	}
	for (int i = 0; i < 4; ++i)
	{
		const auto& pane = viewportLayout.Panes[i];
		auto& canvas = viewportCanvases[i];
		canvas.init(std::max(1u, uint32_t(pane.width)), std::max(1u, uint32_t(pane.height)), dpi);
		canvas.scaling = scaling;
		if (i == 0 || !QuadViewsEnabled() || pane.width == 0)
			continue;
		auto& view = editorScene.orthographic[i - 1];
		const auto& bounds = editorScene.scene.bounds;
		if (bounds.IsValid())
			view.Depth = std::max(1000.0f, wi::math::Distance(bounds.getCenter(), view.Center) + bounds.getRadius() * 2);
		view.Update(pane.width, pane.height);
		auto& target = orthoTargets[i - 1];
		if (!target.IsValid() || target.desc.width != canvas.width || target.desc.height != canvas.height)
		{
			TextureDesc desc;
			desc.width = canvas.width;
			desc.height = canvas.height;
			desc.format = Format::R8G8B8A8_UNORM;
			desc.bind_flags = BindFlag::RENDER_TARGET | BindFlag::SHADER_RESOURCE;
			desc.layout = ResourceState::SHADER_RESOURCE;
			desc.clear.color[0] = 0.025f;
			desc.clear.color[1] = 0.03f;
			desc.clear.color[2] = 0.04f;
			desc.clear.color[3] = 1;
			RequireViewportResource(GetDevice()->CreateTexture(&desc, nullptr, &target), "Orthographic viewport render target creation failed");
			GetDevice()->SetName(&target, "Editor orthographic viewport");
		}
	}
}

void EditorComponent::UpdateSceneViewports()
{
	const auto pointer = wi::input::GetPointer();
	const int hoveredView = GetSceneViewportAtPointer();
	const bool mouseDown = wi::input::Down(wi::input::MOUSE_BUTTON_LEFT) ||
		wi::input::Down(wi::input::MOUSE_BUTTON_RIGHT) || wi::input::Down(wi::input::MOUSE_BUTTON_MIDDLE);
	if (!mouseDown)
		viewportMouseCaptured = false;
	if (!wi::backlog::isActive() && !GetGUI().HasFocus() && !translator.IsDragging() && !viewportMouseCaptured &&
		hoveredView >= 0 && (wi::input::Press(wi::input::MOUSE_BUTTON_LEFT) ||
			wi::input::Press(wi::input::MOUSE_BUTTON_RIGHT) || wi::input::Press(wi::input::MOUSE_BUTTON_MIDDLE) || pointer.z != 0))
	{
		activeView = hoveredView;
		viewportMouseCaptured = mouseDown;
	}
	if (!QuadViewsEnabled() || activeView == 0 || wi::backlog::isActive() || (GetGUI().HasFocus() && !viewportMouseCaptured))
		return;
	auto& view = GetCurrentEditorScene().orthographic[activeView - 1];
	const auto& canvas = viewportCanvases[activeView];
	if (viewportMouseCaptured && (wi::input::Down(wi::input::MOUSE_BUTTON_MIDDLE) || wi::input::Down(wi::input::MOUSE_BUTTON_RIGHT)))
	{
		const auto delta = wi::input::GetMouseState().delta_position;
		view.Pan(delta.x, delta.y, float(canvas.GetPhysicalHeight()));
	}
	if (hoveredView == activeView)
		view.Zoom(pointer.z);
	view.Update(float(canvas.width), float(canvas.height));
}

void EditorComponent::RenderOrthographicViews() const
{
	auto* device = GetDevice();
	auto cmd = device->BeginCommandList();
	const Scene& scene = GetCurrentScene();
	const auto& views = GetCurrentEditorScene().orthographic;
	wi::unordered_set<wi::ecs::Entity> selected;
	for (const auto& pick : translator.selected)
		selected.insert(pick.entity);
	for (int i = 1; i < 4; ++i)
	{
		if (viewportLayout.Panes[i].width == 0)
			continue;
		const auto& canvas = viewportCanvases[i];
		const auto& camera = views[i - 1].Camera;
		auto image = RenderPassImage::RenderTarget(&orthoTargets[i - 1], RenderPassImage::LoadOp::CLEAR);
		device->RenderPassBegin(&image, 1, cmd);
		Viewport vp;
		vp.width = float(canvas.width);
		vp.height = float(canvas.height);
		device->BindViewports(1, &vp, cmd);
		wi::renderer::BindCommonResources(cmd);
		wi::renderer::BindCameraCB(camera, camera, camera, cmd);
		orthoRenderer.Draw(scene, camera, selected, translator.translate_snap, cmd);
		wi::font::SetCanvas(canvas);
		auto marker = [&](wi::ecs::Entity entity, const char* text) {
			const auto* transform = scene.transforms.GetComponent(entity);
			const auto* layer = scene.layers.GetComponent(entity);
			if (!transform || (layer && layer->GetLayerMask() == 0))
				return;
			XMVECTOR projected = XMVector3TransformCoord(transform->GetPositionV(), camera.GetViewProjection());
			XMFLOAT3 ndc;
			XMStoreFloat3(&ndc, projected);
			if (std::abs(ndc.x) > 1 || std::abs(ndc.y) > 1 || ndc.z < 0 || ndc.z > 1)
				return;
			wi::font::Params params;
			params.position = {(ndc.x + 1) * 0.5f * canvas.GetLogicalWidth(), (1 - ndc.y) * 0.5f * canvas.GetLogicalHeight(), 0};
			params.size = 18;
			params.h_align = wi::font::WIFALIGN_CENTER;
			params.v_align = wi::font::WIFALIGN_CENTER;
			params.color = selected.count(entity) ? wi::Color(255, 180, 30) : wi::Color(190, 210, 225);
			wi::font::Draw(text, params, cmd);
		};
		using Filter = ComponentsWindow::Filter;
		auto markers = [&](const auto& components, Filter filter, const char* icon) {
			if (has_flag(componentsWnd.filter, filter))
				for (size_t index = 0; index < components.GetCount(); ++index)
					marker(components.GetEntity(index), icon);
		};
		markers(scene.lights, Filter::Light, ICON_POINTLIGHT);
		markers(scene.cameras, Filter::Camera, ICON_CAMERA);
		markers(scene.probes, Filter::EnvironmentProbe, ICON_ENVIRONMENTPROBE);
		markers(scene.decals, Filter::Decal, ICON_DECAL);
		markers(scene.emitters, Filter::Emitter, ICON_EMITTER);
		markers(scene.forces, Filter::Force, ICON_FORCE);
		markers(scene.sounds, Filter::Sound, ICON_SOUND);
		markers(scene.videos, Filter::Video, ICON_VIDEO);
		markers(scene.hairs, Filter::Hairparticle, ICON_HAIR);
		markers(scene.voxel_grids, Filter::VoxelGrid, ICON_VOXELGRID);
		if (has_flag(componentsWnd.filter, Filter::Metadata))
			for (size_t index = 0; index < scene.metadatas.GetCount(); ++index)
				if (!scene.metadatas[index].HideIcon)
					marker(scene.metadatas.GetEntity(index), ICON_METADATA);
		translator.Draw(camera, currentMouse, cmd);
		device->RenderPassEnd(cmd);
	}
}

void EditorComponent::ComposeSceneViewports(CommandList cmd) const
{
	auto* device = GetDevice();
	for (int i = 1; i < 4; ++i)
	{
		const auto& pane = viewportLayout.Panes[i];
		if (pane.width == 0)
			continue;
		device->BindViewports(1, &pane, cmd);
		wi::image::Params params;
		params.enableFullScreen();
		params.blendFlag = wi::enums::BLENDMODE_OPAQUE;
		wi::image::Draw(&orthoTargets[i - 1], params, cmd);
	}
	Viewport full;
	full.width = float(GetPhysicalWidth());
	full.height = float(GetPhysicalHeight());
	device->BindViewports(1, &full, cmd);
	wi::image::SetCanvas(*this);
	wi::font::SetCanvas(*this);
	const char* labels[] = {"Perspective", "Top (X/Z)", "Front (X/Y)", "Side (Z/Y)"};
	for (int i = 0; i < 4; ++i)
	{
		const auto& pane = viewportLayout.Panes[i];
		if (pane.width == 0)
			continue;
		wi::image::Params border;
		border.pos = {PhysicalToLogical(pane.top_left_x), PhysicalToLogical(pane.top_left_y), 0};
		border.siz = {PhysicalToLogical(pane.width), PhysicalToLogical(pane.height)};
		border.color = i == activeView ? XMFLOAT4(1, 0.6f, 0.1f, 1) : XMFLOAT4(0.3f, 0.35f, 0.4f, 1);
		const auto size = border.siz;
		border.siz.y = 1;
		wi::image::Draw(nullptr, border, cmd);
		border.pos.y += size.y - 1;
		wi::image::Draw(nullptr, border, cmd);
		border.pos.y -= size.y - 1;
		border.siz = {1, size.y};
		wi::image::Draw(nullptr, border, cmd);
		border.pos.x += size.x - 1;
		wi::image::Draw(nullptr, border, cmd);
		border.pos.x -= size.x - 1;
		wi::font::Params label;
		label.position = {border.pos.x + (i == 1 ? 55 : 8), border.pos.y + 6, 0};
		label.size = 14;
		label.shadowColor = wi::Color::Black();
		wi::font::Draw(labels[i], label, cmd);
	}
}
