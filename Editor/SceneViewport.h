#pragma once
#include "wiScene.h"
#include "wiCanvas.h"
#include "wiEventHandler.h"
#include <algorithm>
#include <array>
#include <cmath>

void RequireViewportResource(bool success, const char* message);

struct OrthographicView
{
	wi::scene::CameraComponent Camera;
	XMFLOAT3 Center = {};
	XMFLOAT3 Forward = {0, 0, 1};
	XMFLOAT3 Up = {0, 1, 0};
	float Size = 40;
	float Depth = 1000;

	void Update(float width, float height)
	{
		Camera.width = width;
		Camera.height = height;
		Camera.zNearP = 0.1f;
		Camera.zFarP = Depth * 2;
		Camera.SetOrtho(true);
		Camera.ortho_vertical_size = Size;
		Camera.At = Forward;
		Camera.Up = Up;
		XMStoreFloat3(&Camera.Eye, XMLoadFloat3(&Center) - XMLoadFloat3(&Forward) * Depth);
		Camera.UpdateCamera();
	}

	void Pan(float x, float y, float height)
	{
		XMStoreFloat3(&Center, XMLoadFloat3(&Center) +
			(-Camera.GetRight() * x + Camera.GetUp() * y) * (Size / std::max(1.0f, height)));
	}

	void Zoom(float wheel)
	{
		Size = std::clamp(Size * std::exp(-wheel * 0.15f), 0.01f, 200000.0f);
	}
};

struct SceneViewportLayout
{
	std::array<wi::graphics::Viewport, 4> Panes;

	void Update(const wi::graphics::Viewport& area, bool quad, int maximized)
	{
		Panes = {};
		if (!quad || maximized >= 0)
		{
			Panes[quad ? maximized : 0] = area;
			return;
		}
		const float halfWidth = std::floor(area.width * 0.5f);
		const float halfHeight = std::floor(area.height * 0.5f);
		for (int i = 0; i < 4; ++i)
		{
			auto& pane = Panes[i];
			pane = area;
			const bool right = i == 2 || i == 3;
			const bool bottom = i == 0 || i == 3;
			pane.top_left_x += right ? halfWidth : 0;
			pane.top_left_y += bottom ? halfHeight : 0;
			pane.width = right ? area.width - halfWidth : halfWidth;
			pane.height = bottom ? area.height - halfHeight : halfHeight;
		}
	}

	int Hit(float x, float y) const
	{
		for (int i = 0; i < 4; ++i)
		{
			const auto& pane = Panes[i];
			if (pane.width > 0 && pane.height > 0 && x >= pane.top_left_x && y >= pane.top_left_y &&
				x < pane.top_left_x + pane.width && y < pane.top_left_y + pane.height)
				return i;
		}
		return -1;
	}
};

class OrthographicRenderer
{
	struct MeshBuffers
	{
		wi::graphics::GPUBuffer Vertices;
		wi::graphics::GPUBuffer Indices;
		wi::allocator::shared_ptr<void> Source;
		size_t VertexCount = 0;
		size_t IndexCount = 0;
	};
	wi::unordered_map<wi::ecs::Entity, MeshBuffers> Meshes;
	wi::graphics::PipelineState Pipeline;
	wi::graphics::PipelineState GridPipeline;
	wi::eventhandler::Handle ReloadShaders;

public:
	void Update(const wi::scene::Scene& scene);
	void Draw(const wi::scene::Scene& scene, const wi::scene::CameraComponent& camera,
		const wi::unordered_set<wi::ecs::Entity>& selected, float gridSize, wi::graphics::CommandList cmd) const;
};
