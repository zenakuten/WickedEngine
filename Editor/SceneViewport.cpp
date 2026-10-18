#include "stdafx.h"
#include "SceneViewport.h"
#include "shaders/ShaderInterop_Renderer.h"

using namespace wi::graphics;

namespace
{
struct WireVertex
{
	XMFLOAT4 Position;
	XMFLOAT4 Color = {1, 1, 1, 1};
};
}

void RequireViewportResource(bool success, const char* message)
{
	if (!success)
	{
		wi::backlog::post(message, wi::backlog::LogLevel::Error);
		std::abort();
	}
}

void OrthographicRenderer::Update(const wi::scene::Scene& scene)
{
	auto* device = wi::graphics::GetDevice();
	if (!ReloadShaders.IsValid())
	{
		ReloadShaders = wi::eventhandler::Subscribe(wi::eventhandler::EVENT_RELOAD_SHADERS, [this](uint64_t) {
			Pipeline = {};
			GridPipeline = {};
		});
	}
	if (!Pipeline.IsValid())
	{
		PipelineStateDesc desc;
		desc.vs = wi::renderer::GetShader(wi::enums::VSTYPE_VERTEXCOLOR);
		desc.ps = wi::renderer::GetShader(wi::enums::PSTYPE_VERTEXCOLOR);
		desc.il = wi::renderer::GetInputLayout(wi::enums::ILTYPE_VERTEXCOLOR);
		desc.rs = wi::renderer::GetRasterizerState(wi::enums::RSTYPE_WIRE_DOUBLESIDED);
		desc.dss = wi::renderer::GetDepthStencilState(wi::enums::DSSTYPE_DEPTHDISABLED);
		desc.bs = wi::renderer::GetBlendState(wi::enums::BSTYPE_OPAQUE);
		desc.pt = PrimitiveTopology::TRIANGLELIST;
		RequireViewportResource(device->CreatePipelineState(&desc, &Pipeline), "Orthographic viewport pipeline creation failed");
		desc.pt = PrimitiveTopology::LINELIST;
		RequireViewportResource(device->CreatePipelineState(&desc, &GridPipeline), "Orthographic viewport grid pipeline creation failed");
	}
	for (auto it = Meshes.begin(); it != Meshes.end();)
	{
		if (!scene.meshes.Contains(it->first))
			it = Meshes.erase(it);
		else
			++it;
	}
	for (size_t i = 0; i < scene.meshes.GetCount(); ++i)
	{
		const auto& mesh = scene.meshes[i];
		if (mesh.vertex_positions.empty() || mesh.indices.empty())
			continue;
		auto& buffers = Meshes[scene.meshes.GetEntity(i)];
		if (buffers.Source == mesh.generalBuffer.internal_state && buffers.VertexCount == mesh.vertex_positions.size() &&
			buffers.IndexCount == mesh.indices.size() && buffers.Vertices.IsValid())
			continue;
		wi::vector<WireVertex> vertices;
		vertices.reserve(mesh.vertex_positions.size());
		for (const auto& position : mesh.vertex_positions)
			vertices.push_back({XMFLOAT4(position.x, position.y, position.z, 1)});
		GPUBufferDesc desc;
		desc.size = vertices.size() * sizeof(WireVertex);
		desc.bind_flags = BindFlag::VERTEX_BUFFER;
		RequireViewportResource(device->CreateBuffer(&desc, vertices.data(), &buffers.Vertices), "Orthographic viewport vertex buffer creation failed");
		desc.size = mesh.indices.size() * sizeof(uint32_t);
		desc.bind_flags = BindFlag::INDEX_BUFFER;
		RequireViewportResource(device->CreateBuffer(&desc, mesh.indices.data(), &buffers.Indices), "Orthographic viewport index buffer creation failed");
		buffers.Source = mesh.generalBuffer.internal_state;
		buffers.VertexCount = mesh.vertex_positions.size();
		buffers.IndexCount = mesh.indices.size();
	}
}

void OrthographicRenderer::Draw(const wi::scene::Scene& scene, const wi::scene::CameraComponent& camera,
	const wi::unordered_set<wi::ecs::Entity>& selected, float gridSize, CommandList cmd) const
{
	auto* device = wi::graphics::GetDevice();
	const XMVECTOR right = camera.GetRight();
	const XMVECTOR up = camera.GetUp();
	const XMVECTOR center = camera.GetEye();
	const float centerX = XMVectorGetX(XMVector3Dot(center, right));
	const float centerY = XMVectorGetX(XMVector3Dot(center, up));
	const float spanY = camera.ortho_vertical_size * 0.5f;
	const float spanX = spanY * camera.width / camera.height;
	// Draw the snapping grid; when zoomed out, double the spacing until lines are at least 8 pixels apart.
	const float pixelsPerUnit = float(camera.height) / std::max(camera.ortho_vertical_size, 1e-6f);
	float spacing = gridSize > 0 && std::isfinite(gridSize) ? gridSize : 1.0f;
	for (int i = 0; i < 32 && spacing * pixelsPerUnit < 8; ++i)
		spacing *= 2;
	const float major = spacing * 8;
	wi::vector<WireVertex> grid;
	auto line = [&](float x1, float y1, float x2, float y2, bool axis) {
		const float p = x1 == x2 ? x1 : y1;
		const bool isMajor = std::abs(p - std::round(p / major) * major) < spacing * 0.25f;
		const XMFLOAT4 color = axis ? XMFLOAT4(0.3f, 0.35f, 0.4f, 1) : isMajor ? XMFLOAT4(0.17f, 0.21f, 0.25f, 1) : XMFLOAT4(0.11f, 0.14f, 0.17f, 1);
		WireVertex vertex;
		vertex.Color = color;
		XMStoreFloat4(&vertex.Position, XMVectorSetW(right * x1 + up * y1, 1));
		grid.push_back(vertex);
		XMStoreFloat4(&vertex.Position, XMVectorSetW(right * x2 + up * y2, 1));
		grid.push_back(vertex);
	};
	for (int axis = 0; axis < 2; ++axis)
	{
		const float mid = axis == 0 ? centerX : centerY;
		const float span = axis == 0 ? spanX : spanY;
		const float first = std::floor((mid - span) / spacing) * spacing;
		const int count = int(std::ceil(span * 2 / spacing)) + 2;
		for (int step = 0; step < count; ++step)
		{
			const float p = first + step * spacing;
			if (axis == 0)
				line(p, centerY - spanY, p, centerY + spanY, std::abs(p) < spacing * 0.5f);
			else
				line(centerX - spanX, p, centerX + spanX, p, std::abs(p) < spacing * 0.5f);
		}
	}
	if (!grid.empty())
	{
		auto allocation = device->AllocateGPU(grid.size() * sizeof(WireVertex), cmd);
		std::memcpy(allocation.data, grid.data(), grid.size() * sizeof(WireVertex));
		const GPUBuffer* vb[] = {&allocation.buffer};
		const uint32_t strides[] = {sizeof(WireVertex)};
		const uint64_t offsets[] = {allocation.offset};
		device->BindPipelineState(&GridPipeline, cmd);
		device->BindVertexBuffers(vb, 0, 1, strides, offsets, cmd);
		MiscCB constants = {};
		XMStoreFloat4x4(&constants.g_xTransform, camera.GetViewProjection());
		constants.g_xColor = {1, 1, 1, 1};
		device->BindDynamicConstantBuffer(constants, CBSLOT_RENDERER_MISC, cmd);
		device->Draw(uint32_t(grid.size()), 0, cmd);
	}
	device->BindPipelineState(&Pipeline, cmd);
	for (size_t i = 0; i < scene.objects.GetCount(); ++i)
	{
		const auto& object = scene.objects[i];
		const auto entity = scene.objects.GetEntity(i);
		const auto found = Meshes.find(object.meshID);
		const auto* transform = scene.transforms.GetComponent(entity);
		const auto* layer = scene.layers.GetComponent(entity);
		if (!object.IsRenderable() || found == Meshes.end() || (layer && layer->GetLayerMask() == 0))
			continue;
		if (i < scene.aabb_objects.size() && !camera.frustum.CheckBox(scene.aabb_objects[i]))
			continue;
		const auto& buffers = found->second;
		const auto* mesh = scene.meshes.GetComponent(object.meshID);
		if (!mesh)
			continue;
		const GPUBuffer* vb[] = {&buffers.Vertices};
		const uint32_t strides[] = {sizeof(WireVertex)};
		device->BindVertexBuffers(vb, 0, 1, strides, nullptr, cmd);
		device->BindIndexBuffer(&buffers.Indices, IndexBufferFormat::UINT32, 0, cmd);
		MiscCB constants = {};
		XMStoreFloat4x4(&constants.g_xTransform, (transform ? XMLoadFloat4x4(&transform->world) : XMMatrixIdentity()) * camera.GetViewProjection());
		constants.g_xColor = selected.count(entity) ? XMFLOAT4(1, 0.7f, 0.1f, 1) : XMFLOAT4(0.5f, 0.7f, 0.8f, 1);
		device->BindDynamicConstantBuffer(constants, CBSLOT_RENDERER_MISC, cmd);
		uint32_t firstSubset = 0, lastSubset = 0;
		mesh->GetLODSubsetRange(0, firstSubset, lastSubset);
		for (uint32_t subsetIndex = firstSubset; subsetIndex < lastSubset; ++subsetIndex)
		{
			const auto& subset = mesh->subsets[subsetIndex];
			device->DrawIndexed(subset.indexCount, subset.indexOffset, 0, cmd);
		}
	}
}
