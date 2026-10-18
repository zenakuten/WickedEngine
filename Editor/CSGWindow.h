#pragma once
#include "wiCSGScene.h"
#include <functional>

class EditorComponent;

struct CSGEditState
{
	wi::ecs::Entity root = wi::ecs::INVALID_ENTITY;
	uint64_t brush = 0, face = 0;
	int mode = 0;
	std::vector<uint32_t> vertices;
	wi::csg::Brush preview;
	bool hasPreview = false;
	bool dragging = false, validDrag = false;
	wi::csg::Brush dragStart, dragCandidate;
	wi::csg::Point anchor, snapAnchor; // snapAnchor: the source point placed on the grid while dragging
	XMFLOAT2 pointerStart = {};
	std::string status;
};

class CSGWindow : public wi::gui::Window
{
public:
	EditorComponent* editor = nullptr;
	wi::gui::ComboBox roots, brushes, mode, operation, rotationAxis;
	wi::gui::TextInputField width, height, depth, radius, sides, posX, posY, posZ;
	wi::gui::TextInputField deltaX, deltaY, deltaZ, scale, angle;
	wi::gui::Button cube, cone, commit, cancel, remove, up, down, enabled, rebuild;
	wi::gui::Button move, resize, rotate;
	wi::gui::Label status;
	wi::csg::Point dimensions = {8,8,8}, position = {0,4,0}, delta;
	double coneRadius = 4, coneSides = 16, uniformScale = 1, rotationDegrees = 0;
	void Create(EditorComponent* editor);
	void ResizeLayout() override;
	void UpdateData();
	void UpdateInput();
	void DrawOverlay(wi::graphics::CommandList cmd) const;
	bool Editing() const;
	void RestoreHistory(wi::Archive& archive, bool undo);
	void CancelDrag();

private:
	void Report(const std::string& message, bool error = false);
	void Preview(bool isCone);
	bool Apply(const std::function<bool(wi::scene::CSGComponent&, std::string&)>& edit, bool evaluate = false);
	void ChangeOrder(int direction);
	wi::Archive Snapshot(wi::ecs::Entity root);
};
