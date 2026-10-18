#pragma once
#include "wiScene.h"

namespace wi::csg
{
	// Replaces only this root's generated objects after successful evaluation.
	// Material entities remain alive even when their source face disappears.
	bool Rebuild(wi::scene::Scene& scene, wi::ecs::Entity root, std::string& error);
	bool EvaluateScene(const wi::scene::Scene& scene, wi::ecs::Entity root,
		std::vector<Triangle>& triangles, std::string& error);
}
