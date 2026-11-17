#pragma once

#include <cstddef>

namespace pangu::modules {

// Polygon admission bound, independent of C line-array capacity.
// Must match the Bridge SCENE_MAX_OBSTACLES limit.
inline constexpr int kApaSceneMaxObstacles = 4096;
inline constexpr std::size_t kApaSceneMaxMessageBytes = 2 * 1024 * 1024;

}
