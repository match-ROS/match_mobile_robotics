/**
 * @file distance_monitor.hpp
 * @brief Backward compatibility header - redirects to nodes/distance_monitor_node.hpp
 *
 * This header is kept for backward compatibility. New code should include
 * <scene_builder/nodes/distance_monitor_node.hpp> directly.
 *
 * @deprecated Use <scene_builder/nodes/distance_monitor_node.hpp> instead
 */

#ifndef SCENE_BUILDER_DISTANCE_MONITOR_HPP
#define SCENE_BUILDER_DISTANCE_MONITOR_HPP

#pragma message("scene_builder/distance_monitor.hpp is deprecated. Use scene_builder/nodes/distance_monitor_node.hpp instead.")

#include "scene_builder/nodes/distance_monitor_node.hpp"

namespace scene_builder
{
// Backward compatibility alias
using DistanceMonitor = nodes::DistanceMonitorNode;
}  // namespace scene_builder

#endif  // SCENE_BUILDER_DISTANCE_MONITOR_HPP
