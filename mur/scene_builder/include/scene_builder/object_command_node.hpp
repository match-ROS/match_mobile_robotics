/**
 * @file object_command_node.hpp
 * @brief Backward compatibility header - redirects to nodes/object_command_node.hpp
 *
 * This header is kept for backward compatibility. New code should include
 * <scene_builder/nodes/object_command_node.hpp> directly.
 *
 * @deprecated Use <scene_builder/nodes/object_command_node.hpp> instead
 */

#ifndef SCENE_BUILDER_OBJECT_COMMAND_NODE_HPP
#define SCENE_BUILDER_OBJECT_COMMAND_NODE_HPP

#pragma message("scene_builder/object_command_node.hpp is deprecated. Use scene_builder/nodes/object_command_node.hpp instead.")

#include "scene_builder/nodes/object_command_node.hpp"

namespace scene_builder
{
// Backward compatibility alias
using ObjectCommandNode = nodes::ObjectCommandNode;
}  // namespace scene_builder

#endif  // SCENE_BUILDER_OBJECT_COMMAND_NODE_HPP
