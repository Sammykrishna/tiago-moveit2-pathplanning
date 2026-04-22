#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <atomic>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace moveit_wrapper {

inline double sanitize_dimension(double value, double fallback = 0.01) {
  if (!std::isfinite(value) || value <= 0.0) {
    return fallback;
  }
  return value;
}

/**
 * @brief Convert a visualization_msgs::Marker to a MoveIt CollisionObject.
 *
 * Supported marker types and interpretation:
 * - SPHERE:   scale.x = diameter  -> radius = scale.x * 0.5
 * - CUBE:     scale.{x,y,z}       -> box dimensions
 * - CYLINDER: scale.x = diameter, scale.z = height
 *
 * All dimensions are sanitized to be strictly positive and finite.
 */
inline moveit_msgs::msg::CollisionObject
marker_to_collision_object(const visualization_msgs::msg::Marker &marker) {
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.header = marker.header;
  collision_obj.id =
      marker.ns.empty() ? ("object_" + std::to_string(marker.id)) : marker.ns;

  constexpr double default_dim = 0.01;

  if (marker.type == visualization_msgs::msg::Marker::SPHERE) {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions.resize(1);

    double diameter = sanitize_dimension(marker.scale.x, default_dim);
    double radius = 0.5 * diameter;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] =
        radius;

    collision_obj.primitives.push_back(primitive);
    collision_obj.primitive_poses.push_back(marker.pose);

  } else if (marker.type == visualization_msgs::msg::Marker::CUBE) {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);

    double sx = sanitize_dimension(marker.scale.x, default_dim);
    double sy = sanitize_dimension(marker.scale.y, default_dim);
    double sz = sanitize_dimension(marker.scale.z, default_dim);

    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = sx;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = sy;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = sz;

    collision_obj.primitives.push_back(primitive);
    collision_obj.primitive_poses.push_back(marker.pose);

  } else if (marker.type == visualization_msgs::msg::Marker::CYLINDER) {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);

    double height = sanitize_dimension(marker.scale.z, default_dim);
    double diameter = sanitize_dimension(marker.scale.x, default_dim);
    double radius = 0.5 * diameter;

    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] =
        height;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] =
        radius;

    collision_obj.primitives.push_back(primitive);
    collision_obj.primitive_poses.push_back(marker.pose);
  }

  collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return collision_obj;
}

/**
 * @brief Safe wrapper that returns std::optional.
 *
 * Returns std::nullopt and logs a warning if:
 *  - marker type is unsupported
 *  - resulting CollisionObject has no primitives
 *  - any primitive dimension is non-finite or non-positive
 */
inline std::optional<moveit_msgs::msg::CollisionObject>
marker_to_collision_object_safe(const visualization_msgs::msg::Marker &marker,
                                const rclcpp::Logger &logger) {
  moveit_msgs::msg::CollisionObject obj = marker_to_collision_object(marker);

  if (obj.primitives.empty() || obj.primitive_poses.empty()) {
    RCLCPP_WARN(logger,
                "Skipping marker ns='%s', id=%d: unsupported type %d or "
                "no primitives generated",
                marker.ns.c_str(), marker.id, marker.type);
    return std::nullopt;
  }

  for (const auto &primitive : obj.primitives) {
    for (double dim : primitive.dimensions) {
      if (!std::isfinite(dim) || dim <= 0.0) {
        RCLCPP_WARN(logger,
                    "Skipping marker ns='%s', id=%d: invalid primitive "
                    "dimension detected",
                    marker.ns.c_str(), marker.id);
        return std::nullopt;
      }
    }
  }

  return obj;
}

/**
 * @brief Convert a MoveIt CollisionObject to a visualization_msgs::Marker.
 *
 * Only the first primitive is converted. Colors are set by r/g/b (alpha 0.8).
 */
inline visualization_msgs::msg::Marker
collision_object_to_marker(const moveit_msgs::msg::CollisionObject &obj,
                           const std::string &ns = "", float r = 1.0f,
                           float g = 0.0f, float b = 0.0f) {
  visualization_msgs::msg::Marker marker;
  marker.header = obj.header;
  marker.ns = ns.empty() ? obj.id : ns;
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::ADD;

  if (!obj.primitives.empty() && !obj.primitive_poses.empty()) {
    const auto &primitive = obj.primitives[0];
    marker.pose = obj.primitive_poses[0];

    if (primitive.type == shape_msgs::msg::SolidPrimitive::BOX &&
        primitive.dimensions.size() >= 3) {
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.scale.x =
          primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
      marker.scale.y =
          primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
      marker.scale.z =
          primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];
    } else if (primitive.type == shape_msgs::msg::SolidPrimitive::SPHERE &&
               primitive.dimensions.size() >= 1) {
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      double radius =
          primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS];
      double diameter = 2.0 * radius;
      marker.scale.x = marker.scale.y = marker.scale.z = diameter;
    } else if (primitive.type == shape_msgs::msg::SolidPrimitive::CYLINDER &&
               primitive.dimensions.size() >= 2) {
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      double height =
          primitive
              .dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];
      double radius =
          primitive
              .dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];
      marker.scale.x = marker.scale.y = 2.0 * radius;
      marker.scale.z = height;
    }
  }

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.8f;
  return marker;
}

/**
 * @brief Convert a MoveIt AttachedCollisionObject to a marker for
 * visualization.
 */
inline visualization_msgs::msg::Marker attached_collision_object_to_marker(
    const moveit_msgs::msg::AttachedCollisionObject &attached,
    const std::string &ns = "", float r = 0.0f, float g = 1.0f,
    float b = 0.0f) {
  return collision_object_to_marker(
      attached.object, ns.empty() ? attached.object.id : ns, r, g, b);
}

} // namespace moveit_wrapper
