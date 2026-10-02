#pragma once

#include <string>

#include <offboard_controllers/lee_px4_adapter.hpp>
#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::vehicle_config
{

struct LeePhysicalConfiguration
{
  // Physical inertia expressed in the FRD body frame expected by the
  // rotational controller.
  se3::InertiaMatrix inertia_frd{};

  // Vehicle-specific conversion from Lee's physical wrench to the normalized
  // wrench consumed by the pinned PX4 allocator/output chain.
  lee_px4_adapter::Parameters adapter{};
};


LeePhysicalConfiguration load_lee_physical_configuration(
  const std::string & config_directory,
  const std::string & vehicle);

}  // namespace offboard_controllers::vehicle_config
