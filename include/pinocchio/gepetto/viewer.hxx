#pragma once

#include "pinocchio/gepetto/viewer.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>

namespace pinocchio {
namespace gepetto {
template<typename Model>
void ViewerTpl<Model>::display(Eigen::VectorXd q)
{
  // Update the robot kinematics and geometry.
  pinocchio::forwardKinematics(model,data,q);

  if (collision.display && collision.model != NULL) {
    pinocchio::updateGeometryPlacements(model, data, *collision.model, *collision.data);
    applyCollisions();
  }

  if (visual.display && visual.model != NULL) {
    pinocchio::updateGeometryPlacements(model, data, *visual.model, *visual.data);
    applyVisuals();
  }
}
}
}
