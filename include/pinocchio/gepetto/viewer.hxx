#pragma once

#include "pinocchio/gepetto/viewer.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>

namespace pinocchio {
namespace gepetto {
template<typename Model>
void Viewer<Model>::display(Eigen::VectorXd q)
{
  // Update the robot kinematics and geometry.
  pinocchio::forwardKinematics(model,data,q);

  if (_displayCollisions && cmodel != NULL) {
    pinocchio::updateGeometryPlacements(model, data, *cmodel, *cdata);
    applyCollisions();
  }

  if (_displayVisuals && vmodel != NULL) {
    pinocchio::updateGeometryPlacements(model, data, *vmodel, *vdata);
    applyVisuals();
  }
}
}
}
