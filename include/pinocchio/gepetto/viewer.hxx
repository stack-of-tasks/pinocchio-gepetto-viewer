#ifndef __pinocchio_gepetto_viewer_viewer_hxx__
#define __pinocchio_gepetto_viewer_viewer_hxx__

#include "pinocchio/gepetto/viewer.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>

namespace pinocchio {
namespace gepetto {
template<typename Model>
void ViewerTpl<Model>::display(Eigen::VectorXd q)
{
  // Update the robot kinematics and geometry.
  pinocchio::forwardKinematics(model,data,q);

  if (!frameData.i.empty()) {
    int k = 0;
    for (FrameIndex i : frameData.i)
      convert(pinocchio::updateFramePlacement(model, data, i),
          frameData.qs.col(k++));
    applyFrames();
  }

  if (collision.display && collision.model != NULL) {
    pinocchio::updateGeometryPlacements(model, data, *collision.model, *collision.data);
    applyCollisions();
  }

  if (visual.display && visual.model != NULL) {
    pinocchio::updateGeometryPlacements(model, data, *visual.model, *visual.data);
    applyVisuals();
  }
}

template<typename Model>
void ViewerTpl<Model>::addFrame(FrameIndex i)
{
  if (i < 0 || i > (FrameIndex)model.nframes)
    throw std::invalid_argument("Incorrect frame index.");
  Config pos;
  convert(pinocchio::updateFramePlacement(model, data, i), pos);
  frameData.add(i, pos, scene, model.frames[i].name);
}

template<typename Model>
void ViewerTpl<Model>::toggleFrame(FrameIndex i)
{
  if (i < 0 || i > (FrameIndex)model.nframes)
    throw std::invalid_argument("Incorrect frame index.");
  Config pos;
  convert(pinocchio::updateFramePlacement(model, data, i), pos);
  frameData.toggle(i, pos, scene, model.frames[i].name);
}
}
}

#endif // __pinocchio_gepetto_viewer_viewer_hxx__