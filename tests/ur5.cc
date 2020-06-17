//! [Header include]
#include <pinocchio/gepetto/viewer.hpp>
//! [Header include]

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/utils.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace pin = pinocchio;

int main()
{
  std::string urdf = pin::retrieveResourcePath (
      "package://example-robot-data/robots/ur_description/urdf/ur5_robot.urdf",
      pin::rosPaths());

  pin::Model model;

  pin::urdf::buildModel (urdf, model, false);

  pin::GeometryModel vmodel;
  pin::urdf::buildGeom (model, urdf, pin::VISUAL, vmodel, pin::rosPaths());

//! [Create a viewer]
  pin::gepetto::Viewer viewer (model, &vmodel, NULL);
  viewer.initViewer("pinocchio");
  viewer.loadViewerModel("ur5");
//! [Create a viewer]

//! [Display a configuration]
  viewer.display(pin::neutral(model));
//! [Display a configuration]
  viewer.display(pin::randomConfiguration(model));
}
