#include <pinocchio/gepetto/viewer.hh>

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

  pin::gepetto::Viewer viewer (model, &vmodel, NULL);
  viewer.initViewer("pinocchio");
  viewer.loadViewerModel("ur5");

  viewer.display(pin::neutral(model));
  viewer.display(pin::randomConfiguration(model));
}
