#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace pinocchio {
namespace gepetto {

struct ViewerBase
{
  ViewerBase(GeometryModel const* vm = NULL, GeometryModel const* cm = NULL)
    : vmodel(vm)
    , _displayVisuals(vm)
    , cmodel(cm)
    , _displayCollisions(!vm && cm)
  {
    if (vm) vdata.reset(new GeometryData(*vm));
    if (cm) cdata.reset(new GeometryData(*cm));
  }

  GeometryModel const* vmodel;
  std::unique_ptr<GeometryData> vdata;
  bool _displayVisuals;

  GeometryModel const* cmodel;
  std::unique_ptr<GeometryData> cdata;
  bool _displayCollisions;

  std::string window, scene;

  bool initViewer(const std::string& windowName, bool loadModel = false);

  inline std::string getViewerNodeName(const GeometryObject& go, GeometryType gtype)
  {
    switch(gtype) {
      case VISUAL   : return "v_" + go.name;
      case COLLISION: return "c_" + go.name;
    }
    throw std::logic_error("invalid geometry type");
  }

  bool loadPrimitive(const char* meshName, const GeometryObject& go);

  void loadViewerGeometryObject(const GeometryObject& go, GeometryType gtype);

  void loadViewerModel(const std::string& rootNodeName = "world");

  void displayCollisions(bool visibility);

  void displayVisuals(bool visibility);

  void applyVisuals();

  void applyCollisions();

  bool connected();
};

template<typename Model>
struct Viewer : ViewerBase {
  Viewer(Model const& m, GeometryModel const* vm = NULL, GeometryModel const* cm = NULL)
    : ViewerBase (vm, cm)
    , model(m)
    , data(model)
  {}

  Model const& model;
  typename Model::Data data;

  void display(Eigen::VectorXd q);
};

}
}
