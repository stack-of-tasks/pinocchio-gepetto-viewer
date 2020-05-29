#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace pinocchio {
namespace gepetto {

struct Viewer {
  Viewer(Model const& m, GeometryModel const* vm = NULL, GeometryModel const* cm = NULL)
    : model(m)
    , data(model)
    , vmodel(vm)
    , _displayVisuals(true)
    , cmodel(cm)
    , _displayCollisions(!vm)
  {
    if (vm) vdata.reset(new GeometryData(*vm));
    if (cm) vdata.reset(new GeometryData(*cm));
  }

  Model const& model;
  Data data;

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
      case VISUAL   : return scene + "/visuals/"    + go.name;
      case COLLISION: return scene + "/collisions/" + go.name;
    }
    throw std::logic_error("invalid geometry type");
  }

  bool loadPrimitive(const char* meshName, const GeometryObject& go);

  void loadViewerGeometryObject(const GeometryObject& go, GeometryType gtype);

  void loadViewerModel(const std::string& rootNodeName = "world");

  void displayCollisions(bool visibility);

  void displayVisuals(bool visibility);

  void display(Eigen::VectorXd q);
};

}
}
