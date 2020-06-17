#include "pinocchio/gepetto/viewer.hpp"

#include <gepetto/viewer/corba/client.hh>

namespace pinocchio {
namespace gepetto {
namespace corba = ::gepetto::viewer::corba;

typedef ::gepetto::corbaserver::GraphicalInterface_var GUI_t;

#define MakePosition(name,c) ::gepetto::corbaserver::Position name{(float)(c[0]), (float)(c[1]), (float)(c[2])}

struct ViewerBase::ViewerDataImpl {
  ::gepetto::corbaserver::Names_t names;
  ::gepetto::corbaserver::TransformSeq transforms;
};

ViewerBase::ViewerData::ViewerData(GeometryModel const* m)
  : model(m)
  , data(m == NULL ? NULL : new GeometryData(*m))
  , impl(m == NULL ? NULL : new ViewerDataImpl)
  , display(true)
{}

ViewerBase::ViewerData::~ViewerData() = default;

bool ViewerBase::initViewer(const std::string& windowName, bool loadModel)
{
  corba::connect(windowName.c_str(), true);
  if (!corba::connected()) return false;
  window = windowName;
  if (loadModel) loadViewerModel();
  return true;
}

bool loadPrimitive(const char* meshName, const GeometryObject& go);

void load(ViewerBase::ViewerData& d, const std::string& groupName, const std::string& prefix)
{
  GUI_t& gui (corba::gui());

  gui->createGroup(groupName.c_str());

  if (d.model == NULL) return;

  using namespace ::gepetto::corbaserver;
  CORBA::ULong size ((CORBA::ULong)d.model->geometryObjects.size());

  d.impl->names.length(size);
  d.impl->transforms.length(size);

  CORBA::ULong i = 0;
  for (const auto& go : d.model->geometryObjects) {
    std::string meshName = prefix + go.name;
    if (loadPrimitive(meshName.c_str(), go)) {
      gui->addToGroup(meshName.c_str(), groupName.c_str());
      MakePosition(s, go.meshScale);
      gui->setScale(meshName.c_str(), s);
    }

    d.impl->names[i] = new char[meshName.size()+1];
    memcpy (d.impl->names[i], meshName.c_str(), meshName.size()+1);

    ++i;
  }
}

void ViewerBase::loadViewerModel(const std::string& rootNodeName)
{
  if (!corba::connected()) return;
  GUI_t& gui (corba::gui());

  scene = rootNodeName;
  if (!gui->nodeExists(scene.c_str())) {
    gui->createGroup(scene.c_str());
    gui->addToGroup(scene.c_str(), window.c_str());
  }

  load(collision, scene + "/collisions", "c_");
  load(visual, scene + "/visuals", "v_");

  // iterate over visuals and create the meshes in the viewer
  bool has_col (collision.model!=NULL);
  bool has_vis (visual.model!=NULL);

  // Display collision if we have them and there is no visual
  displayCollisions(has_col && !has_vis);

  displayVisuals(has_vis);
}

bool loadPrimitive(const char* meshName, const GeometryObject& go)
{
  GUI_t& gui (corba::gui());
  if (gui->nodeExists(meshName)) return false;

  ::gepetto::corbaserver::Color color{(float)go.meshColor[0], (float)go.meshColor[1], (float)go.meshColor[2], (float)go.meshColor[3]};

#define CAST(TYPE) const TYPE& o = static_cast<TYPE&> (*go.geometry)
  using namespace hpp::fcl;
  switch (go.geometry->getNodeType())
  {
    default:
      throw std::logic_error("invalid hpp-fcl node type");
      break;
    case BV_AABB:
    case BV_OBB:
    case BV_RSS:
    case BV_kIOS:
    case BV_OBBRSS:
    case BV_KDOP16:
    case BV_KDOP18:
    case BV_KDOP24:
      if (!gui->addMesh(meshName, go.meshPath.c_str()))
        return false;
      if (go.overrideMaterial) {
        gui->setColor(meshName, color);
        if (!go.meshTexturePath.empty())
          gui->setTexture(meshName, go.meshTexturePath.c_str());
      }
      return true;
    case GEOM_BOX: {
                     CAST(Box);
                     return gui->addBox(meshName,
                         2*(float)o.halfSide[0], 2*(float)o.halfSide[1], 2*(float)o.halfSide[2],
                         color);
                   }
    case GEOM_SPHERE: {
                        CAST(Sphere);
                        return gui->addSphere(meshName, (float)o.radius, color);
                      }
    case GEOM_CAPSULE: {
                         CAST(Capsule);
                         return gui->addCapsule(meshName, (float)o.radius, 2*(float)o.halfLength, color);
                       }
    case GEOM_CONE: {
                      CAST(Cone);
                      return gui->addCone(meshName, (float)o.radius, 2*(float)o.halfLength, color);
                    }
    case GEOM_CYLINDER: {
                          CAST(Cylinder);
                          return gui->addCylinder(meshName, (float)o.radius, 2*(float)o.halfLength, color);
                        }
    case GEOM_CONVEX:
  /*
        elif isinstance(geom, hppfcl.Convex):
            pts = [ npToTuple(geom.points(geom.polygons(f)[i])) for f in range(geom.num_polygons) for i in range(3) ]
            gui->addCurve(meshName, pts, color)
            gui->setCurveMode(meshName, "TRIANGLES")
            gui->setLightingMode(meshName, "ON")
            gui->setBoolProperty(meshName, "BackfaceDrawing", True)
            return True
        elif isinstance(geom, hppfcl.ConvexBase):
            pts = [ npToTuple(geom.points(i)) for i in range(geom.num_points) ]
            gui->addCurve(meshName, pts, color)
            gui->setCurveMode(meshName, "POINTS")
            gui->setLightingMode(meshName, "OFF")
            return True
            */
    case GEOM_PLANE:
    case GEOM_HALFSPACE:
    case GEOM_TRIANGLE:
                        throw std::logic_error("unimplement primitive type.");
  }
  return false;
}

void ViewerBase::display(ViewerData& d, bool visibility)
{
  d.display = visibility;

  if (d.model==NULL) return;
  if (!corba::connected()) return;
  GUI_t& gui (corba::gui());

  const char* mode = (visibility ? "ON" : "OFF");

  std::string nodeName;
  for (CORBA::ULong i = 0; i < d.impl->names.length(); ++i)
    gui->setVisibility(d.impl->names[i], mode);
}

void ViewerBase::apply(ViewerData& d)
{
  if (!d.display || d.model == NULL) return;
  if (!corba::connected()) return;
  CORBA::ULong i = 0;
  for (const auto& oMg : d.data->oMg) {
    d.impl->transforms[i][0] = static_cast<float>(oMg.translation()[0]);
    d.impl->transforms[i][1] = static_cast<float>(oMg.translation()[1]);
    d.impl->transforms[i][2] = static_cast<float>(oMg.translation()[2]);
    Eigen::Quaterniond quat(oMg.rotation());
    d.impl->transforms[i][3] = static_cast<float>(quat.x());
    d.impl->transforms[i][4] = static_cast<float>(quat.y());
    d.impl->transforms[i][5] = static_cast<float>(quat.z());
    d.impl->transforms[i][6] = static_cast<float>(quat.w());

    ++i;
  }

  GUI_t& gui (corba::gui());
  gui->applyConfigurations(d.impl->names, d.impl->transforms);
  gui->refresh();
}

bool ViewerBase::connected()
{
  return (corba::connected());
}

template class ViewerTpl<pinocchio::Model>;

}
}
