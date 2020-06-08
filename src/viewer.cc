#include "pinocchio/gepetto/viewer.hpp"

#include <gepetto/viewer/corba/client.hh>

namespace pinocchio {
namespace gepetto {
namespace corba = ::gepetto::viewer::corba;

typedef ::gepetto::corbaserver::GraphicalInterface_var GUI_t;

#define MakePosition(name,c) ::gepetto::corbaserver::Position name{(float)(c[0]), (float)(c[1]), (float)(c[2])}

bool ViewerBase::initViewer(const std::string& windowName, bool loadModel)
{
  corba::connect(windowName.c_str(), true);
  if (!corba::connected()) return false;
  window = windowName;
  if (loadModel) loadViewerModel();
  return true;
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

  std::string viewerCollisionGroupName = scene + "/collisions";
  //if (!gui->nodeExists(viewerCollisionGroupName))
  gui->createGroup(viewerCollisionGroupName.c_str());

  std::string viewerVisualGroupName = scene + "/visuals";
  //if (!gui->nodeExists(viewerVisualGroupName))
  gui->createGroup(viewerVisualGroupName.c_str());

  // iterate over visuals and create the meshes in the viewer
  bool has_col (cmodel!=NULL);
  bool has_vis (vmodel!=NULL);
  if (has_col)
    for (const auto& go : cmodel->geometryObjects)
      loadViewerGeometryObject(go,COLLISION);
  // Display collision if we have them and there is no visual
  displayCollisions(has_col && !has_vis);

  if (has_vis)
    for (const auto& go : vmodel->geometryObjects)
      loadViewerGeometryObject(go,VISUAL);
  displayVisuals(has_vis);
}

bool ViewerBase::loadPrimitive(const char* meshName, const GeometryObject& go)
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
      if (gui->addMesh(meshName, go.meshPath.c_str()))
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

void ViewerBase::loadViewerGeometryObject(const GeometryObject& go, GeometryType gtype)
{
  std::string meshName = getViewerNodeName(go,gtype);
  if (loadPrimitive(meshName.c_str(), go)) {
    GUI_t& gui (corba::gui());
    gui->addToGroup(meshName.c_str(),
        (scene + (gtype == VISUAL ? "/visuals" : "/collisions")).c_str());
    MakePosition(s, go.meshScale);
    gui->setScale(meshName.c_str(), s);
  }
}

void setVisibility(GeometryModel const* gm, bool visible,
    const std::string& prefix)
{
  if (gm==NULL) return;
  if (!corba::connected()) return;
  GUI_t& gui (corba::gui());

  const char* mode = (visible ? "ON" : "OFF");

  std::string nodeName;
  for (const auto& go : gm->geometryObjects) {
    nodeName = prefix + go.name;
    gui->setVisibility(nodeName.c_str(),mode);
  }
}

void ViewerBase::displayCollisions(bool visibility)
{
  _displayCollisions = visibility;
  setVisibility(cmodel, visibility, scene + "/collisions/");
}

void ViewerBase::displayVisuals(bool visibility)
{
  _displayVisuals = visibility;
  setVisibility(vmodel, visibility, scene + "/visuals/");
}

void applyConfigurations (const std::string& prefix,
    const GeometryModel& gm, const GeometryData& gd)
{
  using namespace ::gepetto::corbaserver;
  CORBA::ULong size ((CORBA::ULong)gm.geometryObjects.size());

  char** nameList = Names_t::allocbuf(size);
  Names_t names (size, size, nameList);

  Transform* posSeq = TransformSeq::allocbuf(size);
  TransformSeq seq (size, size, posSeq);

  CORBA::ULong i = 0;
  for (const auto& go : gm.geometryObjects) {
    const auto& oMg = gd.oMg[i];

    posSeq[i][0] = static_cast<float>(oMg.translation()[0]);
    posSeq[i][1] = static_cast<float>(oMg.translation()[1]);
    posSeq[i][2] = static_cast<float>(oMg.translation()[2]);
    Eigen::Quaterniond quat(oMg.rotation());
    posSeq[i][3] = static_cast<float>(quat.x());
    posSeq[i][4] = static_cast<float>(quat.y());
    posSeq[i][5] = static_cast<float>(quat.z());
    posSeq[i][6] = static_cast<float>(quat.w());

    nameList[i] = new char[prefix.size()+go.name.size()+1];
    strcpy (nameList[i], prefix.c_str());
    strcpy (nameList[i]+prefix.size(), go.name.c_str());

    ++i;
  }

  GUI_t& gui (corba::gui());
  gui->applyConfigurations(names, seq);
}

void ViewerBase::applyVisuals()
{
  if (!_displayVisuals || vmodel == NULL) return;
  if (!corba::connected()) return;
  GUI_t& gui (corba::gui());
  applyConfigurations("v_", *vmodel, *vdata);
  gui->refresh();
}

void ViewerBase::applyCollisions()
{
  if (!_displayCollisions || cmodel == NULL) return;
  if (!corba::connected()) return;
  GUI_t& gui (corba::gui());
  applyConfigurations("c_", *cmodel, *cdata);
  gui->refresh();
}

}
}
