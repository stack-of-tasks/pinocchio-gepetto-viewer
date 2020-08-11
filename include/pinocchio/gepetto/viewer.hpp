#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <pinocchio/gepetto/viewer/config.hpp>

namespace pinocchio {
namespace gepetto {

typedef Eigen::Matrix<float, 7, 1> Config;
typedef Eigen::Map<Config> ConfigMap;
typedef Eigen::Map<const Config> ConfigConstMap;

template<typename SE3Derived, typename Vector7Like>
void convert(const SE3Base<SE3Derived>& M, const Eigen::MatrixBase<Vector7Like>& q)
{
  Vector7Like& qout = const_cast<Vector7Like&>(q.derived());

  qout.template head<3>() = M.translation().template cast<typename Vector7Like::Scalar>();
  Eigen::Quaternion<typename SE3Derived::Scalar> quat(M.rotation());
  qout.template tail<4>() = quat.coeffs().template cast<typename Vector7Like::Scalar>();
}

/** \brief Viewer base class.
 *
 * that implements the connection with gepetto-viewer.
 * If the viewer is not available, the methods of this class do *almost* nothing.
 */
struct PINOCCHIO_GEPETTO_VIEWER_DLLAPI ViewerBase
{
  struct ViewerDataImpl;
  struct ViewerData {
    GeometryModel const* model;
    std::unique_ptr<GeometryData> data;
    std::unique_ptr<ViewerDataImpl> impl;
    bool display;

    ViewerData(GeometryModel const* m);
    ~ViewerData();
  };

  /// Contains the information for the visual model, if any.
  ViewerData visual;
  /// Contains the information for the collision model, if any.
  ViewerData collision;

  /// Contains the information for displaying the frames.
  struct FrameData {
    std::vector<FrameIndex> i;
    Eigen::Matrix<float, 7, Eigen::Dynamic> qs;
    std::unique_ptr<ViewerDataImpl> impl;

    void add(FrameIndex i, const std::string& scene, const std::string& name);

    FrameData();
    ~FrameData();
  } frameData;


  std::string window, scene;

  /// \name Initialization
  /// \{

  /// Constructor
  /// \param visual a visual GeometryModel to display. Set to NULL to skip it.
  /// \param collision a collision GeometryModel to display. Set to NULL to skip it.
  ViewerBase(GeometryModel const* visual = NULL, GeometryModel const* collision = NULL)
    : visual(visual)
    , collision(collision)
  {}

  /// Create a window in the viewer.
  /// \param windowName
  /// \param loadModel if \c true, calls \ref loadViewerModel
  bool initViewer(const std::string& windowName, bool loadModel = false);

  /// Load the provided model in the viewer.
  void loadViewerModel(const std::string& rootNodeName = "world");

  /// \}

  /// Toggle the visibility of visual geometries.
  inline void displayVisuals(bool visibility)
  {
    display(visual, visibility);
  }

  /// Toggle the visibility of collision geometries.
  inline void displayCollisions(bool visibility)
  {
    display(collision, visibility);
  }

  /// Move the visual geometries to the placements in \c visual.data
  inline void applyVisuals()
  {
    apply(visual);
  }

  /// Move the collision geometries to the placements in \c collision.data
  inline void applyCollisions()
  {
    apply(collision);
  }

  /// Move the frame to the placements in \c frameData.qs
  void applyFrames();

  /// Tells whether this class is connected to the viewer.
  bool connected();

private:
  void display(ViewerData& d, bool visibility);
  void apply(ViewerData& d);
};

/** \brief A ViewerBase that computes the forward kinematics.
 * \tparam Model a pinocchio::ModelTpl class.
 */
template<typename Model>
struct PINOCCHIO_GEPETTO_VIEWER_DLLAPI ViewerTpl : ViewerBase
{
  /// \copydoc ViewerBase::ViewerBase
  /// \param model the pinocchio::ModelTpl to compute the forward kinematics.
  ViewerTpl(Model const& model, GeometryModel const* visual = NULL, GeometryModel const* collision = NULL)
    : ViewerBase (visual, collision)
    , model(model)
    , data(model)
  {}

  Model const& model;
  typename Model::Data data;

  /// \brief Display a configuration.
  /// It computes geometry placements in configuration \c q and update their
  /// pose in the viewer.
  void display(Eigen::VectorXd q);

  /// Add a visualization of a frame.
  /// \todo check for duplication ?
  void addFrame (FrameIndex i);
};

extern template class PINOCCHIO_GEPETTO_VIEWER_DLLAPI ViewerTpl<pinocchio::Model>;
typedef class ViewerTpl<pinocchio::Model> Viewer;

}
}

#include <pinocchio/gepetto/viewer.hxx>
