#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <pinocchio/gepetto/viewer/config.hpp>

namespace pinocchio {
namespace gepetto {

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
};

extern template class PINOCCHIO_GEPETTO_VIEWER_DLLAPI ViewerTpl<pinocchio::Model>;
typedef class ViewerTpl<pinocchio::Model> Viewer;

}
}

#include <pinocchio/gepetto/viewer.hxx>
