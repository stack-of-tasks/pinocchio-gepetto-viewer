//
// Copyright (c) CNRS
// Authors: Joseph Mirabel and Heidy Dallard
//

#include "plugin.hh"

#include <QAction>
#include <QDockWidget>

#include <gepetto/gui/mainwindow.hh>
#include <gepetto/gui/windows-manager.hh>
#include <gepetto/gui/action-search-bar.hh>
#include <gepetto/gui/safeapplication.hh>

#include "frame-tree.hh"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/utils.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace gepetto {
namespace pinocchio {

using gepetto::gui::MainWindow;

typedef gepetto::viewer::WindowsManager::Color_t OsgColor_t;
typedef gepetto::viewer::Configuration OsgConfiguration_t;
typedef gepetto::gui::ActionSearchBar ActionSearchBar;

PinocchioPlugin::PinocchioPlugin() :
  frameTree_ (NULL)
{
}

PinocchioPlugin::~PinocchioPlugin()
{
  MainWindow* main = MainWindow::instance ();
  foreach (QDockWidget* dock, dockWidgets_) {
    main->removeDockWidget(dock);
    delete dock;
  }
}

void PinocchioPlugin::init()
{
  qDebug() << "init";
  MainWindow* main = MainWindow::instance ();
  QDockWidget* dock;

  // Joint tree widget
  dock = new QDockWidget ("&Joint Tree", main);
  dock->setObjectName ("pinocchio-plugin.frametree");
  frameTree_ = new FrameTreeWidget (this, dock);
  dock->setWidget(frameTree_);
  main->insertDockWidget (dock, Qt::RightDockWidgetArea, Qt::Vertical);
  dock->toggleViewAction()->setShortcut(gepetto::gui::DockKeyShortcutBase + Qt::Key_J);
  dockWidgets_.append(dock);
  main->registerShortcut("FrameTree", "Toggle view", dock->toggleViewAction());

  std::string urdf = ::pinocchio::retrieveResourcePath (
      "package://example-robot-data/robots/ur_description/urdf/ur5_robot.urdf",
      ::pinocchio::rosPaths());

  ::pinocchio::Model* model = new ::pinocchio::Model;
  ::pinocchio::urdf::buildModel (urdf, *model, false);

  ::pinocchio::GeometryModel* vmodel = new ::pinocchio::GeometryModel;
  ::pinocchio::urdf::buildGeom (*model, urdf, ::pinocchio::VISUAL, *vmodel, ::pinocchio::rosPaths());

  loadModel(*model, vmodel);

  //viewer_->initViewer("pinocchio");
  //viewer_->loadViewerModel("ur5");
  //viewer_->display(::pinocchio::neutral(*model));

  update();
}

QString PinocchioPlugin::name() const
{
  return QString ("Widgets for hpp-corbaserver");
}

void PinocchioPlugin::loadModel(
    const ::pinocchio::Model& model,
    const ::pinocchio::GeometryModel* visual,
    const ::pinocchio::GeometryModel* collision)
{
  viewer_.reset(new ::pinocchio::gepetto::Viewer(model, visual, collision));
}

void PinocchioPlugin::setCurrentConfig (const Vector& q)
{
  config_ = q;
  MainWindow::instance()->requestApplyCurrentConfiguration();
}

PinocchioPlugin::Vector const* PinocchioPlugin::getCurrentConfig () const
{
  return &config_;
}

void PinocchioPlugin::setCurrentQtConfig (const QVector<double>& q)
{
  config_.resize (q.size());
  for (int i = 0; i < config_.size(); ++i) config_[i] = q[i];
  MainWindow::instance()->requestApplyCurrentConfiguration();
}

QVector<double> PinocchioPlugin::getCurrentQtConfig () const
{
  QVector<double> c (config_.size());
  for (int i = 0; i < config_.size(); ++i) c[i] = config_[i];
  return c;
}

void PinocchioPlugin::applyCurrentConfiguration()
{
  if (viewer_) {
    viewer_->display(config_);
    MainWindow::instance()->osg()->refresh();
  }
}

//void PinocchioPlugin::configurationValidation()
//{
  // TODO
  //emit configurationValidationStatus (col);
//}

void PinocchioPlugin::update()
{
  frameTree_->makeTree();
}

::pinocchio::gepetto::Viewer* PinocchioPlugin::viewer()
{
  return viewer_.get();
}

FrameTreeWidget* PinocchioPlugin::frameTreeWidget() const
{
  return frameTree_;
}

#if (QT_VERSION < QT_VERSION_CHECK(5,0,0))
Q_EXPORT_PLUGIN2 (pinocchio-plugin, PinocchioPlugin)
#endif
} // namespace gepetto
} // namespace pinocchio
