//
// Copyright (c) CNRS
// Authors: Joseph Mirabel and Heidy Dallard
//

#include "plugin.hh"

#include <QAction>
#include <QMenuBar>
#include <QFileDialog>
#include <QUrl>
#include <QDockWidget>

#include <gepetto/gui/mainwindow.hh>
#include <gepetto/gui/windows-manager.hh>
#include <gepetto/gui/action-search-bar.hh>
#include <gepetto/gui/safeapplication.hh>

#include "frame-tree.hh"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/utils.hpp>

namespace gepetto {
namespace pinocchio {

using gepetto::gui::MainWindow;

PinocchioPlugin::PinocchioPlugin()
{
}

void PinocchioPlugin::init()
{
  registerQtPinocchio();

  // TODO: add a button to load a robot from a file.
  MainWindow* main = MainWindow::instance();
  main->menuBar()->addMenu("Pinocchio")->addAction("Load model from &URDF",
      []() {
        MainWindow* main = MainWindow::instance ();
        QUrl url = QFileDialog::getOpenFileUrl(main, "Select a URDF file",
            QUrl(), "URDF (*.urdf);;All files (*)", nullptr, QFileDialog::Options(),
            QStringList() << "package");

        if (!url.isEmpty()) {
          FrameTreeWidget* frameTree = new FrameTreeWidget();
          frameTree->putInsideDockWidget();
          frameTree->loadModelFromUrdf(url.path());
          frameTree->initViewer(QString());
        }
      });
}

QString PinocchioPlugin::name() const
{
  return QString ("Widgets for hpp-corbaserver");
}

#if (QT_VERSION < QT_VERSION_CHECK(5,0,0))
Q_EXPORT_PLUGIN2 (pinocchio-plugin, PinocchioPlugin)
#endif
} // namespace gepetto
} // namespace pinocchio
