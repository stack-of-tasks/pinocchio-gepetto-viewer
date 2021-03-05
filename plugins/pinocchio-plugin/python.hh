#pragma once

#include <pinocchio/fwd.hpp>

#ifdef PINOCCHIO_WITH_PYTHON_INTERFACE
#include <boost/python.hpp>
#endif

#include <QObject>
#include <QWidget>
#include "frame-tree.hh"

namespace gepetto {
namespace pinocchio {

class QtPinocchio : public QObject
{
  Q_OBJECT

public Q_SLOTS:

  // ------- FrameTreeWidget ------------------------------------------- //
  FrameTreeWidget* new_FrameTreeWidget(QWidget *parent = nullptr);
  void delete_FrameTreeWidget(FrameTreeWidget* o);

  QDockWidget* putInsideDockWidget(FrameTreeWidget* o) { return o->putInsideDockWidget(); }

#ifdef PINOCCHIO_WITH_PYTHON_INTERFACE
  void loadModel (FrameTreeWidget* o, PyObject* model,
      PyObject* visual = nullptr,
      PyObject* collision = nullptr);
#endif

  void initViewer(FrameTreeWidget* o, QString modelName, QString window = QString()) { o->initViewer(modelName, window); }
  // ------- FrameTreeWidget ------------------------------------------- //
};

} // namespace pinocchio
} // namespace gepetto
