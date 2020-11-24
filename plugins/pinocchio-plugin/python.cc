#include "python.hh"
#include "plugin.hh"

#include <PythonQt.h>

namespace gepetto {
namespace pinocchio {

FrameTreeWidget* QtPinocchio::new_FrameTreeWidget(QWidget *parent)
{
  return new FrameTreeWidget(parent);
}

void QtPinocchio::delete_FrameTreeWidget(FrameTreeWidget* o)
{
  delete o;
}

#ifdef PINOCCHIO_WITH_PYTHON_INTERFACE
void QtPinocchio::loadModel (FrameTreeWidget* o,
    PyObject* model,
    PyObject* visual,
    PyObject* collision)
{
  boost::python::import("pinocchio");
  using ::pinocchio::GeometryModel;
  GeometryModel const *gv = nullptr, *gc = nullptr;
  if (visual != nullptr)
    gv = boost::python::extract<GeometryModel const*>(visual);
  if (collision != nullptr)
    gc = boost::python::extract<GeometryModel const*>(collision);
  o->loadModel(
      boost::python::extract<::pinocchio::Model*>(model), gv, gc);
}
#endif

void registerQtPinocchio ()
{
  PythonQt::self()->addDecorators (new QtPinocchio ());
  PythonQt::self()->registerCPPClass ("FrameTreeWidget", "QWidget", "QtPinocchio");
}

} // namespace pinocchio
} // namespace gepetto
