#pragma once

#include "fwd.hh"

#include <gepetto/gui/plugin-interface.hh>

#include <QObject>

class QDockWidget;

namespace gepetto {
namespace pinocchio {

class FrameTreeWidget;

/// Plugin that add features to work with pinocchio.
class PinocchioPlugin : public QObject, public gepetto::gui::PluginInterface
{
  Q_OBJECT
  Q_INTERFACES (gepetto::gui::PluginInterface)

#if (QT_VERSION >= QT_VERSION_CHECK(5,0,0))
Q_PLUGIN_METADATA (IID "pinocchio-gepetto-viewer.pinocchio-plugin")
#endif

  public:
    explicit PinocchioPlugin ();

    // PluginInterface interface
  public:
    /// Initialize the plugin.
    void init();

    /// Returns the plugin's name.
    QString name() const;

    void update();

  Q_SIGNALS:
    //void logSuccess (const QString& text);
    //void logFailure (const QString& text);
};

/// Register QtPincchio to PythonQt
void registerQtPinocchio ();

} // namespace gepetto
} // namespace pinocchio
