#pragma once

#include <gepetto/gui/plugin-interface.hh>
#include <gepetto/gui/windows-manager.hh>

#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/gepetto/viewer.hpp>

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
    typedef ::pinocchio::Model::ConfigVectorType Vector;

    explicit PinocchioPlugin ();

    virtual ~PinocchioPlugin ();

    // PluginInterface interface
  public:
    /// Initialize the plugin.
    void init();

    /// Returns the plugin's name.
    QString name() const;

    const Vector& currentConfig () const
    {
      return config_;
    }

    Vector& currentConfig ()
    {
      return config_;
    }

    const Vector& currentVelocity () const
    {
      return velocity_;
    }

    Vector& currentVelocity ()
    {
      return velocity_;
    }

    void loadModel(
        const ::pinocchio::Model& model,
        const ::pinocchio::GeometryModel* visual = NULL,
        const ::pinocchio::GeometryModel* collision = NULL);

signals:
    //void configurationValidationStatus (bool valid);
    //void configurationValidationStatus (QStringList collision);

    public slots:
    /// Apply the current configuration of the robot.
      void applyCurrentConfiguration ();

    void setCurrentConfig (const Vector& q);

    Vector const* getCurrentConfig () const;

    void setCurrentQtConfig (const QVector<double>& q);

    QVector<double> getCurrentQtConfig () const;

    /// Build a list of bodies in collision.
    //void configurationValidation ();

    void update();

  public:
    /// Get the viewer
    ::pinocchio::gepetto::Viewer* viewer ();

    /// Get the Joint Tree widget.
    FrameTreeWidget* frameTreeWidget() const;

  signals:
    //void logSuccess (const QString& text);
    //void logFailure (const QString& text);

  protected slots:

  protected:
    QList <QDockWidget*> dockWidgets_;
    FrameTreeWidget* frameTree_;

    std::unique_ptr<::pinocchio::gepetto::Viewer> viewer_;
    Vector config_, velocity_;
};
} // namespace gepetto
} // namespace pinocchio
