//
// Copyright (c) CNRS
// Author: Joseph Mirabel
//

#pragma once

#include "fwd.hh"

#include <QWidget>
#include <QStandardItem>
#include <QStandardItemModel>

#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/gepetto/viewer.hpp>

namespace Ui {
class FrameTreeWidget;
}

class QDockWidget;

namespace gepetto {
namespace pinocchio {

class FrameTreeModel;

/// \todo reimplement QStandardItemModel::buddy so that editing a joint
///       brings to editing the joint value.
class FrameTreeWidget : public QWidget
{
  Q_OBJECT

  public:
    typedef ::pinocchio::Model::ConfigVectorType Vector;

    explicit FrameTreeWidget(QWidget *parent = 0);

    virtual ~FrameTreeWidget ();

    QDockWidget* putInsideDockWidget();

    /// Get the currently selected joint.
    std::string selectedJoint () const;

    void makeTree ();

    /// Get the viewer
    ::pinocchio::gepetto::Viewer* viewer () { return viewer_.get(); }

    /// Get the pinocchio model
    ::pinocchio::Model *robotModel () { return robotModel_; }

    /// Get the pinocchio model
    const ::pinocchio::Model *robotModel () const { return robotModel_; }

    void loadModel(
        ::pinocchio::Model* model,
        const ::pinocchio::GeometryModel* visual = NULL,
        const ::pinocchio::GeometryModel* collision = NULL);

    /// \todo this create a memory leak. The model and the geometrical models
    ///       are never deleted.
    void loadModelFromUrdf(QString filename);

    void initViewer(QString modelName, QString window = "pinocchio");

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

    void setCurrentConfig (const Vector& q);

  public Q_SLOTS:
    void applyCurrentConfiguration();

  private Q_SLOTS:
    void itemChanged(QStandardItem *item);

    void customContextMenu (const QPoint& pos);

  private:
    /// Reset the tree.
    void reset ();

    struct FrameItem : QStandardItem {
      int i;
    };


    ::Ui::FrameTreeWidget* ui_;

    std::vector<FrameItem*> items_;

    QStandardItemModel* model_;

    std::unique_ptr<::pinocchio::gepetto::Viewer> viewer_;
    ::pinocchio::Model* robotModel_;
    Vector config_, velocity_;

    friend class FrameTreeModel;
};

class FrameTreeModel : public QStandardItemModel
{
public:
  FrameTreeModel(int rows, int columns, QObject *parent = nullptr)
    : QStandardItemModel(rows, columns, parent) {}

  FrameTreeModel(QObject *parent = nullptr)
    : QStandardItemModel(parent) {}

protected:
   virtual QModelIndex buddy(const QModelIndex &index) const override;
};

} // namespace gepetto
} // namespace pinocchio
