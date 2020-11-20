//
// Copyright (c) CNRS
// Author: Joseph Mirabel
//

#pragma once

#include <QWidget>

#include "plugin.hh"

namespace Ui {
class FrameTreeWidget;
}

namespace gepetto {
namespace pinocchio {

class FrameTreeWidget : public QWidget
{
  Q_OBJECT

  public:
    explicit FrameTreeWidget(PinocchioPlugin *plugin, QWidget *parent = 0);

    virtual ~FrameTreeWidget ();

    /// Get the currently selected joint.
    std::string selectedJoint () const;

    void makeTree ();

  private:
    /// Reset the tree.
    void reset ();

    struct FrameItem : QStandardItem {
      int i;
    };

    PinocchioPlugin* plugin_;
    ::Ui::FrameTreeWidget* ui_;

    std::vector<FrameItem*> items_;

    QStandardItemModel* model_;
};

} // namespace gepetto
} // namespace pinocchio
