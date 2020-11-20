//
// Copyright (c) CNRS
// Authors: Joseph Mirabel
//

#include <QMenu>

#include "frame-tree.hh"
#include "ui_frame-tree.h"

#include <gepetto/viewer/group-node.h>

#include <gepetto/gui/mainwindow.hh>
#include <gepetto/gui/bodytreewidget.hh>
#include <gepetto/gui/windows-manager.hh>
#if GEPETTO_GUI_HAS_PYTHONQT
#include <gepetto/gui/pythonwidget.hh>
#endif

#include "frame-tree.hh"

namespace gepetto {
namespace pinocchio {

using gepetto::gui::MainWindow;

FrameTreeWidget::FrameTreeWidget(PinocchioPlugin *plugin, QWidget *parent) :
  QWidget(parent),
  plugin_ (plugin),
  ui_ (new ::Ui::FrameTreeWidget),
  model_ (new QStandardItemModel)
{
  ui_->setupUi (this);
  ui_->frameTree->setModel(model_);
  /*
  ui_->frameTree->setItemDelegate (
      new JointItemDelegate(ui_->button_forceVelocity,
        plugin_,
        MainWindow::instance()));
        */
  reset ();

  connect(ui_->frameTree, SIGNAL (customContextMenuRequested(QPoint)),
      SLOT (customContextMenu(QPoint)));
  connect(ui_->frameTree, SIGNAL (expanded(QModelIndex)),
      SLOT (resize(QModelIndex)));
  connect(ui_->frameTree->selectionModel(), SIGNAL (currentChanged(QModelIndex, QModelIndex)),
      SLOT (currentJointChanged(QModelIndex,QModelIndex)));
}

FrameTreeWidget::~FrameTreeWidget()
{
  delete ui_;
}

void FrameTreeWidget::makeTree()
{
  qDebug() << "make tree";
  reset ();
  auto viewer (plugin_->viewer());
  if (viewer) {
    const auto& model = viewer->model;
    items_.resize(model.nframes);
    for (int i = 0; i < model.nframes; ++i) {
      FrameItem* item = new FrameItem;
      item->i = i;
      item->setText(QString::fromStdString(model.frames[i].name));
      if (i > 0)
        //item->appendRow(items_[model.frames[i].previousFrame]);
        items_[model.frames[i].previousFrame]->appendRow(item);
      else
        model_->appendRow(item);
      items_[i] = item;
    }
  }
}

void FrameTreeWidget::reset()
{
  model_->clear();
  items_.clear();
  ui_->frameTree->header()->setVisible(true);
  QStringList l; l << "Joint" << "Lower bound" << "Upper bound";
  model_->setHorizontalHeaderLabels(l);
  model_->setColumnCount(3);
  ui_->frameTree->setColumnHidden(1,true);
  ui_->frameTree->setColumnHidden(2,true);
}

} // namespace gepetto
} // namespace pinocchio
