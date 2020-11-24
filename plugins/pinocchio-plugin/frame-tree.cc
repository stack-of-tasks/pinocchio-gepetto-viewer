//
// Copyright (c) CNRS
// Authors: Joseph Mirabel
//

#include "frame-tree.hh"

#include <QMenu>
#include <QDockWidget>

#include "ui_frame-tree.h"

#include <gepetto/viewer/group-node.h>

#include <gepetto/gui/mainwindow.hh>
#include <gepetto/gui/bodytreewidget.hh>
#include <gepetto/gui/windows-manager.hh>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/utils.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "frame-tree.hh"
#include "header-view.hh"

namespace gepetto {
namespace pinocchio {

using gepetto::gui::MainWindow;

constexpr int itemUserRole = Qt::UserRole + 1;
constexpr int itemIndexRole = Qt::UserRole + 2;

constexpr int itemCurrentValue = 0;
constexpr int itemLowerBound = 1;
constexpr int itemUpperBound = 2;

QStandardItem* makeItem (double value, int iq, int role)
{
  QStandardItem *item = new QStandardItem();
  item->setData(value, Qt::DisplayRole);
  item->setData(role, itemUserRole);
  item->setData(iq, itemIndexRole);
  item->setEditable(true);
  return item;
}

FrameTreeWidget::FrameTreeWidget(QWidget *parent) :
  QWidget(parent),
  ui_ (new ::Ui::FrameTreeWidget),
  model_ (new FrameTreeModel(0,4))
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
  connect(model_, SIGNAL(itemChanged(QStandardItem*)),
      SLOT(itemChanged(QStandardItem*)));

  MainWindow* main = MainWindow::instance();
  connect (main, SIGNAL (applyCurrentConfiguration()),
      SLOT (applyCurrentConfiguration()));
}

FrameTreeWidget::~FrameTreeWidget()
{
  delete ui_;
}

QDockWidget* FrameTreeWidget::putInsideDockWidget()
{
  MainWindow* main = MainWindow::instance();

  // Joint tree widget
  QDockWidget* dock = new QDockWidget ("&Joint Tree", main);
  dock->setObjectName ("pinocchio-plugin.frametree");
  dock->setWidget(this);
  main->insertDockWidget (dock, Qt::RightDockWidgetArea, Qt::Vertical);

  return dock;
}

void FrameTreeWidget::loadModel(
    ::pinocchio::Model* model,
    const ::pinocchio::GeometryModel* visual,
    const ::pinocchio::GeometryModel* collision)
{
  robotModel_ = model;
  viewer_.reset(new ::pinocchio::gepetto::Viewer(*model, visual, collision));

  config_ = ::pinocchio::neutral(*robotModel_);

  makeTree();
}

void FrameTreeWidget::loadModelFromUrdf(QString urdfFile)
{
  using namespace ::pinocchio;

  auto dirs = rosPaths();
  std::string urdf = retrieveResourcePath (urdfFile.toStdString(), dirs);

  Model* model = new ::pinocchio::Model;
  ::pinocchio::urdf::buildModel (urdf, *model, false);

  GeometryModel* vmodel = new GeometryModel;
  ::pinocchio::urdf::buildGeom (*model, urdf, VISUAL, *vmodel, dirs);

  GeometryModel* cmodel = new GeometryModel;
  ::pinocchio::urdf::buildGeom (*model, urdf, COLLISION, *cmodel, dirs);

  loadModel(model, vmodel, cmodel);
}

void FrameTreeWidget::initViewer(QString modelName, QString window)
{
  std::string w = window.toStdString();

  MainWindow* main = MainWindow::instance ();
  main->osg()->createWindow(w);
  viewer_->initViewer(w);
  viewer_->loadViewerModel(modelName.toStdString());
  applyCurrentConfiguration();
}

void FrameTreeWidget::setCurrentConfig (const Vector& q)
{
  config_ = q;
  MainWindow::instance()->requestApplyCurrentConfiguration();
}

void FrameTreeWidget::customContextMenu(const QPoint& pos)
{
  QAction* underCursor = NULL;

  QMenu* menu = new QMenu("Frame menu", this);
  QModelIndex index = ui_->frameTree->indexAt(pos);
  FrameItem* item = dynamic_cast<FrameItem*>(model_->itemFromIndex(index));
  if (item != NULL) {
    if (viewer_)
      underCursor = menu->addAction("Display &frame", [this, &item]() { viewer_->toggleFrame(item->i); });
  } else {
  }
  // Show contextual menu
  menu->exec(ui_->frameTree->mapToGlobal(pos), underCursor);
}

void FrameTreeWidget::makeTree()
{
  reset ();
  if (viewer_) {
    items_.resize(robotModel_->nframes);
    for (int i = 0; i < robotModel_->nframes; ++i) {
      const auto& frame = robotModel_->frames[i];
      FrameItem* item = new FrameItem;
      item->i = i;
      item->setText(QString::fromStdString(frame.name));
      item->setEditable(false);
      if (i > 0)
        if (frame.type == ::pinocchio::JOINT) {
          const auto& joint = robotModel_->joints[frame.parent];
          int idx_q = joint.idx_q();
          for (int iq = 0; iq < joint.nq(); ++iq) {
            QList<QStandardItem*> row;
            row << item
                << makeItem (currentConfig()[idx_q+iq], idx_q+iq, itemCurrentValue)
                << makeItem (robotModel_->lowerPositionLimit[idx_q+iq], idx_q+iq, itemLowerBound)
                << makeItem (robotModel_->upperPositionLimit[idx_q+iq], idx_q+iq, itemUpperBound);
            items_[frame.previousFrame]->appendRow(row);
          }
        } else
          items_[frame.previousFrame]->appendRow(item);
      else
        model_->appendRow(item);
      items_[i] = item;
    }
  }

  for (QStandardItem* item : items_)
    ui_->frameTree->setExpanded (item->index(), true);
}

void FrameTreeWidget::reset()
{
  model_->clear();
  items_.clear();
  ui_->frameTree->setHeader(new HeaderView(Qt::Horizontal));
  ui_->frameTree->header()->setVisible(true);
  QStringList l; l << "Joint" << "Current" << "Lower bound" << "Upper bound";
  model_->setHorizontalHeaderLabels(l);
  //model_->setColumnCount(3);
  //ui_->frameTree->setColumnHidden(1,true);
  //ui_->frameTree->setColumnHidden(2,true);
}

void FrameTreeWidget::applyCurrentConfiguration()
{
  if (viewer_) {
    viewer_->display(config_);
    MainWindow::instance()->osg()->refresh();
  }
}

void FrameTreeWidget::itemChanged(QStandardItem *item)
{
  QVariant role (item->data(itemUserRole));
  if (!role.isValid() || !role.canConvert<int>()) return;

  if (!viewer_) return;

  bool ok;
  int iq = item->data(itemIndexRole).toInt(&ok);
  if (!ok) return;

  switch (role.toInt()) {
    case itemLowerBound:
      if (!robotModel_) return;
      robotModel_->lowerPositionLimit[iq] = item->data(Qt::DisplayRole).toDouble();
      qDebug() << "new lower bound" << item->data(Qt::DisplayRole);
      break;
    case itemUpperBound:
      if (!robotModel_) return;
      robotModel_->upperPositionLimit[iq] = item->data(Qt::DisplayRole).toDouble();
      qDebug() << "new upper bound" << item->data(Qt::DisplayRole);
      break;
    case itemCurrentValue:
      {
        Vector q (currentConfig());
        q[iq] = item->data(Qt::DisplayRole).toDouble();
        setCurrentConfig(q);
      }
      break;
    default:
      qDebug() << "new unknown" << item->data(Qt::DisplayRole);
  }
}

QModelIndex FrameTreeModel::buddy(const QModelIndex &index) const
{
  FrameTreeWidget::FrameItem* item =
    dynamic_cast<FrameTreeWidget::FrameItem*>(itemFromIndex(index));
  if (item) {
    QModelIndex next = index.sibling(index.row(), 1);
    if (next.isValid()) return next;
  }
  return index;
}
} // namespace gepetto
} // namespace pinocchio
