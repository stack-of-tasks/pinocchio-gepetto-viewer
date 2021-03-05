#pragma once

#include "fwd.hh"

#include <QHeaderView>
#include <QMenu>
#include <QDebug>
#include <QMouseEvent>

namespace gepetto {
namespace pinocchio {

class HeaderView : public QHeaderView {
public:
  HeaderView(Qt::Orientation orientation, QWidget *parent = nullptr)
    : QHeaderView (orientation, parent)
  {}

protected:
  void mousePressEvent(QMouseEvent *e) override
  {
    if (e->button() == Qt::RightButton) {
      e->accept();

      QMenu* menu = new QMenu("Header menu", this);
      auto model = this->model();
      int nvisible = 0;
      for (int i = 0; i < count(); ++i)
      {
        QAction* action = menu->addAction(model->headerData(i, Qt::Horizontal).toString(),
            [this, i, &nvisible](bool checked) { if (checked || nvisible > 1) setSectionHidden(i, !checked); });
        action->setCheckable(true);
        action->setChecked(!isSectionHidden(i));
        if (!isSectionHidden(i)) ++nvisible;
      }
      menu->exec(mapToGlobal(e->pos()));
    }
  }
};
} // namespace gepetto
} // namespace pinocchio
