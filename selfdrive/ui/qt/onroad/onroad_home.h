#pragma once

#include "selfdrive/ui/qt/onroad/alerts.h"
#include "selfdrive/ui/qt/onroad/annotated_camera.h"

#include <QLabel>
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);

private:
  void paintEvent(QPaintEvent *event);
  OnroadAlerts *alerts;
  AnnotatedCameraWidget *nvg;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QColor bg_long = bg_colors[STATUS_DISENGAGED];
  QHBoxLayout* split;
  void updateStateText();

  QLabel* topLabel;
  QLabel* topLeftLabel;
  QLabel* topRightLabel;
  QLabel* bottomLabel;
  QLabel* bottomLeftLabel;
  QLabel* bottomRightLabel;

private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);
};
