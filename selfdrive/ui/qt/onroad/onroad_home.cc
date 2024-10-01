#include "selfdrive/ui/qt/onroad/onroad_home.h"

#include <QPainter>
#include <QStackedLayout>


#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonArray>


#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/carrot.h"

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  //main_layout->setMargin(UI_BORDER_SIZE);
  main_layout->setContentsMargins(UI_BORDER_SIZE, 0, UI_BORDER_SIZE, 0);

  QFont font;
  font.setPixelSize(27);
  font.setWeight(QFont::DemiBold);
  QHBoxLayout* topLayout = new QHBoxLayout();
  topLeftLabel = new QLabel("", this);
  topLeftLabel->setFixedHeight(27); // 높이를 30 픽셀로 설정
  topLeftLabel->setAlignment(Qt::AlignLeft);
  topLeftLabel->setFont(font);
  topLeftLabel->setStyleSheet("QLabel { color : white; }");
  topLayout->addWidget(topLeftLabel);
  topLabel = new QLabel("", this);
  topLabel->setFixedHeight(27); // 높이를 30 픽셀로 설정
  topLabel->setAlignment(Qt::AlignCenter);
  topLabel->setFont(font);
  topLabel->setStyleSheet("QLabel { color : white; }");
  topLayout->addWidget(topLabel);
  topRightLabel = new QLabel("", this);
  topRightLabel->setFixedHeight(27); // 높이를 30 픽셀로 설정
  topRightLabel->setAlignment(Qt::AlignRight);
  topRightLabel->setFont(font);
  topRightLabel->setStyleSheet("QLabel { color : white; }");
  topLayout->addWidget(topRightLabel);
  main_layout->addLayout(topLayout);

  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  QHBoxLayout* bottomLayout = new QHBoxLayout();
  bottomLeftLabel = new QLabel("", this);
  bottomLeftLabel->setFixedHeight(27); // 높이를 30 픽셀로 설정
  bottomLeftLabel->setAlignment(Qt::AlignLeft);
  bottomLeftLabel->setFont(font);
  bottomLeftLabel->setStyleSheet("QLabel { color : white; }");
  bottomLayout->addWidget(bottomLeftLabel);
  bottomLabel = new QLabel("", this);
  bottomLabel->setFixedHeight(27); // 높이를 30 픽셀로 설정
  bottomLabel->setAlignment(Qt::AlignCenter);
  bottomLabel->setFont(font);
  bottomLabel->setStyleSheet("QLabel { color : white; }");
  bottomLayout->addWidget(bottomLabel);
  bottomRightLabel = new QLabel("", this);
  bottomRightLabel->setFixedHeight(27); // 높이를 30 픽셀로 설정
  bottomRightLabel->setAlignment(Qt::AlignRight);
  bottomRightLabel->setFont(font);
  bottomRightLabel->setStyleSheet("QLabel { color : white; }");
  bottomLayout->addWidget(bottomRightLabel);
  main_layout->addLayout(bottomLayout);


  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, this);
    split->insertWidget(0, arCam);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
}

void OnroadWindow::updateState(const UIState &s) {
  if (!s.scene.started) {
    return;
  }

  //alerts->updateState(s);
  ui_update_alert(OnroadAlerts::getAlert(*(s.sm), s.scene.started_frame));
  nvg->updateState(s);

  QColor bgColor = bg_colors[s.status];
  QColor bgColor_long = bg_colors[s.status];
  const SubMaster& sm = *(s.sm);
  const auto car_control = sm["carControl"].getCarControl();
  //if (s.status == STATUS_DISENGAGED && car_control.getLatActive()) {
  //    bgColor = bg_colors[STATUS_LAT_ACTIVE];
  //}
  const auto car_state = sm["carState"].getCarState();
  if (car_state.getSteeringPressed()) {
      bgColor = bg_colors[STATUS_OVERRIDE];
  }
  else if (car_control.getLatActive()) {
      bgColor = bg_colors[STATUS_ENGAGED];
  }
  else
      bgColor = bg_colors[STATUS_DISENGAGED];

  if (car_state.getGasPressed()) {
      bgColor_long = bg_colors[STATUS_OVERRIDE];
  }
  else if (car_control.getLongActive()) {
      bgColor_long = bg_colors[STATUS_ENGAGED];
  }
  else
      bgColor_long = bg_colors[STATUS_DISENGAGED];
  if (bg != bgColor || bg_long != bgColor_long) {
    // repaint border
    bg = bgColor;
    bg_long = bgColor_long;
    update();
  }
  else {
      updateStateText();
  }
}

void OnroadWindow::offroadTransition(bool offroad) {
  alerts->clear();
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  //p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
  QRect upperRect(0, 0, width(), height() / 2);
  p.fillRect(upperRect, QColor(bg.red(), bg.green(), bg.blue(), 255));

  QRect lowerRect(0, height() / 2, width(), height() / 2);
  p.fillRect(lowerRect, QColor(bg_long.red(), bg_long.green(), bg_long.blue(), 255));

}
void OnroadWindow::updateStateText() {
    //QPainter p(this);
    //QColor text_color = QColor(0, 0, 0, 0xff);
    //QColor text_color = QColor(0xff, 0xff, 0xff, 0xff);
    //QRect rect_top(0, 0, rect().width(), 29);
    //QRect rect_bottom(0, rect().height() - UI_BORDER_SIZE - 1, rect().width(), 29);

    //p.setFont(InterFont(28, QFont::DemiBold));
    //p.setPen(text_color);

    UIState* s = uiState();
    const SubMaster& sm = *(s->sm);
    //auto meta = sm["modelV2"].getModelV2().getMeta();
    //QString debugModelV2 = QString::fromStdString(meta.getDebugText().cStr());
    //auto controls_state = sm["controlsState"].getControlsState();
    //QString debugControlsState = QString::fromStdString(controls_state.getDebugText1().cStr());
    //const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    //QString debugLong2 = QString::fromStdString(lp.getDebugLongText2().cStr());
    //const auto live_params = sm["liveParameters"].getLiveParameters();
    //float   liveSteerRatio = live_params.getSteerRatio();

    auto car_state = sm["carState"].getCarState();
    
    QString top = QString::fromStdString(car_state.getLogCarrot().cStr());

    //if (debugControlsState.length() > 2) {
        //top = debugControlsState;
    //}
    //else if (debugModelV2.length() > 2) {
        //top = debugModelV2;
    //}
    //else top = QString::fromStdString(lp.getDebugLongText().cStr()) + (" LiveSR:" + QString::number(liveSteerRatio, 'f', 2));
    //p.drawText(rect_top, Qt::AlignBottom | Qt::AlignHCenter, top);
    topLabel->setText(top);

    //extern int g_fps;
    //const auto cp = sm["carParams"].getCarParams();
    //top.sprintf("%s Long, FPS: %d", hasLongitudinalControl(cp)?"OP":"Stock", g_fps);
    auto deviceState = sm["deviceState"].getDeviceState();
    const auto freeSpace = deviceState.getFreeSpacePercent();
    const auto memoryUsage = deviceState.getMemoryUsagePercent();
    const auto cpuTempC = deviceState.getCpuTempC();
    float cpuTemp = 0.0f;
    int   size = sizeof(cpuTempC) / sizeof(cpuTempC[0]);
    if (size > 0) {
        for (int i = 0; i < size; i++) {
            cpuTemp += cpuTempC[i];
        }
        cpuTemp /= static_cast<float>(size);
    }

    //top.sprintf("MEM: %d%% DISK: %.0f%% CPU: %.0f°C", memoryUsage, freeSpace, cpuTemp);
    //top.sprintf("MEM: %d%% DISK: %.0f%% CPU: %.0f\xC2\xB0C", memoryUsage, freeSpace, cpuTemp);
    //top.sprintf("MEM: %d%% DISK: %.0f%% CPU: %.0f°C", memoryUsage, freeSpace, cpuTemp);
    top.sprintf("MEM: %d%% DISK: %.0f%% CPU: %.0f\u00B0C", memoryUsage, freeSpace, cpuTemp);

    topRightLabel->setText(top);

    Params params = Params();
    QString carName = QString::fromStdString(params.get("CarName"));
    topLeftLabel->setText(carName);

    //const auto lat_plan = sm["lateralPlan"].getLateralPlan();
    //bottomLabel->setText(lat_plan.getLatDebugText().cStr());

    Params params_memory = Params("/dev/shm/params");
    QString ipAddress = QString::fromStdString(params.get("NetworkAddress"));
    //extern QString gitBranch;
    bottomRightLabel->setText(ipAddress);
    //bottomLeftLabel->setText(gitBranch);

#if 0
    QString navi = QString::fromStdString(params_memory.get("CarrotNavi"));
    QJsonDocument doc = QJsonDocument::fromJson(navi.toUtf8());
    if (doc.isObject()) {
        QJsonObject jsonObject = doc.object();
        QString str = QString("%8, %1Km/h, TBT(%2): %3M, CAM(%4): %5km/h, %6M, %7")
            .arg(jsonObject["desiredSpeed"].toInt())
            .arg(jsonObject["xTurnInfo"].toInt())
            .arg(jsonObject["xDistToTurn"].toInt())
            .arg(jsonObject["xSpdType"].toInt())
            .arg(jsonObject["xSpdLimit"].toInt())
            .arg(jsonObject["xSpdDist"].toInt())
            .arg(jsonObject["szPosRoadName"].toString())
            .arg(jsonObject["active"].toBool());
        topLabel->setText(str);
        //printf("%s\n", str.toStdString().c_str());
    }
#endif
}
