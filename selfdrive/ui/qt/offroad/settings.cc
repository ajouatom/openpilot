#include <algorithm>
#include <cassert>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>
#include <thread> //차선캘리

#include <QDebug>
#include <QProcess>
#include <QScrollArea>
#include <QScroller>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>
#include <QPainterPath>

#include "common/watchdog.h"
#include "common/util.h"
#include "selfdrive/ui/qt/network/networking.h"
#include "selfdrive/ui/qt/offroad/settings.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/prime.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/offroad/developer_panel.h"
#include "selfdrive/ui/qt/offroad/firehose.h"

TogglesPanel::TogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  // param, title, desc, icon
  std::vector<std::tuple<QString, QString, QString, QString>> toggle_defs{
    {
      "OpenpilotEnabledToggle",
      tr("Enable openpilot"),
      tr("Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off."),
      "../assets/img_chffr_wheel.png",
    },
    {
      "ExperimentalMode",
      tr("Experimental Mode"),
      "",
      "../assets/img_experimental_white.svg",
    },
    {
      "DisengageOnAccelerator",
      tr("Disengage on Accelerator Pedal"),
      tr("When enabled, pressing the accelerator pedal will disengage openpilot."),
      "../assets/offroad/icon_disengage_on_accelerator.svg",
    },
    {
      "IsLdwEnabled",
      tr("Enable Lane Departure Warnings"),
      tr("Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31 mph (50 km/h)."),
      "../assets/offroad/icon_warning.png",
    },
    {
      "AlwaysOnDM",
      tr("Always-On Driver Monitoring"),
      tr("Enable driver monitoring even when openpilot is not engaged."),
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "RecordFront",
      tr("Record and Upload Driver Camera"),
      tr("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "RecordAudio",
      tr("Record and Upload Microphone Audio"),
      tr("Record and store microphone audio while driving. The audio will be included in the dashcam video in comma connect."),
      "../assets/offroad/microphone.png",
    },
    {
      "IsMetric",
      tr("Use Metric System"),
      tr("Display speed in km/h instead of mph."),
      "../assets/offroad/icon_metric.png",
    },
  };


  std::vector<QString> longi_button_texts{tr("Aggressive"), tr("Standard"), tr("Relaxed") , tr("MoreRelaxed") };
  long_personality_setting = new ButtonParamControl("LongitudinalPersonality", tr("Driving Personality"),
                                          tr("Standard is recommended. In aggressive mode, openpilot will follow lead cars closer and be more aggressive with the gas and brake. "
                                             "In relaxed mode openpilot will stay further away from lead cars. On supported cars, you can cycle through these personalities with "
                                             "your steering wheel distance button."),
                                          "../assets/offroad/icon_speed_limit.png",
                                          longi_button_texts);

  // set up uiState update for personality setting
  QObject::connect(uiState(), &UIState::uiUpdate, this, &TogglesPanel::updateState);

  for (auto &[param, title, desc, icon] : toggle_defs) {
    auto toggle = new ParamControl(param, title, desc, icon, this);

    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);

    addItem(toggle);
    toggles[param.toStdString()] = toggle;

    // insert longitudinal personality after NDOG toggle
    if (param == "DisengageOnAccelerator") {
      addItem(long_personality_setting);
    }
  }

  // Toggles with confirmation dialogs
  toggles["ExperimentalMode"]->setActiveIcon("../assets/img_experimental.svg");
  toggles["ExperimentalMode"]->setConfirmation(true, true);
}

void TogglesPanel::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  if (sm.updated("selfdriveState")) {
    auto personality = sm["selfdriveState"].getSelfdriveState().getPersonality();
    if (personality != s.scene.personality && s.scene.started && isVisible()) {
      long_personality_setting->setCheckedButton(static_cast<int>(personality));
    }
    uiState()->scene.personality = personality;
  }
}

void TogglesPanel::expandToggleDescription(const QString &param) {
  toggles[param.toStdString()]->showDescription();
}

void TogglesPanel::showEvent(QShowEvent *event) {
  updateToggles();
}

void TogglesPanel::updateToggles() {
  auto experimental_mode_toggle = toggles["ExperimentalMode"];
  const QString e2e_description = QString("%1<br>"
                                          "<h4>%2</h4><br>"
                                          "%3<br>"
                                          "<h4>%4</h4><br>"
                                          "%5<br>")
                                  .arg(tr("openpilot defaults to driving in <b>chill mode</b>. Experimental mode enables <b>alpha-level features</b> that aren't ready for chill mode. Experimental features are listed below:"))
                                  .arg(tr("End-to-End Longitudinal Control"))
                                  .arg(tr("Let the driving model control the gas and brakes. openpilot will drive as it thinks a human would, including stopping for red lights and stop signs. "
                                          "Since the driving model decides the speed to drive, the set speed will only act as an upper bound. This is an alpha quality feature; "
                                          "mistakes should be expected."))
                                  .arg(tr("New Driving Visualization"))
                                  .arg(tr("The driving visualization will transition to the road-facing wide-angle camera at low speeds to better show some turns. The Experimental mode logo will also be shown in the top right corner."));

  const bool is_release = params.getBool("IsReleaseBranch");
  auto cp_bytes = params.get("CarParamsPersistent");
  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    if (hasLongitudinalControl(CP)) {
      // normal description and toggle
      experimental_mode_toggle->setEnabled(true);
      experimental_mode_toggle->setDescription(e2e_description);
      long_personality_setting->setEnabled(true);
    } else {
      // no long for now
      experimental_mode_toggle->setEnabled(false);
      long_personality_setting->setEnabled(false);
      params.remove("ExperimentalMode");

      const QString unavailable = tr("Experimental mode is currently unavailable on this car since the car's stock ACC is used for longitudinal control.");

      QString long_desc = unavailable + " " + \
                          tr("openpilot longitudinal control may come in a future update.");
      if (CP.getAlphaLongitudinalAvailable()) {
        if (is_release) {
          long_desc = unavailable + " " + tr("An alpha version of openpilot longitudinal control can be tested, along with Experimental mode, on non-release branches.");
        } else {
          long_desc = tr("Enable the openpilot longitudinal control (alpha) toggle to allow Experimental mode.");
        }
      }
      experimental_mode_toggle->setDescription("<b>" + long_desc + "</b><br><br>" + e2e_description);
    }

    experimental_mode_toggle->refresh();
  } else {
    experimental_mode_toggle->setDescription(e2e_description);
  }
}

DevicePanel::DevicePanel(SettingsWindow *parent) : ListWidget(parent) {
  setSpacing(50);
  addItem(new LabelControl(tr("Dongle ID"), getDongleId().value_or(tr("N/A"))));
  addItem(new LabelControl(tr("Serial"), params.get("HardwareSerial").c_str()));

  // power buttons
  QHBoxLayout* power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton* reboot_btn = new QPushButton(tr("Reboot"));
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, this, &DevicePanel::reboot);
  //차선캘리
  QPushButton *reset_CalibBtn = new QPushButton(tr("ReCalibration"));
  reset_CalibBtn->setObjectName("reset_CalibBtn");
  power_layout->addWidget(reset_CalibBtn);
  QObject::connect(reset_CalibBtn, &QPushButton::clicked, this, &DevicePanel::calibration);

  QPushButton* poweroff_btn = new QPushButton(tr("Power Off"));
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);

  if (false && !Hardware::PC()) {
      connect(uiState(), &UIState::offroadTransition, poweroff_btn, &QPushButton::setVisible);
  }

  addItem(power_layout);

  QHBoxLayout* init_layout = new QHBoxLayout();
  init_layout->setSpacing(30);

  QPushButton* init_btn = new QPushButton(tr("Git Pull & Reboot"));
  init_btn->setObjectName("init_btn");
  init_layout->addWidget(init_btn);
  //QObject::connect(init_btn, &QPushButton::clicked, this, &DevicePanel::reboot);
  QObject::connect(init_btn, &QPushButton::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Git pull & Reboot?"), tr("Yes"), this)) {
      QString pullscript = "cd /data/openpilot && "
        "git fetch origin && "
        "LOCAL=$(git rev-parse HEAD) && "
        "BRANCH=$(git branch --show-current) && "
        "REMOTE=$(git rev-parse origin/$BRANCH) && "
        "if [ $LOCAL != $REMOTE ]; then "
        "echo 'Local is behind. Pulling updates...' && "
        "git pull --ff-only && "
        "sudo reboot; "
        "else "
        "echo 'Already up to date.'; "
        "fi'";

      bool success = QProcess::startDetached("/bin/sh", QStringList() << "-c" << pullscript);

      if (!success) {
        ConfirmationDialog::alert(tr("Failed to start update process."), this);
      } else {
        ConfirmationDialog::alert(tr("Update process started. Device will reboot if updates are applied."), this);
      }
    }
    });

  QPushButton* default_btn = new QPushButton(tr("Set default"));
  default_btn->setObjectName("default_btn");
  init_layout->addWidget(default_btn);
  //QObject::connect(default_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);
  QObject::connect(default_btn, &QPushButton::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Set to default?"), tr("Yes"), this)) {
      //emit parent->closeSettings();
      QTimer::singleShot(1000, []() {
        printf("Set to default\n");
        Params().putInt("SoftRestartTriggered", 2);
        printf("Set to default2\n");
        });
    }
    });

  QPushButton* remove_mapbox_key_btn = new QPushButton(tr("Remove MapboxKey"));
  remove_mapbox_key_btn->setObjectName("remove_mapbox_key_btn");
  init_layout->addWidget(remove_mapbox_key_btn);
  QObject::connect(remove_mapbox_key_btn, &QPushButton::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Remove Mapbox key?"), tr("Yes"), this)) {
      QTimer::singleShot(1000, []() {
        Params().put("MapboxPublicKey", "");
        Params().put("MapboxSecretKey", "");
        });
    }
    });

  setStyleSheet(R"(
    #reboot_btn { height: 120px; border-radius: 15px; background-color: #2CE22C; }
    #reboot_btn:pressed { background-color: #24FF24; }
    #reset_CalibBtn { height: 120px; border-radius: 15px; background-color: #FFBB00; }
    #reset_CalibBtn:pressed { background-color: #FF2424; }
    #poweroff_btn { height: 120px; border-radius: 15px; background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
    #init_btn { height: 120px; border-radius: 15px; background-color: #2C2CE2; }
    #init_btn:pressed { background-color: #2424FF; }
    #default_btn { height: 120px; border-radius: 15px; background-color: #BDBDBD; }
    #default_btn:pressed { background-color: #A9A9A9; }
    #remove_mapbox_key_btn { height: 120px; border-radius: 15px; background-color: #BDBDBD; }
    #remove_mapbox_key_btn:pressed { background-color: #A9A9A9; }
  )");
  addItem(init_layout);

  pair_device = new ButtonControl(tr("Pair Device"), tr("PAIR"),
                                  tr("Pair your device with comma connect (connect.comma.ai) and claim your comma prime offer."));
  connect(pair_device, &ButtonControl::clicked, [=]() {
    PairingPopup popup(this);
    popup.exec();
  });
  addItem(pair_device);

  // offroad-only buttons

  auto dcamBtn = new ButtonControl(tr("Driver Camera"), tr("PREVIEW"),
                                   tr("Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"));
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });
  addItem(dcamBtn);

  auto retrainingBtn = new ButtonControl(tr("Review Training Guide"), tr("REVIEW"), tr("Review the rules, features, and limitations of openpilot"));
  connect(retrainingBtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to review the training guide?"), tr("Review"), this)) {
      emit reviewTrainingGuide();
    }
  });
  addItem(retrainingBtn);

  auto statusCalibBtn = new ButtonControl(tr("Calibration Status"), tr("SHOW"), "");
  connect(statusCalibBtn, &ButtonControl::showDescriptionEvent, this, &DevicePanel::updateCalibDescription);
  addItem(statusCalibBtn);

  std::string calib_bytes = params.get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != cereal::LiveCalibrationData::Status::UNCALIBRATED) {
        double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
        double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
        QString position = QString("%2 %1° %4 %3°")
                           .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? "↓" : "↑",
                                QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? "←" : "→");
        params.put("DevicePosition", position.toStdString());
      }
    } catch (kj::Exception) {
      qInfo() << "invalid CalibrationParams";
    }
  }

  if (Hardware::TICI()) {
    auto regulatoryBtn = new ButtonControl(tr("Regulatory"), tr("VIEW"), "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file("../assets/offroad/fcc.html");
      ConfirmationDialog::rich(QString::fromStdString(txt), this);
    });
    addItem(regulatoryBtn);
  }

  auto translateBtn = new ButtonControl(tr("Change Language"), tr("CHANGE"), "");
  connect(translateBtn, &ButtonControl::clicked, [=]() {
    QMap<QString, QString> langs = getSupportedLanguages();
    QString selection = MultiOptionDialog::getSelection(tr("Select a language"), langs.keys(), langs.key(uiState()->language), this);
    if (!selection.isEmpty()) {
      // put language setting, exit Qt UI, and trigger fast restart
      params.put("LanguageSetting", langs[selection].toStdString());
      qApp->exit(18);
      watchdog_kick(0);
    }
  });
  addItem(translateBtn);

  QObject::connect(uiState()->prime_state, &PrimeState::changed, [this] (PrimeState::Type type) {
    pair_device->setVisible(type == PrimeState::PRIME_TYPE_UNPAIRED);
  });
  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    for (auto btn : findChildren<ButtonControl *>()) {
      if (btn != pair_device) {
        btn->setEnabled(offroad);
      }
    }
    translateBtn->setEnabled(true);
    statusCalibBtn->setEnabled(true);
  });

}

void DevicePanel::updateCalibDescription() {
  QString desc =
      tr("openpilot requires the device to be mounted within 4° left or right and "
         "within 5° up or 9° down. openpilot is continuously calibrating, resetting is rarely required.");
  std::string calib_bytes = params.get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != cereal::LiveCalibrationData::Status::UNCALIBRATED) {
        double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
        double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
        desc += tr(" Your device is pointed %1° %2 and %3° %4.")
                    .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? tr("down") : tr("up"),
                         QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? tr("left") : tr("right"));
      }
    } catch (kj::Exception) {
      qInfo() << "invalid CalibrationParams";
    }
  }
  qobject_cast<ButtonControl *>(sender())->setDescription(desc);
}

void DevicePanel::reboot() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reboot?"), tr("Reboot"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        params.putBool("DoReboot", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Reboot"), this);
  }
}

//차선캘리
void execAndReboot(const std::string& cmd) {
    system(cmd.c_str());
    Params().putBool("DoReboot", true);
}

void DevicePanel::calibration() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration?"), tr("ReCalibration"), this)) {
      if (!uiState()->engaged()) {
        std::thread worker(execAndReboot, "cd /data/params/d_tmp;  rm -f CalibrationParams");
        worker.detach();
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Reboot & Disengage to Calibration"), this);
  }
}

void DevicePanel::poweroff() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to power off?"), tr("Power Off"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        params.putBool("DoShutdown", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Power Off"), this);
  }
}

void SettingsWindow::showEvent(QShowEvent *event) {
  setCurrentPanel(0);
}

void SettingsWindow::setCurrentPanel(int index, const QString &param) {
  if (!param.isEmpty()) {
    // Check if param ends with "Panel" to determine if it's a panel name
    if (param.endsWith("Panel")) {
      QString panelName = param;
      panelName.chop(5); // Remove "Panel" suffix

      // Find the panel by name
      for (int i = 0; i < nav_btns->buttons().size(); i++) {
        if (nav_btns->buttons()[i]->text() == tr(panelName.toStdString().c_str())) {
          index = i;
          break;
        }
      }
    } else {
      emit expandToggleDescription(param);
    }
  }

  panel_widget->setCurrentIndex(index);
  nav_btns->buttons()[index]->setChecked(true);
}

AutoTunerGraphWidget::AutoTunerGraphWidget(QWidget *parent) : QWidget(parent) {
  setAttribute(Qt::WA_OpaquePaintEvent, false);
}

void AutoTunerGraphWidget::setData(const QList<QString> &ts, const QMap<QString, QList<double>> &histories, const QMap<QString, QColor> &cols) {
  timestamps = ts;
  param_histories = histories;
  colors = cols;
  selected_index = -1;
  update();
}

void AutoTunerGraphWidget::setSelectedParam(const QString &param) {
  selected_param = param;
  update();
}

void AutoTunerGraphWidget::setHiddenParams(const QSet<QString> &params) {
  hidden_params = params;
  update();
}

void AutoTunerGraphWidget::mousePressEvent(QMouseEvent *event) {
  if (timestamps.isEmpty()) return;
  
  int margin_left = 80;
  int margin_right = 40;
  QRect graph_rect = rect().adjusted(margin_left, 80, -margin_right, -40);
  int steps_x = timestamps.size() - 1;
  if (steps_x < 1) steps_x = 1;

  int click_x = event->x();
  int closest_idx = 0;
  int min_dist = 999999;
  
  for (int i = 0; i < timestamps.size(); i++) {
    int node_x = graph_rect.left() + i * graph_rect.width() / steps_x;
    int dist = std::abs(click_x - node_x);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  if (min_dist < 60) {
    selected_index = closest_idx;
  } else {
    selected_index = -1;
  }
  update();
}

void AutoTunerGraphWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Background
  painter.fillRect(rect(), QColor("#1f1f1f"));

  if (timestamps.isEmpty() || param_histories.isEmpty()) {
    painter.setPen(QColor("#888888"));
    painter.setFont(QFont("Arial", 40));
    painter.drawText(rect(), Qt::AlignCenter, tr("No historical data to display"));
    return;
  }

  // Margin - bottom margin reduced to 40 since overlapping labels are removed
  int margin_left = 80;
  int margin_right = 40;
  int margin_top = 80;
  int margin_bottom = 40;

  QRect graph_rect = rect().adjusted(margin_left, margin_top, -margin_right, -margin_bottom);

  int steps_x = timestamps.size() - 1;
  if (steps_x < 1) steps_x = 1;
  
  // Draw Grid Lines (Vertical & Horizontal)
  painter.setPen(QPen(QColor("#2d2d2d"), 2, Qt::SolidLine));
  
  // X grid lines
  for (int i = 0; i <= steps_x; i++) {
    int x = graph_rect.left() + i * graph_rect.width() / steps_x;
    painter.drawLine(x, graph_rect.top(), x, graph_rect.bottom());
  }

  // Y grid lines
  int steps_y = 4;
  for (int i = 0; i <= steps_y; i++) {
    int y = graph_rect.top() + i * graph_rect.height() / steps_y;
    painter.drawLine(graph_rect.left(), y, graph_rect.right(), y);
  }

  // 큰 수치(예: StopDistanceCarrot, 500↑) 파라미터는 작은 수치(≤200) 파라미터와
  // 스케일 차이가 커서 전체보기 가독성을 해친다. 따라서 '큰 수치' 파라미터는
  // 좌측에서 직접 선택(상세보기)했을 때만 그래프에 표시하고, 그 외(전체보기 포함)
  // 에서는 축 범위 계산과 그리기 모두에서 제외한다.
  const double LARGE_SCALE_THRESHOLD = 300.0;
  auto isLargeScale = [&](const QString &param) {
    for (double v : param_histories[param]) {
      if (std::abs(v) > LARGE_SCALE_THRESHOLD) return true;
    }
    return false;
  };
  auto excludedFromView = [&](const QString &param) {
    // 그룹이 접혀(숨김) 있는 파라미터는 그래프에서 제외
    if (hidden_params.contains(param)) return true;
    // 큰 수치 파라미터는 자신이 선택된 경우에만 표시
    return isLargeScale(param) && param != selected_param;
  };

  double global_min = 0.0;
  double global_max = 0.0;
  bool first_val = true;

  // Compute global min/max bounds (large-scale params excluded unless selected)
  for (const QString &param : param_histories.keys()) {
    QList<double> values = param_histories[param];
    if (values.size() != timestamps.size()) continue;
    if (excludedFromView(param)) continue;
    for (double val : values) {
      if (first_val) {
        global_min = val;
        global_max = val;
        first_val = false;
      } else {
        if (val < global_min) global_min = val;
        if (val > global_max) global_max = val;
      }
    }
  }

  // Draw Line Paths
  painter.setBrush(Qt::NoBrush);
  for (const QString &param : param_histories.keys()) {
    QList<double> values = param_histories[param];
    if (values.size() != timestamps.size()) continue;
    if (excludedFromView(param)) continue;  // 큰 수치 파라미터는 선택 시에만 그림

    // Always use global min/max bounds to maintain consistent scaling across all variables
    double min_val = global_min;
    double max_val = global_max;
    double diff = max_val - min_val;

    bool is_highlighted = selected_param.isEmpty() || (selected_param == param);
    int opacity = 255;
    int line_width = 4;
    QColor color;

    if (!selected_param.isEmpty()) {
      if (selected_param == param) {
        color = colors.value(param, QColor(Qt::white));
        opacity = 255;
        line_width = 8;
      } else {
        color = QColor("#444444"); // Dark gray for non-selected parameters
        opacity = 80;
        line_width = 2;
      }
    } else {
      color = colors.value(param, QColor(Qt::white));
      opacity = 255;
      line_width = 4;
    }
    color.setAlpha(opacity);

    bool dimmed = (!selected_param.isEmpty() && selected_param != param);

    // 노드 좌표 미리 계산
    QList<QPoint> pts;
    for (int i = 0; i < values.size(); i++) {
      int x = graph_rect.left() + i * graph_rect.width() / steps_x;
      int y;
      if (diff < 1e-5) {
        y = graph_rect.top() + graph_rect.height() / 2;
      } else {
        y = graph_rect.bottom() - (values[i] - min_val) / diff * graph_rect.height();
      }
      pts.append(QPoint(x, y));
    }

    painter.setBrush(Qt::NoBrush);
    if (dimmed) {
      // 비선택 파라미터: 전체 점선으로 흐리게 표시
      QPen pen(color, line_width);
      pen.setStyle(Qt::DotLine);
      painter.setPen(pen);
      QPainterPath path;
      for (int i = 0; i < pts.size(); i++) {
        if (i == 0) path.moveTo(pts[i]);
        else path.lineTo(pts[i]);
      }
      painter.drawPath(path);
    } else {
      // 강조 표시: 이전 값과 달라진 구간은 실선, 동일한 구간은 점선(가는선)으로
      // 자연스럽게 이어지도록 세그먼트 단위로 그린다.
      // 전체보기(선택 없음)에서는 점선이 촘촘해 거의 실선처럼 보이므로
      // 실선은 조금 더 굵게, 점선은 간격을 더 넓게 한다.
      // 상세보기(특정 파라미터 선택)는 현재 굵기/간격이 적당하므로 그대로 둔다.
      bool all_view = selected_param.isEmpty();
      int solid_width = all_view ? (line_width + 2) : line_width;
      int dot_width = std::max(1, line_width / 2);  // 점선은 더 가늘게
      for (int i = 1; i < pts.size(); i++) {
        bool changed = qAbs(values[i] - values[i - 1]) > 1e-6;
        if (changed) {
          QPen segpen(color, solid_width);
          segpen.setStyle(Qt::SolidLine);
          painter.setPen(segpen);
        } else {
          QPen segpen(color, dot_width);
          if (all_view) {
            segpen.setStyle(Qt::CustomDashLine);
            segpen.setDashPattern({1, 4});  // 점(1) : 간격(4) — 기본 DotLine보다 넓게
          } else {
            segpen.setStyle(Qt::DotLine);
          }
          painter.setPen(segpen);
        }
        painter.drawLine(pts[i - 1], pts[i]);
      }
    }

    // Draw Nodes and Value Labels
    for (int i = 0; i < pts.size(); i++) {
      int x = pts[i].x();
      int y = pts[i].y();

      // 변경된 지점에만 노드(점)를 표시 (시작점 포함). 동일 구간은 점선만 이어짐.
      bool changed = (i == 0) || qAbs(values[i] - values[i - 1]) > 1e-6;

      if (changed) {
        painter.setBrush(color);
        painter.setPen(Qt::NoPen);
        int dot_size = (selected_param == param) ? 16 : 10;
        painter.drawEllipse(QPoint(x, y), dot_size / 2, dot_size / 2);
      }

      // 수치는 시작점(i==0) 또는 이전 값과 달라진 경우만 표시
      // 여러 파라미터가 겹쳐 그려져 가독성이 떨어지므로 폰트를 작게 유지하고,
      // 폰트 색상은 해당 파라미터 선 색상과 동일하게 한다.
      if (is_highlighted && changed) {
        QColor label_color = colors.value(param, QColor(Qt::white));
        painter.setPen(label_color);
        painter.setFont(QFont("Arial", (selected_param == param) ? 22 : 16, QFont::Bold));
        QString val_str = QString::number(values[i], 'g', 4);
        // 선에 너무 붙지 않도록 약간 위로 올려 표시
        painter.drawText(QRect(x - 80, y - 46, 160, 28), Qt::AlignCenter, val_str);
      }
    }
  }

  // Draw Vertical Guide Line and Time Tooltip on touch/click
  if (selected_index >= 0 && selected_index < timestamps.size()) {
    int x = graph_rect.left() + selected_index * graph_rect.width() / steps_x;
    
    // Vertical Guide
    painter.setPen(QPen(QColor("#ffaa00"), 2.5, Qt::DashLine));
    painter.drawLine(x, graph_rect.top(), x, graph_rect.bottom());

    // Tooltip Background & Text
    QString date_str = timestamps[selected_index];
    painter.setFont(QFont("Arial", 28, QFont::Bold));
    
    QFontMetrics fm = painter.fontMetrics();
    int txt_w = fm.horizontalAdvance(date_str) + 30;
    int txt_h = 50;

    QRect tooltip_rect(x - txt_w / 2, margin_top - 65, txt_w, txt_h);
    
    // Boundary check
    if (tooltip_rect.left() < 10) tooltip_rect.moveLeft(10);
    if (tooltip_rect.right() > width() - 10) tooltip_rect.moveRight(width() - 10);

    painter.setBrush(QColor("#2d2d2d"));
    painter.setPen(QPen(QColor("#ffaa00"), 2));
    painter.drawRoundedRect(tooltip_rect, 10, 10);

    painter.setPen(QColor("#ffffff"));
    painter.drawText(tooltip_rect, Qt::AlignCenter, date_str);
  }
}

AutoTunerHistoryPanel::AutoTunerHistoryPanel(QWidget* parent) : QFrame(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setContentsMargins(20, 20, 20, 20);
  main_layout->setSpacing(20);

  // Left Column: Parameters List
  QVBoxLayout *left_layout = new QVBoxLayout();
  left_layout->setSpacing(15);

  QLabel *list_title = new QLabel(tr("Parameters"));
  list_title->setStyleSheet("font-size: 42px; font-weight: bold; color: white;");
  left_layout->addWidget(list_title);

  QScrollArea *scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setFixedWidth(340);
  scroll->setStyleSheet("QScrollArea { background: transparent; } QWidget { background: transparent; }");
  QScroller::grabGesture(scroll->viewport(), QScroller::LeftMouseButtonGesture);

  QWidget *scroll_widget = new QWidget();
  param_list_layout = new QVBoxLayout(scroll_widget);
  param_list_layout->setContentsMargins(0, 0, 0, 0);
  param_list_layout->setSpacing(8);
  scroll->setWidget(scroll_widget);
  left_layout->addWidget(scroll, 1);

  QVBoxLayout *toggles_layout = new QVBoxLayout();
  toggles_layout->setSpacing(10);
  toggles_layout->setContentsMargins(0, 10, 0, 0);

  // 터치 면적을 키우기 위해 가로로 길던 버튼을 정사각형에 가깝게 바꾸고
  // 두 개를 나란히(좌우) 배치한다. 글씨는 3줄로 표시.
  QPushButton *lat_toggle = new QPushButton(this);
  lat_toggle->setFixedHeight(150);

  QPushButton *long_toggle = new QPushButton(this);
  long_toggle->setFixedHeight(150);

  auto updateToggles = [=]() {
    bool apply_lat = Params().getBool("CarrotTunerApplyLat");
    bool apply_long = Params().getBool("CarrotTunerApplyLong");
    // 활성(ON)=초록, 비활성(OFF)=회색. 3줄 텍스트가 박스를 넘지 않도록 폰트 축소.
    QString on_style = "background-color: #178644; font-size: 22px; font-weight: bold; border-radius: 10px; color: white;";
    QString off_style = "background-color: #4a5568; font-size: 22px; font-weight: bold; border-radius: 10px; color: white;";
    lat_toggle->setText(tr("Apply LAT\n(Steering)\n%1").arg(apply_lat ? "ON" : "OFF"));
    lat_toggle->setStyleSheet(apply_lat ? on_style : off_style);
    long_toggle->setText(tr("Apply LONG\n(Accel/Brake)\n%1").arg(apply_long ? "ON" : "OFF"));
    long_toggle->setStyleSheet(apply_long ? on_style : off_style);
  };

  updateToggles();

  connect(lat_toggle, &QPushButton::clicked, this, [=]() {
    bool current = Params().getBool("CarrotTunerApplyLat");
    Params().putBool("CarrotTunerApplyLat", !current);
    updateToggles();
  });

  connect(long_toggle, &QPushButton::clicked, this, [=]() {
    bool current = Params().getBool("CarrotTunerApplyLong");
    Params().putBool("CarrotTunerApplyLong", !current);
    updateToggles();
  });

  // 두 토글을 좌우로 나란히 배치 (각각 정사각형에 가까운 형태)
  QHBoxLayout *apply_row = new QHBoxLayout();
  apply_row->setSpacing(10);
  apply_row->addWidget(lat_toggle);
  apply_row->addWidget(long_toggle);
  toggles_layout->addLayout(apply_row);

  // 공장초기화: 좌측 하단(Apply LONG 아래)에 배치. 모든 튜너 파라미터를
  // params_keys.h 기본값으로 복원하고 학습 데이터/이력을 삭제한다.
  QPushButton *factoryResetBtn = new QPushButton(tr("Parameter Init. Reset"), this);
  factoryResetBtn->setFixedHeight(75);
  factoryResetBtn->setStyleSheet("background-color: #8a1d1d; font-size: 26px; font-weight: bold; border-radius: 10px; color: white;");
  connect(factoryResetBtn, &QPushButton::clicked, this, [=]() {
    if (ConfirmationDialog::confirm(
          tr("Reset all auto-tuned parameters to factory defaults and delete learning data/history?"),
          tr("Parameter Init. Reset"), this)) {
      Params p;
      static const std::vector<std::string> tunerKeys = {
        "CruiseMaxVals0", "CruiseMaxVals1", "CruiseMaxVals2", "CruiseMaxVals3",
        "CruiseMaxVals4", "CruiseMaxVals5", "CruiseMaxVals6",
        "JLeadFactor3", "TFollowGap1", "TFollowGap2", "TFollowGap3", "TFollowGap4",
        "TFollowSpeedFactor", "DynamicTFollow", "TFollowDecelBoost",
        "PathOffset", "SteerActuatorDelay", "SteerRatioRate",
        "LateralTorqueAccelFactor", "LateralTorqueKf", "LateralTorqueFriction",
        "LateralTorqueKiV", "LateralTorqueKpV",
        "AutoCurveSpeedFactor", "AutoCurveSpeedAggressiveness",
        "StoppingAccel", "VEgoStopping", "StopDistanceCarrot",
        "LongTuningKf", "LongTuningKpV", "LongActuatorDelay",
      };
      for (const auto &key : tunerKeys) {
        auto def = p.getKeyDefaultValue(key);
        if (def.has_value()) p.put(key, *def);
      }
      p.remove("CarrotLearningData");
      p.remove("CarrotLearningRecommend");
      p.remove("CarrotLearningHistory");
      p.putBool("CarrotLearningPopupReady", false);
      p.putBool("CarrotTunerFactoryReset", true);
      ConfirmationDialog::alert(tr("Factory reset applied. Tuning parameters restored to defaults."), this);
    }
  });
  toggles_layout->addWidget(factoryResetBtn);
  left_layout->addLayout(toggles_layout);

  // Right Column: Chart + Controls
  QVBoxLayout *right_layout = new QVBoxLayout();
  right_layout->setSpacing(20);

  QHBoxLayout *header_layout = new QHBoxLayout();
  header_layout->addStretch();

  QPushButton *btn_card_list = new QPushButton(tr("View Card Type"));
  btn_card_list->setStyleSheet("background-color: #10b981; font-size: 40px; border-radius: 10px; color: white; font-weight: bold; padding: 0px 50px;");
  btn_card_list->setFixedHeight(110);
  connect(btn_card_list, &QPushButton::clicked, this, [=]() {
    AutoTunerCardListDialog dlg(this);
    dlg.exec();
  });
  header_layout->addWidget(btn_card_list);

  QPushButton *btn_all = new QPushButton(tr("Show All Parameters"));
  btn_all->setStyleSheet("background-color: #0ea5e9; font-size: 40px; border-radius: 10px; color: white; font-weight: bold; padding: 0px 50px;");
  btn_all->setFixedHeight(110);
  connect(btn_all, &QPushButton::clicked, this, [=]() {
    if (graph_widget) graph_widget->setSelectedParam("");
    selected_param = "";
    updateLabelColors();
  });
  header_layout->addWidget(btn_all);

  QPushButton *btn_clear = new QPushButton(tr("Clear All Logs"));
  btn_clear->setStyleSheet("background-color: #eab308; font-size: 40px; border-radius: 10px; color: white; font-weight: bold; padding: 0px 50px;");
  btn_clear->setFixedHeight(110);
  connect(btn_clear, &QPushButton::clicked, this, &AutoTunerHistoryPanel::clearAll);
  header_layout->addWidget(btn_clear);

  QPushButton *close_btn = new QPushButton(tr("Close"));
  close_btn->setStyleSheet("background-color: #bb3333; font-size: 40px; border-radius: 10px; color: white; font-weight: bold; padding: 0px 50px;");
  close_btn->setFixedHeight(110);
  connect(close_btn, &QPushButton::clicked, this, [=]() {
    QWidget* w = this->window();
    if (w) {
      QDialog* dlg = qobject_cast<QDialog*>(w);
      if (dlg) dlg->reject();
      else w->close();
    }
  });
  header_layout->addWidget(close_btn);

  right_layout->addLayout(header_layout);

  graph_widget = new AutoTunerGraphWidget(this);
  graph_widget->setMinimumHeight(750);
  right_layout->addWidget(graph_widget, 1);

  main_layout->addLayout(left_layout);
  main_layout->addLayout(right_layout, 1);

  // Palette initialization
  param_colors.clear();
  QList<QColor> palette = {
    QColor("#3b82f6"), // Blue (파랑)
    QColor("#10b981"), // Mint (민트)
    QColor("#fbbf24"), // Light Yellow (밝은 노랑)
    QColor("#8b5cf6"), // Violet (보라)
    QColor("#ec4899"), // Pink (분홍)
    QColor("#06b6d4"), // Cyan (시안/하늘)
    QColor("#84cc16"), // Lime (연두)
    QColor("#f43f5e"), // Rose (장미)
    QColor("#14b8a6"), // Teal (청록)
    QColor("#ffffff"), // White (흰색)
    QColor("#f97316"), // Orange (주황)
    QColor("#a855f7"), // Purple (자주)
    QColor("#60a5fa"), // Light Blue (연파랑)
    QColor("#34d399"), // Light Green (연초록)
    QColor("#e879f9")  // Light Magenta (연자주)
  };
  // Pre-seed common params to keep consistent colors
  param_colors["CruiseMaxVals0"] = QColor("#3b82f6"); // Blue
  param_colors["CruiseMaxVals1"] = QColor("#60a5fa"); // Light Blue
  param_colors["CruiseMaxVals2"] = QColor("#10b981"); // Mint Green
  param_colors["CruiseMaxVals3"] = QColor("#84cc16"); // Lime
  param_colors["CruiseMaxVals4"] = QColor("#fbbf24"); // Yellow
  param_colors["CruiseMaxVals5"] = QColor("#f97316"); // Orange
  param_colors["CruiseMaxVals6"] = QColor("#ec4899"); // Pink
  param_colors["JLeadFactor3"] = QColor("#8b5cf6");   // Violet
  param_colors["TFollowGap1"] = QColor("#06b6d4");    // Cyan
  param_colors["TFollowGap2"] = QColor("#14b8a6");    // Teal
  param_colors["TFollowGap3"] = QColor("#ffffff");    // White
  param_colors["TFollowGap4"] = QColor("#a855f7");    // Purple
  param_colors["PathOffset"] = QColor("#e879f9");     // Light Magenta
  param_colors["SteerActuatorDelay"] = QColor("#f43f5e"); // Rose
  param_colors["AutoCurveSpeedAggressiveness"] = QColor("#ff5722"); // Highlight curve learning with unique red-orange

  refreshHistory();
}

void AutoTunerHistoryPanel::showEvent(QShowEvent *event) {
  refreshHistory();
  QFrame::showEvent(event);
}

void AutoTunerHistoryPanel::refreshHistory() {
  // Clear parameter list layout
  QLayoutItem *child;
  while ((child = param_list_layout->takeAt(0)) != nullptr) {
    if (child->widget()) delete child->widget();
    delete child;
  }
  param_labels.clear();

  QString raw = QString::fromStdString(Params().get("CarrotLearningHistory"));
  if (raw.isEmpty()) {
    latest_id = "";
    if (graph_widget) {
      graph_widget->setData(QList<QString>(), QMap<QString, QList<double>>(), QMap<QString, QColor>());
    }
    return;
  }

  QJsonArray arr = QJsonDocument::fromJson(raw.toUtf8()).array();
  
  // Set latest entry info
  QJsonObject latest_item = arr[0].toObject();
  latest_id = latest_item["id"].toString();

  // Re-build historical timeline (max 30 points)
  int chart_limit = 50;
  int n_points = std::min(arr.size(), chart_limit);

  QList<QString> timestamps;
  QList<QJsonObject> entries;
  for (int i = n_points - 1; i >= 0; i--) {
    QJsonObject item = arr[i].toObject();
    timestamps.append(item["timestamp"].toString());
    entries.append(item);
  }

  // 1. Gather all parameters present in the timeline, remembering each
  //    parameter's group label (가속/조향/거리/주행 등)으로 좌측 리스트를 묶기 위함.
  QSet<QString> param_set;
  group_params.clear();
  for (const auto& entry : entries) {
    QJsonObject changes = entry["changes"].toObject();
    for (const QString& group : changes.keys()) {
      QJsonObject g_items = changes[group].toObject();
      for (const QString& key : g_items.keys()) {
        param_set.insert(key);
        if (!group_params[group].contains(key)) {
          group_params[group].append(key);
        }
      }
    }
  }

  // 그룹 표시 순서: 가속 → 조향 → 곡선 → 거리 → 주행 → (초기값 프로파일 등 기타)
  auto groupRank = [](const QString &g) -> int {
    if (g.contains("Acceleration") && !g.contains("Profile")) return 0;
    if (g.contains("Steering")) return 1;
    if (g.contains("Curve")) return 2;
    if (g.contains("Following Distance") && !g.contains("Profile")) return 3;
    if (g.contains("Driving")) return 4;
    return 10;  // 초기값 프로파일 및 기타 그룹은 뒤에
  };
  group_order = group_params.keys();
  std::stable_sort(group_order.begin(), group_order.end(),
                   [&](const QString &a, const QString &b) {
                     int ra = groupRank(a), rb = groupRank(b);
                     if (ra != rb) return ra < rb;
                     return a.compare(b, Qt::CaseInsensitive) < 0;
                   });

  // 2. Extrapolate timeline values for each parameter
  QMap<QString, QList<double>> param_histories;
  QMap<QString, double> last_values;

  for (int t = 0; t < n_points; t++) {
    QJsonObject changes = entries[t]["changes"].toObject();
    QMap<QString, double> current_changes;
    for (const QString& group : changes.keys()) {
      QJsonObject g_items = changes[group].toObject();
      for (const QString& key : g_items.keys()) {
        current_changes[key] = g_items[key].toObject()["recommended"].toDouble();
      }
    }

    for (const QString& param : param_set) {
      if (current_changes.contains(param)) {
        double val = current_changes[param];
        last_values[param] = val;
        param_histories[param].append(val);
      } else {
        if (last_values.contains(param)) {
          param_histories[param].append(last_values[param]);
        } else {
          // Find the first occurrence in the future to extract 'current' initial value
          double initial_val = 0.0;
          for (int future_t = t; future_t < n_points; future_t++) {
            QJsonObject f_changes = entries[future_t]["changes"].toObject();
            bool found = false;
            for (const QString& group : f_changes.keys()) {
              QJsonObject fg_items = f_changes[group].toObject();
              if (fg_items.contains(param)) {
                initial_val = fg_items[param].toObject()["current"].toDouble();
                found = true;
                break;
              }
            }
            if (found) break;
          }
          last_values[param] = initial_val;
          param_histories[param].append(initial_val);
        }
      }
    }
  }

  // Assign colors dynamically for new parameters
  QList<QColor> palette = {
    QColor("#3b82f6"), QColor("#10b981"), QColor("#f59e0b"), QColor("#8b5cf6"),
    QColor("#ec4899"), QColor("#06b6d4"), QColor("#84cc16"), QColor("#f43f5e"),
    QColor("#14b8a6"), QColor("#a855f7")
  };
  int color_idx = 0;
  for (const QString &param : param_set) {
    if (!param_colors.contains(param)) {
      param_colors[param] = palette[color_idx % palette.size()];
      color_idx++;
    }
  }

  // 각 그룹 내 파라미터는 알파벳순 정렬
  for (const QString &group : group_order) {
    group_params[group].sort(Qt::CaseInsensitive);
  }

  // 더 이상 존재하지 않는 그룹은 collapsed 상태에서 제거 (이력 변경 대응)
  for (const QString &g : collapsed_groups.values()) {
    if (!group_params.contains(g)) collapsed_groups.remove(g);
  }

  // 좌측 리스트를 그룹 단위로 렌더링
  rebuildParamList();

  if (graph_widget) {
    graph_widget->setData(timestamps, param_histories, param_colors);
  }

  // Ensure selected_param is still valid
  if (!param_set.contains(selected_param)) {
    selected_param = "";
    if (graph_widget) graph_widget->setSelectedParam("");
  }
  applyHiddenParams();
  updateLabelColors();
}

// 좌측 파라미터 리스트를 그룹 헤더 + (펼쳐진 경우) 소속 파라미터 버튼으로 다시 그린다.
// 그룹 헤더를 누르면 toggleGroup()으로 접기/펴기 + 그래프 표시가 토글된다.
void AutoTunerHistoryPanel::rebuildParamList() {
  // 기존 항목 제거
  QLayoutItem *child;
  while ((child = param_list_layout->takeAt(0)) != nullptr) {
    if (child->widget()) delete child->widget();
    delete child;
  }
  param_labels.clear();

  // 그룹 라벨에서 표시용 짧은 이름 추출: "거리 (Following Distance)" → "Following Distance"
  // 단, 이력 데이터의 그룹 키(=정렬/매칭 기준)는 그대로 두고 표시명만 바꾼다.
  auto groupShortName = [](const QString &group) -> QString {
    QString name;
    int open = group.indexOf('(');
    int close = group.lastIndexOf(')');
    if (open >= 0 && close > open) {
      name = group.mid(open + 1, close - open - 1).trimmed();
    } else {
      name = group.trimmed();
    }
    // 표시명만 "Following Distance" → "Following Gap" 으로 치환
    if (name == "Following Distance") name = "Following Gap";
    return name;
  };

  for (const QString &group : group_order) {
    bool collapsed = collapsed_groups.contains(group);

    // ── 그룹 헤더 (클릭 시 접기/펴기) ──
    QPushButton *header_btn = new QPushButton();
    // 헤더 바는 파라미터 항목(#252525)과 확실히 구분되도록 더 밝은 회색으로
    header_btn->setStyleSheet("text-align: left; padding: 0px 12px; border-radius: 10px; background-color: #5a5f6b; color: white; font-size: 26px;");
    header_btn->setFixedHeight(60);

    QHBoxLayout *header_layout = new QHBoxLayout(header_btn);
    header_layout->setContentsMargins(12, 0, 12, 0);
    header_layout->setSpacing(12);

    // 접힘/펼침 표시 화살표
    QLabel *arrow = new QLabel(collapsed ? "▸" : "▾");
    arrow->setStyleSheet(QString("color: %1; font-size: 26px; font-weight: bold; background: transparent;")
                             .arg(collapsed ? "#777777" : "#ffffff"));
    header_layout->addWidget(arrow);

    QLabel *header_lbl = new QLabel(groupShortName(group));
    header_lbl->setStyleSheet(QString("color: %1; font-size: 26px; font-weight: bold; background: transparent;")
                                  .arg(collapsed ? "#777777" : "#ffffff"));
    header_layout->addWidget(header_lbl, 1);

    connect(header_btn, &QPushButton::clicked, this, [=]() { toggleGroup(group); });
    param_list_layout->addWidget(header_btn);

    // ── 소속 파라미터들 (접혀있으면 그리지 않음) ──
    if (collapsed) continue;
    for (const QString &param : group_params[group]) {
      QColor color = param_colors.value(param, QColor(Qt::white));

      QPushButton *btn = new QPushButton();
      btn->setStyleSheet("text-align: left; padding: 0px 15px; border-radius: 10px; background-color: #252525; color: white; font-size: 28px;");
      btn->setFixedHeight(55);

      QHBoxLayout *btn_layout = new QHBoxLayout(btn);
      btn_layout->setContentsMargins(22, 0, 10, 0);  // 그룹 소속 표시를 위해 약간 들여쓰기
      btn_layout->setSpacing(15);

      QLabel *dot = new QLabel();
      dot->setFixedSize(20, 20);
      dot->setStyleSheet(QString("background-color: %1; border-radius: 10px;").arg(color.name()));
      btn_layout->addWidget(dot);

      QLabel *lbl = new QLabel(param);
      lbl->setStyleSheet("color: white; font-size: 28px; font-weight: bold; background: transparent;");
      btn_layout->addWidget(lbl, 1);
      param_labels[param] = lbl;

      connect(btn, &QPushButton::clicked, this, [=]() {
        if (graph_widget) graph_widget->setSelectedParam(param);
        selected_param = param;
        updateLabelColors();
      });

      param_list_layout->addWidget(btn);
    }
  }
  param_list_layout->addStretch();
}

// 접혀있는 그룹들의 소속 파라미터를 모아 그래프에서 숨긴다.
void AutoTunerHistoryPanel::applyHiddenParams() {
  QSet<QString> hidden;
  for (const QString &group : collapsed_groups.values()) {
    for (const QString &param : group_params.value(group)) {
      hidden.insert(param);
    }
  }
  if (graph_widget) graph_widget->setHiddenParams(hidden);
}

// 그룹 헤더 클릭 핸들러: 접기/펴기 상태를 토글하고 리스트·그래프를 갱신한다.
void AutoTunerHistoryPanel::toggleGroup(const QString &group) {
  if (collapsed_groups.contains(group)) {
    collapsed_groups.remove(group);
  } else {
    collapsed_groups.insert(group);
    // 접는 그룹에 현재 선택된 파라미터가 있으면 선택 해제 (숨겨지므로)
    if (!selected_param.isEmpty() && group_params.value(group).contains(selected_param)) {
      selected_param = "";
      if (graph_widget) graph_widget->setSelectedParam("");
    }
  }
  rebuildParamList();
  applyHiddenParams();
  updateLabelColors();
}

void AutoTunerHistoryPanel::restoreItem(const QString& id) {
  if (ConfirmationDialog::confirm(tr("Are you sure you want to restore the parameters to this state?"), tr("Restore"), this)) {
    QString raw = QString::fromStdString(Params().get("CarrotLearningHistory"));
    QJsonArray arr = QJsonDocument::fromJson(raw.toUtf8()).array();
    QJsonArray new_arr;
    
    for (int i = 0; i < arr.size(); i++) {
      QJsonObject entry = arr[i].toObject();
      if (entry["id"].toString() == id) {
        QJsonObject changes = entry["changes"].toObject();
        for (const QString& group : changes.keys()) {
          QJsonObject g_items = changes[group].toObject();
          for (const QString& key : g_items.keys()) {
            int prev_val = g_items[key].toObject()["current"].toInt();
            Params().putInt(key.toStdString(), prev_val);
          }
        }
      } else {
        new_arr.append(entry);
      }
    }
    
    if (new_arr.isEmpty()) {
      Params().remove("CarrotLearningHistory");
    } else {
      Params().put("CarrotLearningHistory", QJsonDocument(new_arr).toJson(QJsonDocument::Compact).toStdString());
    }
    refreshHistory();
    ConfirmationDialog::alert(tr("Restored to previous values successfully."), this);
  }
}

void AutoTunerHistoryPanel::deleteItem(const QString& id) {
  if (ConfirmationDialog::confirm(tr("Are you sure you want to delete this item?"), tr("Delete"), this)) {
    QString raw = QString::fromStdString(Params().get("CarrotLearningHistory"));
    QJsonArray arr = QJsonDocument::fromJson(raw.toUtf8()).array();
    QJsonArray new_arr;
    for (int i = 0; i < arr.size(); i++) {
      if (arr[i].toObject()["id"].toString() != id) {
        new_arr.append(arr[i]);
      }
    }
    if (new_arr.isEmpty()) {
      Params().remove("CarrotLearningHistory");
    } else {
      Params().put("CarrotLearningHistory", QJsonDocument(new_arr).toJson(QJsonDocument::Compact).toStdString());
    }
    refreshHistory();
  }
}

void AutoTunerHistoryPanel::clearAll() {
  if (ConfirmationDialog::confirm(tr("Are you sure you want to delete all history and restore parameters to their factory default values?"), tr("Clear All"), this)) {
    Params params;
    std::map<std::string, std::string> defaults = {
      {"CruiseMaxVals0", "160"},
      {"CruiseMaxVals1", "200"},
      {"CruiseMaxVals2", "160"},
      {"CruiseMaxVals3", "130"},
      {"CruiseMaxVals4", "110"},
      {"CruiseMaxVals5", "95"},
      {"CruiseMaxVals6", "80"},
      {"JLeadFactor3", "0"},
      {"TFollowGap1", "110"},
      {"TFollowGap2", "120"},
      {"TFollowGap3", "140"},
      {"TFollowGap4", "160"},
      {"DynamicTFollow", "0"},
      {"TFollowDecelBoost", "10"},
      {"PathOffset", "0"},
      {"SteerActuatorDelay", "0"}
    };
    for (const auto& [key, val] : defaults) {
      params.put(key, val);
    }
    params.remove("CarrotLearningHistory");
    refreshHistory();
  }
}

void AutoTunerHistoryPanel::updateLabelColors() {
  for (auto k : param_labels.keys()) {
    if (!param_labels[k]) continue;
    if (selected_param.isEmpty()) {
      param_labels[k]->setStyleSheet("color: white; font-size: 28px; font-weight: bold; background: transparent;");
    } else if (k == selected_param) {
      param_labels[k]->setStyleSheet("color: red; font-size: 28px; font-weight: bold; background: transparent;");
    } else {
      param_labels[k]->setStyleSheet("color: #777777; font-size: 28px; font-weight: bold; background: transparent;");
    }
  }
}

// AutoTunerHistoryDialog implementation
AutoTunerHistoryDialog::AutoTunerHistoryDialog(QWidget *parent) : DialogBase(parent) {
  QFrame *container = new QFrame(this);
  container->setStyleSheet("QFrame { background-color: #1B1B1B; border-radius: 20px; }");
  QVBoxLayout *main_layout = new QVBoxLayout(container);
  main_layout->setContentsMargins(20, 20, 20, 20);
  main_layout->setSpacing(20);

  AutoTunerHistoryPanel *panel = new AutoTunerHistoryPanel(this);
  main_layout->addWidget(panel, 1);

  QVBoxLayout *outer_layout = new QVBoxLayout(this);
  outer_layout->setContentsMargins(30, 30, 30, 30);
  outer_layout->addWidget(container);
}

// AutoTunerCardListDialog implementation
AutoTunerCardListDialog::AutoTunerCardListDialog(QWidget *parent) : DialogBase(parent) {
  QFrame *container = new QFrame(this);
  container->setStyleSheet("QFrame { background-color: #1B1B1B; border-radius: 20px; }");
  QVBoxLayout *main_layout = new QVBoxLayout(container);
  main_layout->setContentsMargins(50, 50, 50, 50);
  main_layout->setSpacing(30);

  // Header layout: Title and Close button
  QHBoxLayout *header_layout = new QHBoxLayout();
  QLabel *title = new QLabel(tr("Tuning History Card List"), this);
  title->setStyleSheet("font-size: 60px; font-weight: bold; color: white;");
  header_layout->addWidget(title);
  header_layout->addStretch();

  QPushButton *close_btn = new QPushButton(tr("Close"), this);
  close_btn->setFixedSize(250, 100);
  close_btn->setStyleSheet("background-color: #bb3333; font-size: 40px; border-radius: 10px; color: white;");
  connect(close_btn, &QPushButton::clicked, this, &AutoTunerCardListDialog::reject);
  header_layout->addWidget(close_btn);
  main_layout->addLayout(header_layout);

  // Scroll Area
  QScrollArea *scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setStyleSheet("QScrollArea { background: transparent; } QWidget { background: transparent; }");
  QScroller::grabGesture(scroll->viewport(), QScroller::LeftMouseButtonGesture);

  QWidget *scroll_widget = new QWidget();
  list_layout = new QVBoxLayout(scroll_widget);
  list_layout->setContentsMargins(0, 0, 0, 0);
  list_layout->setSpacing(10);
  scroll->setWidget(scroll_widget);
  main_layout->addWidget(scroll, 1);

  QVBoxLayout *outer_layout = new QVBoxLayout(this);
  outer_layout->setContentsMargins(100, 100, 100, 100);
  outer_layout->addWidget(container);

  refreshHistory();
}

void AutoTunerCardListDialog::refreshHistory() {
  // Clear parameter list layout
  QLayoutItem *child;
  while ((child = list_layout->takeAt(0)) != nullptr) {
    if (child->widget()) delete child->widget();
    delete child;
  }

  QString raw = QString::fromStdString(Params().get("CarrotLearningHistory"));
  if (raw.isEmpty()) {
    QLabel *lbl = new QLabel(tr("No historical data to display"), this);
    lbl->setStyleSheet("font-size: 45px; color: #888888;");
    lbl->setAlignment(Qt::AlignCenter);
    list_layout->addWidget(lbl);
    list_layout->addStretch();
    return;
  }

  QJsonArray arr = QJsonDocument::fromJson(raw.toUtf8()).array();
  for (int i = 0; i < arr.size(); i++) {
    QJsonObject item = arr[i].toObject();
    QString id = item["id"].toString();
    QString time_str = item["timestamp"].toString();
    QJsonObject changes = item["changes"].toObject();

    QFrame *row = new QFrame();
    row->setStyleSheet("background-color: #2b2b2b; border-radius: 15px; padding: 5px 25px;");
    QHBoxLayout *row_layout = new QHBoxLayout(row);
    row_layout->setContentsMargins(25, 5, 25, 5);

    QString text = QString("<span style='font-size: 35px; color: #aaaaaa;'>%1</span><br>").arg(tr("[%1 Applied]").arg(time_str));
    for (const QString& group : changes.keys()) {
      QJsonObject g_items = changes[group].toObject();
      QString short_group;
      bool is_ko = (QString::fromStdString(Params().get("LanguageSetting")) == "main_ko");
      if (!is_ko && group.contains("(")) {
        short_group = group.split("(").last().replace(")", "");
      } else {
        short_group = group.split(" ").first();
      }
      for (const QString& key : g_items.keys()) {
        QJsonObject info = g_items[key].toObject();
        text += QString("<span style='font-size: 40px; color: white;'><span style='color:#aaaaaa;'>[%1]</span> <b>%2</b> <span style='font-size:35px; color:#bbbbbb;'>[%3]</span> &nbsp;:&nbsp; %4 ➔ <span style='color:#00ff00; font-weight:bold;'>%5</span></span><br>")
                  .arg(short_group)
                  .arg(key)
                  .arg(info["band_kph"].toString())
                  .arg(info["current"].toInt())
                  .arg(info["recommended"].toInt());
      }
    }

    QLabel *lbl = new QLabel(text);
    lbl->setWordWrap(true);
    row_layout->addWidget(lbl, 1);

    bool is_latest = (i == 0);

    QPushButton *btn_restore = new QPushButton(tr("Restore"));
    if (is_latest) {
      btn_restore->setStyleSheet("background-color: #178644; font-size: 40px; padding: 20px; border-radius: 10px; color: white; font-weight: bold;");
    } else {
      btn_restore->setStyleSheet("background-color: #333333; font-size: 40px; padding: 20px; border-radius: 10px; color: #666666;");
    }
    btn_restore->setEnabled(is_latest);
    btn_restore->setFixedSize(220, 110);
    connect(btn_restore, &QPushButton::clicked, this, [=]() { restoreItem(id); });
    row_layout->addWidget(btn_restore);

    QPushButton *btn_del = new QPushButton(tr("Delete"));
    if (is_latest) {
      btn_del->setStyleSheet("background-color: #555555; font-size: 40px; padding: 20px; border-radius: 10px; color: white; font-weight: bold;");
    } else {
      btn_del->setStyleSheet("background-color: #333333; font-size: 40px; padding: 20px; border-radius: 10px; color: #666666;");
    }
    btn_del->setEnabled(is_latest);
    btn_del->setFixedSize(220, 110);
    connect(btn_del, &QPushButton::clicked, this, [=]() { deleteItem(id); });
    row_layout->addWidget(btn_del);

    list_layout->addWidget(row);
  }
  list_layout->addStretch();
}

void AutoTunerCardListDialog::restoreItem(const QString& id) {
  if (ConfirmationDialog::confirm(tr("Are you sure you want to restore the parameters to this state?"), tr("Restore"), this)) {
    QString raw = QString::fromStdString(Params().get("CarrotLearningHistory"));
    QJsonArray arr = QJsonDocument::fromJson(raw.toUtf8()).array();
    QJsonArray new_arr;
    
    for (int i = 0; i < arr.size(); i++) {
      QJsonObject entry = arr[i].toObject();
      if (entry["id"].toString() == id) {
        QJsonObject changes = entry["changes"].toObject();
        for (const QString& group : changes.keys()) {
          QJsonObject g_items = changes[group].toObject();
          for (const QString& key : g_items.keys()) {
            int prev_val = g_items[key].toObject()["current"].toInt();
            Params().putInt(key.toStdString(), prev_val);
          }
        }
      } else {
        new_arr.append(entry);
      }
    }
    
    if (new_arr.isEmpty()) {
      Params().remove("CarrotLearningHistory");
    } else {
      Params().put("CarrotLearningHistory", QJsonDocument(new_arr).toJson(QJsonDocument::Compact).toStdString());
    }
    refreshHistory();
    
    // Trigger refresh of parent panel
    AutoTunerHistoryPanel *p = qobject_cast<AutoTunerHistoryPanel*>(parent());
    if (p) {
      p->refreshHistory();
    }
    ConfirmationDialog::alert(tr("Restored to previous values successfully."), this);
  }
}

void AutoTunerCardListDialog::deleteItem(const QString& id) {
  if (ConfirmationDialog::confirm(tr("Are you sure you want to delete this item?"), tr("Delete"), this)) {
    QString raw = QString::fromStdString(Params().get("CarrotLearningHistory"));
    QJsonArray arr = QJsonDocument::fromJson(raw.toUtf8()).array();
    QJsonArray new_arr;
    for (int i = 0; i < arr.size(); i++) {
      if (arr[i].toObject()["id"].toString() != id) {
        new_arr.append(arr[i]);
      }
    }
    if (new_arr.isEmpty()) {
      Params().remove("CarrotLearningHistory");
    } else {
      Params().put("CarrotLearningHistory", QJsonDocument(new_arr).toJson(QJsonDocument::Compact).toStdString());
    }
    refreshHistory();
    
    // Trigger refresh of parent panel
    AutoTunerHistoryPanel *p = qobject_cast<AutoTunerHistoryPanel*>(parent());
    if (p) {
      p->refreshHistory();
    }
  }
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  panel_widget = new QStackedWidget();

  // close button
  QPushButton *close_btn = new QPushButton(tr("×"));
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 140px;
      padding-bottom: 20px;
      border-radius: 100px;
      background-color: #292929;
      font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(200, 200);
  sidebar_layout->addSpacing(45);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignCenter);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);

  TogglesPanel *toggles = new TogglesPanel(this);
  QObject::connect(this, &SettingsWindow::expandToggleDescription, toggles, &TogglesPanel::expandToggleDescription);

  auto networking = new Networking(this);
  QObject::connect(uiState()->prime_state, &PrimeState::changed, networking, &Networking::setPrimeType);

  QList<QPair<QString, QWidget *>> panels = {
    {tr("Device"), device},
    {tr("Network"), networking},
    {tr("Toggles"), toggles},
  };
  if(Params().getBool("SoftwareMenu")) {
    panels.append({tr("Software"), new SoftwarePanel(this)});
  }
  if(false) {
    panels.append({tr("Firehose"), new FirehosePanel(this)});
  }
  panels.append({ tr("CarrotPilot"), new CarrotPanel(this) });
  panels.append({ tr("Developer"), new DeveloperPanel(this) });

  nav_btns = new QButtonGroup(this);
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 65px;
        font-weight: 500;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )");
    btn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);

    const int lr_margin = name != tr("Network") ? 50 : 0;  // Network panel handles its own margins
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(500);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
    QStackedWidget, ScrollView {
      background-color: #292929;
      border-radius: 30px;
    }
  )");
}


#include <QScroller>
#include <QListWidget>

static QStringList get_list(const char* path) {
  QStringList stringList;
  QFile textFile(path);
  if (textFile.open(QIODevice::ReadOnly)) {
    QTextStream textStream(&textFile);
    while (true) {
      QString line = textStream.readLine();
      if (line.isNull()) {
        break;
      } else {
        stringList.append(line);
      }
    }
  }
  return stringList;
}

CarrotPanel::CarrotPanel(QWidget* parent) : QWidget(parent) {
  main_layout = new QStackedLayout(this);
  homeScreen = new QWidget(this);
  carrotLayout = new QVBoxLayout(homeScreen);
  carrotLayout->setMargin(10);

  QHBoxLayout* select_layout = new QHBoxLayout();
  select_layout->setSpacing(10);


  QPushButton* start_btn = new QPushButton(tr("Start"));
  start_btn->setObjectName("start_btn");
  QObject::connect(start_btn, &QPushButton::clicked, this, [this]() {
    this->currentCarrotIndex = 0;
    this->togglesCarrot(0);
    updateButtonStyles();
  });

  QPushButton* cruise_btn = new QPushButton(tr("Cruise"));
  cruise_btn->setObjectName("cruise_btn");
  QObject::connect(cruise_btn, &QPushButton::clicked, this, [this]() {
    this->currentCarrotIndex = 1;
    this->togglesCarrot(1);
    updateButtonStyles();
  });

  QPushButton* speed_btn = new QPushButton(tr("Speed"));
  speed_btn->setObjectName("speed_btn");
  QObject::connect(speed_btn, &QPushButton::clicked, this, [this]() {
    this->currentCarrotIndex = 2;
    this->togglesCarrot(2);
    updateButtonStyles();
  });

  QPushButton* latLong_btn = new QPushButton(tr("Tuning"));
  latLong_btn->setObjectName("latLong_btn");
  QObject::connect(latLong_btn, &QPushButton::clicked, this, [this]() {
    this->currentCarrotIndex = 3;
    this->togglesCarrot(3);
    updateButtonStyles();
  });

  QPushButton* disp_btn = new QPushButton(tr("Disp"));
  disp_btn->setObjectName("disp_btn");
  QObject::connect(disp_btn, &QPushButton::clicked, this, [this]() {
    this->currentCarrotIndex = 4;
    this->togglesCarrot(4);
    updateButtonStyles();
  });

  QPushButton* path_btn = new QPushButton(tr("Path"));
  path_btn->setObjectName("path_btn");
  QObject::connect(path_btn, &QPushButton::clicked, this, [this]() {
    this->currentCarrotIndex = 5;
    this->togglesCarrot(5);
    updateButtonStyles();
  });

  updateButtonStyles();

  select_layout->addWidget(start_btn);
  select_layout->addWidget(cruise_btn);
  select_layout->addWidget(speed_btn);
  select_layout->addWidget(latLong_btn);
  select_layout->addWidget(disp_btn);
  select_layout->addWidget(path_btn);
  carrotLayout->addLayout(select_layout, 0);

  QWidget* toggles = new QWidget();
  QVBoxLayout* toggles_layout = new QVBoxLayout(toggles);

  cruiseToggles = new ListWidget(this);
  cruiseToggles->addItem(new CValueControl("CruiseButtonMode", tr("Button: Cruise Button Mode"), tr("0:Normal,1:User1,2:User2"), 0, 2, 1));
  cruiseToggles->addItem(new CValueControl("CancelButtonMode", tr("Button: Cancel Button Mode"), tr("0:Long,1:Long+Lat"), 0, 1, 1));
  cruiseToggles->addItem(new CValueControl("LfaButtonMode", tr("Button: LFA Button Mode"), tr("0:Normal,1:Decel&Stop&LeadCarReady"), 0, 1, 1));
  cruiseToggles->addItem(new CValueControl("CruiseSpeedUnitBasic", tr("Button: Cruise Speed Unit(Basic)"), "", 1, 20, 1));
  cruiseToggles->addItem(new CValueControl("CruiseSpeedUnit", tr("Button: Cruise Speed Unit(Extra)"), "", 1, 20, 1));
  cruiseToggles->addItem(new CValueControl("CruiseEcoControl", tr("CRUISE: Eco control(4km/h)"), tr("Temporarily increasing the set speed to improve fuel efficiency."), 0, 10, 1));
  cruiseToggles->addItem(new CValueControl("AutoSpeedUptoRoadSpeedLimit", tr("CRUISE: Auto speed up (0%)"), tr("Auto speed up based on the lead car up to RoadSpeedLimit."), 0, 200, 10));
  cruiseToggles->addItem(new CValueControl("TFollowGap1", tr("GAP1: Apply TFollow (110)x0.01s"), "", 70, 300, 5));
  cruiseToggles->addItem(new CValueControl("TFollowGap2", tr("GAP2: Apply TFollow (120)x0.01s"), "", 70, 300, 5));
  cruiseToggles->addItem(new CValueControl("TFollowGap3", tr("GAP3: Apply TFollow (160)x0.01s"), "", 70, 300, 5));
  cruiseToggles->addItem(new CValueControl("TFollowGap4", tr("GAP4: Apply TFollow (180)x0.01s"), "", 70, 300, 5));
  cruiseToggles->addItem(new CValueControl("DynamicTFollow", tr("Dynamic GAP control"), "", 0, 100, 5));
  cruiseToggles->addItem(new CValueControl("DynamicTFollowLC", tr("Dynamic GAP control (LaneChange)"), "", 0, 100, 5));
  cruiseToggles->addItem(new CValueControl("MyDrivingMode", tr("DRIVEMODE: Select"), tr("1:ECO,2:SAFE,3:NORMAL,4:HIGH"), 1, 4, 1));
  cruiseToggles->addItem(new CValueControl("MyDrivingModeAuto", tr("DRIVEMODE: Auto"), tr("NORMAL mode only"), 0, 1, 1));
  cruiseToggles->addItem(new CValueControl("TrafficLightDetectMode", tr("TrafficLight DetectMode"), tr("0:None, 1:Stopping only, 2: Stop & Go"), 0, 2, 1));
  cruiseToggles->addItem(new CValueControl("AChangeCostStarting", tr("AChangeCostStarting"), "", 0, 200, 10));
  cruiseToggles->addItem(new CValueControl("TrafficStopDistanceAdjust", tr("TrafficStopDistanceAdjust"), "", -600, 600, 50));
  //cruiseToggles->addItem(new CValueControl("CruiseSpeedMin", "CRUISE: Speed Lower limit(10)", "Cruise control MIN speed", 5, 50, 1));
  //cruiseToggles->addItem(new CValueControl("AutoResumeFromGas", "GAS CRUISE ON: Use", "Auto Cruise on when GAS pedal released, 60% Gas Cruise On automatically", 0, 3, 1));
  //cruiseToggles->addItem(new CValueControl("AutoResumeFromGasSpeed", "GAS CRUISE ON: Speed(30)", "Driving speed exceeds the set value, Cruise ON", 20, 140, 5));
  //cruiseToggles->addItem(new CValueControl("TFollowSpeedAddM", "GAP: Additional TFs 40km/h(0)x0.01s", "Speed-dependent additional max(100km/h) TFs", -100, 200, 5));
  //cruiseToggles->addItem(new CValueControl("TFollowSpeedAdd", "GAP: Additional TFs 100Km/h(0)x0.01s", "Speed-dependent additional max(100km/h) TFs", -100, 200, 5));
  //cruiseToggles->addItem(new CValueControl("MyEcoModeFactor", "DRIVEMODE: ECO Accel ratio(80%)", "Acceleration ratio in ECO mode", 10, 95, 5));
  //cruiseToggles->addItem(new CValueControl("MySafeModeFactor", "DRIVEMODE: SAFE ratio(60%)", "Accel/StopDistance/DecelRatio/Gap control ratio", 10, 90, 10));
  //cruiseToggles->addItem(new CValueControl("MyHighModeFactor", "DRIVEMODE: HIGH ratio(100%)", "AccelRatio control ratio", 100, 300, 10));

  latLongToggles = new ListWidget(this);

  QPushButton* viewHistoryBtn = new QPushButton(tr("View Tuning History"));
  viewHistoryBtn->setObjectName("viewHistoryBtn");
  viewHistoryBtn->setStyleSheet(R"(
    QPushButton {
      margin-top: 10px; margin-bottom: 20px; padding: 10px; height: 120px; border-radius: 15px;
      color: #FFFFFF; background-color: #2C2CE2;
      font-size: 50px; font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #2424FF;
    }
  )");
  connect(viewHistoryBtn, &QPushButton::clicked, this, [=]() {
    AutoTunerHistoryDialog dlg(this);
    dlg.exec();
  });
  latLongToggles->addItem(viewHistoryBtn);
  // 공장초기화(Factory Reset) 버튼은 튜닝 이력 그래프 화면 좌측 하단으로 이동함.

  CValueControl* learningActiveCtrl = new CValueControl("CarrotLearningActive", tr("Auto-Tuner: Driving-Based Learning"), tr("Learn from driver interventions (gas/brake) and recommend parameter adjustments when parking. 0=Off, 1=On"), 0, 1, 1);
  connect(learningActiveCtrl, &CValueControl::valueChanged, this, [=](int val) {
    viewHistoryBtn->setVisible(val == 1);
  });
  viewHistoryBtn->setVisible(Params().getBool("CarrotLearningActive"));
  latLongToggles->addItem(learningActiveCtrl);
  latLongToggles->addItem(new CValueControl("UseLaneLineSpeed", tr("Laneline mode speed(0)"), tr("Laneline mode, lat_mpc control used"), 0, 200, 5));
  latLongToggles->addItem(new CValueControl("UseLaneLineCurveSpeed", tr("Laneline mode curve speed(0)"), tr("Laneline mode, high speed only"), 0, 200, 5));
  latLongToggles->addItem(new CValueControl("AdjustLaneOffset", tr("AdjustLaneOffset(0)cm"), "", 0, 500, 5));
  latLongToggles->addItem(new CValueControl("LaneChangeNeedTorque", tr("LaneChange need torque"), tr("-1:Disable lanechange, 0: no need torque, 1:need torque"), -1, 1, 1));
  latLongToggles->addItem(new CValueControl("LaneChangeDelay", tr("LaneChange delay"), tr("x0.1sec"), 0, 100, 5));
  latLongToggles->addItem(new CValueControl("LaneChangeBsd", tr("LaneChange Bsd"), tr("-1:ignore bsd, 0:BSD detect, 1: block steer torque"), -1, 1, 1));
  latLongToggles->addItem(new CValueControl("LaneLineCheck", tr("LaneChange LineCheck"), tr("0:Color+Type, 1:Type only, 2:Type+torque override solid"), 0, 2, 1));
  latLongToggles->addItem(new CValueControl("CustomSR", tr("LAT: SteerRatiox0.1(0)"), tr("Custom SteerRatio"), 0, 300, 1));
  latLongToggles->addItem(new CValueControl("SteerRatioRate", tr("LAT: SteerRatioRatex0.01(100)"), tr("SteerRatio apply rate"), 30, 170, 1));
  latLongToggles->addItem(new CValueControl("PathOffset", tr("LAT: PathOffset"), tr("(-)left, (+)right"), -150, 150, 1));
  latLongToggles->addItem(new CValueControl("SteerActuatorDelay", tr("LAT:SteerActuatorDelay(30)"), tr("x0.01, 0:LiveDelay"), 0, 100, 1));
  latLongToggles->addItem(new CValueControl("LatSmoothSec", tr("LAT:LatSmoothSec(13)"), tr("x0.01"), 1, 30, 1));
  latLongToggles->addItem(new CValueControl("LateralTorqueCustom", tr("LAT: TorqueCustom(0)"), "", 0, 2, 1));
  latLongToggles->addItem(new CValueControl("LateralTorqueAccelFactor", tr("LAT: TorqueAccelFactor(2500)"), "", 1000, 6000, 10));
  latLongToggles->addItem(new CValueControl("LateralTorqueFriction", tr("LAT: TorqueFriction(100)"), "", 0, 1000, 10));
  latLongToggles->addItem(new CValueControl("CustomSteerMax", tr("LAT: CustomSteerMax(0)"), "", 0, 30000, 5));
  latLongToggles->addItem(new CValueControl("CustomSteerDeltaUp", tr("LAT: CustomSteerDeltaUp(0)"), "", 0, 50, 1));
  latLongToggles->addItem(new CValueControl("CustomSteerDeltaDown", tr("LAT: CustomSteerDeltaDown(0)"), "", 0, 50, 1));
  latLongToggles->addItem(new CValueControl("LongTuningKpV", tr("LONG: P Gain(100)"), "", 0, 150, 5));
  latLongToggles->addItem(new CValueControl("LongTuningKiV", tr("LONG: I Gain(0)"), "", 0, 2000, 5));
  latLongToggles->addItem(new CValueControl("LongTuningKf", tr("LONG: FF Gain(100)"), "", 0, 200, 5));
  latLongToggles->addItem(new CValueControl("LongActuatorDelay", tr("LONG: ActuatorDelay(20)"), "", 0, 200, 5));
  latLongToggles->addItem(new CValueControl("VEgoStopping", tr("LONG: VEgoStopping(50)"), tr("Stopping factor"), 1, 100, 5));
  latLongToggles->addItem(new CValueControl("RadarReactionFactor", tr("LONG: Radar reaction factor(100)"), "", 0, 200, 10));
  latLongToggles->addItem(new CValueControl("StoppingAccel", tr("LONG: StoppingStartAccelx0.01(-40)"), "", -100, 0, 5));
  latLongToggles->addItem(new CValueControl("StopDistanceCarrot", tr("LONG: StopDistance (600)cm"), "", 300, 1000, 10));
  latLongToggles->addItem(new CValueControl("JLeadFactor3", tr("LONG: Jerk Lead Factor (0)"), tr("x0.01"), 0, 100, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals0", tr("ACCEL:0km/h(160)"), tr("Acceleration needed at specified speed.(x0.01m/s^2)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals1", tr("ACCEL:10km/h(160)"), tr("Acceleration needed at specified speed.(x0.01m/s^2)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals2", tr("ACCEL:40km/h(120)"), tr("Acceleration needed at specified speed.(x0.01m/s^2)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals3", tr("ACCEL:60km/h(100)"), tr("Acceleration needed at specified speed.(x0.01m/s^2)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals4", tr("ACCEL:80km/h(80)"), tr("Acceleration needed at specified speed.(x0.01m/s^2)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals5", tr("ACCEL:110km/h(70)"), tr("Acceleration needed at specified speed.(x0.01m/s^2)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals6", tr("ACCEL:140km/h(60)"), tr("Acceleration needed at specified speed.(x0.01m/s^2)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("MaxAngleFrames", tr("MaxAngleFrames(89)"), tr("89:Basic, steering instrument panel error 85~87"), 80, 100, 1));
  //latLongToggles->addItem(new CValueControl("AutoLaneChangeSpeed", "LaneChangeSpeed(20)", "", 1, 100, 5));
  //latLongToggles->addItem(new CValueControl("JerkStartLimit", "LONG: JERK START(10)x0.1", "Starting Jerk.", 1, 50, 1));
  //latLongToggles->addItem(new CValueControl("LongitudinalTuningApi", "LONG: ControlType", "0:velocity pid, 1:accel pid, 2:accel pid(comma)", 0, 2, 1));
  //latLongToggles->addItem(new CValueControl("StartAccelApply", "LONG: StartingAccel 2.0x(0)%", "정지->출발시 가속도의 가속율을 지정합니다 0: 사용안함.", 0, 100, 10));
  //latLongToggles->addItem(new CValueControl("StopAccelApply", "LONG: StoppingAccel -2.0x(0)%", "정지유지시 브레이크압을 조정합니다. 0: 사용안함. ", 0, 100, 10));
  //latLongToggles->addItem(new CValueControl("TraffStopDistanceAdjust", "LONG: TrafficStopDistance adjust(150)cm", "", -1000, 1000, 10));
  //latLongToggles->addItem(new CValueControl("CruiseMinVals", "DECEL:(120)", "Sets the deceleration rate.(x0.01m/s^2)", 50, 250, 5));

  dispToggles = new ListWidget(this);
  dispToggles->addItem(new CValueControl("ShowDebugUI", tr("Debug Info"), "", 0, 2, 1));
  dispToggles->addItem(new CValueControl("ShowTpms", tr("Tpms Info"), "", 0, 3, 1));
  dispToggles->addItem(new CValueControl("ShowDateTime", tr("Time Info"), tr("0:None,1:Time/Date,2:Time,3:Date"), 0, 3, 1));
  dispToggles->addItem(new CValueControl("ShowPathEnd", tr("Path End"), tr("0:None,1:Display"), 0, 1, 1));
  dispToggles->addItem(new CValueControl("ShowDeviceState", tr("Device State"), tr("0:None,1:Display"), 0, 1, 1));
  dispToggles->addItem(new CValueControl("ShowLaneInfo", tr("Lane Info"), tr("-1:None, 0:Path, 1:Path+Lane, 2: Path+Lane+RoadEdge"), -1, 2, 1));
  dispToggles->addItem(new CValueControl("ShowRadarInfo", tr("Radar Info"), tr("0:None,1:Display,2:RelPos,3:Stopped Car"), 0, 3, 1));
  dispToggles->addItem(new CValueControl("ShowRouteInfo", tr("Route Info"), tr("0:None,1:Display"), 0, 1, 1));
  dispToggles->addItem(new CValueControl("ShowPlotMode", tr("Debug plot"), "", 0, 10, 1));
  dispToggles->addItem(new CValueControl("ShowCustomBrightness", tr("Brightness ratio"), "", 0, 100, 10));
  //dispToggles->addItem(new CValueControl("ShowHudMode", "Display Mode", "0:Frog,1:APilot,2:Bottom,3:Top,4:Left,5:Left-Bottom", 0, 5, 1));
  //dispToggles->addItem(new CValueControl("ShowSteerRotate", "Handle rotate", "0:None,1:Rotate", 0, 1, 1));
  //dispToggles->addItem(new CValueControl("ShowAccelRpm", "Accel meter", "0:None,1:Display,1:Accel+RPM", 0, 2, 1));
  //dispToggles->addItem(new CValueControl("ShowTpms", "TPMS", "0:None,1:Display", 0, 1, 1));
  //dispToggles->addItem(new CValueControl("ShowSteerMode", "Handle Display Mode", "0:Black,1:Color,2:None", 0, 2, 1));
  //dispToggles->addItem(new CValueControl("ShowConnInfo", "APM connection", "0:NOne,1:Display", 0, 1, 1));
  //dispToggles->addItem(new CValueControl("ShowBlindSpot", "BSD Info", "0:None,1:Display", 0, 1, 1));
  //dispToggles->addItem(new CValueControl("ShowGapInfo", "GAP Info", "0:None,1:Display", -1, 1, 1));
  //dispToggles->addItem(new CValueControl("ShowDmInfo", "DM Info", "0:None,1:Display,-1:Disable(Reboot)", -1, 1, 1));

  pathToggles = new ListWidget(this);
  pathToggles->addItem(new CValueControl("CarrotTireTrajectory", tr("Tire Trajectory"), tr("Display tire paths with a gradient effect on the lane markers."), 0, 1, 1));
  pathToggles->addItem(new CValueControl("ShowPathColorCruiseOff", tr("Path Color: Cruise OFF"), tr("(+10:Stroke)0:Red,1:Orange,2:Yellow,3:Green,4:Blue,5:Indigo,6:Violet,7:Brown,8:White,9:Black"), 0, 19, 1));
  pathToggles->addItem(new CValueControl("ShowPathMode", tr("Path Mode: Laneless"), tr("0:Normal,1,2:Rec,3,4:^^,5,6:Rec,7,8:^^,9,10,11,12:Smooth^^"), 0, 15, 1));
  pathToggles->addItem(new CValueControl("ShowPathColor", tr("Path Color: Laneless"), tr("(+10:Stroke)0:Red,1:Orange,2:Yellow,3:Green,4:Blue,5:Indigo,6:Violet,7:Brown,8:White,9:Black"), 0, 19, 1));
  pathToggles->addItem(new CValueControl("ShowPathModeLane", tr("Path Mode: LaneMode"), tr("0:Normal,1,2:Rec,3,4:^^,5,6:Rec,7,8:^^,9,10,11,12:Smooth^^"), 0, 15, 1));
  pathToggles->addItem(new CValueControl("ShowPathColorLane", tr("Path Color: LaneMode"), tr("(+10:Stroke)0:Red,1:Orange,2:Yellow,3:Green,4:Blue,5:Indigo,6:Violet,7:Brown,8:White,9:Black"), 0, 19, 1));
  pathToggles->addItem(new CValueControl("ShowPathWidth", tr("Path Width ratio(100%)"), "", 10, 200, 10));

  startToggles = new ListWidget(this);
  QString selected = QString::fromStdString(Params().get("CarSelected3"));
  QPushButton* selectCarBtn = new QPushButton(selected.length() > 1 ? selected : tr("SELECT YOUR CAR"));
  selectCarBtn->setObjectName("selectCarBtn");
  selectCarBtn->setStyleSheet(R"(
    QPushButton {
      margin-top: 20px; margin-bottom: 20px; padding: 10px; height: 120px; border-radius: 15px;
      color: #FFFFFF; background-color: #2C2CE2;
    }
    QPushButton:pressed {
      background-color: #2424FF;
    }
  )");
  //selectCarBtn->setFixedSize(350, 100);
  connect(selectCarBtn, &QPushButton::clicked, [=]() {
    QString selected = QString::fromStdString(Params().get("CarSelected3"));

    QStringList all_items = get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars").toStdString().c_str());
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_gm").toStdString().c_str()));
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_toyota").toStdString().c_str()));
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_mazda").toStdString().c_str()));
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_honda").toStdString().c_str()));
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_ford").toStdString().c_str()));
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_tesla").toStdString().c_str()));
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_volkswagen").toStdString().c_str()));

    QMap<QString, QStringList> car_groups;
    for (const QString& car : all_items) {
      QStringList parts = car.split(" ", QString::SkipEmptyParts);
      if (!parts.isEmpty()) {
        QString manufacturer = parts.first();
        car_groups[manufacturer].append(car);
      }
    }

    QStringList manufacturers = car_groups.keys();
    QString selectedManufacturer = MultiOptionDialog::getSelection(tr("Select Manufacturer"), manufacturers, manufacturers.isEmpty() ? "" : manufacturers.first(), this);

    if (!selectedManufacturer.isEmpty()) {
      QStringList cars = car_groups[selectedManufacturer];
      QString selectedCar = MultiOptionDialog::getSelection(tr("Select your car"), cars, selected, this);

      if (!selectedCar.isEmpty()) {
        if (selectedCar == "[ Not Selected ]") {
          Params().remove("CarSelected3");
        } else {
          printf("Selected Car: %s\n", selectedCar.toStdString().c_str());
          Params().put("CarSelected3", selectedCar.toStdString());
          QTimer::singleShot(1000, []() {
            Params().putInt("SoftRestartTriggered", 1);
          });
          ConfirmationDialog::alert(selectedCar, this);
        }
        selected = QString::fromStdString(Params().get("CarSelected3"));
        selectCarBtn->setText((selected.isEmpty() || selected == "[ Not Selected ]") ? tr("SELECT YOUR CAR") : selected);
      }
    }
  });

  startToggles->addItem(selectCarBtn);
  startToggles->addItem(new CValueControl("HyundaiCameraSCC", tr("HYUNDAI: CAMERA SCC"), tr("1:Connect the SCC's CAN line to CAM, 2:Sync Cruise state, 3:StockLong"), 0, 3, 1));
  startToggles->addItem(new CValueControl("CanfdHDA2", tr("CANFD: HDA2 mode"), tr("1:HDA2,2:HDA2+BSM"), 0, 2, 1));
  startToggles->addItem(new CValueControl("EnableRadarTracks", tr("Enable Radar Track"), tr("1:Enable RadarTrack, -1,2:Disable use HKG SCC radar at all times"), -1, 3, 1));
  startToggles->addItem(new CValueControl("AutoCruiseControl", tr("Auto Cruise control"), tr("Softhold, Auto Cruise ON/OFF control"), 0, 3, 1));
  startToggles->addItem(new CValueControl("CruiseOnDist", tr("CRUISE: Auto ON distance(0cm)"), tr("When GAS/Brake is OFF, Cruise ON when the lead car gets closer."), 0, 2500, 50));
  startToggles->addItem(new CValueControl("AutoEngage", tr("Auto Engage control on start"), tr("1:SteerEnable, 2:Steer/Cruise Engage"), 0, 2, 1));
  startToggles->addItem(new CValueControl("AutoGasTokSpeed", tr("Auto AccelTok speed"), tr("Gas(Accel)Tok enable speed"), 0, 200, 5));
  startToggles->addItem(new CValueControl("SpeedFromPCM", tr("Read Cruise Speed from PCM"), tr("Toyota must set to 1, Honda 3"), 0, 3, 1));
  startToggles->addItem(new CValueControl("SoundVolumeAdjust", tr("Sound Volume(100%)"), "", 5, 200, 5));
  startToggles->addItem(new CValueControl("SoundVolumeAdjustEngage", tr("Sound Volume, Engage(10%)"), "", 5, 200, 5));
  startToggles->addItem(new CValueControl("MaxTimeOffroadMin", tr("Power off time (min)"), "", 1, 600, 10));
  startToggles->addItem(new CValueControl("EnableConnect", tr("EnableConnect"), tr("Your device may be banned by Comma"), 0, 2, 1));
  startToggles->addItem(new CValueControl("MapboxStyle", tr("Mapbox Style(0)"), "", 0, 2, 1));
  startToggles->addItem(new CValueControl("RecordRoadCam", tr("Record Road camera(0)"), tr("1:RoadCam, 2:RoadCam+WideRoadCam"), 0, 2, 1));
  startToggles->addItem(new CValueControl("HDPuse", tr("Use HDP(CCNC)(0)"), tr("1:While Using APN, 2:Always"), 0, 2, 1));
  startToggles->addItem(new CValueControl("NNFF", tr("NNFF"), tr("Twilsonco's NNFF(Reboot required)"), 0, 1, 1));
  startToggles->addItem(new CValueControl("NNFFLite", tr("NNFFLite"), tr("Twilsonco's NNFF-Lite(Reboot required)"), 0, 1, 1));
  startToggles->addItem(new CValueControl("AutoGasSyncSpeed", tr("Auto update Cruise speed"), "", 0, 1, 1));
  startToggles->addItem(new CValueControl("DisableMinSteerSpeed", tr("Disable Min.SteerSpeed"), "", 0, 1, 1));
  startToggles->addItem(new CValueControl("DisableDM", tr("Disable DM"), "", 0, 2, 1));
  startToggles->addItem(new CValueControl("HotspotOnBoot", tr("Hotspot enabled on boot"), "", 0, 1, 1));
  startToggles->addItem(new CValueControl("SoftwareMenu", tr("Enable Software Menu"), "", 0, 1, 1));
  startToggles->addItem(new CValueControl("IsLdwsCar", tr("IsLdwsCar"), "", 0, 1, 1));
  startToggles->addItem(new CValueControl("HardwareC3xLite", tr("Hardware is C3x Lite"), "", 0, 1, 1));
  startToggles->addItem(new CValueControl("ShareData", tr("Share Data"), tr("0:None, 1:TCP JSON Data(Reboot required)"), 0, 1, 1));
  //startToggles->addItem(new CValueControl("CarrotCountDownSpeed", "NaviCountDown Speed(10)", "", 0, 200, 5));
  //startToggles->addItem(new ParamControl("NoLogging", "Disable Logger", "", this));
  //startToggles->addItem(new ParamControl("LaneChangeNeedTorque", "LaneChange: Need Torque", "", this));
  //startToggles->addItem(new CValueControl("LaneChangeLaneCheck", "LaneChange: Check lane exist", "(0:No,1:Lane,2:+Edge)", 0, 2, 1));

  speedToggles = new ListWidget(this);
  speedToggles->addItem(new CValueControl("AutoCurveSpeedLowerLimit", tr("CURVE: Lower limit speed(30)"), tr("When you approach a curve, reduce your speed. Minimum speed"), 30, 200, 5));
  speedToggles->addItem(new CValueControl("AutoCurveSpeedFactor", tr("CURVE: Auto Control ratio(100%)"), "", 50, 300, 1));
  speedToggles->addItem(new CValueControl("AutoRoadSpeedLimitOffset", tr("RoadSpeedLimitOffset(-1)"), tr("-1:NotUsed,RoadLimitSpeed+Offset"), -1, 100, 1));
  speedToggles->addItem(new CValueControl("AutoRoadSpeedAdjust", tr("Auto Roadlimit Speed adjust (50%)"), "", -1, 100, 5));
  speedToggles->addItem(new CValueControl("AutoNaviSpeedCtrlEnd", tr("SpeedCamDecelEnd(6s)"), tr("Sets the deceleration completion point. A larger value completes deceleration farther away from the camera."), 3, 20, 1));
  speedToggles->addItem(new CValueControl("AutoNaviSpeedCtrlMode", tr("NaviSpeedControlMode(2)"), tr("0:No slowdown, 1: speed camera, 2: + accident prevention bump, 3: + mobile camera"), 0, 3, 1));
  speedToggles->addItem(new CValueControl("AutoNaviSpeedDecelRate", tr("SpeedCamDecelRatex0.01m/s^2(80)"), tr("Lower number, slows down from a greater distance"), 10, 200, 10));
  speedToggles->addItem(new CValueControl("AutoNaviSpeedSafetyFactor", tr("SpeedCamSafetyFactor(105%)"), "", 80, 120, 1));
  speedToggles->addItem(new CValueControl("AutoNaviSpeedBumpTime", tr("SpeedBumpTimeDistance(1s)"), "", 1, 50, 1));
  speedToggles->addItem(new CValueControl("AutoNaviSpeedBumpSpeed", tr("SpeedBumpSpeed(35Km/h)"), "", 10, 100, 5));
  speedToggles->addItem(new CValueControl("AutoNaviCountDownMode", tr("NaviCountDown mode(2)"), tr("0: off, 1:tbt+camera, 2:tbt+camera+bump"), 0, 2, 1));
  speedToggles->addItem(new CValueControl("TurnSpeedControlMode", tr("Turn Speed control mode(1)"), tr("0: off, 1:vision, 2:vision+route, 3: route"), 0, 3, 1));
  speedToggles->addItem(new CValueControl("CarrotSmartSpeedControl", tr("Smart Speed Control(0)"), tr("0: off, 1:accel, 2:decel, 3: all"), 0, 3, 1));
  speedToggles->addItem(new CValueControl("MapTurnSpeedFactor", tr("Map TurnSpeed Factor(100)"), "", 50, 300, 5));
  speedToggles->addItem(new CValueControl("ModelTurnSpeedFactor", tr("Model TurnSpeed Factor(0)"), "", 0, 80, 10));
  speedToggles->addItem(new CValueControl("AutoTurnControl", tr("ATC: Auto turn control(0)"), tr("0:None, 1: lane change, 2: lane change + speed, 3: speed"), 0, 3, 1));
  speedToggles->addItem(new CValueControl("AutoTurnControlSpeedTurn", tr("ATC: Turn Speed (20)"), tr("0:None, turn speed"), 0, 100, 5));
  speedToggles->addItem(new CValueControl("AutoTurnControlTurnEnd", tr("ATC: Turn CtrlDistTime (6)"), tr("dist=speed*time"), 0, 30, 1));
  speedToggles->addItem(new CValueControl("AutoTurnMapChange", tr("ATC Auto Map Change(0)"), "", 0, 1, 1));

  toggles_layout->addWidget(cruiseToggles);
  toggles_layout->addWidget(latLongToggles);
  toggles_layout->addWidget(dispToggles);
  toggles_layout->addWidget(pathToggles);
  toggles_layout->addWidget(startToggles);
  toggles_layout->addWidget(speedToggles);
  ScrollView* toggles_view = new ScrollView(toggles, this);

  content_stack = new QStackedWidget(this);
  content_stack->addWidget(toggles_view);

  carrotLayout->addWidget(content_stack, 1);

  homeScreen->setLayout(carrotLayout);
  main_layout->addWidget(homeScreen);
  main_layout->setCurrentWidget(homeScreen);

  togglesCarrot(0);
}

void CarrotPanel::togglesCarrot(int widgetIndex) {
  content_stack->setCurrentIndex(0);
  startToggles->setVisible(widgetIndex == 0);
  cruiseToggles->setVisible(widgetIndex == 1);
  speedToggles->setVisible(widgetIndex == 2);
  latLongToggles->setVisible(widgetIndex == 3);
  dispToggles->setVisible(widgetIndex == 4);
  pathToggles->setVisible(widgetIndex == 5);
}

void CarrotPanel::updateButtonStyles() {
  QString styleSheet = R"(
      #start_btn, #cruise_btn, #speed_btn, #latLong_btn, #disp_btn, #path_btn {
        height: 120px;
        border-radius: 15px;
        background-color: #393939;
        color: #E4E4E4;
      }
      #start_btn:pressed, #cruise_btn:pressed, #speed_btn:pressed, #latLong_btn:pressed, #disp_btn:pressed, #path_btn:pressed {
        background-color: #4a4a4a;
      }
  )";

  switch (currentCarrotIndex) {
  case 0:
    styleSheet += "#start_btn { background-color: #33ab4c; }";
    break;
  case 1:
    styleSheet += "#cruise_btn { background-color: #33ab4c; }";
    break;
  case 2:
    styleSheet += "#speed_btn { background-color: #33ab4c; }";
    break;
  case 3:
    styleSheet += "#latLong_btn { background-color: #33ab4c; }";
    break;
  case 4:
    styleSheet += "#disp_btn { background-color: #33ab4c; }";
    break;
  case 5:
    styleSheet += "#path_btn { background-color: #33ab4c; }";
    break;
  }

  setStyleSheet(styleSheet);
}


CValueControl::CValueControl(const QString& params, const QString& title, const QString& desc, int min, int max, int unit)
  : AbstractControl(title, desc), m_params(params), m_min(min), m_max(max), m_unit(unit) {

  label.setAlignment(Qt::AlignVCenter | Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  QString btnStyle = R"(
    QPushButton {
      padding: 0;
      border-radius: 50px;
      font-size: 20px;
      font-weight: 300;
      color: #E4E4E4;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #4a4a4a;
    }
  )";

  btnminus.setStyleSheet(btnStyle);
  btnplus.setStyleSheet(btnStyle);
  btnminus.setFixedSize(100, 100);
  btnplus.setFixedSize(100, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  connect(&btnminus, &QPushButton::released, this, &CValueControl::decreaseValue);
  connect(&btnplus, &QPushButton::released, this, &CValueControl::increaseValue);

  refresh();
}

void CValueControl::showEvent(QShowEvent* event) {
  AbstractControl::showEvent(event);
  refresh();
}

void CValueControl::refresh() {
  QString strVal = QString::fromStdString(Params().get(m_params.toStdString()));
  label.setText(strVal);
  emit valueChanged(strVal.toInt());
}

void CValueControl::adjustValue(int delta) {
  int value = QString::fromStdString(Params().get(m_params.toStdString())).toInt();
  value = qBound(m_min, value + delta, m_max);
  Params().putInt(m_params.toStdString(), value);
  refresh();
  emit valueChanged(value);
}

void CValueControl::increaseValue() {
  adjustValue(m_unit);
}

void CValueControl::decreaseValue() {
  adjustValue(-m_unit);
}
