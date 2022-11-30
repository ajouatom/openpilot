#include "selfdrive/ui/qt/offroad/settings.h"

#include <cassert>
#include <cmath>
#include <string>

#include <QDebug>

#ifndef QCOM
#include "selfdrive/ui/qt/offroad/networking.h"
#endif

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "selfdrive/common/params.h"
#include "selfdrive/common/watchdog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/input.h"

#include <QComboBox>
#include <QAbstractItemView>
#include <QScroller>
#include <QListView>
#include <QListWidget>

#include <QProcess> // opkr
#include <QDateTime> // opkr
#include <QTimer> // opkr
#include <QFileInfo> // opkr

TogglesPanel::TogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  // param, title, desc, icon, confirm
  std::vector<std::tuple<QString, QString, QString, QString>> toggle_defs{
    {
      "OpenpilotEnabledToggle",
      tr("Enable openpilot"),
      tr("Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off."),
      "../assets/offroad/icon_openpilot.png",
    },
    {
      "ExperimentalMode",
      tr("Experimental Mode"),
      "",
      "../assets/img_experimental_white.svg",
    },
    {
      "ExperimentalLongitudinalEnabled",
      tr("openpilot Longitudinal Control (Alpha)"),
      QString("<b>%1</b><br><br>%2")
      .arg(tr("WARNING: openpilot longitudinal control is in alpha for this car and will disable Automatic Emergency Braking (AEB)."))
      .arg(tr("On this car, openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. Enable this to switch to openpilot longitudinal control. Enabling Experimental mode is recommended when enabling openpilot longitudinal control alpha.")),
      "../assets/offroad/icon_speed_limit.png",
    },
    {
      "SccConnectedBus2",
      "SCC Module connected BUS2",
      "",
      "../assets/offroad/icon_warning.png",
    },
    {
      "EnableRadarTracks",
      "EnableRadarTracks",
      "Using RadarTracks instead of SCC data. SANTAFE2022HEV",
      "../assets/offroad/icon_warning.png",
    },
    {
      "IsLdwEnabled",
      tr("Enable Lane Departure Warnings"),
      tr("Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31 mph (50 km/h)."),
      "../assets/offroad/icon_warning.png",
    },
    {
      "IsMetric",
      tr("Use Metric System"),
      tr("Display speed in km/h instead of mph."),
      "../assets/offroad/icon_metric.png",
    },
    {
      "RecordFront",
      tr("Record and Upload Driver Camera"),
      tr("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "DisengageOnAccelerator",
      tr("Disengage on Accelerator Pedal"),
      tr("When enabled, pressing the accelerator pedal will disengage openpilot."),
      "../assets/offroad/icon_disengage_on_accelerator.svg",
    },
#ifdef ENABLE_MAPS
    {
      "NavSettingTime24h",
      tr("Show ETA in 24h Format"),
      tr("Use 24h format instead of am/pm"),
      "../assets/offroad/icon_metric.png",
    },
    {
      "NavSettingLeftSide",
      tr("Show Map on Left Side of UI"),
      tr("Show map on left side when in split screen view."),
      "../assets/offroad/icon_road.png",
    },
#endif
  };

  for (auto &[param, title, desc, icon] : toggle_defs) {
    auto toggle = new ParamControl(param, title, desc, icon, this);

    bool locked = false;// params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);

    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }

  // Toggles with confirmation dialogs
  toggles["ExperimentalMode"]->setActiveIcon("../assets/img_experimental.svg");
  toggles["ExperimentalMode"]->setConfirmation(true, true);
  toggles["ExperimentalLongitudinalEnabled"]->setConfirmation(true, false);

  connect(toggles["ExperimentalLongitudinalEnabled"], &ToggleControl::toggleFlipped, [=]() {
    updateToggles();
  });
}

void TogglesPanel::expandToggleDescription(const QString &param) {
  toggles[param.toStdString()]->showDescription();
}

void TogglesPanel::showEvent(QShowEvent *event) {
  updateToggles();
}

void TogglesPanel::updateToggles() {
  auto e2e_toggle = toggles["ExperimentalMode"];
  auto op_long_toggle = toggles["ExperimentalLongitudinalEnabled"];
  const QString e2e_description = QString("%1<br>"
                                          "<h4>%2</h4><br>"
                                          "%3<br>"
                                          "<h4>%4</h4><br>"
                                          "%5")
                                  .arg(tr("openpilot defaults to driving in <b>chill mode</b>. Experimental mode enables <b>alpha-level features</b> that aren't ready for chill mode. Experimental features are listed below:"))
                                  .arg(tr("🌮 End-to-End Longitudinal Control 🌮"))
                                  .arg(tr("Let the driving model control the gas and brakes. openpilot will drive as it thinks a human would, including stopping for red lights and stop signs. "
                                       "Since the driving model decides the speed to drive, the set speed will only act as an upper bound. This is an alpha quality feature; mistakes should be expected."))
                                  .arg(tr("New Driving Visualization"))
                                  .arg(tr("The driving visualization will transition to the road-facing wide-angle camera at low speeds to better show some turns. The Experimental mode logo will also be shown in the top right corner."));

  auto cp_bytes = params.get("CarParamsPersistent");
  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    if (!CP.getExperimentalLongitudinalAvailable()) {
      params.remove("ExperimentalLongitudinalEnabled");
    }
    op_long_toggle->setVisible(CP.getExperimentalLongitudinalAvailable());

    const bool op_long = CP.getOpenpilotLongitudinalControl() && !CP.getExperimentalLongitudinalAvailable();
    const bool exp_long_enabled = CP.getExperimentalLongitudinalAvailable() && params.getBool("ExperimentalLongitudinalEnabled");
    if (op_long || exp_long_enabled) {
      // normal description and toggle
      e2e_toggle->setEnabled(true);
      e2e_toggle->setDescription(e2e_description);
    } else {
      // no long for now
      e2e_toggle->setEnabled(false);
      params.remove("ExperimentalMode");

      const QString no_long = tr("Experimental mode is currently unavailable on this car, since the car's stock ACC is used for longitudinal control.");
      const QString exp_long = tr("Enable experimental longitudinal control to allow experimental mode.");
      e2e_toggle->setDescription("<b>" + (CP.getExperimentalLongitudinalAvailable() ? exp_long : no_long) + "</b><br><br>" + e2e_description);
    }

    e2e_toggle->refresh();
  } else {
    e2e_toggle->setDescription(e2e_description);
    op_long_toggle->setVisible(true);
  }
}

DevicePanel::DevicePanel(SettingsWindow *parent) : ListWidget(parent) {
  setSpacing(50);
  addItem(new LabelControl(tr("Dongle ID"), getDongleId().value_or(tr("N/A"))));
  addItem(new LabelControl(tr("Serial"), params.get("HardwareSerial").c_str()));

  QHBoxLayout *reset_layout = new QHBoxLayout();
  reset_layout->setSpacing(30);

  // reset calibration button
  QPushButton *restart_openpilot_btn = new QPushButton(tr("Soft restart"));
  restart_openpilot_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(restart_openpilot_btn);
  QObject::connect(restart_openpilot_btn, &QPushButton::released, [=]() {
    emit closeSettings();
    QTimer::singleShot(1000, []() {
      Params().putBool("SoftRestartTriggered", true);
    });
  });

  // reset calibration button
  QPushButton *reset_calib_btn = new QPushButton(tr("Reset Calibration"));
  reset_calib_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(reset_calib_btn);
  QObject::connect(reset_calib_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration and live params?"), tr("Reset"), this)) {
      Params().remove("CalibrationParams");
      Params().remove("LiveParameters");
      emit closeSettings();
      QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", true);
      });
    }
  });

  addItem(reset_layout);

  // offroad-only buttons

  auto dcamBtn = new ButtonControl(tr("Driver Camera"), tr("PREVIEW"),
                                   tr("Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"));
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });
  addItem(dcamBtn);

  auto resetCalibBtn = new ButtonControl(tr("Reset Calibration"), tr("RESET"), "");
  connect(resetCalibBtn, &ButtonControl::showDescriptionEvent, this, &DevicePanel::updateCalibDescription);
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration?"), tr("Reset"), this)) {
      params.remove("CalibrationParams");
      params.remove("LiveTorqueParameters");
    }
  });
  addItem(resetCalibBtn);

  if (!params.getBool("Passive")) {
    auto retrainingBtn = new ButtonControl(tr("Review Training Guide"), tr("REVIEW"), tr("Review the rules, features, and limitations of openpilot"));
    connect(retrainingBtn, &ButtonControl::clicked, [=]() {
      if (ConfirmationDialog::confirm(tr("Are you sure you want to review the training guide?"), tr("Review"), this)) {
        emit reviewTrainingGuide();
      }
    });
    addItem(retrainingBtn);
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
      Params().put("LanguageSetting", langs[selection].toStdString());
      qApp->exit(18);
      watchdog_kick();
    }
  });
  addItem(translateBtn);

  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    for (auto btn : findChildren<ButtonControl *>()) {
        //btn->setEnabled(offroad);
        btn->setEnabled(true);
    }
  });

  // power buttons
  QHBoxLayout *power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton *reboot_btn = new QPushButton(tr("Reboot"));
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, this, &DevicePanel::reboot);

  QPushButton *rebuild_btn = new QPushButton(tr("Rebuild"));
  rebuild_btn->setObjectName("rebuild_btn");
  power_layout->addWidget(rebuild_btn);
  QObject::connect(rebuild_btn, &QPushButton::clicked, [=]() {

    if (ConfirmationDialog::confirm(tr("Are you sure you want to rebuild?"), tr("Reset"), this)) {
      std::system("cd /data/openpilot && scons -c");
      std::system("rm /data/openpilot/.sconsign.dblite");
      std::system("rm /data/openpilot/prebuilt");
      std::system("rm -rf /tmp/scons_cache");
      if (Hardware::TICI())
        std::system("sudo reboot");
      else
        std::system("reboot");
    }
  });

  QPushButton *poweroff_btn = new QPushButton(tr("Power Off"));
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);

  if (!Hardware::PC()) {
    connect(uiState(), &UIState::offroadTransition, poweroff_btn, &QPushButton::setVisible);
  }

  setStyleSheet(R"(
    #reboot_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #reboot_btn:pressed { background-color: #4a4a4a; }
    #rebuild_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #rebuild_btn:pressed { background-color: #4a4a4a; }
    #poweroff_btn { height: 120px; border-radius: 15px; background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
  addItem(power_layout);
}

void DevicePanel::updateCalibDescription() {
  QString desc =
      tr("openpilot requires the device to be mounted within 4° left or right and "
         "within 5° up or 8° down. openpilot is continuously calibrating, resetting is rarely required.");
  std::string calib_bytes = Params().get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != 0) {
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
        Params().putBool("DoReboot", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Reboot"), this);
  }
}

void DevicePanel::poweroff() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to power off?"), tr("Power Off"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        Params().putBool("DoShutdown", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Power Off"), this);
  }
}

C2NetworkPanel::C2NetworkPanel(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 0, 50, 0);

  ListWidget *list = new ListWidget();
  list->setSpacing(30);
  // wifi + tethering buttons
#ifdef QCOM
  auto wifiBtn = new ButtonControl("Wi-Fi Settings", "OPEN");
  QObject::connect(wifiBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_wifi(); });
  list->addItem(wifiBtn);

  auto tetheringBtn = new ButtonControl("Tethering Settings", "OPEN");
  QObject::connect(tetheringBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_tethering(); });
  list->addItem(tetheringBtn);
#endif
  ipaddress = new LabelControl("IP Address", "");
  list->addItem(ipaddress);

  // SSH key management
  list->addItem(new SshToggle());
  list->addItem(new SshControl());
  layout->addWidget(list);
  layout->addStretch(1);
}

void C2NetworkPanel::showEvent(QShowEvent *event) {
  ipaddress->setText(getIPAddress());
}

QString C2NetworkPanel::getIPAddress() {
  std::string result = util::check_output("ifconfig wlan0");
  if (result.empty()) return "";

  const std::string inetaddrr = "inet addr:";
  std::string::size_type begin = result.find(inetaddrr);
  if (begin == std::string::npos) return "";

  begin += inetaddrr.length();
  std::string::size_type end = result.find(' ', begin);
  if (end == std::string::npos) return "";

  return result.substr(begin, end - begin).c_str();
}

QWidget *network_panel(QWidget *parent) {
#ifdef QCOM
  return new C2NetworkPanel(parent);
#else
  return new Networking(parent);
#endif
}

static QStringList get_list(const char* path)
{
  QStringList stringList;
  QFile textFile(path);
  if(textFile.open(QIODevice::ReadOnly))
  {
      QTextStream textStream(&textFile);
      while (true)
      {
        QString line = textStream.readLine();
        if (line.isNull())
            break;
        else
            stringList.append(line);
      }
  }

  return stringList;
}

void SettingsWindow::showEvent(QShowEvent *event) {
  setCurrentPanel(0);
}

void SettingsWindow::setCurrentPanel(int index, const QString &param) {
  panel_widget->setCurrentIndex(index);
  nav_btns->buttons()[index]->setChecked(true);
  if (!param.isEmpty()) {
    emit expandToggleDescription(param);
  }
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  sidebar_layout->setMargin(0);
  panel_widget = new QStackedWidget();
  panel_widget->setStyleSheet(R"(
    border-radius: 30px;
    background-color: #292929;
  )");

  // close button
  QPushButton *close_btn = new QPushButton(tr("← Back"));
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 50px;
      font-weight: bold;
      margin: 0px;
      padding: 15px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #444444;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(300, 110);
  sidebar_layout->addSpacing(10);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignRight);
  sidebar_layout->addSpacing(10);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);
  QObject::connect(device, &DevicePanel::closeSettings, this, &SettingsWindow::closeSettings);
  TogglesPanel *toggles = new TogglesPanel(this);
  QObject::connect(this, &SettingsWindow::expandToggleDescription, toggles, &TogglesPanel::expandToggleDescription);

  QList<QPair<QString, QWidget *>> panels = {
    {tr("Device"), device},
#ifdef QCOM
    {tr("Network"), new C2NetworkPanel(this)},
#else
    {tr("Network"), new Networking(this)},
#endif
    {tr("Toggles"), toggles},
    {tr("Software"), new SoftwarePanel(this)},
    {tr("Cruise"), new CruisePanel(this)},
    {tr("Tuning"), new TuningPanel(this)},
    {tr("ETC"), new CommunityPanel(this)},
  };

#ifdef ENABLE_MAPS
  //auto map_panel = new MapPanel(this);
  //panels.push_back({tr("Navigation"), map_panel});
  //QObject::connect(map_panel, &MapPanel::closeSettings, this, &SettingsWindow::closeSettings);
#endif

  const int padding = panels.size() > 3 ? 25 : 35;

  nav_btns = new QButtonGroup(this);
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(QString(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 55px;
        font-weight: 500;
        padding-top: %1px;
        padding-bottom: %1px;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )").arg(padding));

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
  sidebar_layout->setContentsMargins(5, 50, 10, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(320);
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
  )");
}



/////////////////////////////////////////////////////////////////////////

CommunityPanel::CommunityPanel(QWidget* parent) : QWidget(parent) {

  main_layout = new QStackedLayout(this);

  homeScreen = new QWidget(this);
  QVBoxLayout* vlayout = new QVBoxLayout(homeScreen);
  vlayout->setContentsMargins(0, 20, 0, 20);

  QString selected = QString::fromStdString(Params().get("SelectedCar"));

  QPushButton* selectCarBtn = new QPushButton(selected.length() ? selected : tr("Select your car"));
  selectCarBtn->setObjectName("selectCarBtn");
  //selectCarBtn->setStyleSheet("margin-right: 30px;");
  //selectCarBtn->setFixedSize(350, 100);
  connect(selectCarBtn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(selectCar); });

  homeWidget = new QWidget(this);
  QVBoxLayout* toggleLayout = new QVBoxLayout(homeWidget);
  homeWidget->setObjectName("homeWidget");

  ScrollView *scroller = new ScrollView(homeWidget, this);
  scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  main_layout->addWidget(homeScreen);

  selectCar = new SelectCar(this);
  connect(selectCar, &SelectCar::backPress, [=]() { main_layout->setCurrentWidget(homeScreen); });
  connect(selectCar, &SelectCar::selectedCar, [=]() {

     QString selected = QString::fromStdString(Params().get("SelectedCar"));
     selectCarBtn->setText(selected.length() ? selected : tr("Select your car"));
     main_layout->setCurrentWidget(homeScreen);
  });
  main_layout->addWidget(selectCar);
  QHBoxLayout* layoutBtn = new QHBoxLayout(homeWidget);

  layoutBtn->addWidget(selectCarBtn);
  vlayout->addSpacing(10);
  vlayout->addLayout(layoutBtn, 0);
  
  auto tmuxlog_btn = new ButtonControl("Tmux error log", tr("RUN"));
  QObject::connect(tmuxlog_btn, &ButtonControl::clicked, [=]() {
    const std::string txt = util::read_file("/data/tmux_error.log");
    ConfirmationDialog::alert(QString::fromStdString(txt), this);
  });
  vlayout->addWidget(tmuxlog_btn);

  vlayout->addWidget(scroller, 1);

  auto updateBtn = new ButtonControl("Check Update and Apply", "Update");
  QObject::connect(updateBtn, &ButtonControl::clicked, [=]()
  {
      const char* gitcommit = "/data/openpilot/selfdrive/assets/addon/sh/gitcommit.sh";
      const char* gitpull = "/data/openpilot/selfdrive/assets/addon/sh/gitpull.sh";


      std::system(gitcommit);
      std::system("date '+%F %T' > /data/params/d/LastUpdateTime");
      QString desc = "";
      QString commit_local = QString::fromStdString(Params().get("GitCommit").substr(0, 7));
      QString commit_remote = QString::fromStdString(Params().get("GitCommitRemote").substr(0, 7));

      desc += QString("(Local/Remote): %1/%2\n").arg(commit_local, commit_remote);
      if (commit_local == commit_remote) {
          desc += QString("No Update exist.");
      }
      else {
          desc += QString("Update exist.");
      }
      if (ConfirmationDialog::alert(desc, this)) {
          //Params().putBool("OpkrPrebuiltOn", 0);
          std::system("cd /data/openpilot; rm -f prebuilt");
          std::system(gitpull);
      }
  });
  toggleLayout->addWidget(updateBtn);
  toggleLayout->addWidget(new CPrebuiltToggle());

  // 기타 (Community)
  //toggleLayout->addWidget(horizontal_line());
  //toggleLayout->addWidget(new ParamControl("SccConnectedBus2", "SCC배선이 BUS2에 연결됨", "SCC배선을 개조하여 BUS2에 연결된경우 켭니다.", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new CValueControl("EnableAutoEngage", "EnableAutoEngage", "0:Not used,1:Auto Engage/Cruise OFF,2:Auto Engage/Cruise ON", "../assets/offroad/icon_shell.png", 0, 2, 1));
  toggleLayout->addWidget(new CValueControl("MixRadarInfo", "MixRadarInfo for SCC Rardar", "0:Not used,1:Use", "../assets/offroad/icon_shell.png", 0, 1, 1));

  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new CValueControl("AutoNaviSpeedCtrl", tr("SpeedCameraControl(1)"), tr("0:Not used, 1:NDA"), "../assets/offroad/icon_road.png", 0, 1, 1));
  toggleLayout->addWidget(new CValueControl("AutoNaviSpeedCtrlStart", tr("SpeedCameraDecelStart(22s)"), "감속시작시점을 설정합니다. 값이 크면 감속을 카메라에서 멀리 시작", "../assets/offroad/icon_road.png", 10, 50, 1));
  toggleLayout->addWidget(new CValueControl("AutoNaviSpeedCtrlEnd", tr("SpeedCameraDecelEnd(6s)"), "감속완료시점을 설정합니다.값이 크면 카메라에서 멀리 감속 완료", ".. / assets / offroad / icon_road.png", 3, 20, 1));
  toggleLayout->addWidget(new CValueControl("LongControlActiveSound", "Notify sound 0:OFF,1:Half, 2:ON", "", "../assets/offroad/icon_road.png", 0, 2, 1));
  toggleLayout->addWidget(new ParamControl("CustomMapbox", "CustomMapBox입력", "http://IP주소:8082 에 접속하여 mapbox token을 입력하면 자동으로 켜집니다. 끄면, 초기화됩니다.", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new ParamControl("KeepEngage", "Keep Enagage mode", "", "../assets/offroad/icon_shell.png", this));
  toggleLayout->addWidget(new CValueControl("UseLaneLineSpeed", "Auto LaneMode change speed(0)", "0: Always laneless, 으로 하면 안함", "../assets/offroad/icon_road.png", 0, 200, 10));
  toggleLayout->addWidget(new CValueControl("PathOffset", "PathOffset", "(-)Left, (+)Right", "../assets/offroad/icon_road.png", -200, 200, 1));
  toggleLayout->addWidget(new CValueControl("HapticFeedbackWhenSpeedCamera", "Haptic handle function", "0:사용안함,1:진동,2:계기판,3:HUD표시", "../assets/offroad/icon_road.png", 0, 3, 1));
  toggleLayout->addWidget(new CValueControl("SoftHoldMode", "SoftHold(1)", "0:Not used,1:Use,2: with SCC(단,사이드가 걸리는 차량이 있음)", "../assets/offroad/icon_road.png", 0, 2, 1));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new CValueControl("ShowHudMode", "DISP:Display Mode", "0:Normal,1:APilot,2:Bottom,3:Top,4:Left,5:Left-Bottom", "../assets/offroad/icon_shell.png", 0, 5, 1));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("ShowDebugUI", "DISP:Debug Info", "", "../assets/offroad/icon_shell.png", this));
  toggleLayout->addWidget(new CValueControl("ShowDateTime", "DISP:Time Info", "0:None,1:Time/Date,2:Time,3:Date", "../assets/offroad/icon_shell.png", 0, 3, 1));
  toggleLayout->addWidget(new CValueControl("ShowSteerRotate", "DISP:Handle rotate", "0:None,1:Rotate", "../assets/offroad/icon_shell.png", 0, 1, 1));
  toggleLayout->addWidget(new CValueControl("ShowPathEnd", "DISP:Path End", "0:None,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
  toggleLayout->addWidget(new CValueControl("ShowAccelRpm", "DISP:Accel meter", "0:None,1:Display,1:Accel+RPM", "../assets/offroad/icon_shell.png", 0, 2, 1));
  toggleLayout->addWidget(new CValueControl("ShowTpms", "DISP:TPMS", "0:None,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
  toggleLayout->addWidget(new CValueControl("ShowSteerMode", "DISP:Handle Display Mode", "0:Black,1:Color,2:None", "../assets/offroad/icon_shell.png", 0, 2, 1));
  toggleLayout->addWidget(new CValueControl("ShowDeviceState", "DISP:Device State", "0:None,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
  toggleLayout->addWidget(new CValueControl("ShowConnInfo", "DISP:NDA connection", "0:NOne,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
  toggleLayout->addWidget(new CValueControl("ShowLaneInfo", "DISP:Lane Info", "-1:None, 0:Path, 1:Path+Lane, 2: Path+Lane+RoadEdge", "../assets/offroad/icon_shell.png", -1, 2, 1));
  toggleLayout->addWidget(new CValueControl("ShowBlindSpot", "DISP:BSD Info", "0:None,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
  toggleLayout->addWidget(new CValueControl("ShowGapInfo", "DISP:GAP Info", "0:None,1:Display", "../assets/offroad/icon_shell.png", -1, 1, 1));
  toggleLayout->addWidget(new CValueControl("ShowDmInfo", "DISP:DM Info", "0:None,1:Display,-1:Disable(Reboot)", "../assets/offroad/icon_shell.png", -1, 1, 1));
  toggleLayout->addWidget(new CValueControl("ShowRadarInfo", "DISP:Radar Info", "0:None,1:Display,2:RelPos,3:Stopped Car", "../assets/offroad/icon_shell.png", 0, 3, 1));
  toggleLayout->addWidget(new CValueControl("ShowPlotMode", "DISP:Debug plot", "", "../assets/offroad/icon_shell.png", 0, 5, 1));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new CValueControl("ShowZOffset", "DISP:Path Height adjust(170)", "(+) Down, (-) Up", "../assets/offroad/icon_shell.png", -300, 300, 10));
  toggleLayout->addWidget(new CValueControl("ShowPathModeCruiseOff", "DISP: Path Mode: Cruise OFFF", "0:Normal,1,2:Rec,3,4:^^,5,6:Rec,7,8:^^,9,10,11,12:Smooth^^", "../assets/offroad/icon_shell.png", 0, 11, 1));
  toggleLayout->addWidget(new CValueControl("ShowPathColorCruiseOff", "DISP: Path Color: Cruise OFF", "(+10:Stroke)0:Red,1:Orange,2:Yellow,3:Green,4:Blue,5:Indigo,6:Violet,7:Brown,8:White,9:Black", "../assets/offroad/icon_shell.png", 0, 19, 1));
  toggleLayout->addWidget(new CValueControl("ShowPathMode", "DISP:Path Mode: Laneless", "0:Normal,1,2:Rec,3,4:^^,5,6:Rec,7,8:^^,9,10,11,12:Smooth^^", "../assets/offroad/icon_shell.png", 0, 11, 1));
  toggleLayout->addWidget(new CValueControl("ShowPathColor", "DISP:Path Color: Laneless", "(+10:Stroke)0:Red,1:Orange,2:Yellow,3:Green,4:Blue,5:Indigo,6:Violet,7:Brown,8:White,9:Black", "../assets/offroad/icon_shell.png", 0, 19, 1));
  toggleLayout->addWidget(new CValueControl("ShowPathModeLane", "DISP:Path Mode: LaneMode", "0:Normal,1,2:Rec,3,4:^^,5,6:Rec,7,8:^^,9,10,11,12:Smooth^^", "../assets/offroad/icon_shell.png", 0, 11, 1));
  toggleLayout->addWidget(new CValueControl("ShowPathColorLane", "DISP:Path Color: LaneMode", "(+10:Stroke)0:Red,1:Orange,2:Yellow,3:Green,4:Blue,5:Indigo,6:Violet,7:Brown,8:White,9:Black", "../assets/offroad/icon_shell.png", 0, 19, 1));
  toggleLayout->addWidget(new CValueControl("ShowPathWidth", "DISP:Path Width ratio(100%)", "", "../assets/offroad/icon_shell.png", 10, 200, 10));

}

TuningPanel::TuningPanel(QWidget* parent) : QWidget(parent) {

    main_layout = new QStackedLayout(this);

    homeScreen = new QWidget(this);
    QVBoxLayout* vlayout = new QVBoxLayout(homeScreen);
    vlayout->setContentsMargins(0, 20, 0, 20);

    homeWidget = new QWidget(this);
    QVBoxLayout* toggleLayout = new QVBoxLayout(homeWidget);
    homeWidget->setObjectName("homeWidget");

    ScrollView* scroller = new ScrollView(homeWidget, this);
    scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    main_layout->addWidget(homeScreen);

    vlayout->addWidget(scroller, 1);

    // 튜닝
    toggleLayout->addWidget(new CValueControl("StopDistance", "StopDistance(600cm)", "선행차와 정지하는 거리를 입력합니다.", "../assets/offroad/icon_road.png", 200, 1000, 50));
    toggleLayout->addWidget(new CValueControl("TrafficStopDistanceAdjust", "TrafficStop Adjust(200cm)", "+값으로 하면 정지선에 다가갑니다.", "../assets/offroad/icon_road.png", -1000, 1000, 10));
    toggleLayout->addWidget(horizontal_line());    
    toggleLayout->addWidget(new CValueControl("LiveSteerRatioApply", "LAT: LiveSteerRatioApply(100)", "오버스티어가 발생하면 줄입니다.", "../assets/offroad/icon_road.png", 50, 110, 1));
    toggleLayout->addWidget(new CValueControl("SteeringRateCost", "LAT: SteeringRateCost(700)", "", "../assets/offroad/icon_road.png", 0, 2000, 10));
    toggleLayout->addWidget(new CValueControl("PathCostApply", "LAT: PathCostApply(100)", "", "../assets/offroad/icon_road.png", 0, 200, 5));
    toggleLayout->addWidget(new CValueControl("PathCostApplyLow", "LAT: PathCostApplyLow(100)", "", "../assets/offroad/icon_road.png", 0, 200, 5));
    toggleLayout->addWidget(new CValueControl("LateralAccelCost", "LAT: LateralAccelCost(0)", "", "../assets/offroad/icon_road.png", 0, 300, 1));
    toggleLayout->addWidget(new CValueControl("LateralMotionCost", "LAT: LateralMotionCost(11)", "", "../assets/offroad/icon_road.png", 0, 50, 1));
    toggleLayout->addWidget(new CValueControl("LateralJerkCost", "LAT: LateralJerkCost(4)", "", "../assets/offroad/icon_road.png", 0, 50, 1));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new CValueControl("LateralTorqueKp", "LAT: TorqueKp(100)", "", "../assets/offroad/icon_road.png", 0, 200, 1));
    toggleLayout->addWidget(new CValueControl("LateralTorqueKi", "LAT: TorqueKi(10)", "", "../assets/offroad/icon_road.png", 0, 100, 1));
    toggleLayout->addWidget(new CValueControl("LateralTorqueKd", "LAT: TorqueKd(0)", "", "../assets/offroad/icon_road.png", 0, 100, 1));
    toggleLayout->addWidget(new CValueControl("LateralTorqueKf", "LAT: TorqueKf(100)", "", "../assets/offroad/icon_road.png", 0, 200, 1));
    toggleLayout->addWidget(new CValueControl("LateralTorqueCustom", "LAT: TorqueCustom(0)", "", "../assets/offroad/icon_road.png", 0, 1, 1));
    toggleLayout->addWidget(new CValueControl("LateralTorqueAccelFactor", "LAT: TorqueAccelFactor(2500)", "", "../assets/offroad/icon_road.png", 1000, 4000, 10));
    toggleLayout->addWidget(new CValueControl("LateralTorqueFriction", "LAT: TorqueFriction(100)", "", "../assets/offroad/icon_road.png", 0, 1000, 10));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new CValueControl("SteerActuatorDelay", "LAT:SteerActuatorDelay(30)", "표준", "../assets/offroad/icon_road.png", 0, 100, 1));
    toggleLayout->addWidget(new CValueControl("SteerDeltaUp", "LAT: SteerDeltaUp(3)", "", "../assets/offroad/icon_road.png", 1, 20, 1));
    toggleLayout->addWidget(new CValueControl("SteerDeltaDown", "LAT: SteerDeltaDown(7)", "", "../assets/offroad/icon_road.png", 1, 20, 1));
    toggleLayout->addWidget(new CValueControl("SteerRatioApply", "LAT: SteerRatio적용(0x0.1)", "0:사용안함", "../assets/offroad/icon_road.png", 0, 300, 2));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new CValueControl("JerkUpperLowerLimit", "LONG: JERK(8)", "값이 커지면 가감속반응이 빨라지지만, 기분이 안좋음.", "../assets/offroad/icon_road.png", 1, 50, 1));
    toggleLayout->addWidget(new CValueControl("LongitudinalTuningKpV", "LONG: P Gain(100)", "(시험용) ", "../assets/offroad/icon_road.png", 50, 150, 1));
    toggleLayout->addWidget(new CValueControl("LongitudinalTuningKiV", "LONG: I Gain(200)", "(시험용) ", "../assets/offroad/icon_road.png", 0, 2000, 5));
    toggleLayout->addWidget(new CValueControl("StartAccelApply", "LONG: StartingAccel 2.0x(0%)", "정지->출발시 가속도의 가속율을 지정합니다 0: 사용안함.", "../assets/offroad/icon_road.png", 0, 100, 10));
    toggleLayout->addWidget(new CValueControl("StopAccelApply", "LONG: StoppingAccel -2.0x(30%)", "정지유지시 브레이크압을 조정합니다. 0: 사용안함. ", "../assets/offroad/icon_road.png", 0, 100, 10));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new CValueControl("XEgoObstacleCost", "X_EGO_COST(5)", "증가할수록 정지선정지가 정확해지나, 급감속이 강해집니다.", "../assets/offroad/icon_road.png", 3, 50, 1));
    toggleLayout->addWidget(new CValueControl("JEgoCost", "J_EGO_COST(5)", "", "../assets/offroad/icon_road.png", 4, 10, 1));
    toggleLayout->addWidget(new CValueControl("AChangeCost", "A_CHANGE_COST(150)", "적으면 선행차에 대한 반응이 강해집니다. ", "../assets/offroad/icon_road.png", 20, 400, 10));
    toggleLayout->addWidget(new CValueControl("DangerZoneCost", "DANGER_ZONE_COST(100)", "", "../assets/offroad/icon_road.png", 0, 400, 10));
    toggleLayout->addWidget(new CValueControl("LeadDangerFactor", "LEAD_DANGER_FACTOR(80)", "", "../assets/offroad/icon_road.png", 75, 100, 1));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new ParamControl("ApplyLongDynamicCost", "GAP: Dynamic Control(0)", "전방차량의 간격을 최대한 유지하도록 응답속도가 빨라집니다.", "../assets/offroad/icon_road.png", this));
    toggleLayout->addWidget(new CValueControl("ApplyDynamicTFollow", "GAP:REL.Speed-(105%)", "선행차와 점점 가까와지면 차량거리를 안전하게 증가시키도록 합니다.", "../assets/offroad/icon_road.png", 100, 300, 1));
    toggleLayout->addWidget(new CValueControl("ApplyDynamicTFollowApart", "GAP: REL.Speed+(95%)", "선행차와 점점 멀어지면 차량거리를 줄이도록 합니다.", "../assets/offroad/icon_road.png", 20, 100, 1));
    toggleLayout->addWidget(new CValueControl("ApplyDynamicTFollowDecel", "GAP: DECEL.Speed(105%)", "차량이 급감속 할 수록 차량간격을 벌리도록 제어합니다.", "../assets/offroad/icon_road.png", 100, 300, 1));
    toggleLayout->addWidget(new CValueControl("TFollowRatio", "GAP: Apply ratio: Caution(100%)", "선행차와의 간격을 조정합니다. 100%이하로 하면 매우 위험합니다.", "../assets/offroad/icon_road.png", 70, 120, 1));
    
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new CValueControl("TrafficStopMode", "STOPPING: Traffice Stop Mode (1)", "0:사용안함,1:Apilot모드,2:Mix실험적모드", "../assets/offroad/icon_road.png", 0, 2, 1));
    toggleLayout->addWidget(new CValueControl("TrafficStopAccel", "STOPPING: DECEL. rate (80%)", "신호를 만나면 서서히 감속하여 정지합니다.", "../assets/offroad/icon_road.png", 10, 120, 10));
    toggleLayout->addWidget(new CValueControl("ApplyModelDistOrder", "STOPPING: DECEL. model (30)", "숫자가적을수록 미리감속하고 서서히 정지합니다.", "../assets/offroad/icon_road.png", 1, 32, 1));
    toggleLayout->addWidget(new CValueControl("TrafficStopUpdateDist", "STOPPING: Stop line update dist (10M)", "", "../assets/offroad/icon_road.png", 0, 30, 1));
    toggleLayout->addWidget(new CValueControl("TrafficDetectBrightness", "STOPPING: Traffic detect outdoor brightness (100)", "", "../assets/offroad/icon_road.png", 0, 3000, 10));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new CValueControl("CruiseMaxVals1", "ACCEL:0km/h(200)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 1, 250, 5));
    toggleLayout->addWidget(new CValueControl("CruiseMaxVals2", "ACCEL:40km/h(150)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 1, 250, 5));
    toggleLayout->addWidget(new CValueControl("CruiseMaxVals3", "ACCEL:60km/h(50)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 1, 250, 5));
    toggleLayout->addWidget(new CValueControl("CruiseMaxVals4", "ACCEL:80km/h(20)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 1, 250, 5));
    toggleLayout->addWidget(new CValueControl("CruiseMaxVals5", "ACCEL:110km/h(15)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 1, 250, 5));
    toggleLayout->addWidget(new CValueControl("CruiseMaxVals6", "ACCEL:140km/h(15)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 1, 250, 5));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new CValueControl("AutoCurveSpeedCtrlUse", "TURN: Auto Control(1)", "곡선도로를 만나면 속도를 줄여줍니다. 0:사용안함,1:도로설계기준", "../assets/offroad/icon_road.png", 0, 3, 1));
    toggleLayout->addWidget(new CValueControl("AutoCurveSpeedFactor", "TURN: Auto Control ratio(100%)", "커브속도조절(커브속도 조절 3일때 170)", "../assets/offroad/icon_road.png", 50, 300, 1));
    toggleLayout->addWidget(new CValueControl("AutoCurveSpeedFactorIn", "TURN: Auto Control ratio In(0%)", "커브속도조절진입", "../assets/offroad/icon_road.png", 0, 300, 1));
    toggleLayout->addWidget(new CValueControl("AutoTurnControl", "ATURN: Model turn(0)", "저속 깜박이시 DESIRE제어", "../assets/offroad/icon_road.png", 0, 2, 1));
    toggleLayout->addWidget(new CValueControl("AutoTurnSpeed", "ATURN: Model turn: speed(40)", "해당속도이하에서 자동턴시작", "../assets/offroad/icon_road.png", 2, 60, 5));
    toggleLayout->addWidget(new CValueControl("AutoTurnTimeMax", "ATURN: Model turn: timeout(200)", "자동턴 시간제한 설정", "../assets/offroad/icon_road.png", 30, 500, 5));
    toggleLayout->addWidget(new CValueControl("AutoLaneChangeSpeed", "LANE CHANGE: Speed (30)", "해당속도 이상에서만 자동차선변경", "../assets/offroad/icon_road.png", 5, 60, 5));

}
CruisePanel::CruisePanel(QWidget* parent) : QWidget(parent) {

    main_layout = new QStackedLayout(this);

    homeScreen = new QWidget(this);
    QVBoxLayout* vlayout = new QVBoxLayout(homeScreen);
    vlayout->setContentsMargins(0, 20, 0, 20);

    homeWidget = new QWidget(this);
    QVBoxLayout* toggleLayout = new QVBoxLayout(homeWidget);
    homeWidget->setObjectName("homeWidget");

    ScrollView* scroller = new ScrollView(homeWidget, this);
    scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    main_layout->addWidget(homeScreen);

    vlayout->addWidget(scroller, 1);

    // 크루즈
    toggleLayout->addWidget(new CValueControl("CruiseControlMode", "CRUISE: Eco control(4km/h)", "Temporarily increasing the set speed to improve fuel efficiency.", "../assets/offroad/icon_road.png", 0, 10, 1));
    toggleLayout->addWidget(new CValueControl("CruiseOnDist", "CRUISE: Auto ON distance(0cm)", "When GAS/Brake is OFF, Cruise ON when the lead car gets closer or warning (- value).", "../assets/offroad/icon_road.png", -500, 500, 50));
    toggleLayout->addWidget(new CValueControl("AutoSyncCruiseSpeedMax", "CRUISE: Auto sync speed limit (120km/h)", "Driving speed exceeds target during acceleration, sync. target speed.", "../assets/offroad/icon_road.png", 0, 200, 10));
    toggleLayout->addWidget(new CValueControl("AutoSpeedUptoRoadSpeedLimit", "CRUISE: Auto speed up (100%)", "Auto speed up based on the lead car upto RoadSpeedLimit.", "../assets/offroad/icon_road.png", 0, 200, 10));
    toggleLayout->addWidget(new CValueControl("AutoSpeedAdjustWithLeadCar", "CRUISE: Auto speed set to lead Car(+0)", "useless, 선행차량의 속도에 옵셋속도를 더한 속도를 설정합니다.", "../assets/offroad/icon_road.png", 0, 100, 5));
    toggleLayout->addWidget(new CValueControl("InitMyDrivingMode", "DRIVEMODE: On boot(3)", "1:ECO,2:SAFE,3:NORMAL,4:HIGH(non E2E mode)", "../assets/offroad/icon_road.png", 1, 4, 1));
    toggleLayout->addWidget(new CValueControl("MyEcoModeFactor", "DRIVEMODE: ECO Accel ratio(80%)", "Acceleartion ratio in ECO mode", "../assets/offroad/icon_road.png", 10, 95, 5));
    toggleLayout->addWidget(new CValueControl("MySafeModeFactor", "DRIVEMODE: SAFE ratio(80%)", "Accel/StopDistance/DecelRatio/Gap control ratio", "../assets/offroad/icon_road.png", 10, 90, 10));
    toggleLayout->addWidget(new CValueControl("CruiseButtonMode", "Button: Cruise Speed Mode", "0:Normal,1:User1, 2:User2, 3:User3, 4:User4", "../assets/offroad/icon_road.png", 0, 4, 1));
    toggleLayout->addWidget(new CValueControl("CruiseSpeedUnit", "Button: Cruise Speed Unit", "", "../assets/offroad/icon_road.png", 1, 20, 1));
    toggleLayout->addWidget(new CValueControl("GapButtonMode", "Button: Cruise Gap Mode", "0:1,2,3,4(Auto),1:1,2,3,4(Auto),5(Cruise OFF),2:4(Auto),5(Cruise OFF),3:Cruise ON/OFF(Auto)", "../assets/offroad/icon_road.png", 0, 3, 1));
    toggleLayout->addWidget(new CValueControl("PrevCruiseGap", "CruiseGap: On boot(4)", "1,2,3,4(Auto)", "../assets/offroad/icon_road.png", 1, 4, 1));
    toggleLayout->addWidget(new CValueControl("CruiseSpeedMin", "Cruise Speed: Lower limit(10)", "Cruise control MIN speed", "../assets/offroad/icon_road.png", 5, 50, 1));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new ParamControl("AutoResumeFromGas", "GAS CRUISE ON: Use", "Auto Cruise on when GAS pedal released, 60% Gas Cruise On automatically", "../assets/offroad/icon_road.png", this));
    toggleLayout->addWidget(new CValueControl("AutoResumeFromGasSpeed", "GAS CRUISE ON: Speed(30)", "Driving speed exceeds the set value, Cruise ON", "../assets/offroad/icon_road.png", 20, 140, 5));
    toggleLayout->addWidget(new CValueControl("AutoResumeFromGasSpeedMode", "GAS CRUISE ON: Set Speed option(0)", "0:Current Speed, 1:Prev. speed, 2: Lead car speed, 3: 60M이상상직진시 이전속도", "../assets/offroad/icon_road.png", 0, 3, 1));
    toggleLayout->addWidget(new CValueControl("AutoCancelFromGasMode", "GAS CRUISE OFF: Mode", "Cruise OFF below speed(GAS CRUISE ON: speed). 1:Always, 2:No lear car", "../assets/offroad/icon_road.png", 0, 140, 1));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new ParamControl("AutoResumeFromBrakeRelease", "BRAKE CRUISE ON: Use", "Auto Cruise On when Brake released", "../assets/offroad/icon_road.png", this));
    toggleLayout->addWidget(new CValueControl("AutoResumeFromBrakeReleaseDist", "BRAKE CRUISE ON: Moving, LeadCar(20)", "on moving, lead car is greater than the set dist", "../assets/offroad/icon_road.png", 0, 80, 5));
    toggleLayout->addWidget(new ParamControl("AutoResumeFromBrakeReleaseLeadCar", "BRAKE CRUISE ON: Stopping, LeadCar", "on stopping, lead car within 10meters", "../assets/offroad/icon_road.png", this));
    toggleLayout->addWidget(new CValueControl("AutoResumeFromBrakeCarSpeed", "BRAKE CRUISE ON: Moving, Speed(40)", "on moving, my car is greater then the set dist(no lead car)", "../assets/offroad/icon_road.png", 0, 80, 5));
    toggleLayout->addWidget(new ParamControl("AutoResumeFromBrakeReleaseTrafficSign", "BRAKE CRUISE ON: Moving, Traffic signal", "on moving, if traffic signal detected", "../assets/offroad/icon_road.png", this));

}

// ajouatom
CValueControl::CValueControl(const QString& params, const QString& title, const QString& desc, const QString& icon, int min, int max, int unit/*=1*/) : AbstractControl(title, desc, icon)
{

    m_params = params;
    m_min = min;
    m_max = max;
    m_unit = unit;

    label.setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    label.setStyleSheet("color: #e0e879");
    hlayout->addWidget(&label);

    btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
    btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
    btnminus.setFixedSize(150, 100);
    btnplus.setFixedSize(150, 100);
    hlayout->addWidget(&btnminus);
    hlayout->addWidget(&btnplus);

    QObject::connect(&btnminus, &QPushButton::released, [=]() {
        auto str = QString::fromStdString(Params().get(m_params.toStdString()));
        int value = str.toInt();
        value = value - m_unit;
        if (value < m_min) {
            value = m_min;
        }
        else {
        }

        //UIScene& scene = uiState()->scene;//QUIState::ui_state.scene;
        //scene.scr.autoFocus = value;
        QString values = QString::number(value);
        Params().put(m_params.toStdString(), values.toStdString());
        refresh();
    });

    QObject::connect(&btnplus, &QPushButton::released, [=]() {
        auto str = QString::fromStdString(Params().get(m_params.toStdString()));
        int value = str.toInt();
        value = value + m_unit;
        if (value > m_max) {
            value = m_max;
        }
        else {
        }

        //UIScene& scene = uiState()->scene;//QUIState::ui_state.scene;
        //scene.scr.autoFocus = value;
        QString values = QString::number(value);
        Params().put(m_params.toStdString(), values.toStdString());
        refresh();
    });
    refresh();
}
TimeZoneSelectCombo::TimeZoneSelectCombo() : AbstractControl("TZ", "", "../assets/offroad/icon_shell.png") 
{
  combobox.setStyleSheet(R"(
    subcontrol-origin: padding;
    subcontrol-position: top left;
    selection-background-color: #111;
    selection-color: yellow;
    color: white;
    background-color: #393939;
    border-style: solid;
    border: 0px solid #1e1e1e;
    border-radius: 0;
    width: 100px;
  )");

  combobox.addItem(tr("Select Your TimeZone"));
  QFile timezonelistfile("/data/openpilot/selfdrive/assets/addon/param/TimeZone");
  if (timezonelistfile.open(QIODevice::ReadOnly)) {
    QTextStream timezonename(&timezonelistfile);
    while (!timezonename.atEnd()) {
      QString line = timezonename.readLine();
      combobox.addItem(line);
    }
    timezonelistfile.close();
  }

  combobox.setFixedWidth(950);

  btn.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btn.setFixedSize(150, 100);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    if (btn.text() == tr("UNSET")) {
      if (ConfirmationDialog::confirm(tr("Do you want to set default?"), tr("Reboot"), this)) {
        params.put("OPKRTimeZone", "UTC");
        combobox.setCurrentIndex(0);
        refresh();
      }
    }
  });

  //combobox.view()->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  hlayout->addWidget(&combobox, Qt::AlignLeft);
  hlayout->addWidget(&btn, Qt::AlignRight);

  QObject::connect(&combobox, static_cast<void(QComboBox::*)(int)>(&QComboBox::activated), [=](int index)
  {
    combobox.itemData(combobox.currentIndex());
    QString str = combobox.currentText();
    if (combobox.currentIndex() != 0) {
      if (ConfirmationDialog::confirm(tr("Press OK to set your timezone as") + "\n" + str, tr("Reboot"), this)) {
        params.put("OPKRTimeZone", str.toStdString());
      }
    }
    refresh();
  });
  refresh();
}

void TimeZoneSelectCombo::refresh() {
  QString selected_timezonename = QString::fromStdString(params.get("OPKRTimeZone"));
  int index = combobox.findText(selected_timezonename);
  if (index >= 0) combobox.setCurrentIndex(index);
  if (selected_timezonename.length()) {
    btn.setEnabled(true);
    btn.setText(tr("UNSET"));
  } else {
    btn.setEnabled(false);
    btn.setText(tr("SET"));
  }
}

void CValueControl::refresh()
{
    label.setText(QString::fromStdString(Params().get(m_params.toStdString())));
    btnminus.setText("－");
    btnplus.setText("＋");
}

GitHash::GitHash() : AbstractControl("커밋(로컬/리모트)", "", "") {

    QString lhash = QString::fromStdString(Params().get("GitCommit").substr(0, 10));
    QString rhash = QString::fromStdString(Params().get("GitCommitRemote").substr(0, 10));
    hlayout->addStretch(2);

    local_hash.setText(QString::fromStdString(Params().get("GitCommit").substr(0, 10)));
    remote_hash.setText(QString::fromStdString(Params().get("GitCommitRemote").substr(0, 10)));
    local_hash.setAlignment(Qt::AlignVCenter);
    remote_hash.setAlignment(Qt::AlignVCenter);
    local_hash.setStyleSheet("color: #aaaaaa");
    if (lhash == rhash) {
        remote_hash.setStyleSheet("color: #aaaaaa");
    }
    else {
        remote_hash.setStyleSheet("color: #0099ff");
    }
    hlayout->addWidget(&local_hash);
    hlayout->addWidget(&remote_hash);
}
SelectCar::SelectCar(QWidget* parent): QWidget(parent) {

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setMargin(20);
  main_layout->setSpacing(20);

  // Back button
  QPushButton* back = new QPushButton(tr("Back"));
  back->setObjectName("back_btn");
  back->setFixedSize(500, 100);
  connect(back, &QPushButton::clicked, [=]() { emit backPress(); });
  main_layout->addWidget(back, 0, Qt::AlignLeft);

  QListWidget* list = new QListWidget(this);
  list->setStyleSheet("QListView {padding: 40px; background-color: #393939; border-radius: 15px; height: 140px;} QListView::item{height: 100px}");
  //list->setAttribute(Qt::WA_AcceptTouchEvents, true);
  QScroller::grabGesture(list->viewport(), QScroller::LeftMouseButtonGesture);
  list->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

  list->addItem(tr("[ Not selected ]"));

  QStringList items = get_list("/data/params/d/SupportedCars");
  QStringList items_gm = get_list("/data/params/d/SupportedCars_gm");
  list->addItems(items);
  list->addItems(items_gm);
  list->setCurrentRow(0);

  QString selected = QString::fromStdString(Params().get("SelectedCar"));

  int index = 0;
  for(QString item : items) {
    if(selected == item) {
        list->setCurrentRow(index + 1);
        break;
    }
    index++;
  }

  QObject::connect(list, QOverload<QListWidgetItem*>::of(&QListWidget::itemClicked),
    [=](QListWidgetItem* item){

    if(list->currentRow() == 0)
        Params().remove("SelectedCar");
    else
        Params().put("SelectedCar", list->currentItem()->text().toStdString());

    emit selectedCar();
    });

  main_layout->addWidget(list);
}
