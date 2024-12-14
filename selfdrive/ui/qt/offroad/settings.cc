#include <cassert>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include <QDebug>

#include "common/watchdog.h"
#include "common/util.h"
#include "selfdrive/ui/qt/offroad/driverview.h"
#include "selfdrive/ui/qt/network/networking.h"
#include "selfdrive/ui/qt/offroad/settings.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/prime.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/offroad/developer_panel.h"

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
      "ExperimentalLongitudinalEnabled",
      tr("openpilot Longitudinal Control (Alpha)"),
      QString("<b>%1</b><br><br>%2")
      .arg(tr("WARNING: openpilot longitudinal control is in alpha for this car and will disable Automatic Emergency Braking (AEB)."))
      .arg(tr("On this car, openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. "
              "Enable this to switch to openpilot longitudinal control. Enabling Experimental mode is recommended when enabling openpilot longitudinal control alpha.")),
      "../assets/offroad/icon_speed_limit.png",
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
  toggles["ExperimentalLongitudinalEnabled"]->setConfirmation(true, false);

  connect(toggles["ExperimentalLongitudinalEnabled"], &ToggleControl::toggleFlipped, [=]() {
    updateToggles();
  });
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
  auto op_long_toggle = toggles["ExperimentalLongitudinalEnabled"];
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

    if (!CP.getExperimentalLongitudinalAvailable() || is_release) {
      params.remove("ExperimentalLongitudinalEnabled");
    }
    op_long_toggle->setVisible(CP.getExperimentalLongitudinalAvailable() && !is_release);
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
      if (CP.getExperimentalLongitudinalAvailable()) {
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
    op_long_toggle->setVisible(false);
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

  QPushButton* poweroff_btn = new QPushButton(tr("Power Off"));
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);

  if (false && !Hardware::PC()) {
      connect(uiState(), &UIState::offroadTransition, poweroff_btn, &QPushButton::setVisible);
  }

  setStyleSheet(R"(
    #reboot_btn { height: 120px; border-radius: 15px; background-color: #2CE22C; }
    #reboot_btn:pressed { background-color: #24FF24; }
    #poweroff_btn { height: 120px; border-radius: 15px; background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
  addItem(power_layout);


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
  connect(dcamBtn, &ButtonControl::clicked, [this, dcamBtn]() {
    dcamBtn->setEnabled(false);
    DriverViewDialog driver_view(this);
    driver_view.exec();
    dcamBtn->setEnabled(true);
  });
  addItem(dcamBtn);

  auto resetCalibBtn = new ButtonControl(tr("Reset Calibration"), tr("RESET"), "");
  connect(resetCalibBtn, &ButtonControl::showDescriptionEvent, this, &DevicePanel::updateCalibDescription);
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration?"), tr("Reset"), this)) {
      params.remove("CalibrationParams");
      params.remove("LiveTorqueParameters");
      emit parent->closeSettings();
      QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", true);
      });
    }
  });
  addItem(resetCalibBtn);

  auto retrainingBtn = new ButtonControl(tr("Review Training Guide"), tr("REVIEW"), tr("Review the rules, features, and limitations of openpilot"));
  connect(retrainingBtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to review the training guide?"), tr("Review"), this)) {
      emit reviewTrainingGuide();
    }
  });
  addItem(retrainingBtn);

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
    resetCalibBtn->setEnabled(true);
    translateBtn->setEnabled(true);
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

  TogglesPanel *toggles = new TogglesPanel(this);
  QObject::connect(this, &SettingsWindow::expandToggleDescription, toggles, &TogglesPanel::expandToggleDescription);

  auto networking = new Networking(this);
  QObject::connect(uiState()->prime_state, &PrimeState::changed, networking, &Networking::setPrimeType);

  QList<QPair<QString, QWidget *>> panels = {
    {tr("Device"), device},
    {tr("Network"), networking},
    {tr("Toggles"), toggles},
    {tr("Software"), new SoftwarePanel(this)},
    {tr("Carrot"), new CarrotPanel(this)},
    {tr("Developer"), new DeveloperPanel(this)},
  };

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

static QStringList get_list(const char* path)
{
    QStringList stringList;
    QFile textFile(path);
    if (textFile.open(QIODevice::ReadOnly))
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

CarrotPanel::CarrotPanel(QWidget* parent) : QWidget(parent) {

    main_layout = new QStackedLayout(this);
    homeScreen = new QWidget(this);
    carrotLayout = new QVBoxLayout(homeScreen);
    carrotLayout->setMargin(40);

    QHBoxLayout* select_layout = new QHBoxLayout();
    select_layout->setSpacing(30);

    QPushButton* cruise_btn = new QPushButton(tr("Cruise"));
    cruise_btn->setObjectName("cruise_btn");
    QObject::connect(cruise_btn, &QPushButton::clicked, this, [this]() {this->togglesCarrot(0); });

    QPushButton* latLong_btn = new QPushButton(tr("Tuning"));
    latLong_btn->setObjectName("latLong_btn");
    QObject::connect(latLong_btn, &QPushButton::clicked, this, [this]() {this->togglesCarrot(1); });

    QPushButton* disp_btn = new QPushButton(tr("Disp"));
    disp_btn->setObjectName("disp_btn");
    QObject::connect(disp_btn, &QPushButton::clicked, this, [this]() {this->togglesCarrot(2); });

    QPushButton* path_btn = new QPushButton(tr("Path"));
    path_btn->setObjectName("path_btn");
    QObject::connect(path_btn, &QPushButton::clicked, this, [this]() {this->togglesCarrot(3); });

    QPushButton* start_btn = new QPushButton(tr("Start"));
    start_btn->setObjectName("start_btn");
    QObject::connect(start_btn, &QPushButton::clicked, this, [this]() {this->togglesCarrot(4); });

    QPushButton* speed_btn = new QPushButton(tr("Speed"));
    speed_btn->setObjectName("speed_btn");
    QObject::connect(speed_btn, &QPushButton::clicked, this, [this]() {this->togglesCarrot(5); });

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
    cruiseToggles->addItem(new CValueControl("CruiseButtonMode", "Button: Cruise Button Mode", "0:Normal,1:User1,2:User2", "../assets/offroad/icon_road.png", 0, 2, 1));
    cruiseToggles->addItem(new CValueControl("CruiseSpeedUnit", "Button: Cruise Speed Unit", "", "../assets/offroad/icon_road.png", 1, 20, 1));
    //cruiseToggles->addItem(new CValueControl("CruiseEcoControl", "CRUISE: Eco control(4km/h)", "Temporarily increasing the set speed to improve fuel efficiency.", "../assets/offroad/icon_road.png", 0, 10, 1));
    //cruiseToggles->addItem(new CValueControl("CruiseSpeedMin", "CRUISE: Speed Lower limit(10)", "Cruise control MIN speed", "../assets/offroad/icon_road.png", 5, 50, 1));
    cruiseToggles->addItem(new CValueControl("AutoSpeedUptoRoadSpeedLimit", "CRUISE: Auto speed up (0%)", "Auto speed up based on the lead car upto RoadSpeedLimit.", "../assets/offroad/icon_road.png", 0, 200, 10));
    //cruiseToggles->addItem(new CValueControl("AutoResumeFromGas", "GAS CRUISE ON: Use", "Auto Cruise on when GAS pedal released, 60% Gas Cruise On automatically", "../assets/offroad/icon_road.png", 0, 3, 1));
    //cruiseToggles->addItem(new CValueControl("AutoResumeFromGasSpeed", "GAS CRUISE ON: Speed(30)", "Driving speed exceeds the set value, Cruise ON", "../assets/offroad/icon_road.png", 20, 140, 5));
    //cruiseToggles->addItem(new CValueControl("TFollowSpeedAddM", "GAP: Additinal TFs 40km/h(0)x0.01s", "Speed-dependent additinal max(100km/h) TFs", "../assets/offroad/icon_road.png", -100, 200, 5));
    //cruiseToggles->addItem(new CValueControl("TFollowSpeedAdd", "GAP: Additinal TFs 100Km/h(0)x0.01s", "Speed-dependent additinal max(100km/h) TFs", "../assets/offroad/icon_road.png", -100, 200, 5));
    cruiseToggles->addItem(new CValueControl("TFollowGap1", "GAP1: Apply TFollow (110)x0.01s", "", "../assets/offroad/icon_road.png", 70, 300, 5));
    cruiseToggles->addItem(new CValueControl("TFollowGap2", "GAP2: Apply TFollow (120)x0.01s", "", "../assets/offroad/icon_road.png", 70, 300, 5));
    cruiseToggles->addItem(new CValueControl("TFollowGap3", "GAP3: Apply TFollow (160)x0.01s", "", "../assets/offroad/icon_road.png", 70, 300, 5));
    cruiseToggles->addItem(new CValueControl("TFollowGap4", "GAP4: Apply TFollow (180)x0.01s", "", "../assets/offroad/icon_road.png", 70, 300, 5));
    cruiseToggles->addItem(new CValueControl("DynamicTFollow", "Dynamic GAP control", "", "../assets/offroad/icon_road.png", 0, 100, 5));
    cruiseToggles->addItem(new CValueControl("DynamicTFollowLC", "Dynamic GAP control (LaneChange)", "", "../assets/offroad/icon_road.png", 0, 100, 5));
    cruiseToggles->addItem(new CValueControl("MyDrivingMode", "DRIVEMODE: Select", "1:ECO,2:SAFE,3:NORMAL,4:HIGH", "../assets/offroad/icon_road.png", 1, 4, 1));
    //cruiseToggles->addItem(new CValueControl("MyEcoModeFactor", "DRIVEMODE: ECO Accel ratio(80%)", "Acceleartion ratio in ECO mode", "../assets/offroad/icon_road.png", 10, 95, 5));
    //cruiseToggles->addItem(new CValueControl("MySafeModeFactor", "DRIVEMODE: SAFE ratio(60%)", "Accel/StopDistance/DecelRatio/Gap control ratio", "../assets/offroad/icon_road.png", 10, 90, 10));
    //cruiseToggles->addItem(new CValueControl("MyHighModeFactor", "DRIVEMODE: HIGH ratio(100%)", "AccelRatio control ratio", "../assets/offroad/icon_road.png", 100, 300, 10));

    latLongToggles = new ListWidget(this);
    //latLongToggles->addItem(new CValueControl("AutoLaneChangeSpeed", "LaneChangeSpeed(20)", "", "../assets/offroad/icon_road.png", 1, 100, 5));
    latLongToggles->addItem(new CValueControl("UseLaneLineSpeed", "Laneline mode speed(0)", "Laneline mode, lat_mpc control used", "../assets/offroad/icon_logic.png", 0, 200, 5));
    latLongToggles->addItem(new CValueControl("UseLaneLineCurveSpeed", "Laneline mode curve speed(0)", "Laneline mode, high speed only", "../assets/offroad/icon_logic.png", 0, 200, 5));
    latLongToggles->addItem(new CValueControl("AdjustLaneOffset", "AdjustLaneOffset(0)cm", "", "../assets/offroad/icon_logic.png", 0, 500, 5));
    latLongToggles->addItem(new CValueControl("AdjustCurveOffset", "AdjustLaneCurveOffset(0)cm", "", "../assets/offroad/icon_logic.png", 0, 500, 5));
    latLongToggles->addItem(new CValueControl("AdjustLaneTime", "AdjustLaneTimeOffset(5)x0.01s", "", "../assets/offroad/icon_logic.png", 0, 20, 1));
    latLongToggles->addItem(new CValueControl("CustomSR", "LAT: SteerRatiox0.1(0)", "Custom SteerRatio", "../assets/offroad/icon_logic.png", 0, 300, 1));
    latLongToggles->addItem(new CValueControl("SteerRatioRate", "LAT: SteerRatioRatex0.01(100)", "SteerRatio apply rate", "../assets/offroad/icon_logic.png", 30, 170, 1));
    //latLongToggles->addItem(new CValueControl("PathOffset", "PathOffset", "(-)left, (+)right, when UseLaneLineSpeed > 0", "../assets/offroad/icon_road.png", -50, 50, 1));
    //latLongToggles->addItem(horizontal_line());
    //latLongToggles->addItem(new CValueControl("JerkStartLimit", "LONG: JERK START(10)x0.1", "Starting Jerk.", "../assets/offroad/icon_road.png", 1, 50, 1));
    //latLongToggles->addItem(new CValueControl("LongitudinalTuningApi", "LONG: ControlType", "0:velocity pid, 1:accel pid, 2:accel pid(comma)", "../assets/offroad/icon_road.png", 0, 2, 1));
    latLongToggles->addItem(new CValueControl("LongTuningKpV", "LONG: P Gain(100)", "", "../assets/offroad/icon_logic.png", 0, 150, 5));
    latLongToggles->addItem(new CValueControl("LongTuningKiV", "LONG: I Gain(200)", "", "../assets/offroad/icon_logic.png", 0, 2000, 5));
    latLongToggles->addItem(new CValueControl("LongTuningKf", "LONG: FF Gain(100)", "", "../assets/offroad/icon_logic.png", 0, 200, 5));
    latLongToggles->addItem(new CValueControl("LongActuatorDelay", "LONG: ActuatorDelay(20)", "", "../assets/offroad/icon_logic.png", 0, 200, 5));
    latLongToggles->addItem(new CValueControl("RadarReactionFactor", "LONG: Radar reaction factor(10)", "", "../assets/offroad/icon_logic.png", 0, 200, 10));
    //latLongToggles->addItem(new CValueControl("StartAccelApply", "LONG: StartingAccel 2.0x(0)%", "정지->출발시 가속도의 가속율을 지정합니다 0: 사용안함.", "../assets/offroad/icon_road.png", 0, 100, 10));
    //latLongToggles->addItem(new CValueControl("StopAccelApply", "LONG: StoppingAccel -2.0x(0)%", "정지유지시 브레이크압을 조정합니다. 0: 사용안함. ", "../assets/offroad/icon_road.png", 0, 100, 10));
    latLongToggles->addItem(new CValueControl("LaneChangeNeedTorque", "LaneChange need torque", "", "../assets/offroad/icon_logic.png", 0, 1, 1));
    latLongToggles->addItem(new CValueControl("StoppingAccel", "LONG: StoppingStartAccelx0.01(-40)", "", "../assets/offroad/icon_logic.png", -100, 0, 5));
    latLongToggles->addItem(new CValueControl("StopDistanceCarrot", "LONG: StopDistance (600)cm", "", "../assets/offroad/icon_logic.png", 300, 1000, 10));
    //latLongToggles->addItem(new CValueControl("TraffStopDistanceAdjust", "LONG: TrafficStopDistance adjust(150)cm", "", "../assets/offroad/icon_road.png", -1000, 1000, 10));
    latLongToggles->addItem(new CValueControl("ComfortBrake", "LONG: Comfort Brake (240)", "x0.01", "../assets/offroad/icon_logic.png", 200, 300, 1));
    latLongToggles->addItem(new CValueControl("CruiseMaxVals1", "ACCEL:0km/h(160)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_logic.png", 1, 250, 5));
    latLongToggles->addItem(new CValueControl("CruiseMaxVals2", "ACCEL:40km/h(120)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_logic.png", 1, 250, 5));
    latLongToggles->addItem(new CValueControl("CruiseMaxVals3", "ACCEL:60km/h(100)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_logic.png", 1, 250, 5));
    latLongToggles->addItem(new CValueControl("CruiseMaxVals4", "ACCEL:80km/h(80)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_logic.png", 1, 250, 5));
    latLongToggles->addItem(new CValueControl("CruiseMaxVals5", "ACCEL:110km/h(70)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_logic.png", 1, 250, 5));
    latLongToggles->addItem(new CValueControl("CruiseMaxVals6", "ACCEL:140km/h(60)", "속도별 가속도를 지정합니다.(x0.01m/s^2)", "../assets/offroad/icon_logic.png", 1, 250, 5));
    //latLongToggles->addItem(new CValueControl("CruiseMinVals", "DECEL:(120)", "감속도를 설정합니다.(x0.01m/s^2)", "../assets/offroad/icon_road.png", 50, 250, 5));
    latLongToggles->addItem(new CValueControl("MaxAngleFrames", "MaxAngleFrames(89)", "89:기본, 스티어계기판에러시 85~87", "../assets/offroad/icon_logic.png", 80, 100, 1));
    latLongToggles->addItem(new CValueControl("SteerActuatorDelay", "LAT:SteerActuatorDelay(40)", "표준", "../assets/offroad/icon_logic.png", 1, 100, 1));
    latLongToggles->addItem(new CValueControl("LateralTorqueCustom", "LAT: TorqueCustom(0)", "", "../assets/offroad/icon_logic.png", 0, 2, 1));
    latLongToggles->addItem(new CValueControl("LateralTorqueAccelFactor", "LAT: TorqueAccelFactor(2500)", "", "../assets/offroad/icon_logic.png", 1000, 6000, 10));
    latLongToggles->addItem(new CValueControl("LateralTorqueFriction", "LAT: TorqueFriction(100)", "", "../assets/offroad/icon_logic.png", 0, 1000, 10));
    latLongToggles->addItem(new CValueControl("CustomSteerMax", "LAT: CustomSteerMax(0)", "", "../assets/offroad/icon_logic.png", 0, 30000, 5));
    latLongToggles->addItem(new CValueControl("CustomSteerDeltaUp", "LAT: CustomSteerDeltaUp(0)", "", "../assets/offroad/icon_logic.png", 0, 50, 1));
    latLongToggles->addItem(new CValueControl("CustomSteerDeltaDown", "LAT: CustomSteerDeltaDown(0)", "", "../assets/offroad/icon_logic.png", 0, 50, 1));

    dispToggles = new ListWidget(this);
    //dispToggles->addItem(new CValueControl("ShowHudMode", "DISP:Display Mode", "0:Frog,1:APilot,2:Bottom,3:Top,4:Left,5:Left-Bottom", "../assets/offroad/icon_shell.png", 0, 5, 1));
    dispToggles->addItem(new CValueControl("ShowDebugUI", "DISP:Debug Info", "", "../assets/offroad/icon_shell.png", 0, 2, 1));
    dispToggles->addItem(new CValueControl("ShowDateTime", "DISP:Time Info", "0:None,1:Time/Date,2:Time,3:Date", "../assets/offroad/icon_calendar.png", 0, 3, 1));
    //dispToggles->addItem(new CValueControl("ShowSteerRotate", "DISP:Handle rotate", "0:None,1:Rotate", "../assets/offroad/icon_shell.png", 0, 1, 1));
    dispToggles->addItem(new CValueControl("ShowPathEnd", "DISP:Path End", "0:None,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
    //dispToggles->addItem(new CValueControl("ShowAccelRpm", "DISP:Accel meter", "0:None,1:Display,1:Accel+RPM", "../assets/offroad/icon_shell.png", 0, 2, 1));
    //dispToggles->addItem(new CValueControl("ShowTpms", "DISP:TPMS", "0:None,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
    //dispToggles->addItem(new CValueControl("ShowSteerMode", "DISP:Handle Display Mode", "0:Black,1:Color,2:None", "../assets/offroad/icon_shell.png", 0, 2, 1));
    //dispToggles->addItem(new CValueControl("ShowDeviceState", "DISP:Device State", "0:None,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
    //dispToggles->addItem(new CValueControl("ShowConnInfo", "DISP:APM connection", "0:NOne,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
    dispToggles->addItem(new CValueControl("ShowLaneInfo", "DISP:Lane Info", "-1:None, 0:Path, 1:Path+Lane, 2: Path+Lane+RoadEdge", "../assets/offroad/icon_shell.png", -1, 2, 1));
    //dispToggles->addItem(new CValueControl("ShowBlindSpot", "DISP:BSD Info", "0:None,1:Display", "../assets/offroad/icon_shell.png", 0, 1, 1));
    //dispToggles->addItem(new CValueControl("ShowGapInfo", "DISP:GAP Info", "0:None,1:Display", "../assets/offroad/icon_shell.png", -1, 1, 1));
    //dispToggles->addItem(new CValueControl("ShowDmInfo", "DISP:DM Info", "0:None,1:Display,-1:Disable(Reboot)", "../assets/offroad/icon_shell.png", -1, 1, 1));
    dispToggles->addItem(new CValueControl("ShowRadarInfo", "DISP:Radar Info", "0:None,1:Display,2:RelPos,3:Stopped Car", "../assets/offroad/icon_shell.png", 0, 3, 1));
    dispToggles->addItem(new CValueControl("ShowPlotMode", "DISP:Debug plot", "", "../assets/offroad/icon_shell.png", 0, 10, 1));
    dispToggles->addItem(new CValueControl("ShowCustomBrightness", "Brightness ratio", "", "../assets/offroad/icon_brightness.png", 0, 100, 10));

    pathToggles = new ListWidget(this);
    pathToggles->addItem(new CValueControl("ShowPathModeCruiseOff", "DISP: Path Mode: Cruise OFFF", "0:Normal,1,2:Rec,3,4:^^,5,6:Rec,7,8:^^,9,10,11,12:Smooth^^", "../assets/offroad/icon_shell.png", 0, 15, 1));
    pathToggles->addItem(new CValueControl("ShowPathColorCruiseOff", "DISP: Path Color: Cruise OFF", "(+10:Stroke)0:Red,1:Orange,2:Yellow,3:Green,4:Blue,5:Indigo,6:Violet,7:Brown,8:White,9:Black", "../assets/offroad/icon_shell.png", 0, 19, 1));
    pathToggles->addItem(new CValueControl("ShowPathMode", "DISP:Path Mode: Laneless", "0:Normal,1,2:Rec,3,4:^^,5,6:Rec,7,8:^^,9,10,11,12:Smooth^^", "../assets/offroad/icon_shell.png", 0, 15, 1));
    pathToggles->addItem(new CValueControl("ShowPathColor", "DISP:Path Color: Laneless", "(+10:Stroke)0:Red,1:Orange,2:Yellow,3:Green,4:Blue,5:Indigo,6:Violet,7:Brown,8:White,9:Black", "../assets/offroad/icon_shell.png", 0, 19, 1));
    pathToggles->addItem(new CValueControl("ShowPathModeLane", "DISP:Path Mode: LaneMode", "0:Normal,1,2:Rec,3,4:^^,5,6:Rec,7,8:^^,9,10,11,12:Smooth^^", "../assets/offroad/icon_shell.png", 0, 15, 1));
    pathToggles->addItem(new CValueControl("ShowPathColorLane", "DISP:Path Color: LaneMode", "(+10:Stroke)0:Red,1:Orange,2:Yellow,3:Green,4:Blue,5:Indigo,6:Violet,7:Brown,8:White,9:Black", "../assets/offroad/icon_shell.png", 0, 19, 1));
    pathToggles->addItem(new CValueControl("ShowPathWidth", "DISP:Path Width ratio(100%)", "", "../assets/offroad/icon_shell.png", 10, 200, 10));

    startToggles = new ListWidget(this);
    QString selected = QString::fromStdString(Params().get("CarSelected3"));
    QPushButton* selectCarBtn = new QPushButton(selected.length() > 1 ? selected : tr("SELECT YOUR CAR"));
    selectCarBtn->setObjectName("selectCarBtn");
    selectCarBtn->setStyleSheet(R"(
      QPushButton {
        margin-top: 20px;  /* 위쪽 여백 */
        margin-bottom: 20px;  /* 아래쪽 여백 */
        padding: 10px;  /* 버튼 안쪽 패딩 */
        height: 120px;  /* 버튼 높이 */
        border-radius: 15px;  /* 버튼 모서리 둥글게 */
        color: #FFFFFF;  /* 버튼 색상 */
        background-color: #2C2CE2;  /* 버튼 배경색 */
      }
      QPushButton:pressed {
        background-color: #2424FF;  /* 버튼 클릭 시 배경색 */
      }
    )");
    //selectCarBtn->setFixedSize(350, 100);
    connect(selectCarBtn, &QPushButton::clicked, [=]() {
      QString selected = QString::fromStdString(Params().get("CarSelected3"));

      QStringList cars = {"[ Not Selected ]"};
      QStringList items = get_list("/data/params/d/SupportedCars");
      QStringList items_gm = get_list("/data/params/d/SupportedCars_gm");
      QStringList items_toyota = get_list("/data/params/d/SupportedCars_toyota");
      cars.append(items);
      cars.append(items_gm);
      cars.append(items_toyota);

      QString selectedCar = MultiOptionDialog::getSelection("Select your car", cars, ((selected == "[ Not Selected ]") || (selected.length() == 0)) ? cars[0] : selected, this);
      if (!selectedCar.isEmpty()) {
        if (selectedCar == "[ Not Selected ]") {
          Params().remove("CarSelected3");
        } else {
          Params().put("CarSelected3", selectedCar.toStdString());
          ConfirmationDialog::alert(selectedCar, this);
        }
      }

      selected = QString::fromStdString(Params().get("CarSelected3"));
      selectCarBtn->setText(((selected == "[ Not Selected ]") || (selected.length() == 0)) ? tr("SELECT YOUR CAR") : selected);
    });

    startToggles->addItem(selectCarBtn);
    startToggles->addItem(new ParamControl("HyundaiCameraSCC", "HYUNDAI: CAMERA SCC", "Connect the SCC's CAN line to CAM", "../assets/offroad/icon_shell.png", this));
    startToggles->addItem(new ParamControl("EnableRadarTracks", "Enable RadarTrack", "", "../assets/offroad/icon_shell.png", this));
    startToggles->addItem(new ParamControl("CanfdHDA2", "CANFD: HDA2 mode", "", "../assets/offroad/icon_shell.png", this));
    startToggles->addItem(new CValueControl("AutoCruiseControl", "Auto Cruise control", "Softhold, Auto Cruise ON/OFF control", "../assets/offroad/icon_road.png", 0, 3, 1));
    startToggles->addItem(new CValueControl("CruiseOnDist", "CRUISE: Auto ON distance(0cm)", "When GAS/Brake is OFF, Cruise ON when the lead car gets closer.", "../assets/offroad/icon_road.png", 0, 2500, 50));
    startToggles->addItem(new CValueControl("AutoEngage", "Auto Engage control on start", "1:SteerEnable, 2:Steer/Cruise Engage", "../assets/offroad/icon_road.png", 0, 2, 1));
    startToggles->addItem(new ParamControl("DisableMinSteerSpeed", "Disable Min.SteerSpeed", "", "../assets/offroad/icon_road.png", this));
    startToggles->addItem(new CValueControl("AutoGasTokSpeed", "Auto AccelTok speed", "Gas(Accel)Tok enable speed", "../assets/offroad/icon_road.png", 0, 200, 5));
    startToggles->addItem(new ParamControl("AutoGasSyncSpeed", "Auto update Cruise speed", "", "../assets/offroad/icon_road.png", this));
    startToggles->addItem(new CValueControl("SpeedFromPCM", "Read Cruise Speed from PCM", "Toyota must set to 1, Honda 3", "../assets/offroad/icon_road.png", 0, 3, 1));
    startToggles->addItem(new CValueControl("SoundVolumeAdjust", "Adjust Sound Volume(100%)", "", "../assets/offroad/icon_sound.png", 5, 200, 5));
    startToggles->addItem(new CValueControl("SoundVolumeAdjustEngage", "Adjust Sound Volume, Engage(10%)", "", "../assets/offroad/icon_sound.png", 5, 200, 5));
    startToggles->addItem(new CValueControl("MaxTimeOffroadMin", "Power off time (min)", "", "../assets/offroad/icon_sandtimer.png", 1, 600, 10));
    startToggles->addItem(new ParamControl("DisableDM", "Disable DM", "", "../assets/img_driver_face_static_x.png", this));
    //startToggles->addItem(new CValueControl("CarrotCountDownSpeed", "NaviCountDown Speed(10)", "", "../assets/offroad/icon_shell.png", 0, 200, 5));
    //startToggles->addItem(new CValueControl("MapboxStyle", "Mapbox Style(0)", "", "../assets/offroad/icon_shell.png", 0, 2, 1));
    startToggles->addItem(new ParamControl("HotspotOnBoot", "Hotspot enabled on boot", "", "../assets/offroad/icon_shell.png", this));
    //startToggles->addItem(new ParamControl("NoLogging", "Disable Logger", "", "../assets/offroad/icon_shell.png", this));
    //startToggles->addItem(new ParamControl("LaneChangeNeedTorque", "LaneChange: Need Torque", "", "../assets/offroad/icon_shell.png", this));
    //startToggles->addItem(new CValueControl("LaneChangeLaneCheck", "LaneChange: Check lane exist", "(0:No,1:Lane,2:+Edge)", "../assets/offroad/icon_shell.png", 0, 2, 1));

    speedToggles = new ListWidget(this);
    speedToggles->addItem(new CValueControl("AutoCurveSpeedLowerLimit", "CURVE: Lower limit speed(30)", "곡선도로를 만나면 속도를 줄여줍니다. 최저속도", "../assets/offroad/icon_road.png", 30, 200, 5));
    speedToggles->addItem(new CValueControl("AutoCurveSpeedFactor", "CURVE: Auto Control ratio(100%)", "", "../assets/offroad/icon_road.png", 50, 300, 1));
    speedToggles->addItem(new CValueControl("AutoCurveSpeedAggressiveness", "CURVE: Aggressiveness (100%)", "", "../assets/offroad/icon_road.png", 50, 300, 1));
    speedToggles->addItem(new CValueControl("AutoNaviSpeedCtrlEnd", "SpeedCameraDecelEnd(6s)", "감속완료시점을 설정합니다.값이 크면 카메라에서 멀리 감속 완료", "../assets/offroad/icon_road.png", 3, 20, 1));
    speedToggles->addItem(new CValueControl("AutoNaviSpeedDecelRate", "SpeedCameraDecelRatex0.01m/s^2(80)", "낮으면 멀리서부터 감속함", "../assets/offroad/icon_road.png", 10, 200, 10));
    speedToggles->addItem(new CValueControl("AutoNaviSpeedSafetyFactor", "SpeedCameraSafetyFactor(105%)", "", "../assets/offroad/icon_road.png", 80, 120, 1));
    speedToggles->addItem(new CValueControl("AutoNaviSpeedBumpTime", "SpeedBumpTimeDistance(1s)", "", "../assets/offroad/icon_road.png", 1, 50, 1));
    speedToggles->addItem(new CValueControl("AutoNaviSpeedBumpSpeed", "SpeedBumpSpeed(35Km/h)", "", "../assets/offroad/icon_road.png", 10, 100, 5));
    speedToggles->addItem(new CValueControl("AutoNaviCountDownMode", "NaviCountDown mode(2)", "0: off, 1:tbt+camera, 2:tbt+camera+bump", "../assets/offroad/icon_road.png", 0, 2, 1));
    speedToggles->addItem(new CValueControl("TurnSpeedControlMode", "Turn Speed control mode(1)", "0: off, 1:vision, 2:vision+route, 3: route", "../assets/offroad/icon_road.png", 0, 3, 1));
    speedToggles->addItem(new CValueControl("MapTurnSpeedFactor", "Map TurnSpeed Factor(100)", "", "../assets/offroad/icon_map.png", 50, 300, 5));
    speedToggles->addItem(new CValueControl("AutoTurnControl", "ATC: Auto turn control(0)", "0:없음,1:차선변경,2:차선변경+속도,3:속도", "../assets/offroad/icon_road.png", 0, 3, 1));
    speedToggles->addItem(new CValueControl("AutoTurnControlSpeedTurn", "ATC: Turn Speed (20)", "0:없음, 턴속도", "../assets/offroad/icon_road.png", 0, 100, 5));
    speedToggles->addItem(new CValueControl("AutoTurnControlTurnEnd", "ATC: Turn CtrlDistTime (6)", "dist=speed*time", "../assets/offroad/icon_road.png", 0, 30, 1));
    speedToggles->addItem(new CValueControl("AutoRoadSpeedAdjust", "Auto Roadlimit Speed adjust (50%)", "", "../assets/offroad/icon_road.png", 0, 100, 10));
    //speedToggles->addItem(new CValueControl("AutoTurnMapChange", "NOO Helper Auto Map Change(0)", "", "../assets/offroad/icon_road.png", 0, 2, 1));

    toggles_layout->addWidget(cruiseToggles);
    toggles_layout->addWidget(latLongToggles);
    toggles_layout->addWidget(dispToggles);
    toggles_layout->addWidget(pathToggles);
    toggles_layout->addWidget(startToggles);
    toggles_layout->addWidget(speedToggles);
    ScrollView* toggles_view = new ScrollView(toggles, this);
    carrotLayout->addWidget(toggles_view, 1);

    homeScreen->setLayout(carrotLayout);
    main_layout->addWidget(homeScreen);
    main_layout->setCurrentWidget(homeScreen);

    togglesCarrot(4);

    setStyleSheet(R"(
    #start_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #start_btn:pressed { background-color: #4a4a4a; }
    #cruise_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #cruise_btn:pressed { background-color: #4a4a4a; }
    #speed_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #speed_btn:pressed { background-color: #4a4a4a; }
    #latLong_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #latLong_btn:pressed { background-color: #4a4a4a; }
    #disp_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #disp_btn:pressed { background-color: #4a4a4a; }
    #path_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #path_btn:pressed { background-color: #4a4a4a; }
  )");

}
void CarrotPanel::togglesCarrot(int widgetIndex) {
    cruiseToggles->setVisible(widgetIndex == 0);
    latLongToggles->setVisible(widgetIndex == 1);
    dispToggles->setVisible(widgetIndex == 2);
    pathToggles->setVisible(widgetIndex == 3);
    startToggles->setVisible(widgetIndex == 4);
    speedToggles->setVisible(widgetIndex == 5);
}


CValueControl::CValueControl(const QString& params, const QString& title, const QString& desc, const QString& icon, int min, int max, int unit)
    : AbstractControl(title, desc, icon), m_params(params), m_min(min), m_max(max), m_unit(unit) {

    label.setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    label.setStyleSheet("color: #e0e879");
    hlayout->addWidget(&label);

    QString btnStyle = R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )";

    btnminus.setStyleSheet(btnStyle);
    btnplus.setStyleSheet(btnStyle);
    btnminus.setFixedSize(150, 100);
    btnplus.setFixedSize(150, 100);
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
    label.setText(QString::fromStdString(Params().get(m_params.toStdString())));
}

void CValueControl::adjustValue(int delta) {
    int value = QString::fromStdString(Params().get(m_params.toStdString())).toInt();
    value = qBound(m_min, value + delta, m_max);
    Params().putInt(m_params.toStdString(), value);
    refresh();
}

void CValueControl::increaseValue() {
    adjustValue(m_unit);
}

void CValueControl::decreaseValue() {
    adjustValue(-m_unit);
}
