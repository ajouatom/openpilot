#pragma once

#include <QButtonGroup>
#include <QFileSystemWatcher>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QWidget>
#include <QStackedLayout>
#include <QComboBox>

#include "selfdrive/ui/qt/widgets/controls.h"

// ********** settings window + top-level panels **********
class SettingsWindow : public QFrame {
  Q_OBJECT

public:
  explicit SettingsWindow(QWidget *parent = 0);
  void setCurrentPanel(int index, const QString &param = "");

protected:
  void showEvent(QShowEvent *event) override;

signals:
  void closeSettings();
  void reviewTrainingGuide();
  void showDriverView();
  void expandToggleDescription(const QString &param);

private:
  QPushButton *sidebar_alert_widget;
  QWidget *sidebar_widget;
  QButtonGroup *nav_btns;
  QStackedWidget *panel_widget;
};

class DevicePanel : public ListWidget {
  Q_OBJECT
public:
  explicit DevicePanel(SettingsWindow *parent);
signals:
  void reviewTrainingGuide();
  void showDriverView();
  void closeSettings();

private slots:
  void poweroff();
  void reboot();
  void updateCalibDescription();

private:
  Params params;
};

class TogglesPanel : public ListWidget {
  Q_OBJECT
public:
  explicit TogglesPanel(SettingsWindow *parent);
  void showEvent(QShowEvent *event) override;

public slots:
  void expandToggleDescription(const QString &param);

private:
  Params params;
  std::map<std::string, ParamControl*> toggles;

  void updateToggles();
};

class SoftwarePanel : public ListWidget {
  Q_OBJECT
public:
  explicit SoftwarePanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  void updateLabels();
  void checkForUpdates();

  bool is_onroad = false;

  QLabel *onroadLbl;
  LabelControl *versionLbl;
  ButtonControl *installBtn;
  ButtonControl *downloadBtn;
  ButtonControl *targetBranchBtn;

  Params params;
  QFileSystemWatcher *fs_watch;
};

class C2NetworkPanel: public QWidget {
  Q_OBJECT
public:
  explicit C2NetworkPanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  QString getIPAddress();
  LabelControl *ipaddress;
};




class SelectCar : public QWidget {
  Q_OBJECT
public:
  explicit SelectCar(QWidget* parent = 0);

private:

signals:
  void backPress();
  void selectedCar();

};

class CommunityPanel : public QWidget {
    Q_OBJECT

private:
    QStackedLayout* main_layout = nullptr;
    QWidget* homeScreen = nullptr;
    SelectCar* selectCar = nullptr;

    QWidget* homeWidget;

public:
    explicit CommunityPanel(QWidget* parent = nullptr);
};
class TuningPanel : public QWidget {
    Q_OBJECT

private:
    QStackedLayout* main_layout = nullptr;
    QWidget* homeScreen = nullptr;

    QWidget* homeWidget;

public:
    explicit TuningPanel(QWidget* parent = nullptr);
};
class CruisePanel : public QWidget {
    Q_OBJECT

private:
    QStackedLayout* main_layout = nullptr;
    QWidget* homeScreen = nullptr;

    QWidget* homeWidget;

public:
    explicit CruisePanel(QWidget* parent = nullptr);
};

// ajouatom:
class CValueControl : public AbstractControl {
    Q_OBJECT

public:
    CValueControl(const QString& params, const QString& title, const QString& desc, const QString& icon, int min, int max, int unit = 1);

private:
    QPushButton btnplus;
    QPushButton btnminus;
    QLabel label;

    QString m_params;
    int     m_min;
    int     m_max;
    int     m_unit;

    void refresh();
};
class CPrebuiltToggle : public ToggleControl {
    Q_OBJECT

public:
    CPrebuiltToggle() : ToggleControl("Prebuilt", "Prebuilt.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrPrebuiltOn")) {
        QObject::connect(this, &CPrebuiltToggle::toggleFlipped, [=](int state) {
            Params().putBool("OpkrPrebuiltOn", (bool)state);

            if (state)
            {
                std::system("cd /data/openpilot; touch prebuilt");
            }
            else
            {
                std::system("cd /data/openpilot; rm -f prebuilt");
            }

        });
    }
};
class TimeZoneSelectCombo : public AbstractControl 
{
  Q_OBJECT

public:
  TimeZoneSelectCombo();

private:
  QPushButton btn;
  QComboBox combobox;
  Params params;

  void refresh();
};

class UseBaseTorqueToggle : public ToggleControl {
  Q_OBJECT

public:
  UseBaseTorqueToggle() : ToggleControl(tr("Use Base Torque Values"), "", "../assets/offroad/icon_openpilot.png", Params().getBool("UseBaseTorqueValues")) {
    QObject::connect(this, &UseBaseTorqueToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("UseBaseTorqueValues", status);
    });
  }
};

class GitHash : public AbstractControl {
    Q_OBJECT

public:
    GitHash();

private:
    QLabel local_hash;
    QLabel remote_hash;
};

