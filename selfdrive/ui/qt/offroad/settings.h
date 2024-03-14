#pragma once

#include <map>
#include <string>

#include <QButtonGroup>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QWidget>


#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
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

private slots:
  void updateState(const UIState &s);

private:
  Params params;
  std::map<std::string, ParamControl*> toggles;
  ButtonParamControl *long_personality_setting;

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
  ParamWatcher *fs_watch;
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
class CarrotPanel : public QWidget {
    Q_OBJECT

private:
    QStackedLayout* main_layout = nullptr;
    QWidget* homeScreen = nullptr;

    QWidget* homeWidget;
    QVBoxLayout* carrotLayout;

    ListWidget* cruiseToggles;
    ListWidget* latLongToggles;
    ListWidget* pathToggles;
    ListWidget* dispToggles;
    ListWidget* startToggles;
    ListWidget* speedToggles;

    void togglesCarrot(int widgetIndex);

public:
    explicit CarrotPanel(QWidget* parent = nullptr);
};
class CarsPanel : public QWidget {
    Q_OBJECT

private:
    SelectCar* selectCar = nullptr;
    QStackedLayout* main_layout = nullptr;
    QWidget* homeScreen = nullptr;

    QWidget* homeWidget;
    QVBoxLayout* carrotLayout;

    ListWidget* commonToggles;
    ListWidget* hyundaiToggles;
    ListWidget* gmToggles;
    ListWidget* toyotaToggles;

    void togglesCarrot(int widgetIndex);

public:
    explicit CarsPanel(QWidget* parent = nullptr);
};

// ajouatom:
class CValueControl : public AbstractControl {
    Q_OBJECT

public:
    CValueControl(const QString& params, const QString& title, const QString& desc, const QString& icon, int min, int max, int unit = 1);

private:
    void showEvent(QShowEvent* event) override;
    QPushButton btnplus;
    QPushButton btnminus;
    QLabel label;

    QString m_params;
    int     m_min;
    int     m_max;
    int     m_unit;

    void refresh();
};
