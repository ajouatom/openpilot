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
  //re_Calibration
  void calibration();
  void updateCalibDescription();

private:
  Params params;
  ButtonControl *pair_device;
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

// Forward declaration
class FirehosePanel;

class CarrotPanel : public QWidget {
  Q_OBJECT

private:
  QStackedLayout* main_layout = nullptr;
  QWidget* homeScreen = nullptr;
  int currentCarrotIndex = 0;

  QWidget* homeWidget;
  QVBoxLayout* carrotLayout;

  QStackedWidget* content_stack = nullptr;

  ListWidget* cruiseToggles;
  ListWidget* latLongToggles;
  ListWidget* pathToggles;
  ListWidget* dispToggles;
  ListWidget* startToggles;
  ListWidget* speedToggles;

  void togglesCarrot(int widgetIndex);
  void updateButtonStyles();

public:
  explicit CarrotPanel(QWidget* parent = nullptr);
};

class CValueControl : public AbstractControl {
  Q_OBJECT

public:
  CValueControl(const QString& params, const QString& title, const QString& desc, int min, int max, int unit = 1);

signals:
  void valueChanged(int val);

private slots:
  void increaseValue();
  void decreaseValue();

private:
  void showEvent(QShowEvent* event) override;
  void refresh();
  void adjustValue(int delta);

  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;

  QString m_params;
  int m_min;
  int m_max;
  int m_unit;
};

#include <QMap>
#include <QList>
#include <QColor>
#include <QSet>
#include <QStringList>

class AutoTunerGraphWidget : public QWidget {
  Q_OBJECT
public:
  explicit AutoTunerGraphWidget(QWidget *parent = nullptr);
  void setData(const QList<QString> &timestamps, const QMap<QString, QList<double>> &param_histories, const QMap<QString, QColor> &colors);
  void setSelectedParam(const QString &param);
  void setHiddenParams(const QSet<QString> &params);

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;

private:
  QList<QString> timestamps;
  QMap<QString, QList<double>> param_histories;
  QMap<QString, QColor> colors;
  QString selected_param;
  QSet<QString> hidden_params;
  int selected_index = -1;
};

class AutoTunerCardListDialog : public DialogBase {
  Q_OBJECT
public:
  explicit AutoTunerCardListDialog(QWidget *parent = nullptr);
private slots:
  void refreshHistory();
  void deleteItem(const QString& id);
  void restoreItem(const QString& id);
private:
  QVBoxLayout *list_layout;
};

class AutoTunerHistoryDialog : public DialogBase {
  Q_OBJECT
public:
  explicit AutoTunerHistoryDialog(QWidget *parent = nullptr);
};

class AutoTunerHistoryPanel : public QFrame {
  Q_OBJECT
public:
  explicit AutoTunerHistoryPanel(QWidget* parent = nullptr);
public slots:
  void refreshHistory();
  void updateLabelColors();
private slots:
  void deleteItem(const QString& id);
  void restoreItem(const QString& id);
  void clearAll();
private:
  void rebuildParamList();
  void toggleGroup(const QString &group);
  void applyHiddenParams();
  AutoTunerGraphWidget *graph_widget;
  QVBoxLayout *param_list_layout;
  QMap<QString, QLabel*> param_labels;
  QString selected_param;
  QString latest_id;
  QMap<QString, QColor> param_colors;
  // 좌측 파라미터 리스트를 그룹(가속/조향/거리/주행 등)으로 묶어
  // 그룹 헤더 클릭 시 접기/펴기 + 그래프 표시 토글을 지원하기 위한 상태
  QStringList group_order;                  // 표시 순서대로 정렬된 그룹 라벨
  QMap<QString, QStringList> group_params;  // 그룹 라벨 → 소속 파라미터들
  QSet<QString> collapsed_groups;           // 접혀있는(그래프 숨김) 그룹 라벨
protected:
  void showEvent(QShowEvent *event) override;
};
