#include "selfdrive/ui/qt/onroad/onroad_home.h"

#include <QPainter>
#include <QStackedLayout>


#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonArray>
#include <QDialog>
#include <QMouseEvent>

#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/carrot.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/ui/qt/maps/map_panel.h"
#endif

class OverlayDialog : public QWidget {
  Q_OBJECT

public:
  explicit OverlayDialog(QWidget* parent = nullptr) : QWidget(parent) {
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint); // 다이얼로그처럼 동작
    setStyleSheet("background-color: rgba(0, 0, 0, 0.8); border-radius: 10px;");
    resize(400, 300); // 기본 크기 설정
  }

  void setContent(QWidget* content) {
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(content);
    layout->setMargin(0);
    setLayout(layout);
  }
};

OnroadWindow::OnroadWindow(QWidget *parent) : QOpenGLWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  //main_layout->setContentsMargins(UI_BORDER_SIZE, 0, UI_BORDER_SIZE, 0);

  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

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

  if (getenv("MAP_RENDER_VIEW")) {
    CameraWidget *map_render = new CameraWidget("navd", VISION_STREAM_MAP, this);
    split->insertWidget(0, map_render);
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
  UIState* ss = uiState();
  if (!s.scene.started) {
    ss->scene._current_carrot_display_prev = -1;
    return;
  }

  //alerts->updateState(s);
  ui_update_alert(OnroadAlerts::getAlert(*(s.sm), s.scene.started_frame));
  if (s.scene.map_on_left) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
  }
  nvg->updateState(s);

  QColor bgColor = bg_colors[s.status];
  QColor bgColor_long = bg_colors[s.status];
  const SubMaster& sm = *(s.sm);
  const auto car_control = sm["carControl"].getCarControl();
  auto selfdrive_state = sm["selfdriveState"].getSelfdriveState();

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
  else if (car_state.getLatEnabled()) {
      bgColor = bg_colors[STATUS_ACTIVE];
  }
  else
      bgColor = bg_colors[STATUS_DISENGAGED];

  if (car_state.getGasPressed()) {
      bgColor_long = bg_colors[STATUS_OVERRIDE];
  }
  else if (selfdrive_state.getEnabled()) {
      bgColor_long = bg_colors[STATUS_ENGAGED];
  }
  else if (car_state.getCruiseState().getAvailable()) {
	  bgColor_long = bg_colors[STATUS_ACTIVE];
  }
  else
      bgColor_long = bg_colors[STATUS_DISENGAGED];
  if (bg != bgColor || bg_long != bgColor_long) {
    // repaint border
    bg = bgColor;
    bg_long = bgColor_long;
    //update();
  }
  update();
  if (true) { 
      int carrot_display = 0;

      static int carrot_cmd_index_last = 0;
      if (sm.alive("carrotMan")) {
        const auto& carrot = sm["carrotMan"].getCarrotMan();
        int carrot_cmd_index = carrot.getCarrotCmdIndex();
        if (carrot_cmd_index != carrot_cmd_index_last) {
          carrot_cmd_index_last = carrot_cmd_index;
          QString carrot_cmd = QString::fromStdString(carrot.getCarrotCmd());
          QString carrot_arg = QString::fromStdString(carrot.getCarrotArg());
          if (carrot_cmd == "DISPLAY") {
            if (carrot_arg == "TOGGLE") {
              carrot_display = 5;
              //printf("Display toggle\n");
            }
            else if (carrot_arg == "DEFAULT") {
              carrot_display = 1;
              //printf("Display 1\n");
            }
            else if (carrot_arg == "ROAD") {
              carrot_display = 2;
              //printf("Display 2\n");
            }
            else if (carrot_arg == "MAP") {
              carrot_display = 3;
              //printf("Display 3\n");
            }
            else if (carrot_arg == "FULLMAP") {
              carrot_display = 4;
              //printf("Display 4\n");
            }
          }
        }
      }

      if (carrot_display == 5) ss->scene._current_carrot_display = (ss->scene._current_carrot_display % 3) + 1;
      else if(carrot_display > 0) ss->scene._current_carrot_display = carrot_display;
      if (map == nullptr && ss->scene._current_carrot_display > 2) ss->scene._current_carrot_display = 1;
      //if (offroad) _current_carrot_display = 1;
      switch (ss->scene._current_carrot_display) {
      case 1: // default
          if (map != nullptr) map->setVisible(false);
          if (ss->scene._current_carrot_display_prev != ss->scene._current_carrot_display) ss->scene._display_time_count = 100; // 100: about 5 seconds
          if (ss->scene._display_time_count-- <= 0) ss->scene._current_carrot_display = 2; // change to road view
          break;
      case 2: // road          
          if (map != nullptr) map->setVisible(false);
          break;
      case 3: // map
          if (map == nullptr) ss->scene._current_carrot_display = 1;
          else {
              map->setVisible(true);
              //map->setFixedWidth(topWidget(this)->width() / 2 - UI_BORDER_SIZE);
          }
          break;
      case 4: // fullmap
          if (map == nullptr) ss->scene._current_carrot_display = 1;
          else {
              map->setVisible(true);
              //map->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
              //map->setFixedWidth(topWidget(this)->width() - UI_BORDER_SIZE);
          }
          break;
      }
      ss->scene._current_carrot_display_prev = ss->scene._current_carrot_display;
  }
  
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  //printf("uiState()->scene.navigate_on_openpilot = %d\n", uiState()->scene.navigate_on_openpilot);
//#ifdef ENABLE_MAPS
//  if (map != nullptr) {
    // Switch between map and sidebar when using navigate on openpilot
    //bool sidebarVisible = geometry().x() > 0;
    //bool show_map = uiState()->scene.navigate_on_openpilot ? sidebarVisible : !sidebarVisible;
    //map->setVisible(show_map && !map->isVisible());
//  }
//#endif
  // propagation event to parent(HomeWindow)
  int x = e->x();   // 430 - 500 : gap window
  int y = height() - e->y();  // 60 - 180 : gap window
  int ey = e->y();
  printf("x=%d, y=%d, ey=%d\n", x, y, ey);
  double now = millis_since_boot();
  static double last_click_time = 0;
  static int _click_count = 0;
  // 40,150, 200, 150
  Params	params;
  if (x > 40 && x < 370 && ey > 30 && ey < 240) {   // date & time
    int show_date_time = params.getInt("ShowDateTime");
    params.putIntNonBlocking("ShowDateTime", (show_date_time + 1) % 3);
  }
  else if (x > 40 && x < 500 && y > 400 && y < 530) {   // device info
    int show_device_state = params.getInt("ShowDeviceState");
    params.putIntNonBlocking("ShowDeviceState", (show_device_state + 1) % 2);
  }
  else if (x > 40 && x < 200 && y > 20 && y < 150) {   // driving mode
    int my_driving_mode = params.getInt("MyDrivingMode");
    params.putIntNonBlocking("MyDrivingMode", (my_driving_mode) % 4 + 1);
  }
  else if (x > 350 && x < 550 && y > 20 && y < 250) { // gap control
    int longitudinalPersonalityMax = params.getInt("LongitudinalPersonalityMax");
    int personality = (params.getInt("LongitudinalPersonality") - 1 + longitudinalPersonalityMax) % longitudinalPersonalityMax;
    params.putIntNonBlocking("LongitudinalPersonality", personality);

  }
  else {
    if (now - last_click_time < 500) {
      _click_count++;
    }
    else {
      _click_count = 0;
    }
    last_click_time = now;
    if (_click_count == 3) {
      params.putIntNonBlocking("SoftRestartTriggered", 1);
    }
    
    UIState* s = uiState();
    s->scene._current_carrot_display = (s->scene._current_carrot_display % 3) + 1;  // 4번: full map은 안보여줌.
    printf("_current_carrot_display1=%d\n", s->scene._current_carrot_display);
    QWidget::mousePressEvent(e);
  }
}
//OverlayDialog* mapDialog = nullptr;
void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (!MAPBOX_TOKEN.isEmpty())) {
      printf("####### Initialize MapPanel\n");
#if 0
      auto m = new MapPanel(get_mapbox_settings());
      map = m;

      m->setFixedWidth(topWidget(this)->width() / 2 - UI_BORDER_SIZE);
      split->insertWidget(0, m);

      // hidden by default, made visible when navRoute is published
      m->setVisible(false);
#else
      OverlayDialog* mapDialog = new OverlayDialog(this);
      mapDialog->setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
      mapDialog->setAttribute(Qt::WA_TranslucentBackground);
      mapDialog->setAttribute(Qt::WA_NoSystemBackground);

      // MapPanel 추가
      auto m = new MapPanel(get_mapbox_settings(), mapDialog);
      map = m;
      mapDialog->setContent(m);

      // 특정 위치에 배치 (오른쪽 하단)
      mapDialog->setGeometry(topWidget(this)->width() - 790 - UI_BORDER_SIZE, UI_BORDER_SIZE + 15, 775, topWidget(this)->height() - 400);

      //mapDialog->hide(); // 기본적으로 숨김 상태
      mapDialog->show();
      mapDialog->raise();
      uiState()->scene._current_carrot_display = 1;

#endif
    }
  }
#endif
  alerts->clear();
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
    QPainter p(this);
    p.beginNativePainting();
    UIState* s = uiState();
    extern void ui_draw_border(UIState * s, int w, int h, QColor bg, QColor bg_long);
    ui_draw_border(s, width(), height(), bg, bg_long);
    p.endNativePainting();
}


// OnroadWindow.cpp에서 OpenGL 초기화 및 그리기 구현
void OnroadWindow::initializeGL() {
    initializeOpenGLFunctions(); // QOpenGLFunctions 초기화

    // Parent widget을 위한 NanoVG 컨텍스트 생성
    //s->vg = nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
    //if (s->vg == nullptr) {
    //    printf("Could not init nanovg.\n");
    //    return;
    //}
}


#include "selfdrive/ui/qt/onroad/onroad_home.moc"
