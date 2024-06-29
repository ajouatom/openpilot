﻿#include "selfdrive/ui/qt/maps/map.h"

#include <algorithm>
#include <eigen3/Eigen/Dense>

#include <QDebug>

#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/ui.h"

#include "selfdrive/controls/neokii/navi_gps_manager.h"
NaviGpsManager navi_gps_manager;

const int INTERACTION_TIMEOUT = 100;

//const float MAX_ZOOM = 20;// 17;
const float MIN_ZOOM = 14;
const float MAX_PITCH = 50;
const float MIN_PITCH = 0;
const float MAP_SCALE = 2;

MapWindow::MapWindow(const QMapLibre::Settings &settings) : m_settings(settings), velocity_filter(0, 10, 0.05, false) {
  QObject::connect(uiState(), &UIState::uiUpdate, this, &MapWindow::updateState);

  map_overlay = new QWidget (this);
  map_overlay->setAttribute(Qt::WA_TranslucentBackground, true);
  QVBoxLayout *overlay_layout = new QVBoxLayout(map_overlay);
  overlay_layout->setContentsMargins(0, 0, 0, 0);

  // Instructions
  map_instructions = new MapInstructions(this);
  map_instructions->setVisible(false);

  map_eta = new MapETA(this);
  map_eta->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  map_eta->setFixedHeight(120);

  error = new QLabel(this);
  error->setStyleSheet(R"(color:white;padding:50px 11px;font-size: 90px; background-color:rgba(0, 0, 0, 150);)");
  error->setAlignment(Qt::AlignCenter);

  overlay_layout->addWidget(error);
  overlay_layout->addWidget(map_instructions);
  overlay_layout->addStretch(1);
  overlay_layout->addWidget(map_eta);

  last_position = coordinate_from_param("LastGPSPosition");
  grabGesture(Qt::GestureType::PinchGesture);
  qDebug() << "MapWindow initialized";
}

MapWindow::~MapWindow() {
  makeCurrent();
}

void MapWindow::initLayers() {
  // This doesn't work from initializeGL
  if (!m_map->layerExists("modelPathLayer")) {
    qDebug() << "Initializing modelPathLayer";
    QVariantMap modelPath;
    //modelPath["id"] = "modelPathLayer";
    modelPath["type"] = "line";
    modelPath["source"] = "modelPathSource";
    m_map->addLayer("modelPathLayer", modelPath);
    m_map->setPaintProperty("modelPathLayer", "line-color", QColor("red"));
    m_map->setPaintProperty("modelPathLayer", "line-width", 5.0);
    m_map->setLayoutProperty("modelPathLayer", "line-cap", "round");
  }
  if (!m_map->layerExists("navLayer")) {
    qDebug() << "Initializing navLayer";
    QVariantMap nav;
    nav["type"] = "line";
    nav["source"] = "navSource";
    m_map->addLayer("navLayer", nav, "road-intersection");

    QVariantMap transition;
    transition["duration"] = 400;  // ms
    m_map->setPaintProperty("navLayer", "line-color", QColor("#31a1ee"));
    m_map->setPaintProperty("navLayer", "line-color-transition", transition);
    m_map->setPaintProperty("navLayer", "line-width", 7.5);
    m_map->setLayoutProperty("navLayer", "line-cap", "round");
  }
  if (!m_map->layerExists("pinLayer")) {
    qDebug() << "Initializing pinLayer";
    m_map->addImage("default_marker", QImage("../assets/navigation/default_marker.svg"));
    QVariantMap pin;
    pin["type"] = "symbol";
    pin["source"] = "pinSource";
    m_map->addLayer("pinLayer", pin);
    m_map->setLayoutProperty("pinLayer", "icon-pitch-alignment", "viewport");
    m_map->setLayoutProperty("pinLayer", "icon-image", "default_marker");
    m_map->setLayoutProperty("pinLayer", "icon-ignore-placement", true);
    m_map->setLayoutProperty("pinLayer", "icon-allow-overlap", true);
    m_map->setLayoutProperty("pinLayer", "symbol-sort-key", 0);
    m_map->setLayoutProperty("pinLayer", "icon-anchor", "bottom");
  }
  if (!m_map->layerExists("carPosLayer")) {
    qDebug() << "Initializing carPosLayer";
    m_map->addImage("label-arrow", QImage("../assets/images/triangle.svg"));

    QVariantMap carPos;
    carPos["type"] = "symbol";
    carPos["source"] = "carPosSource";
    m_map->addLayer("carPosLayer", carPos);
    m_map->setLayoutProperty("carPosLayer", "icon-pitch-alignment", "map");
    m_map->setLayoutProperty("carPosLayer", "icon-image", "label-arrow");
    m_map->setLayoutProperty("carPosLayer", "icon-size", 0.5);
    m_map->setLayoutProperty("carPosLayer", "icon-ignore-placement", true);
    m_map->setLayoutProperty("carPosLayer", "icon-allow-overlap", true);
    // TODO: remove, symbol-sort-key does not seem to matter outside of each layer
    m_map->setLayoutProperty("carPosLayer", "symbol-sort-key", 0);
  }

  if (!m_map->layerExists("buildingsLayer")) {
    qDebug() << "Initializing buildingsLayer";
    QVariantMap buildings;
    //buildings["id"] = "buildingsLayer";
    buildings["source"] = "composite";
    buildings["source-layer"] = "building";
    buildings["type"] = "fill-extrusion";
    buildings["minzoom"] = 15;
    m_map->addLayer("buildingsLayer", buildings);
    m_map->setFilter("buildingsLayer", QVariantList({"==", "extrude", "true"}));

    QVariantList fillExtrusionHight = {
      "interpolate",
      QVariantList{"linear"},
      QVariantList{"zoom"},
      15, 0,
      15.05, QVariantList{"get", "height"}
    };

    QVariantList fillExtrusionBase = {
      "interpolate",
      QVariantList{"linear"},
      QVariantList{"zoom"},
      15, 0,
      15.05, QVariantList{"get", "min_height"}
    };

    QVariantList fillExtrusionOpacity = {
      "interpolate", 
      QVariantList{"linear"},
      QVariantList{"zoom"},
      15, 0,
      15.5, .6,
      17, .6,
      20, 0
    };

    m_map->setPaintProperty("buildingsLayer", "fill-extrusion-color", QColor("grey"));
    m_map->setPaintProperty("buildingsLayer", "fill-extrusion-opacity", fillExtrusionOpacity);
    m_map->setPaintProperty("buildingsLayer", "fill-extrusion-height", fillExtrusionHight);
    m_map->setPaintProperty("buildingsLayer", "fill-extrusion-base", fillExtrusionBase);
    m_map->setLayoutProperty("buildingsLayer", "visibility", "visible");
  }
}

void MapWindow::updateState(const UIState &s) {
  if (!uiState()->scene.started) {
    return;
  }
  const SubMaster &sm = *(s.sm);
  update();

  if(navi_gps_manager.check()) {
    locationd_valid = true;
    QMapLibre::Coordinate position;
    float bearing = 0.0f, speed = 0.0f;
    navi_gps_manager.update(position, bearing, speed);
    last_position = position;
    last_bearing = bearing;
    velocity_filter.update(std::max(10.0, (double)speed));
    if(m_map && m_map->isFullyLoaded()) {
      navi_gps_manager.setPosition(m_map.data(), position);
      navi_gps_manager.setBearing(m_map.data(), bearing);
    }
  }
  else if (sm.updated("liveLocationKalman")) {
    auto locationd_location = sm["liveLocationKalman"].getLiveLocationKalman();
    auto locationd_pos = locationd_location.getPositionGeodetic();
    auto locationd_orientation = locationd_location.getCalibratedOrientationNED();
    auto locationd_velocity = locationd_location.getVelocityCalibrated();
    auto locationd_ecef = locationd_location.getPositionECEF();

    locationd_valid = (locationd_pos.getValid() && locationd_orientation.getValid() && locationd_velocity.getValid() && locationd_ecef.getValid());
    if (locationd_valid) {
      // Check std norm
      auto pos_ecef_std = locationd_ecef.getStd();
      bool pos_accurate_enough = sqrt(pow(pos_ecef_std[0], 2) + pow(pos_ecef_std[1], 2) + pow(pos_ecef_std[2], 2)) < 100;
      locationd_valid = pos_accurate_enough;
    }

    if (locationd_valid) {
      last_position = QMapLibre::Coordinate(locationd_pos.getValue()[0], locationd_pos.getValue()[1]);
      last_bearing = RAD2DEG(locationd_orientation.getValue()[2]);
      velocity_filter.update(std::max(10.0, locationd_velocity.getValue()[0]));
    }
  }
  auto roadLimitSpeed = sm["roadLimitSpeed"].getRoadLimitSpeed();
  float lat = roadLimitSpeed.getXPosLat();
  float lon = roadLimitSpeed.getXPosLon();
  float angle = roadLimitSpeed.getXPosAngle();
  float speed = roadLimitSpeed.getXPosSpeed();

  int validCount = roadLimitSpeed.getXPosValidCount();
  if (validCount > 0) {
      locationd_valid = true;
      float bearing = (angle > 180) ? angle - 360 : angle;

      last_position = QMapLibre::Coordinate(lat, lon);
      last_bearing = bearing;
      velocity_filter.update(std::max(10.0, (double)speed / 3.6));
  }

  bool allow_open = false;  // carrot
  if (sm.updated("navRoute") && sm["navRoute"].getNavRoute().getCoordinates().size()) {
    auto nav_dest = coordinate_from_param("NavDestination");
#if 0
    bool allow_open = std::exchange(last_valid_nav_dest, nav_dest) != nav_dest &&
        nav_dest && !isVisible();
#else
    // carrot: 왜? 경로가 바뀌었는데 목적지만 비교? navRoute가 수신되면 무조건 해야지.. 보내는곳은 만들어서 보냈는데..
    std::exchange(last_valid_nav_dest, nav_dest);
    allow_open = nav_dest && !isVisible();
#endif

    allow_open = true; // carrot : 왜? 경로가 바뀌었는데?
    qWarning() << "Got new navRoute from navd. Opening map:" << allow_open;

    // Show map on destination set/change
    if (allow_open && (params.getInt("AutoTurnMapChange") > 0)) {
        printf("###########MapWindow : requestVisible\n");
      emit requestSettings(false);
      emit requestVisible(true);

      // carrot
      extern int _current_carrot_display;
      _current_carrot_display = 3;
      extern int _display_time_count;
      _display_time_count = 100;
    }
  }

  loaded_once = loaded_once || (m_map && m_map->isFullyLoaded());
  if (!loaded_once) {
    setError(tr("Map Loading"));
    return;
  }
  initLayers();

  if (!locationd_valid) {
    setError(tr("Waiting for GPS"));
  } else if (routing_problem) {
    setError(tr("Waiting for route"));
  } else {
    setError("");
  }

  if (locationd_valid) {
    // Update current location marker
    auto point = coordinate_to_collection(*last_position);
    QMapLibre::Feature feature1(QMapLibre::Feature::PointType, point, {}, {});
    QVariantMap carPosSource;
    carPosSource["type"] = "geojson";
    carPosSource["data"] = QVariant::fromValue<QMapLibre::Feature>(feature1);
    m_map->updateSource("carPosSource", carPosSource);

    // Map bearing isn't updated when interacting, keep location marker up to date
    if (last_bearing) {
      if(!navi_gps_manager.isValid())
        m_map->setLayoutProperty("carPosLayer", "icon-rotate", *last_bearing - m_map->bearing());
      else
        m_map->setLayoutProperty("carPosLayer", "icon-rotate", 0);
    }
  }

  if (interaction_counter == 0) {
    if(!navi_gps_manager.isValid()) {
      if (last_position) m_map->setCoordinate(*last_position);
      if (last_bearing) m_map->setBearing(*last_bearing);
    }
    m_map->setZoom(util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM));
  } else {
    interaction_counter--;
  }

  if (sm.updated("navInstruction")) {
    // an invalid navInstruction packet with a nav destination is only possible if:
    // - API exception/no internet
    // - route response is empty
    // - any time navd is waiting for recompute_countdown
    routing_problem = !sm.valid("navInstruction") && coordinate_from_param("NavDestination").has_value();

    if (sm.valid("navInstruction")) {
      auto i = sm["navInstruction"].getNavInstruction();
      map_eta->updateETA(i.getTimeRemaining(), i.getTimeRemainingTypical(), i.getDistanceRemaining());

      if (locationd_valid) {
        if(!navi_gps_manager.isValid())  m_map->setPitch(MAX_PITCH); // TODO: smooth pitching based on maneuver distance
        map_instructions->updateInstructions(i);
      }
    } else {
      clearRoute();
    }
  }

  //if (sm.rcv_frame("navRoute") != route_rcv_frame) {
  if ((sm.rcv_frame("navRoute") != route_rcv_frame) || allow_open) {        // carrot
      clearRoute();
    qWarning() << "Updating navLayer with new route";
    auto route = sm["navRoute"].getNavRoute();
    auto route_points = capnp_coordinate_list_to_collection(route.getCoordinates());
    QMapLibre::Feature feature(QMapLibre::Feature::LineStringType, route_points, {}, {});
    QVariantMap navSource;
    navSource["type"] = "geojson";
    navSource["data"] = QVariant::fromValue<QMapLibre::Feature>(feature);
    m_map->updateSource("navSource", navSource);
    m_map->setLayoutProperty("navLayer", "visibility", "visible");

    route_rcv_frame = sm.rcv_frame("navRoute");
    updateDestinationMarker();
  }
}

void MapWindow::setError(const QString &err_str) {
  if (err_str != error->text()) {
    error->setText(err_str);
    error->setVisible(!err_str.isEmpty());
    if (!err_str.isEmpty()) map_instructions->setVisible(false);
  }
}

void MapWindow::resizeGL(int w, int h) {
  m_map->resize(size() / MAP_SCALE);
  map_overlay->setFixedSize(width(), height());
}

void MapWindow::initializeGL() {
  m_map.reset(new QMapLibre::Map(this, m_settings, size(), 1));

  if (last_position) {
    m_map->setCoordinateZoom(*last_position, MAX_ZOOM);
  } else {
    m_map->setCoordinateZoom(QMapLibre::Coordinate(64.31990695292795, -149.79038934046247), MIN_ZOOM);
  }

  m_map->setMargins({0, 350, 0, 50});
  m_map->setPitch(MIN_PITCH);
  
  // carrot
  int mapbox_style = Params().getInt("MapboxStyle");
  
  MAX_ZOOM = 17;
  switch(mapbox_style) {
    case 1:
      m_map->setStyleUrl("mapbox://styles/mapbox/navigation-night-v1"); 
      break;
    case 2:
      m_map->setStyleUrl("mapbox://styles/mapbox/satellite-streets-v12");
      MAX_ZOOM = 20;
      break;
    case 0:  
      m_map->setStyleUrl("mapbox://styles/commaai/clkqztk0f00ou01qyhsa5bzpj");
    default:
      break;
  }

  QObject::connect(m_map.data(), &QMapLibre::Map::mapChanged, [=](QMapLibre::Map::MapChange change) {
    // set global animation duration to 0 ms so visibility changes are instant
    if (change == QMapLibre::Map::MapChange::MapChangeDidFinishLoadingStyle) {
      m_map->setTransitionOptions(0, 0);
    }
    if (change == QMapLibre::Map::MapChange::MapChangeDidFinishLoadingMap) {
      loaded_once = true;
    }
  });
}

void MapWindow::paintGL() {
  if (!isVisible() || m_map.isNull()) return;
  m_map->render();
}

void MapWindow::clearRoute() {
  if (!m_map.isNull()) {
    m_map->setLayoutProperty("navLayer", "visibility", "none");
    m_map->setPitch(MIN_PITCH);
    updateDestinationMarker();
  }

  map_instructions->setVisible(false);
  map_eta->setVisible(false);
  last_valid_nav_dest = std::nullopt;
}

void MapWindow::mousePressEvent(QMouseEvent *ev) {
  m_lastPos = ev->localPos();
  ev->accept();
}

void MapWindow::mouseDoubleClickEvent(QMouseEvent *ev) {
  if (last_position) m_map->setCoordinate(*last_position);
  if (last_bearing) m_map->setBearing(*last_bearing);
  m_map->setZoom(util::map_val<float>(velocity_filter.x(), 0, 30, MAX_ZOOM, MIN_ZOOM));
  update();

  interaction_counter = 0;
}

void MapWindow::mouseMoveEvent(QMouseEvent *ev) {
  QPointF delta = ev->localPos() - m_lastPos;

  if (!delta.isNull()) {
    interaction_counter = INTERACTION_TIMEOUT;
    m_map->moveBy(delta / MAP_SCALE);
    update();
  }

  m_lastPos = ev->localPos();
  ev->accept();
}

void MapWindow::wheelEvent(QWheelEvent *ev) {
  if (ev->orientation() == Qt::Horizontal) {
      return;
  }

  float factor = ev->delta() / 1200.;
  if (ev->delta() < 0) {
      factor = factor > -1 ? factor : 1 / factor;
  }

  m_map->scaleBy(1 + factor, ev->pos() / MAP_SCALE);
  update();

  interaction_counter = INTERACTION_TIMEOUT;
  ev->accept();
}

bool MapWindow::event(QEvent *event) {
  if (event->type() == QEvent::Gesture) {
    return gestureEvent(static_cast<QGestureEvent*>(event));
  }

  return QWidget::event(event);
}

bool MapWindow::gestureEvent(QGestureEvent *event) {
  if (QGesture *pinch = event->gesture(Qt::PinchGesture)) {
    pinchTriggered(static_cast<QPinchGesture *>(pinch));
  }
  return true;
}

void MapWindow::pinchTriggered(QPinchGesture *gesture) {
  QPinchGesture::ChangeFlags changeFlags = gesture->changeFlags();
  if (changeFlags & QPinchGesture::ScaleFactorChanged) {
    // TODO: figure out why gesture centerPoint doesn't work
    m_map->scaleBy(gesture->scaleFactor(), {width() / 2.0 / MAP_SCALE, height() / 2.0 / MAP_SCALE});
    update();
    interaction_counter = INTERACTION_TIMEOUT;
  }
}

void MapWindow::offroadTransition(bool offroad) {
  if (offroad) {
    clearRoute();
    routing_problem = false;
  } else {
    auto dest = coordinate_from_param("NavDestination");
    emit requestVisible(dest.has_value());
  }
  last_bearing = {};
}

void MapWindow::updateDestinationMarker() {
  auto nav_dest = coordinate_from_param("NavDestination");
  if (nav_dest.has_value()) {
    auto point = coordinate_to_collection(*nav_dest);
    QMapLibre::Feature feature(QMapLibre::Feature::PointType, point, {}, {});
    QVariantMap pinSource;
    pinSource["type"] = "geojson";
    pinSource["data"] = QVariant::fromValue<QMapLibre::Feature>(feature);
    m_map->updateSource("pinSource", pinSource);
    m_map->setPaintProperty("pinLayer", "visibility", "visible");
  } else {
    m_map->setPaintProperty("pinLayer", "visibility", "none");
  }
}
