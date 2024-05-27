#pragma once

#include <QMapLibre/Map>
#include <QSharedPointer>
#include <QPropertyAnimation>
#include <cmath>
#include "cereal/messaging/messaging.h"

#define ANIM_DURATION_POSITION 300
#define ANIM_DURATION_BEARING 600

class NaviGpsManager {
private:
  std::unique_ptr<SubMaster> sm;
  bool valid;
  QSharedPointer<QPropertyAnimation> animationLatitude;
  QSharedPointer<QPropertyAnimation> animationLongitude;
  QSharedPointer<QPropertyAnimation> animationBearing;

public:
  NaviGpsManager(): valid(false) {}

  bool check() {
    if(!sm)
      sm.reset(new SubMaster({"naviGps"}, {"naviGps"}));
    sm->update(0);
    auto now = nanos_since_boot();
    valid = now - (*sm)["naviGps"].getLogMonoTime() < 3000*1000000LL;
    return valid;
  }

  bool isValid() {
    return valid;
  }

  void update(QMapLibre::Coordinate& position, float& bearing, float& speed) {
    auto gps = (*sm)["naviGps"].getNaviGps();
    position.first = gps.getLatitude();
    position.second = gps.getLongitude();
    bearing = gps.getHeading();
    speed = gps.getSpeed();
  }

  void setPosition(QMapLibre::Map* map, QMapLibre::Coordinate& position) {
    if(!(map && map->isFullyLoaded()))
      return;

    if(animationLatitude) animationLatitude->stop();
    if(animationLongitude) animationLongitude->stop();

    animationLatitude.reset(new QPropertyAnimation(map, "latitude"));
    animationLatitude->setDuration(ANIM_DURATION_POSITION);
    animationLatitude->setStartValue(map->latitude());
    animationLatitude->setEndValue(position.first);
    animationLatitude->start();

    animationLongitude.reset(new QPropertyAnimation(map, "longitude"));
    animationLongitude->setDuration(ANIM_DURATION_POSITION);
    animationLongitude->setStartValue(map->longitude());
    animationLongitude->setEndValue(position.second);
    animationLongitude->start();
  }

  void setBearing(QMapLibre::Map* map, float targetBearing) {
    if(!(map && map->isFullyLoaded()))
      return;
    float currentBearing = map->bearing();
    float diff = targetBearing - currentBearing;
    if (diff > 180) diff -= 360;
    else if (diff < -180) diff += 360;

    if(std::abs(diff) < 1.0)
      return;

    float newBearing = currentBearing + diff;
    if (animationBearing) animationBearing->stop();
    animationBearing.reset(new QPropertyAnimation(map, "bearing"));
    animationBearing->setDuration(ANIM_DURATION_BEARING);
    animationBearing->setStartValue(currentBearing);
    animationBearing->setEndValue(newBearing);
    animationBearing->start();
  }
};
