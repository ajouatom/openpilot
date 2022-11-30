#include <QMap>
#include <QSoundEffect>
#include <QString>

#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/ui.h"

const std::tuple<AudibleAlert, QString, int> sound_list[] = {
  // AudibleAlert, file name, loop count
  {AudibleAlert::ENGAGE, "engage.wav", 0},
  {AudibleAlert::DISENGAGE, "disengage.wav", 0},
  {AudibleAlert::REFUSE, "refuse.wav", 0},

  {AudibleAlert::PROMPT, "prompt.wav", 0},
  {AudibleAlert::PROMPT_REPEAT, "prompt.wav", QSoundEffect::Infinite},
  {AudibleAlert::PROMPT_DISTRACTED, "prompt_distracted.wav", QSoundEffect::Infinite},

  {AudibleAlert::WARNING_SOFT, "warning_soft.wav", QSoundEffect::Infinite},
  //{AudibleAlert::WARNING_IMMEDIATE, "warning_immediate.wav", QSoundEffect::Infinite},
  {AudibleAlert::WARNING_IMMEDIATE, "warning_immediate.wav", 2},
  {AudibleAlert::LONG_ENGAGED, "tici_engaged.wav", 0},
  {AudibleAlert::LONG_DISENGAGED, "tici_disengaged.wav", 0},
  {AudibleAlert::TRAFFIC_SIGN_GREEN, "traffic_sign_green.wav", 0},
  {AudibleAlert::TRAFFIC_SIGN_CHANGED, "traffic_sign_changed.wav", 0},
  {AudibleAlert::TRAFFIC_ERROR, "audio_traffic_error.wav", 0},
  {AudibleAlert::BSD_WARNING, "audio_car_watchout.wav", 0},
  {AudibleAlert::LANE_CHANGE, "audio_lane_change.wav", 0},
  {AudibleAlert::STOP_STOP, "audio_stopstop.wav", 0},
  {AudibleAlert::STOPPING, "audio_stopping.wav", 0},
  {AudibleAlert::AUTO_HOLD, "audio_auto_hold.wav", 0},
  {AudibleAlert::ENGAGE2, "audio_engage.wav", 0},
  {AudibleAlert::DISENGAGE2, "audio_disengage.wav", 0},
  {AudibleAlert::SPEED_DOWN, "audio_speed_down.wav", 0},
};

class Sound : public QObject {
public:
  explicit Sound(QObject *parent = 0);

protected:
  void update();
  void setAlert(const Alert &alert);

  Alert current_alert = {};
  QMap<AudibleAlert, QPair<QSoundEffect *, int>> sounds;
  SubMaster sm;
  uint64_t started_frame;
};
