#pragma once

#include <memory>
#include <cstdint>
#include <QPainter>
#include <QPushButton>
#include <thread>
#include <chrono>

#include "omx_encoder.h"
#include "blocking_queue.h"
#include "selfdrive/ui/ui.h"

#ifdef WSL2
class ScreenRecoder : public QPushButton {
public:
    ScreenRecoder(QWidget* parent = nullptr) {}
    virtual ~ScreenRecoder() {}

    void update_screen() {}
    void toggle() {}
    void start() {}
    void stop() {}
};
#else

class ScreenRecoder : public QPushButton {
  Q_OBJECT

public:
  ScreenRecoder(QWidget *parent = 0);
  virtual ~ScreenRecoder();

protected:
  void paintEvent(QPaintEvent*) override;

private:
  bool recording;
  long long started;
  int src_width, src_height;
  int dst_width, dst_height;

  QColor recording_color;
  int frame;

  void applyColor();

  std::unique_ptr<OmxEncoder> encoder;
  std::unique_ptr<uint8_t[]> rgb_buffer;
  std::unique_ptr<uint8_t[]> rgb_scale_buffer;

  std::thread encoding_thread;
  BlockingQueue<QImage> image_queue;
  QWidget* rootWidget;
  void encoding_thread_func();
  void openEncoder(const char* filename);
  void closeEncoder();

public:
    void start();
    void stop();
    void toggle();
    void update_screen();

};
#endif
