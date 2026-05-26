#include <CL/cl.h>
#include <algorithm>
#include <time.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <atomic>
#include <mutex>

#include "libyuv.h"
#include "common/params.h"
#include "common/clutil.h"

#include "selfdrive/ui/qt/screenrecorder/screenrecorder.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/ui.h"
#include "system/hardware/hw.h"

static long long milliseconds(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (((long long)tv.tv_sec) * 1000) + (tv.tv_usec / 1000);
}

static void screenrecord_report_error(const char *stage, const char *detail) {
  printf("[screenrecord] ERROR %s: %s\n", stage, detail);
  fflush(stdout);
  Params().put("CarrotException", "exception");
}

ScreenRecoder::ScreenRecoder(QWidget* parent) : QPushButton(parent), image_queue(30) {
  recording.store(false);
  started = 0;
  frame = 0;

  const int size = 190;
  setFixedSize(size, size);
  setFocusPolicy(Qt::NoFocus);

  QObject::connect(this, &QPushButton::clicked, [=]() { toggle(); });
  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    if (offroad) {
      stop();
    }
    });

  std::string path = "/data/media/0/videos";
  src_width = 2160;
  src_height = 1080;

  dst_height = 720;
  dst_width = src_width * dst_height / src_height;
  if (dst_width % 2 != 0)
    dst_width += 1;

  rgb_buffer = std::make_unique<uint8_t[]>(src_width * src_height * 4);
  rgb_scale_buffer = std::make_unique<uint8_t[]>(dst_width * dst_height * 4);
  printf("[screenrecord] init src=%dx%d dst=%dx%d fps=%d path=%s\n",
         src_width, src_height, dst_width, dst_height, UI_FREQ, path.c_str());
  fflush(stdout);
  encoder = std::make_unique<OmxEncoder>(path.c_str(), dst_width, dst_height, UI_FREQ, 2 * 1024 * 1024, false, false);
}

ScreenRecoder::~ScreenRecoder() {
  stop();
}

void ScreenRecoder::applyColor() {
  if (frame % (UI_FREQ / 2) == 0) {
    if (frame % UI_FREQ < (UI_FREQ / 2))
      recording_color = QColor::fromRgbF(1, 0, 0, 0.6);
    else
      recording_color = QColor::fromRgbF(0, 0, 0, 0.3);

    update();
  }
}

void ScreenRecoder::paintEvent(QPaintEvent* event) {
  QRect r = QRect(0, 0, width(), height());
  r -= QMargins(5, 5, 5, 5);

  QPainter p(this);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);
  p.setPen(QPen(QColor::fromRgbF(1, 1, 1, 0.4), 10, Qt::SolidLine, Qt::FlatCap));
  p.setBrush(QBrush(QColor::fromRgbF(0, 0, 0, 0)));

  r -= QMargins(40, 40, 40, 40);
  p.setPen(Qt::NoPen);

  QColor bg = recording.load() ? recording_color : QColor::fromRgbF(0, 0, 0, 0.3);
  p.setBrush(QBrush(bg));
  p.drawEllipse(r);
}

void ScreenRecoder::openEncoder(const char* filename) {
  if (encoder) {
    encoder->encoder_open(filename);
  }
}

void ScreenRecoder::closeEncoder() {
  if (encoder) {
    encoder->encoder_close();
  }
}

void ScreenRecoder::toggle() {
  std::lock_guard<std::mutex> lk(record_lock);
  if (!recording.load()) {
    start_locked();
  }
  else {
    stop_locked();
  }
}

void ScreenRecoder::start() {
  std::lock_guard<std::mutex> lk(record_lock);
  start_locked();
}

void ScreenRecoder::stop() {
  std::lock_guard<std::mutex> lk(record_lock);
  stop_locked();
}

void ScreenRecoder::start_locked() {
  if (recording.load())
    return;

  if (encoding_thread.joinable()) {
    encoding_thread.join();
  }

  char filename[64];
  time_t t = time(NULL);
  struct tm tm_buf;
  localtime_r(&t, &tm_buf);

  snprintf(filename, sizeof(filename), "%04d%02d%02d-%02d%02d%02d.mp4",
    tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
    tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);

  frame = 0;

  QWidget* widget = this;
  while (widget->parentWidget() != nullptr)
    widget = widget->parentWidget();

  rootWidget = widget;

  image_queue.clear();
  printf("[screenrecord] start filename=%s root=%p dst=%dx%d\n", filename, rootWidget, dst_width, dst_height);
  fflush(stdout);
  openEncoder(filename);
  recording.store(true);

  encoding_thread = std::thread([this] { encoding_thread_func(); });

  started = milliseconds();
  update();
}

void ScreenRecoder::stop_locked() {
  if (!recording.load()) {
    if (encoding_thread.joinable()) {
      encoding_thread.join();
    }
    return;
  }

  recording.store(false);
  update();
  printf("[screenrecord] stop joining encoder thread\n");
  fflush(stdout);

  if (encoding_thread.joinable()) {
    encoding_thread.join();
  }

  image_queue.clear();
  closeEncoder();
  printf("[screenrecord] stop complete\n");
  fflush(stdout);
}

void ScreenRecoder::encoding_thread_func() {
  uint64_t start_time = nanos_since_boot() - 1;

  while (recording.load()) {
    QImage popImage;
    if (!image_queue.pop_wait_for(popImage, std::chrono::milliseconds(10))) {
      continue;
    }

    if (!recording.load()) {
      break;
    }

    QImage image = popImage.convertToFormat(QImage::Format_RGBA8888);

    try {
      libyuv::ARGBScale(image.bits(), image.width() * 4,
        image.width(), image.height(),
        rgb_scale_buffer.get(), dst_width * 4,
        dst_width, dst_height,
        libyuv::kFilterLinear);

      if (recording.load() && encoder) {
        encoder->encode_frame_rgba(
          rgb_scale_buffer.get(),
          dst_width,
          dst_height,
          ((uint64_t)nanos_since_boot() - start_time)
        );
      }
    }
    catch (...) {
      screenrecord_report_error("encoding_thread", "Encoding failed, skipping frame");
      continue;
    }
  }
}

void ScreenRecoder::update_screen() {
  bool need_restart = false;

  if (recording.load()) {
    if (milliseconds() - started > 1000 * 60 * 20) {
      need_restart = true;
    }
    else {
      applyColor();

      if (rootWidget != nullptr && recording.load()) {
        QPixmap pixmap = rootWidget->grab();
        if (pixmap.isNull()) {
          screenrecord_report_error("update_screen", "rootWidget->grab returned null pixmap");
          return;
        }
        if (recording.load()) {
          image_queue.push(pixmap.toImage());
        }
      }
    }
  }

  if (need_restart) {
    std::lock_guard<std::mutex> lk(record_lock);
    stop_locked();
    start_locked();
    return;
  }

  frame++;
}
