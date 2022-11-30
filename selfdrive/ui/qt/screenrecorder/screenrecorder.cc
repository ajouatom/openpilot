#include <CL/cl.h>
#include <algorithm>
#include <time.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "libyuv.h"
#include "selfdrive/camerad/transforms/rgb_to_yuv.h"
#include "selfdrive/common/clutil.h"

#include "selfdrive/ui/qt/screenrecorder/screenrecorder.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/hardware/hw.h"

static long long milliseconds(void) {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return (((long long)tv.tv_sec)*1000)+(tv.tv_usec/1000);
}

ScreenRecoder::ScreenRecoder(QWidget *parent) : QPushButton(parent)
#ifdef QCOM2
, image_queue(30)
#endif
{

  recording = false;
  started = 0;
  frame = 0;

  const int size = 190;
  setFixedSize(size, size);
  setFocusPolicy(Qt::NoFocus);
  connect(this, SIGNAL(pressed()),this,SLOT(btnPressed()));
  connect(this, SIGNAL(released()),this,SLOT(btnReleased()));

  std::string path = "/data/media/0/videos";
  src_width = 2160;
  src_height = 1080;

  dst_height = 720;
  dst_width = src_width * dst_height / src_height;
  if(dst_width % 2 != 0)
      dst_width += 1;

#ifdef QCOM2
  rgb_buffer = std::make_unique<uint8_t[]>(src_width*src_height*4);
  rgb_scale_buffer = std::make_unique<uint8_t[]>(dst_width*dst_height*4);
  encoder = std::make_unique<OmxEncoder>(path.c_str(), dst_width, dst_height, UI_FREQ, 2*1024*1024, false, false);
#endif

  soundStart.setSource(QUrl::fromLocalFile("../assets/sounds/start_record.wav"));
  soundStop.setSource(QUrl::fromLocalFile("../assets/sounds/stop_record.wav"));

  soundStart.setVolume(0.5f);
  soundStop.setVolume(0.5f);
}

ScreenRecoder::~ScreenRecoder() {
  stop(false);
}

void ScreenRecoder::applyColor() {

  if(frame % (UI_FREQ/2) == 0) {

    if(frame % UI_FREQ < (UI_FREQ/2))
      recording_color = QColor::fromRgbF(1, 0, 0, 0.6);
    else
      recording_color = QColor::fromRgbF(0, 0, 0, 0.3);

    update();
  }
}

void ScreenRecoder::paintEvent(QPaintEvent *event) {

    QRect r = QRect(0, 0, width(), height());
    r -= QMargins(5, 5, 5, 5);
    QPainter p(this);
    p.setCompositionMode(QPainter::CompositionMode_SourceOver);
    p.setPen(QPen(QColor::fromRgbF(1, 1, 1, 0.4), 10, Qt::SolidLine, Qt::FlatCap));

    p.setBrush(QBrush(QColor::fromRgbF(0, 0, 0, 0)));
    //p.drawEllipse(r);

    r -= QMargins(40, 40, 40, 40);
    p.setPen(Qt::NoPen);

    QColor bg = recording ? recording_color : QColor::fromRgbF(0, 0, 0, 0.3);
    p.setBrush(QBrush(bg));
    p.drawEllipse(r);
}

void ScreenRecoder::btnReleased(void) {
    toggle();
}

void ScreenRecoder::btnPressed(void) {
}

#ifdef QCOM2
void ScreenRecoder::openEncoder(const char* filename) {
    encoder->encoder_open(filename);
}

void ScreenRecoder::closeEncoder() {
  if(encoder)
      encoder->encoder_close();
}
#endif

void ScreenRecoder::toggle() {

  if(!recording)
      start(true);
  else
      stop(true);
}

void ScreenRecoder::start(bool sound) {

  if(recording)
    return;

  char filename[64];
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);
  snprintf(filename,sizeof(filename),"%04d%02d%02d-%02d%02d%02d.mp4", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

  recording = true;
  frame = 0;
#ifdef QCOM2

  QWidget* widget = this;
  while (widget->parentWidget() != Q_NULLPTR)
    widget = widget->parentWidget();

  rootWidget = widget;

  openEncoder(filename);
  encoding_thread = std::thread([=] { encoding_thread_func(); });
#else
  char cmd[128] = "";
  const char* videos_dir = "/storage/emulated/0/videos";

  struct stat st = {0};
  if (stat(videos_dir, &st) == -1) {
    mkdir(videos_dir,0700);
  }

  snprintf(cmd,sizeof(cmd),"screenrecord --size 1280x720 --bit-rate 2000000 %s/%s&", videos_dir, filename);
  system(cmd);
#endif

  update();

  started = milliseconds();

  if(sound)
      soundStart.play();
}

#ifdef QCOM2
void ScreenRecoder::encoding_thread_func() {

  while(recording && encoder) {
    QImage popImage;
    if(image_queue.pop_wait_for(popImage, std::chrono::milliseconds(10))) {

      QImage image = popImage.convertToFormat(QImage::Format_RGBA8888);

      libyuv::ARGBScale(image.bits(), image.width()*4,
            image.width(), image.height(),
            rgb_scale_buffer.get(), dst_width*4,
            dst_width, dst_height,
            libyuv::kFilterLinear);

      encoder->encode_frame_rgba(rgb_scale_buffer.get(), dst_width, dst_height, (uint64_t)nanos_since_boot());
    }
  }
}
#endif

void ScreenRecoder::stop(bool sound) {

  if(recording) {
    recording = false;
    update();

#ifdef QCOM2
  closeEncoder();
  image_queue.clear();
  if(encoding_thread.joinable())
    encoding_thread.join();
#else
  system("killall -SIGINT screenrecord");
#endif

  if(sound)
      soundStop.play();
  }
}

void ScreenRecoder::update_screen() {

if(recording) {

  if(milliseconds() - started > 1000*60*3) {
    stop(false);
    start(false);
    return;
  }

  applyColor();

#ifdef QCOM2
  if(rootWidget != nullptr) {
    QPixmap pixmap = rootWidget->grab();
    image_queue.push(pixmap.toImage());
  }
#endif
  }

  frame++;
}