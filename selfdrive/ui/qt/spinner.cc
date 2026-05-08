#include "selfdrive/ui/qt/spinner.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <QApplication>
#include <QCoreApplication>
#include <QFont>
#include <QFontDatabase>
#include <QGridLayout>
#include <QPainter>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QTransform>
#include <QVBoxLayout>

#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/util.h"

static QString getLocalIPv4() {
  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) return QString();

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 300000;
  ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  struct sockaddr_in target;
  std::memset(&target, 0, sizeof(target));
  target.sin_family = AF_INET;
  target.sin_port = htons(80);
  ::inet_pton(AF_INET, "8.8.8.8", &target.sin_addr);

  if (::connect(sock, (struct sockaddr*)&target, sizeof(target)) < 0) {
    ::close(sock);
    return QString();
  }

  struct sockaddr_in local;
  socklen_t local_len = sizeof(local);
  if (::getsockname(sock, (struct sockaddr*)&local, &local_len) < 0) {
    ::close(sock);
    return QString();
  }

  char ip_str[INET_ADDRSTRLEN];
  ::inet_ntop(AF_INET, &local.sin_addr, ip_str, INET_ADDRSTRLEN);
  ::close(sock);
  return QString(ip_str);
}

static QString recoveryLabel() {
  QString ip = getLocalIPv4();
  if (ip.isEmpty()) return QStringLiteral("no network");
  return ip + QStringLiteral(":6999");
}

static QString spinnerFontFamily() {
  QString font_path = QCoreApplication::applicationDirPath() + "/../assets/fonts/Pretendard-SemiBold.ttf";
  int font_id = QFontDatabase::addApplicationFont(font_path);
  if (font_id < 0) return QStringLiteral("Inter");

  QStringList families = QFontDatabase::applicationFontFamilies(font_id);
  return families.isEmpty() ? QStringLiteral("Inter") : families.first();
}

TrackWidget::TrackWidget(QWidget *parent) : QWidget(parent) {
  setAttribute(Qt::WA_OpaquePaintEvent);
  setFixedSize(spinner_size);

  // pre-compute all the track imgs. make this a gif instead?
  QPixmap comma_img = loadPixmap("../assets/img_spinner_comma.png", spinner_size);
  QPixmap track_img = loadPixmap("../assets/img_spinner_track.png", spinner_size);

  QTransform transform(1, 0, 0, 1, width() / 2, height() / 2);
  QPixmap pm(spinner_size);
  QPainter p(&pm);
  p.setRenderHint(QPainter::SmoothPixmapTransform);
  for (int i = 0; i < track_imgs.size(); ++i) {
    p.resetTransform();
    p.fillRect(0, 0, spinner_size.width(), spinner_size.height(), Qt::black);
    p.drawPixmap(0, 0, comma_img);
    p.setTransform(transform.rotate(360 / spinner_fps));
    p.drawPixmap(-width() / 2, -height() / 2, track_img);
    track_imgs[i] = pm.copy();
  }

  m_anim.setDuration(1000);
  m_anim.setStartValue(0);
  m_anim.setEndValue(int(track_imgs.size() -1));
  m_anim.setLoopCount(-1);
  m_anim.start();
  connect(&m_anim, SIGNAL(valueChanged(QVariant)), SLOT(update()));
}

void TrackWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.drawPixmap(0, 0, track_imgs[m_anim.currentValue().toInt()]);
}

// Spinner

Spinner::Spinner(QWidget *parent) : QWidget(parent) {
  QGridLayout *main_layout = new QGridLayout(this);
  main_layout->setSpacing(0);
  main_layout->setContentsMargins(200, 170, 200, 70);

  QString font_family = spinnerFontFamily();

  ipLabel = new QLabel(recoveryLabel());
  ipLabel->setObjectName("ipLabel");
  ipLabel->setFont(QFont(font_family, 84));
  ipLabel->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(ipLabel, 0, 0, Qt::AlignHCenter | Qt::AlignTop);

  main_layout->addWidget(new TrackWidget(this), 1, 0, Qt::AlignHCenter | Qt::AlignVCenter);

  text = new QLabel();
  text->setObjectName("statusLabel");
  text->setFixedWidth(1000);
  text->setFixedHeight(68);
  text->setFont(QFont(font_family, 52));
  text->setWordWrap(false);
  text->setVisible(false);
  text->setAlignment(Qt::AlignCenter);

  progress_bar = new QProgressBar();
  progress_bar->setRange(5, 100);
  progress_bar->setTextVisible(false);
  progress_bar->setVisible(false);
  progress_bar->setFixedWidth(1000);
  progress_bar->setFixedHeight(20);

  QVBoxLayout *progress_layout = new QVBoxLayout();
  progress_layout->setContentsMargins(0, 0, 0, 0);
  progress_layout->setSpacing(6);
  progress_layout->addWidget(text, 0, Qt::AlignHCenter);
  progress_layout->addWidget(progress_bar, 0, Qt::AlignHCenter);
  main_layout->addLayout(progress_layout, 2, 0, Qt::AlignHCenter);

  main_layout->setRowStretch(0, 0);
  main_layout->setRowStretch(1, 1);
  main_layout->setRowStretch(2, 0);

  QTimer *ipTimer = new QTimer(this);
  QObject::connect(ipTimer, &QTimer::timeout, this, &Spinner::refreshIPLabel);
  ipTimer->start(5000);

  setStyleSheet(QString(R"(
    Spinner {
      background-color: black;
    }
    QLabel {
      color: white;
      font-size: 80px;
      background-color: transparent;
    }
    QLabel#ipLabel {
      color: white;
      font-size: 84px;
      font-family: "%1";
    }
    QLabel#statusLabel {
      color: #cfcfcf;
      font-size: 52px;
      font-family: "%1";
    }
    QProgressBar {
      background-color: #373737;
      width: 1000px;
      border solid white;
      border-radius: 10px;
    }
    QProgressBar::chunk {
      border-radius: 10px;
      background-color: white;
    }
  )").arg(font_family));

  notifier = new QSocketNotifier(fileno(stdin), QSocketNotifier::Read);
  QObject::connect(notifier, &QSocketNotifier::activated, this, &Spinner::update);
}

void Spinner::refreshIPLabel() {
  if (ipLabel) ipLabel->setText(recoveryLabel());
}

void Spinner::update(int n) {
  std::string line;
  std::getline(std::cin, line);

  if (line.length()) {
    bool number = std::all_of(line.begin(), line.end(), ::isdigit);
    if (number) {
      progress_bar->setVisible(true);
      progress_bar->setValue(std::stoi(line));
    } else {
      QString status = QString::fromStdString(line);
      text->setText(text->fontMetrics().elidedText(status, Qt::ElideRight, text->width()));
      text->setVisible(true);
    }
  }
}

int main(int argc, char *argv[]) {
  initApp(argc, argv);
  QApplication a(argc, argv);
  Spinner spinner;
  setMainWindow(&spinner);
  return a.exec();
}
