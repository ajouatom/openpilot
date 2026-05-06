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
#include <QGridLayout>
#include <QPainter>
#include <QString>
#include <QTimer>
#include <QTransform>

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
  main_layout->setMargin(200);

  ipLabel = new QLabel(recoveryLabel());
  ipLabel->setObjectName("ipLabel");
  ipLabel->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(ipLabel, 0, 0, Qt::AlignHCenter | Qt::AlignTop);

  main_layout->addWidget(new TrackWidget(this), 1, 0, Qt::AlignHCenter | Qt::AlignVCenter);

  text = new QLabel();
  text->setWordWrap(true);
  text->setVisible(false);
  text->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(text, 2, 0, Qt::AlignHCenter);

  progress_bar = new QProgressBar();
  progress_bar->setRange(5, 100);
  progress_bar->setTextVisible(false);
  progress_bar->setVisible(false);
  progress_bar->setFixedHeight(20);
  main_layout->addWidget(progress_bar, 2, 0, Qt::AlignHCenter);

  main_layout->setRowStretch(0, 0);
  main_layout->setRowStretch(1, 1);
  main_layout->setRowStretch(2, 0);

  QTimer *ipTimer = new QTimer(this);
  QObject::connect(ipTimer, &QTimer::timeout, this, &Spinner::refreshIPLabel);
  ipTimer->start(5000);

  setStyleSheet(R"(
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
      font-size: 50px;
      font-family: monospace;
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
  )");

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
    text->setVisible(!number);
    progress_bar->setVisible(number);
    text->setText(QString::fromStdString(line));
    if (number) {
      progress_bar->setValue(std::stoi(line));
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
