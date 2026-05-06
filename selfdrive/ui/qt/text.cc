#include <QApplication>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollBar>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <QProcess>

#include <cstring>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"

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

int main(int argc, char *argv[]) {
  initApp(argc, argv);
  QApplication a(argc, argv);
  QWidget window;
  setMainWindow(&window);

  QGridLayout *main_layout = new QGridLayout(&window);
  main_layout->setMargin(50);
  main_layout->setSpacing(20);

  QLabel *ipLabel = new QLabel(recoveryLabel());
  ipLabel->setObjectName("ipLabel");
  ipLabel->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(ipLabel, 0, 0, Qt::AlignHCenter | Qt::AlignTop);

  QTimer *ipTimer = new QTimer(&window);
  QObject::connect(ipTimer, &QTimer::timeout, [=]() {
    ipLabel->setText(recoveryLabel());
  });
  ipTimer->start(5000);

  QLabel *label = new QLabel(argv[1]);
  label->setWordWrap(true);
  label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
  ScrollView *scroll = new ScrollView(label);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  main_layout->addWidget(scroll, 1, 0, Qt::AlignTop);

  // Scroll to the bottom
  QObject::connect(scroll->verticalScrollBar(), &QAbstractSlider::rangeChanged, [=]() {
    scroll->verticalScrollBar()->setValue(scroll->verticalScrollBar()->maximum());
  });

  QPushButton *btn = new QPushButton();
#ifdef __aarch64__
  btn->setText(QObject::tr("Reboot"));
  QObject::connect(btn, &QPushButton::clicked, [=]() {
    QString cmd = "git pull";
    QProcess::execute(cmd);
    Hardware::reboot();
  });
#else
  btn->setText(QObject::tr("Exit"));
  QObject::connect(btn, &QPushButton::clicked, &a, &QApplication::quit);
#endif
  main_layout->addWidget(btn, 1, 0, Qt::AlignRight | Qt::AlignBottom);

  main_layout->setRowStretch(0, 0);
  main_layout->setRowStretch(1, 1);

  window.setStyleSheet(R"(
    * {
      outline: none;
      color: white;
      background-color: black;
      font-size: 60px;
    }
    QLabel#ipLabel {
      color: white;
      font-size: 50px;
      font-family: monospace;
    }
    QPushButton {
      padding: 50px;
      padding-right: 100px;
      padding-left: 100px;
      border: 2px solid white;
      border-radius: 20px;
      margin-right: 40px;
    }
  )");

  return a.exec();
}
