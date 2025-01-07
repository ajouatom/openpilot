#include <csignal>
#include <sys/resource.h>

#include <QApplication>
#include <QDebug>

#include "common/util.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/navd/map_renderer.h"
#include "system/hardware/hw.h"

void customMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg) {
    QByteArray localMsg = msg.toLocal8Bit();
    const char* file = context.file ? context.file : "";
    const char* function = context.function ? context.function : "";
    int line = context.line;

    // 메시지 타입에 따라 출력 형식 설정
    switch (type) {
    case QtDebugMsg:
        printf("Debug: %s (%s:%d, %s)\n", localMsg.constData(), file, line, function);
        break;
    case QtInfoMsg:
        printf("Info: %s (%s:%d, %s)\n", localMsg.constData(), file, line, function);
        break;
    case QtWarningMsg:
        printf("Warning: %s (%s:%d, %s)\n", localMsg.constData(), file, line, function);
        break;
    case QtCriticalMsg:
        printf("Critical: %s (%s:%d, %s)\n", localMsg.constData(), file, line, function);
        break;
    case QtFatalMsg:
        printf("Fatal: %s (%s:%d, %s)\n", localMsg.constData(), file, line, function);
        abort();
    }
}

int main(int argc, char *argv[]) {
  Hardware::config_cpu_rendering(true);

  qInstallMessageHandler(customMessageHandler);
  setpriority(PRIO_PROCESS, 0, -20);
  int ret = util::set_core_affinity({0, 1, 2, 3});
  assert(ret == 0);

  //setenv("QT_QPA_PLATFORM", "xcb", 1);
  //setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
  //setenv("QT_LOGGING_RULES", "qt.qpa.*=true;qt.opengl.*=true", 1);

  QApplication app(argc, argv);
  std::signal(SIGINT, sigTermHandler);
  std::signal(SIGTERM, sigTermHandler);

  MapRenderer * m = new MapRenderer(get_mapbox_settings());
  assert(m);

  return app.exec();
}
