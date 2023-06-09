#include "tools/cabana/mainwin.h"

#include <iostream>
#include <QClipboard>
#include <QDesktopWidget>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QResizeEvent>
#include <QShortcut>
#include <QTextDocument>
#include <QUndoView>
#include <QVBoxLayout>
#include <QWidgetAction>

#include "tools/cabana/commands.h"
#include "tools/cabana/streamselector.h"
#include "tools/cabana/streams/replaystream.h"

static MainWindow *main_win = nullptr;
void qLogMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg) {
  if (type == QtDebugMsg) std::cout << msg.toStdString() << std::endl;
  if (main_win) emit main_win->showMessage(msg, 2000);
}

MainWindow::MainWindow() : QMainWindow() {
  createDockWindows();
  center_widget = new CenterWidget(charts_widget, this);
  setCentralWidget(center_widget);
  createActions();
  createStatusBar();
  createShortcuts();

  // restore states
  restoreGeometry(settings.geometry);
  if (isMaximized()) {
    setGeometry(QApplication::desktop()->availableGeometry(this));
  }
  restoreState(settings.window_state);

  qRegisterMetaType<uint64_t>("uint64_t");
  qRegisterMetaType<SourceSet>("SourceSet");
  qRegisterMetaType<ReplyMsgType>("ReplyMsgType");
  installMessageHandler([this](ReplyMsgType type, const std::string msg) {
    // use queued connection to recv the log messages from replay.
    emit showMessage(QString::fromStdString(msg), 2000);
  });
  installDownloadProgressHandler([this](uint64_t cur, uint64_t total, bool success) {
    emit updateProgressBar(cur, total, success);
  });

  main_win = this;
  qInstallMessageHandler(qLogMessageHandler);

  for (const QString &fn : {"./dbc/car_fingerprint_to_dbc.json", "./tools/cabana/dbc/car_fingerprint_to_dbc.json"}) {
    QFile json_file(fn);
    if (json_file.open(QIODevice::ReadOnly)) {
      fingerprint_to_dbc = QJsonDocument::fromJson(json_file.readAll());
      break;
    }
  }

  setStyleSheet(QString(R"(QMainWindow::separator {
    width: %1px; /* when vertical */
    height: %1px; /* when horizontal */
  })").arg(style()->pixelMetric(QStyle::PM_SplitterWidth)));

  QObject::connect(this, &MainWindow::showMessage, statusBar(), &QStatusBar::showMessage);
  QObject::connect(this, &MainWindow::updateProgressBar, this, &MainWindow::updateDownloadProgress);
  QObject::connect(messages_widget, &MessagesWidget::msgSelectionChanged, center_widget, &CenterWidget::setMessage);
  QObject::connect(charts_widget, &ChartsWidget::dock, this, &MainWindow::dockCharts);
  QObject::connect(can, &AbstractStream::streamStarted, this, &MainWindow::loadDBCFromFingerprint);
  QObject::connect(can, &AbstractStream::eventsMerged, this, &MainWindow::updateStatus);
  QObject::connect(dbc(), &DBCManager::DBCFileChanged, this, &MainWindow::DBCFileChanged);
  QObject::connect(can, &AbstractStream::sourcesUpdated, dbc(), &DBCManager::updateSources);
  QObject::connect(can, &AbstractStream::sourcesUpdated, this, &MainWindow::updateSources);
  QObject::connect(UndoStack::instance(), &QUndoStack::cleanChanged, this, &MainWindow::undoStackCleanChanged);
  QObject::connect(UndoStack::instance(), &QUndoStack::indexChanged, this, &MainWindow::undoStackIndexChanged);
  QObject::connect(&settings, &Settings::changed, this, &MainWindow::updateStatus);
}

void MainWindow::createActions() {
  QMenu *file_menu = menuBar()->addMenu(tr("&File"));
  if (!can->liveStreaming()) {
    file_menu->addAction(tr("Open Route..."), this, &MainWindow::openRoute);
    file_menu->addSeparator();
  }

  file_menu->addAction(tr("New DBC File"), this, &MainWindow::newFile)->setShortcuts(QKeySequence::New);
  file_menu->addAction(tr("Open DBC File..."), this, &MainWindow::openFile)->setShortcuts(QKeySequence::Open);

  manage_dbcs_menu = file_menu->addMenu(tr("Manage &DBC Files"));

  open_recent_menu = file_menu->addMenu(tr("Open &Recent"));
  for (int i = 0; i < MAX_RECENT_FILES; ++i) {
    recent_files_acts[i] = new QAction(this);
    recent_files_acts[i]->setVisible(false);
    QObject::connect(recent_files_acts[i], &QAction::triggered, this, &MainWindow::openRecentFile);
    open_recent_menu->addAction(recent_files_acts[i]);
  }
  updateRecentFileActions();

  file_menu->addSeparator();
  QMenu *load_opendbc_menu = file_menu->addMenu(tr("Load DBC from commaai/opendbc"));
  // load_opendbc_menu->setStyleSheet("QMenu { menu-scrollable: true; }");
  auto dbc_names = allDBCNames();
  std::sort(dbc_names.begin(), dbc_names.end());
  for (const auto &name : dbc_names) {
    load_opendbc_menu->addAction(QString::fromStdString(name), this, &MainWindow::openOpendbcFile);
  }

  file_menu->addAction(tr("Load DBC From Clipboard"), [=]() { loadFromClipboard(); });

  file_menu->addSeparator();
  save_dbc = file_menu->addAction(tr("Save DBC..."), this, &MainWindow::save);
  save_dbc->setShortcuts(QKeySequence::Save);

  save_dbc_as = file_menu->addAction(tr("Save DBC As..."), this, &MainWindow::saveAs);
  save_dbc_as->setShortcuts(QKeySequence::SaveAs);

  copy_dbc_to_clipboard = file_menu->addAction(tr("Copy DBC To Clipboard"), this, &MainWindow::saveToClipboard);

  file_menu->addSeparator();
  file_menu->addAction(tr("Settings..."), this, &MainWindow::setOption)->setShortcuts(QKeySequence::Preferences);

  file_menu->addSeparator();
  file_menu->addAction(tr("E&xit"), qApp, &QApplication::closeAllWindows)->setShortcuts(QKeySequence::Quit);

  QMenu *edit_menu = menuBar()->addMenu(tr("&Edit"));
  auto undo_act = UndoStack::instance()->createUndoAction(this, tr("&Undo"));
  undo_act->setShortcuts(QKeySequence::Undo);
  edit_menu->addAction(undo_act);
  auto redo_act = UndoStack::instance()->createRedoAction(this, tr("&Rndo"));
  redo_act->setShortcuts(QKeySequence::Redo);
  edit_menu->addAction(redo_act);
  edit_menu->addSeparator();

  QMenu *commands_menu = edit_menu->addMenu(tr("Command &List"));
  auto undo_view = new QUndoView(UndoStack::instance());
  undo_view->setWindowTitle(tr("Command List"));
  QWidgetAction *commands_act = new QWidgetAction(this);
  commands_act->setDefaultWidget(undo_view);
  commands_menu->addAction(commands_act);

  QMenu *tools_menu = menuBar()->addMenu(tr("&Tools"));
  tools_menu->addAction(tr("Find &Similar Bits"), this, &MainWindow::findSimilarBits);

  QMenu *help_menu = menuBar()->addMenu(tr("&Help"));
  help_menu->addAction(tr("Help"), this, &MainWindow::onlineHelp)->setShortcuts(QKeySequence::HelpContents);
  help_menu->addAction(tr("About &Qt"), qApp, &QApplication::aboutQt);
}

void MainWindow::createDockWindows() {
  // left panel
  messages_widget = new MessagesWidget(this);
  QDockWidget *dock = new QDockWidget(tr("MESSAGES"), this);
  dock->setObjectName("MessagesPanel");
  dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::TopDockWidgetArea | Qt::BottomDockWidgetArea);
  dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
  dock->setWidget(messages_widget);
  addDockWidget(Qt::LeftDockWidgetArea, dock);

  // right panel
  charts_widget = new ChartsWidget(this);
  QWidget *charts_container = new QWidget(this);
  charts_layout = new QVBoxLayout(charts_container);
  charts_layout->setContentsMargins(0, 0, 0, 0);
  charts_layout->addWidget(charts_widget);

  // splitter between video and charts
  video_splitter = new QSplitter(Qt::Vertical, this);
  video_widget = new VideoWidget(this);
  video_splitter->addWidget(video_widget);
  QObject::connect(charts_widget, &ChartsWidget::rangeChanged, video_widget, &VideoWidget::rangeChanged);

  video_splitter->addWidget(charts_container);
  video_splitter->setStretchFactor(1, 1);
  video_splitter->restoreState(settings.video_splitter_state);
  if (can->liveStreaming() || video_splitter->sizes()[0] == 0) {
    // display video at minimum size.
    video_splitter->setSizes({1, 1});
  }

  video_dock = new QDockWidget(can->routeName(), this);
  video_dock->setObjectName(tr("VideoPanel"));
  video_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  video_dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
  video_dock->setWidget(video_splitter);
  addDockWidget(Qt::RightDockWidgetArea, video_dock);
}

void MainWindow::createStatusBar() {
  progress_bar = new QProgressBar();
  progress_bar->setRange(0, 100);
  progress_bar->setTextVisible(true);
  progress_bar->setFixedSize({300, 16});
  progress_bar->setVisible(false);
  statusBar()->addWidget(new QLabel(tr("For Help, Press F1")));
  statusBar()->addPermanentWidget(progress_bar);

  statusBar()->addPermanentWidget(status_label = new QLabel(this));
  updateStatus();
}

void MainWindow::createShortcuts() {
  auto shortcut = new QShortcut(QKeySequence(Qt::Key_Space), this, nullptr, nullptr, Qt::ApplicationShortcut);
  QObject::connect(shortcut, &QShortcut::activated, []() { can->pause(!can->isPaused()); });
  shortcut = new QShortcut(QKeySequence(QKeySequence::FullScreen), this, nullptr, nullptr, Qt::ApplicationShortcut);
  QObject::connect(shortcut, &QShortcut::activated, this, &MainWindow::toggleFullScreen);
  // TODO: add more shortcuts here.
}

void MainWindow::undoStackIndexChanged(int index) {
  int count = UndoStack::instance()->count();
  if (count >= 0) {
    QString command_text;
    if (index == count) {
      command_text = (count == prev_undostack_count ? "Redo " : "") + UndoStack::instance()->text(index - 1);
    } else if (index < prev_undostack_index) {
      command_text = tr("Undo %1").arg(UndoStack::instance()->text(index));
    } else if (index > prev_undostack_index) {
      command_text = tr("Redo %1").arg(UndoStack::instance()->text(index - 1));
    }
    statusBar()->showMessage(command_text, 2000);
  }
  prev_undostack_index = index;
  prev_undostack_count = count;
  autoSave();
  updateLoadSaveMenus();
}

void MainWindow::undoStackCleanChanged(bool clean) {
  if (clean) {
    prev_undostack_index = 0;
    prev_undostack_count = 0;
  }
  setWindowModified(!clean);
}

void MainWindow::DBCFileChanged() {
  UndoStack::instance()->clear();
  updateLoadSaveMenus();
}

void MainWindow::openRoute() {
  StreamSelector dlg(this);
  dlg.addStreamWidget(ReplayStream::widget(&can));
  if (dlg.exec()) {
    center_widget->clear();
    charts_widget->removeAll();
    statusBar()->showMessage(tr("Route %1 loaded").arg(can->routeName()), 2000);
  } else if (dlg.failed()) {
    close();
  }
}

void MainWindow::newFile() {
  remindSaveChanges();
  dbc()->closeAll();
  dbc()->open(SOURCE_ALL, "", "");
  updateLoadSaveMenus();
}

void MainWindow::openFile() {
  remindSaveChanges();
  QString fn = QFileDialog::getOpenFileName(this, tr("Open File"), settings.last_dir, "DBC (*.dbc)");
  if (!fn.isEmpty()) {
    loadFile(fn);
  }
}

void MainWindow::openFileForSource(SourceSet s) {
  QString fn = QFileDialog::getOpenFileName(this, tr("Open File"), settings.last_dir, "DBC (*.dbc)");
  if (!fn.isEmpty()) {
    loadFile(fn, s, false);
  }
}

void MainWindow::newFileForSource(SourceSet s) {
  remindSaveChanges();

  dbc()->close(s);
  dbc()->open(s, "", "");
}

void MainWindow::loadFile(const QString &fn, SourceSet s, bool close_all) {
  if (!fn.isEmpty()) {
    QString dbc_fn = fn;

    // Prompt user to load auto saved file if it exists.
    if (QFile::exists(fn + AUTO_SAVE_EXTENSION)) {
      auto ret = QMessageBox::question(this, tr("Auto saved DBC found"), tr("Auto saved DBC file from previous session found. Do you want to load it instead?"));
      if (ret == QMessageBox::Yes) {
        dbc_fn += AUTO_SAVE_EXTENSION;
        UndoStack::instance()->resetClean(); // Force user to save on close so the auto saved file is not lost
      }
    }

    auto dbc_name = QFileInfo(fn).baseName();
    QString error;

    if (close_all) {
      dbc()->closeAll();
    }

    dbc()->close(s);
    bool ret = dbc()->open(s, dbc_fn, &error);
    if (ret) {
      updateRecentFiles(fn);
      statusBar()->showMessage(tr("DBC File %1 loaded").arg(fn), 2000);
    } else {
      QMessageBox msg_box(QMessageBox::Warning, tr("Failed to load DBC file"), tr("Failed to parse DBC file %1").arg(fn));
      msg_box.setDetailedText(error);
      msg_box.exec();
    }

    updateLoadSaveMenus();
  }
}

void MainWindow::openOpendbcFile() {
  if (auto action = qobject_cast<QAction *>(sender())) {
    remindSaveChanges();
    loadDBCFromOpendbc(action->text());
  }
}

void MainWindow::openRecentFile() {
  if (auto action = qobject_cast<QAction *>(sender())) {
    remindSaveChanges();
    loadFile(action->data().toString());
  }
}

void MainWindow::loadDBCFromOpendbc(const QString &name) {
  remindSaveChanges();

  QString opendbc_file_path = QString("%1/%2.dbc").arg(OPENDBC_FILE_PATH, name);

  dbc()->closeAll();
  dbc()->open(SOURCE_ALL, opendbc_file_path);

  updateLoadSaveMenus();
}

void MainWindow::loadFromClipboard(SourceSet s, bool close_all) {
  remindSaveChanges();
  QString dbc_str = QGuiApplication::clipboard()->text();
  QString error;

  if (close_all) {
    dbc()->closeAll();
  }

  dbc()->close(s);
  bool ret = dbc()->open(s, "", dbc_str, &error);
  if (ret && dbc()->msgCount() > 0) {
    QMessageBox::information(this, tr("Load From Clipboard"), tr("DBC Successfully Loaded!"));
  } else {
    QMessageBox msg_box(QMessageBox::Warning, tr("Failed to load DBC from clipboard"), tr("Make sure that you paste the text with correct format."));
    if (!error.isEmpty()) {
      msg_box.setDetailedText(error);
    }
    msg_box.exec();
  }
}

void MainWindow::loadDBCFromFingerprint() {
  // Don't overwrite already loaded DBC
  if (dbc()->msgCount()) {
    return;
  }

  remindSaveChanges();
  auto fingerprint = can->carFingerprint();
  if (can->liveStreaming()) {
    video_dock->setWindowTitle(can->routeName());
  } else {
    video_dock->setWindowTitle(tr("ROUTE: %1  FINGERPRINT: %2").arg(can->routeName()).arg(fingerprint.isEmpty() ? tr("Unknown Car") : fingerprint));
  }
  if (!fingerprint.isEmpty()) {
    auto dbc_name = fingerprint_to_dbc[fingerprint];
    if (dbc_name != QJsonValue::Undefined) {
      loadDBCFromOpendbc(dbc_name.toString());
      return;
    }
  }
  newFile();
}

void MainWindow::save() {
  // Save all open DBC files
  for (auto &[s, dbc_file] : dbc()->dbc_files) {
    if (dbc_file->isEmpty()) continue;
    saveFile(dbc_file);
  }
}

void MainWindow::saveAs() {
  // Save as all open DBC files. Should not be called with more than 1 file open
  for (auto &[s, dbc_file] : dbc()->dbc_files) {
    if (dbc_file->isEmpty()) continue;
    saveFileAs(dbc_file);
  }
}


void MainWindow::autoSave() {
  if (!UndoStack::instance()->isClean()) {
    for (auto &[_, dbc_file] : dbc()->dbc_files) {
      if (!dbc_file->filename.isEmpty()) {
        dbc_file->autoSave();
      }
    }
  }
}

void MainWindow::cleanupAutoSaveFile() {
  for (auto &[_, dbc_file] : dbc()->dbc_files) {
    dbc_file->cleanupAutoSaveFile();
  }
}

void MainWindow::closeFile(DBCFile *dbc_file) {
  assert(dbc_file != nullptr);
  remindSaveChanges();

  dbc()->close(dbc_file);

  // Ensure we always have at least one file open
  if (dbc()->dbcCount() == 0) {
    newFile();
  }
}

void MainWindow::saveFile(DBCFile *dbc_file) {
  assert(dbc_file != nullptr);

  SourceSet s;
  for (auto &[s_, dbc_file_] : dbc()->dbc_files) {
    if (dbc_file_ == dbc_file) {
      s = s_;
      break;
    }
  }

  if (!dbc_file->filename.isEmpty()) {
    dbc_file->save();
    updateRecentFiles(dbc_file->filename);
  } else if (!dbc_file->isEmpty()) {
    QString fn = QFileDialog::getSaveFileName(this, tr("Save File (bus: %1)").arg(toString(s)), QDir::cleanPath(settings.last_dir + "/untitled.dbc"), tr("DBC (*.dbc)"));
    if (!fn.isEmpty()) {
      dbc_file->saveAs(fn);
      updateRecentFiles(fn);
    }
  }

  UndoStack::instance()->setClean();
  statusBar()->showMessage(tr("File saved"), 2000);
  updateLoadSaveMenus();
}

void MainWindow::saveFileAs(DBCFile *dbc_file) {
  assert(dbc_file != nullptr);

  SourceSet s;
  for (auto &[s_, dbc_file_] : dbc()->dbc_files) {
    if (dbc_file_ == dbc_file) {
      s = s_;
      break;
    }
  }

  QString fn = QFileDialog::getSaveFileName(this, tr("Save File (bus: %1)").arg(toString(s)), QDir::cleanPath(settings.last_dir + "/untitled.dbc"), tr("DBC (*.dbc)"));
  if (!fn.isEmpty()) {
    dbc_file->saveAs(fn);
    updateRecentFiles(fn);
    updateLoadSaveMenus();
  }
}

void MainWindow::removeBusFromFile(DBCFile *dbc_file, uint8_t source) {
  assert(dbc_file != nullptr);
  SourceSet ss = {source, uint8_t(source + 128), uint8_t(source + 192)};
  dbc()->removeSourcesFromFile(dbc_file, ss);
  updateLoadSaveMenus();
}


void MainWindow::saveToClipboard() {
  // Copy all open DBC files to clipboard. Should not be called with more than 1 file open
  for (auto &[s, dbc_file] : dbc()->dbc_files) {
    if (dbc_file->isEmpty()) continue;
    saveFileToClipboard(dbc_file);
  }
}

void MainWindow::saveFileToClipboard(DBCFile *dbc_file) {
  assert(dbc_file != nullptr);
  QGuiApplication::clipboard()->setText(dbc_file->generateDBC());
  QMessageBox::information(this, tr("Copy To Clipboard"), tr("DBC Successfully copied!"));
}

void MainWindow::updateSources(const SourceSet &s) {
  sources = s;
  updateLoadSaveMenus();
}

void MainWindow::updateLoadSaveMenus() {
  int cnt = dbc()->nonEmptyDBCCount();
  save_dbc->setEnabled(cnt > 0);

  if (cnt > 1) {
    save_dbc->setText(tr("Save %1 DBCs...").arg(dbc()->dbcCount()));
  } else {
    save_dbc->setText(tr("Save DBC..."));
  }

  save_dbc_as->setEnabled(cnt == 1);

  // TODO: Support clipboard for multiple files
  copy_dbc_to_clipboard->setEnabled(cnt == 1);


  QList<uint8_t> sources_sorted = sources.toList();
  std::sort(sources_sorted.begin(), sources_sorted.end());

  manage_dbcs_menu->clear();

  for (uint8_t source : sources_sorted) {
    if (source >= 64) continue; // Sent and blocked buses are handled implicitly

    SourceSet ss = {source, uint8_t(source + 128), uint8_t(source + 192)};

    QMenu *bus_menu = new QMenu(this);

    // New
    QAction *new_action = new QAction(this);
    new_action->setText(tr("New DBC File..."));
    QObject::connect(new_action, &QAction::triggered, [=]() { newFileForSource(ss); });
    bus_menu->addAction(new_action);

    // Open
    QAction *open_action = new QAction(this);
    open_action->setText(tr("Open DBC File..."));
    QObject::connect(open_action, &QAction::triggered, [=]() { openFileForSource(ss); });
    bus_menu->addAction(open_action);

    // Open
    QAction *load_clipboard_action = new QAction(this);
    load_clipboard_action->setText(tr("Load DBC From Clipboard..."));
    QObject::connect(load_clipboard_action, &QAction::triggered, [=]() { loadFromClipboard(ss, false); });
    bus_menu->addAction(load_clipboard_action);

    // Show sub-menu for each dbc for this source.
    QStringList bus_menu_fns;
    for (auto it : dbc()->dbc_files) {
      auto &[src, dbc_file] = it;
      if (!src.contains(source) && (src != SOURCE_ALL)) {
        continue;
      }

      QString fn = dbc_file->filename.isEmpty() ? "untitled" : QFileInfo(dbc_file->filename).baseName();
      // QMenu *manage_menu = bus_menu;

      bus_menu->addSeparator();
      QAction *fn_action = new QAction(this);
      fn_action->setText(fn + " (" + toString(src) + ")");
      fn_action->setEnabled(false);
      bus_menu->addAction(fn_action);

      // Save
      QAction *save_action = new QAction(this);
      save_action->setText(tr("Save..."));
      bus_menu->addAction(save_action);
      QObject::connect(save_action, &QAction::triggered, [=](){ saveFile(it.second); });

      // Save as
      QAction *save_as_action = new QAction(this);
      save_as_action->setText(tr("Save As..."));
      bus_menu->addAction(save_as_action);
      QObject::connect(save_as_action, &QAction::triggered, [=](){ saveFileAs(it.second); });

      // Copy to clipboard
      QAction *save_clipboard_action = new QAction(this);
      save_clipboard_action->setText(tr("Copy to Clipboard..."));
      bus_menu->addAction(save_clipboard_action);
      QObject::connect(save_clipboard_action, &QAction::triggered, [=](){ saveFileToClipboard(it.second); });

      // Remove from this bus
      QAction *remove_action = new QAction(this);
      remove_action->setText(tr("Remove from this bus..."));
      bus_menu->addAction(remove_action);
      QObject::connect(remove_action, &QAction::triggered, [=](){ removeBusFromFile(it.second, source); });

      // Close/Remove from all buses
      QAction *close_action = new QAction(this);
      close_action->setText(tr("Remove from all buses..."));
      bus_menu->addAction(close_action);
      QObject::connect(close_action, &QAction::triggered, [=](){ closeFile(it.second); });

      bus_menu_fns << fn;
    }

    manage_dbcs_menu->addMenu(bus_menu);
    QString bus_menu_title = bus_menu_fns.size() ? bus_menu_fns.join(", ") : "No DBCs loaded";
    bus_menu->setTitle(tr("Bus %1 (%2)").arg(source).arg(bus_menu_title));
  }

  QStringList title;
  for (auto &[src, dbc_file] : dbc()->dbc_files) {
    QString fn = dbc_file->filename.isEmpty() ? "untitled" : QFileInfo(dbc_file->filename).baseName();
    title.push_back(tr("(%1) %2").arg(toString(src)).arg(fn));
  }
  setWindowFilePath(title.join(" | "));
}

void MainWindow::updateRecentFiles(const QString &fn) {
  settings.recent_files.removeAll(fn);
  settings.recent_files.prepend(fn);
  while (settings.recent_files.size() > MAX_RECENT_FILES) {
    settings.recent_files.removeLast();
  }
  settings.last_dir = QFileInfo(fn).absolutePath();
  updateRecentFileActions();
}

void MainWindow::updateRecentFileActions() {
  int num_recent_files = std::min<int>(settings.recent_files.size(), MAX_RECENT_FILES);

  for (int i = 0; i < num_recent_files; ++i) {
    QString text = tr("&%1 %2").arg(i + 1).arg(QFileInfo(settings.recent_files[i]).fileName());
    recent_files_acts[i]->setText(text);
    recent_files_acts[i]->setData(settings.recent_files[i]);
    recent_files_acts[i]->setVisible(true);
  }
  for (int i = num_recent_files; i < MAX_RECENT_FILES; ++i) {
    recent_files_acts[i]->setVisible(false);
  }
  open_recent_menu->setEnabled(num_recent_files > 0);
}

void MainWindow::remindSaveChanges() {
  bool discard_changes = false;
  while (!UndoStack::instance()->isClean() && !discard_changes) {
    int ret = (QMessageBox::question(this, tr("Unsaved Changes"),
                                     tr("You have unsaved changes. Press ok to save them, cancel to discard."),
                                     QMessageBox::Ok | QMessageBox::Cancel));
    if (ret == QMessageBox::Ok) {
      save();
    } else {
      discard_changes = true;
    }
  }
  UndoStack::instance()->clear();
}

void MainWindow::updateDownloadProgress(uint64_t cur, uint64_t total, bool success) {
  if (success && cur < total) {
    progress_bar->setValue((cur / (double)total) * 100);
    progress_bar->setFormat(tr("Downloading %p% (%1)").arg(formattedDataSize(total).c_str()));
    progress_bar->show();
  } else {
    progress_bar->hide();
  }
}

void MainWindow::updateStatus() {
  status_label->setText(tr("Cached Minutes:%1 FPS:%2").arg(settings.max_cached_minutes).arg(settings.fps));
}

void MainWindow::dockCharts(bool dock) {
  if (dock && floating_window) {
    floating_window->removeEventFilter(charts_widget);
    charts_layout->insertWidget(0, charts_widget, 1);
    floating_window->deleteLater();
    floating_window = nullptr;
  } else if (!dock && !floating_window) {
    floating_window = new QWidget(this);
    floating_window->setWindowFlags(Qt::Window);
    floating_window->setWindowTitle("Charts");
    floating_window->setLayout(new QVBoxLayout());
    floating_window->layout()->addWidget(charts_widget);
    floating_window->installEventFilter(charts_widget);
    floating_window->showMaximized();
  }
}

void MainWindow::closeEvent(QCloseEvent *event) {
  cleanupAutoSaveFile();
  remindSaveChanges();

  main_win = nullptr;
  if (floating_window)
    floating_window->deleteLater();

  // save states
  settings.geometry = saveGeometry();
  settings.window_state = saveState();
  if (!can->liveStreaming()) {
    settings.video_splitter_state = video_splitter->saveState();
  }
  settings.message_header_state = messages_widget->saveHeaderState();
  settings.save();
  QWidget::closeEvent(event);
}

void MainWindow::setOption() {
  SettingsDlg dlg(this);
  dlg.exec();
}

void MainWindow::findSimilarBits() {
  FindSimilarBitsDlg *dlg = new FindSimilarBitsDlg(this);
  QObject::connect(dlg, &FindSimilarBitsDlg::openMessage, messages_widget, &MessagesWidget::selectMessage);
  dlg->show();
}

void MainWindow::onlineHelp() {
  if (auto help = findChild<HelpOverlay*>()) {
    help->close();
  } else {
    help = new HelpOverlay(this);
    help->setGeometry(rect());
    help->show();
    help->raise();
  }
}

void MainWindow::toggleFullScreen() {
  if (isFullScreen()) {
    menuBar()->show();
    statusBar()->show();
    showNormal();
    showMaximized();
  } else {
    menuBar()->hide();
    statusBar()->hide();
    showFullScreen();
  }
}

// HelpOverlay
HelpOverlay::HelpOverlay(MainWindow *parent) : QWidget(parent) {
  setAttribute(Qt::WA_NoSystemBackground, true);
  setAttribute(Qt::WA_TranslucentBackground, true);
  setAttribute(Qt::WA_DeleteOnClose);
  parent->installEventFilter(this);
}

void HelpOverlay::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.fillRect(rect(), QColor(0, 0, 0, 50));
  MainWindow *parent = (MainWindow *)parentWidget();
  drawHelpForWidget(painter, parent->findChild<MessagesWidget *>());
  drawHelpForWidget(painter, parent->findChild<BinaryView *>());
  drawHelpForWidget(painter, parent->findChild<SignalView *>());
  drawHelpForWidget(painter, parent->findChild<ChartsWidget *>());
  drawHelpForWidget(painter, parent->findChild<VideoWidget *>());
}

void HelpOverlay::drawHelpForWidget(QPainter &painter, QWidget *w) {
  if (w && w->isVisible() && !w->whatsThis().isEmpty()) {
    QPoint pt = mapFromGlobal(w->mapToGlobal(w->rect().center()));
    if (rect().contains(pt)) {
      QTextDocument document;
      document.setHtml(w->whatsThis());
      QSize doc_size = document.size().toSize();
      QPoint topleft = {pt.x() - doc_size.width() / 2, pt.y() - doc_size.height() / 2};
      painter.translate(topleft);
      painter.fillRect(QRect{{0, 0}, doc_size}, palette().toolTipBase());
      document.drawContents(&painter);
      painter.translate(-topleft);
    }
  }
}

bool HelpOverlay::eventFilter(QObject *obj, QEvent *event) {
  if (obj == parentWidget() && event->type() == QEvent::Resize) {
    QResizeEvent *resize_event = (QResizeEvent *)(event);
    setGeometry(QRect{QPoint(0, 0), resize_event->size()});
  }
  return false;
}

void HelpOverlay::mouseReleaseEvent(QMouseEvent *event) {
  close();
}
