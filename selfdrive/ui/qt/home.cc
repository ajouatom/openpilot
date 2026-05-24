#include "selfdrive/ui/qt/home.h"

#include <QHBoxLayout>
#include <QMouseEvent>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>

#include "selfdrive/ui/qt/offroad/experimental_mode.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/prime.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "common/params.h"
#include <QScrollArea>
#include <QScroller>
#include <QDialogButtonBox>
#include <QCheckBox>
#include <QMap>
#include <QDateTime>
#include <QDialogButtonBox>
#include <QScrollArea>
#include <QScroller>
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/input.h"

class AutoTunerGuideDialog : public DialogBase {
public:
  explicit AutoTunerGuideDialog(const QString &html_content, QWidget *parent = nullptr) : DialogBase(parent) {
    setWindowFlags(Qt::Popup | Qt::FramelessWindowHint);
    setAttribute(Qt::WA_TranslucentBackground);
    setStyleSheet(R"(
      DialogBase { background: transparent; }
      #container { background-color: #1b1b1b; border-radius: 20px; }
      QLabel { color: #dddddd; font-size: 45px; margin: 20px; }
      QPushButton { padding: 20px; height: 100px; font-size: 45px; border-radius: 10px; color: white; background-color: #465BEA; }
      QPushButton:pressed { background-color: #3049F4; }
    )");

    QVBoxLayout *outer_layout = new QVBoxLayout(this);
    outer_layout->setContentsMargins(40, 40, 40, 40); // Maximize layout

    QFrame *container = new QFrame(this);
    container->setObjectName("container");
    QVBoxLayout *main_layout = new QVBoxLayout(container);
    main_layout->setContentsMargins(20, 20, 20, 20);

    QLabel *text = new QLabel(html_content);
    text->setWordWrap(true);
    text->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    
    QScrollArea *scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setStyleSheet("QScrollArea { background: transparent; } QWidget { background: transparent; }");
    QScroller::grabGesture(scroll->viewport(), QScroller::LeftMouseButtonGesture);
    scroll->setWidget(text);

    main_layout->addWidget(scroll, 1);

    QPushButton *btn_ok = new QPushButton(tr("OK"));
    btn_ok->setFixedWidth(400);
    main_layout->addWidget(btn_ok, 0, Qt::AlignCenter);

    outer_layout->addWidget(container);

    connect(btn_ok, &QPushButton::clicked, this, &QDialog::accept);
  }

  void showEvent(QShowEvent *event) override {
    setMainWindow(this);
    QDialog::showEvent(event);
  }
};

class AutoTunerDialog : public DialogBase {
public:
  QMap<QString, QCheckBox*> item_checkboxes;
  QJsonObject recommendations;

  explicit AutoTunerDialog(const QString &title_text, const QJsonObject &recs, QWidget *parent = nullptr) : DialogBase(parent), recommendations(recs) {
    setWindowFlags(Qt::Popup | Qt::FramelessWindowHint);
    setAttribute(Qt::WA_TranslucentBackground);
    setStyleSheet(R"(
      DialogBase { background: transparent; }
      #container { background-color: #2b2b2b; border-radius: 30px; border: 2px solid #555555; }
      QLabel { color: white; }
      QCheckBox { font-size: 45px; color: white; spacing: 20px; }
      QCheckBox::indicator { width: 50px; height: 50px; }
      QPushButton { padding: 25px; font-size: 45px; font-weight: 500; border-radius: 10px; color: white; background-color: #444444; }
      QPushButton:pressed { background-color: #333333; }
    )");

    QVBoxLayout *outer_layout = new QVBoxLayout(this);
    outer_layout->setContentsMargins(200, 40, 200, 40); // Maximize vertically, slightly wider horizontally

    QFrame *container = new QFrame(this);
    container->setObjectName("container");
    QVBoxLayout *main_layout = new QVBoxLayout(container);
    main_layout->setContentsMargins(40, 30, 40, 30);
    main_layout->setSpacing(15);

    QLabel *title = new QLabel(title_text);
    title->setStyleSheet("font-size: 55px; font-weight: bold; margin-bottom: 10px;");
    title->setAlignment(Qt::AlignCenter);
    main_layout->addWidget(title);

    QScrollArea *scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setStyleSheet("QScrollArea { background: transparent; } QWidget { background: transparent; }");
    QScroller::grabGesture(scroll->viewport(), QScroller::LeftMouseButtonGesture);
    
    QWidget *scroll_widget = new QWidget();
    QVBoxLayout *scroll_layout = new QVBoxLayout(scroll_widget);
    scroll_layout->setContentsMargins(0, 0, 0, 0);
    scroll_layout->setSpacing(15);

    for (const QString& group : recommendations.keys()) {
      QJsonObject group_items = recommendations[group].toObject();
      
      QString short_group;
      bool is_en = (QString::fromStdString(Params().get("LanguageSetting")) != "main_ko");
      if (is_en && group.contains("(")) {
        short_group = group.split("(").last().replace(")", "");
      } else {
        short_group = group.split(" ").first();
      }

      for (const QString& key : group_items.keys()) {
        QJsonObject info = group_items[key].toObject();
        QString item_text = QString("<span style='color:#aaaaaa;'>[%1]</span> <b>%2</b> <span style='font-size:40px; color:#bbbbbb;'>[%3]</span> &nbsp;:&nbsp; %4 ➔ <span style='color:#00ff00; font-weight:bold;'>%5</span>")
                                    .arg(short_group)
                                    .arg(key)
                                    .arg(info["band_kph"].toString())
                                    .arg(info["current"].toInt())
                                    .arg(info["recommended"].toInt());
        
        QCheckBox *cb = new QCheckBox();
        cb->setText(item_text);
        // Qt uses rich text in QCheckBox only if we set it this way or implicitly, but since Qt 5.11 text formats are auto-detected.
        // If rich text doesn't render in QCheckBox text, we will use a QLabel alongside a checkbox.
        // Wait, QCheckBox text does not support rich text by default natively in all styles.
        // Let's create a horizontal layout for each item: Checkbox + QLabel
        
        QHBoxLayout *item_layout = new QHBoxLayout();
        item_layout->setContentsMargins(0, 0, 0, 15);
        item_layout->setSpacing(20);
        
        QCheckBox *item_cb = new QCheckBox();
        item_cb->setChecked(true);
        item_cb->setStyleSheet("QCheckBox::indicator { width: 50px; height: 50px; }");
        
        QLabel *item_label = new QLabel(item_text);
        item_label->setStyleSheet("font-size: 45px; color: white;");
        item_label->setWordWrap(true);
        
        item_layout->addWidget(item_cb);
        item_layout->addWidget(item_label, 1);
        
        scroll_layout->addLayout(item_layout);
        item_checkboxes[key] = item_cb;
      }
    }
    scroll_layout->addStretch();
    scroll->setWidget(scroll_widget);
    main_layout->addWidget(scroll, 1);

    QHBoxLayout *btn_layout = new QHBoxLayout();
    
    QPushButton *btn_guide = new QPushButton(tr("Guide"));
    btn_guide->setStyleSheet("background-color: #3b5998;");
    
    QPushButton *btn_later = new QPushButton(tr("Later"));
    btn_later->setStyleSheet("background-color: #555555;");
    
    QPushButton *btn_clear = new QPushButton(tr("Reset"));
    btn_clear->setStyleSheet("background-color: #8a1d1d;");
    
    QPushButton *btn_apply = new QPushButton(tr("Apply Selected"));
    btn_apply->setStyleSheet("background-color: #178644;");

    btn_layout->addWidget(btn_guide);
    btn_layout->addWidget(btn_later);
    btn_layout->addWidget(btn_clear);
    btn_layout->addWidget(btn_apply);
    main_layout->addLayout(btn_layout);

    outer_layout->addWidget(container);

    connect(btn_guide, &QPushButton::clicked, this, [=]() {
      QString guide_html;
      if (QString::fromStdString(Params().get("LanguageSetting")) != "main_ko") {
        guide_html = R"(
        <div style='font-size: 45px;'>
        <div style='text-align:center; font-size: 55px; font-weight: bold; margin-bottom: 20px;'>🥕 CarrotPilot Auto-Tuner Guide</div><hr>
        <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>📊 Data Collection &amp; Application</div>
        <ul>
        <li><b>Data Collection</b>: Records driving data in the background, focusing on <b>overrides (gas pedal)</b> and <b>interventions (brake pedal)</b>.</li>
        <li><b>Pattern Analysis</b>: Analyzes the gap between current settings and driving behavior to calculate ideal parameters.</li>
        <li><b>Recommendation &amp; Apply</b>: Shows a popup when parking (P). Click <b>[Apply Selected]</b> to apply. (Manage in <b>Tuning History</b>)</li>
        </ul><hr>
        <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>⚙️ Group Parameter Details</div>
        <b>🚀 [Acceleration]</b><br>
        Adjusts cruise control acceleration capabilities.<br>
        - <b>CruiseMaxVals0~6</b>: Increases accel limits per speed band to fix sluggish starts and acceleration.<br>
        <b>🚙 [Driving]</b><br>
        Adjusts longitudinal (brake) control.<br>
        - <b>JLeadFactor3</b>: Controls how early the car starts braking for a lead vehicle. Higher values = earlier, smoother braking (range 50~200).<br>
        - <b>TFollowSpeedFactor</b>: Automatically widens the following distance as speed increases above 80km/h for high-speed cruising safety.<br>
        - <b>DynamicTFollow</b>: Adjusts gap based on lead vehicle's sudden acceleration or deceleration.<br>
        <b>🛣️ [Following Distance]</b><br>
        Optimizes highway following distance. If gas is pressed often while following a lead car at speed, it means the gap is too wide. Recommends decreasing TFollowGap for that gap level.<br>
        - <b>TFollowGap1~4</b>: Per-GAP-level time-gap setting (seconds x100). Lower = closer.<br>
        <b>🎛️ [Dynamic Control]</b><br>
        If multiple brake params trigger at once, only the strongest signal is recommended this session to avoid over-correction. Others deferred to next session.<br>
        - <b>DynamicTFollow</b>: When lead car decelerates suddenly, driver may brake to compensate. More events = system widens gap faster when lead decelerates. (0=off)<br>
        - <b>TFollowDecelBoost</b>: During strong ego deceleration, gap shrinks naturally. More events = system maintains extra buffer during braking. (default 10, range 0~100)<br>
        <b>🔄 [Steering]</b><br>
        Adjusts handling responsiveness.<br>
        - <b>PathOffset</b>: Compensates for lane drifts.<br>
        - <b>SteerActuatorDelay</b>: Reduces cornering delay.<br>
        <hr>
        <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>🧠 Autonomous Pattern Detection</div>
        Even without your pedal input, the system monitors its own control quality:<br>
        - <b>(auto) Late Braking</b>: If the system brakes too hard at the last moment, it recommends increasing JLeadFactor3 to start braking earlier.<br>
        - <b>(auto) Aggressive Accel</b>: If the system accelerates too harshly relative to the lead car, it recommends lowering CruiseMaxVals.<br>
        - <b>(auto) Hunting</b>: If the system oscillates between accel/decel while following, it recommends widening the gap for stability.<br>
        <hr>
        <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>⚖️ Accel-Brake Cross Inhibition</div>
        - Prevents the infinite inflation spiral where stronger acceleration leads to stronger braking.<br>
        - Even if you step on the gas often, <b>frequent braking will block any increase recommendations for CruiseMaxVals</b>.<br>
        - If the system detects **self-led rapid acceleration and rapid braking (Auto-Surging)** during autopilot, it automatically suggests <b>reducing</b> CruiseMaxVals with an **excessive auto-surging penalty** to restore passenger comfort.<br>
        <hr>
        <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>🧬 Driving Style Profiler (DSP)</div>
        - Before your first openpilot engage, DSP collects your <b>manual driving habits</b> (acceleration style, following distance, braking timing).<br>
        - After ~10 minutes of manual driving, DSP suggests <b>personalized initial values</b> for CruiseMaxVals, TFollowGap, and JLeadFactor3.<br>
        - Once applied, DSP completes and the Auto-Tuner takes over for continuous fine-tuning.
        </div>
        )";
      } else {
        guide_html = R"(
        <div style='font-size: 45px;'>
        <div style='text-align:center; font-size: 55px; font-weight: bold; margin-bottom: 20px;'>🥕 CarrotPilot Auto-Tuner 사용 안내</div><hr>
        <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>📊 데이터 수집 및 적용 방식</div>
        <ul>
        <li><b>주행 중 데이터 수집</b>: 백그라운드에서 차량의 주행 데이터를 실시간으로 기록하며, 특히 <b>오버라이드(가속 페달 밟음)</b>와 <b>개입(직접 브레이크)</b> 순간을 중점 수집합니다.</li>
        <li><b>패턴 분석</b>: 현재 설정값과 운전자 주행 성향의 차이를 분석하여 이상적인 파라미터를 계산합니다.</li>
        <li><b>추천 및 적용</b>: 주차(P) 시 팝업으로 추천값을 안내하며, <b>[선택 적용]</b>을 누르면 즉시 반영됩니다. (설정의 <b>CarrotPilot -> Tuning history</b>에서 이력 확인/삭제 가능)</li>
        </ul><hr>
        <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>⚙️ 그룹별 튜닝 설정 안내</div>
        <b>🚀 [가속] (Acceleration)</b><br>
        오픈파일럿의 크루즈 가속 능력을 조정합니다.<br>
        - <b>CruiseMaxVals0~6</b>: 속도 대역별 가속 한계치를 높여 굼뜬 출발과 답답한 가속을 개선합니다.<br>
        <b>🚙 [주행] (Driving)</b><br>
        종방향(브레이크) 제어 능력을 조정합니다.<br>
        - <b>JLeadFactor3</b>: 선행차에 대한 제동 시작 시점입니다. 수치가 높을수록 더 멀리서 부드럽게 감속을 시작합니다. (범위 50~200)<br>
        - <b>TFollowSpeedFactor</b>: 80km/h 이상의 고속 주행 시 자동으로 차간 거리를 추가로 확보하여 안전 마진을 늘려줍니다.<br>
        - <b>DynamicTFollow</b>: 선행차의 거친 움직임(급가감속)에 대응하여 일시적으로 거리를 조절합니다.<br>
        고속도 주행 중 선행차 추종 거리를 최적화합니다. 선행차가 있는데도 가속 페달을 자주 밟는다면, 시스템이 지나치게 거리를 넓게 유지하는 것으로 판단하여 해당 GAP의 TFollowGap 값을 줄이는 방향으로 추천합니다.<br>
        - <b>TFollowGap1~4</b>: GAP 단계별 추종 거리 시간(x0.01초). 낙을수록 가깄워집니다. 최소 0.70초 보장.<br>
        <b>🎛️ [동적제어] (Dynamic Control)</b><br>
        브레이크 관련 파라미터가 동시 여러 개 발동되면 합산 과보정 위험이 있어, 이벤트가 가장 많은 1개만 먼저 추천합니다. 나머지는 다음 세션 재평가.<br>
        - <b>DynamicTFollow</b>: 앞차가 급감속할 때 브레이크 개입이 쌓이면, 시스템이 앞차 감속에 더 빠르게 반응하도록 조정합니다. (0=사용 안 함)<br>
        - <b>TFollowDecelBoost</b>: 내 차 강감속 중 브레이크 개입이 쌓이면, 감속이 강할수록 자동으로 더 넉넉한 간격을 유지하도록 보완합니다. (기본값 10, 범위 0~100)<br>
        <b>🔄 [조향] (Steering)</b><br>
        핸들링 반응성을 조정합니다.<br>
        - <b>PathOffset / SteerActuatorDelay</b>: 조향 편차 및 지연 보정.<br>
        <hr>
        <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>🧠 자율 주행 패턴 자동 감지</div>
        운전자가 페달을 밟지 않아도, 시스템은 스스로의 제어 품질을 감시합니다:<br>
        - <b>(auto) 늦은 제동</b>: 시스템이 뒤늦게 급제동을 거는 패턴이 감지되면, 더 일찍 부드럽게 감속하도록 제동 시점 상향을 추천합니다.<br>
        - <b>(auto) 과도한 가속</b>: 선행차 흐름에 비해 시스템이 너무 거칠게 가속하면, 가속 한계치를 낮추도록 추천합니다.<br>
        - <b>(auto) 주행 요동(Hunting)</b>: 가속과 감속을 반복하며 거리를 불안정하게 맞추는 패턴이 감지되면, 제어의 여유를 위해 차간 거리를 넓히도록 추천합니다.<br>
        <hr>
        <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>⚖️ 가속-제동 상호 억제 (Inflation Lock)</div>
        - 가속력을 높이면 뒤이어 급제동이 유발되는 '악순환(인플레이션)'을 방지합니다.<br>
        - 가속 페달을 많이 밟았더라도 <b>브레이크 개입이 잦으면 가속도 상승 제안을 차단</b>합니다.<br>
        - 자율 주행 중 스스로 <b>급가속과 급제동을 반복하는 요동 현상(Auto-Surging)</b>이 감지되면, 운전자 개입이 전혀 없었더라도 최우선적으로 가속 설정을 낮추도록 <b>감소(excessive auto-surging penalty)</b>를 추천하여 부드러운 주행을 강제합니다.<br>
        <hr>
        <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>🧬 수동 주행 성향 프로파일러 (DSP)</div>
        - 오픈파일럿 첫 인게이지 전, 수동 운전 중의 <b>가속 스타일, 차간 거리 습관, 제동 시점</b>을 자동으로 수집합니다.<br>
        - 약 10분 이상 수동 주행 후, CruiseMaxVals, TFollowGap, JLeadFactor3의 <b>개인화된 초기값</b>을 제안합니다.<br>
        - 적용 후 DSP는 완료되고, 이후부터는 Auto-Tuner가 자율주행 데이터를 기반으로 지속적으로 미세 조정합니다.
        </div>
        )";
      }
      AutoTunerGuideDialog *d = new AutoTunerGuideDialog(guide_html, this);
      d->exec();
      d->deleteLater();
    });

    connect(btn_later, &QPushButton::clicked, this, &QDialog::reject);
    
    connect(btn_clear, &QPushButton::clicked, [=]() {
      if (ConfirmationDialog::confirm(tr("Are you sure you want to delete all learning data collected so far without applying it?"), tr("Reset"), this)) {
        Params().putBool("CarrotLearningClear", true);
        this->reject();
      }
    });

    connect(btn_apply, &QPushButton::clicked, this, &QDialog::accept);
  }

  QJsonObject getSelectedItems() {
    QJsonObject selected;
    for (const QString& group : recommendations.keys()) {
      QJsonObject group_items = recommendations[group].toObject();
      QJsonObject selected_group_items;
      
      for (const QString& key : group_items.keys()) {
        if (item_checkboxes.contains(key) && item_checkboxes[key]->isChecked()) {
          selected_group_items[key] = group_items[key];
        }
      }
      
      if (!selected_group_items.isEmpty()) {
        selected[group] = selected_group_items;
      }
    }
    return selected;
  }
};

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome(this);
  QObject::connect(home, &OffroadHome::openSettings, this, &HomeWindow::openSettings);
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  QObject::connect(onroad, &OnroadWindow::mapPanelRequested, this, [=] { sidebar->hide(); });
  slayout->addWidget(onroad);

  body = new BodyWindow(this);
  slayout->addWidget(body);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
  setAttribute(Qt::WA_NoSystemBackground);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &HomeWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &HomeWindow::offroadTransition);
  QObject::connect(uiState(), &UIState::offroadTransition, sidebar, &Sidebar::offroadTransition);
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::showMapPanel(bool show) {
  onroad->showMapPanel(show);
}

void HomeWindow::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  // switch to the generic robot UI
  if (onroad->isVisible() && !body->isEnabled() && sm["carParams"].getCarParams().getNotCar()) {
    body->setEnabled(true);
    slayout->setCurrentWidget(body);
  }
  switch (s.scene._current_carrot_display) {
  case 1: // default
      sidebar->setVisible(true);
      break;
  case 2: // road
      sidebar->setVisible(false);
      break;
  case 3: // map
      sidebar->setVisible(false);
      break;
  case 4: // fullmap
      sidebar->setVisible(false);
      break;
  }

  // Auto-Tuner + DSP: 주차(P단) 전환 시 팝업 표시 (1초 주기로 체크)
  // 우선순위: DSP(초기값 설정) > Auto-Tuner(미세 조정)
  // 두 팝업이 동시에 Ready이면 DSP 팝업을 먼저 표시하고,
  // Auto-Tuner는 다음 사이클(~1초 후)에 자동으로 표시됩니다.
  static int carrot_tuner_frame = 0;
  if (carrot_tuner_frame++ % 20 == 0) {
    Params params;

    // ── [우선순위 1] DSP: 수동 주행 프로파일링 초기값 추천 ──────────────
    if (params.getBool("CarrotDSPPopupReady")) {
      params.putBool("CarrotDSPPopupReady", false);

      QString dsp_raw = QString::fromStdString(params.get("CarrotDSPRecommend"));
      QJsonDocument dsp_doc = QJsonDocument::fromJson(dsp_raw.toUtf8());
      if (!dsp_raw.isEmpty() && dsp_doc.isObject()) {
        QJsonObject dsp_obj = dsp_doc.object();
        QString dsp_msg = tr("🧬 DSP: Manual driving style analyzed! (Auto-Tuner will follow next)");

        AutoTunerDialog *dsp_dialog = new AutoTunerDialog(dsp_msg, dsp_obj, this);
        connect(dsp_dialog, &QDialog::accepted, [=]() {
          Params p;
          QJsonObject selected = dsp_dialog->getSelectedItems();
          if (!selected.isEmpty()) {
            for (const QString& group : selected.keys()) {
              QJsonObject group_items = selected[group].toObject();
              for (const QString& key : group_items.keys()) {
                QJsonObject info = group_items[key].toObject();
                int recommended = info["recommended"].toInt(0);
                if (recommended > 0) {
                  p.put(key.toStdString(), std::to_string(recommended));
                }
              }
            }
          }
          // 프로파일링 완료 마킹 → DSP 비활성화
          p.putBool("CarrotDSPComplete", true);
          p.remove("CarrotDSPData");
          p.remove("CarrotDSPRecommend");
          dsp_dialog->deleteLater();
          // Auto-Tuner 팝업이 대기 중이면 다음 사이클에서 자동 표시됨
        });

        connect(dsp_dialog, &QDialog::rejected, [=]() {
          dsp_dialog->deleteLater();
        });

        setMainWindow(dsp_dialog);
      }  // if (!dsp_raw.isEmpty() && dsp_doc.isObject())

    // ── [우선순위 2] Auto-Tuner: 자율주행 패턴 미세 조정 추천 ───────────
    } else if (params.getBool("CarrotLearningPopupReady")) {
      params.putBool("CarrotLearningPopupReady", false);

      QString raw = QString::fromStdString(params.get("CarrotLearningRecommend"));
      QJsonDocument doc = QJsonDocument::fromJson(raw.toUtf8());
      if (!raw.isEmpty() && doc.isObject()) {
        QJsonObject obj = doc.object();
        QString msg = tr("Auto-Tuner: Driving pattern learned!");

        AutoTunerDialog *dialog = new AutoTunerDialog(msg, obj, this);
        connect(dialog, &QDialog::accepted, [=]() {
          Params p;
          QJsonObject selected = dialog->getSelectedItems();
          if (!selected.isEmpty()) {
            QJsonArray history_array;
            QString history_raw = QString::fromStdString(p.get("CarrotLearningHistory"));
            if (!history_raw.isEmpty()) {
              QJsonDocument h_doc = QJsonDocument::fromJson(history_raw.toUtf8());
              if (h_doc.isArray()) history_array = h_doc.array();
            }

            QJsonObject history_entry;
            history_entry["timestamp"] = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm");
            history_entry["changes"] = selected;
            history_entry["id"] = QString::number(QDateTime::currentMSecsSinceEpoch());

            history_array.prepend(history_entry);
            while (history_array.size() > 50) history_array.removeLast();

            p.put("CarrotLearningHistory", QJsonDocument(history_array).toJson(QJsonDocument::Compact).toStdString());

            for (const QString& group : selected.keys()) {
              QJsonObject group_items = selected[group].toObject();
              for (const QString& key : group_items.keys()) {
                QJsonObject info = group_items[key].toObject();
                int recommended = info["recommended"].toInt(0);
                if (recommended > 0) {
                  p.put(key.toStdString(), std::to_string(recommended));
                }
              }
            }
          }
          Params().putBool("CarrotLearningClear", true);
          dialog->deleteLater();
        });

        connect(dialog, &QDialog::rejected, [=]() {
          dialog->deleteLater();
        });

        setMainWindow(dialog);
      }
    }
  }

}

void HomeWindow::offroadTransition(bool offroad) {
  body->setEnabled(false);
  sidebar->setVisible(offroad);
  UIState* s = uiState();
  if (offroad) {

    s->scene._current_carrot_display = 1;
    slayout->setCurrentWidget(home);

  } else {
    slayout->setCurrentWidget(onroad);

    s->show_brightness_timer = (int)(10. / 0.05);
  }
}

void HomeWindow::showDriverView(bool show) {
  if (show) {
    emit closeSettings();
    slayout->setCurrentWidget(driver_view);
  } else {
    slayout->setCurrentWidget(home);
  }
  sidebar->setVisible(show == false);
}

void HomeWindow::mousePressEvent(QMouseEvent* e) {
  // Handle sidebar collapsing
  //if ((onroad->isVisible() || body->isVisible()) && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    //sidebar->setVisible(!sidebar->isVisible() && !onroad->isMapVisible());
  //}

  UIState* s = uiState();
  s->show_brightness_timer = 100;
}

void HomeWindow::mouseDoubleClickEvent(QMouseEvent* e) {
  HomeWindow::mousePressEvent(e);
  const SubMaster &sm = *(uiState()->sm);
  if (sm["carParams"].getCarParams().getNotCar()) {
    if (onroad->isVisible()) {
      slayout->setCurrentWidget(body);
    } else if (body->isVisible()) {
      slayout->setCurrentWidget(onroad);
    }
    showSidebar(false);
  }
}

// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 40, 40, 40);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(0, 0, 0, 0);
  header_layout->setSpacing(16);

  update_notif = new QPushButton(tr("UPDATE"));
  update_notif->setVisible(false);
  update_notif->setStyleSheet("background-color: #364DEF;");
  QObject::connect(update_notif, &QPushButton::clicked, [=]() { center_layout->setCurrentIndex(1); });
  header_layout->addWidget(update_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  alert_notif = new QPushButton();
  alert_notif->setVisible(false);
  alert_notif->setStyleSheet("background-color: #E22C2C;");
  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
  header_layout->addWidget(alert_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  version = new ElidedLabel();
  header_layout->addWidget(version, 0, Qt::AlignHCenter | Qt::AlignRight);

  main_layout->addLayout(header_layout);

  // main content
  main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  QWidget *home_widget = new QWidget(this);
  {
    QHBoxLayout *home_layout = new QHBoxLayout(home_widget);
    home_layout->setContentsMargins(0, 0, 0, 0);
    home_layout->setSpacing(30);

    // left: PrimeAdWidget
    QStackedWidget *left_widget = new QStackedWidget(this);
    QVBoxLayout *left_prime_layout = new QVBoxLayout();
    left_prime_layout->setContentsMargins(0, 0, 0, 0);
    QWidget *prime_user = new PrimeUserWidget();
    prime_user->setStyleSheet(R"(
    border-radius: 10px;
    background-color: #333333;
    )");
    left_prime_layout->addWidget(prime_user);
    left_prime_layout->addStretch();
    left_widget->addWidget(new LayoutWidget(left_prime_layout));
    left_widget->addWidget(new PrimeAdWidget);
    left_widget->setStyleSheet("border-radius: 10px;");

    connect(uiState()->prime_state, &PrimeState::changed, [left_widget]() {
      left_widget->setCurrentIndex(uiState()->prime_state->isSubscribed() ? 0 : 1);
    });

    home_layout->addWidget(left_widget, 1);

    // right: ExperimentalModeButton, SetupWidget
    QWidget* right_widget = new QWidget(this);
    QVBoxLayout* right_column = new QVBoxLayout(right_widget);
    right_column->setContentsMargins(0, 0, 0, 0);
    right_widget->setFixedWidth(750);
    right_column->setSpacing(30);

    ExperimentalModeButton *experimental_mode = new ExperimentalModeButton(this);
    QObject::connect(experimental_mode, &ExperimentalModeButton::openSettings, this, &OffroadHome::openSettings);
    right_column->addWidget(experimental_mode, 1);

    SetupWidget *setup_widget = new SetupWidget;
    QObject::connect(setup_widget, &SetupWidget::openSettings, this, &OffroadHome::openSettings);
    right_column->addWidget(setup_widget, 1);

    home_layout->addWidget(right_widget, 1);
  }
  center_layout->addWidget(home_widget);

  // add update & alerts widgets
  update_widget = new UpdateAlert();
  QObject::connect(update_widget, &UpdateAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(update_widget);
  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(alerts_widget);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);

  setStyleSheet(R"(
    * {
      color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QPushButton {
      padding: 15px 30px;
      border-radius: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    OffroadHome > QLabel {
      font-size: 55px;
    }
  )");
}

void OffroadHome::showEvent(QShowEvent *event) {
  refresh();
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

void OffroadHome::refresh() {
  version->setText(getBrand() + " " +  QString::fromStdString(params.get("UpdaterCurrentDescription")));

  bool updateAvailable = update_widget->refresh();
  int alerts = alerts_widget->refresh();

  // pop-up new notification
  int idx = center_layout->currentIndex();
  if (!updateAvailable && !alerts) {
    idx = 0;
  } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
    idx = 1;
  } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
    idx = 2;
  }
  center_layout->setCurrentIndex(idx);

  update_notif->setVisible(updateAvailable);
  alert_notif->setVisible(alerts);
  if (alerts) {
    alert_notif->setText(QString::number(alerts) + (alerts > 1 ? tr(" ALERTS") : tr(" ALERT")));
  }
}
