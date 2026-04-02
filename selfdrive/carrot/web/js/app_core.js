const DEBUG_UI = false;

function disableViewportZoomGestures() {
  const preventGesture = (e) => e.preventDefault();

  ["gesturestart", "gesturechange", "gestureend"].forEach((type) => {
    document.addEventListener(type, preventGesture, { passive: false });
  });

  document.addEventListener("touchmove", (e) => {
    if (e.touches && e.touches.length > 1) e.preventDefault();
  }, { passive: false });

  let lastTouchEnd = 0;
  document.addEventListener("touchend", (e) => {
    const now = Date.now();
    if (now - lastTouchEnd <= 300) e.preventDefault();
    lastTouchEnd = now;
  }, { passive: false });
}

disableViewportZoomGestures();

let SETTINGS = null;
let CURRENT_GROUP = null;
let LANG = "ko"; // "ko" | "en" | "zh"
const LANG_STORAGE_KEY = "carrot_web_lang";
const LANG_EMOJI = {
  ko: "🇰🇷",
  en: "🇺🇸",
  zh: "🇨🇳",
};

const UI_STRINGS = {
  ko: {
    home: "홈",
    setting: "설정",
    tools: "도구",
    terminal: "터미널",
    fleet: "Fleet",
    carrot: "당근",
    lang: "언어",
    branch_select: "브랜치 선택",
    branch_current: "현재",
    server_state: "서버 상태",
    quick_link: "퀵 링크",
    car_select: "차량 선택",
    makers: "제조사",
    models: "모델",
    groups: "그룹",
    items: "항목",
    back: "뒤로",
    change: "변경",
    git_commands: "Git Commands",
    user_system: "User / System",
    reboot: "Reboot",
    backup: "Backup",
    restore: "Restore",
    apply: "적용",
    confirm_car: "이 차량을 선택하시겠습니까?",
    confirm_reboot: "지금 재부팅하시겠습니까?",
    confirm_reboot_after_install: "설치가 완료되었습니다.\n변경 사항을 적용하려면 재부팅이 필요합니다.\n지금 재부팅하시겠습니까?",
    reboot_later: "선택되었습니다. 적용하려면 나중에 재부팅하세요.",
    rebooting: "재부팅 중...",
    git_sync_confirm: "브랜치를 동기화합니다.\n로컬 브랜치가 정리됩니다. 계속하시겠습니까?",
    git_reset_confirm: "코드 변경사항을 되돌립니다.\n수정한 내용이 사라질 수 있습니다. 계속하시겠습니까?",
    git_reset_mode_prompt: "리셋 방식을 선택하세요\n\n• hard: 모든 변경 삭제\n• soft: 커밋만 취소\n• mixed: 스테이지만 취소",
    git_reset_target_prompt: "리셋 대상을 입력하세요\n예: HEAD (현재), origin/master (원본)",
    delete_videos_confirm: "모든 주행 영상을 삭제합니다.\n삭제 후 복구할 수 없습니다. 계속하시겠습니까?",
    delete_logs_confirm: "모든 로그 파일을 삭제합니다.\n삭제 후 복구할 수 없습니다. 계속하시겠습니까?",
    rebuild_confirm: "전체 재빌드를 실행합니다.\n빌드 파일 삭제 후 자동으로 재부팅됩니다.\n수 분이 소요될 수 있습니다. 계속하시겠습니까?",
    select_backup_file: "먼저 백업 json 파일을 선택하세요.",
    restore_confirm: "파일에서 설정을 복구하시겠습니까?\n\n많은 Params 값이 덮어씌워집니다.",
    restore_done_reboot: "복구가 완료되었습니다.\n지금 재부팅하시겠습니까?",
    checkout_confirm: "브랜치를 변경하시겠습니까?",
    branch_changed: "브랜치가 변경되었습니다.",
    fleet_open_confirm: "Fleet를 여시겠습니까?",
    quick_link_hint: "길게 눌러 링크 저장",
    failed_set_car: "차량 선택 저장 실패: ",
    reboot_failed: "재부팅 실패: ",
    set_failed: "설정 실패: ",
    branch_dom_missing: "브랜치 DOM 요소를 찾을 수 없습니다.",
    fullscreen_not_supported: "이 브라우저는 전체화면을 지원하지 않습니다.",
    record: "녹화",
    record_on: "녹화중",
    record_off: "녹화대기",
    ready: "준비됨",
    open: "열기",
    close: "접기",
    save: "저장",
    copied: "복사됨",
    not_set: "미설정",
    connecting: "연결중...",
    connected: "연결됨",
    reconnecting: "재연결중...",
    error: "오류",
    notice: "알림",
    confirm_title: "확인",
    input_title: "입력",
    ok: "확인",
    cancel: "취소",
    quick_link_empty: "GithubUsername 없음",
    // Tools section titles (keep English)
    section_settings_backup: "Settings",
    section_sys_cmd: "System Command",
    section_output: "Output",
    sys_cmd_help: "Allowed: pull, status, branch, log, git ..., df, free, uptime",
    terminal_session: "tmux carrot-web",
    terminal_placeholder: "git status",
    terminal_send: "전송",
    terminal_reconnect: "재연결",
    terminal_ctrl_c: "Ctrl+C",
    terminal_clear: "Clear",
    terminal_ready: "tmux 준비됨",
    terminal_disconnected: "연결끊김",
    terminal_unavailable: "터미널 접속 실패",
    terminal_offline: "터미널 오프라인",
    setting_search: "설정 검색",
    setting_search_placeholder: "이름, 설명, 그룹 검색",
    setting_search_empty: "검색 결과가 없습니다.",
    setting_search_idle: "검색어를 입력하면 세부 설정을 찾을 수 있습니다.",
    setting_search_results: "검색 결과",
  },
  en: {
    home: "Home",
    setting: "Setting",
    tools: "Tools",
    terminal: "Terminal",
    fleet: "Fleet",
    carrot: "Carrot",
    lang: "Lang",
    branch_select: "Branch Select",
    branch_current: "Current",
    server_state: "Server Status",
    quick_link: "Quick Link",
    car_select: "Car Select",
    makers: "Makers",
    models: "Models",
    groups: "Groups",
    items: "Items",
    back: "Back",
    change: "Change",
    git_commands: "Software Update",
    user_system: "System Management",
    reboot: "Reboot",
    backup: "Backup Settings",
    restore: "Restore Settings",
    apply: "Apply",
    confirm_car: "Select this car?",
    confirm_reboot: "Reboot now?",
    confirm_reboot_after_install: "Installation is complete.\nA reboot is required to apply the changes.\nReboot now?",
    reboot_later: "Selected. Reboot later to apply.",
    rebooting: "Rebooting...",
    git_sync_confirm: "This will sync branches.\nLocal branches will be cleaned up. Continue?",
    git_reset_confirm: "This will revert code changes.\nYour modifications may be lost. Continue?",
    git_reset_mode_prompt: "Select reset mode\n\n• hard: discard all changes\n• soft: undo commit only\n• mixed: unstage only",
    git_reset_target_prompt: "Enter reset target\ne.g. HEAD (current), origin/master (remote)",
    delete_videos_confirm: "Delete ALL driving videos?\nThis cannot be undone. Continue?",
    delete_logs_confirm: "Delete ALL log files?\nThis cannot be undone. Continue?",
    rebuild_confirm: "Run full rebuild?\nBuild files will be deleted and the device will reboot.\nThis may take several minutes. Continue?",
    select_backup_file: "Select a backup json file first.",
    restore_confirm: "Restore settings from file?\n\nThis will overwrite many Params values.",
    restore_done_reboot: "Restore done.\nReboot now?",
    checkout_confirm: "Switch to this branch?",
    branch_changed: "Branch changed.",
    fleet_open_confirm: "Open Fleet?",
    quick_link_hint: "Long press to save link",
    failed_set_car: "Failed to set car: ",
    reboot_failed: "Reboot failed: ",
    set_failed: "Set failed: ",
    branch_dom_missing: "Branch DOM elements missing.",
    fullscreen_not_supported: "Fullscreen not supported on this browser.",
    record: "Record",
    record_on: "Recording",
    record_off: "Idle",
    ready: "Ready",
    open: "Open",
    close: "Close",
    save: "Save",
    copied: "Copied",
    not_set: "Not set",
    connecting: "Connecting...",
    connected: "Connected",
    reconnecting: "Reconnecting...",
    error: "Error",
    notice: "Notice",
    confirm_title: "Confirm",
    input_title: "Input",
    ok: "OK",
    cancel: "Cancel",
    quick_link_empty: "GithubUsername not set",
    section_settings_backup: "Settings",
    section_sys_cmd: "System Command",
    section_output: "Output",
    sys_cmd_help: "Allowed: pull, status, branch, log, git ..., df, free, uptime",
    terminal_session: "tmux carrot-web",
    terminal_placeholder: "git status",
    terminal_send: "Send",
    terminal_reconnect: "Reconnect",
    terminal_ctrl_c: "Ctrl+C",
    terminal_clear: "Clear",
    terminal_ready: "tmux ready",
    terminal_disconnected: "disconnected",
    terminal_unavailable: "terminal unavailable",
    terminal_offline: "terminal offline",
    setting_search: "Search Settings",
    setting_search_placeholder: "Search name, description, group",
    setting_search_empty: "No matching settings found.",
    setting_search_idle: "Type to find detailed settings.",
    setting_search_results: "results",
  },
  zh: {
    home: "首页",
    setting: "设置",
    tools: "工具",
    terminal: "终端",
    fleet: "车队",
    carrot: "胡萝卜",
    lang: "语言",
    branch_select: "分支选择",
    branch_current: "当前",
    server_state: "服务器状态",
    quick_link: "快速链接",
    car_select: "车辆选择",
    makers: "制造商",
    models: "车型",
    groups: "分组",
    items: "项",
    back: "返回",
    change: "修改",
    git_commands: "Git Commands",
    user_system: "User / System",
    reboot: "Reboot",
    backup: "Backup",
    restore: "Restore",
    apply: "应用",
    confirm_car: "选择此车辆吗？",
    confirm_reboot: "现在重启吗？",
    confirm_reboot_after_install: "安装已完成。\n需要重新启动设备以应用更改。\n现在重新启动吗？",
    reboot_later: "已选择。请稍后重启以应用更改。",
    rebooting: "正在重启...",
    git_sync_confirm: "将同步分支。\n本地分支将被清理。继续吗？",
    git_reset_confirm: "将恢复代码更改。\n您的修改可能会丢失。继续吗？",
    git_reset_mode_prompt: "选择重置模式\n\n• hard: 删除所有更改\n• soft: 仅撤消提交\n• mixed: 仅取消暂存",
    git_reset_target_prompt: "输入重置目标\n例如: HEAD (当前), origin/master (远程)",
    delete_videos_confirm: "删除所有行车视频？\n删除后无法恢复。继续吗？",
    delete_logs_confirm: "删除所有日志文件？\n删除后无法恢复。继续吗？",
    rebuild_confirm: "执行全部重建？\n构建文件将被删除并自动重启。\n可能需要几分钟。继续吗？",
    select_backup_file: "请先选择一个备份 JSON 文件。",
    restore_confirm: "从文件还原设置吗？\n\n这将覆盖许多参数值。",
    restore_done_reboot: "还原完成。\n现在重启吗？",
    checkout_confirm: "切换到此分支吗？",
    branch_changed: "分支已切换。",
    fleet_open_confirm: "要打开 Fleet 吗？",
    quick_link_hint: "长按保存链接",
    failed_set_car: "保存车辆选择失败: ",
    reboot_failed: "重启失败: ",
    set_failed: "设置失败: ",
    branch_dom_missing: "找不到分支 DOM 元素。",
    fullscreen_not_supported: "此浏览器不支持全屏。",
    record: "录制",
    record_on: "录制中",
    record_off: "待机",
    ready: "就绪",
    open: "打开",
    close: "收起",
    save: "保存",
    copied: "已复制",
    not_set: "未设置",
    connecting: "连接中...",
    connected: "已连接",
    reconnecting: "重连中...",
    error: "错误",
    notice: "提示",
    confirm_title: "确认",
    input_title: "输入",
    ok: "确定",
    cancel: "取消",
    quick_link_empty: "GithubUsername 未设置",
    section_settings_backup: "Settings",
    section_sys_cmd: "System Command",
    section_output: "Output",
    sys_cmd_help: "Allowed: pull, status, branch, log, git ..., df, free, uptime",
    terminal_session: "tmux carrot-web",
    terminal_placeholder: "git status",
    terminal_send: "发送",
    terminal_reconnect: "重连",
    terminal_ctrl_c: "Ctrl+C",
    terminal_clear: "Clear",
    terminal_ready: "tmux 已就绪",
    terminal_disconnected: "连接已断开",
    terminal_unavailable: "终端不可用",
    terminal_offline: "终端离线",
    setting_search: "设置搜索",
    setting_search_placeholder: "搜索名称、描述、分组",
    setting_search_empty: "没有匹配的设置项。",
    setting_search_idle: "输入关键词以查找详细设置。",
    setting_search_results: "项结果",
  }
};

/* ── Action Labels (user-friendly status messages) ──────── */
const ACTION_LABELS = {
  ko: {
    git_pull:         { running: "업데이트 확인 중...",       done: "업데이트 완료",      failed: "업데이트 실패" },
    git_sync:         { running: "브랜치 동기화 중...",       done: "동기화 완료",        failed: "동기화 실패" },
    git_reset:        { running: "되돌리는 중...",           done: "되돌리기 완료",      failed: "되돌리기 실패" },
    git_checkout:     { running: "브랜치 전환 중...",         done: "브랜치 변경됨",      failed: "브랜치 전환 실패" },
    git_branch_list:  { running: "브랜치 목록 조회 중...",    done: "브랜치 목록 로드됨", failed: "목록 조회 실패" },
    reboot:           { running: "재부팅 요청 중...",         done: "재부팅 시작됨",      failed: "재부팅 실패" },
    send_tmux_log:    { running: "로그 다운로드 중...",       done: "다운로드 완료",      failed: "다운로드 실패" },
    server_tmux_log:  { running: "서버 로그 전송 중...",      done: "전송 완료",          failed: "전송 실패" },
    backup_settings:  { running: "설정 백업 중...",           done: "백업 완료",          failed: "백업 실패" },
    delete_all_videos:{ running: "영상 삭제 중...",           done: "삭제 완료",          failed: "삭제 실패" },
    delete_all_logs:  { running: "로그 삭제 중...",           done: "삭제 완료",          failed: "삭제 실패" },
    rebuild_all:      { running: "전체 재빌드 중...",         done: "재빌드+재부팅 시작", failed: "재빌드 실패" },
    shell_cmd:        { running: "명령 실행 중...",           done: "실행 완료",          failed: "실행 실패" },
    install_required: { running: "패키지 설치 중...",         done: "설치 완료",          failed: "설치 실패" },
  },
  en: {
    git_pull:         { running: "Checking for updates...",  done: "Update complete",     failed: "Update failed" },
    git_sync:         { running: "Syncing branches...",      done: "Sync complete",       failed: "Sync failed" },
    git_reset:        { running: "Resetting...",             done: "Reset complete",      failed: "Reset failed" },
    git_checkout:     { running: "Switching branch...",      done: "Branch changed",      failed: "Branch switch failed" },
    git_branch_list:  { running: "Loading branches...",      done: "Branches loaded",     failed: "Failed to load" },
    reboot:           { running: "Requesting reboot...",     done: "Reboot started",      failed: "Reboot failed" },
    send_tmux_log:    { running: "Downloading log...",       done: "Download complete",   failed: "Download failed" },
    server_tmux_log:  { running: "Sending server log...",    done: "Sent",                failed: "Send failed" },
    backup_settings:  { running: "Backing up settings...",   done: "Backup complete",     failed: "Backup failed" },
    delete_all_videos:{ running: "Deleting videos...",       done: "Deleted",             failed: "Delete failed" },
    delete_all_logs:  { running: "Deleting logs...",         done: "Deleted",             failed: "Delete failed" },
    rebuild_all:      { running: "Rebuilding all...",        done: "Rebuild+reboot started", failed: "Rebuild failed" },
    shell_cmd:        { running: "Running command...",       done: "Complete",            failed: "Command failed" },
    install_required: { running: "Installing packages...",   done: "Installed",           failed: "Install failed" },
  },
  zh: {
    git_pull:         { running: "检查更新中...",             done: "更新完成",            failed: "更新失败" },
    git_sync:         { running: "同步分支中...",             done: "同步完成",            failed: "同步失败" },
    git_reset:        { running: "重置中...",                done: "重置完成",            failed: "重置失败" },
    git_checkout:     { running: "切换分支中...",             done: "分支已切换",          failed: "切换失败" },
    git_branch_list:  { running: "加载分支列表...",           done: "分支列表已加载",      failed: "加载失败" },
    reboot:           { running: "请求重启中...",             done: "重启已开始",          failed: "重启失败" },
    send_tmux_log:    { running: "下载日志中...",             done: "下载完成",            failed: "下载失败" },
    server_tmux_log:  { running: "发送服务器日志中...",       done: "发送完成",            failed: "发送失败" },
    backup_settings:  { running: "备份设置中...",             done: "备份完成",            failed: "备份失败" },
    delete_all_videos:{ running: "删除视频中...",             done: "删除完成",            failed: "删除失败" },
    delete_all_logs:  { running: "删除日志中...",             done: "删除完成",            failed: "删除失败" },
    rebuild_all:      { running: "全部重建中...",             done: "重建+重启已开始",     failed: "重建失败" },
    shell_cmd:        { running: "运行命令中...",             done: "运行完成",            failed: "命令失败" },
    install_required: { running: "安装包中...",               done: "安装完成",            failed: "安装失败" },
  }
};

/* ── Error Code → Friendly Message ────────────────────────── */
const ERROR_MESSAGES = {
  ko: {
    GIT_CMD_NOT_ALLOWED: (d) => `허용되지 않는 git 명령입니다: ${d}`,
    CMD_NOT_ALLOWED:     (d) => `허용되지 않는 명령입니다: ${d}`,
    INVALID_RESET_MODE:  () => "잘못된 리셋 모드입니다",
    MISSING_BRANCH:      () => "브랜치를 선택해주세요",
    CMD_TIMEOUT:         () => "명령 실행 시간이 초과되었습니다",
    TMUX_CAPTURE_FAIL:   () => "로그 캡처에 실패했습니다",
  },
  en: {
    GIT_CMD_NOT_ALLOWED: (d) => `This git command is not allowed: ${d}`,
    CMD_NOT_ALLOWED:     (d) => `This command is not allowed: ${d}`,
    INVALID_RESET_MODE:  () => "Invalid reset mode",
    MISSING_BRANCH:      () => "Please select a branch",
    CMD_TIMEOUT:         () => "Command timed out",
    TMUX_CAPTURE_FAIL:   () => "Failed to capture log",
  },
  zh: {
    GIT_CMD_NOT_ALLOWED: (d) => `不允许的git命令: ${d}`,
    CMD_NOT_ALLOWED:     (d) => `不允许的命令: ${d}`,
    INVALID_RESET_MODE:  () => "无效的重置模式",
    MISSING_BRANCH:      () => "请选择分支",
    CMD_TIMEOUT:         () => "命令执行超时",
    TMUX_CAPTURE_FAIL:   () => "日志捕获失败",
  }
};

function friendlyError(json) {
  if (!json) return null;
  const code = json.error_code;
  const detail = json.error_detail || "";
  const langMap = ERROR_MESSAGES[LANG] || ERROR_MESSAGES.en;
  if (code && langMap[code]) return langMap[code](detail);
  // fallback: try to make raw errors more readable
  const raw = json.error || "";
  if (raw.startsWith("git subcommand not allowed:")) {
    const sub = raw.replace("git subcommand not allowed:", "").trim();
    return (ERROR_MESSAGES[LANG] || ERROR_MESSAGES.en).GIT_CMD_NOT_ALLOWED(sub);
  }
  if (raw.startsWith("not allowed:")) {
    const cmd = raw.replace("not allowed:", "").trim();
    return (ERROR_MESSAGES[LANG] || ERROR_MESSAGES.en).CMD_NOT_ALLOWED(cmd);
  }
  if (raw === "timeout") return (ERROR_MESSAGES[LANG] || ERROR_MESSAGES.en).CMD_TIMEOUT();
  if (raw === "bad mode") return (ERROR_MESSAGES[LANG] || ERROR_MESSAGES.en).INVALID_RESET_MODE();
  if (raw === "missing branch") return (ERROR_MESSAGES[LANG] || ERROR_MESSAGES.en).MISSING_BRANCH();
  return raw || null;
}

function getActionLabel(action) {
  const labels = (ACTION_LABELS[LANG] || ACTION_LABELS.en)[action];
  return labels || { running: action + "...", done: action, failed: action };
}

const DRIVE_MODES = {
  ko: { normal: "일반", eco: "연비", safe: "안전", sport: "고속" },
  en: { normal: "Normal", eco: "Eco", safe: "Safe", sport: "Sport" },
  zh: { normal: "标准", eco: "经济", safe: "安全", sport: "运动" }
};

let UNIT_CYCLE = [1, 2, 5, 10, 50, 100];
const UNIT_INDEX = {}; // per name

// Car select data
let CARS = null;                 // { makers: {Hyundai:[...], Genesis:[...]} }
let CURRENT_MAKER = null;

const btnHome = document.getElementById("btnHome");
const btnSetting = document.getElementById("btnSetting");
const btnTerminal = document.getElementById("btnTerminal");
const btnFleet = document.getElementById("btnFleet");
const btnLang = document.getElementById("btnLang");
const langLabel = document.getElementById("langLabel");
const btnTools = document.getElementById("btnTools");
const btnRecordToggle = document.getElementById("btnRecordToggle");
const btnSettingSearch = document.getElementById("btnSettingSearch");
const settingSearchBackdrop = document.getElementById("settingSearchBackdrop");
const settingSearchPanel = document.getElementById("settingSearchPanel");
const settingSearchTitle = document.getElementById("settingSearchTitle");
const settingSearchForm = document.getElementById("settingSearchForm");
const settingSearchInput = document.getElementById("settingSearchInput");
const btnSettingSearchSubmit = document.getElementById("btnSettingSearchSubmit");
const settingSearchMeta = document.getElementById("settingSearchMeta");
const settingSearchResults = document.getElementById("settingSearchResults");
const appToastHost = document.getElementById("appToastHost");
const appDialog = document.getElementById("appDialog");
const appDialogBackdrop = document.getElementById("appDialogBackdrop");
const appDialogTitle = document.getElementById("appDialogTitle");
const appDialogBody = document.getElementById("appDialogBody");
const appDialogInputWrap = document.getElementById("appDialogInputWrap");
const appDialogInput = document.getElementById("appDialogInput");
const appDialogCancel = document.getElementById("appDialogCancel");
const appDialogConfirm = document.getElementById("appDialogConfirm");
const appBranchPicker = document.getElementById("appBranchPicker");
const appBranchPickerBackdrop = document.getElementById("appBranchPickerBackdrop");
const appBranchPickerTitle = document.getElementById("appBranchPickerTitle");
const appBranchPickerMeta = document.getElementById("appBranchPickerMeta");
const appBranchPickerList = document.getElementById("appBranchPickerList");
const appBranchPickerClose = document.getElementById("appBranchPickerClose");
const swipeContainer = document.getElementById("swipeContainer");
const PAGE_ELEMENTS = {
  setting: document.getElementById("pageSetting"),
  car: document.getElementById("pageCar"),
  tools: document.getElementById("pageTools"),
  terminal: document.getElementById("pageTerminal"),
  branch: document.getElementById("pageBranch"),
  carrot: document.getElementById("pageCarrot"),
};

function normalizeLangCode(raw) {
  const value = String(raw || "").trim().toLowerCase();
  if (value.startsWith("ko")) return "ko";
  if (value.startsWith("zh")) return "zh";
  if (value.startsWith("en")) return "en";
  return "";
}

function detectDefaultLang() {
  try {
    const stored = normalizeLangCode(localStorage.getItem(LANG_STORAGE_KEY));
    if (stored) return stored;
  } catch {}

  const browserLangs = Array.isArray(navigator.languages) && navigator.languages.length
    ? navigator.languages
    : [navigator.language, navigator.userLanguage];
  for (const candidate of browserLangs) {
    const normalized = normalizeLangCode(candidate);
    if (normalized) return normalized;
  }
  return "ko";
}

LANG = detectDefaultLang();
const PAGE_TRANSITION_CLASSES = [
  "page-transitioning",
  "page-active",
  "page-enter-from-right",
  "page-enter-from-left",
  "page-exit-to-left",
  "page-exit-to-right",
];
const PAGE_TRANSITION_MS = 280;
const SWIPE_SETTLE_MS = 220;
const SWIPE_COMMIT_RATIO = 0.22;
const SWIPE_VELOCITY_THRESHOLD = 0.45;
const SWIPE_EDGE_RESISTANCE = 0.18;
let pageTransitionTimer = null;
let pageTransitionToken = 0;
let CURRENT_PAGE = "carrot";
let appToastSerial = 0;
let activeAppToast = null;
let appToastHideTimer = null;
let appToastRemoveTimer = null;
let activeAppDialog = null;
let appDialogSerial = 0;

btnTools.onclick = () => showPage("tools", true, getSwipeTransition(CURRENT_PAGE, "tools"));

const btnChangeCar = document.getElementById("btnChangeCar");
const curCarLabelCar = document.getElementById("curCarLabelCar");
const curCarLabelSetting = document.getElementById("curCarLabelSetting");

// Setting screens
const settingTitle = document.getElementById("settingTitle");
const btnBackGroups = document.getElementById("btnBackGroups");
const settingCarRow = document.getElementById("settingCarRow");
const settingScreenHost = document.getElementById("settingScreenHost");
const screenGroups = document.getElementById("settingScreenGroups");
const screenItems = document.getElementById("settingScreenItems");
const settingSubnavWrap = document.getElementById("settingSubnavWrap");
const settingSubnav = document.getElementById("settingSubnav");
const itemsTitle = document.getElementById("itemsTitle");

// Car screens
const carTitle = document.getElementById("carTitle");
const btnBackCar = document.getElementById("btnBackCar");
const carMeta = document.getElementById("carMeta");
const carScreenMakers = document.getElementById("carScreenMakers");
const carScreenModels = document.getElementById("carScreenModels");
const makerList = document.getElementById("makerList");
const modelList = document.getElementById("modelList");
const modelTitle = document.getElementById("modelTitle");
const modelMeta = document.getElementById("modelMeta");

btnHome.onclick = () => showPage("carrot", true, getSwipeTransition(CURRENT_PAGE, "carrot"));
btnRecordToggle.onclick = () => toggleRecord();
btnSetting.onclick = () => showPage("setting", true, getSwipeTransition(CURRENT_PAGE, "setting"));
btnTerminal.onclick = () => showPage("terminal", true, getSwipeTransition(CURRENT_PAGE, "terminal"));

btnFleet.onclick = async () => {
  const ip = location.hostname;
  const url = `http://${ip}:8082/`;
  const ok = await appConfirm(
    `${getUIText("fleet_open_confirm", "Open Fleet?")}\n\n${url}`,
    { title: UI_STRINGS[LANG].fleet || "Fleet" },
  );
  if (!ok) return;
  window.open(url, "_blank", "noopener");
};

btnLang.onclick = () => toggleLang();

btnChangeCar.onclick = () => showPage("car", true);
btnBackCar.onclick = () => history.back();
carTitle.onclick = () => history.back();
modelTitle.onclick = () => showCarScreen("makers");

// Branch select
let BRANCHES = [];
let CURRENT_BRANCH_NAME = "";
const branchTitle = document.getElementById("branchTitle");
const btnBackBranch = document.getElementById("btnBackBranch");
const branchMeta = document.getElementById("branchMeta");
const branchList = document.getElementById("branchList");

// Quick Link
const quickLink = document.getElementById("toolsQuickLink");
const chipQuickLabel = document.getElementById("toolsQuickLinkTitle");
const btnSaveQuickLink = document.getElementById("btnToolsQuickLink");
let QUICK_LINK_URL = "";
let QUICK_LINK_STATUS = "loading";
let QUICK_LINK_MESSAGE = "";
let quickLinkActionTimer = null;

btnBackBranch.onclick = () => history.back();
branchTitle.onclick = () => history.back();

function clearPageTransitionClasses(el) {
  if (!el) return;
  el.classList.remove(...PAGE_TRANSITION_CLASSES);
}

function resetPageRuntimeStyles(el) {
  if (!el) return;
  el.style.transition = "";
  el.style.transform = "";
  el.style.opacity = "";
  el.style.zIndex = "";
  el.style.willChange = "";
  el.style.position = "";
  el.style.top = "";
  el.style.left = "";
  el.style.width = "";
}

function setDisplayedPage(page) {
  Object.entries(PAGE_ELEMENTS).forEach(([name, el]) => {
    if (!el) return;
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
    el.style.display = (name === page) ? "" : "none";
  });
  if (swipeContainer) swipeContainer.style.minHeight = "";
  if (settingScreenHost) settingScreenHost.style.minHeight = "";
}

function getSwipeTransition(fromPage, toPage) {
  if (fromPage === "terminal" || toPage === "terminal") return null;
  const fromIdx = SWIPE_PAGES.indexOf(fromPage);
  const toIdx = SWIPE_PAGES.indexOf(toPage);
  if (fromIdx < 0 || toIdx < 0 || fromIdx === toIdx) return null;
  return toIdx > fromIdx ? "forward" : "backward";
}

function getSwipeViewportMetrics(host = swipeContainer) {
  if (!host) {
    return { host: null, top: 0, left: 0, width: window.innerWidth || 1 };
  }

  const styles = window.getComputedStyle(host);
  const paddingTop = parseFloat(styles.paddingTop) || 0;
  const paddingLeft = parseFloat(styles.paddingLeft) || 0;
  const paddingRight = parseFloat(styles.paddingRight) || 0;
  const width = Math.max((host.clientWidth || window.innerWidth || 1) - paddingLeft - paddingRight, 1);

  return { host, top: paddingTop, left: paddingLeft, width };
}

function pinSwipeLayer(el, metrics) {
  if (!el) return;
  el.style.top = `${metrics.top}px`;
  el.style.left = `${metrics.left}px`;
  el.style.width = `${metrics.width}px`;
}

function updateSwipeFrameHeight(frame) {
  if (!frame?.host) return;
  const heights = [frame.fromEl, frame.toEl]
    .filter(Boolean)
    .map((el) => el.offsetHeight || 0);
  const maxHeight = Math.max(...heights, 0);
  frame.host.style.minHeight = maxHeight > 0 ? `${maxHeight}px` : "";
}

function prepareSwipeFrame(host, fromEl, toEl = null) {
  if (!host || !fromEl) return null;

  const metrics = getSwipeViewportMetrics(host);
  fromEl.style.display = "";
  fromEl.classList.add("page-transitioning", "page-active");
  fromEl.style.transition = "none";
  fromEl.style.willChange = "transform, opacity";
  pinSwipeLayer(fromEl, metrics);

  if (toEl) {
    toEl.style.display = "";
    toEl.classList.add("page-transitioning");
    toEl.style.transition = "none";
    toEl.style.willChange = "transform, opacity";
    pinSwipeLayer(toEl, metrics);
  }

  const frame = { host, fromEl, toEl, metrics, width: metrics.width };
  updateSwipeFrameHeight(frame);
  return frame;
}

function animatePageTransition(fromPage, toPage, transition) {
  const fromEl = PAGE_ELEMENTS[fromPage];
  const toEl = PAGE_ELEMENTS[toPage];
  if (!swipeContainer || !fromEl || !toEl || fromEl === toEl || !transition) {
    setDisplayedPage(toPage);
    return;
  }

  pageTransitionToken += 1;
  const token = pageTransitionToken;

  if (pageTransitionTimer) {
    clearTimeout(pageTransitionTimer);
    pageTransitionTimer = null;
  }

  Object.values(PAGE_ELEMENTS).forEach((el) => {
    if (!el) return;
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
    if (el !== fromEl && el !== toEl) el.style.display = "none";
  });

  const frame = prepareSwipeFrame(swipeContainer, fromEl, toEl);
  if (!frame) {
    setDisplayedPage(toPage);
    return;
  }

  toEl.classList.add(transition === "forward" ? "page-enter-from-right" : "page-enter-from-left");

  requestAnimationFrame(() => {
    requestAnimationFrame(() => {
      if (token !== pageTransitionToken) return;
      toEl.classList.add("page-active");
      toEl.classList.remove(
        transition === "forward" ? "page-enter-from-right" : "page-enter-from-left",
      );
      fromEl.classList.add(
        transition === "forward" ? "page-exit-to-left" : "page-exit-to-right",
      );
    });
  });

  pageTransitionTimer = setTimeout(() => {
    if (token !== pageTransitionToken) return;
    setDisplayedPage(toPage);
    pageTransitionTimer = null;
  }, PAGE_TRANSITION_MS);
}

function stopPageTransition() {
  if (pageTransitionTimer) {
    clearTimeout(pageTransitionTimer);
    pageTransitionTimer = null;
  }
  pageTransitionToken += 1;
  setDisplayedPage(CURRENT_PAGE);
}

function prepareSwipePages(fromPage, toPage) {
  const fromEl = PAGE_ELEMENTS[fromPage];
  const toEl = PAGE_ELEMENTS[toPage];
  if (!swipeContainer || !fromEl || !toEl) return null;

  stopPageTransition();

  Object.values(PAGE_ELEMENTS).forEach((el) => {
    if (!el) return;
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
    if (el !== fromEl && el !== toEl) el.style.display = "none";
  });

  return prepareSwipeFrame(swipeContainer, fromEl, toEl);
}

function applySwipeDrag(frame, dx, direction, withResistance = false) {
  if (!frame) return;
  const { fromEl, toEl, width } = frame;
  const dragX = withResistance ? dx * SWIPE_EDGE_RESISTANCE : dx;
  const progress = Math.min(Math.abs(dragX) / width, 1);
  const targetBase = direction === "forward" ? width : -width;

  fromEl.style.transform = `translateX(${dragX}px)`;
  fromEl.style.opacity = `${1 - (progress * 0.14)}`;

  if (toEl) {
    toEl.style.transform = `translateX(${targetBase + dragX}px)`;
    toEl.style.opacity = `${0.82 + (progress * 0.18)}`;
    toEl.style.zIndex = "2";
  }

  updateSwipeFrameHeight(frame);
}

function settleSwipe(frame, direction, commit, done) {
  if (!frame) {
    done();
    return;
  }
  const { fromEl, toEl, width } = frame;
  const outX = direction === "forward" ? -width : width;
  const inX = direction === "forward" ? width : -width;
  const transition = `transform ${SWIPE_SETTLE_MS}ms cubic-bezier(0.22, 1, 0.36, 1), opacity ${SWIPE_SETTLE_MS}ms ease`;

  fromEl.style.transition = transition;
  if (toEl) toEl.style.transition = transition;

  requestAnimationFrame(() => {
    fromEl.style.transform = commit ? `translateX(${outX}px)` : "translateX(0px)";
    fromEl.style.opacity = commit ? "0" : "1";

    if (toEl) {
      toEl.style.transform = commit ? "translateX(0px)" : `translateX(${inX}px)`;
      toEl.style.opacity = commit ? "1" : "0.82";
    }
  });

  window.setTimeout(done, SWIPE_SETTLE_MS);
}

function showPage(page, pushHistory = false, transition = null) {
  const prevPage = CURRENT_PAGE;
  if (prevPage === "terminal" && page !== "terminal" && typeof teardownTerminalPage === "function") {
    teardownTerminalPage();
  }
  CURRENT_PAGE = page;

  if (transition && prevPage !== page) animatePageTransition(prevPage, page, transition);
  else setDisplayedPage(page);

  document.body.dataset.page = page;
  window.dispatchEvent(new CustomEvent("carrot:pagechange", { detail: { page, prevPage } }));

  btnHome.classList.toggle("active", page === "carrot");
  btnSetting.classList.toggle("active", page === "setting");
  btnTools.classList.toggle("active", page === "tools");
  btnTerminal.classList.toggle("active", page === "terminal");

  if (page !== "setting" && typeof closeSettingSearchPanel === "function") {
    closeSettingSearchPanel({ clear: false });
  }

  // Terminal uses its own fixed viewport layout. Resetting the window scroll
  // while entering/leaving it causes visible jumps on mobile.
  if (prevPage !== "terminal" && page !== "terminal") {
    window.scrollTo(0, 0);
  }

  if (page === "setting") {
    if (!SETTINGS) loadSettings();
    else if (typeof syncSettingViewportLayout === "function" && typeof isCompactLandscapeMode === "function" && isCompactLandscapeMode()) {
      syncSettingViewportLayout().catch(() => {});
    } else if (pushHistory || !CURRENT_GROUP) showSettingScreen("groups", false);
  }

  if (page === "car") {
    showCarScreen("makers", false);
    if (!CARS) loadCars();
  }
  if (page === "tools") {
    initToolsPage();
    updateQuickLink().catch(() => {});
  }
  if (page === "terminal" && typeof initTerminalPage === "function") {
    initTerminalPage();
  }
  if (page === "carrot" && window.HomeDrive && typeof window.HomeDrive.refresh === "function") {
    window.HomeDrive.refresh();
  }
  if (page === "carrot") {
    loadRecordState().catch(() => {});
  }

  const state =
    (page === "setting") ? { page: "setting", screen: "groups", group: null } :
    (page === "car") ? { page: "car", screen: "makers", maker: null } :
    (page === "tools") ? { page: "tools" } :
    (page === "terminal") ? { page: "terminal" } :
    (page === "carrot") ? { page: "carrot" } :
    (page === "branch") ? { page: "branch" } :
    { page: "carrot" };

  if (pushHistory) history.pushState(state, "");
  else history.replaceState(state, "");
}

/* ---------- screen transitions (Setting) ---------- */
function showSettingScreen(which, pushHistory = false) {
  const isGroups = (which === "groups");
  const showEl = isGroups ? screenGroups : screenItems;
  const hideEl = isGroups ? screenItems : screenGroups;
  const currentGroupLabel = (!isGroups && CURRENT_GROUP && typeof getSettingGroupLabel === "function")
    ? getSettingGroupLabel(CURRENT_GROUP)
    : (CURRENT_GROUP || "");
  const splitLandscape = (CURRENT_PAGE === "setting" && typeof isCompactLandscapeMode === "function" && isCompactLandscapeMode());

  if (splitLandscape) {
    settingTitle.textContent = UI_STRINGS[LANG].setting || "Setting";
    if (settingSubnavWrap) settingSubnavWrap.style.display = "none";
    if (showEl) {
      showEl.style.display = "";
      showEl.classList.remove("hidden");
    }
    if (hideEl) {
      hideEl.style.display = "";
      hideEl.classList.remove("hidden");
    }
    if (pushHistory) {
      history.replaceState({ page: "setting", screen: "items", group: CURRENT_GROUP || null }, "");
    }
    if (settingScreenHost) settingScreenHost.style.minHeight = "";
    return;
  }

  if (btnBackGroups) btnBackGroups.style.display = "none";
  settingTitle.textContent = isGroups ? (UI_STRINGS[LANG].setting || "Setting") : ((UI_STRINGS[LANG].setting || "Setting") + " - " + currentGroupLabel);
  if (settingSubnavWrap) settingSubnavWrap.style.display = isGroups ? "none" : "";

  showEl.style.display = "";
  requestAnimationFrame(() => showEl.classList.remove("hidden"));

  hideEl.classList.add("hidden");
  setTimeout(() => { hideEl.style.display = "none"; }, 170);

  if (pushHistory) {
    history.pushState({ page: "setting", screen: which, group: CURRENT_GROUP || null }, "");
  }

  if (settingScreenHost) settingScreenHost.style.minHeight = "";
  if (isGroups && typeof setSettingItemsScrollTop === "function") {
    requestAnimationFrame(() => setSettingItemsScrollTop(0));
  }
}

if (btnBackGroups) btnBackGroups.onclick = () => history.back();
settingTitle.onclick = () => history.back();
if (itemsTitle) itemsTitle.onclick = () => history.back();

/* ---------- screen transitions (Car) ---------- */
function showCarScreen(which, pushHistory = false) {
  const isMakers = (which === "makers");
  const showEl = isMakers ? carScreenMakers : carScreenModels;
  const hideEl = isMakers ? carScreenModels : carScreenMakers;

  showEl.style.display = "";
  requestAnimationFrame(() => showEl.classList.remove("hidden"));

  hideEl.classList.add("hidden");
  setTimeout(() => { hideEl.style.display = "none"; }, 170);

  if (pushHistory) {
    history.pushState({ page: "car", screen: which, maker: CURRENT_MAKER || null }, "");
  }
}

function toggleLang() {
  if (LANG === "ko") LANG = "en";
  else if (LANG === "en") LANG = "zh";
  else LANG = "ko";
  try {
    localStorage.setItem(LANG_STORAGE_KEY, LANG);
  } catch {}

  updateLangLabel();

  // Update static UI text
  renderUIText();
  loadRecordState().catch(() => {});

  if (SETTINGS) {
    if (typeof rebuildSettingSearchEntries === "function") rebuildSettingSearchEntries();
    renderGroups();
    if (typeof renderSettingSubnav === "function") renderSettingSubnav();
    if (CURRENT_GROUP) {
      const currentTop = typeof getSettingItemsScrollTop === "function"
        ? getSettingItemsScrollTop()
        : 0;
      renderItems(CURRENT_GROUP, { scrollMode: "restore", scrollTop: currentTop });
    }
  }
}

function renderUIText() {
  const s = UI_STRINGS[LANG];
  if (!s) return;
  document.title = s.home || "Home";

  // Nav bar (nested spans — set last child text)
  setNavText("btnHome", s.home);
  setNavText("btnSetting", s.setting);
  setNavText("btnTools", s.tools);
  setNavText("btnTerminal", s.terminal);
  setNavText("btnFleet", s.fleet);

  setText("carrotTitle", s.home || "Home");

  // Car Select
  setText("carTitle", s.car_select);
  setText("btnBackCar", s.back);
  setText("makersTitle", s.makers);
  setText("modelTitle", s.models);

  // Setting
  setText("settingTitleText", s.setting);
  setText("btnBackGroups", s.back);
  setText("btnChangeCar", s.change);
  setText("groupsTitle", s.groups);
  setText("itemsTitle", s.items);

  // Tools
  setText("toolsTitle", s.tools);
  setText("gitCommandsTitle", s.git_commands);
  setText("userSystemTitle", s.user_system);
  setText("toolsQuickLinkTitle", s.quick_link);
  setText("btnToolsQuickLink", s.open);
  setText("userSettingsTitle", s.section_settings_backup);
  setText("btnReboot", s.reboot);
  setText("btnBackupSettings", s.backup);
  setText("btnRestoreSettings", s.restore);
  setText("sysCmdTitle", s.section_sys_cmd);
  setText("sysCmdHelp", s.sys_cmd_help);
  setText("outputTitle", s.section_output);
  setText("terminalTitle", s.terminal);
  setText("terminalSessionMeta", "/data/openpilot");
  setText("btnTerminalCtrlC", s.terminal_ctrl_c);
  setText("btnTerminalClear", s.terminal_clear);
  setText("btnTerminalReconnect", s.terminal_reconnect);
  setText("btnTerminalSend", s.terminal_send);
  const terminalInput = document.getElementById("terminalInput");
  if (terminalInput) terminalInput.placeholder = "";
  setText("settingSearchTitle", s.setting_search);
  if (settingSearchInput) settingSearchInput.placeholder = s.setting_search_placeholder || "";
  if (settingSearchMeta && (!settingSearchInput || !settingSearchInput.value.trim())) {
    settingSearchMeta.textContent = s.setting_search_idle || "";
  }
  if (btnSettingSearch) {
    btnSettingSearch.setAttribute("aria-label", s.setting_search || "Search Settings");
    btnSettingSearch.title = s.setting_search || "Search Settings";
  }
  if (btnSettingSearchSubmit) {
    btnSettingSearchSubmit.setAttribute("aria-label", s.setting_search || "Search Settings");
    btnSettingSearchSubmit.title = s.setting_search || "Search Settings";
  }
  if (typeof renderSettingSearchResults === "function" && settingSearchPanel && !settingSearchPanel.hidden) {
    renderSettingSearchResults(settingSearchInput?.value || "");
  }
  setText("appBranchPickerTitle", s.branch_select);
  setText("appBranchPickerClose", s.close);
  updateLangLabel();
  syncHomeUtilityButtons();
  if (window.DrivingHud && typeof window.DrivingHud.renderText === "function") {
    window.DrivingHud.renderText();
  }
  renderQuickLinkUI();
}

function setNavText(id, txt) {
  const el = document.getElementById(id);
  if (!el) return;
  // The label is the last <span> child
  const spans = el.querySelectorAll(":scope > span");
  if (spans.length >= 2) spans[spans.length - 1].textContent = txt;
  else el.textContent = txt;
}

function setText(id, txt) {
  const el = document.getElementById(id);
  if (el) el.textContent = txt;
}

function updateLangLabel() {
  if (!langLabel) return;

  const main = langLabel.querySelector(".lang-label__main");
  const sub = langLabel.querySelector(".lang-label__sub");
  const emoji = LANG_EMOJI[LANG] || "🌐";
  if (main && sub) {
    main.textContent = emoji;
    sub.textContent = "";
    sub.hidden = true;
  } else {
    langLabel.textContent = emoji;
  }

  if (btnLang) {
    const text = `${getUIText("lang", "lang")} (${LANG})`;
    btnLang.setAttribute("aria-label", text);
    btnLang.title = text;
  }
  document.documentElement.lang = LANG;
}

function getUIText(key, fallback = "") {
  return UI_STRINGS[LANG]?.[key] || fallback;
}

function syncModalBodyLock() {
  const hasOpenDialog =
    Boolean(appDialog && !appDialog.hidden) ||
    Boolean(appBranchPicker && !appBranchPicker.hidden) ||
    Boolean(settingSearchPanel && !settingSearchPanel.hidden);
  document.body.classList.toggle("dialog-open", hasOpenDialog);
}

function showAppToast(message, opts = {}) {
  if (!appToastHost || !message) return;

  const tone = opts.tone || "default";
  const duration = opts.duration ?? 2600;
  let toast = activeAppToast;
  if (!toast || !toast.isConnected) {
    toast = document.createElement("div");
    appToastHost.innerHTML = "";
    appToastHost.appendChild(toast);
    activeAppToast = toast;
  }

  toast.className = "app-toast";
  if (tone && tone !== "default") toast.classList.add(`is-${tone}`);
  toast.textContent = String(message);

  if (appToastHideTimer) {
    clearTimeout(appToastHideTimer);
    appToastHideTimer = null;
  }
  if (appToastRemoveTimer) {
    clearTimeout(appToastRemoveTimer);
    appToastRemoveTimer = null;
  }

  appToastSerial += 1;
  const toastSerial = appToastSerial;
  requestAnimationFrame(() => {
    if (!activeAppToast || toastSerial !== appToastSerial) return;
    activeAppToast.classList.add("is-visible");
  });

  appToastHideTimer = window.setTimeout(() => {
    if (!activeAppToast || toastSerial !== appToastSerial) return;
    activeAppToast.classList.remove("is-visible");
    appToastHideTimer = null;
    appToastRemoveTimer = window.setTimeout(() => {
      if (!activeAppToast || toastSerial !== appToastSerial) return;
      activeAppToast.remove();
      activeAppToast = null;
      appToastRemoveTimer = null;
    }, 180);
  }, duration);
}

function resolveAppDialog(result) {
  if (!activeAppDialog || !appDialog) return;

  const state = activeAppDialog;
  activeAppDialog = null;
  const dialogSerial = state.serial;
  appDialog.classList.remove("is-open");

  window.setTimeout(() => {
    if (dialogSerial !== appDialogSerial) {
      state.resolve(result);
      return;
    }
    appDialog.hidden = true;
    syncModalBodyLock();
    if (appDialogInputWrap) appDialogInputWrap.hidden = true;
    if (appDialogInput) {
      appDialogInput.value = "";
      appDialogInput.placeholder = "";
    }
    if (state.lastFocus && typeof state.lastFocus.focus === "function") {
      state.lastFocus.focus();
    }
    state.resolve(result);
  }, 180);
}

function cancelAppDialog() {
  if (!activeAppDialog) return;
  const result = activeAppDialog.mode === "prompt" ? null : false;
  resolveAppDialog(result);
}

function confirmAppDialog() {
  if (!activeAppDialog) return;
  const result = activeAppDialog.mode === "prompt"
    ? (appDialogInput ? appDialogInput.value : "")
    : true;
  resolveAppDialog(result);
}

function openAppDialog(options = {}) {
  if (!appDialog || !appDialogTitle || !appDialogBody || !appDialogConfirm || !appDialogCancel) {
    if (options.mode === "prompt") return Promise.resolve(null);
    return Promise.resolve(options.mode === "alert");
  }

  if (activeAppDialog) cancelAppDialog();

  const mode = options.mode || "alert";
  const title =
    options.title ||
    (mode === "confirm"
      ? getUIText("confirm_title", "Confirm")
      : mode === "prompt"
        ? getUIText("input_title", "Input")
        : getUIText("notice", "Notice"));
  const message = options.message || "";
  const confirmLabel = options.confirmLabel || getUIText("ok", "OK");
  const cancelLabel = options.cancelLabel || getUIText("cancel", "Cancel");
  const showCancel = mode !== "alert";

  appDialogTitle.textContent = title;
  appDialogBody.textContent = String(message);
  appDialogConfirm.textContent = confirmLabel;
  appDialogCancel.textContent = cancelLabel;
  appDialogCancel.hidden = !showCancel;
  appDialogCancel.setAttribute("aria-hidden", showCancel ? "false" : "true");

  if (appDialogInputWrap && appDialogInput) {
    const isPrompt = mode === "prompt";
    appDialogInputWrap.hidden = !isPrompt;
    appDialogInput.value = options.defaultValue ?? "";
    appDialogInput.placeholder = options.placeholder || "";
  }

  return new Promise((resolve) => {
    const dialogSerial = ++appDialogSerial;
    activeAppDialog = {
      resolve,
      mode,
      serial: dialogSerial,
      lastFocus: document.activeElement instanceof HTMLElement ? document.activeElement : null,
    };

    appDialog.hidden = false;
    syncModalBodyLock();

    requestAnimationFrame(() => {
      appDialog.classList.add("is-open");
      if (mode === "prompt" && appDialogInput) {
        appDialogInput.focus();
        appDialogInput.select();
      } else {
        appDialogConfirm.focus();
      }
    });
  });
}

function appAlert(message, opts = {}) {
  return openAppDialog({
    mode: "alert",
    title: opts.title,
    message,
    confirmLabel: opts.confirmLabel,
  });
}

function appConfirm(message, opts = {}) {
  return openAppDialog({
    mode: "confirm",
    title: opts.title,
    message,
    confirmLabel: opts.confirmLabel,
    cancelLabel: opts.cancelLabel,
  });
}

function appPrompt(message, opts = {}) {
  return openAppDialog({
    mode: "prompt",
    title: opts.title,
    message,
    confirmLabel: opts.confirmLabel,
    cancelLabel: opts.cancelLabel,
    defaultValue: opts.defaultValue,
    placeholder: opts.placeholder,
  });
}

if (appDialogBackdrop) appDialogBackdrop.onclick = cancelAppDialog;
if (appDialogCancel) appDialogCancel.onclick = cancelAppDialog;
if (appDialogConfirm) appDialogConfirm.onclick = confirmAppDialog;

document.addEventListener("keydown", (ev) => {
  if (!activeAppDialog) return;

  if (ev.key === "Escape") {
    ev.preventDefault();
    if (activeAppDialog.mode === "alert") resolveAppDialog(true);
    else cancelAppDialog();
    return;
  }

  if (ev.key === "Enter" && !ev.shiftKey) {
    const targetTag = ev.target?.tagName;
    if (targetTag === "TEXTAREA") return;
    ev.preventDefault();
    confirmAppDialog();
  }
});

function syncHomeUtilityButtons() {
  if (btnSaveQuickLink) {
    const label = getUIText("open", "Open");
    btnSaveQuickLink.textContent = label;
    btnSaveQuickLink.setAttribute("aria-label", label);
    btnSaveQuickLink.title = label;
  }
}

function flashQuickLinkActionLabel(label, duration = 1400) {
  if (!btnSaveQuickLink) return;
  if (quickLinkActionTimer) clearTimeout(quickLinkActionTimer);
  btnSaveQuickLink.textContent = label;
  quickLinkActionTimer = window.setTimeout(() => {
    quickLinkActionTimer = null;
    syncHomeUtilityButtons();
  }, duration);
}

function renderQuickLinkUI() {
  const hasUrl = Boolean(QUICK_LINK_URL);
  const emptyMessage = QUICK_LINK_MESSAGE || getUIText("quick_link_empty", "GithubUsername not set");
  const loadingMessage = getUIText("connecting", "Connecting...");
  const errorMessage = QUICK_LINK_MESSAGE || getUIText("error", "Error");
  const inlineText = hasUrl
    ? QUICK_LINK_URL
    : (
      QUICK_LINK_STATUS === "loading"
        ? loadingMessage
        : (QUICK_LINK_STATUS === "error" ? errorMessage : emptyMessage)
    );

  if (quickLink) {
    if (hasUrl) {
      quickLink.href = QUICK_LINK_URL;
      quickLink.textContent = inlineText;
      quickLink.removeAttribute("aria-disabled");
    } else {
      quickLink.removeAttribute("href");
      quickLink.setAttribute("aria-disabled", "true");
      quickLink.textContent = inlineText;
    }
  }

  if (btnSaveQuickLink) {
    btnSaveQuickLink.disabled = !hasUrl;
    btnSaveQuickLink.setAttribute("aria-disabled", hasUrl ? "false" : "true");
  }
}

function setServerStateStatus() {}

async function updateQuickLink() {
  QUICK_LINK_URL = "";
  QUICK_LINK_STATUS = "loading";
  QUICK_LINK_MESSAGE = "";
  renderQuickLinkUI();

  try {
    const values = await bulkGet(["GithubUsername"]);
    const githubId = String(values["GithubUsername"] || "").trim();

    if (!githubId) {
      QUICK_LINK_STATUS = "empty";
      QUICK_LINK_MESSAGE = "";
      renderQuickLinkUI();
      return;
    }

    QUICK_LINK_URL = `https://shind0.synology.me/carrot/go/?id=${encodeURIComponent(githubId)}`;
    QUICK_LINK_STATUS = "ready";
    QUICK_LINK_MESSAGE = "";
    renderQuickLinkUI();
  } catch (e) {
    QUICK_LINK_STATUS = "error";
    QUICK_LINK_MESSAGE = `QuickLink error: ${e?.message || e}`;
    renderQuickLinkUI();
    console.log("[QuickLink] failed:", e);
  }
}

async function openQuickLink() {
  if (!QUICK_LINK_URL) return;
  window.open(QUICK_LINK_URL, "_blank", "noopener");
}

if (btnSaveQuickLink) {
  btnSaveQuickLink.onclick = () => {
    openQuickLink().catch((e) => console.log("[QuickLink] open failed:", e));
  };
}

function escapeHtml(s) {
  return String(s)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function formatItemText(p, keyKo, keyEn, fallback = "") {
  if (LANG === "zh") return (p["c" + keyEn.slice(1)] || p[keyEn] || p[keyKo] || fallback);
  if (LANG === "ko") return (p[keyKo] ?? fallback);
  return (p[keyEn] ?? p[keyKo] ?? fallback);
}

function clamp(v, mn, mx) {
  if (Number.isFinite(mn) && v < mn) return mn;
  if (Number.isFinite(mx) && v > mx) return mx;
  return v;
}

/* ---------- Params helpers ---------- */
async function bulkGet(names) {
  const q = encodeURIComponent(names.join(","));
  const r = await fetch("/api/params_bulk?names=" + q);
  const j = await r.json();
  if (!j.ok) throw new Error(j.error || "bulk failed");
  return j.values || {};
}

async function setParam(name, value) {
  const r = await fetch("/api/param_set", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name, value })
  });
  const j = await r.json();
  if (!j.ok) throw new Error(j.error || "set failed");
  return true;
}

/* ── Swipe Navigation ──────────────────────────────────── */
const SWIPE_PAGES = ["carrot", "setting", "tools", "terminal"];
const SETTING_BACK_EDGE_WIDTH = 32;

function isLandscapeRailMode() {
  return window.matchMedia("(orientation: landscape) and (max-height: 560px) and (pointer: coarse)").matches;
}

function isSettingItemsScreenActive() {
  return Boolean(
    CURRENT_PAGE === "setting" &&
    screenItems &&
    screenItems.style.display !== "none" &&
    !screenItems.classList.contains("hidden")
  );
}

function prepareSettingBackFrame() {
  if (!settingScreenHost || !screenItems || !screenGroups) return null;
  if (typeof stopSettingSubnavMotion === "function") stopSettingSubnavMotion();

  [screenItems, screenGroups].forEach((el) => {
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
    el.classList.remove("hidden");
  });

  const frame = prepareSwipeFrame(settingScreenHost, screenItems, screenGroups);
  if (!frame) return null;
  settingScreenHost.classList.add("setting-back-swiping");
  screenItems.style.zIndex = "2";
  screenGroups.style.zIndex = "1";
  return frame;
}

function cleanupSettingBackFrame() {
  if (!settingScreenHost || !screenItems || !screenGroups) return;
  settingScreenHost.style.minHeight = "";
  settingScreenHost.classList.remove("setting-back-swiping");
  [screenItems, screenGroups].forEach((el) => {
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
  });
}

(function initSwipe() {
  const el = swipeContainer;
  if (!el) return;

  let gesture = null;

  el.addEventListener("touchstart", (e) => {
    if (isLandscapeRailMode()) {
      gesture = null;
      return;
    }
    const inSettingItems = isSettingItemsScreenActive();
    if (
      e.touches.length !== 1 ||
      !SWIPE_PAGES.includes(CURRENT_PAGE) ||
      inSettingItems ||
      (inSettingItems && e.target?.closest?.("#settingSubnavWrap"))
    ) {
      gesture = null;
      return;
    }

    const touch = e.touches[0];
    gesture = {
      tracking: true,
      dragging: false,
      startX: touch.clientX,
      startY: touch.clientY,
      dx: 0,
      direction: null,
      targetPage: null,
      settingGroupTarget: null,
      settingBackTarget: false,
      frame: null,
      velocity: 0,
      lastX: touch.clientX,
      lastTime: performance.now(),
    };
  }, { passive: true });

  el.addEventListener("touchmove", (e) => {
    if (isLandscapeRailMode()) {
      gesture = null;
      return;
    }
    const inSettingItems = isSettingItemsScreenActive();
    if (inSettingItems) {
      gesture = null;
      return;
    }
    if (!gesture?.tracking || e.touches.length !== 1) return;

    const touch = e.touches[0];
    const dx = touch.clientX - gesture.startX;
    const dy = touch.clientY - gesture.startY;

    if (!gesture.dragging) {
      if (Math.abs(dx) < 10 && Math.abs(dy) < 10) return;
      if (Math.abs(dy) > Math.abs(dx) * 0.9) {
        gesture = null;
        return;
      }

      const inSettingItems = isSettingItemsScreenActive();
      const direction = dx < 0 ? "forward" : "backward";
      const isSettingEdgeBack = inSettingItems && direction === "backward" && gesture.startX <= SETTING_BACK_EDGE_WIDTH;
      if (isSettingEdgeBack) {
        gesture = null;
        return;
      }

      const idx = SWIPE_PAGES.indexOf(CURRENT_PAGE);
      const nextSettingGroup = inSettingItems && typeof getSettingSubnavShiftTarget === "function"
        ? getSettingSubnavShiftTarget(direction)
        : null;
      const settingBackTarget = Boolean(inSettingItems && direction === "backward" && nextSettingGroup?.reachedEdge);
      const settingGroupTarget = (nextSettingGroup && !nextSettingGroup.reachedEdge)
        ? nextSettingGroup.group
        : null;
      const targetPage = settingGroupTarget
        ? null
        : (
          inSettingItems
            ? (direction === "forward" ? "tools" : null)
            : (direction === "forward" ? SWIPE_PAGES[idx + 1] : SWIPE_PAGES[idx - 1])
        );

      gesture.dragging = true;
      gesture.direction = direction;
      gesture.targetPage = targetPage || null;
      gesture.settingGroupTarget = settingGroupTarget || null;
      gesture.settingBackTarget = settingBackTarget;
      gesture.edgeResistance = !targetPage && !settingGroupTarget && !settingBackTarget;
      gesture.frame = targetPage
        ? prepareSwipePages(CURRENT_PAGE, targetPage)
        : (
          settingGroupTarget
            ? null
            : (
              settingBackTarget
                ? prepareSettingBackFrame()
                : (stopPageTransition(), prepareSwipeFrame(swipeContainer, PAGE_ELEMENTS[CURRENT_PAGE]))
            )
        );
    }

    if (!gesture.dragging) return;

    e.preventDefault();

    const constrainedDx =
      gesture.direction === "forward" ? Math.min(dx, 0) : Math.max(dx, 0);

    const now = performance.now();
    const dt = Math.max(now - gesture.lastTime, 1);
    gesture.velocity = (touch.clientX - gesture.lastX) / dt;
    gesture.lastX = touch.clientX;
    gesture.lastTime = now;
    gesture.dx = constrainedDx;

    if (gesture.frame) applySwipeDrag(gesture.frame, constrainedDx, gesture.direction, gesture.edgeResistance);
  }, { passive: false });

  el.addEventListener("touchend", (e) => {
    if (isLandscapeRailMode()) {
      gesture = null;
      return;
    }
    const inSettingItems = isSettingItemsScreenActive();
    if (inSettingItems) {
      gesture = null;
      return;
    }
    if (!gesture) return;

    if (!gesture.dragging) {
      gesture = null;
      return;
    }

    const dx = gesture.dx;
    const width = gesture.frame?.width || getSwipeViewportMetrics(swipeContainer).width;
    const travel = Math.abs(dx) / width;
    const velocityOk =
      (gesture.direction === "forward" && gesture.velocity < -SWIPE_VELOCITY_THRESHOLD) ||
      (gesture.direction === "backward" && gesture.velocity > SWIPE_VELOCITY_THRESHOLD);
    const shouldCommitPage = Boolean(gesture.targetPage) && (travel > SWIPE_COMMIT_RATIO || velocityOk);
    const shouldCommitGroup = Boolean(gesture.settingGroupTarget) && (Math.abs(dx) > 48 || velocityOk);
    const shouldCommitBack = Boolean(gesture.settingBackTarget) && (travel > SWIPE_COMMIT_RATIO || velocityOk);

    if (gesture.frame && gesture.targetPage) {
      const targetPage = gesture.targetPage;
      const direction = gesture.direction;
      const frame = gesture.frame;
      gesture = null;
      settleSwipe(frame, direction, shouldCommitPage, () => {
        if (shouldCommitPage && targetPage) showPage(targetPage, true, null);
        else setDisplayedPage(CURRENT_PAGE);
      });
      return;
    }

    if (gesture.settingGroupTarget) {
      const groupTarget = gesture.settingGroupTarget;
      const direction = gesture.direction;
      gesture = null;
      if (shouldCommitGroup && typeof animateSettingGroupSwitch === "function") {
        animateSettingGroupSwitch(groupTarget, direction).catch((e) => console.log("[SettingSwipe] switch failed:", e));
      } else if (shouldCommitGroup && typeof selectGroup === "function") {
        selectGroup(groupTarget, false);
      } else if (typeof centerActiveSettingSubnavTab === "function") {
        centerActiveSettingSubnavTab("smooth");
      }
      return;
    }

    if (gesture.settingBackTarget && gesture.frame) {
      const frame = gesture.frame;
      gesture = null;
      settleSwipe(frame, "backward", shouldCommitBack, () => {
        cleanupSettingBackFrame();
        if (shouldCommitBack) history.back();
        else showSettingScreen("items", false);
      });
      return;
    }

    if (gesture.frame) {
      const frame = gesture.frame;
      const direction = gesture.direction;
      gesture = null;
      settleSwipe(frame, direction, false, () => setDisplayedPage(CURRENT_PAGE));
      return;
    }
    gesture = null;
  }, { passive: true });

  el.addEventListener("touchcancel", () => {
    if (!gesture) return;
    if (gesture.settingBackTarget && gesture.frame) {
      const frame = gesture.frame;
      gesture = null;
      settleSwipe(frame, "backward", false, () => {
        cleanupSettingBackFrame();
        showSettingScreen("items", false);
      });
      return;
    }
    if (gesture.frame) {
      const frame = gesture.frame;
      const direction = gesture.direction;
      gesture = null;
      settleSwipe(frame, direction, false, () => setDisplayedPage(CURRENT_PAGE));
      return;
    }
    setDisplayedPage(CURRENT_PAGE);
    gesture = null;
  }, { passive: true });
})();

(function initSettingBackSwipe() {
  const host = settingScreenHost;
  if (!host || !screenItems || !screenGroups) return;

  let gesture = null;

  host.addEventListener("touchstart", (e) => {
    if (isLandscapeRailMode()) {
      gesture = null;
      return;
    }
    if (
      e.touches.length !== 1 ||
      !isSettingItemsScreenActive() ||
      e.target?.closest?.("#settingSubnav") ||
      e.touches[0].clientX > SETTING_BACK_EDGE_WIDTH
    ) {
      gesture = null;
      return;
    }

    const touch = e.touches[0];
    gesture = {
      dragging: false,
      startX: touch.clientX,
      startY: touch.clientY,
      dx: 0,
      velocity: 0,
      lastX: touch.clientX,
      lastTime: performance.now(),
      frame: null,
    };
  }, { passive: true });

  host.addEventListener("touchmove", (e) => {
    if (isLandscapeRailMode()) {
      gesture = null;
      return;
    }
    if (!gesture || e.touches.length !== 1) return;

    const touch = e.touches[0];
    const dx = touch.clientX - gesture.startX;
    const dy = touch.clientY - gesture.startY;

    if (!gesture.dragging) {
      if (dx < 10 && Math.abs(dy) < 10) return;
      if (dx <= 0 || Math.abs(dy) > Math.abs(dx) * 0.9) {
        gesture = null;
        return;
      }

      if (typeof stopSettingSubnavMotion === "function") stopSettingSubnavMotion();
      gesture.dragging = true;
      gesture.frame = prepareSettingBackFrame();
    }

    e.preventDefault();

    const constrainedDx = Math.max(dx, 0);
    const now = performance.now();
    const dt = Math.max(now - gesture.lastTime, 1);
    gesture.velocity = (touch.clientX - gesture.lastX) / dt;
    gesture.lastX = touch.clientX;
    gesture.lastTime = now;
    gesture.dx = constrainedDx;

    applySwipeDrag(gesture.frame, constrainedDx, "backward");
  }, { passive: false });

  host.addEventListener("touchend", () => {
    if (isLandscapeRailMode()) {
      gesture = null;
      return;
    }
    if (!gesture) return;
    if (!gesture.dragging || !gesture.frame) {
      gesture = null;
      return;
    }

    const travel = gesture.dx / gesture.frame.width;
    const shouldCommit = travel > SWIPE_COMMIT_RATIO || gesture.velocity > SWIPE_VELOCITY_THRESHOLD;
    const frame = gesture.frame;
    gesture = null;

    settleSwipe(frame, "backward", shouldCommit, () => {
      cleanupSettingBackFrame();
      if (shouldCommit) history.back();
      else showSettingScreen("items", false);
    });
  }, { passive: true });

  host.addEventListener("touchcancel", () => {
    if (!gesture) return;
    const frame = gesture.frame;
    gesture = null;

    if (!frame) {
      cleanupSettingBackFrame();
      showSettingScreen("items", false);
      return;
    }

    settleSwipe(frame, "backward", false, () => {
      cleanupSettingBackFrame();
      showSettingScreen("items", false);
    });
  }, { passive: true });
})();

syncHomeUtilityButtons();
renderQuickLinkUI();
