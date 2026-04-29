"use strict";

// All DOM references used across shared/ and pages/ modules.
// Declared once here so other files can reference these as plain globals
// (top-level let/const are shared across non-module <script> tags).

// Bottom navigation
const btnHome = document.getElementById("btnHome");
const btnSetting = document.getElementById("btnSetting");
const btnLogs = document.getElementById("btnLogs");
const btnTerminal = document.getElementById("btnTerminal");
const btnTools = document.getElementById("btnTools");
const btnRecordToggle = document.getElementById("btnRecordToggle");

// Language toggle
const btnLang = document.getElementById("btnLang");
const langLabel = document.getElementById("langLabel");
const btnSettingLang = document.getElementById("btnSettingLang");

// Quick link
const btnQuickLinkWeb = document.getElementById("btnQuickLinkWeb");
const quickLink = document.getElementById("toolsQuickLink");
const chipQuickLabel = document.getElementById("toolsQuickLinkTitle");
const btnSaveQuickLink = document.getElementById("btnToolsQuickLink");

// Setting search
const btnSettingSearch = document.getElementById("btnSettingSearch");
const settingSearchBackdrop = document.getElementById("settingSearchBackdrop");
const settingSearchPanel = document.getElementById("settingSearchPanel");
const settingSearchTitle = document.getElementById("settingSearchTitle");
const settingSearchForm = document.getElementById("settingSearchForm");
const settingSearchInput = document.getElementById("settingSearchInput");
const btnSettingSearchSubmit = document.getElementById("btnSettingSearchSubmit");
const settingSearchMeta = document.getElementById("settingSearchMeta");
const settingSearchResults = document.getElementById("settingSearchResults");

// Toast + dialog
const appToastHost = document.getElementById("appToastHost");
const appDialog = document.getElementById("appDialog");
const appDialogBackdrop = document.getElementById("appDialogBackdrop");
const appDialogTitle = document.getElementById("appDialogTitle");
const appDialogBody = document.getElementById("appDialogBody");
const appDialogChoices = document.getElementById("appDialogChoices");
const appDialogInputWrap = document.getElementById("appDialogInputWrap");
const appDialogInput = document.getElementById("appDialogInput");
const appDialogCancel = document.getElementById("appDialogCancel");
const appDialogCopy = document.getElementById("appDialogCopy");
const appDialogConfirm = document.getElementById("appDialogConfirm");

// Branch picker
const appBranchPicker = document.getElementById("appBranchPicker");
const appBranchPickerBackdrop = document.getElementById("appBranchPickerBackdrop");
const appBranchPickerTitle = document.getElementById("appBranchPickerTitle");
const appBranchPickerMeta = document.getElementById("appBranchPickerMeta");
const appBranchPickerList = document.getElementById("appBranchPickerList");
const appBranchPickerClose = document.getElementById("appBranchPickerClose");

// Car picker
const appCarPicker = document.getElementById("appCarPicker");
const appCarPickerBackdrop = document.getElementById("appCarPickerBackdrop");
const appCarPickerTitle = document.getElementById("appCarPickerTitle");
const appCarPickerMeta = document.getElementById("appCarPickerMeta");
const appCarPickerList = document.getElementById("appCarPickerList");
const appCarPickerClose = document.getElementById("appCarPickerClose");

// Page container + page elements
const swipeContainer = document.getElementById("swipeContainer");
const PAGE_ELEMENTS = {
  setting: document.getElementById("pageSetting"),
  car: document.getElementById("pageCar"),
  tools: document.getElementById("pageTools"),
  logs: document.getElementById("pageLogs"),
  terminal: document.getElementById("pageTerminal"),
  branch: document.getElementById("pageBranch"),
  carrot: document.getElementById("pageCarrot"),
};

// Current car labels (used by car page logic in app_pages.js)
const curCarLabelCar = document.getElementById("curCarLabelCar");
const curCarLabelSetting = document.getElementById("curCarLabelSetting");

// Setting page sub-elements
const settingTitle = document.getElementById("settingTitle");
const btnBackGroups = document.getElementById("btnBackGroups");
const settingCarRow = document.getElementById("settingCarRow");
const settingScreenHost = document.getElementById("settingScreenHost");
const screenGroups = document.getElementById("settingScreenGroups");
const screenItems = document.getElementById("settingScreenItems");
const settingSubnavWrap = document.getElementById("settingSubnavWrap");
const settingSubnav = document.getElementById("settingSubnav");
const itemsTitle = document.getElementById("itemsTitle");

// Car page sub-elements
const carTitle = document.getElementById("carTitle");
const btnBackCar = document.getElementById("btnBackCar");
const carMeta = document.getElementById("carMeta");
const carScreenMakers = document.getElementById("carScreenMakers");
const carScreenModels = document.getElementById("carScreenModels");
const makerList = document.getElementById("makerList");
const modelList = document.getElementById("modelList");
const modelTitle = document.getElementById("modelTitle");
const modelMeta = document.getElementById("modelMeta");

// Branch page sub-elements
const branchTitle = document.getElementById("branchTitle");
const btnBackBranch = document.getElementById("btnBackBranch");
const branchMeta = document.getElementById("branchMeta");
const branchList = document.getElementById("branchList");
