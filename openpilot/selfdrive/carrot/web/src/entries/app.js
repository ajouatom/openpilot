import { installAssetUrlFacade } from "../shared/assets/asset_url.js";
import { installParamCommitFacade } from "../shared/params/facade.js";
import { installDialogFacade } from "../ui/components/dialog/facade.js";
import { installDiffTableFacade } from "../ui/components/diff_table/facade.js";
import { installToastFacade } from "../ui/components/feedback/facade.js";
import { installMenuFacade } from "../ui/components/menu/facade.js";
import { installNumericStepperFacade } from "../ui/components/numeric_stepper/facade.js";
import { installPopoverFacade } from "../ui/components/popover/facade.js";
import { installSegmentedControlFacade } from "../ui/components/segmented_control/facade.js";
import { installSettingRowFacade } from "../ui/components/setting_row/facade.js";
import { installStateSurfaceFacade } from "../ui/components/state_surface/facade.js";

const target = globalThis;
const documentRoot = target.document;

if (documentRoot.getElementById("carrotAssetManifest")) installAssetUrlFacade(documentRoot, target);
installDialogFacade(target, documentRoot);
installMenuFacade(target);
installPopoverFacade(target);
installToastFacade(target, { documentRoot });
installStateSurfaceFacade(target, { document: documentRoot });
installSegmentedControlFacade(target);
installNumericStepperFacade(target);
installParamCommitFacade(target);
installSettingRowFacade(target);
installDiffTableFacade(target);
