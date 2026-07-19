import { installAssetUrlFacade } from "../shared/assets/asset_url.js";
import { installDialogFacade } from "../ui/components/dialog/facade.js";
import { installToastFacade } from "../ui/components/feedback/facade.js";
import { installMenuFacade } from "../ui/components/menu/facade.js";
import { installPopoverFacade } from "../ui/components/popover/facade.js";
import { installSegmentedControlFacade } from "../ui/components/segmented_control/facade.js";
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
