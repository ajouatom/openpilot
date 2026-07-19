const HASH_PATTERN = /^[0-9a-f]{64}$/;
const PATH_PATTERN = /^(?!\/)(?!.*(?:^|\/)\.\.(?:\/|$))(?!.*\\).+$/;

export class AssetManifestError extends Error {
  constructor(message, options) {
    super(message, options);
    this.name = "AssetManifestError";
  }
}

function assetMap(manifest) {
  if (manifest?.schemaVersion !== 1 || !Array.isArray(manifest.assets)) {
    throw new AssetManifestError("Invalid asset manifest schema");
  }
  const assets = new Map();
  const paths = new Set();
  for (const asset of manifest.assets) {
    if (
      typeof asset?.id !== "string"
      || typeof asset.path !== "string"
      || !PATH_PATTERN.test(asset.path)
      || typeof asset.hash !== "string"
      || !HASH_PATTERN.test(asset.hash)
      || assets.has(asset.id)
      || paths.has(asset.path)
    ) {
      throw new AssetManifestError("Invalid asset manifest entry");
    }
    assets.set(asset.id, asset);
    paths.add(asset.path);
  }
  return assets;
}

export function createAssetUrlResolver(manifest) {
  const assets = assetMap(manifest);
  return Object.freeze({
    resolve(logicalId) {
      const asset = assets.get(logicalId);
      if (!asset) throw new AssetManifestError(`Unknown asset id: ${logicalId}`);
      return `/${asset.path}?v=${asset.hash}`;
    },
  });
}

export function installAssetUrlFacade(documentRoot = document, target = globalThis) {
  const manifestNode = documentRoot.getElementById("carrotAssetManifest");
  if (!manifestNode) throw new AssetManifestError("Asset manifest element is missing");
  let manifest;
  try {
    manifest = JSON.parse(manifestNode.textContent || "");
  } catch (error) {
    throw new AssetManifestError("Asset manifest JSON is invalid", { cause: error });
  }
  const facade = createAssetUrlResolver(manifest);
  target.CarrotAssetUrl = facade;
  return facade;
}
