"""Keep legacy top-level imports compatible with repository-root tools."""
from pkgutil import extend_path

# Cluster entry points also expose openpilot/ on sys.path. When that makes
# this package resolve as `tools`, include the root namespace containing
# jetlink and signal_analysis instead of hiding it behind this regular package.
if __name__ == 'tools':
  __path__ = extend_path(__path__, __name__)
