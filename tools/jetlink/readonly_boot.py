"""Preserve NVIDIA initialization while avoiding writes to already-correct links."""

MARKER = '# Carrot: retain matching immutable NVIDIA links'
HELPER = '''
# Carrot: retain matching immutable NVIDIA links
CARROT_LINK_ERROR=0
carrot_nv_link() {
  if [ -L "$2" ] && [ "$(readlink -- "$2")" = "$1" ]; then
    return 0
  fi
  command ln -sf -- "$1" "$2" || { CARROT_LINK_ERROR=1; return 1; }
}
'''


def patch_nv_script(source):
  if MARKER in source:
    return source
  if not source.startswith('#!/bin/bash\n') or 'ln -sf ' not in source:
    raise ValueError('Unexpected NVIDIA boot script')
  # Keep all NVIDIA runtime initialization and missing/incorrect-link failures.
  return source.replace('#!/bin/bash\n', '#!/bin/bash\n' + HELPER, 1).replace(
    'ln -sf "', 'carrot_nv_link "') + '\n[ "$CARROT_LINK_ERROR" -eq 0 ]\n'
