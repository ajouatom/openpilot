"""Data-only R2 boot patch. No kernel, firmware or model replacement."""
import hashlib

R2_INIT_SHA = '83174f01887fb443de24e4c9a64ccab3543d10ddebcfc532b8ec2d462b0b3a94'
APP_UUID = 'd3fb8cf2-60ea-44b1-bec1-63a7417719cf'

# This runs AFTER NVIDIA loads the PCIe/NVMe drivers and BEFORE mounting APP.
# Resolve exactly one matching partition; a cloned SD+NVMe pair is ambiguous.
RESOLVE = '''
# Carrot R2 portable storage: select one SD/NVMe APP, never a global DATA label.
carrot_root=""
for carrot_attempt in {1..50}; do
  carrot_matches=$(blkid -t "PARTUUID=APP_UUID" -o device)
  if [ -n "${carrot_matches}" ]; then
    if [ "$(printf '%s\\n' "${carrot_matches}" | wc -l)" -ne 1 ]; then
      echo "CARROT: remove the duplicate SD/NVMe image" > /dev/kmsg
      exec /bin/bash
    fi
    carrot_root=$(readlink -f -- "${carrot_matches}")
    break
  fi
  sleep 0.2
done
if [[ ! "${carrot_root}" =~ ^/dev/(mmcblk[0-9]+|nvme[0-9]+n[0-9]+)p1$ ]]; then
  echo "CARROT: expected one SD/NVMe APP partition" > /dev/kmsg
  exec /bin/bash
fi
rootdev="${carrot_root#/dev/}"
'''.replace('APP_UUID', APP_UUID)


def patch(body):
  if hashlib.sha256(body).hexdigest() != R2_INIT_SHA:
    raise ValueError('Only the exact reviewed R2 init program can be patched')
  text = body.decode('utf-8')
  replacements = {
    'if [ "${rootdev}" != "mmcblk0p1" ]; then':
      f'if [ "${{rootdev}}" != "PARTUUID={APP_UUID}" ]; then',
    '\nrootfs_is_encrypted=0\n': RESOLVE + '\nrootfs_is_encrypted=0\n',
    'chroot . /sbin/blockdev --setro /dev/mmcblk0p1':
      'chroot . /sbin/blockdev --setro "${carrot_root}"',
  }
  for old, new in replacements.items():
    if text.count(old) != 1:
      raise ValueError('Unexpected portable initrd anchor')
    text = text.replace(old, new, 1)
  return text.encode('utf-8')
