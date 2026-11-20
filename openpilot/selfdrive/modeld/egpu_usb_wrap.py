"""Backport tinygrad 42ea17d1's USB SDMA ring-wrap zero-fill rule.

The fix belongs to the d39227 host lowering path, not the driving graph. Keep
all compiled model/runtime pairs and older USB implementations unchanged.
"""
from openpilot.selfdrive.modeld.egpu_usb_wait import LIGHT_SPEED_RUNTIME_SHA


def _usb_zero_fill(dst, n):
  from tinygrad.runtime.support import usb
  return usb.usb_memcpy(dst, usb.usb_zeros(dst.src[0].device).index(0), n, fill=True)


def configure_usb_wrap(manifest):
  runtime = manifest.get('runtime')
  if not isinstance(runtime, dict) or runtime.get('sha256') != LIGHT_SPEED_RUNTIME_SHA:
    return False

  from tinygrad.device import Compiled
  from tinygrad.dtype import dtypes
  from tinygrad.uop.ops import PatternMatcher, UPat
  from tinygrad.runtime.support import usb

  if getattr(usb, '_carrot_sdma_wrap_fixed', False):
    return True
  # Upstream's UPat.const(0).or_casted() catches the CAST(0) generated when
  # padding a wrapped SDMA ring. Otherwise it falls through to host memset
  # against an address belonging to the USB GPU. Prepend the corrected rule
  # so it takes precedence over the old bare-zero rule, before any load/JIT.
  fixed = PatternMatcher([
    (UPat.custom_function('memset', dtypes.uint64).call(UPat.var('dst'), UPat.const(0).or_casted(), UPat.var('n')),
     _usb_zero_fill),
  ])
  Compiled.pm_lower = fixed + Compiled.pm_lower
  usb._carrot_sdma_wrap_fixed = True
  return True
