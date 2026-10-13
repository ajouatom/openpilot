# Settings Wiki patches

These patches preserve reviewed Wiki edits in the code branch when the author
cannot push to the separate Wiki repository. Committing a patch here does not
publish its content to the live Wiki.

## RAV4 PCM mode 0

`rav4-pcm-mode0.patch` updates only the Korean and English `SpeedFromPCM`
manual regions from public Wiki commit `fbe96d08c066844a60dc909500c1e2d9393bca5d`.
It describes observed SET/RES speed synchronization for the exact 2022 RAV4
platform with PCM and direct longitudinal control, local-input priority, and
the preserved lower deceleration targets. It does not enable corner hardware
or add button transmission. This patch is for review with the code PR; the live
Wiki has not been changed.

```sh
git apply --check --unidiff-zero /absolute/path/to/rav4-pcm-mode0.patch
git apply --unidiff-zero /absolute/path/to/rav4-pcm-mode0.patch
```

The patch uses zero context lines. Validate both edited pages and regenerate the catalog using the authoring guide
before publishing. Reconcile later manual edits if the check fails.

## RAV4 radar display

`rav4-radar-display.patch` updates the Korean and English `ShowRadarInfo` manual
regions from Wiki commit `fbe96d08c066844a60dc909500c1e2d9393bca5d`. It describes
the separate ARS display stream on the standard device screen in mode 3,
unchanged control targets, freshness checks, and the web/mici limitation.
The live Wiki has not been changed.

```sh
git apply --check --unidiff-zero /absolute/path/to/rav4-radar-display.patch
git apply --unidiff-zero /absolute/path/to/rav4-radar-display.patch
```

Only the matching manual regions are edited. Validate both pages and regenerate
the catalog before publishing, following the authoring guide.
