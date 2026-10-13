# Settings Wiki patches

These patches preserve reviewed Wiki edits in the code branch when the author
cannot push to the separate Wiki repository. Committing a patch here does not
publish its content to the live Wiki.

## Bosch passive reception

`bosch-passive-reception.patch` changes only the Korean and English
`EnableRadarTracks` manual regions. It documents the Bosch startup exception,
the unchanged activation-result parameter, and the distinction between passive
radar reception and vehicle control. It also corrects the stale mode-0 manual
description to match the current SCC-only policy.

The patch is based on `ajouatom/openpilot.wiki.git` commit
`d7117c2db74a5e683f9e8d4d2338ac68bebabff0` and corresponds to Wiki commit
`6fba8803dec100aeaecd9d4751bc205402699f78`. The Bosch behavior was verified on
the Bosch code branch; the text explicitly applies to versions supporting this
receiver. The candidate-platform wording (Elantra AD plus unvalidated i30 PD, Kona,
Kona Electric and K3 BD/Forte) was added on 2026-10-12, after that Wiki commit.

From a Wiki checkout, use the absolute path to this patch:

```sh
git apply --check --unidiff-zero /absolute/path/to/bosch-passive-reception.patch
git apply --unidiff-zero /absolute/path/to/bosch-passive-reception.patch
```

The patch uses zero context lines, so `--unidiff-zero` is required. Follow the
[authoring guide](../AUTHORING_GUIDE.md), validate the edited pages and regenerate
the Wiki catalog before publishing. If the check fails after later Wiki edits,
reconcile only the matching manual regions instead of forcing the patch.

## RAV4 PCM mode 0

`rav4-pcm-mode0.patch` updates only the Korean and English `SpeedFromPCM`
manual regions from public Wiki commit `fbe96d08c066844a60dc909500c1e2d9393bca5d`.
It describes observed SET/RES speed synchronization for the exact 2022 RAV4
platform with PCM and direct longitudinal control, local-input priority, and
the preserved lower deceleration targets. It does not enable corner hardware
or add button transmission. The reviewed text was published to the live Wiki on
October 14 in commit `e4f2435`; the patch remains as the original review artifact.

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
The reviewed text was published to the live Wiki on October 14 in commit `e4f2435`.

```sh
git apply --check --unidiff-zero /absolute/path/to/rav4-radar-display.patch
git apply --unidiff-zero /absolute/path/to/rav4-radar-display.patch
```

Only the matching manual regions are edited. Validate both pages and regenerate
the catalog before publishing, following the authoring guide.
