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
