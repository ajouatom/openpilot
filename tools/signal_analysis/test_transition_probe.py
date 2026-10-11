import copy

from tools.signal_analysis.train_transition_probe import fit, predict


def test_evaluation_rows_cannot_change_training_weights_or_normalization():
  rows = [dict(group='train', split='train', truth='red', features=[1., 0.]),
          dict(group='train', split='train', truth='green', features=[0., 1.]),
          dict(group='held_out', split='evaluation', truth='red', features=[1e9, -1e9])]
  before = fit(rows, 2)
  changed = copy.deepcopy(rows)
  changed[-1].update(truth='green', features=[-1e20, 1e20])
  assert before == fit(changed, 2)
  assert predict(before, [float('nan'), 0.]) is None
  assert predict(before, [1., 0.]) < .1
  assert predict(before, [0., 1.]) > .9
