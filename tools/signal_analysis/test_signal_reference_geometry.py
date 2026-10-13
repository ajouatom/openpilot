import math
from types import SimpleNamespace as NS
import pytest
from tools.signal_analysis.signal_reference_geometry import project_reference,rank_candidates


def model():
  curves=[NS(x=[0.,30.,70.,100.],y=[y]*4,z=[1.5]*4) for y in (-5.4,-1.8,1.8,5.4)]
  return NS(laneLines=curves,laneLineProbs=[.9]*4,laneLineStds=[.1]*4,
            roadEdges=[curves[0],curves[3]],roadEdgeStds=[.1]*2)


@pytest.mark.parametrize('distance',[30.,70.])
def test_reference_projects_above_road_not_below(distance):
  p=project_reference(model(),[0.,0.,0.],distance=distance)
  assert p.reliable and p.xyz==(distance,0.,-3.5)
  assert p.pixel==pytest.approx((672.,380.-1141.5*3.5/distance))


@pytest.mark.parametrize('distance',[101.,-1.,float('nan')])
def test_never_extrapolate_or_use_invalid_distance(distance):
  assert project_reference(model(),[0.,0.,0.],distance=distance) is None


def test_uncertain_geometry_stays_diagnostic_even_when_a_point_can_be_drawn():
  m=model();m.laneLineProbs=[.1]*4;m.roadEdgeStds=[3.,3.]
  assert not project_reference(m,[0.,0.,0.]).reliable
  assert not project_reference(m,[0.,0.,0.],source='edges').reliable


@pytest.mark.parametrize('bad',['nonmonotonic','nan','wrong_shape','crossed'])
def test_invalid_curves_have_no_reference(bad):
  m=model();c=m.laneLines[1]
  if bad=='nonmonotonic':c.x=[0.,70.,30.,100.]
  if bad=='nan':c.z[2]=math.nan
  if bad=='wrong_shape':c.y=[0.]
  if bad=='crossed':c.y=[5.]*4
  assert project_reference(m,[0.,0.,0.]) is None


def test_ranking_excludes_stale_boxes_and_separates_horizon_without_color_permission():
  p=project_reference(model(),[0.,0.,0.]);x,y=p.pixel
  tracks=[dict(id=1,box=[x-10,y-5,x+10,y+5],age=0),
          dict(id=2,box=[662,375,682,385],age=0),
          dict(id=3,box=[x-10,y-5,x+10,y+5],age=.1)]
  r=rank_candidates(p,tracks,[0.,0.,0.])
  assert [t['id'] for t in r]==[1,2]
  assert r[0]['reference_plane_offset']==pytest.approx(0.)
  assert r[0]['near_reference'] and not r[1]['near_reference']
  assert 'signal_distance' not in r[0] and 'color' not in r[0]


def test_same_pixel_does_not_supply_metric_range():
  tracks=[dict(id=1,box=[752,295,772,305],age=0)]
  scores=[rank_candidates(project_reference(model(),[0.,0.,0.],distance=d),tracks,[0.,0.,0.])[0]['reference_plane_offset'] for d in (30.,70.)]
  assert scores[0]!=scores[1]  # depends on the assumed plane, not a measured range
