import unittest
import numpy as np
from lastmile.workflow.station_geometry import candidates,wrap_yaw,gaussian_scores
from lastmile.workflow.station_map import select_physics
class StationMapTest(unittest.TestCase):
 def config(self):return {'include_exact_A':True,'corner_extension_m':.15,'along_spacing_m':.3,'edge_offsets_m':[.32,.48,.64],'yaw_offsets_deg':[-20,0,20]}
 def test_approved_sides_only(self):
  rows=candidates(np.array([0,0,0]),np.array([1,1.5,1]),[0,.5,1],[0,1.2,-1.5],['north','west'],self.config())
  self.assertEqual(set(r['side'] for r in rows),{'north','west'});self.assertEqual(rows[0]['pose_id'],'A')
  self.assertEqual(len(set(r['pose_id'] for r in rows)),len(rows))
 def test_deterministic(self):
  args=(np.array([0,0,0]),np.array([1,1.5,1]),[0,.5,1],[0,1.2,-1.5],['south'],self.config())
  self.assertEqual(candidates(*args),candidates(*args))
 def test_wrapped_angles(self):
  self.assertAlmostEqual(wrap_yaw(4*np.pi),0);self.assertTrue(-np.pi<=wrap_yaw(17)<=np.pi)
 def test_unknown_interpolation_not_zero(self):
  scores,count=gaussian_scores([[0,0]],[1],np.array([[0,0],[2,2]]))
  self.assertEqual(scores[0],1);self.assertTrue(np.isnan(scores[1]));self.assertEqual(count.tolist(),[1,0])
 def test_empty_labels(self):
  scores,count=gaussian_scores([],[],np.zeros((3,2)));self.assertTrue(np.isnan(scores).all());self.assertEqual(count.tolist(),[0,0,0])
 def test_interpolation_binary_symmetry(self):
  score,_=gaussian_scores([[-.1,0],[.1,0]],[0,1],np.array([[0,0]]));self.assertAlmostEqual(score[0],.5)
 def test_selection_A_first(self):
  rows=[{'pose_id':'A','base':[0,0,0],'geometry_status':'eligible','target_distance_m':.5},{'pose_id':'P001','base':[.3,0,0],'geometry_status':'eligible','target_distance_m':.8}]
  plan=[{'pose_id':'A','side':'left','h':.738,'status':'full_path_planned'},{'pose_id':'P001','side':'left','h':.738,'status':'full_path_planned'}]
  chosen=select_physics(rows,plan,4);self.assertEqual(chosen[0]['pose_id'],'A');self.assertEqual(chosen[1]['pose_id'],'P001')
if __name__=='__main__':unittest.main()
