import unittest
import numpy as np
from lastmile.navigation import astar_grid,simplify_grid_path
from lastmile.workflow.closed_loop import validate_navigation_rows
from lastmile.workflow.hard_start import snapshot_base_only
from types import SimpleNamespace
class NavigationTest(unittest.TestCase):
 def test_astar_empty(self):
  free=np.ones((4,4),bool);r=astar_grid(free,[0,0],[3,3]);self.assertEqual(tuple(r[-1]),(3,3))
 def test_astar_blocked(self):
  free=np.ones((4,4),bool);free[0,0]=False
  with self.assertRaises(ValueError):astar_grid(free,[0,0],[3,3])
 def test_no_diagonal_corner_cut(self):
  free=np.eye(2,dtype=bool)
  with self.assertRaises(ValueError):astar_grid(free,[0,0],[1,1])
 def test_astar_detour(self):
  free=np.ones((5,5),bool);free[2,:4]=False;r=astar_grid(free,[0,0],[4,0]);self.assertTrue(any(p[1]==4 for p in r))
 def test_simplify_straight(self):
  p=np.array([[0,0],[1,1],[2,2]],float);r=simplify_grid_path(p,np.ones((3,3),bool),np.zeros(2),1);self.assertEqual(len(r),2)
 def test_simplify_retains_detour(self):
  free=np.ones((5,5),bool);free[2,:4]=False;path=astar_grid(free,[0,0],[4,0]);r=simplify_grid_path(path,free,np.zeros(2),1);self.assertGreater(len(r),2)
 def data(self):
  initial={'time_s':0.,'head':[0,0],'left_arm':[0]*7,'right_arm':[0]*7,'target_pose':np.eye(4).tolist()}
  row={'time_s':.004,'illegal':[],'forbidden_navigation_contact':[],'torso':[0]*6,'head':[0,0],'selected_arm_q':[0]*7,'idle_arm':[0]*7,'target_pose':np.eye(4).tolist()}
  return initial,row
 def test_nav_good(self):
  initial,row=self.data();self.assertIsNone(validate_navigation_rows([row],initial,{'torso_error_rad':.002}))
 def test_nav_empty(self):
  initial,row=self.data();self.assertIsNotNone(validate_navigation_rows([],initial,{'torso_error_rad':.002}))
 def test_nav_time_gap(self):
  initial,row=self.data();row['time_s']=.01;self.assertIsNotNone(validate_navigation_rows([row],initial,{'torso_error_rad':.002}))
 def test_nav_collision(self):
  initial,row=self.data();row['forbidden_navigation_contact']=['table'];self.assertEqual(validate_navigation_rows([row],initial,{'torso_error_rad':.002}),'navigation_collision')
 def test_nav_torso(self):
  initial,row=self.data();row['torso'][0]=.0021;self.assertEqual(validate_navigation_rows([row],initial,{'torso_error_rad':.002}),'navigation_torso')
 def test_nav_target_move(self):
  initial,row=self.data();row['target_pose'][0][3]=.003;self.assertEqual(validate_navigation_rows([row],initial,{'torso_error_rad':.002}),'navigation_target_translation')
 def test_placement_only_base(self):
  sim=SimpleNamespace(model=SimpleNamespace(nq=5),qpos_adr={'base_x':0,'base_y':1,'base_theta':2},data=SimpleNamespace(qpos=np.array([1,1,1,0,0]),qvel=np.zeros(4)))
  self.assertTrue(snapshot_base_only(sim,np.zeros(5),np.zeros(4))['only_base_qpos_changed']);sim.data.qpos[4]=1
  with self.assertRaises(ValueError):snapshot_base_only(sim,np.zeros(5),np.zeros(4))
if __name__=='__main__':unittest.main()

class ClosedLoopCliTest(unittest.TestCase):
 def test_closed_loop_arguments(self):
  from lastmile.workflow.cli import main
  from unittest.mock import patch
  from contextlib import redirect_stdout
  import io
  args=['lastmile','closed-loop','--case-id','case1-test','--construction-run-id','c1','--run-id','r1','--trials','3']
  with patch('sys.argv',args),patch('lastmile.workflow.closed_loop.run_closed_loop',return_value={}) as run,redirect_stdout(io.StringIO()):main()
  run.assert_called_once_with('case1-test','c1','r1',3)
