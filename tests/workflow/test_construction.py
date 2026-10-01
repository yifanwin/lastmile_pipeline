import unittest,tempfile,math
from pathlib import Path
from unittest.mock import patch
from lastmile.workflow import core as c
from lastmile.workflow.construction import same_pose,evaluate_original_start,validate_human_method,checked_file,seal_decision,require_constructed_candidate
class ConstructionTest(unittest.TestCase):
 def test_pose_wrap(self):self.assertTrue(same_pose([1,2,0],[1,2,2*math.pi]))
 def test_pose_translation(self):self.assertFalse(same_pose([1,2,0],[1.001,2,0]))
 def test_pose_bad(self):
  for b in [[1,2],[1,2,float('nan')],[1,2,float('inf')]]:self.assertFalse(same_pose(b,[1,2,0]))
 def witness(self,base,status):return {'base':base,'verdict':{'status':status},'witness_id':'T0'}
 def test_original_success_excludes(self):
  r=evaluate_original_start([1,2,0],[self.witness([1,2,0],'verified_success')]);self.assertTrue(r['screening_complete']);self.assertFalse(r['can_enter_step5']);self.assertFalse(r['candidate_selected'])
 def test_other_station_not_original(self):self.assertFalse(evaluate_original_start([1,2,0],[self.witness([3,2,0],'verified_success')])['screening_complete'])
 def test_failure_not_unreachable(self):self.assertFalse(evaluate_original_start([1,2,0],[self.witness([1,2,0],'executed_failure')])['screening_complete'])
 def test_empty_unknown(self):self.assertFalse(evaluate_original_start([1,2,0],[])['can_enter_step5'])
 def choice(self):return dict(case_id='case1-test',actor='human',decision='method_selected',method='original_only',allow_scene_edits=[])
 def test_valid_choice(self):self.assertEqual(validate_human_method(self.choice(),'case1-test')['method'],'original_only')
 def test_invalid_choice(self):
  for changes in [{'actor':'agent'},{'method':'robot_only'},{'case_id':'other'},{'allow_scene_edits':['base']},{'decision':'approved'}]:
   with self.subTest(changes=changes),self.assertRaises(ValueError):validate_human_method({**self.choice(),**changes},'case1-test')
 def test_checked_file(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);(p/'a').write_text('a');self.assertEqual(checked_file(p,'a',c.digest(p/'a')),p/'a')
   with self.assertRaises(ValueError):checked_file(p,'a','wrong')
   with self.assertRaises(ValueError):checked_file(p,'../outside','wrong')
 def test_decision_tamper_blocks(self):
  with tempfile.TemporaryDirectory() as d,patch.object(c,'WORKSPACE',Path(d)):
   folder=c.case_dir('case1-test')/c.STAGES[3];folder.mkdir(parents=True)
   for name in ['summary.json','human_choice.json','screening_registry.json']:c.write(folder/name,{})
   seal_decision('case1-test');c.write(folder/'summary.json',{'candidate_selected':True,'can_enter_step5':True})
   with self.assertRaisesRegex(ValueError,'证据已改变'):require_constructed_candidate('case1-test')
if __name__=='__main__':unittest.main()
