import unittest,tempfile
from pathlib import Path
from unittest.mock import patch
from lastmile.workflow import core
class WorkflowTest(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.patch=patch.object(core,'WORKSPACE',Path(self.tmp.name));self.patch.start();self.id='case1-test'
  f=core.case_dir(self.id)/core.STAGES[0];f.mkdir(parents=True)
  core.write(f/'scene_facts.json',{'test':True});(f/'compiled_model.mjb').write_bytes(b'model');(f/'initial_snapshot.npz').write_bytes(b'snapshot')
  core.write(f.parent/'inputs/source_episode.json',{'source_index':14})
  core.write(f/'review_packet.json',{'files':{'scene_facts.json':core.digest(f/'scene_facts.json')},'compiled_model_sha256':core.digest(f/'compiled_model.mjb'),'initial_snapshot_sha256':core.digest(f/'initial_snapshot.npz'),'source_episode_sha256':core.digest(f.parent/'inputs/source_episode.json')})
  self.f=f;self.decision={'review_packet_sha256':core.digest(f/'review_packet.json'),'reviewer':'tester','decision':'approved','target_correct':True,'is_tabletop':True,'suitable_sides':['north'],'retain_case_type':'case1_distance'}
 def tearDown(self):self.patch.stop();self.tmp.cleanup()
 def test_safe_identifier(self):
  for x in ['../bad','/tmp/x','Case 1','x'*65]:
   with self.assertRaises(ValueError):core.case_dir(x)
 def test_pending_blocks(self):
  with self.assertRaises(ValueError):core.require_next_stage(self.id)
 def test_agent_advice_not_approval(self):
  core.write(self.f/'agent_review.json',{'decision':'approved','human_approved':True})
  with self.assertRaises(ValueError):core.require_review(self.id)
 def test_review_alone_not_physics(self):
  core.confirm_review(self.id,self.decision)
  with self.assertRaises(ValueError):core.require_next_stage(self.id)
 def test_rejected_cannot_advance(self):
  core.confirm_review(self.id,{**self.decision,'decision':'rejected'})
  with self.assertRaises(ValueError):core.require_review(self.id)
 def test_invalid_review(self):
  for updates in [{'target_correct':False},{'is_tabletop':False},{'suitable_sides':[]},{'reviewer':''},{'review_packet_sha256':'wrong'},{'decision':'pending'}]:
   with self.assertRaises(ValueError):core.confirm_review(self.id,{**self.decision,**updates})
 def test_tampered_facts(self):
  core.write(self.f/'scene_facts.json',{'test':False})
  with self.assertRaises(ValueError):core.review_packet_valid(self.id)
 def test_tampered_model(self):
  (self.f/'compiled_model.mjb').write_bytes(b'other')
  with self.assertRaises(ValueError):core.review_packet_valid(self.id)
 def test_tampered_snapshot(self):
  (self.f/'initial_snapshot.npz').write_bytes(b'other')
  with self.assertRaises(ValueError):core.review_packet_valid(self.id)
 def test_source_tamper(self):
  core.write(self.f.parent/'inputs/source_episode.json',{'source_index':1})
  with self.assertRaises(ValueError):core.review_packet_valid(self.id)
 def test_history_retained(self):
  core.confirm_review(self.id,self.decision);core.confirm_review(self.id,{**self.decision,'decision':'rejected'})
  self.assertEqual(len(list((self.f/'review_history').glob('*.json'))),1)
 def make_physics(self):
  folder=core.case_dir(self.id)/core.STAGES[1];core.write(folder/'trace.json',{'success':True})
  core.write(folder/'historical_revalidation.json',{'traces':[]})
  core.write(folder/'physics_evidence.json',{'files':{'trace.json':core.digest(folder/'trace.json')}})
  protocol=core.WORKSPACE/'configs/protocols/protocol_v1.json';core.write(protocol,{'version':1})
  core.write(folder/'summary.json',{'physics_verified':True,'fresh_A_witness':{'status':'verified_success'},'protocol_sha256':core.digest(protocol)})
  core.update(self.id,core.STAGES[1],physics_verified=True)
  core.confirm_review(self.id,self.decision)
  return folder
 def test_gate_success(self):
  self.make_physics();self.assertTrue(core.require_next_stage(self.id))
 def test_physics_tamper(self):
  folder=self.make_physics();core.write(folder/'trace.json',{'success':False})
  with self.assertRaises(ValueError):core.require_next_stage(self.id)
 def test_protocol_tamper(self):
  self.make_physics();core.write(core.WORKSPACE/'configs/protocols/protocol_v1.json',{'version':2})
  with self.assertRaises(ValueError):core.require_next_stage(self.id)
if __name__=='__main__':unittest.main()
