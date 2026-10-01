import unittest,tempfile,copy
from pathlib import Path
from unittest.mock import patch
from lastmile.workflow import core as c
from lastmile.workflow.final_review import CHECKS,validate_final_decision,validate_repeat_summary,require_final_acceptance
from lastmile.workflow.case_export import package_path,verify_export_bundle,assemble_case
class FinalDecisionTest(unittest.TestCase):
 def setUp(self):
  self.p={'case_id':'case1-test','review_id':'r1','qualification':{'physics_qualified':True}}
  self.d={'case_id':'case1-test','review_id':'r1','review_packet_sha256':'sha','actor':'human','decision':'accepted','reviewer':'test-reviewer',**{k:True for k in CHECKS}}
 def test_accept(self):self.assertEqual(validate_final_decision(self.d,self.p,'sha')['decision'],'accepted')
 def test_no_agent(self):
  with self.assertRaises(ValueError):validate_final_decision({**self.d,'actor':'agent'},self.p,'sha')
 def test_pending_blocked(self):
  with self.assertRaises(ValueError):validate_final_decision({**self.d,'decision':'pending'},self.p,'sha')
 def test_bindings(self):
  for changes in [{'case_id':'other'},{'review_id':'other'},{'review_packet_sha256':'stale'},{'reviewer':''}]:
   with self.subTest(changes=changes),self.assertRaises(ValueError):validate_final_decision({**self.d,**changes},self.p,'sha')
 def test_every_checkbox_required(self):
  for field in CHECKS:
   for v in [False,'true',1,None]:
    with self.subTest(field=field,value=v),self.assertRaises(ValueError):validate_final_decision({**self.d,field:v},self.p,'sha')
 def test_human_no_physics_override(self):
  with self.assertRaises(ValueError):validate_final_decision(self.d,{**self.p,'qualification':{'physics_qualified':False}},'sha')
 def test_reject_reconstruct(self):
  for decision in ['rejected','reconstruct']:self.assertEqual(validate_final_decision({**self.d,'decision':decision,**{k:False for k in CHECKS}},self.p,'sha')['decision'],decision)
 def summary(self):return {'trials_requested':3,'trials_completed':3,'strict_successes':3,'closed_loop_verified':True,'trials':[{'trial':i,'status':'verified_success','new_physical_execution':True,'arrival_used_without_reset':True} for i in range(3)]}
 def test_three_allowed(self):self.assertTrue(validate_repeat_summary(self.summary(),3)['physics_qualified'])
 def test_count_not_default_five(self):
  with self.assertRaises(ValueError):validate_repeat_summary(self.summary(),5)
 def test_failure_blocks(self):
  s=self.summary();s['trials'][2]['status']='executed_failure'
  with self.assertRaises(ValueError):validate_repeat_summary(s,3)
 def test_incomplete_blocks(self):
  for k in ['strict_successes','trials_completed','trials_requested']:
   s=self.summary();s[k]=2
   with self.assertRaises(ValueError):validate_repeat_summary(s,3)
 def test_duplicate_blocks(self):
  s=self.summary();s['trials'][2]['trial']=0
  with self.assertRaises(ValueError):validate_repeat_summary(s,3)
 def test_not_new_or_reset_blocks(self):
  for key in ['new_physical_execution','arrival_used_without_reset']:
   s=self.summary();s['trials'][0][key]=False
   with self.assertRaises(ValueError):validate_repeat_summary(s,3)
 def test_package_path_safe(self):
  for path in ['/tmp/a','../a','.env','a/.git/config','a/../../secret']:
   with self.assertRaises(ValueError):package_path(path,'sha','model')
 def test_model_dedup(self):self.assertEqual(package_path('cases/a/compiled_model.mjb','model','model'),'assets/compiled_model.mjb')
 def test_missing_acceptance_blocks(self):
  with tempfile.TemporaryDirectory() as d,patch.object(c,'WORKSPACE',Path(d)):
   folder=Path(d)/'review';folder.mkdir()
   with patch('lastmile.workflow.final_review.require_review_packet',return_value=(folder,self.p,'sha')):
    with self.assertRaises(ValueError):require_final_acceptance('case1-test','r1')

class ExportFixtureTest(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.patcher=patch.object(c,'WORKSPACE',self.root);self.patcher.start();self.case=c.case_dir('case1-test');self.review=self.case/c.STAGES[5]/'reviews/r1';self.review.mkdir(parents=True);self.sources={}
  def add(path,data=None):
   path.parent.mkdir(parents=True,exist_ok=True)
   if isinstance(data,dict):c.write(path,data)
   else:path.write_bytes(data or b'fixture')
   self.sources[str(path.relative_to(self.root))]=c.digest(path)
  self.add=add;first=self.case/c.STAGES[0];add(first/'compiled_model.mjb',b'fake-model');c.write(first/'review_packet.json',{'compiled_model_sha256':c.digest(first/'compiled_model.mjb')});add(first/'initial_snapshot.npz')
  construction=self.case/c.STAGES[3]/'runs/c1';add(construction/'case_initial_snapshot.npz');c.write(construction/'case_config.json',{})
  root=self.case/c.STAGES[4]/'runs/run1';self.root5=root
  c.write(root/'summary.json',{'trials':[{'trial':i,'status':'verified_success','pick_backend_result':{'seed':i}} for i in range(3)]})
  for i in range(3):
   for name in ['navigation_trace.json','pick/torso_adjust_trace.json','pick/grasp_0794_trace.json','actual_arrival_snapshot.npz','video_states.npz']:add(root/f'trial_{i:02d}'/name)
  add(root/'delivery/case1_r_closed_loop.mp4');add(construction/'reach_certificate.json',{'bound':1.2});add(self.root/'configs/protocols/protocol_v1.json',{'version':1});add(self.case/c.STAGES[2]/'runs/coarse_v1/station_map.png')
  asset_xml=self.root/'asset.xml';asset_xml.write_bytes(b'<fixture/>');grasp=self.root/'grasp.npz';grasp.write_bytes(b'fixture-grasp');c.write(self.case/'inputs/asset.json',{'asset_xml':str(asset_xml),'asset_xml_sha256':c.digest(asset_xml),'grasp_path':str(grasp),'grasp_sha256':c.digest(grasp)})
  c.write(self.review/'case_draft.json',{'case_id':'case1-test','status':'draft_pending_human_acceptance'})
  self.packet={'case_id':'case1-test','review_id':'r1','closed_loop_run_id':'run1','construction_run_id':'c1','qualification':{'physics_qualified':True},'source_files':self.sources,'review_files':{'case_draft.json':c.digest(self.review/'case_draft.json')}};c.write(self.review/'review_packet.json',self.packet);self.sha=c.digest(self.review/'review_packet.json')
 def tearDown(self):self.patcher.stop();self.tmp.cleanup()
 def test_draft_round_trip(self):
  with patch('lastmile.workflow.case_export.require_review_packet',return_value=(self.review,self.packet,self.sha)):
   receipt=assemble_case('case1-test','r1','export1',True)
   folder=self.root/receipt['bundle_path'];v=verify_export_bundle(folder);self.assertFalse(v['dataset_admitted']);self.assertTrue((self.root/receipt['zip_path']).exists())
   (folder/'assets/compiled_model.mjb').write_bytes(b'tamper')
   with self.assertRaises(ValueError):verify_export_bundle(folder)
 def test_accepted_round_trip_fixture_only(self):
  decision={'case_id':'case1-test','review_id':'r1','review_packet_sha256':self.sha,'actor':'human','decision':'accepted','reviewer':'unit-test-only',**{k:True for k in CHECKS}}
  with patch('lastmile.workflow.case_export.require_final_acceptance',return_value=(self.review,self.packet,decision)):
   receipt=assemble_case('case1-test','r1','export1',False);self.assertTrue(verify_export_bundle(self.root/receipt['bundle_path'])['dataset_admitted'])
 def test_no_overwrite(self):
  with patch('lastmile.workflow.case_export.require_review_packet',return_value=(self.review,self.packet,self.sha)):
   assemble_case('case1-test','r1','same',True)
   with self.assertRaises(ValueError):assemble_case('case1-test','r1','same',True)
if __name__=='__main__':unittest.main()
