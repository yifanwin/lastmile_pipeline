import json
from pathlib import Path
import unittest
import numpy as np
from lastmile.protocol import unpack_action, waypoint_action, torso_joints, arm_only_config, to_base_frame
from lastmile.validation import validate_trace, select_grasps

W = Path(__file__).resolve().parents[2]
P = json.loads((W/'configs/protocols/protocol_v1.json').read_text())

class ProtocolTests(unittest.TestCase):
    def action(self):
        a = np.zeros(20); a[10] = a[18] = -.05
        return a
    def test_dimensions(self):
        self.assertEqual(sum(v.size for v in unpack_action(self.action()).values()),20)
    def test_bad_dimensions(self):
        for a in [np.zeros(19),np.zeros((20,1)),np.full(20,np.nan)]:
            with self.assertRaises(ValueError): unpack_action(a)
    def test_pick_freezes_base(self):
        a = self.action(); a[0] = .01
        with self.assertRaises(ValueError): unpack_action(a,phase='pick')
    def test_navigation_allows_base(self):
        a = self.action(); a[0] = .01
        self.assertEqual(unpack_action(a)['base'][0],.01)
    def test_gripper_limits(self):
        a = self.action(); a[10] = 1
        with self.assertRaises(ValueError): unpack_action(a)
    def test_h_is_angle_not_height(self):
        np.testing.assert_allclose(torso_joints(.3),[0,.3,-.6,.3,0,0])
        for h in [-.1,.739,np.nan]:
            with self.assertRaises(ValueError): torso_joints(h)
    def test_absolute_to_relative(self):
        a = waypoint_action(np.ones(7), np.ones(7)*.9, 'right', .3)
        np.testing.assert_allclose(a[11:18], .1)
        np.testing.assert_allclose(a[3:10], 0)
        self.assertEqual(a[19],.3)
    def test_arm_only_locks(self):
        names = ['base_x','base_y','base_theta']+[f'torso_{i}' for i in range(6)]+[f'{s}_arm_{i}' for s in ('right','left') for i in range(7)]
        template = {'robot_cfg':{'kinematics':{'cspace':{'joint_names':names,'retract_config':list(range(23))},'collision_link_names':['base','link_left_arm_1','link_right_arm_1']}}}
        for side in ('left','right'):
            c = arm_only_config(template,side,.3,np.ones(7),[.1,.2])['robot_cfg']['kinematics']
            self.assertEqual(c['cspace']['joint_names'],[f'{side}_arm_{i}' for i in range(7)])
            self.assertEqual(c['lock_joints']['torso_2'],-.6)
            self.assertEqual(c['lock_joints']['base_x'],0)
            self.assertEqual(c['lock_joints']['head_1'],.2)
            self.assertEqual(len(c['collision_link_names']),3)
        self.assertEqual(len(template['robot_cfg']['kinematics']['cspace']['joint_names']),23)
    def test_frame_transform(self):
        b=np.eye(4);b[:3,3]=[2,3,0];w=np.eye(4);w[:3,3]=[3,4,1]
        np.testing.assert_allclose(to_base_frame(w,b)[:3,3],[1,1,1])
    def test_handle_fail_closed(self):
        with self.assertRaises(ValueError): select_grasps([0,1], [], 'handle')
        self.assertEqual(select_grasps([0,1,2],[1],'handle'),[1])

class TraceTests(unittest.TestCase):
    def fixture(self):
        initial={'base':[0,0,0],'head':[0,0],'idle_arm':[0]*7,'height_m':.75,'target_id':'target','selected_fingers':[1,2]}
        rows=[{'time_s':i*.004,'target_id':'target','base':[0,0,0],'torso':[0]*6,'head':[0,0],'idle_arm':[0]*7,'h':0,'illegal':[],
               'contact_sampler_version':'force-aware-v2','finger_normal_forces_N':{1:1.,2:1.},'height_m':.81,'support_contact':False,'selected_fingers':[1,2],'relative_pose':np.eye(4).tolist(),'upright_rad':0,'handle_fingers':[1,2],'forbidden_part_contact':False,'pregrasp_motion_violation':False}
              for i in range(501)]
        return rows, initial
    def verdict(self, rows, initial, handle=False):
        return validate_trace(rows,initial,P['thresholds'],handle=handle)
    def test_success(self):
        r,i=self.fixture();self.assertEqual(self.verdict(r,i)['status'],'verified_success')
    def test_handle_success(self):
        r,i=self.fixture();self.assertEqual(self.verdict(r,i,True)['status'],'verified_success')
    def test_hold_short(self):
        r,i=self.fixture();self.assertNotEqual(self.verdict(r[:-1],i)['status'],'verified_success')
    def test_illegal_early_contact(self):
        r,i=self.fixture();r[0]['illegal']=['wall'];self.assertEqual(self.verdict(r,i)['reason'],'illegal_collision')
    def test_early_base_drift(self):
        r,i=self.fixture();r[0]['base'][0]=.003;self.assertEqual(self.verdict(r,i)['reason'],'base_drift')
    def test_torso_protocol(self):
        r,i=self.fixture();r[0]['torso'][0]=.01;self.assertEqual(self.verdict(r,i)['reason'],'torso_protocol')
    def test_support_and_single_finger(self):
        for field,value in [('support_contact',True),('selected_fingers',[1])]:
            r,i=self.fixture();r[-1][field]=value;self.assertNotEqual(self.verdict(r,i)['status'],'verified_success')
    def test_zero_force_is_not_contact(self):
        r,i=self.fixture();r[-1]['finger_normal_forces_N'][1]=0.
        self.assertEqual(self.verdict(r,i)['reason'],'invalid_contact_forces')
    def test_old_distance_sampler_excluded(self):
        r,i=self.fixture();r[0]['contact_sampler_version']='distance-only-v1'
        self.assertEqual(self.verdict(r,i)['reason'],'unsupported_contact_sampler')
    def test_handle_push_before_bilateral_grasp_fails(self):
        r,i=self.fixture();r[0]['pregrasp_motion_violation']=True
        self.assertEqual(self.verdict(r,i,True)['reason'],'handle_semantics')
    def test_missing_trace(self):
        r,i=self.fixture();del r[0]['head'];self.assertEqual(self.verdict(r,i)['status'],'infrastructure_error')
    def test_slip(self):
        r,i=self.fixture();r[-1]['relative_pose'][0][3]=.02;self.assertNotEqual(self.verdict(r,i)['status'],'verified_success')
    def test_no_handle_body_contact(self):
        r,i=self.fixture();r[0]['forbidden_part_contact']=True;self.assertEqual(self.verdict(r,i,True)['reason'],'handle_semantics')
    def test_missing_handle_evidence(self):
        r,i=self.fixture();del r[-1]['handle_fingers'];self.assertNotEqual(self.verdict(r,i,True)['status'],'verified_success')
    def test_sampling_gap(self):
        r,i=self.fixture();del r[20];self.assertEqual(self.verdict(r,i)['reason'],'trace_gap')
    def test_wrong_target(self):
        r,i=self.fixture();r[0]['target_id']='other';self.assertEqual(self.verdict(r,i)['reason'],'wrong_target')
    def test_invalid_rotation(self):
        r,i=self.fixture();r[-1]['relative_pose'][0][0]=2;self.assertEqual(self.verdict(r,i)['status'],'infrastructure_error')


class AdmissionTests(unittest.TestCase):
    def test_empty_white_list(self):
        from lastmile.admission import admit_asset
        self.assertEqual(admit_asset('a','ordinary',[],'hash',P)['status'],'pending_physics')
    def test_handle_not_confirmed(self):
        from lastmile.admission import admit_asset
        self.assertEqual(admit_asset('a','handle',[],'hash',P)['status'],'pending_annotation')
    def test_boolean_success_is_not_evidence(self):
        from lastmile.admission import admit_asset
        trials=[{'kind':'nominal' if i<5 else 'perturbed','trial_id':i,'protocol_sha256':'hash','snapshot_restored':True,'success':True} for i in range(25)]
        self.assertEqual(admit_asset('a','ordinary',trials,'hash',P)['status'],'infrastructure_error')

class PlannerReturnTests(unittest.TestCase):
    def test_failed_plan_nonempty_is_rejected(self):
        from types import SimpleNamespace
        from lastmile.protocol import successful_plan
        class Flag:
            def item(self):return False
        self.assertFalse(successful_plan(SimpleNamespace(success=Flag()),[[0]*7]))
    def test_empty_success_is_rejected(self):
        from types import SimpleNamespace
        from lastmile.protocol import successful_plan
        class Flag:
            def item(self):return True
        self.assertFalse(successful_plan(SimpleNamespace(success=Flag()),[]))


class PartRegionTests(unittest.TestCase):
    def test_contact_uses_actual_object_frame(self):
        from lastmile.parts import classify_contact
        pose=np.eye(4);pose[:3,3]=[2,3,4]
        allowed=[{'min':[.07,-.02,-.01],'max':[.10,.05,.01]}]
        self.assertEqual(classify_contact([2.08,3.01,4],pose,allowed,[],.001),'handle')
        self.assertEqual(classify_contact([2.01,3.01,4],pose,allowed,[],.001),'unknown')
    def test_forbidden_region_wins_overlap(self):
        from lastmile.parts import classify_contact
        r=[{'min':[0,0,0],'max':[1,1,1]}]
        self.assertEqual(classify_contact([.5,.5,.5],np.eye(4),r,r,.001),'forbidden')
    def test_upright_axis_not_assumed_z(self):
        from lastmile.parts import upright_angle
        self.assertAlmostEqual(upright_angle(np.eye(4),[0,1,0]),np.pi/2)

if __name__ == '__main__': unittest.main()
