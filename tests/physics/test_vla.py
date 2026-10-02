import copy
import json
import gzip
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from lastmile.vla import CAMERAS, VLAValidator, action_vector, apply_vla_action, observation, select_xy_starts
from lastmile.workflow.vla_station import require_frozen_match, audit_trial

ROOT = Path(__file__).resolve().parents[2]
THRESHOLDS = json.loads((ROOT/'configs/protocols/protocol_v1.json').read_text())['thresholds']


class VLATest(unittest.TestCase):
    def initial(self):
        return {'time_s': 0., 'height_m': .8, 'head': [0., .4], 'target_id': 'cup',
                'arms': {s: {'expected_fingers': ids} for s, ids in [('left', [1, 2]), ('right', [3, 4])]}}

    def row(self, tick, side='left'):
        initial = self.initial()
        return {'time_s': tick*.004, 'height_m': .86, 'target_id': 'cup', 'head': [0., .4], 'base': [tick*.002, 0., .2],
            'torso': [0., .2, -.4, .2, 0., 0.], 'illegal': [], 'illegal_contacts': [], 'robot_contacts': [], 'support_contact': False, 'joint_limits_ok': True,
            'contact_sampler_version': 'vla-force-aware-v1', 'arms': {s: {**initial['arms'][s],
                'relative_pose': np.eye(4).tolist(), 'finger_normal_forces_N': {i: 2. for i in initial['arms'][s]['expected_fingers']} if s == side else {}} for s in ('left', 'right')}}

    def run_rows(self, modify=lambda row: None, side='left', count=501):
        validator = VLAValidator(self.initial(), THRESHOLDS)
        for tick in range(1, count+1):
            row = self.row(tick, side)
            modify(row)
            if validator.feed(row):
                break
        return validator.finish()

    def test_left_and_right_success_with_base_motion(self):
        for side in ('left', 'right'):
            result = self.run_rows(side=side)
            self.assertEqual(result['status'], 'verified_success')
            self.assertEqual(result['selected_arm'], side)

    def test_no_early_success(self):
        self.assertEqual(self.run_rows(count=500)['status'], 'executed_failure')

    def test_single_finger_failure(self):
        self.assertEqual(self.run_rows(lambda r: r['arms']['left']['finger_normal_forces_N'].pop(2))['status'], 'executed_failure')

    def test_no_force_failure(self):
        self.assertEqual(self.run_rows(lambda r: r['arms']['left'].update(finger_normal_forces_N={}))['status'], 'executed_failure')
        self.assertEqual(self.run_rows(lambda r: r['arms']['left'].update(finger_normal_forces_N={1: 0., 2: 1.}))['reason'], 'invalid_contact_forces')

    def test_support_lift_and_slip_failures(self):
        for change in (lambda r: r.update(support_contact=True), lambda r: r.update(height_m=.82),
                       lambda r: r['arms']['left']['relative_pose'][0].__setitem__(3, r['time_s']*.1)):
            self.assertEqual(self.run_rows(change)['status'], 'executed_failure')

    def test_continuity_and_same_gripper(self):
        def drop(r):
            if abs(r['time_s']-.8) < .001:
                r['arms']['left']['finger_normal_forces_N'] = {}
        self.assertEqual(self.run_rows(drop)['status'], 'executed_failure')
        v = VLAValidator(self.initial(), THRESHOLDS)
        for tick in range(1, 502):
            v.feed(self.row(tick, 'left' if tick < 250 else 'right'))
        self.assertEqual(v.finish()['status'], 'executed_failure')

    def test_collision_head_torso_and_limits(self):
        changes = [(lambda r: r.update(illegal=['environment:wall']), 'illegal_collision'),
                   (lambda r: r.update(head=[.1, .4]), 'head_protocol'),
                   (lambda r: r.update(torso=[0., .2, -.3, .2, 0., 0.]), 'torso_linkage'),
                   (lambda r: r.update(joint_limits_ok=False), 'joint_limits')]
        for change, reason in changes:
            self.assertEqual(self.run_rows(change)['reason'], reason)

    def test_warn_torso_feedback_preserves_grasp_and_safety(self):
        for side in ('left', 'right'):
            v = VLAValidator(self.initial(), THRESHOLDS, 'warn')
            for tick in range(1, 502):
                row = self.row(tick, side)
                row['torso'][0] = .1
                verdict = v.feed(row)
            self.assertEqual(verdict['status'], 'verified_success')
            self.assertEqual(verdict['torso_feedback_warnings']['torso_linkage']['samples'], 501)
            self.assertEqual(verdict['selected_arm'], side)
        for change, reason in ((lambda r: r.update(illegal=['wall']), 'illegal_collision'),
                               (lambda r: r.update(joint_limits_ok=False), 'joint_limits'),
                               (lambda r: r.update(head=[.1, .4]), 'head_protocol')):
            v = VLAValidator(self.initial(), THRESHOLDS, 'warn')
            row = self.row(1)
            row['torso'][0] = .1
            change(row)
            self.assertEqual(v.feed(row)['reason'], reason)

    def test_warn_range_no_grasp_continues_and_no_warning_leak(self):
        v = VLAValidator(self.initial(), THRESHOLDS, 'warn')
        for tick in range(1, 601):
            row = self.row(tick)
            row['torso'] = [0., .75, -1.5, .75, 0., 0.]
            row['support_contact'] = True
            self.assertIsNone(v.feed(row))
        result = v.finish()
        self.assertEqual(result['reason'], 'horizon_exhausted_lift_contact_hold_or_slip')
        self.assertEqual(result['torso_feedback_warnings']['torso_range']['samples'], 600)
        self.assertFalse(VLAValidator(self.initial(), THRESHOLDS, 'warn').torso_warnings)
        with self.assertRaises(ValueError):
            VLAValidator(self.initial(), THRESHOLDS, 'ignore')

    def test_warn_replay_and_command_evidence_tampering(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            initial = self.initial()
            (folder/'initial.json').write_text(json.dumps(initial))
            v = VLAValidator(initial, THRESHOLDS, 'warn', 'warn_minor')
            with gzip.open(folder/'trace.jsonl.gz', 'wt') as f:
                for tick in range(1, 502):
                    row = self.collision_row(tick)
                    row['torso'][0] = .1
                    f.write(json.dumps(row)+'\n')
                    v.feed(row)
            (folder/'result.json').write_text(json.dumps(v.finish()))
            actions = [apply_vla_action(self.fake_sim(), self.action())]
            (folder/'actions.json').write_text(json.dumps(actions))
            self.assertEqual(audit_trial(folder, THRESHOLDS, 'warn', 'warn_minor')['samples'], 501)
            actions[0]['applied_targets']['link1_act'] = .01
            (folder/'actions.json').write_text(json.dumps(actions))
            with self.assertRaisesRegex(ValueError, 'torso_command_linkage'):
                audit_trial(folder, THRESHOLDS, 'warn', 'warn_minor')

    def test_command_linkage_checked_before_any_dispatch(self):
        sim = self.fake_sim()
        sim.model.actuator_ctrlrange[sim.act['link3_act']] = [-.1, 0.]
        original = sim.data.ctrl.copy()
        with self.assertRaisesRegex(ValueError, 'torso_command_linkage'):
            apply_vla_action(sim, self.action())
        np.testing.assert_array_equal(sim.data.ctrl, original)
        valid = apply_vla_action(self.fake_sim(), self.action())
        self.assertTrue(valid['torso_command_verified'])
        self.assertEqual(valid['torso_command_error_rad'], 0.)

    def collision_row(self, tick, depth=.003):
        row = self.row(tick)
        row['illegal'] = ['environment:wall']
        row['illegal_contacts'] = [{'reason': 'environment:wall', 'penetration_m': depth,
                                    'geom1': 1, 'geom2': 2, 'normal_force_N': 2.}]
        row['robot_contacts'] = copy.deepcopy(row['illegal_contacts'])
        return row

    def test_minor_collision_warns_and_can_pick(self):
        for side in ('left', 'right'):
            v = VLAValidator(self.initial(), THRESHOLDS, 'warn', 'warn_minor', .02)
            for tick in range(1, 502):
                row = self.collision_row(tick)
                row['arms'] = self.row(tick, side)['arms']
                row['torso'][0] = .1
                verdict = v.feed(row)
            self.assertEqual(verdict['status'], 'verified_success')
            self.assertEqual(verdict['collision_warnings']['environment:wall']['samples'], 501)
            self.assertEqual(verdict['torso_feedback_warnings']['torso_linkage']['samples'], 501)
        self.assertFalse(VLAValidator(self.initial(), THRESHOLDS, 'warn', 'warn_minor').collision_warnings)

    def test_severe_depth_boundary_and_missing_evidence(self):
        for depth in (.02, .021):
            v = VLAValidator(self.initial(), THRESHOLDS, 'warn', 'warn_minor', .02)
            self.assertEqual(v.feed(self.collision_row(1, depth))['reason'], 'severe_penetration')
        row = self.collision_row(1, .03)
        row['illegal'] = []
        row['illegal_contacts'] = []
        self.assertEqual(VLAValidator(self.initial(), THRESHOLDS, 'warn', 'warn_minor').feed(row)['reason'], 'severe_penetration')
        for depth in (.0011, .019999):
            v = VLAValidator(self.initial(), THRESHOLDS, 'warn', 'warn_minor', .02)
            self.assertIsNone(v.feed(self.collision_row(1, depth)))
        for change in (lambda r: r.pop('illegal_contacts'), lambda r: r.update(illegal_contacts=[]),
                       lambda r: r['illegal_contacts'][0].update(penetration_m=float('nan'))):
            row = self.collision_row(1)
            change(row)
            v = VLAValidator(self.initial(), THRESHOLDS, 'warn', 'warn_minor', .02)
            self.assertEqual(v.feed(row)['status'], 'infrastructure_error')
        self.assertEqual(VLAValidator(self.initial(), THRESHOLDS).feed(self.collision_row(1))['reason'], 'illegal_collision')

    def test_minor_collision_does_not_replace_contact_hold_or_safety(self):
        v = VLAValidator(self.initial(), THRESHOLDS, 'warn', 'warn_minor')
        for tick in range(1, 502):
            row = self.collision_row(tick)
            row['support_contact'] = True
            self.assertIsNone(v.feed(row))
        self.assertEqual(v.finish()['status'], 'executed_failure')
        for field, value, reason in (('head', [.1, .4], 'head_protocol'), ('joint_limits_ok', False, 'joint_limits')):
            v = VLAValidator(self.initial(), THRESHOLDS, 'warn', 'warn_minor')
            row = self.collision_row(1)
            row[field] = value
            self.assertEqual(v.feed(row)['reason'], reason)

    def test_actual_torso_linkage_not_command_tracking(self):
        def change(r):
            h = min(.3, r['time_s']*.1)
            r['torso'] = [0., h, -2*h, h, 0., 0.]
        self.assertEqual(self.run_rows(change)['status'], 'verified_success')

    def test_missing_gap_nonfinite_and_wrong_target(self):
        for change in (lambda r: r.pop('arms'), lambda r: r.update(time_s=.1), lambda r: r.update(base=[np.nan, 0, 0])):
            self.assertEqual(self.run_rows(change)['status'], 'infrastructure_error')
        self.assertEqual(self.run_rows(lambda r: r.update(target_id='other'))['reason'], 'wrong_target')
        self.assertEqual(VLAValidator(self.initial(), THRESHOLDS).finish()['status'], 'infrastructure_error')

    def test_xy_coverage_and_outside_bound_retained(self):
        source = ROOT/'cases/case1-cup-001/03_station_map/runs/coarse_v1/candidates.json'
        selected = select_xy_starts(json.loads(source.read_text()))
        self.assertEqual(len(selected), 58)
        self.assertEqual(selected[0]['pose_id'], 'A')
        self.assertEqual(sum(r['geometry_status']=='initial_pose_collision' for r in selected), 23)
        self.assertEqual(sum(r['geometry_status']=='outside_contact_bound' for r in selected), 9)
        self.assertEqual(sum(r['geometry_status']!='initial_pose_collision' for r in selected), 35)
        self.assertTrue(all(r['is_A'] or r['yaw_offset_deg']==0 for r in selected))
        with self.assertRaises(ValueError):
            select_xy_starts(selected+[selected[0]])

    def fake_sim(self):
        names = [s+'_arm_'+str(i+1)+'_act' for s in ('left', 'right') for i in range(7)]
        names += ['left_finger_act', 'right_finger_act', 'base_x_act', 'base_y_act', 'base_theta_act']
        names += [f'link{i+1}_act' for i in range(6)] + ['head_0_act', 'head_1_act']
        act = {n: i for i, n in enumerate(names)}
        limits = np.tile([-1., 1.], (len(names), 1))
        for s in ('left', 'right'):
            limits[act[s+'_finger_act']] = [-.05, 0.]
        limits[act['link3_act']] = [-2., 0.]
        state = {'base': np.zeros(3), 'left_arm': np.zeros(7), 'right_arm': np.zeros(7), 'torso': np.zeros(6)}
        return SimpleNamespace(act=act, model=SimpleNamespace(actuator_ctrlrange=limits, actuator_ctrllimited=np.ones(len(names))),
             data=SimpleNamespace(ctrl=np.zeros(len(names))), group=lambda n: state[n].copy(),
             q=lambda n: np.array([-.05]), vla_head_reference=np.array([0., .4]))

    def action(self):
        return {'base': np.array([.1, .2, .3]), 'left_arm': np.full(7, .01), 'right_arm': np.full(7, -.01),
                'left_gripper': np.array([-100.]), 'right_gripper': np.array([100.]), 'torso': np.array([.3])}

    def test_20d_and_auditable_saturation(self):
        sim = self.fake_sim()
        result = apply_vla_action(sim, self.action())
        self.assertEqual(len(result['raw_action']), 20)
        self.assertEqual(result['applied_targets']['left_finger_act'], -.05)
        self.assertEqual(result['applied_targets']['right_finger_act'], 0.)
        self.assertEqual(result['applied_targets']['base_y_act'], .2)
        self.assertEqual(result['applied_targets']['link3_act'], -.6)
        self.assertIn('left_finger_act', result['limits'])
        action = self.action()
        action['torso'][0] = -.1
        self.assertIn('torso_height', apply_vla_action(sim, action)['limits'])

    def test_no_nan_or_bad_dimension(self):
        for value in (np.ones(6), np.full(7, np.nan), np.ones((7, 1))):
            action = self.action()
            action['left_arm'] = value
            with self.assertRaises(ValueError):
                apply_vla_action(self.fake_sim(), action)

    def test_observation_22d_three_rgb_only(self):
        images = {n: np.zeros((576, 1024, 3), dtype=np.uint8) for n in CAMERAS}
        obs = observation(self.fake_sim(), images, 'pick up the cup', reset=True)
        self.assertEqual(tuple(k for k in obs if k in CAMERAS), CAMERAS)
        self.assertEqual(sum(v.size if k != 'torso' else 3 for k, v in obs['qpos'].items()), 22)
        self.assertNotIn('object_image_points', obs)
        with self.assertRaises(ValueError):
            observation(self.fake_sim(), dict(reversed(list(images.items()))), 'pick')

    def test_resume_rejects_config_source_or_model_change(self):
        saved = {'config': {'seed': 0}, 'files': {'model.pt': 'x', 'snapshot': 'y'}}
        require_frozen_match(saved, copy.deepcopy(saved))
        for key in ('config', 'files'):
            changed = copy.deepcopy(saved)
            changed[key] = {}
            with self.assertRaises(ValueError):
                require_frozen_match(saved, changed)

    def test_no_validator_state_leak(self):
        self.assertEqual(self.run_rows()['status'], 'verified_success')
        self.assertEqual(self.run_rows(lambda r: r.update(support_contact=True))['status'], 'executed_failure')


if __name__ == '__main__':
    unittest.main()
