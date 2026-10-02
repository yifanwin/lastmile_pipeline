"""用真实上游缓冲策略、假的推理网络验证服务接口与逐试验 reset。"""
import importlib
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


class VLAServerTest(unittest.TestCase):
    def test_local_service_never_uses_proxy_and_waits_for_model(self):
        import msgpack_numpy
        import websockets.sync.client
        from lastmile.workflow.vla_station import PolicyService
        from lastmile.workflow.core import digest
        from unittest.mock import MagicMock
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder/'config.yaml').write_text('test\n')
            metadata = {'checkpoint_path': str(folder.resolve()), 'config_sha256': digest(folder/'config.yaml'),
                'action_dim': 20, 'state_dim': 22, 'camera_names': ['wrist_camera_r', 'head_camera', 'wrist_camera_l'],
                'clamp_gripper': False, 'execute_horizon': 8, 'action_horizon': 16, 'seed': 0}
            connection = MagicMock()
            connection.recv.return_value = msgpack_numpy.packb(metadata)
            process = MagicMock()
            process.poll.return_value = None
            sock = MagicMock()
            sock.getsockname.return_value = ('127.0.0.1', 12345)
            config = {'seed': 0, 'server_start_timeout_s': 20, 'inference_timeout_s': 10}
            with patch('lastmile.workflow.vla_station.subprocess.Popen', return_value=process), \
                 patch('lastmile.workflow.vla_station.socket.socket', return_value=sock), \
                 patch('lastmile.workflow.vla_station.time.sleep'), \
                 patch.object(websockets.sync.client, 'connect', side_effect=[ConnectionRefusedError(), connection]) as connect:
                service = PolicyService(folder, folder, config)
                self.assertEqual(connect.call_count, 2)
                self.assertIsNone(connect.call_args.kwargs['proxy'])
                self.assertTrue(connect.call_args.args[0].startswith('ws://127.0.0.1:'))
                service.close()
                process.terminate.assert_called_once()

    def test_real_policy_buffer_reset_and_evidence(self):
        sys.path.insert(0, str(ROOT.parent/'MolmoBot/MolmoBot'))
        from olmo.eval import real_robot_molmobot_rby1_multitask as upstream
        from olmo.eval.real_robot_molmobot_rby1_door import MolmoBotRBY1DoorPolicy
        from lastmile.vla_server import main
        calls, captured = [], []
        class Agent:
            action_dim, action_horizon = 20, 16
            def get_action_chunk(self, **kw):
                calls.append(kw)
                return np.tile(np.arange(20, dtype=np.float32), (16, 1))
        def prepare(policy):
            if policy.agent is None:
                policy.agent = Agent()
        class Server:
            def __init__(self, policies, *args, **kwargs):
                captured.append((policies[0], kwargs))
            def serve_forever(self):
                captured[0][0].prepare_model()
        fake_torch = SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: True, manual_seed_all=lambda n: None), manual_seed=lambda n: None)
        fake_server = SimpleNamespace(WebsocketPolicyServer=Server)
        config_cls = upstream.MolmoBotRBY1PickPnPPolicyConfig
        def config(**kwargs):
            return config_cls(**kwargs, cameras_to_warp=[])
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'config.yaml').write_text('fake checkpoint for interface tests\n')
            argv = ['vla-server', '--checkpoint-path', tmp, '--port', '12345', '--evidence-dir', str(root/'trials')]
            with patch.object(sys, 'argv', argv), patch.dict(sys.modules, {'torch': fake_torch, 'olmo.eval.websocket_server': fake_server}), \
                 patch.object(MolmoBotRBY1DoorPolicy, 'prepare_model', prepare), patch.object(upstream, 'MolmoBotRBY1PickPnPPolicyConfig', config):
                main()
                policy, settings = captured[0]
                self.assertEqual(settings['host'], '127.0.0.1')
                self.assertFalse(settings['metadata']['clamp_gripper'])
                obs = {n: np.zeros((2, 2, 3), dtype=np.uint8) for n in policy.camera_names}
                obs.update(qpos={'base': np.zeros(3), 'left_arm': np.zeros(7), 'left_gripper': np.zeros(1),
                    'right_arm': np.zeros(7), 'right_gripper': np.zeros(1), 'torso': np.array([0., .2, -.4, .2, 0., 0.])},
                    task='pick up the cup', reset=True, _evidence_key='A_attempt_000')
                action = policy.get_action(obs)
                self.assertEqual(action['torso'].tolist(), [19.])
                self.assertEqual(len(calls), 1)
                self.assertEqual(calls[0]['state'].shape, (22,))
                np.testing.assert_allclose(calls[0]['state'][-3:], [.2, -.4, .2])
                obs['reset'] = False
                for _ in range(7):
                    policy.get_action(obs)
                self.assertEqual(len(calls), 1)
                policy.get_action(obs)
                self.assertEqual(len(calls), 2)
                obs.update(reset=True, _evidence_key='P002_attempt_000')
                policy.get_action(obs)
                self.assertEqual(len(calls), 3)
                self.assertEqual(policy.buffer_index, 1)
                self.assertTrue((root/'trials/P002_attempt_000/model_inputs/chunk_000/predicted_actions.npz').exists())
                obs.update(reset=False, _evidence_key='P005_attempt_000')
                with self.assertRaises(ValueError):
                    policy.get_action(obs)


if __name__ == '__main__':
    unittest.main()
