"""在 MolmoBot 虚拟环境运行的本机推理服务，不修改上游源码。"""
import argparse
import hashlib
import json
import logging
import os
from pathlib import Path
import re
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--checkpoint-path', required=True)
    p.add_argument('--port', type=int, required=True)
    p.add_argument('--evidence-dir', required=True)
    p.add_argument('--seed', type=int, default=0)
    args = p.parse_args()
    os.environ.setdefault('HF_HUB_OFFLINE', '1')
    os.environ.setdefault('TRANSFORMERS_OFFLINE', '1')
    import torch
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA 不可用，禁止默默回退 CPU')
    from olmo.eval.real_robot_molmobot_rby1_multitask import MolmoBotRBY1MultitaskPolicy, MolmoBotRBY1PickPnPPolicyConfig
    from olmo.eval.websocket_server import WebsocketPolicyServer
    from PIL import Image

    class EvidencePolicy(MolmoBotRBY1MultitaskPolicy):
        def get_action(self, obs):
            return self.model_output_to_action(self.inference_model(self.obs_to_model_input(obs)))

        def reset(self):
            super().reset()
            np.random.seed(args.seed)
            torch.manual_seed(args.seed)
            torch.cuda.manual_seed_all(args.seed)
            self.evidence_key = None
            self.chunk = 0

        def prepare_model(self):
            super().prepare_model()
            if getattr(self, '_evidence_wrapped', False):
                return
            if (self.agent.action_dim, self.agent.action_horizon) != (20, 16):
                raise ValueError('checkpoint 的动作维度／horizon 不匹配')
            original = self.agent.get_action_chunk
            def record_chunk(*, images, task_description, state):
                prediction = original(images=images, task_description=task_description, state=state)
                if prediction.shape != (16, 20) or not np.isfinite(prediction).all():
                    raise ValueError('模型动作 chunk 非有限值或维度错误')
                dest = Path(args.evidence_dir)/self.evidence_key/'model_inputs'/f'chunk_{self.chunk:03d}'
                dest.mkdir(parents=True, exist_ok=False)
                for name, img in zip(self.camera_names, images):
                    Image.fromarray(img).save(dest/(name+'.png'))
                (dest/'input.json').write_text(json.dumps({'task': task_description, 'state': state.tolist(),
                    'camera_order': self.camera_names, 'image_shapes': [list(im.shape) for im in images]}, indent=2)+'\n')
                np.savez_compressed(dest/'predicted_actions.npz', actions=prediction)
                self.chunk += 1
                return prediction
            self.agent.get_action_chunk = record_chunk
            self._evidence_wrapped = True

        def inference_model(self, observation):
            # 上游 reset 标志会在 inference 和 populate 两处 reset；在此只处理一次。
            obs = dict(observation)
            key = obs.pop('_evidence_key')
            if not re.fullmatch(r'(A|P[0-9]{3})_attempt_[0-9]{3}', key):
                raise ValueError('非法 evidence key')
            if obs.pop('reset', False):
                self.reset()
            if self.evidence_key not in (None, key):
                raise ValueError('跨试验未 reset')
            self.evidence_key = key
            return super().inference_model(obs)

    checkpoint = str(Path(args.checkpoint_path).resolve())
    config = MolmoBotRBY1PickPnPPolicyConfig(checkpoint_path=checkpoint, clamp_gripper=False)
    policy = EvidencePolicy(config)
    policy.reset()
    metadata = {'checkpoint_path': checkpoint, 'config_sha256': hashlib.sha256((Path(checkpoint)/'config.yaml').read_bytes()).hexdigest(),
                'task_type': 'pick_pnp', 'camera_names': config.camera_names, 'state_dim': 22, 'action_dim': 20,
                'clamp_gripper': False, 'execute_horizon': 8, 'action_horizon': 16, 'seed': args.seed}
    logging.basicConfig(level=logging.INFO)
    WebsocketPolicyServer([policy], 'lastmile-molmobot-multitask-pick', host='127.0.0.1', port=args.port, metadata=metadata).serve_forever()


if __name__ == '__main__':
    main()
