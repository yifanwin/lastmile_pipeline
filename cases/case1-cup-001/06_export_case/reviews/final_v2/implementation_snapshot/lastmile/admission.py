"""资产白名单只接受同一冻结协议下的物理证据，不接受规划或历史结果。"""
from .validation import validate_trace


def admit_asset(asset_id, mode, trials, protocol_hash, protocol, confirmed_handle=False):
    if mode not in ('ordinary','handle'):
        raise ValueError('未知抓取模式')
    if mode=='handle' and not confirmed_handle:
        return {'asset_id':asset_id,'status':'pending_annotation','reason':'handle_region_unconfirmed'}
    nominal=[t for t in trials if t.get('kind')=='nominal']
    perturbed=[t for t in trials if t.get('kind')=='perturbed']
    need=protocol['admission']
    if len(nominal)!=need['nominal_trials'] or len(perturbed)!=need['perturbed_trials']:
        return {'asset_id':asset_id,'status':'pending_physics','reason':'insufficient_independent_trials'}
    selections={t.get('selection_protocol_sha256') for t in trials}
    if None in selections or len(selections)!=1:
        return {'asset_id':asset_id,'status':'infrastructure_error','reason':'selection_protocol_not_frozen'}
    identifiers=[t.get('trial_id') for t in trials]
    if None in identifiers or len(set(identifiers)) != len(identifiers):
        return {'asset_id':asset_id,'status':'infrastructure_error','reason':'duplicate_or_missing_trial_id'}
    for t in trials:
        if t.get('protocol_sha256') != protocol_hash or t.get('snapshot_restored') is not True:
            return {'asset_id':asset_id,'status':'infrastructure_error','reason':'protocol_or_snapshot_mismatch'}
        if not t.get('trace') or not t.get('initial'):
            return {'asset_id':asset_id,'status':'infrastructure_error','reason':'missing_physical_trace'}
    verdicts=[validate_trace(t['trace'],t['initial'],protocol['thresholds'],handle=mode=='handle') for t in trials]
    if any(v['status']=='infrastructure_error' for v in verdicts):
        return {'asset_id':asset_id,'status':'infrastructure_error','reason':'invalid_physical_trace'}
    n=sum(v['status']=='verified_success' for t,v in zip(trials,verdicts) if t['kind']=='nominal')
    p=sum(v['status']=='verified_success' for t,v in zip(trials,verdicts) if t['kind']=='perturbed')
    passed=n>=need['nominal_successes'] and p>=need['perturbed_successes']
    return {'asset_id':asset_id,'mode':mode,'status':'admitted' if passed else 'rejected',
            'nominal_successes':n,'nominal_trials':len(nominal),'perturbed_successes':p,'perturbed_trials':len(perturbed),
            'reason':None if passed else 'insufficient_stability','protocol_sha256':protocol_hash}
