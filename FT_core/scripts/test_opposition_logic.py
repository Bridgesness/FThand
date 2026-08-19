# ==============================================================================
# 对指吸附逻辑纯测 (不接硬件, 只验证 TeleoperationController 的吸附状态机)
# 运行: python scripts/test_opposition_logic.py
# 验证: 进入吸附/锁定最近手指/软吸附数值/远离不吸/迟滞保持/缺关节跳过
# ==============================================================================
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from teleoperation import TeleoperationController

POSE_INDEX = {
    'thumb_abd': 0.0, 'thumb_mcp': -30.0, 'thumb_pip': 40.0, 'thumb_dip': 30.0,
    'index_abd': 0.0, 'index_mcp': 60.0, 'index_pip': 55.0,
}
DEFAULT = {'thumb_abd': 20.0, 'thumb_mcp': 20.0, 'thumb_pip': 20.0, 'thumb_dip': 20.0}

c = TeleoperationController(left_hand_model_path='orca_core/models/orcahand_v1_left')
c._opp_poses['left'] = {
    'index':  POSE_INDEX,
    'middle': dict(DEFAULT, middle_mcp=90, middle_pip=80),
    'ring':   dict(DEFAULT, ring_mcp=90, ring_pip=80),
    'pinky':  dict(DEFAULT, pinky_mcp=90, pinky_pip=80),
}

def reset():
    c._opp_snapping['left'] = False
    c._opp_target['left'] = None

# 测试1: 接近 -> 进入吸附, 锁定 index, 软吸附拉向位姿
reset()
near = {k: v + 10.0 for k, v in POSE_INDEX.items()}
orig = dict(near)  # _apply_opposition_snap 原地修改并返回, 期望值要用吸附前的原始值
out = c._apply_opposition_snap('left', near)
assert c._opp_snapping['left'] is True, "应进入吸附"
assert c._opp_target['left'] == 'index', "应锁定 index"
for j in POSE_INDEX:
    exp = orig[j] + 0.7 * (POSE_INDEX[j] - orig[j])
    assert abs(out[j] - exp) < 1e-6, f"{j} 未软吸附 (out={out[j]:.2f}, exp={exp:.2f})"
print("测试1 通过: 接近 -> 进入吸附并锁定 index")

# 测试2: 远离 -> 不吸附, 输出原样
reset()
far = {k: v + 60.0 for k, v in POSE_INDEX.items()}
out = c._apply_opposition_snap('left', far)
assert c._opp_snapping['left'] is False, "远距离不应吸附"
assert out == far, "未吸附时输出应原样"
print("测试2 通过: 远离不吸附")

# 测试3: 已吸附中距离超释放阈值 -> 释放
c._opp_snapping['left'] = True
c._opp_target['left'] = 'index'
out = c._apply_opposition_snap('left', far)
assert c._opp_snapping['left'] is False, "应释放"
assert out == far
print("测试3 通过: 超过释放阈值 -> 释放")

# 测试4: 迟滞区间(25~45度)保持吸附
reset()
c._opp_snapping['left'] = True
c._opp_target['left'] = 'index'
mid = {k: v + 35.0 for k, v in POSE_INDEX.items()}
out = c._apply_opposition_snap('left', mid)
assert c._opp_snapping['left'] is True, "迟滞区间应保持吸附"
assert c._opp_target['left'] == 'index'
print("测试4 通过: 迟滞区间保持吸附")

# 测试5: 缺关节跳过, 不崩且能吸附
reset()
pose_missing = dict(POSE_INDEX); pose_missing.pop('thumb_dip')
c._opp_poses['left']['index'] = pose_missing
out = c._apply_opposition_snap('left', near)
assert c._opp_snapping['left'] is True, "缺关节仍应能吸附"
print("测试5 通过: 缺关节自动跳过")

print("\n=== 全部对指吸附逻辑测试通过 ===")
