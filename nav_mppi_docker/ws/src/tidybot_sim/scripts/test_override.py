#!/usr/bin/env python3
"""override() 회귀 테스트.  python3 scripts/test_override.py

핵심은 첫 번째 검사다: 빈 문자열이면 MPPIConfig() 와 완전히 같은 설정이 나와야 한다.
기준선(controller:=nav2 / mppi)은 mppi_overrides: '' 로 돌므로, 이것이 성립하는 한
override() 에 키를 추가해도 벤치마크 수치는 영향을 받을 수 없다.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mppi_controller import override            # noqa: E402
from nav.mppi import MPPIConfig                 # noqa: E402

base = MPPIConfig()

assert override(MPPIConfig(), '') == base, '빈 spec 이 기본 설정을 바꿨다'
assert override(MPPIConfig(), '   ') == base, '공백 spec 이 기본 설정을 바꿨다'

c = override(MPPIConfig(), 'v_max_xy=0.2')
assert c.v_max == (0.2, 0.2, base.v_max[2]), c.v_max

c = override(MPPIConfig(), 'v_max_wz=0.6')
assert c.v_max == (base.v_max[0], base.v_max[0], 0.6), c.v_max

c = override(MPPIConfig(), 'v_max_xy=0.2 v_max_wz=0.6')
assert c.v_max == (0.2, 0.2, 0.6), c.v_max

for bad in ('v_max_xy=0', 'v_max_wz=0', 'v_max_xy=-0.1'):
    try:
        override(MPPIConfig(), bad)
    except AssertionError:
        continue
    raise AssertionError(f'{bad} 가 거부되지 않았다')

c = override(MPPIConfig(), 'K=256 sigma_v=0.1 v_max_wz=0.6')
assert (c.K, c.sigma, c.v_max[2]) == (256, (0.1, 0.1, base.sigma[2]), 0.6)

print('override() 회귀 OK: 빈 spec 은 기본값과 동일, v_max_xy/v_max_wz 독립 적용, 0 이하 거부')
