import re

import numpy as np

from nav import scene
from nav.grid import Grid, is_free, nearest_free, plan_grid, static_grid


def test_cell_world_roundtrip():
    g = Grid(origin=(-1.0, -1.0), res=0.05, shape=(220, 220))
    for xy in [(0.0, 0.0), (3.14, -0.37), (8.6, 9.42)]:
        c, r = g.to_cell(*xy)
        back = g.to_world(c, r)
        assert np.all(np.abs(back - np.asarray(xy)) <= g.res / 2 + 1e-9), (xy, back)


def test_box_rasterised_where_the_xml_puts_it(tmp_path):
    """The MJCF and the grid must come from the same box list -- verify by
    re-parsing the generated XML instead of trusting the shared constant.

    tmp_path 로 쓴다. 기본 경로(models/stanford_tidybot/nav_scene.xml)는 MuJoCo
    런너가 읽는 공유 애셋이라 테스트가 덮어쓸 자리가 아니고, 그 디렉터리는 TidyBot++
    저장소에만 있어서 ROS 이미지에서는 존재하지 않는다 (메시를 가져올 이유도 없다 --
    이 테스트는 XML 을 쓰고 되읽기만 한다)."""
    path = scene.write_scene(n_peds=0, path=tmp_path / 'nav_scene.xml')
    g = static_grid()
    boxes = []
    for pos, size in re.findall(r'name="wall\d+".*?pos="([^"]+)".*?size="([^"]+)"', path.read_text()):
        (cx, cy, _), (hx, hy, _) = (np.fromstring(s, sep=' ') for s in (pos, size))
        boxes.append(tuple(round(float(v), 4) for v in (cx - hx, cy - hy, cx + hx, cy + hy)))

    assert sorted(boxes) == sorted(tuple(round(float(v), 4) for v in b) for b in scene.WALLS)
    for xmin, ymin, xmax, ymax in boxes:
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        c, r = g.to_cell(cx, cy)
        assert g.occ[r, c], f'wall centre ({cx}, {cy}) not marked occupied'


def test_free_space_is_free():
    g = static_grid()
    for xy in [(0.0, 0.0), (8.0, 8.0), (3.15, 4.0), (1.0, 1.0)]:   # incl. the doorway
        c, r = g.to_cell(*xy)
        assert not g.occ[r, c], xy


def test_bake_cost_field():
    g = static_grid()
    inside = g.lookup(np.array([3.15, 1.0]))          # inside the doorway wall
    # Open space has to be a point that stays open: (1.5, 1.5) was, until the
    # corridor's east wall landed 0.1 m away at x = 1.6. CROWD_AREA is kept clear
    # of interior walls by scene.py, so a point in it cannot rot the same way.
    far = g.lookup(np.array([5.0, 2.0]))
    assert inside == 10.0
    assert far < 0.05
    assert g.dist.min() == 0.0 and g.dist.max() > 1.0


def test_nearest_free_snaps_out_of_inflation():
    pg = plan_grid()
    stuck = (3.0, 2.0)                                 # inside the wall
    assert not is_free(pg, stuck, scene.PLAN_CLEARANCE)
    snapped = nearest_free(pg, stuck, scene.PLAN_CLEARANCE)
    assert is_free(pg, snapped, scene.PLAN_CLEARANCE)
    assert np.linalg.norm(snapped - np.asarray(stuck)) < 1.5


def test_from_boxes_rejects_out_of_bounds():
    try:
        Grid.from_boxes([(-5.0, 0.0, -4.0, 1.0)])      # would wrap the slice
    except AssertionError:
        return
    raise AssertionError('expected out-of-grid box to fail loudly')
