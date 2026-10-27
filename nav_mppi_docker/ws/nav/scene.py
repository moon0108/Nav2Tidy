"""Scene geometry: the single source of truth for both the MJCF and the grid.

Walls live here as plain (xmin, ymin, xmax, ymax) tuples in world metres.
`write_scene()` renders them to MJCF, `grid.static_grid()` rasterises the same
list. Nothing else may define wall coordinates -- a mismatch between the two
makes every downstream debugging session meaningless.
"""

from pathlib import Path

Box = tuple[float, float, float, float]  # (xmin, ymin, xmax, ymax) world metres

# Arena interior; boundary walls sit just outside these lines.
ARENA: Box = (-0.5, -0.5, 9.5, 9.5)
WALL_T = 0.2   # wall thickness (m)
WALL_H = 1.0   # wall height (m)

BOUNDARY: list[Box] = [
    (ARENA[0] - WALL_T, ARENA[1] - WALL_T, ARENA[2] + WALL_T, ARENA[1]),           # south
    (ARENA[0] - WALL_T, ARENA[3], ARENA[2] + WALL_T, ARENA[3] + WALL_T),           # north
    (ARENA[0] - WALL_T, ARENA[1] - WALL_T, ARENA[0], ARENA[3] + WALL_T),           # west
    (ARENA[2], ARENA[1] - WALL_T, ARENA[2] + WALL_T, ARENA[3] + WALL_T),           # east
]

# Interior walls: a 1.2 m doorway at y ~= 4.0, and a U-shaped trap opening west
# (towards the usual start) so greedy local planning falls into a local minimum.
INTERIOR: list[Box] = [
    (3.0, -0.5, 3.3, 3.4),   # doorway wall, lower half
    (3.0, 4.6, 3.3, 9.5),    # doorway wall, upper half
    (6.8, 5.0, 7.1, 8.0),    # U trap: back
    (4.8, 7.7, 7.1, 8.0),    # U trap: top
    (4.8, 5.0, 7.1, 5.3),    # U trap: bottom
]

# 서쪽 스트립을 가로지르는 폭 1.2 m, 길이 7 m 채널. 폭은 문틈과 같게 두었다 --
# PLAN_CLEARANCE가 확실히 통과한다고 알려진 유일한 폭이고, 이 구역의 난이도는
# 좁기가 아니라 길이다. 서쪽에 남는 0.7 m 틈은 의도적으로 PLAN_CLEARANCE 미만이라
# 우회로가 생기지 않는다.
CORRIDOR: list[Box] = [
    (0.2, 1.2, 0.4, 8.2),    # corridor: west wall
    (1.6, 1.2, 1.8, 8.2),    # corridor: east wall
]

# U 트랩 북동쪽 기둥밭. 행마다 좌우를 번갈아 두어 직선으로는 빠져나갈 수 없고,
# 모든 틈은 1.0 m 이상 -- 2 * PLAN_CLEARANCE 바로 위다.
CLUTTER: list[Box] = [
    (7.10, 5.40, 7.40, 5.70),
    (8.15, 5.40, 8.45, 5.70),
    (7.10, 6.60, 7.40, 6.90),
    (9.20, 6.60, 9.50, 6.90),
    # 3행은 가운데를 비운다: runner.py의 기본 목표 (8, 8)이 여기 있고, 그 지점의
    # PLAN_CLEARANCE를 먹으면 MuJoCo 쪽 기본 에피소드가 그대로 막힌다.
    (7.10, 7.80, 7.40, 8.10),
    (8.60, 7.80, 8.90, 8.10),
]

WALLS: list[Box] = BOUNDARY + INTERIOR + CORRIDOR + CLUTTER

# 벤치마크 시작/목표 쌍, 구역당 하나. (x, y, theta) -> (x, y). 의존하는 기하와 같은
# 파일에 둔다. 네 쌍 모두 A*로 도달 가능한지는 gen_world.py가 assert 한다.
SCENARIOS: dict[str, tuple[tuple[float, float, float], tuple[float, float]]] = {
    'corridor': ((1.00, 0.40, 1.5708), (1.00, 8.80)),
    'doorway':  ((2.40, 8.50, 0.0000), (5.00, 2.00)),
    'utrap':    ((3.90, 6.50, 0.0000), (8.50, 4.00)),
    'clutter':  ((8.30, 4.20, 1.5708), (8.90, 8.90)),
}

# Grid covering the arena with a 0.5 m margin. Cost volumes must match exactly.
GRID_ORIGIN = (-1.0, -1.0)
GRID_RES = 0.05          # cost field resolution (MPPI + cost volume)
GRID_SPAN = 11.0         # m, square -> covers [-1, 10] on both axes
GRID_SHAPE = (round(GRID_SPAN / GRID_RES),) * 2
PLAN_RES = 0.10          # A* runs on a coarser grid: 30 ms instead of 130 ms per plan

# Open region east of the doorway, free of interior walls: where the crowd walks
# and where the benchmark episode runs, so pedestrian avoidance is measured
# without static obstacles confounding it (PLAN section 5).
CROWD_AREA: Box = (3.6, -0.2, 9.3, 4.8)
CROWD_START = (3.9, 0.1, 0.0)
CROWD_GOAL = (9.0, 4.3)

ROBOT_RADIUS = 0.35  # circumscribing circle of the TidyBot++ base footprint
# A* plans with margin on top of the robot radius so tracking error cannot graze
# a wall. Must stay below half the 1.2 m doorway width.
PLAN_CLEARANCE = 0.45
PED_RADIUS = 0.3
PED_HEIGHT = 0.8

MODELS_DIR = Path(__file__).resolve().parent.parent / 'models'
# Must live next to scene_base.xml: base.xml's meshdir="../assets" is resolved
# relative to this file's directory, so models/nav_scene.xml cannot find the meshes.
SCENE_PATH = MODELS_DIR / 'stanford_tidybot' / 'nav_scene.xml'

_HEADER = """<mujoco model="nav scene">
  <include file="base.xml"/>

  <statistic center="4.5 4.5 0.5" extent="8.0" meansize="0.05"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-40" offwidth="1280" offheight="960"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5"/>
    <material name="wall" rgba="0.55 0.55 0.6 1"/>
  </asset>

  <worldbody>
    <light pos="4.5 4.5 6" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
"""


def _wall_xml(i: int, box: Box) -> str:
    xmin, ymin, xmax, ymax = box
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    hx, hy = (xmax - xmin) / 2, (ymax - ymin) / 2
    return (f'    <geom name="wall{i}" type="box" pos="{cx:.4f} {cy:.4f} {WALL_H / 2:.4f}" '
            f'size="{hx:.4f} {hy:.4f} {WALL_H / 2:.4f}" material="wall"/>\n')


def _ped_xml(i: int, pos: tuple[float, float]) -> str:
    # contype/conaffinity 0: mocap bodies do not respond to physics anyway, and
    # robot-pedestrian collisions are judged by distance, not by MuJoCo contacts.
    return (f'    <body name="ped{i}" mocap="true" pos="{pos[0]:.3f} {pos[1]:.3f} {PED_HEIGHT:.3f}">\n'
            f'      <geom type="cylinder" size="{PED_RADIUS} {PED_HEIGHT}" rgba="0.8 0.3 0.3 1" '
            f'contype="0" conaffinity="0"/>\n'
            f'    </body>\n')


def write_scene(n_peds: int = 0, path: Path = SCENE_PATH) -> Path:
    """Render WALLS (+ n_peds mocap cylinders) to MJCF. Called before every run
    so the XML can never drift from the python geometry."""
    xml = [_HEADER]
    xml += [_wall_xml(i, b) for i, b in enumerate(WALLS)]
    # Parked off-arena; pedestrians.py moves them on the first step.
    xml += [_ped_xml(i, (-5.0 - 1.5 * i, -5.0)) for i in range(n_peds)]
    xml.append('  </worldbody>\n</mujoco>\n')
    path.write_text(''.join(xml))
    return path


if __name__ == '__main__':
    print(write_scene(n_peds=8))
