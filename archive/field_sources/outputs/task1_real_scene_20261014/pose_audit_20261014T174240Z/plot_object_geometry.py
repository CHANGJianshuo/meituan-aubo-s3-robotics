"""Plot a static geometric comparison; does not run a physical simulation."""
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from scipy.spatial.transform import Rotation

OUT = Path(__file__).resolve().parent
cfg = json.loads((OUT.parent/'inputs/scenario.json').read_text())
candidate = json.loads((OUT/'current_object_placement.json').read_text())
tool = np.array(json.loads((OUT/'gap16_candidate.json').read_text())['T_base_tool'])
old = np.array(cfg['initial_objects']['A'])*1000
new = np.array(candidate['candidate_battery_origin_base_mm'])
R = Rotation.from_euler('z', cfg['yaw']).as_matrix()


def box(center, size):
    return np.array([[x, y, z] for x in [-1, 1] for y in [-1, 1] for z in [-1, 1]])*np.array(size)/2+center


fig, axes = plt.subplots(1, 2, figsize=(12, 6), layout='constrained')
for ax, dims in zip(axes, [(0, 1), (0, 2)]):
    for origin, color, alpha, label in [(old, '#777777', .13, 'Original battery'), (new, '#df493b', .32, 'Current candidate')]:
        for j, (center, size) in enumerate([([0, 0, 25], [70, 80, 50]),
                                          ([-30, 0, 60], [10, 26, 20]),
                                          ([30, 0, 60], [10, 26, 20]),
                                          ([0, 0, 74], [70, 26, 12])]):
            v = box(center, size)@R.T+origin
            hull = cv2.convexHull(v[:, dims].astype(np.float32)).reshape(-1, 2)
            ax.add_patch(Polygon(hull, facecolor=color, alpha=alpha, edgecolor=color,
                                 linewidth=1.5, label=label if j == 0 else None))
    for j, (center, size) in enumerate([([0, 0, 58], [20, 30, 100]),
                                       ([32.5, 0, 110.5], [85, 44, 5]),
                                       ([72.5, 0, 105.5], [5, 44, 5])]):
        v = box(center, size)@tool[:3, :3].T+tool[:3, 3]*1000
        hull = cv2.convexHull(v[:, dims].astype(np.float32)).reshape(-1, 2)
        ax.add_patch(Polygon(hull, facecolor='#147fae', alpha=.5, edgecolor='#147fae', label='Fixed V1 tool' if j == 0 else None))
    ax.grid(alpha=.2)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('Robot base X [mm]')
    ax.set_xlim(210, 410)
    ax.legend(fontsize=8, loc='upper left')
axes[0].set_ylim(185, 315)
axes[0].set_ylabel('Robot base Y [mm]')
axes[0].set_title('Top view: current bar lies under tongue')
axes[1].set_ylim(-90, 175)
axes[1].set_ylabel('Robot base Z [mm]')
axes[1].set_title('Side view: support height remains unresolved')
axes[1].axhline(cfg['table_z']*1000, color='#777777', linestyle='--', linewidth=1)
axes[1].text(214, cfg['table_z']*1000-8, 'Old support height', fontsize=8, color='#555555')
axes[1].axhline(new[2], color='#d64b3e', linestyle=':', linewidth=1.2)
axes[1].text(214, new[2]+4, 'Support required by candidate', fontsize=8, color='#b23d30')
a = np.array(candidate['candidate_handle_top_base_mm'])
axes[1].annotate('', xy=(a[0], a[2]), xytext=(a[0], a[2]+16), arrowprops=dict(arrowstyle='<->', color='black', lw=1))
axes[1].text(a[0]+6, a[2]+7, '~16 mm', fontsize=8)
axes[1].annotate('', xy=(396, new[2]), xytext=(396, cfg['table_z']*1000), arrowprops=dict(arrowstyle='<->', color='#b23d30', lw=1))
axes[1].text(389, (new[2]+cfg['table_z']*1000)/2, '~36 mm', ha='right', va='center', fontsize=8, color='#b23d30')
fig.suptitle('Geometry review only - candidate position, not an executable scene', fontsize=13)
fig.savefig(OUT/'object_geometry_review.png', dpi=150)
plt.close(fig)
