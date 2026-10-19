#!/usr/bin/env python3
"""Prepare a road-only comparison sheet offline; never called by the race launcher."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
from patrol_world_geometry import DEFAULT_WORLD_DIR

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PRESETS = (
    ('base0', 'Middle ~ East | North ~ South'),
    ('base3', 'East wider | North ~ South'),
    ('base4', 'Middle wider | North ~ South'),
    ('base5', 'Middle ~ East | South taller'),
    ('base6', 'Middle ~ East | North taller'),
    ('base15', 'Middle wider | North taller'),
    ('base16', 'Middle wider | South taller'),
)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--world-dir', type=Path, default=DEFAULT_WORLD_DIR)
    ap.add_argument('--output', type=Path,
                    default=Path(__file__).resolve().parent.parent / 'docs/patrol_maps/road_choices.png')
    args = ap.parse_args()
    fig, axes = plt.subplots(4, 2, figsize=(13, 15))
    for i, ((name, description), ax) in enumerate(zip(PRESETS, axes.flat), 1):
        world = ET.parse(args.world_dir / (name + '.world')).getroot().find('world')
        for road in world.findall('road'):
            key = road.get('name')
            if not key.startswith('road_'):
                continue
            pts = [tuple(map(float, p.text.split()[:2])) for p in road.findall('point')]
            color = '#3778b8' if key == 'road_y_3' else '#d75b55' if key == 'road_x_2' else '#a8afb7'
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, linewidth=4,
                    solid_capstyle='butt')
        ax.set(xlim=(-50, 125), ylim=(-53, 55), aspect='equal')
        ax.set_title('route%d\n%s' % (i, description), fontsize=12)
        ax.text(0.02, .98, 'NORTH', va='top', transform=ax.transAxes, fontsize=9)
        ax.text(.98, .5, 'EAST', ha='right', transform=ax.transAxes, rotation=90, fontsize=9)
        ax.axis('off')
    axes.flat[-1].axis('off')
    fig.suptitle('Choose a patrol by visible roads\nRed: main east-west street | Blue: middle north-south street',
                 fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, .95))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150, facecolor='white')
    plt.close(fig)
    print(args.output)


if __name__ == '__main__':
    main()
