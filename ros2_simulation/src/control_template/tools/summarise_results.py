#!/usr/bin/env python3
"""Summarise the scored test runs in ~/trace_results into evidence tables.

mission_logger.py adds one row per mission to results.csv. This groups the rows
by test condition (world, drones, casualty pose) and prints a markdown table
for the report, then writes:

    <results dir>/summary.md    the same tables
    <results dir>/summary.png   search time per casualty pose, one bar per
                                drone setup, one panel per world (needs matplotlib)

    python3 tools/summarise_results.py [results dir, default ~/trace_results]

Rows from an older logger (results_old_*.csv, no world/drones columns) are
read too and shown as world "?", drones "1".
"""
import csv
import glob
import math
import os
import statistics
import sys
from collections import OrderedDict, defaultdict


def num(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


def load(results_dir):
    rows = []
    files = sorted(glob.glob(os.path.join(results_dir, 'results_old_*.csv')))
    files += [os.path.join(results_dir, 'results.csv')]
    for path in files:
        if not os.path.exists(path):
            continue
        with open(path, newline='') as fh:
            for r in csv.DictReader(fh):
                r.setdefault('world', '?')
                r.setdefault('drones', '1')
                if not r.get('outcome'):
                    state = r.get('mission_state', '')
                    r['outcome'] = (state if state in ('DONE', '') else
                                    'FAILED_BLOCKED' if num(r.get('routes_planned')) else
                                    'FAILED_UNREACHABLE' if state == 'FAILED' else
                                    f'INCOMPLETE_{state}')
                rows.append(r)
    return rows


def mean(values):
    values = [v for v in values if v is not None]
    return statistics.mean(values) if values else None


def fmt(v, digits=1):
    return '-' if v is None else f'{v:.{digits}f}'


def pose_key(p):
    return (0, int(p)) if str(p).isdigit() else (1, str(p))


def table(rows):
    groups = OrderedDict()
    for r in sorted(rows, key=lambda r: (r['world'], r['drones'], pose_key(r['human_pose']))):
        groups.setdefault((r['world'], r['drones'], r['human_pose']), []).append(r)
    out = ['| World | Drones | Pose | Runs | Outcome | Search (s) | Detection error (m) | '
           'Stop dist. to body (m) | Confirm to end (s) | Moved after arrival (m) | Found by |',
           '|---|---|---|---|---|---|---|---|---|---|---|']
    for (world, drones, pose), rs in groups.items():
        outcomes = defaultdict(int)
        for r in rs:
            outcomes[r['outcome']] += 1
        found = defaultdict(int)
        for r in rs:
            if r.get('found_by'):
                found[r['found_by']] += 1
        moved = [num(r.get('moved_after_arrival_m')) for r in rs]
        moved = [m for m in moved if m is not None]
        out.append('| ' + ' | '.join([
            world, drones, pose, str(len(rs)),
            ', '.join(f'{k} {v}/{len(rs)}' for k, v in sorted(outcomes.items())),
            fmt(mean(num(r.get('search_s')) for r in rs)),
            fmt(mean(num(r.get('detection_error_body_m')) for r in rs), 2),
            fmt(mean(num(r.get('stop_dist_to_body_m')) for r in rs), 2),
            fmt(mean(num(r.get('confirm_to_end_s')) for r in rs)),
            fmt(max(moved), 2) if moved else '-',
            ', '.join(f'{k} {v}' for k, v in sorted(found.items())) or '-',
        ]) + ' |')
    return out


def overall(rows):
    groups = OrderedDict()
    for r in sorted(rows, key=lambda r: (r['world'], r['drones'])):
        if r['human_pose'] == '6':
            continue   # the unreachable test is reported on its own line above
        groups.setdefault((r['world'], r['drones']), []).append(r)
    out = ['| World | Drones | Runs (poses 0-5) | Reached casualty | Mean search (s) | '
           'Mean detection error (m) |', '|---|---|---|---|---|---|']
    for (world, drones), rs in groups.items():
        done = sum(r['outcome'] == 'DONE' for r in rs)
        out.append(f'| {world} | {drones} | {len(rs)} | {done}/{len(rs)} | '
                   f'{fmt(mean(num(r.get("search_s")) for r in rs))} | '
                   f'{fmt(mean(num(r.get("detection_error_body_m")) for r in rs), 2)} |')
    return out


def chart(rows, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('(matplotlib not installed: no chart. sudo apt install python3-matplotlib)')
        return False
    rows = [r for r in rows if num(r.get('search_s')) is not None]
    if not rows:
        return False
    surface, ink, ink2, grid = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
    series = ['#2a78d6', '#eb6834', '#1baf7a']   # categorical slots 1-3, fixed order
    setups = sorted({r['drones'] for r in rows}, key=lambda d: (d != '1', d))[:3]
    worlds = sorted({r['world'] for r in rows}, key=lambda w: (w != 'terrain_demo', w))
    poses = sorted({r['human_pose'] for r in rows}, key=pose_key)
    fig, axes = plt.subplots(1, len(worlds), figsize=(5.5 * len(worlds), 3.8), sharey=True,
                             squeeze=False, facecolor=surface)
    width = 0.8 / len(setups)
    for ax, world in zip(axes[0], worlds):
        ax.set_facecolor(surface)
        for i, setup in enumerate(setups):
            xs, ys = [], []
            for j, pose in enumerate(poses):
                v = mean(num(r['search_s']) for r in rows
                         if r['world'] == world and r['drones'] == setup and r['human_pose'] == pose)
                if v is not None:
                    xs.append(j - 0.4 + width * (i + 0.5))
                    ys.append(v)
            label = '1 drone' if setup == '1' else setup.replace('2 ', '2 drones, ')
            ax.bar(xs, ys, width, color=series[i], edgecolor=surface, linewidth=2, label=label,
                   zorder=2)
        ax.set_xticks(range(len(poses)))
        ax.set_xticklabels([f'pose {p}' for p in poses], color=ink2, fontsize=9)
        ax.set_title(world, color=ink, fontsize=11, loc='left')
        ax.grid(axis='y', color=grid, linewidth=0.8, zorder=0)
        for side in ('top', 'right', 'left'):
            ax.spines[side].set_visible(False)
        ax.spines['bottom'].set_color(grid)
        ax.tick_params(colors=ink2, length=0)
    axes[0][0].set_ylabel('Search time to first detection (s)', color=ink2, fontsize=9)
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, frameon=False, fontsize=9, labelcolor=ink2, loc='upper right',
               ncol=len(setups))
    fig.suptitle('Time to find the casualty, by drone setup', color=ink, fontsize=12, x=0.01,
                 ha='left')
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(path, dpi=200, facecolor=surface)
    return True


def main():
    results_dir = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/trace_results')
    rows = load(results_dir)
    if not rows:
        sys.exit(f'No results in {results_dir}: run missions with the logger on first.')
    lines = ['## TRACE test results', '', f'{len(rows)} scored runs from {results_dir}.', '',
             '### Per condition', ''] + table(rows) + ['', '### Overall (reachable poses)', ''] + overall(rows)
    text = '\n'.join(lines) + '\n'
    print(text)
    with open(os.path.join(results_dir, 'summary.md'), 'w') as fh:
        fh.write(text)
    png = os.path.join(results_dir, 'summary.png')
    if chart(rows, png):
        print(f'Chart: {png}')
    print(f'Tables: {os.path.join(results_dir, "summary.md")}')


if __name__ == '__main__':
    main()
