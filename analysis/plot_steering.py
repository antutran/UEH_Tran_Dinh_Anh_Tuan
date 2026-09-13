#!/usr/bin/env python3
"""plot_steering.py – Compare steering behaviour across multiple runs.

Usage:
    python3 analysis/plot_steering.py /tmp/crc_logs/run_A.csv /tmp/crc_logs/run_B.csv ...

Produces:
    analysis/steering_comparison.png   – overlaid lane error traces
    analysis/speed_comparison.png      – overlaid speed traces
    analysis/error_histogram.png       – histogram of |lane_error| per run

Use this to compare behaviour before/after changing a parameter (e.g. base_speed).
"""

import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load(path):
    df = pd.read_csv(path)
    for col in ['time_s', 'cmd_v', 'cmd_w', 'lane_error_px', 'front_dist_m']:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    df['time_s'] = df['time_s'] - df['time_s'].iloc[0]
    return df


def save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  saved: {path}')


def main():
    paths = sys.argv[1:]
    if not paths:
        print('Usage: python3 plot_steering.py run1.csv run2.csv ...')
        sys.exit(1)

    dfs    = [load(p) for p in paths]
    labels = [os.path.basename(p).replace('.csv', '') for p in paths]
    out    = os.path.dirname(os.path.abspath(__file__))

    # ---- Steering comparison ------------------------------------------------
    fig, ax = plt.subplots(figsize=(12, 4))
    for df, lbl in zip(dfs, labels):
        ax.plot(df['time_s'], df['lane_error_px'], lw=1.0, label=lbl, alpha=0.8)
    ax.axhline(0, color='k', lw=0.8, linestyle='--')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Lane error (px)')
    ax.set_title('Lane error comparison across runs')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    save(fig, os.path.join(out, 'steering_comparison.png'))

    # ---- Speed comparison ---------------------------------------------------
    fig, ax = plt.subplots(figsize=(12, 4))
    for df, lbl in zip(dfs, labels):
        ax.plot(df['time_s'], df['cmd_v'], lw=1.0, label=lbl, alpha=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('cmd_v (m/s)')
    ax.set_title('Commanded speed comparison across runs')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    save(fig, os.path.join(out, 'speed_comparison.png'))

    # ---- Error histogram ----------------------------------------------------
    fig, ax = plt.subplots(figsize=(8, 4))
    for df, lbl in zip(dfs, labels):
        ax.hist(df['lane_error_px'].abs().dropna(),
                bins=40, alpha=0.6, label=lbl, density=True)
    ax.set_xlabel('|Lane error| (px)')
    ax.set_ylabel('Density')
    ax.set_title('Distribution of absolute lane error')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    save(fig, os.path.join(out, 'error_histogram.png'))

    # ---- Summary table -------------------------------------------------------
    print(f'\n{"Run":<40}  {"Duration":>8}  {"Mean|err|":>9}  {"Mean v":>7}')
    print('-' * 70)
    for df, lbl in zip(dfs, labels):
        t   = df['time_s'].iloc[-1]
        err = df['lane_error_px'].abs().mean()
        v   = df['cmd_v'].mean()
        print(f'{lbl:<40}  {t:>8.1f}s  {err:>9.1f}px  {v:>7.3f}m/s')


if __name__ == '__main__':
    main()
