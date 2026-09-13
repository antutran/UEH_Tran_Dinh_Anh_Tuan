#!/usr/bin/env python3
"""plot_run.py – Generate summary figures from a recorded CSV run.

Usage:
    python3 analysis/plot_run.py /tmp/crc_logs/run_20260913_120000.csv

Produces (saved alongside the CSV):
    run_<ts>_speed.png      – linear and angular speed over time
    run_<ts>_lane_error.png – lane error over time
    run_<ts>_lidar.png      – front LiDAR distance over time
    run_<ts>_events.png     – timeline of STOP / RED_LIGHT / PEDESTRIAN events
    run_<ts>_path.png       – x-y path from odometry

All figures are publication-ready (axes labelled, grid, legend).
Do NOT run this with fabricated data – it reads only real CSV logs.
"""

import os
import sys

import matplotlib
matplotlib.use('Agg')   # headless backend
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd


def load_csv(path):
    df = pd.read_csv(path)
    # Numeric columns
    for col in ['time_s', 'odom_x', 'odom_y', 'distance_m',
                'odom_v', 'odom_w', 'cmd_v', 'cmd_w',
                'lane_error_px', 'front_dist_m']:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    df['front_dist_m'] = df['front_dist_m'].clip(upper=4.0)
    return df


def save(fig, stem):
    fig.tight_layout()
    fig.savefig(stem, dpi=150)
    plt.close(fig)
    print(f'  saved: {stem}')


def plot_speed(df, stem):
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(df['time_s'], df['cmd_v'],  label='cmd linear v (m/s)',  lw=1.5)
    ax.plot(df['time_s'], df['cmd_w'],  label='cmd angular w (rad/s)', lw=1.0, alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity')
    ax.set_title('Commanded velocity over time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    save(fig, stem + '_speed.png')


def plot_lane_error(df, stem):
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(df['time_s'], df['lane_error_px'], lw=1.0, color='darkorange')
    ax.axhline(0, color='k', lw=0.8, linestyle='--')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Lane error (pixels)')
    ax.set_title('Lane centre error over time\n(positive = lane centre is LEFT of image centre)')
    ax.grid(True, alpha=0.3)
    save(fig, stem + '_lane_error.png')


def plot_lidar(df, stem):
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(df['time_s'], df['front_dist_m'], lw=1.0, color='steelblue')
    ax.axhline(0.25, color='red',    lw=1.0, linestyle='--', label='Emergency stop (0.25 m)')
    ax.axhline(0.55, color='orange', lw=1.0, linestyle='--', label='Slow zone (0.55 m)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    ax.set_title('LiDAR front distance over time')
    ax.set_ylim(0, 4.1)
    ax.legend()
    ax.grid(True, alpha=0.3)
    save(fig, stem + '_lidar.png')


def plot_events(df, stem):
    """Show a timeline of detected signs, lights, pedestrian events."""
    fig, ax = plt.subplots(figsize=(10, 3))

    # Events
    stop_t = df.loc[df['sign'] == 'STOP', 'time_s'].values
    red_t  = df.loc[df['light'].isin(['RED', 'YELLOW']), 'time_s'].values
    ped_t  = df.loc[df['pedestrian'] == 1, 'time_s'].values

    if len(stop_t):
        ax.scatter(stop_t, np.ones_like(stop_t) * 2, marker='v',
                   color='red', s=40, label='STOP sign detected', zorder=3)
    if len(red_t):
        ax.scatter(red_t, np.ones_like(red_t) * 1, marker='s',
                   color='darkred', s=20, label='RED/YELLOW light', zorder=3)
    if len(ped_t):
        ax.scatter(ped_t, np.ones_like(ped_t) * 0, marker='o',
                   color='purple', s=20, label='Pedestrian blocking', zorder=3)

    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(['Pedestrian', 'Light', 'STOP sign'])
    ax.set_xlabel('Time (s)')
    ax.set_title('Detection event timeline')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, axis='x', alpha=0.3)
    save(fig, stem + '_events.png')


def plot_path(df, stem):
    fig, ax = plt.subplots(figsize=(8, 5))
    sc = ax.scatter(df['odom_x'], df['odom_y'],
                    c=df['time_s'], cmap='viridis', s=2, linewidths=0)
    plt.colorbar(sc, ax=ax, label='Time (s)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Robot path (odometry)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    # Mark start
    ax.scatter(df['odom_x'].iloc[0], df['odom_y'].iloc[0],
               color='green', s=80, zorder=5, label='Start')
    ax.scatter(df['odom_x'].iloc[-1], df['odom_y'].iloc[-1],
               color='red', s=80, zorder=5, marker='x', label='End')
    ax.legend()
    save(fig, stem + '_path.png')


def print_summary(df, csv_path):
    t_total    = df['time_s'].iloc[-1] - df['time_s'].iloc[0]
    dist       = df['distance_m'].iloc[-1]
    mean_v     = df['cmd_v'].mean()
    max_err    = df['lane_error_px'].abs().max()
    mean_err   = df['lane_error_px'].abs().mean()
    n_stop     = (df['sign'] == 'STOP').sum()
    n_red      = df['light'].isin(['RED', 'YELLOW']).sum()
    min_lidar  = df['front_dist_m'].min()

    print(f'\n=== Run Summary: {os.path.basename(csv_path)} ===')
    print(f'  Duration:         {t_total:.1f} s')
    print(f'  Distance:         {dist:.2f} m')
    print(f'  Mean cmd_v:       {mean_v:.3f} m/s')
    print(f'  Lane error mean:  {mean_err:.1f} px')
    print(f'  Lane error max:   {max_err:.1f} px')
    print(f'  STOP detections:  {n_stop} frames')
    print(f'  RED/YEL detects:  {n_red} frames')
    print(f'  Min front dist:   {min_lidar:.3f} m')


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 plot_run.py <run_csv_path>')
        sys.exit(1)

    csv_path = sys.argv[1]
    if not os.path.isfile(csv_path):
        print(f'File not found: {csv_path}')
        sys.exit(1)

    df   = load_csv(csv_path)
    stem = csv_path.replace('.csv', '')

    print(f'Loaded {len(df)} rows from {csv_path}')
    print_summary(df, csv_path)

    plot_speed(df, stem)
    plot_lane_error(df, stem)
    plot_lidar(df, stem)
    plot_events(df, stem)
    plot_path(df, stem)

    print('\nAll figures saved.')


if __name__ == '__main__':
    main()
