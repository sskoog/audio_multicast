#!/usr/bin/env python3
"""
generate_benchmark_plots.py
Generates presentation-grade plots for the ESP32-C6 LC3 Hardware Encoder Benchmark.
Saves plots to both the conversation artifacts directory and docs/assets/.
"""

import os
import shutil
import numpy as np
import matplotlib.pyplot as plt

# Artifact destination and docs directory
ARTIFACT_DIR = r"C:\Users\stefa\.gemini\antigravity-ide\brain\362c5e0d-1e79-487b-ac18-f1e63260ca67"
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS_ASSETS_DIR = os.path.join(REPO_ROOT, "docs", "assets")

os.makedirs(ARTIFACT_DIR, exist_ok=True)
os.makedirs(DOCS_ASSETS_DIR, exist_ok=True)

# Data structure
rates = ["48.0 kHz", "44.1 kHz", "32.0 kHz", "24.0 kHz", "16.0 kHz", "8.0 kHz"]
rate_vals = [48.0, 44.1, 32.0, 24.0, 16.0, 8.0]

# 10.0 ms frame duration data
avg_10ms = [7.03, 7.04, 5.56, 5.06, 3.92, 3.05]
p95_10ms = [7.68, 7.73, 6.17, 5.68, 4.50, 3.61]
min_10ms = [6.26, 6.32, 4.73, 4.38, 3.32, 2.57]
max_10ms = [15.40, 14.98, 12.14, 10.61, 8.92, 6.22]
cpu_10ms = [70.3, 70.4, 55.6, 50.6, 39.2, 30.5]
rt_10ms  = [1.42, 1.42, 1.80, 1.98, 2.55, 3.28]

# 7.5 ms frame duration data
avg_75ms = [6.00, 6.02, 4.89, 4.34, 3.53, 2.93]
p95_75ms = [6.56, 6.58, 5.42, 4.88, 4.08, 3.47]
min_75ms = [5.28, 5.30, 4.20, 3.68, 2.94, 2.45]
max_75ms = [12.50, 12.44, 11.45, 10.53, 7.53, 7.59]
cpu_75ms = [80.0, 80.3, 65.2, 57.8, 47.1, 39.1]
rt_75ms  = [1.25, 1.25, 1.53, 1.73, 2.12, 2.56]

# Plot styling configuration
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
plt.rcParams['font.sans-serif'] = 'Segoe UI, Helvetica, Arial, DejaVu Sans'
plt.rcParams['axes.edgecolor'] = '#CCCCCC'
plt.rcParams['axes.linewidth'] = 0.8

def save_and_sync(fig, filename):
    art_path = os.path.join(ARTIFACT_DIR, filename)
    doc_path = os.path.join(DOCS_ASSETS_DIR, filename)
    fig.savefig(art_path, dpi=300, bbox_inches='tight')
    fig.savefig(doc_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {art_path}")
    print(f"Synced: {doc_path}")

# ==========================================
# PLOT 1: Execution Time (Avg, P95, Min-Max)
# ==========================================
def plot_execution_times():
    fig, ax = plt.subplots(figsize=(10, 5.5))
    x = np.arange(len(rates))
    width = 0.35

    # 10 ms bars
    rects1 = ax.bar(x - width/2, avg_10ms, width, label='10.0 ms Frame (Avg)', color='#1976D2', alpha=0.9, edgecolor='black', linewidth=0.5)
    # 7.5 ms bars
    rects2 = ax.bar(x + width/2, avg_75ms, width, label='7.5 ms Frame (Avg)', color='#E65100', alpha=0.9, edgecolor='black', linewidth=0.5)

    # Add error bars showing P95
    ax.errorbar(x - width/2, avg_10ms, yerr=[np.zeros(len(avg_10ms)), np.array(p95_10ms) - np.array(avg_10ms)],
                fmt='none', ecolor='#0D47A1', elinewidth=2, capsize=4, label='P95 Upper Bound')
    ax.errorbar(x + width/2, avg_75ms, yerr=[np.zeros(len(avg_75ms)), np.array(p95_75ms) - np.array(avg_75ms)],
                fmt='none', ecolor='#BF360C', elinewidth=2, capsize=4)

    # Frame budget lines
    ax.axhline(10.0, color='#1976D2', linestyle='--', linewidth=1.2, alpha=0.7, label='10.0 ms Real-Time Budget Limit')
    ax.axhline(7.5, color='#E65100', linestyle='--', linewidth=1.2, alpha=0.7, label='7.5 ms Real-Time Budget Limit')

    # Value labels
    for rect in rects1:
        h = rect.get_height()
        ax.annotate(f'{h:.2f} ms', xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9, fontweight='bold')
    for rect in rects2:
        h = rect.get_height()
        ax.annotate(f'{h:.2f} ms', xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9, fontweight='bold')

    ax.set_title('ESP32-C6 LC3 Fixed-Point Encoder: Frame Execution Time (160 MHz)', fontsize=13, fontweight='bold', pad=15)
    ax.set_xlabel('Sample Rate', fontsize=11, labelpad=8)
    ax.set_ylabel('Execution Time per Frame (ms)', fontsize=11, labelpad=8)
    ax.set_xticks(x)
    ax.set_xticklabels(rates, fontsize=10, fontweight='bold')
    ax.set_ylim(0, 12.0)
    ax.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.9, fontsize=9)
    ax.grid(axis='y', linestyle=':', alpha=0.6)

    save_and_sync(fig, "lc3_execution_time_plot.png")

# ==========================================
# PLOT 2: CPU Load (%) Comparison & Zones
# ==========================================
def plot_cpu_load():
    fig, ax = plt.subplots(figsize=(10, 5.5))
    x = np.arange(len(rates))
    width = 0.35

    # Safe / Caution / Danger zones
    ax.axhspan(0, 60, facecolor='#E8F5E9', alpha=0.7, label='Safe Operating Zone (<60% CPU)')
    ax.axhspan(60, 75, facecolor='#FFF9C4', alpha=0.7, label='Marginal / Warning Zone (60-75% CPU)')
    ax.axhspan(75, 100, facecolor='#FFEBEE', alpha=0.7, label='Starvation Risk Zone (>75% CPU)')

    rects1 = ax.bar(x - width/2, cpu_10ms, width, label='10.0 ms Frame Dur', color='#2E7D32', alpha=0.95, edgecolor='black', linewidth=0.5)
    rects2 = ax.bar(x + width/2, cpu_75ms, width, label='7.5 ms Frame Dur', color='#C62828', alpha=0.95, edgecolor='black', linewidth=0.5)

    for rect in rects1:
        h = rect.get_height()
        ax.annotate(f'{h:.1f}%', xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9, fontweight='bold')
    for rect in rects2:
        h = rect.get_height()
        ax.annotate(f'{h:.1f}%', xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9, fontweight='bold')

    ax.set_title('ESP32-C6 CPU Core Load by LC3 Mono Encoder (160 MHz RISC-V)', fontsize=13, fontweight='bold', pad=15)
    ax.set_xlabel('Audio Sample Rate', fontsize=11, labelpad=8)
    ax.set_ylabel('Core Utilization (%)', fontsize=11, labelpad=8)
    ax.set_xticks(x)
    ax.set_xticklabels(rates, fontsize=10, fontweight='bold')
    ax.set_ylim(0, 100)
    ax.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.95, fontsize=8.5)
    ax.grid(axis='y', linestyle=':', alpha=0.6)

    save_and_sync(fig, "lc3_cpu_load_plot.png")

# ====================================================
# PLOT 3: Mono vs Stereo Theoretical Core Load
# ====================================================
def plot_stereo_feasibility():
    fig, ax = plt.subplots(figsize=(10, 5.5))
    x = np.arange(len(rates))
    width = 0.35

    stereo_10ms = [c * 2 for c in cpu_10ms]
    
    rects1 = ax.bar(x - width/2, cpu_10ms, width, label='1x Mono Channel (10 ms, Measured)', color='#0288D1', edgecolor='black', linewidth=0.5)
    rects2 = ax.bar(x + width/2, stereo_10ms, width, label='2x Stereo Channels (10 ms, Projected)', color='#7B1FA2', edgecolor='black', linewidth=0.5)

    ax.axhline(100.0, color='#D32F2F', linestyle='-', linewidth=2.0, label='100% Single Core Hardware Ceiling')
    ax.axhspan(100, 160, facecolor='#FFCDD2', alpha=0.5, label='Impossible on Single Core (>100%)')

    for rect in rects1:
        h = rect.get_height()
        ax.annotate(f'{h:.1f}%', xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=8.5, fontweight='bold')
    for rect in rects2:
        h = rect.get_height()
        color = '#B71C1C' if h > 100 else '#4A148C'
        ax.annotate(f'{h:.1f}%', xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=8.5, fontweight='bold', color=color)

    ax.set_title('Single-Core ESP32-C6 Feasibility: 1-Channel Mono vs 2-Channel Stereo (10 ms)', fontsize=13, fontweight='bold', pad=15)
    ax.set_xlabel('Sample Rate', fontsize=11, labelpad=8)
    ax.set_ylabel('Total CPU Core Load (%)', fontsize=11, labelpad=8)
    ax.set_xticks(x)
    ax.set_xticklabels(rates, fontsize=10, fontweight='bold')
    ax.set_ylim(0, 160)
    ax.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.95, fontsize=8.5)
    ax.grid(axis='y', linestyle=':', alpha=0.6)

    save_and_sync(fig, "lc3_headroom_stereo_comparison.png")

def main():
    print("Generating benchmark plots...")
    plot_execution_times()
    plot_cpu_load()
    plot_stereo_feasibility()
    print("All plots generated successfully!")

if __name__ == '__main__':
    main()
