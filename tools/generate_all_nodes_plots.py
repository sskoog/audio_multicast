#!/usr/bin/env python3
"""
generate_all_nodes_plots.py
Generates presentation-grade plots comparing LC3 encoding performance across all 3 SoC nodes:
1. ESP32-C6 (160 MHz RISC-V RV32IMAC - Fixed-Point)
2. ESP32-WROOM-32 (240 MHz Dual-Core Xtensa LX6 - Fixed-Point & Float FPU)
3. XIAO ESP32-S3 (240 MHz Dual-Core Xtensa LX7 - Fixed-Point & Float FPU)
"""

import os
import numpy as np
import matplotlib.pyplot as plt

ARTIFACT_DIR = r"C:\Users\stefa\.gemini\antigravity-ide\brain\362c5e0d-1e79-487b-ac18-f1e63260ca67"
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS_ASSETS_DIR = os.path.join(REPO_ROOT, "docs", "assets")

os.makedirs(ARTIFACT_DIR, exist_ok=True)
os.makedirs(DOCS_ASSETS_DIR, exist_ok=True)

plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
plt.rcParams['font.sans-serif'] = 'Segoe UI, Helvetica, Arial, DejaVu Sans'
plt.rcParams['axes.edgecolor'] = '#CCCCCC'
plt.rcParams['axes.linewidth'] = 0.8

# ==========================================
# 1. Reference Levels Comparison (5 Engines)
# ==========================================
levels = [
    "HIGH\n48 kHz / 7.5 ms\n120 B (128 kbps)",
    "MEDIUM\n32 kHz / 10.0 ms\n80 B (64 kbps)",
    "LOW\n16 kHz / 10.0 ms\n40 B (32 kbps)"
]
budgets = [7.5, 10.0, 10.0]

# Measured Avg ms
c6_fixp    = [6.43, 5.55, 3.89]
esp32_fixp = [7.36, 6.34, 4.47]
esp32_fpu  = [6.00, 5.46, 4.11]
s3_fixp    = [4.30, 4.07, 3.17]
s3_fpu     = [3.20, 3.10, 2.72]

# CPU Load %
c6_fixp_cpu    = [85.8, 55.5, 38.9]
esp32_fixp_cpu = [98.2, 63.4, 44.7]
esp32_fpu_cpu  = [80.0, 54.6, 41.1]
s3_fixp_cpu    = [57.3, 40.7, 31.7]
s3_fpu_cpu     = [42.7, 31.0, 27.2]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

x = np.arange(len(levels))
width = 0.16

r1 = ax1.bar(x - 2*width, c6_fixp,    width, label='C6 (160MHz RISC-V FixP)', color='#2E7D32', edgecolor='black', linewidth=0.5)
r2 = ax1.bar(x - 1*width, esp32_fixp, width, label='ESP32 (240MHz Xtensa LX6 FixP)', color='#E65100', edgecolor='black', linewidth=0.5)
r3 = ax1.bar(x,           esp32_fpu,  width, label='ESP32 (240MHz Xtensa LX6 Float FPU)', color='#0288D1', edgecolor='black', linewidth=0.5)
r4 = ax1.bar(x + 1*width, s3_fixp,    width, label='S3 (240MHz Xtensa LX7 FixP)', color='#8E24AA', edgecolor='black', linewidth=0.5)
r5 = ax1.bar(x + 2*width, s3_fpu,     width, label='S3 (240MHz Xtensa LX7 Float FPU)', color='#D81B60', edgecolor='black', linewidth=0.5)

# Budget lines
for i, b in enumerate(budgets):
    ax1.plot([x[i] - 0.44, x[i] + 0.44], [b, b], color='#212121', linestyle='--', linewidth=1.5)
    ax1.text(x[i], b + 0.2, f'Budget: {b:.1f} ms', ha='center', va='bottom', fontsize=8.5, color='#333333', fontweight='bold')

for rects, color in [(r1, '#1B5E20'), (r2, '#BF360C'), (r3, '#01579B'), (r4, '#4A148C'), (r5, '#880E4F')]:
    for rect in rects:
        h = rect.get_height()
        ax1.annotate(f'{h:.2f}m', xy=(rect.get_x() + rect.get_width() / 2, h),
                     xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=7.5, fontweight='bold', color=color)

ax1.set_title('Frame Execution Time (ms)', fontsize=12, fontweight='bold', pad=12)
ax1.set_ylabel('Execution Time per Frame (ms)', fontsize=10, labelpad=6)
ax1.set_xticks(x)
ax1.set_xticklabels(levels, fontsize=9.5, fontweight='bold')
ax1.set_ylim(0, 11.5)
ax1.legend(loc='upper left', frameon=True, facecolor='white', framealpha=0.95, fontsize=8)
ax1.grid(axis='y', linestyle=':', alpha=0.6)

# Subplot 2: Single Core CPU Load %
ax2.axhspan(0, 50, facecolor='#E8F5E9', alpha=0.6, label='Ideal (<50% CPU)')
ax2.axhspan(50, 75, facecolor='#FFF9C4', alpha=0.6, label='Moderate (50-75% CPU)')
ax2.axhspan(75, 100, facecolor='#FFEBEE', alpha=0.6, label='Starvation Risk (>75% CPU)')

r6  = ax2.bar(x - 2*width, c6_fixp_cpu,    width, label='C6 FixP', color='#388E3C', edgecolor='black', linewidth=0.5)
r7  = ax2.bar(x - 1*width, esp32_fixp_cpu, width, label='ESP32 FixP', color='#F57C00', edgecolor='black', linewidth=0.5)
r8  = ax2.bar(x,           esp32_fpu_cpu,  width, label='ESP32 FPU', color='#039BE5', edgecolor='black', linewidth=0.5)
r9  = ax2.bar(x + 1*width, s3_fixp_cpu,    width, label='S3 FixP', color='#AB47BC', edgecolor='black', linewidth=0.5)
r10 = ax2.bar(x + 2*width, s3_fpu_cpu,     width, label='S3 FPU', color='#EC407A', edgecolor='black', linewidth=0.5)

for rects, color in [(r6, '#1B5E20'), (r7, '#BF360C'), (r8, '#01579B'), (r9, '#4A148C'), (r10, '#880E4F')]:
    for rect in rects:
        h = rect.get_height()
        ax2.annotate(f'{h:.1f}%', xy=(rect.get_x() + rect.get_width() / 2, h),
                     xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=7.5, fontweight='bold', color=color)

ax2.set_title('Single-Core CPU Load (%)', fontsize=12, fontweight='bold', pad=12)
ax2.set_ylabel('CPU Utilization (%)', fontsize=10, labelpad=6)
ax2.set_xticks(x)
ax2.set_xticklabels(levels, fontsize=9.5, fontweight='bold')
ax2.set_ylim(0, 108)
ax2.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.95, fontsize=8)
ax2.grid(axis='y', linestyle=':', alpha=0.6)

fig.suptitle('LC3 Reference Levels Performance Across All 3 SoCs (ESP32-C6, ESP32, ESP32-S3)', fontsize=13, fontweight='bold', y=0.98)
plt.tight_layout()

for out_dir in [ARTIFACT_DIR, DOCS_ASSETS_DIR]:
    p = os.path.join(out_dir, "all_nodes_lc3_reference_levels_plot.png")
    fig.savefig(p, dpi=300, bbox_inches='tight')
    print(f"Saved: {p}")
plt.close(fig)

# =======================================================
# 2. Sample Rate Scaling (48k, 32k, 24k, 16k, 8k @ 10ms)
# =======================================================
sr_labels = ["48 kHz\n(80 B)", "32 kHz\n(60 B)", "24 kHz\n(45 B)", "16 kHz\n(30 B)", "8 kHz\n(20 B)"]
sr_c6_fixp    = [7.05, 5.45, 4.96, 3.77, 2.95]
sr_esp32_fixp = [8.04, 6.22, 5.56, 4.41, 3.78]
sr_esp32_fpu  = [6.12, 5.34, 5.02, 4.09, 3.65]
sr_s3_fixp    = [4.80, 4.05, 3.74, 3.15, 2.70]
sr_s3_fpu     = [3.48, 3.08, 2.93, 2.70, 2.47]

fig, ax = plt.subplots(figsize=(11, 5.5))
x_sr = np.arange(len(sr_labels))

ax.plot(x_sr, sr_esp32_fixp, marker='s', linewidth=2, color='#E65100', label='ESP32 (240MHz Xtensa LX6 FixP)')
ax.plot(x_sr, sr_c6_fixp,    marker='^', linewidth=2, color='#2E7D32', label='ESP32-C6 (160MHz RISC-V FixP)')
ax.plot(x_sr, sr_esp32_fpu,  marker='o', linewidth=2, color='#0288D1', label='ESP32 (240MHz Xtensa LX6 Float FPU)')
ax.plot(x_sr, sr_s3_fixp,    marker='D', linewidth=2, color='#8E24AA', label='ESP32-S3 (240MHz Xtensa LX7 FixP)')
ax.plot(x_sr, sr_s3_fpu,     marker='*', markersize=9, linewidth=2.5, color='#D81B60', label='ESP32-S3 (240MHz Xtensa LX7 Float FPU)')

ax.axhline(10.0, color='red', linestyle='--', linewidth=1.5, label='10.0 ms Real-Time Deadline')

for x_val, y_val in zip(x_sr, sr_s3_fpu):
    ax.annotate(f'{y_val:.2f}ms', (x_val, y_val), textcoords="offset points", xytext=(0, -14), ha='center', fontsize=8.5, fontweight='bold', color='#880E4F')
for x_val, y_val in zip(x_sr, sr_esp32_fixp):
    ax.annotate(f'{y_val:.2f}ms', (x_val, y_val), textcoords="offset points", xytext=(0, 6), ha='center', fontsize=8.5, fontweight='bold', color='#BF360C')

ax.set_title('LC3 Encoder Frame Execution Time Across Sample Rates (10.0 ms Frames)', fontsize=12, fontweight='bold', pad=12)
ax.set_ylabel('Execution Time per Frame (ms)', fontsize=10, labelpad=6)
ax.set_xlabel('Sample Rate & Standard Octets', fontsize=10, labelpad=6)
ax.set_xticks(x_sr)
ax.set_xticklabels(sr_labels, fontsize=9.5, fontweight='bold')
ax.set_ylim(1.5, 10.8)
ax.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.95, fontsize=9)
ax.grid(True, linestyle=':', alpha=0.6)
plt.tight_layout()

for out_dir in [ARTIFACT_DIR, DOCS_ASSETS_DIR]:
    p = os.path.join(out_dir, "all_nodes_sample_rate_comparison_plot.png")
    fig.savefig(p, dpi=300, bbox_inches='tight')
    print(f"Saved: {p}")
plt.close(fig)
