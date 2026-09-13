#!/usr/bin/env python3
"""
generate_c6_reference_plot.py
Generates presentation plots comparing the 3 Reference Encoding Levels (High, Medium, Low)
across ESP32-C6 (160 MHz RISC-V) and ESP32-WROOM-32 (240 MHz Fixed-Point & Float FPU).
"""

import os
import numpy as np
import matplotlib.pyplot as plt

ARTIFACT_DIR = r"C:\Users\stefa\.gemini\antigravity-ide\brain\362c5e0d-1e79-487b-ac18-f1e63260ca67"
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS_ASSETS_DIR = os.path.join(REPO_ROOT, "docs", "assets")

os.makedirs(ARTIFACT_DIR, exist_ok=True)
os.makedirs(DOCS_ASSETS_DIR, exist_ok=True)

levels = [
    "HIGH\n48 kHz / 7.5 ms\n120 B (128 kbps)",
    "MEDIUM\n32 kHz / 10.0 ms\n80 B (64 kbps)",
    "LOW\n16 kHz / 10.0 ms\n40 B (32 kbps)"
]

budgets = [7.5, 10.0, 10.0]

# Execution times in ms
c6_fixp_avg    = [6.43, 5.55, 3.89]
c6_fixp_p95    = [6.98, 6.31, 4.53]
c6_fixp_cpu    = [85.8, 55.5, 38.9]

esp32_fixp_avg = [7.36, 6.34, 4.47]
esp32_fixp_p95 = [7.76, 6.78, 4.83]
esp32_fixp_cpu = [98.2, 63.4, 44.7]

esp32_fpu_avg  = [6.00, 5.46, 4.11]
esp32_fpu_p95  = [6.45, 5.90, 4.47]
esp32_fpu_cpu  = [80.0, 54.6, 41.1]

plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
plt.rcParams['font.sans-serif'] = 'Segoe UI, Helvetica, Arial, DejaVu Sans'
plt.rcParams['axes.edgecolor'] = '#CCCCCC'
plt.rcParams['axes.linewidth'] = 0.8

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.8))

x = np.arange(len(levels))
width = 0.26

# Subplot 1: Frame Execution Time (ms)
r1 = ax1.bar(x - width, c6_fixp_avg,    width, label='ESP32-C6 @ 160MHz (RISC-V FixP)', color='#2E7D32', edgecolor='black', linewidth=0.5)
r2 = ax1.bar(x,         esp32_fixp_avg, width, label='ESP32 @ 240MHz (Xtensa FixP)',     color='#E65100', edgecolor='black', linewidth=0.5)
r3 = ax1.bar(x + width, esp32_fpu_avg,  width, label='ESP32 @ 240MHz (Xtensa Float FPU)',color='#0288D1', edgecolor='black', linewidth=0.5)

# Error bars showing P95
ax1.errorbar(x - width, c6_fixp_avg,    yerr=[np.zeros(3), np.array(c6_fixp_p95) - np.array(c6_fixp_avg)],       fmt='none', ecolor='#1B5E20', elinewidth=2, capsize=3)
ax1.errorbar(x,         esp32_fixp_avg, yerr=[np.zeros(3), np.array(esp32_fixp_p95) - np.array(esp32_fixp_avg)], fmt='none', ecolor='#BF360C', elinewidth=2, capsize=3)
ax1.errorbar(x + width, esp32_fpu_avg,  yerr=[np.zeros(3), np.array(esp32_fpu_p95) - np.array(esp32_fpu_avg)],   fmt='none', ecolor='#01579B', elinewidth=2, capsize=3)

# Budget indicators
for i, b in enumerate(budgets):
    ax1.plot([x[i] - 0.42, x[i] + 0.42], [b, b], color='#212121', linestyle='--', linewidth=1.5)
    ax1.text(x[i], b + 0.18, f'Budget: {b:.1f} ms', ha='center', va='bottom', fontsize=8.5, color='#333333', fontweight='bold')

for rect in r1:
    h = rect.get_height()
    ax1.annotate(f'{h:.2f}ms', xy=(rect.get_x() + rect.get_width() / 2, h + 0.45),
                 ha='center', va='bottom', fontsize=8, fontweight='bold', color='#1B5E20')
for rect in r2:
    h = rect.get_height()
    ax1.annotate(f'{h:.2f}ms', xy=(rect.get_x() + rect.get_width() / 2, h + 0.45),
                 ha='center', va='bottom', fontsize=8, fontweight='bold', color='#BF360C')
for rect in r3:
    h = rect.get_height()
    ax1.annotate(f'{h:.2f}ms', xy=(rect.get_x() + rect.get_width() / 2, h + 0.45),
                 ha='center', va='bottom', fontsize=8, fontweight='bold', color='#01579B')

ax1.set_title('Frame Execution Time (ms)', fontsize=12, fontweight='bold', pad=12)
ax1.set_ylabel('Execution Time per Frame (ms)', fontsize=10, labelpad=6)
ax1.set_xticks(x)
ax1.set_xticklabels(levels, fontsize=9.5, fontweight='bold')
ax1.set_ylim(0, 11.5)
ax1.legend(loc='upper left', frameon=True, facecolor='white', framealpha=0.95, fontsize=8.5)
ax1.grid(axis='y', linestyle=':', alpha=0.6)

# Subplot 2: Single-Core CPU Load (%)
ax2.axhspan(0, 60, facecolor='#E8F5E9', alpha=0.6, label='Safe Zone (<60% CPU)')
ax2.axhspan(60, 75, facecolor='#FFF9C4', alpha=0.6, label='Warning Zone (60-75% CPU)')
ax2.axhspan(75, 100, facecolor='#FFEBEE', alpha=0.6, label='Starvation Risk (>75% CPU)')

r4 = ax2.bar(x - width, c6_fixp_cpu,    width, label='C6 (RISC-V FixP)', color='#388E3C', edgecolor='black', linewidth=0.5)
r5 = ax2.bar(x,         esp32_fixp_cpu, width, label='ESP32 (Xtensa FixP)', color='#F57C00', edgecolor='black', linewidth=0.5)
r6 = ax2.bar(x + width, esp32_fpu_cpu,  width, label='ESP32 (Float FPU)', color='#039BE5', edgecolor='black', linewidth=0.5)

for rect in r4:
    h = rect.get_height()
    ax2.annotate(f'{h:.1f}%', xy=(rect.get_x() + rect.get_width() / 2, h),
                 xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=8, fontweight='bold', color='#1B5E20')
for rect in r5:
    h = rect.get_height()
    ax2.annotate(f'{h:.1f}%', xy=(rect.get_x() + rect.get_width() / 2, h),
                 xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=8, fontweight='bold', color='#BF360C')
for rect in r6:
    h = rect.get_height()
    ax2.annotate(f'{h:.1f}%', xy=(rect.get_x() + rect.get_width() / 2, h),
                 xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=8, fontweight='bold', color='#01579B')

ax2.set_title('Single-Core CPU Load (%)', fontsize=12, fontweight='bold', pad=12)
ax2.set_ylabel('CPU Utilization (%)', fontsize=10, labelpad=6)
ax2.set_xticks(x)
ax2.set_xticklabels(levels, fontsize=9.5, fontweight='bold')
ax2.set_ylim(0, 108)
ax2.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.95, fontsize=8.5)
ax2.grid(axis='y', linestyle=':', alpha=0.6)

fig.suptitle('Cross-SoC Performance for Reference Encoding Levels (ESP32-C6 vs ESP32-WROOM-32)', fontsize=13, fontweight='bold', y=0.98)
plt.tight_layout()

for out_dir in [ARTIFACT_DIR, DOCS_ASSETS_DIR]:
    p = os.path.join(out_dir, "reference_levels_c6_vs_esp32_plot.png")
    fig.savefig(p, dpi=300, bbox_inches='tight')
    print(f"Saved: {p}")
plt.close(fig)
