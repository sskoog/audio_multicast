#!/usr/bin/env python3
"""
generate_wroom32_plots.py
Generates presentation-grade plots for the ESP32-WROOM-32 LC3 Hardware Benchmark,
comparing Fixed-Point vs Floating-Point (Hardware FPU) and comparing against ESP32-C6.
Saves plots to conversation artifact directory and docs/assets/.
"""

import os
import numpy as np
import matplotlib.pyplot as plt

ARTIFACT_DIR = r"C:\Users\stefa\.gemini\antigravity-ide\brain\362c5e0d-1e79-487b-ac18-f1e63260ca67"
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS_ASSETS_DIR = os.path.join(REPO_ROOT, "docs", "assets")

os.makedirs(ARTIFACT_DIR, exist_ok=True)
os.makedirs(DOCS_ASSETS_DIR, exist_ok=True)

# Sample rates for 10ms comparisons
rates_5 = ["48.0 kHz", "32.0 kHz", "24.0 kHz", "16.0 kHz", "8.0 kHz"]

# ESP32-C6 (160 MHz RISC-V FixP) - 10ms standard
c6_avg_10ms = [7.03, 5.56, 5.06, 3.92, 3.05]

# ESP32 WROOM-32 (240 MHz Xtensa FixP) - 10ms standard
esp32_fixp_avg = [8.04, 6.26, 5.59, 4.32, 3.47]
esp32_fixp_p95 = [8.53, 6.67, 5.98, 4.69, 3.79]

# ESP32 WROOM-32 (240 MHz Xtensa Float FPU) - 10ms standard
esp32_fpu_avg  = [6.12, 5.35, 4.75, 3.99, 3.31]
esp32_fpu_p95  = [6.55, 5.75, 5.10, 4.32, 3.64]

# Octet scaling on ESP32 Float FPU @ 10ms (48k, 32k, 24k)
rates_oct = ["48.0 kHz", "32.0 kHz", "24.0 kHz"]
fpu_std_oct = [6.12, 5.35, 4.75]   # 80B, 60B, 45B
fpu_100_oct = [6.34, 5.76, 5.25]   # 100B
fpu_120_oct = [6.48, 5.89, 5.35]   # 120B

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

# =========================================================================
# PLOT 1: ESP32 Fixed-Point vs Floating-Point (Hardware FPU)
# =========================================================================
def plot_esp32_fix_vs_float():
    fig, ax = plt.subplots(figsize=(10, 5.5))
    x = np.arange(len(rates_5))
    width = 0.35

    rects1 = ax.bar(x - width/2, esp32_fixp_avg, width, label='ESP32 Fixed-Point (Espressif)', color='#D32F2F', alpha=0.9, edgecolor='black', linewidth=0.5)
    rects2 = ax.bar(x + width/2, esp32_fpu_avg,  width, label='ESP32 Floating-Point + Hardware FPU (liblc3)', color='#1976D2', alpha=0.9, edgecolor='black', linewidth=0.5)

    # Error bars showing P95
    ax.errorbar(x - width/2, esp32_fixp_avg, yerr=[np.zeros(len(esp32_fixp_avg)), np.array(esp32_fixp_p95) - np.array(esp32_fixp_avg)],
                fmt='none', ecolor='#7F0000', elinewidth=2, capsize=4, label='P95 Peak Bound')
    ax.errorbar(x + width/2, esp32_fpu_avg,  yerr=[np.zeros(len(esp32_fpu_avg)), np.array(esp32_fpu_p95) - np.array(esp32_fpu_avg)],
                fmt='none', ecolor='#0D47A1', elinewidth=2, capsize=4)

    ax.axhline(10.0, color='#333333', linestyle='--', linewidth=1.2, alpha=0.7, label='10.0 ms Real-Time Budget Limit')

    for rect in rects1:
        h = rect.get_height()
        ax.annotate(f'{h:.2f} ms\n({h*10:.1f}%)', xy=(rect.get_x() + rect.get_width() / 2, h + 0.6),
                    ha='center', va='bottom', fontsize=8.5, fontweight='bold')
    for rect in rects2:
        h = rect.get_height()
        ax.annotate(f'{h:.2f} ms\n({h*10:.1f}%)', xy=(rect.get_x() + rect.get_width() / 2, h + 0.6),
                    ha='center', va='bottom', fontsize=8.5, fontweight='bold', color='#0D47A1')

    ax.set_title('ESP32-WROOM-32: Fixed-Point vs Floating-Point (Hardware FPU) LC3 Encoding (240 MHz)', fontsize=12.5, fontweight='bold', pad=15)
    ax.set_xlabel('Audio Sample Rate', fontsize=11, labelpad=8)
    ax.set_ylabel('Frame Execution Time (ms / 10ms frame)', fontsize=11, labelpad=8)
    ax.set_xticks(x)
    ax.set_xticklabels(rates_5, fontsize=10, fontweight='bold')
    ax.set_ylim(0, 12.0)
    ax.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.95, fontsize=9)
    ax.grid(axis='y', linestyle=':', alpha=0.6)

    save_and_sync(fig, "wroom32_fix_vs_float_plot.png")

# =========================================================================
# PLOT 2: 3-Way Architectural Comparison (ESP32-C6 vs ESP32 FixP vs Float)
# =========================================================================
def plot_arch_comparison():
    fig, ax = plt.subplots(figsize=(11, 5.5))
    x = np.arange(len(rates_5))
    width = 0.25

    rects1 = ax.bar(x - width, c6_avg_10ms,   width, label='ESP32-C6 @ 160 MHz (RISC-V Fixed-Point)', color='#388E3C', edgecolor='black', linewidth=0.5)
    rects2 = ax.bar(x,         esp32_fixp_avg, width, label='ESP32 @ 240 MHz (Xtensa Fixed-Point)',   color='#E65100', edgecolor='black', linewidth=0.5)
    rects3 = ax.bar(x + width, esp32_fpu_avg,  width, label='ESP32 @ 240 MHz (Xtensa Hardware FPU)',  color='#0288D1', edgecolor='black', linewidth=0.5)

    ax.axhline(10.0, color='#333333', linestyle='--', linewidth=1.2, alpha=0.7, label='10.0 ms Real-Time Budget Limit')

    for rect in rects1:
        h = rect.get_height()
        ax.annotate(f'{h:.2f}ms', xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 2), textcoords="offset points", ha='center', va='bottom', fontsize=8, fontweight='bold')
    for rect in rects2:
        h = rect.get_height()
        ax.annotate(f'{h:.2f}ms', xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 2), textcoords="offset points", ha='center', va='bottom', fontsize=8, fontweight='bold')
    for rect in rects3:
        h = rect.get_height()
        ax.annotate(f'{h:.2f}ms', xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 2), textcoords="offset points", ha='center', va='bottom', fontsize=8, fontweight='bold', color='#01579B')

    ax.set_title('Cross-SoC LC3 Architectural Performance Comparison (10.0 ms Frame Duration)', fontsize=12.5, fontweight='bold', pad=15)
    ax.set_xlabel('Sample Rate', fontsize=11, labelpad=8)
    ax.set_ylabel('Execution Time per Frame (ms)', fontsize=11, labelpad=8)
    ax.set_xticks(x)
    ax.set_xticklabels(rates_5, fontsize=10, fontweight='bold')
    ax.set_ylim(0, 11.5)
    ax.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.95, fontsize=8.5)
    ax.grid(axis='y', linestyle=':', alpha=0.6)

    save_and_sync(fig, "cross_soc_lc3_comparison.png")

# =========================================================================
# PLOT 3: Payload Scaling (Standard vs 100 Octets vs 120 Octets)
# =========================================================================
def plot_octet_scaling():
    fig, ax = plt.subplots(figsize=(9, 5.5))
    x = np.arange(len(rates_oct))
    width = 0.25

    rects1 = ax.bar(x - width, fpu_std_oct, width, label='Standard Bitrate (45-80 Octets)', color='#4CAF50', edgecolor='black', linewidth=0.5)
    rects2 = ax.bar(x,         fpu_100_oct, width, label='100 Octets/frame (80 kbps)',       color='#FF9800', edgecolor='black', linewidth=0.5)
    rects3 = ax.bar(x + width, fpu_120_oct, width, label='120 Octets/frame (96 kbps)',       color='#9C27B0', edgecolor='black', linewidth=0.5)

    for rect in rects1:
        h = rect.get_height()
        ax.annotate(f'{h:.2f} ms\n({h*10:.1f}%)', xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 2), textcoords="offset points", ha='center', va='bottom', fontsize=8.5, fontweight='bold')
    for rect in rects2:
        h = rect.get_height()
        ax.annotate(f'{h:.2f} ms\n({h*10:.1f}%)', xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 2), textcoords="offset points", ha='center', va='bottom', fontsize=8.5, fontweight='bold')
    for rect in rects3:
        h = rect.get_height()
        ax.annotate(f'{h:.2f} ms\n({h*10:.1f}%)', xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 2), textcoords="offset points", ha='center', va='bottom', fontsize=8.5, fontweight='bold')

    ax.set_title('ESP32 Floating-Point LC3: CPU Load vs Frame Octets (10.0 ms Frame)', fontsize=12.5, fontweight='bold', pad=15)
    ax.set_xlabel('Sample Rate', fontsize=11, labelpad=8)
    ax.set_ylabel('Execution Time per Frame (ms)', fontsize=11, labelpad=8)
    ax.set_xticks(x)
    ax.set_xticklabels(rates_oct, fontsize=10, fontweight='bold')
    ax.set_ylim(0, 9.5)
    ax.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.95, fontsize=9)
    ax.grid(axis='y', linestyle=':', alpha=0.6)

    save_and_sync(fig, "wroom32_octet_scaling_plot.png")

def main():
    print("Generating WROOM-32 benchmark plots...")
    plot_esp32_fix_vs_float()
    plot_arch_comparison()
    plot_octet_scaling()
    print("All WROOM-32 plots generated successfully!")

if __name__ == '__main__':
    main()
