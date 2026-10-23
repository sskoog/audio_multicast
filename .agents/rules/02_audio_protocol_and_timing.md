---
trigger: always_on
---

# Rule: Audio Protocol & Timing Standards

1. **Hardware Timer Pacing**:
   - Periodic audio generation tasks MUST calculate absolute next execution deadlines using `esp_timer_get_time()`.
   - Never use relative delays (`vTaskDelay`) or FreeRTOS tick-based pacing for LC3 packet generation, as FreeRTOS tick granularity (100 Hz / 10 ms or 1000 Hz / 1 ms) causes cumulative frame creep and sink FIFO underruns.

2. **Buffer and Packet Sizing**:
   - Max LC3 frame octets: 120 bytes. 
   - Max PCM samples per frame: 480 samples (10 ms @ 48 kHz).
   - I2S DMA must contain exactly two descriptors to minimize total audio latency.
   - A LC3 FIFO between 802.11 RX and audio decode should absorb most of the temporal buffer.

3. **Packet Loss Concealment (PLC)**:
   - SINK nodes must track missing sequence numbers.
   - If LC3 packets are missing, invoke `lc3_decode(..., NULL, pcm_out)` for standard PLC interpolation.

4. **Per-Node Audio Gain Calibration**:
   - SINK nodes with the same DAC hardware may exhibit volume differences due to component tolerances (amplifier gain, driver impedance).
   - Per-node software post-gain calibration is implemented in `main.cpp` via a `switch` on `matched_node->node_id` inside the `is_pcm5102a` block.
   - When adding new SINK nodes, measure relative volume and add calibration entries to the switch statement.
   - The boot log prints the applied post-gain value per node for verification.