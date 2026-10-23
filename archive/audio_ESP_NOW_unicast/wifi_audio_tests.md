# Test suite for audio_ESP_NOW_unicast

## Assumptions
1. The system consists of one SOURCE node and 1-6 SINK nodes.
   - Nodes are based on ESP32 -S3 or C6. SOURCE node must be ESP32-S3 due to the increased compute for LC3 encoder.
2. Nodes are connected to PC via USB (virtual serial port)
3. Nodes have a 1 Hz heartbeat telemetry stats printing feature enabled, showing detailed stats about the audio pipeline
4. Nodes can be controlled via serial port using specific command set, e.g. change audio mode, hard reset, change system state.

## Continious audio streaming test & clock drift
Purpose:
* Verify that the system can run continiously for an extended period of time 

Procedure:
1. Start audio streaming using local synth mode (SOURCE node generates sine waves and local LC3 encoding)
2. Let the system run for an extended period of time (at least 10 minutes)
3. Record the serial output of all nodes
4. Stop the streaming
5. Evaluate the recorded serial output for errors, all incremental buffer counters.
6. TBD: Evaluate time drift between SINK nodes

Pass criteria:
* No state changes out from STREAMING for SInnK nodes.
* No state changes for SOURCE node.
* PLC for SINK nodes below 60 (one per 10 seconds on average)
* No FIFO overflow or underflow errors
* TBD: Time difference between any two SINK nodes should be less than 0.1 ms.

## Bumble streaming
Purpose:
* Verify that the SOURCE can stream audio from a PC, acting as an audio USB dongle.

Procedure:
1. Start audio streaming via PC Bumble-app/script. PC does the LC3 encoding, SOURCE-node acts as a ESP-NOW unicast sender.
2. Use any dynamic audio source from the PC, e.g. YouTube Music, Spotify, local mp3, internet radio. Let play continiously for 10 minutes.
3. Record the serial output of all nodes
4. Stop the streaming

Pass criteria:
* No state changes out from STREAMING for SINK nodes.
* No state changes for SOURCE node during streaming.
* PLC for SINK nodes below 60 (one per 10 seconds on average)
* No FIFO overflow or underflow errors


## SINK drop-out test
Purpose:
* Verify that the system can handle SINK node drop-outs gracefully

Procedure:
1. Start audio streaming with 2-6 SINK nodes
2. Let run for 5 seconds.
3. Monitor SOURCE and SINK1 telemetry stats 
4. Reset SINK2 via serial com, hold reset for 5 seconds.
5. Enable/release reset of SINK2.
6. Monitor stats for all nodes

Pass criteria:
* SOURCE registers SINK2 drop-out and stops sending audio to SINK2
* SINK1 is not affected and keeps streaming: No packet loss, no PLC, no state changes
* Once SINK2 is back online, it handshakes with SOURCE node and immediately starts streaming again
* SINK1 audio stream is not affected by SINK2 re-appearing (no drop-outs)


## Live change of sample rate
Purpose:
* Verify that the system can change sample rate during streaming without artefacts
* Verify sample rates: 8, 16, 24, 32, 48 kHz

Procedure:
1. Start audio streaming using local synth mode (SOURCE node generates sine waves and local LC3 encoding) with default sample rate
2. Let the system run for 5 seconds.
3. Monitor telemetry of all nodes via COM-ports
4. Change sample rate on SOURCE node via serial command
5. Let the system run for 5 seconds.
6. Record telemetry status for all nodes.
7. Repeat 2-6 for all sample rates
8. Evaluate the recorded serial output for errors, all incremental buffer counters.

Pass criteria:
* No state changes for SOURCE node.
* All sinks register the sample rate change
* Allowed states for SINK nodes: STREAMING, PREFILL
* PLC for SINK nodes below 5
* FIFO and DMA counters are reset during the state change
