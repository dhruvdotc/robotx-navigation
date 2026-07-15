# Team setup checklist

1. **Mac:** clone this repo, run `bash mavlink_comms/scripts/setup_mavlink_env.sh` once (or let `run_gcs_mac.sh` create `.venv-mavlink`).
2. **Jetson:** clone to `~/robotx-navigation`, run `bash jetson_setup.sh`.
3. **Network:** both devices on same router WiFi (not UCSD-GUEST). See `scripts/jetson_wifi_setup.sh`.
4. **Demo:** Mac `run_gcs_mac.sh` first, then Jetson `GCS_IP=<mac-ip> run_detection_jetson.sh`.
5. **Success:** Jetson `[TX]` lines match Mac `[GCS]` JSON on UDP port 14555.

Do not commit router passwords, Jetson SSH passwords, or local `detection_logs/` video dumps.
