#!/usr/bin/env bash
#
# calibrate_mass.sh : interactive load-cell calibration through the Nexus bridge.
#
# Procedure:
#   1. slope := 1.0  -> the published "mass" becomes raw offset-corrected counts
#                       (0 would flatline the topic and make step 4 impossible)
#   2. tare with the scale EMPTY
#   3. operator places a known reference mass
#   4. wait for a steady reading, average it
#   5. slope := reference / counts  -> sent to the MCU + printed for hardcoding
#
# The MCU keeps the new slope in RAM only: hardcode the printed value in
# MassThread.h (MassType::slope) to survive a reset.
#
# Usage: ./calibrate_mass.sh <cell-id 0|1> <reference-mass-grams>

set -euo pipefail

ID="${1:?usage: $0 <cell-id 0|1> <reference-mass-grams>}"
REF="${2:?usage: $0 <cell-id 0|1> <reference-mass-grams>}"

TOPIC_REQ=/EL/mass_req
TOPIC_PKT=/EL/mass_packet
MSG=custom_msg/msg/MassRequest
NSAMPLES=8      # samples per steadiness check (10 Hz -> ~1 s window)
TOL_PCT=2       # steady when (max-min) <= TOL_PCT% of |mean|
TRIES=15        # steadiness retries (moving average needs ~2 s to flush)

send_req() { # $1=tare $2=change_scale $3=scale
    ros2 topic pub --once "$TOPIC_REQ" "$MSG" \
        "{id: $ID, tare: $1, change_scale: $2, scale: $3}" >/dev/null
}

# print NSAMPLES mass values for our cell, one per line
grab() {
    # `|| true`: when awk exits after n samples, `ros2 topic echo` dies of
    # SIGPIPE; under pipefail that non-zero status would errexit the script.
    { timeout 20 ros2 topic echo "$TOPIC_PKT" 2>/dev/null || true; } \
        | awk -v want="$ID" -v n="$NSAMPLES" '
        /^id:/   { id = $2 }
        /^mass:/ { if (id == want) { print $2; if (++c == n) exit } }'
}

# stdin: samples -> mean, or "unstable"/"none"
steady() {
    awk -v tol="$TOL_PCT" '
        { s += $1; if (NR == 1 || $1 < lo) lo = $1; if (NR == 1 || $1 > hi) hi = $1 }
        END {
            if (NR == 0) { print "none"; exit }
            m = s / NR; span = hi - lo
            am = (m < 0) ? -m : m
            if (am < 1e-9 || span > am * tol / 100) print "unstable"
            else printf "%.6f\n", m
        }'
}

measure_steady() {
    local val i
    for ((i = 1; i <= TRIES; i++)); do
        val=$(grab | steady)
        case "$val" in
            none)     echo "no packets for cell $ID on $TOPIC_PKT - is the bridge running?" >&2; exit 1 ;;
            unstable) echo "  not steady yet (try $i/$TRIES)..." >&2 ;;
            *)        echo "$val"; return ;;
        esac
    done
    echo "reading never settled - check for vibration or a railed sensor (0x7FFFFF)" >&2
    exit 1
}

echo "== load-cell calibration: cell $ID, reference ${REF} g =="

echo "[1/4] slope := 1.0 (topic now shows raw counts)"
send_req false true 1.0
sleep 1

read -rp "[2/4] EMPTY the scale, then press Enter to tare... "
send_req true false 0.0
sleep 4   # tare averages 20 samples at 10 SPS

read -rp "[3/4] place the ${REF} g reference on the scale, then press Enter... "
echo "  waiting for a steady reading..."
counts=$(measure_steady)
echo "  steady raw delta: $counts counts"
case "$counts" in -*)
    echo "  note: counts DECREASE with load (A+/A- inverted or cell loaded backwards)."
    echo "        harmless: the slope will be negative and weights still read positive."
esac

scale=$(awk -v m="$REF" -v c="$counts" 'BEGIN {
    ac = (c < 0) ? -c : c
    if (ac < 1) { print "bad"; exit }
    printf "%.10f\n", m / c }')
if [ "$scale" = "bad" ]; then
    echo "raw delta is ~zero: reference not detected (wrong cell id, or tare ate it)" >&2
    exit 1
fi

echo "[4/4] sending new slope: $scale"
send_req false true "$scale"
sleep 2

verif=$(grab | steady)
echo "verification (expect ~${REF} g): $verif g"

# a reading still equal to the raw counts means the MCU ignored the request
# (classic cause: firmware older than the 7-byte MassRequest -> silent no-op)
ok=$(awk -v v="$verif" -v r="$REF" 'BEGIN {
    d = v - r; if (d < 0) d = -d
    print (d <= r * 0.10) ? "yes" : "no" }')
if [ "$ok" != "yes" ]; then
    echo
    echo "CALIBRATION FAILED: verification is not within 10% of the reference." >&2
    echo "The MCU is likely ignoring MassRequests (firmware/bridge wire-format" >&2
    echo "mismatch - reflash the firmware) or the reference moved. Nothing valid" >&2
    echo "was calibrated; do not hardcode this value." >&2
    exit 1
fi

echo
echo "done. to survive resets, hardcode in MassThread.h:"
echo "    float slope = ${scale}f;"
