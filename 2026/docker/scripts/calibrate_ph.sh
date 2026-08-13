#!/usr/bin/env bash
#
# calibrate_ph.sh : interactive 3-point pH calibration through the Nexus bridge.
#
# Procedure:
#   1. slope := 1.0, offset := 0.0  -> the published "ph" becomes the raw ADS1114
#                                      differential reading in VOLTS, which is what
#                                      we actually need to fit against
#   2. operator dips the probe in each of three buffers of known pH, one at a time
#   3. wait for a steady reading in each, average it
#   4. least-squares fit ph = slope * volts + offset through the three points
#   5. send the fitted line to the MCU, then check it back against every buffer
#
# Three points rather than two because the third is the only thing that can tell
# you the electrode is BAD. Two points always fit a line perfectly, so a lazy or
# contaminated electrode produces a beautiful calibration and a wrong reading; the
# third point's residual is what catches it. That is also why this reports the
# slope as a percentage of the theoretical Nernst response - see below.
#
# Why the volts come back over the pH topic: PhPacket carries only the finished pH
# value, no raw field, because the MCU owns the volts -> pH conversion. Setting the
# calibration to the identity (slope 1, offset 0) turns that conversion into a
# passthrough, so the topic reports volts without any change to the wire format.
# Same trick as calibrate_mass.sh step 1.
#
# The MCU keeps the new calibration in RAM only. To survive a reset, put the
# printed values in ph_cal.yaml (Nexus replays them on every link-up) or, as the
# fallback used when nothing is configured there, in pHMeterThread.h.
#
# Usage: ./calibrate_ph.sh <pH_a> <pH_b> <pH_c>
#   e.g. ./calibrate_ph.sh 4.01 6.86 9.18     (common NIST-traceable buffer set)
#
# Order does not matter, but the buffers must span a real range - see PH_SPAN_MIN.

set -euo pipefail

PH1="${1:?usage: $0 <pH_a> <pH_b> <pH_c>   e.g. $0 4.01 6.86 9.18}"
PH2="${2:?usage: $0 <pH_a> <pH_b> <pH_c>   e.g. $0 4.01 6.86 9.18}"
PH3="${3:?usage: $0 <pH_a> <pH_b> <pH_c>   e.g. $0 4.01 6.86 9.18}"

TOPIC_REQ=/EL/ph_req
TOPIC_PKT=/EL/ph_packet
MSG=custom_msg/msg/PhRequest

NSAMPLES=6         # samples per steadiness check (1.6 Hz -> ~3.75 s window)
TOL_V=0.002        # steady when (max-min) <= 2 mV, about 0.034 pH
TOL_PH=0.05        # same test in the pH domain, for the read-back at the end
TRIES=60           # steadiness retries (~3.75 s each -> gives up after ~4 min)
SETTLE_S=60        # dead time after a dip, before we even start checking

# TWO tolerances because the topic reports two different quantities during a run:
# volts under the identity calibration (steps 1-4), pH under the fitted one (step
# 6 onwards). One number cannot serve both. TOL_V/16.9 is about 0.034 pH, so
# reusing it on the read-back would demand 34 uV of electrode stability - which no
# glass electrode delivers, and which fails SOFT, silently skipping the one check
# that catches firmware ignoring the request. TOL_PH is that same 2 mV of real
# noise with the headroom the pH domain actually needs.

# SETTLE_S pays for two things. The MCU averages PH_AVG_SIZE=10 samples at ~1.6 Hz,
# so a 6.25 s window has to flush after every dip before the mean stops carrying
# the previous buffer - and then the electrode itself has to equilibrate, which is
# the far longer of the two. Note the averaging also makes consecutive samples
# correlated (they share 9 of 10 entries), so steady()'s max-min understates real
# movement: the dead time, not the tolerance, is what protects the fit. Cutting it
# short does not fail loudly; it quietly fits a line through half-transitioned
# readings, which is the worst possible outcome.
#
# SETTLE_S/TRIES were 20/20 and that was not enough. Moving a probe into a new
# buffer swings it tens of mV in the first minute before it asymptotes, so the
# old 20 s dead time started checking while the transient was still running and
# the old 20 retries gave up ~75 s later - the run died mid-calibration and the
# cleanup trap fired with the identity calibration still on the MCU. 60/60 rides
# the transient out and gives up after ~4 min per buffer instead.
#
# The 2 mV criterion itself is deliberately UNCHANGED. Widening it would have
# made the script finish too, by accepting readings that had not settled - which
# is the failure the paragraph above is about. More patience, not more slack.
#
# What this does NOT fix: an electrode whose OFFSET is still walking over hours,
# which is what a probe that was stored dry does while its gel layer forms. That
# drift is ~2 mV/hour, i.e. ~0.0005 mV/s, about a thousand times slower than the
# 0.53 mV/s this gate can even see - so every steadiness check passes and the fit
# is still a snapshot of a moving target. The tell is a slope that matches the
# previous calibration while the offset does not. Soak the probe for a few hours
# first; no tolerance in this file can detect that, let alone correct it.

NERNST_MV=59.16    # theoretical mV per pH at 25 C, ideal glass electrode
PH_SPAN_MIN=2.0    # buffers must span at least this much pH for the fit to mean anything
RESID_MAX=0.30     # max acceptable |residual| at any buffer, in pH
                   # Least squares spreads one bad point across all three, so a
                   # single buffer wrong by d shows up as a worst residual of
                   # about 2d/3. This threshold therefore trips at d ~ 0.45 pH:
                   # tight enough to catch a stale or cross-contaminated buffer,
                   # loose enough to ignore ordinary electrode nonlinearity.
VERIFY_TOL=0.30    # live read-back must land within this of the last buffer, in pH
RESPONSE_MIN=30    # measured volt span must be >= this % of the theoretical span

send_cal() { # $1=slope $2=offset
    # --times 5, NOT --once. The subscription is best-effort (Nexus.cpp:30,
    # KeepLast(1).best_effort()), so the transport gives no delivery guarantee, and
    # --once destroys the publisher the instant it has handed the message over. A
    # message still in flight when the node exits is dropped - no retry, no error,
    # exit status 0. `ros2 topic pub` cannot tell you otherwise: -w waits for a
    # matching SUBSCRIPTION, which is discovery, not delivery.
    #
    # That is why this fails intermittently rather than never. Five copies at 10 Hz
    # cost half a second, and change_cal is idempotent - the MCU assigning the same
    # slope/offset five times is indistinguishable from doing it once.
    ros2 topic pub --times 5 --rate 10 "$TOPIC_REQ" "$MSG" \
        "{change_cal: true, slope: $1, offset: $2}" >/dev/null
}

# print NSAMPLES values from the pH topic, one per line
grab() {
    # `|| true`: when awk exits after n samples, `ros2 topic echo` dies of
    # SIGPIPE; under pipefail that non-zero status would errexit the script.
    { timeout 40 ros2 topic echo "$TOPIC_PKT" 2>/dev/null || true; } \
        | awk -v n="$NSAMPLES" '/^ph:/ { print $2; if (++c == n) exit }'
}

# $1=tolerance; stdin: samples -> mean, or "unstable"/"none"
#
# ABSOLUTE tolerance, unlike calibrate_mass.sh. The quantity being measured here
# passes through zero (0 V is pH 7 on an ideal electrode), so a percentage-of-mean
# tolerance would be unsatisfiable near neutral and far too loose at the ends.
#
# The tolerance is an argument, not $TOL_V, because the caller knows which domain
# the topic is in and this function cannot - see the TOL_V/TOL_PH note above.
steady() {
    awk -v tol="$1" '
        { s += $1; if (NR == 1 || $1 < lo) lo = $1; if (NR == 1 || $1 > hi) hi = $1 }
        END {
            if (NR == 0) { print "none"; exit }
            if (hi - lo > tol) print "unstable"
            else printf "%.6f\n", s / NR
        }'
}

measure_steady() { # $1=tolerance, in whatever unit the topic is currently reporting
    local val i
    for ((i = 1; i <= TRIES; i++)); do
        val=$(grab | steady "$1")
        case "$val" in
            none)     echo "no packets on $TOPIC_PKT - is the bridge running, and does this board carry the probe?" >&2; exit 1 ;;
            unstable) echo "  not steady yet (try $i/$TRIES)..." >&2 ;;
            *)        echo "$val"; return ;;
        esac
    done
    echo "reading never settled - check the probe is fully immersed, the solution is still," >&2
    echo "and the BNC shell is connected (a floating input wanders forever)" >&2
    exit 1
}

# Leaving the MCU on the identity transform is a silent trap: the topic keeps
# publishing, but it publishes volts labelled as pH. Say so on every abnormal exit
# - but ONLY while that is actually true. The window opens when step 1 sends the
# identity and closes when step 6 sends the fit, so this is gated on its own flag
# rather than on "did we finish": an exit at the span check has sent the MCU
# nothing at all, and an exit at the residual or read-back check leaves it holding
# the FITTED line. Claiming volts in either case sends the next person hunting the
# wrong fault.
IDENTITY_LOADED=0
cleanup() {
    if [ "$IDENTITY_LOADED" -eq 1 ]; then
        echo >&2
        echo "NOTE: the MCU may still hold the identity calibration from step 1, so" >&2
        echo "      $TOPIC_PKT is reporting VOLTS, not pH. Restart the bridge to" >&2
        echo "      replay ph_cal.yaml, or re-run this script to completion." >&2
    fi
}
trap cleanup EXIT

echo "== pH calibration: buffers $PH1, $PH2, $PH3 =="

# Reject buffers too close together before asking the operator to do any work: the
# fit is a division by the volt span, so a narrow span amplifies noise without bound.
span_ok=$(awk -v a="$PH1" -v b="$PH2" -v c="$PH3" -v m="$PH_SPAN_MIN" 'BEGIN {
    lo = a; hi = a
    if (b < lo) lo = b; if (b > hi) hi = b
    if (c < lo) lo = c; if (c > hi) hi = c
    print (hi - lo >= m) ? "yes" : "no" }')
if [ "$span_ok" != "yes" ]; then
    echo "buffers span less than $PH_SPAN_MIN pH - a fit over that range is noise." >&2
    echo "Use a spread set, e.g. 4.01 6.86 9.18." >&2
    exit 1
fi

# Resolve the message types before touching the hardware. `ros2 topic pub` reports
# an unavailable type as the bare line "The passed message type is invalid" and
# exits non-zero, which under set -e kills the run at step 1 with no hint at all.
#
# get_message(), NOT `ros2 interface show`: that verb looks for
# share/custom_msg/msg/<Name>.msg, but custom_msg keeps its sources in
# msg/avionics/, so they install to share/custom_msg/avionics/ and only the
# generated .idl lands in msg/. The verb falls back to the .idl and its parser
# chokes on the leading "// generated from" comment - so it fails for EVERY
# message in the package, in a completely healthy workspace. get_message() is what
# `ros2 topic pub` itself calls, so it is the only probe that answers the question
# being asked.
if ! python3 -c '
import sys
from rosidl_runtime_py.utilities import get_message
for t in sys.argv[1:]:
    get_message(t)
' "$MSG" custom_msg/msg/PhPacket >/dev/null 2>&1; then
    echo "custom_msg does not resolve in this shell." >&2
    echo >&2
    echo "This is almost always the wrong shell rather than a bad build - the host has" >&2
    echo "ROS 2 at /opt/ros/humble but no custom_msg, so running this script outside" >&2
    echo "the container fails here every time. Check you are inside it:" >&2
    echo "    ./run.sh      (or ./attach.sh for a second shell)" >&2
    echo >&2
    echo "If you are already inside and it still fails, the overlay is not sourced:" >&2
    echo "    source ~/dev_ws/install/setup.bash" >&2
    echo >&2
    echo "and if THAT does not fix it, install/ predates the message - src/ is" >&2
    echo "bind-mounted but generated interfaces are not, so a new .msg needs a build:" >&2
    echo "    cd ~/dev_ws && colcon build --packages-select custom_msg avionics_nexus" >&2
    exit 1
fi

echo "[1/6] slope := 1.0, offset := 0.0 (topic now shows raw volts)"
send_cal 1.0 0.0
IDENTITY_LOADED=1
sleep 2   # round trip + a couple of conversions; no filter window to flush now

# Confirm the MCU APPLIED it, rather than trusting a publish that structurally
# cannot report delivery (see send_cal). If the identity never lands, steps 2-4
# sample pH while believing they are sampling volts, and the fit is a line through
# the wrong quantity: buffer pH against itself, so slope 1.0 pH/V, offset 0, and
# residuals of exactly zero. The residual guard is blind to it.
#
# It is caught eventually - the health check reports ~1690% of theoretical and the
# step-6 read-back fails - but only after the operator has rinsed and settled three
# buffers for nothing. Failing here costs four seconds instead of ten minutes, and
# names the actual cause instead of the two symptoms.
#
# The test is a range check, because the two domains cannot overlap: under the
# identity the topic carries volts, which the ADS1114's +-0.512 V PGA bounds, and
# any usable calibration buffer reads pH 4-9. One sample separates them.
first_v=$(grab)
first_v=${first_v%%$'\n'*}
if [ -z "$first_v" ]; then
    echo "no packets on $TOPIC_PKT - is the bridge running, and does this board carry the probe?" >&2
    exit 1
fi
if [ "$(awk -v v="$first_v" 'BEGIN { if (v < 0) v = -v; print (v <= 0.6) ? "yes" : "no" }')" != "yes" ]; then
    echo >&2
    echo "CALIBRATION FAILED: the identity calibration was not applied." >&2
    echo "$TOPIC_PKT still reads ${first_v}, which is a pH value - under slope 1.0 /" >&2
    echo "offset 0.0 it would be volts, bounded by the ADC's +-0.512 V range." >&2
    echo >&2
    echo "The request was published but never reached the MCU. In order of likelihood:" >&2
    echo "  - the bridge is up but the serial link is not (check the Nexus log for" >&2
    echo "    'serial write failed' or a reconnect loop, and that /dev/ttyNova* exists)" >&2
    echo "  - the board does not carry a pH probe, so nothing consumes PhRequest" >&2
    echo "  - firmware/bridge wire-format mismatch on the 9-byte PhRequest - reflash" >&2
    exit 1
fi

readings=""
i=0
for ph in "$PH1" "$PH2" "$PH3"; do
    i=$((i + 1))
    read -rp "[$((i + 1))/6] rinse the probe, immerse it in the pH ${ph} buffer, then press Enter... "
    echo "  settling for ${SETTLE_S}s (electrode response)..."
    sleep "$SETTLE_S"
    echo "  waiting for a steady reading..."
    v=$(measure_steady "$TOL_V")   # topic is in VOLTS here (identity calibration)
    printf '  pH %-6s -> %s V\n' "$ph" "$v"
    readings+="$v $ph"$'\n'
done

echo "[5/6] fitting ph = slope * volts + offset through 3 points"

# Least squares rather than solving through two of the points: with three points
# the line that minimises total error is the honest answer, and the residuals it
# leaves are the diagnostic. Picking two points would hide the third's disagreement.
fit=$(printf '%s' "$readings" | awk '
    # n=0 explicitly: an unset awk variable used as a subscript is the empty
    # STRING, not 0, so the first point would land under key "" and the indexed
    # loop below would silently read it back as zero.
    BEGIN { n = 0 }
    { x[n] = $1; y[n] = $2; n++ }
    END {
        for (i = 0; i < n; i++) { sx += x[i]; sy += y[i]; sxy += x[i]*y[i]; sxx += x[i]*x[i] }
        # d is n^2 * variance(x), so it is >= 0 by construction; it only reaches
        # zero when every reading is the same voltage.
        d = n*sxx - sx*sx
        if (d < 1e-12) { print "singular"; exit }
        m = (n*sxy - sx*sy) / d
        b = (sy - m*sx) / n
        printf "%.6f %.6f\n", m, b
    }')

if [ "$fit" = "singular" ]; then
    echo "the three readings are all at the same voltage: the electrode is not responding." >&2
    echo "Check it is actually immersed, and that the BNC centre is not open-circuit." >&2
    exit 1
fi
slope=$(echo "$fit" | cut -d' ' -f1)
offset=$(echo "$fit" | cut -d' ' -f2)

echo "  slope:  $slope pH/V"
echo "  offset: $offset pH"

# Is the electrode actually responding? Compare the measured volt span against what
# an ideal electrode would give over this buffer range. A lazy or dead electrode
# still fits a line - just a very steep one - so the fit alone cannot catch this.
resp=$(printf '%s' "$readings" | awk -v nernst="$NERNST_MV" '
    BEGIN { n = 0 }   # see the note on the fit above
    { v[n] = $1; p[n] = $2; n++ }
    END {
        vlo = v[0]; vhi = v[0]; plo = p[0]; phi = p[0]
        for (i = 1; i < n; i++) {
            if (v[i] < vlo) vlo = v[i]; if (v[i] > vhi) vhi = v[i]
            if (p[i] < plo) plo = p[i]; if (p[i] > phi) phi = p[i]
        }
        expected = (phi - plo) * nernst / 1000.0
        printf "%.1f\n", (vhi - vlo) / expected * 100.0
    }')
resp_ok=$(awk -v r="$resp" -v m="$RESPONSE_MIN" 'BEGIN { print (r >= m) ? "yes" : "no" }')
if [ "$resp_ok" != "yes" ]; then
    echo >&2
    echo "CALIBRATION FAILED: the electrode moved only ${resp}% of the expected voltage" >&2
    echo "span across these buffers. That is a dead or badly fouled probe, an open BNC," >&2
    echo "or buffers that are not what the labels say. Nothing valid was calibrated." >&2
    exit 1
fi

# Electrode health, the number this whole exercise exists to produce. The measured
# response in mV/pH against the theoretical 59.16: a good electrode is 90-105%, and
# below about 85% it is due for replacement no matter how well the line fits.
health=$(awk -v s="$slope" -v nernst="$NERNST_MV" 'BEGIN {
    mv = 1000.0 / s
    if (mv < 0) mv = -mv
    printf "%.1f %.1f\n", mv, mv / nernst * 100.0 }')
mvperph=$(echo "$health" | cut -d' ' -f1)
pct=$(echo "$health" | cut -d' ' -f2)
echo "  electrode response: ${mvperph} mV/pH = ${pct}% of theoretical (${NERNST_MV} mV/pH)"

awk -v p="$pct" 'BEGIN {
    if (p < 85) {
        print "  WARNING: below 85% - this electrode is worn out and should be replaced."
    } else if (p < 90) {
        print "  note: 85-90% - aging electrode, still usable but watch it."
    } else if (p > 105) {
        print "  WARNING: above 105% - that is better than physics allows. Suspect wrong"
        print "           buffer values, or two buffers mixed up with each other."
    } else {
        print "  electrode health: good."
    }
}'

# The electrode EMF must FALL as pH rises, so the slope is negative on correct
# wiring. Positive means AINP and AINN are swapped at the BNC.
awk -v s="$slope" 'BEGIN { if (s > 0) {
    print "  WARNING: positive slope. The electrode EMF should fall as pH rises, so"
    print "           AINP/AINN are almost certainly swapped (BNC centre vs shell)."
    print "           The fit compensates and readings will be correct, but fix the"
    print "           wiring before this confuses someone." } }'

echo "[6/6] sending the fitted calibration"
send_cal "$slope" "$offset"
IDENTITY_LOADED=0   # the MCU holds the fit from here on, not volts
sleep 2

# --- check the line back against every buffer -------------------------------
#
# Two different checks, and they catch different things. The residuals are the fit
# disagreeing with the buffers, computed from readings already taken - that is a
# statement about the ELECTRODE. The live read-back is the MCU disagreeing with the
# fit - that is a statement about the LINK, and it is the one that catches firmware
# that silently ignored the request.

echo
echo "residuals (fitted line vs each buffer):"
printf '%s' "$readings" | awk -v m="$slope" -v b="$offset" '
    { pred = m * $1 + b
      printf "  pH %-6s measured %8.4f V -> fitted %6.3f   residual %+.3f\n", $2, $1, pred, pred - $2 }'

maxres=$(printf '%s' "$readings" | awk -v m="$slope" -v b="$offset" '
    { r = m * $1 + b - $2; if (r < 0) r = -r; if (r > mx) mx = r }
    END { printf "%.4f\n", mx }')
echo "  worst residual: ${maxres} pH"

res_ok=$(awk -v r="$maxres" -v m="$RESID_MAX" 'BEGIN { print (r <= m) ? "yes" : "no" }')
if [ "$res_ok" != "yes" ]; then
    echo >&2
    echo "CALIBRATION FAILED: worst residual ${maxres} pH exceeds ${RESID_MAX}." >&2
    echo "The three points do not lie on a line. Usual causes, in order: a buffer" >&2
    echo "that has gone off or been cross-contaminated by an unrinsed probe, not" >&2
    echo "enough settling time between dips, or an electrode near end of life." >&2
    echo "Do not use these values." >&2
    exit 1
fi

echo
echo "live read-back (probe is still in the pH ${PH3} buffer):"
# TOL_PH, not TOL_V: the MCU is publishing pH again as of step 6. This one is a
# hard failure rather than a skip - an unverifiable read-back is exactly what a
# board that ignored the request looks like, so passing the run anyway would
# defeat the only check that can tell the difference.
verif=$(measure_steady "$TOL_PH")
echo "  reads ${verif} pH, expected ${PH3}"

ok=$(awk -v v="$verif" -v r="$PH3" -v t="$VERIFY_TOL" 'BEGIN {
    d = v - r; if (d < 0) d = -d
    print (d <= t) ? "yes" : "no" }')
if [ "$ok" != "yes" ]; then
    echo >&2
    echo "CALIBRATION FAILED: read-back is not within ${VERIFY_TOL} pH of the buffer." >&2
    echo "The residuals were fine, so the fit itself is sound - which points at the" >&2
    echo "MCU never applying the request (firmware/bridge wire-format mismatch on the" >&2
    echo "9-byte PhRequest - reflash the firmware). Do not hardcode these values." >&2
    exit 1
fi
echo "  within ${VERIFY_TOL} pH - the MCU applied the calibration."
echo
echo "done. the MCU holds this calibration in RAM only - to survive a reset, either:"
echo
echo "  [preferred] set both values in src/avionics_nexus/config/ph_cal.yaml, then"
echo "              restart the bridge (Nexus replays them on every link-up;"
echo "              the file is bind-mounted, so no colcon build needed):"
echo "      ph_slope_solution:  ${slope}"
echo "      ph_offset_solution: ${offset}"
echo
echo "  BOTH keys or neither - Nexus replays nothing unless both are present."
echo
echo "  [firmware fallback] hardcode in pHMeterThread.h (PhType::slope/offset) -"
echo "              only used when no calibration is configured above:"
echo "      float slope  = ${slope}f;"
echo "      float offset = ${offset}f;"
