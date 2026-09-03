#!/usr/bin/env bash
#
# calib_ph_simple.sh : quick N-point pH calibration - you read the numbers, this
#                      fits the line.
#
# The difference from calibrate_ph.sh is where the measurements come from. That
# script drives the whole dip sequence itself and spends most of its ~10 minutes
# deciding whether a reading has settled. This one takes the readings FROM YOU and
# does nothing else: set the identity, read the volts off the topic yourself, type
# in what you saw, get slope and offset back. Two minutes, no state machine.
#
# What you give up is exactly the settling gates, so the electrode being actually
# steady when you read it is now your job - watch the topic until the last digits
# stop walking rather than reading the first plausible number. Everything after
# the measurements is identical arithmetic: the same least-squares fit, the same
# residual and electrode-health checks.
#
# Procedure:
#   1. slope := 1.0, offset := 0.0  -> /EL/ph_packet now publishes the raw
#                                      ADS1114 differential reading in VOLTS
#   2. dip the probe in each buffer, watch the topic, note the steady volts
#   3. type in each (buffer pH, measured volts) pair here
#   4. least-squares fit ph = slope * volts + offset
#   5. optionally send the fit to the MCU; paste it into ph_cal.yaml to keep it
#
# Usage:
#   ./calib_ph_simple.sh                          interactive, asks how many points
#   ./calib_ph_simple.sh -n 3                     interactive, 3 points
#   ./calib_ph_simple.sh 4.00:0.1743 6.86:0.0262 9.00:-0.1178
#                                                 non-interactive, refit given pairs
#   ./calib_ph_simple.sh --offline ...            pure calculator, never touches ROS
#
# Pairs are pH:volts. With pairs on the command line the identity is NOT sent -
# you are refitting numbers you already have, so there is nothing to measure.
#
# HOW MANY POINTS
#
# Three is the default. Two points always fit a line perfectly, so a fouled or
# unsettled electrode produces a flawless-looking calibration and wrong readings -
# with n=2 the residuals below are structurally zero and the script says so rather
# than printing a meaningless 0.000. The third point's residual is the only thing
# that catches it. Five is worth the extra dips when you want the residual pattern
# to actually localise a bad buffer.

set -euo pipefail

usage() {
    echo "usage: $0 [-n POINTS] [--offline] [--send] [pH:volts ...]" >&2
    echo "  e.g. $0 -n 3" >&2
    echo "       $0 4.00:0.1743 6.86:0.0262 9.00:-0.1178" >&2
    exit 1
}

TOPIC_REQ=/EL/ph_req
TOPIC_PKT=/EL/ph_packet
MSG=custom_msg/msg/PhRequest

NERNST_MV=59.16    # theoretical mV per pH at 25 C, ideal glass electrode
PH_SPAN_MIN=2.0    # buffers must span at least this much pH for a fit to mean anything
RESID_MAX=0.05     # max acceptable |residual| at any buffer, in pH (n>=3 only)
RESPONSE_MIN=85    # measured response, as % of theoretical, below which the probe is suspect
ASYM_MAX_MV=60     # |asymmetry potential| at pH 7 must stay under this, in mV
PGA_V=0.512        # ADS1114 full-scale range - a "volts" reading outside it is not volts

# RESID_MAX is half the 0.1 pH the rover actually needs, and that headroom is the
# point: the residual is only the fit's own inconsistency, and drift, temperature
# and the buffers' own tolerance all spend from the same 0.1 budget afterwards. A
# fit that lands at 0.09 has already spent it all. Do not widen this to make a run
# pass - see the 2026-09-02 entry in ph_cal.yaml for what that would have hidden.

NPOINTS=""
OFFLINE=0
SEND=""            # "" = ask, 1 = send without asking, 0 = never send
PAIRS=()
while [ $# -gt 0 ]; do
    case "$1" in
        -n|--points)  NPOINTS="${2:?-n needs a value}"; shift 2 ;;
        -n=*|--points=*) NPOINTS="${1#*=}"; shift ;;
        --offline)    OFFLINE=1; shift ;;
        --send)       SEND=1; shift ;;
        --no-send)    SEND=0; shift ;;
        -h|--help)    usage ;;
        -*)           echo "unknown option: $1" >&2; usage ;;
        *)            PAIRS+=("$1"); shift ;;
    esac
done

is_num() { awk -v x="$1" 'BEGIN { exit !(x ~ /^[-+]?([0-9]+\.?[0-9]*|\.[0-9]+)([eE][-+]?[0-9]+)?$/) }'; }

PH=()
VOLTS=()

# ---------------------------------------------------------------- command line
if [ ${#PAIRS[@]} -gt 0 ]; then
    for p in "${PAIRS[@]}"; do
        case "$p" in
            *:*) ;;
            *) echo "'$p' is not a pH:volts pair." >&2; usage ;;
        esac
        ph="${p%%:*}"; v="${p#*:}"
        is_num "$ph" || { echo "'$ph' is not a pH value." >&2; exit 1; }
        is_num "$v"  || { echo "'$v' is not a voltage." >&2; exit 1; }
        PH+=("$ph"); VOLTS+=("$v")
    done
    OFFLINE=1                       # refitting given numbers: nothing to measure
    [ -n "$NPOINTS" ] && [ "$NPOINTS" -ne ${#PH[@]} ] && {
        echo "-n $NPOINTS but ${#PH[@]} pairs given." >&2; exit 1; }
fi

# --------------------------------------------------------------- ros2, if used
have_ros=0
if [ "$OFFLINE" -eq 0 ]; then
    # set +u: the ROS/colcon setup files read unbound variables, which under -u
    # aborts inside setup.bash with an error pointing nowhere near the cause.
    if ! command -v ros2 >/dev/null 2>&1; then
        set +u
        [ -f /opt/ros/humble/setup.bash ] && . /opt/ros/humble/setup.bash
        [ -f "$HOME/dev_ws/install/setup.bash" ] && . "$HOME/dev_ws/install/setup.bash"
        set -u
    fi
    command -v ros2 >/dev/null 2>&1 && have_ros=1
    if [ "$have_ros" -eq 0 ]; then
        echo "no ros2 in this shell - continuing as a calculator only." >&2
        echo "(you are probably outside the container; ./run.sh or ./attach.sh)" >&2
        echo >&2
        OFFLINE=1
    fi
fi

send_cal() { # $1=slope $2=offset
    # --times 5, NOT --once. The subscription is best-effort (Nexus.cpp), so the
    # transport gives no delivery guarantee and --once destroys the publisher the
    # instant it has handed the message over - a message still in flight when the
    # node exits is dropped, with exit status 0. change_cal is idempotent, so five
    # copies at 10 Hz cost half a second and are indistinguishable from one.
    ros2 topic pub --times 5 --rate 10 "$TOPIC_REQ" "$MSG" \
        "{change_cal: true, slope: $1, offset: $2}" >/dev/null
}

IDENTITY_LOADED=0
cleanup() {
    # Leaving the MCU on the identity is a silent trap: the topic keeps publishing
    # and every value on it is volts wearing a pH label. Nothing downstream can
    # tell. Say so loudly on any early exit.
    [ "$IDENTITY_LOADED" -eq 1 ] || return 0
    echo >&2
    echo "!! the MCU is still on slope 1.0 / offset 0.0 - $TOPIC_PKT is reporting" >&2
    echo "   VOLTS, not pH. Restart the bridge (it replays ph_cal.yaml on link-up)" >&2
    echo "   or re-run this script to completion." >&2
}
trap cleanup EXIT

# ------------------------------------------------------------------- measuring
if [ ${#PH[@]} -eq 0 ]; then
    if [ -z "$NPOINTS" ]; then
        read -rp "how many points? [3] " NPOINTS
        NPOINTS="${NPOINTS:-3}"
    fi
    case "$NPOINTS" in
        2|3|5) ;;
        *) echo "points must be 2, 3 or 5, not '$NPOINTS'." >&2; exit 1 ;;
    esac

    if [ "$OFFLINE" -eq 0 ]; then
        echo "slope := 1.0, offset := 0.0 ($TOPIC_PKT now shows raw volts)"
        send_cal 1.0 0.0
        IDENTITY_LOADED=1
        echo
        echo "watch it with:  ros2 topic echo $TOPIC_PKT"
        echo "the MCU averages 10 samples at ~1.6 Hz, so give it ~7 s after each dip"
        echo "and read the value only once the last digits have stopped walking."
    else
        echo "offline: set the MCU to slope 1.0 / offset 0.0 yourself to read volts."
    fi
    echo

    i=1
    while [ "$i" -le "$NPOINTS" ]; do
        ph=""; v=""
        while [ -z "$ph" ]; do
            read -rp "point $i/$NPOINTS - buffer pH: " ph
            is_num "$ph" || { echo "  '$ph' is not a number."; ph=""; }
        done
        while [ -z "$v" ]; do
            read -rp "point $i/$NPOINTS - measured (V): " v
            if ! is_num "$v"; then
                echo "  '$v' is not a number."; v=""; continue
            fi
            # A value outside the ADS1114's range is not volts. Almost always mV
            # typed in whole, which fits a line 1000x too shallow and looks fine.
            if awk -v x="$v" -v p="$PGA_V" 'BEGIN { exit !((x < 0 ? -x : x) > p) }'; then
                echo "  $v V is outside the ADS1114 +-$PGA_V V range - did you mean" \
                     "$(awk -v x="$v" 'BEGIN { printf "%.6f", x / 1000 }') V (i.e. mV)?"
                read -rp "  keep $v V anyway? [y/N] " k
                [ "$k" = y ] || [ "$k" = Y ] || v=""
            fi
        done
        PH+=("$ph"); VOLTS+=("$v")
        i=$((i + 1))
    done
    echo
fi

N=${#PH[@]}
[ "$N" -ge 2 ] || { echo "need at least 2 points, got $N." >&2; exit 1; }

# ------------------------------------------------------------------------ fit
FIT=$(paste -d' ' <(printf '%s\n' "${PH[@]}") <(printf '%s\n' "${VOLTS[@]}") \
      | awk -v nernst="$NERNST_MV" '
    { ph[NR] = $1; v[NR] = $2
      sx += $2; sy += $1; sxx += $2 * $2; sxy += $1 * $2
      if (NR == 1 || $1 < phlo) phlo = $1
      if (NR == 1 || $1 > phhi) phhi = $1
      if (NR == 1 || $2 < vlo)  vlo  = $2
      if (NR == 1 || $2 > vhi)  vhi  = $2 }
    END {
        n = NR
        den = n * sxx - sx * sx
        if (den == 0 || (den < 0 ? -den : den) < 1e-12) { print "DEGENERATE"; exit }
        slope  = (n * sxy - sx * sy) / den
        offset = (sy - slope * sx) / n

        worst = 0; wi = 0
        for (i = 1; i <= n; i++) {
            r = slope * v[i] + offset - ph[i]
            res[i] = r
            a = (r < 0) ? -r : r
            if (a > worst) { worst = a; wi = i }
        }
        # mV per pH from the fitted slope, and the volts at pH 7 - the electrode
        # asymmetry potential, which is what the offset means physically.
        mvph = (slope == 0) ? 0 : 1000 / ((slope < 0) ? -slope : slope)
        asym = (slope == 0) ? 0 : ((7 - offset) / slope) * 1000

        printf "slope %.6f\n", slope
        printf "offset %.6f\n", offset
        printf "mvph %.2f\n", mvph
        printf "pct %.1f\n", mvph / nernst * 100
        printf "asym %.1f\n", asym
        printf "phspan %.4f\n", phhi - phlo
        printf "vspan %.6f\n", vhi - vlo
        printf "worst %.4f\n", worst
        printf "worsti %d\n", wi
        for (i = 1; i <= n; i++) printf "point %.4f %.6f %.4f %+.4f\n", \
            ph[i], v[i], slope * v[i] + offset, res[i]
    }')

[ "$FIT" = DEGENERATE ] && {
    echo "all points sit at the same voltage - no line through them." >&2
    echo "the probe is not responding, or the identity never reached the MCU." >&2
    exit 1; }

get() { awk -v k="$1" '$1 == k { print $2 }' <<<"$FIT"; }
SLOPE=$(get slope); OFFSET=$(get offset)
MVPH=$(get mvph);   PCT=$(get pct);   ASYM=$(get asym)
PHSPAN=$(get phspan); WORST=$(get worst); WORSTI=$(get worsti)

# --------------------------------------------------------------------- report
echo "  buffer   measured      fitted   residual"
awk '$1 == "point" { printf "  %6.2f  %+9.4f V  %8.4f   %+8.4f\n", $2, $3, $4, $5 }' <<<"$FIT"
echo
printf 'slope  %s pH/V\noffset %s pH\n' "$SLOPE" "$OFFSET"
printf 'response %s mV/pH = %s%% of theoretical %s\n' "$MVPH" "$PCT" "$NERNST_MV"
printf 'asymmetry %s mV at pH 7\n' "$ASYM"
echo

FAIL=0
note() { echo "  $*"; }

# span: a fit over a narrow pH range extrapolates badly to the ends
if awk -v s="$PHSPAN" -v m="$PH_SPAN_MIN" 'BEGIN { exit !(s < m) }'; then
    note "FAIL  buffers span only $PHSPAN pH (need >= $PH_SPAN_MIN) - the line is"
    note "      being extrapolated everywhere it matters."
    FAIL=1
fi

# residual: the only check that can see a bad buffer or an unsettled probe, and
# it needs a third point to exist at all - two points always fit perfectly.
if [ "$N" -lt 3 ]; then
    note "n=2: residuals are structurally zero and prove nothing. A worn or"
    note "     unsettled electrode fits this line just as perfectly as a good one."
else
    if awk -v w="$WORST" -v m="$RESID_MAX" 'BEGIN { exit !(w > m) }'; then
        bad=$(awk -v i="$WORSTI" '$1 == "point" { if (++c == i) printf "%g", $2 }' <<<"$FIT")
        note "FAIL  worst residual $WORST pH (limit $RESID_MAX), at the pH $bad buffer."
        note "      One point off the line is a bad buffer or a probe read before it"
        note "      settled, not something a different slope/offset can fix. Redo that"
        note "      dip - fresh buffer, rinse and blot, and let it sit."
        FAIL=1
    else
        note "OK    worst residual $WORST pH (limit $RESID_MAX)"
    fi
fi

# electrode health: slope sign, response, asymmetry
if awk -v s="$SLOPE" 'BEGIN { exit !(s >= 0) }'; then
    note "FAIL  slope is positive. A glass electrode gives LESS voltage at higher pH,"
    note "      so this is a swapped BNC, a pH/volts column swap, or the identity"
    note "      never reached the MCU (in which case you fitted pH against itself)."
    FAIL=1
fi
if awk -v p="$PCT" -v m="$RESPONSE_MIN" 'BEGIN { exit !(p < m) }'; then
    note "WARN  response $PCT% is below $RESPONSE_MIN% - the electrode is worn or the"
    note "      BNC is leaking. Dry the connector and refit before trusting this."
fi
if awk -v a="$ASYM" -v m="$ASYM_MAX_MV" 'BEGIN { exit !((a < 0 ? -a : a) > m) }'; then
    note "WARN  asymmetry $ASYM mV is outside the +-$ASYM_MAX_MV mV a healthy electrode"
    note "      holds. A leak drags this negative - see ph_cal.yaml."
fi
echo

if [ "$FAIL" -ne 0 ]; then
    echo "NOT GOOD ENOUGH - do not fly this pair." >&2
    [ "$IDENTITY_LOADED" -eq 1 ] && {
        echo >&2
        echo "restoring the previous calibration is on you: restart the bridge, or" >&2
        echo "re-run once you have better readings." >&2; }
    exit 2
fi

echo "paste into src/avionics_nexus/config/ph_cal.yaml:"
printf '    ph_slope_solution:  %11s\n' "$SLOPE"
printf '    ph_offset_solution: %11s\n' "$OFFSET"
echo

# --------------------------------------------------------------------- send it
if [ "$have_ros" -eq 1 ] && [ "$SEND" != "0" ]; then
    if [ "$SEND" != "1" ]; then
        read -rp "send this to the MCU now? [Y/n] " ans
        case "$ans" in n|N) SEND=0 ;; *) SEND=1 ;; esac
    fi
    if [ "$SEND" = "1" ]; then
        send_cal "$SLOPE" "$OFFSET"
        IDENTITY_LOADED=0       # the MCU is on the fitted line now, not the identity
        echo "sent. $TOPIC_PKT is back in the pH domain."
        echo "The MCU keeps this in RAM only - the yaml above is what survives a reset."
    fi
fi
