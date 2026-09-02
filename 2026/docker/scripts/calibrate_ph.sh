#!/usr/bin/env bash
#
# calibrate_ph.sh : interactive 2- or 3-point pH calibration through the Nexus bridge.
#
# Procedure:
#   1. slope := 1.0, offset := 0.0  -> the published "ph" becomes the raw ADS1114
#                                      differential reading in VOLTS, which is what
#                                      we actually need to fit against
#   2. operator dips the probe in each buffer of known pH, one at a time
#   3. wait for a steady reading in each, average it
#   4. least-squares fit ph = slope * volts + offset through the points
#   5. send the fitted line to the MCU, then check it back against every buffer
#
# TWO POINTS OR THREE
#
# Three is the default and is what you want unless you have a reason not to. Two
# points always fit a line perfectly, so a lazy or contaminated electrode produces
# a beautiful calibration and a wrong reading; the third point's residual is the
# only thing that catches it. Under --points 2 the residual check below is
# structurally vacuous and is skipped rather than faked - the script says so out
# loud, because a "0.000 worst residual" that cannot be anything else is worse
# than no number at all.
#
# What survives at two points: the span check, the electrode-response check
# (measured volt span vs the theoretical Nernst span), the slope-sign check, and
# the live read-back. Those catch a dead probe, a swapped BNC, and a firmware that
# ignored the request. They do NOT catch a mildly sick electrode - only the third
# buffer does.
#
# So use --points 2 when you are time-limited on the pad, when you only have two
# buffers to hand, or when you are re-checking a probe whose health you already
# established with a 3-point run. Use three whenever the answer matters.
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
# Usage: ./calibrate_ph.sh [--points 2|3] <pH_a> <pH_b> [pH_c]
#   e.g. ./calibrate_ph.sh 4.01 6.86 9.18            (common NIST-traceable set)
#        ./calibrate_ph.sh --points 2 4.01 9.18      (two-buffer quick calibration)
#
# --points is optional: with no flag the count of buffers you pass decides. Pass it
# when you want the script to hold you to a number - it errors out if the buffer
# count disagrees, which is what catches a typo that drops a buffer off the command
# line and would otherwise silently downgrade a 3-point run to a 2-point one.
#
# Order does not matter, but the buffers must span a real range - see PH_SPAN_MIN.

set -euo pipefail

usage() {
    echo "usage: $0 [--points 2|3] <pH_a> <pH_b> [pH_c]" >&2
    echo "  e.g. $0 4.01 6.86 9.18" >&2
    echo "       $0 --points 2 4.01 9.18" >&2
    exit 1
}

WANT_POINTS=""     # empty = infer from how many buffers were given
PH=()
while [ $# -gt 0 ]; do
    case "$1" in
        --points|-p)  WANT_POINTS="${2:?--points needs a value (2 or 3)}"; shift 2 ;;
        --points=*)   WANT_POINTS="${1#*=}"; shift ;;
        -h|--help)    usage ;;
        -*)           echo "unknown option: $1" >&2; usage ;;
        *)            PH+=("$1"); shift ;;
    esac
done

NPOINTS=${#PH[@]}
if [ "$NPOINTS" -ne 2 ] && [ "$NPOINTS" -ne 3 ]; then
    echo "give 2 or 3 buffer pH values, not $NPOINTS." >&2
    usage
fi
if [ -n "$WANT_POINTS" ]; then
    if [ "$WANT_POINTS" != "2" ] && [ "$WANT_POINTS" != "3" ]; then
        echo "--points must be 2 or 3, not '$WANT_POINTS'." >&2
        exit 1
    fi
    if [ "$WANT_POINTS" -ne "$NPOINTS" ]; then
        echo "--points $WANT_POINTS but $NPOINTS buffer value(s) given: ${PH[*]}" >&2
        echo "Fix one or the other - this mismatch is usually a buffer missing from" >&2
        echo "the command line, which would otherwise quietly become a $NPOINTS-point run." >&2
        exit 1
    fi
fi

# Every buffer must be a number, checked before the operator does ten minutes of
# work. A non-numeric argument would otherwise reach awk, which reads it as 0 and
# happily fits a line through pH 0 - a plausible-looking, entirely wrong result.
for ph in "${PH[@]}"; do
    if ! awk -v x="$ph" 'BEGIN { exit !(x ~ /^-?[0-9]+(\.[0-9]+)?$/) }'; then
        echo "'$ph' is not a pH value." >&2
        usage
    fi
done

PH_LAST="${PH[$((NPOINTS - 1))]}"   # the buffer the probe is still sitting in at the read-back

TOPIC_REQ=/EL/ph_req
TOPIC_PKT=/EL/ph_packet
MSG=custom_msg/msg/PhRequest

# Step numbering: identity + one dip per buffer + fit + send.
TOTAL_STEPS=$((NPOINTS + 3))
STEP_FIT=$((NPOINTS + 2))

NSAMPLES=12        # samples per steadiness check (1.6 Hz -> ~7.5 s window)
TOL_V=0.0005       # SPREAD gate: steady when (max-min) <= 0.5 mV, ~0.008 pH
                   # 2 mV -> 0.3 -> 0.35 -> 0.5. The tight end of that range was
                   # chasing settling with the wrong instrument: this gate is
                   # NOT what catches a settling probe - STEADY_RUNS is - and
                   # sizing it as though it were is what made 0.3 mV reject half
                   # of all windows for noise the fit does not care about.
                   # Measured on this rig, sigma_raw ~0.5 mV through the MCU's
                   # 10-deep average puts the median window range at 0.28 mV
                   # with a long tail (observed rejects: 0.328, 0.353, 0.436,
                   # 0.566, 0.573, 0.697 mV).
                   #
                   # 0.5 vs 0.8 was measured rather than argued, 30 settled runs
                   # each: 0.5 mV completed 30/30 at a mean 126 s and a worst
                   # 150 s; 0.8 mV completed 30/30 at a flat 120 s. Loosening
                   # further buys ~6 s and admits noisier windows into the mean,
                   # so 0.5 is kept. Windows in the 0.55-0.7 mV tail are simply
                   # skipped - they do not reset the streak - which is why the
                   # cost is seconds rather than a restart.
                   #
                   # Its only job is rejecting windows too disturbed to mean
                   # anything. A window admitted here contributes at most
                   # 0.008 pH of spread to a mean averaged over 16 windows.
                   #
                   # NOT for precision. The window MEAN feeds the fit, and 12
                   # samples spanning 2 mV already gave a mean good to 0.0025 pH,
                   # 40x finer than the 0.1 pH the rover needs. 0.3 mV takes that
                   # to 0.0004 pH, which buys nothing. The reason to tighten it
                   # is settling, and the reason is worth writing down:
                   #
                   # A probe approaching its asymptote exponentially with R mV
                   # still to go moves R*(1-exp(-7.5/tau)) across one window, so
                   # passing a spread gate of TOL bounds the error left in the
                   # reading at R <= TOL/(1-exp(-7.5/tau)). That bound depends
                   # entirely on tau, and gets bad fast when tau is long:
                   #
                   #     tau     TOL=2 mV        TOL=0.3 mV
                   #      30 s   9.0 mV 0.15 pH  1.4 mV 0.023 pH
                   #     120 s  33.0 mV 0.56 pH  5.0 mV 0.084 pH
                   #     300 s  81.0 mV 1.37 pH 12.2 mV 0.205 pH
                   #
                   # The tau=30 s / 2 mV cell is 0.153 pH, which is the 0.157 pH
                   # drift incident the comments below were written around - the
                   # model reproduces the one event we have measured.
                   #
                   # So the spread gate alone is only a settling check for a FAST
                   # probe. At tau=300 s - a probe on its first dip out of dry
                   # storage - even 0.3 mV admits 12 mV, and the 2026-09-02 run
                   # missed by 11.1 mV on the buffer it read first. That is why
                   # the trend gate has to stay alive: it tests whether the drift
                   # is real and consistently signed, which does not care what
                   # tau is. Tightening this is not a substitute for it.
                   #
                   # THE ONE HARD CONSTRAINT: steady() tests spread FIRST and
                   # returns, so any window reaching the drift test has range
                   # <= TOL_V, and for n=12 an OLS fit over range R cannot report
                   # a net drift above 1.385*R. TOL_DRIFT_V must therefore stay
                   # under TOL_V/1.385 or `ad > dtol` is unsatisfiable and the
                   # trend gate is dead code. At TOL_V=0.0003 the cap is 0.217 mV
                   # - the old 0.5 mV floor could never fire. Move these two
                   # together, always.
TOL_PH=0.03        # spread gate in the pH domain, for the read-back
                   # Sized against TOL_V's 0.034 pH equivalent, and now sitting
                   # just under it (0.08 -> 0.03) rather than above: the buffer
                   # samples feed the fit and their mean must be precise, while
                   # the read-back only has to confirm the MCU applied the line
                   # and the electrode held still - both of which the deviation
                   # test below actually measures. 0.08 was the ceiling, not a
                   # preference: it sets the read-back's mean uncertainty, which
                   # sets how big STABILITY_TOL must be to stay a 4-sigma gate,
                   # and past 0.08 that gate grows past the 0.157 pH drift it
                   # was added to catch. Coming DOWN from it is what lets
                   # STABILITY_TOL go back to 0.10 (see its note below).
                   #
                   # The cost is patience, not correctness: 0.03 pH is ~1.8 mV
                   # of electrode stability at the read-back, about what TOL_V
                   # already demands of a buffer window. If the last step starts
                   # timing out on a healthy probe, this is the number to relax
                   # - and STABILITY_TOL has to move with it.
TOL_DRIFT_V=0.001  # TREND gate: net drift across the window, volts (1 mV)
                   # Effectively a no-op at the current TOL_V, by design. Do not
                   # "fix" it into firing more often.
                   #
                   # steady() tests spread first and returns, so a window
                   # reaching this test has range <= TOL_V, and for n=12 an OLS
                   # fit over range R cannot report net drift above 1.385*R.
                   # At TOL_V=0.0005 that caps drift at 0.36 mV, well under
                   # this 1 mV threshold, so the branch cannot fire at all.
                   # That is deliberate. Every threshold low enough to clear the
                   # cap is also below the noise floor: measured at sigma_raw =
                   # 0.5 mV, noise-driven drift has a median of 0.18 mV and a
                   # p99 of 0.67 mV, and 0.1 mV tripped on 71% of windows from a
                   # PERFECTLY SETTLED probe with the sign flipping every pass.
                   # The two constraints have no overlap, and the t >= TREND_T
                   # test does not rescue it - the MCU's averaging correlates
                   # consecutive samples, shrinking the residuals about the
                   # fitted line, which deflates SE and INFLATES t.
                   #
                   # It is the wrong measurement on the wrong timescale. Slow
                   # settling is caught by STEADY_RUNS over a 2 min baseline
                   # instead. Kept, not deleted, so the branch means something
                   # again if TOL_V is ever loosened past ~1.4 mV.
                   #
                   # The history is worth keeping because it was expensive:
                   #   0.1 mV  tripped on 71% of windows from a PERFECTLY
                   #           SETTLED probe, sign flipping every pass
                   #   0.5 mV  tripped on 4.8%, and since a drift trip RESETS
                   #           the streak that is a 0.952^16 = 45% chance of
                   #           ever completing a 16-window run
                   # Noise-driven drift at sigma_raw = 0.5 mV has a median of
                   # 0.18 mV and a p99 of 0.67 mV, so any threshold low enough
                   # to catch real settling inside 7.5 s is also low enough to
                   # fire constantly on noise. The t >= TREND_T test does not
                   # rescue it: the MCU's averaging correlates consecutive
                   # samples, which shrinks the residuals about the fitted line,
                   # which deflates SE and INFLATES t. It is the wrong
                   # measurement on the wrong timescale, and no threshold fixes
                   # that.
                   #
                   # Slow settling - the thing this was really for - is caught
                   # properly by STEADY_RUNS over a 2 min baseline instead.
TOL_DRIFT_PH=0.015 # trend gate in the pH domain - deliberately MUCH looser
                   # than TOL_DRIFT_V, because the two protect different things.
                   # A buffer sample feeds the fit and is used minutes later, so
                   # drift in it becomes fit error and must stay tight. The
                   # read-back is consumed immediately by the deviation test
                   # below, which measures the very thing this would be guarding
                   # against - so a tight gate here only re-checks, badly, what
                   # is already checked well.
                   #
                   # 0.015 is set by keeping the two consistent: drift that
                   # slips through must still be caught downstream. At 0.015 pH
                   # per 7.5 s window, a ~30 s gap to the read-back accumulates
                   # 0.06 pH, which STABILITY_TOL=0.10 catches. Anything under
                   # that gate is drift too small to matter by definition.
                   #
                   # It also has to clear the same 1.385x cap TOL_V's note
                   # explains: spread is tested first, so a drift tolerance at
                   # or above TOL_PH/1.385 = 0.022 can never fire. 0.03/0.015
                   # keeps the documented 2:1 separation; 0.03/0.03 did not.
                   #
                   # This also stops the gate fighting WANDER. The significance
                   # test in steady() rejects noise that merely mimics a trend
                   # within one window, but slow wander produces a genuinely
                   # significant trend inside a window whose direction then flips
                   # in the next one (+0.025 then -0.032 pH, observed). A single
                   # window cannot tell those apart; a threshold set by what
                   # actually matters downstream does not need to.
STEADY_RUNS=8      # consecutive passing windows required before a reading is
                   # accepted, each ~7.5 s -> a 60 s floor on every dip.
                   #
                   # This is the gate that refuses a quick dip, and it exists
                   # because every other gate in this file looks INSIDE one
                   # 7.5 s window. That is a timescale, and settling slower than
                   # it is invisible: at tau=300 s a probe 12 mV from its
                   # asymptote moves 0.30 mV per window, so a single window can
                   # look perfectly steady while the reading is 0.2 pH from
                   # where it is heading.
                   #
                   # WHY 8, AND WHY NOT 4. The first cut used 4 windows and compared
                   # max-min of their means, which does not work, and measuring
                   # it is the only way to see that. The MCU publishes a 10-deep
                   # moving average, so 12 samples inside one window are worth
                   # only ~1-2 independent ones and a window mean is barely
                   # quieter than a single sample. Averaging therefore has to
                   # come from a LONGER BASELINE, not from more samples inside a
                   # window. Measured on a perfectly settled probe at
                   # sigma_raw = 0.5 mV, comparing mean(first half) against
                   # mean(second half) of a streak of N windows:
                   #
                   #    N   baseline   p99 of |step|   slowest march it can hide
                   #    4      30 s      0.31 mV       73 mV/hr -> 0.207 pH
                   #    8      60 s      0.21 mV       25 mV/hr -> 0.070 pH
                   #   16     120 s      0.15 mV        9 mV/hr -> 0.025 pH
                   #   24     180 s      0.15 mV        6 mV/hr -> 0.016 pH
                   #
                   # ("hide" = drift small enough to pass, accumulated over a
                   # 10 min calibration.) At N=4 the threshold has to sit above
                   # 0.3 mV just to clear noise, and a gate that loose hides
                   # 0.2 pH - it would have passed the 2026-09-02 run, so 4 is
                   # out on evidence.
                   #
                   # 8 is a deliberate trade of accuracy for dwell: it spends
                   # 0.051 pH of the 0.1 pH budget on undetected drift where 16
                   # spends 0.025, and it leaves roughly twice as much
                   # un-settled error in a reading from a sluggish probe
                   # (measured: 1.8 mV vs 0.8 mV at tau=300 s with 2 mV to go).
                   # What it buys is 64 s per buffer instead of 126 s, about
                   # 3 min off a three-point run. It still rejects the case this
                   # gate was built for - tau=300 s with 20 mV to go, the
                   # 2026-09-02 failure - which is what makes the trade
                   # defensible rather than a rollback. Go to 16 if the budget
                   # ever gets tight; do not go below 8.
                   #
                   # It is a measurement, not the old SETTLE_S clock coming
                   # back. The dwell is a floor, not a fixed wait: a probe still
                   # moving keeps failing and keeps waiting, and one that
                   # settled during the previous dip clears it in 120 s flat.
                   # The file's rule still holds - the data decides WHEN, this
                   # only decides that it has to be asked over a long enough
                   # baseline to answer.
SETTLE_STEP_V=0.00015   # max |mean(first half) - mean(second half)| across the
SETTLE_STEP_PH=0.004    # streak, volts / pH. Sized at the p99 of that statistic
                   # on a settled probe. At STEADY_RUNS=8 it blocks ~8% of
                   # individual checks - which costs seconds, not runs, because
                   # the streak slides: measured 30/30 completions at a mean
                   # 64 s against the 60 s floor. It bounds undetected drift at
                   # 18 mV/hour = 0.051 pH over a 10 min calibration (at 16
                   # windows the same threshold bounds it at 0.025 pH). That is the residual budget this gate
                   # protects - do not widen it without redoing the table above.
                   #
                   # A STEP, not a max-min span: max-min is an extreme-value
                   # statistic that grows with N, so it gets WORSE as the
                   # baseline lengthens, which is precisely backwards. The
                   # half-to-half step averages instead, and improves with N.
                   # Regression-tested against a simulated exponential probe
                   # driven through the MCU's 10-deep average at sigma_raw =
                   # 0.5 mV, running THIS code rather than a model of it - which
                   # is how two bad thresholds were caught before they shipped:
                   #
                   #   settled probe, re-dip      accept  75 s, +0.07 mV
                   #   tau=30 s,  20 mV to go     accept 173 s, +0.34 mV
                   #   tau=120 s, 10 mV to go     accept 338 s, +0.76 mV
                   #   tau=300 s,  2 mV to go     accept  75 s, +1.80 mV
                   #   tau=300 s, 20 mV to go     REJECT  <- the 2026-09-02 run
                   #   2 mV/hour E0 walk          accept  75 s (correctly: no
                   #     gate here can see that one, only soaking fixes it)
                   #
                   # 30/30 settled runs completed, mean 64 s against the 60 s
                   # floor. So on a good probe budget ~1 min per buffer, on a
                   # sluggish one 3-6 min, and expect a genuinely unsettled one
                   # to fail rather than finish slowly - the answer there is
                   # soaking, not a bigger TRIES. The floor is ~1.8 mV =
                   # 0.030 pH of un-settled error left in a reading, against a
                   # 0.1 pH budget: it does not make slow settling impossible,
                   # it makes it visible and bounded.
TREND_T=3          # a trend must reach this many standard errors to count as real
                   # 3 sigma: pure noise clears it in ~0.3% of windows, so a
                   # settled probe is not held here, while a genuine transient
                   # (smooth, same sign every sample) reaches t in the tens.
TRIES=80           # steadiness retries (~7.5 s each -> gives up after ~10 min).
                   # 48 not 40 because the settling now happens INSIDE this loop:
                   # the old budget was a 60 s sleep plus 40 tries, and folding the
                   # sleep in without raising the cap would have cut a slow buffer
                   # short at exactly the point the old code was patient.

# TWO GATES, because max-min cannot see a settling electrode.
#
# Spread alone accepts any slow monotonic ramp that happens to fit inside the
# tolerance, and a relaxing electrode is exactly that. Measured on this rig, eight
# consecutive samples read:
#
#   -52.459 -52.158 -51.864 -51.584 -51.328 -51.086 -50.863 -50.698 mV
#
# Every delta positive, decaying smoothly - a textbook exponential with tau ~10 s,
# still 2.7 mV from its asymptote. Spread across that window is 1.76 mV, INSIDE
# the 2 mV tolerance, so the old gate called it steady and would have fitted a
# buffer point 0.047 pH away from where the electrode was actually heading - about
# eight times the worst residual of the best fit on record.
#
# So the trend is tested separately: a least-squares slope against sample index,
# and the net drift it predicts across the window must be under TOL_DRIFT_*. Noise
# averages out of a regression, drift does not, which is precisely the split
# needed - the spread gate catches noise, the trend gate catches settling. They
# fail for different reasons and want different fixes (shielding vs patience), so
# measure_steady names which one tripped rather than saying "not steady".
#
# NSAMPLES went 6 -> 12 for the same reason: a trend needs a baseline. 12 samples
# is 7.5 s, comparable to the electrode's own time constant, and TRIES drops to 40
# to keep the per-buffer timeout at roughly five minutes.
#
# What this still does NOT catch is the hours-long offset walk described further
# down. A drift of ~2 mV/hour is ~4 uV across a 7.5 s window - far below any
# threshold that would pass a real settled reading. The trend gate fixes the
# seconds-to-minutes regime only. Soak the probe; no gate here substitutes for it.

# Both of these exist because the MCU's moving average is NOT cleared when a new
# calibration lands (pHMeterThread.cpp keeps folding into the same ring), so every
# calibration change is followed by a window's worth of blended output.
PH_AVG_FLUSH_S=8   # PH_AVG_SIZE=10 at ~1.6 Hz = 6.25 s, plus margin
IDENTITY_TRIES=6   # backstop polls after that sleep, one sample each (~2 s)

# TWO tolerances because the topic reports two different quantities during a run:
# volts under the identity calibration (early steps), pH under the fitted one (the
# final step onwards). One number cannot serve both. TOL_V/16.9 is about 0.034 pH,
# so reusing it on the read-back would demand 34 uV of electrode stability - which
# no glass electrode delivers, and which fails SOFT, silently skipping the one
# check that catches firmware ignoring the request. TOL_PH is that same 2 mV of
# real noise with the headroom the pH domain actually needs.

# THERE IS NO DEAD TIME AFTER A DIP. The old SETTLE_S=60 was a clock standing in
# for a measurement: it waited the same 60 s whether the probe was mid-transient
# or had been sitting in the buffer for a minute already, and it never once
# looked at the data to decide. The drift gate below is the measurement, so the
# loop now starts immediately and the electrode sets its own pace.
#
# The arithmetic says the gate covers what the sleep was covering. A probe moved
# into a new buffer approaches its asymptote exponentially - call it 20 mV away
# with a 30 s time constant. The gate trips on drift > TOL_DRIFT_V that also
# reaches TREND_T standard errors; at n=12 with ~2 mV of noise that is ~1.7 mV
# per 7.5 s window, i.e. 0.23 mV/s, which that transient does not fall under
# until ~32 s in. On a quiet rig SE shrinks, the 0.5 mV floor binds instead, and
# it holds out past 60 s. Either way it waits as long as the electrode needs and
# no longer - and on an already-settled probe (a re-dip, a re-run after an abort)
# it accepts the first window.
#
# The MCU's moving average does not undermine this. A 10-deep box filter delays a
# ramp, it does not flatten it: the averaged output has the same slope as the
# input, so a settling electrode reaches the drift gate at its true rate. What
# the averaging does do is correlate consecutive samples (they share 9 of 10
# entries), which makes the scatter - and therefore SE - an UNDER-estimate, which
# makes the trend test trip sooner. That errs toward more patience, not less.
#
# The 2 mV criterion itself is deliberately UNCHANGED. Removing the dead time is
# not licence to widen the tolerance: widening it would make the script finish by
# accepting readings that had not settled, which is the exact failure the gate
# exists to prevent. Let the data decide when, not whether.
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
RESID_MAX=0.05     # max acceptable |residual| at any buffer, in pH (3-point runs only)
                   # Least squares spreads one bad point across all three, so a
                   # single buffer wrong by d shows up as a worst residual of
                   # about 2d/3. This threshold therefore trips at d ~ 0.075 pH.
                   # Tightened 0.30 -> 0.05 because 0.30 could not see a buffer
                   # wrong by a third of a pH: the 2026-09-02 run below came in
                   # at 0.2166 and would have PASSED at 0.30.
                   #
                   # 0.05 is affordable - the best fit on record is 0.0056 - and
                   # it leaves the rest of the 0.1 pH budget for the ~2 h hold.
                   # It is a gate on the DATA, not a knob on the fit: no
                   # slope/offset pair can lower a residual that least squares
                   # has already minimised, so a run that trips this wants new
                   # readings, never new numbers.
                   # Meaningless with two points, where the fit passes exactly
                   # through both by construction - hence the skip below.
VERIFY_TOL=0.30    # read-back miss above this means the MCU ignored the request
STABILITY_TOL=0.1 # read-back miss above THIS means the electrode moved, in pH
                   # 0.10 because the deviation carries the noise of TWO window
                   # means - the buffer sample the fit was built on, and the
                   # read-back sample - which combine to ~0.025 pH with a
                   # perfectly stable electrode and no drift whatsoever. 0.05 sat
                   # at 2 sigma of that and rejected roughly one clean run in
                   # twenty (observed: a 0.0587 pH failure, 2.4 sigma, pure
                   # noise). Raised 0.10 -> 0.14 when TOL_PH went 0.05 -> 0.08:
                   # a looser spread gate admits a noisier window, whose mean is
                   # correspondingly less certain, so the 4-sigma point moves with
                   # it. The drift this gate exists to catch measured 0.157 pH and
                   # is still caught - but only just, which is why TOL_PH cannot
                   # go higher without giving up the check entirely.
                   # The read-back is compared against the FITTED value at the
                   # last buffer, not the buffer's nominal pH. At two points the
                   # line passes exactly through it, so the expected error is
                   # zero; at three the expected error is that buffer's residual,
                   # which the fit already reports. Either way anything left over
                   # is the electrode moving between the dip and the read-back,
                   # and 0.30 pH was far too coarse to see it - a run drifting
                   # 9 mV (0.157 pH) passed and printed "the MCU applied the
                   # calibration", which was true and beside the point.
RESPONSE_MIN=30    # measured volt span must be >= this % of the theoretical span
ASYM_MAX_MV=60     # |asymmetry potential| at pH 7 must stay under this, in mV
                   # THE OFFSET GATE. Every other check in this script looks at
                   # the SLOPE (response %, health band, sign, residuals) or at
                   # the link (identity, read-back). Nothing looked at the offset,
                   # and that is the hole a leak walks through: a resistive path
                   # on the input attenuates the slope a little and drags the
                   # offset a lot, so a leaky rig produces a fit that passes every
                   # slope-based test while sitting nowhere near the right zero.
                   #
                   # Observed 2026-08-20: a run fitted -18.736416 / 5.452840,
                   # i.e. -82.6 mV asymmetry, and PASSED - the response landed at
                   # 90.2%, clearing the "good" band by 0.2%, with a measured
                   # 2-4 GOhm leak on the input at the time.
                   #
                   # A glass electrode's asymmetry potential lives within +-30 mV.
                   # 60 mV is deliberately generous - twice spec, so a cheap or
                   # aging probe is not rejected for being merely mediocre - and
                   # it still fails that run by 22 mV. All three fits on the
                   # previous probe (-6.2, +3.1, +7.4 mV) pass with room to spare.
                   #
                   # This gate cannot prove a rig is clean. It only catches a leak
                   # big enough to have moved the zero, which is the failure that
                   # was silently producing calibrations.

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
grab() { # $1=how many samples (default NSAMPLES)
    # Count is an argument because the two callers want very different things.
    # The steadiness gate needs a full window to fit a trend through; step 1 only
    # asks which DOMAIN the topic is in, which one sample answers - and at
    # NSAMPLES=12 that would otherwise cost 7.5 s per poll to look at one number.
    #
    # `|| true`: when awk exits after n samples, `ros2 topic echo` dies of
    # SIGPIPE; under pipefail that non-zero status would errexit the script.
    # </dev/null: this runs between `read -rp` prompts, and a child that inherits
    # the terminal can swallow the operator's Enter. Nothing here needs stdin.
    # --no-daemon: the ros2cli daemon is a background process in the container,
    # and ros2cli spawns it in the SHELL'S OWN process group (daemonize.py has no
    # start_new_session on Linux). So a Ctrl+C aimed at this script also SIGINTs
    # the daemon: its rclpy context shuts down while its XML-RPC server keeps
    # answering, and every later `ros2 topic echo` dies in choose_qos with
    # "Fault 1: RuntimeError: !rclpy.ok()" - stderr, which the 2>/dev/null below
    # swallows, so the script just reports "no packets" on a perfectly healthy
    # graph, until someone restarts the container. Nothing here needs the
    # daemon's cache; echo builds its own node either way.
    #
    # Safe for QoS too: with no daemon, choose_qos may find no publishers yet and
    # fall back to its default, which is `sensor_data` (echo.py:41) - BEST_EFFORT,
    # so it still matches Nexus.cpp's KeepLast(1).best_effort() publishers. The
    # reliable-subscriber-vs-best-effort-publisher trap does not apply.
    { timeout 40 ros2 topic echo --no-daemon "$TOPIC_PKT" </dev/null 2>/dev/null || true; } \
        | awk -v n="${1:-$NSAMPLES}" '/^ph:/ { print $2; if (++c == n) exit }'
}

# $1=tolerance; stdin: samples -> mean, or "unstable"/"none"
#
# ABSOLUTE tolerance, unlike calibrate_mass.sh. The quantity being measured here
# passes through zero (0 V is pH 7 on an ideal electrode), so a percentage-of-mean
# tolerance would be unsatisfiable near neutral and far too loose at the ends.
#
# The tolerance is an argument, not $TOL_V, because the caller knows which domain
# the topic is in and this function cannot - see the TOL_V/TOL_PH note above.
# Emits one of: "none", "spread <span>", "drift <net> <remaining>", or the mean.
steady() { # $1=spread tolerance  $2=drift tolerance
    awk -v tol="$1" -v dtol="$2" -v tmin="$TREND_T" '
        { y[NR] = $1; s += $1
          if (NR == 1 || $1 < lo) lo = $1
          if (NR == 1 || $1 > hi) hi = $1 }
        END {
            n = NR
            if (n == 0) { print "none"; exit }

            # A short window CANNOT be judged, so refuse it rather than average it.
            # The trend test needs n >= 4 (n-2 degrees of freedom for the scatter),
            # and below that this function would fall straight through to the mean
            # with no drift check at all - accepting exactly the reading it exists
            # to reject. grab() returns short when the topic is slow or its 40 s
            # timeout fires, which is precisely when the data is least trustworthy.
            if (n < 4) { printf "short %d\n", n; exit }
            if (hi - lo > tol) { printf "spread %.6f\n", hi - lo; exit }

            # Trend against sample index. Below 3 samples there is no baseline to
            # fit, so fall through to the mean rather than invent a slope.
            if (n >= 4) {
                for (i = 1; i <= n; i++) { sx += i; sy += y[i]; sxy += i*y[i]; sxx += i*i }
                slope = (n*sxy - sx*sy) / (n*sxx - sx*sx)
                b0 = (sy - slope*sx) / n
                drift = slope * (n - 1)
                ad = (drift < 0) ? -drift : drift

                # Is that trend REAL, or is it noise fitting a line to itself?
                #
                # Comparing the raw slope to a fixed threshold does not work, and
                # the arithmetic says why: for n samples the drift that pure noise
                # produces has SE = sigma*(n-1)/sqrt(n(n^2-1)/12). At n=12 with the
                # spread gate at its 2 mV limit that is 0.564 mV - so the old fixed
                # 0.500 mV threshold sat at 0.9 sigma and rejected a PERFECTLY
                # settled probe in about a third of windows, forever, with the sign
                # flipping every pass. Observed in the field as
                #   -0.980, +0.782, -0.918 mV per window
                # which is not a settling electrode; a settling electrode drifts
                # the same direction every time.
                #
                # So the scatter about the fitted line sets the scale, and the
                # threshold becomes self-calibrating: quiet signal, tight gate;
                # noisy signal, loose gate. Failing needs the trend to be BOTH
                # statistically real (t >= tmin) AND large enough to bias the fit
                # (> dtol). Wander clears neither for long.
                rss = 0
                for (i = 1; i <= n; i++) { e = y[i] - (slope*i + b0); rss += e*e }
                sxxc = sxx - sx*sx/n
                sd = sqrt(rss/(n-2))
                se = (sxxc > 0) ? sd/sqrt(sxxc)*(n-1) : 0
                # se == 0 means the points sit exactly on the line - zero scatter,
                # so the trend is perfectly determined, i.e. INFINITELY significant.
                # Reading that as t = 0 would let a noiseless ramp through the one
                # gate built to catch ramps. Floating point usually leaves a crumb
                # of residual and hides this, which is what makes it worth pinning.
                t = (se > 0) ? drift/se : (ad > 0 ? tmin : 0)
                at = (t < 0) ? -t : t

                if (at >= tmin && ad > dtol) {
                    # How much further it still has to travel, from the standard
                    # three-point exponential asymptote. Equally spaced and as WIDE
                    # as the window allows: on the real data above a 3-in-a-row
                    # triple overshot by 4x while the widest landed within 20%.
                    # Reported to help the operator decide whether to keep waiting,
                    # never gated on - it is an estimate, not a measurement.
                    k = int((n + 1) / 2)
                    den = y[1] + y[2*k - 1] - 2*y[k]
                    ok = 0
                    if (den > 1e-12 || den < -1e-12) {
                        rem = (y[1]*y[2*k - 1] - y[k]*y[k])/den - y[n]
                        ar = (rem < 0) ? -rem : rem
                        # Trust it only if it points the same way the signal is
                        # moving and is not absurd. A straight ramp drives den to
                        # zero and has no asymptote at all; reporting "0.000 to go"
                        # there would read as "nearly done" and is exactly backwards.
                        if (rem * drift > 0 && ar < 50 * ad) ok = 1
                    }
                    if (ok) printf "drift %.6f %.6f\n", drift, rem
                    else    printf "drift %.6f ?\n", drift
                    exit
                }
            }
            printf "%.6f\n", s / n
        }'
}

# Scale a machine value into display units (volts -> mV, pH -> pH).
disp() { awk -v v="$1" -v s="$2" 'BEGIN { printf "%.3f", v * s }'; }

# |mean(first half) - mean(second half)| and overall mean, for a list of window
# means. See SETTLE_STEP_V for why this shape rather than max-min.
half_step() { awk -v s="$1" 'BEGIN {
    n = split(s, a, " "); h = int(n / 2); lo = 0; hi = 0; t = 0
    for (i = 1; i <= h; i++)         lo += a[i]
    for (i = n - h + 1; i <= n; i++) hi += a[i]
    for (i = 1; i <= n; i++)         t  += a[i]
    d = lo / h - hi / h; if (d < 0) d = -d
    printf "%.6f %.6f", d, t / n }'; }

measure_steady() { # $1=spread tol  $2=drift tol  $3=unit label  $4=display scale  $5=step tol
    local val kind rem i last="" runs="" nruns=0 step avg
    for ((i = 1; i <= TRIES; i++)); do
        val=$(grab | steady "$1" "$2")
        kind=${val%% *}
        # A window that shows the probe MOVING breaks the streak - that is the
        # whole point. A window merely too NOISY does not: noise is a property
        # of the rig, not of the electrode, and dropping the streak for it makes
        # the settle gate hostage to the BNC leak. The lurch case that argues
        # for resetting is already covered, because a probe that settles, moves,
        # and re-settles produces halves that disagree, and the step check below
        # is exactly that test.
        #
        # A consequence worth knowing when reading the log: nruns counts PASSING
        # windows, not attempts, so "7/16" may have taken 7 tries or 20, and the
        # streak's real baseline is >= STEADY_RUNS * 7.5 s - longer whenever
        # noisy windows are interleaved. That errs toward more sensitivity in the
        # step test, never less, so it is left as is. The try counter on each
        # line is what tells you how much of TRIES is gone.
        case "$kind" in none|short|drift) runs=""; nruns=0 ;; esac
        case "$kind" in
            none)
                echo "no packets on $TOPIC_PKT - is the bridge running, and does this board carry the probe?" >&2
                exit 1 ;;
            short)
                last=short
                printf '  only %s samples arrived - topic is slow or stalled (try %d/%d)...\n' \
                    "$(echo "$val" | cut -d' ' -f2)" "$i" "$TRIES" >&2 ;;
            spread)
                last=spread
                printf '  noisy: %s %s spread across the window (try %d/%d)...\n' \
                    "$(disp "$(echo "$val" | cut -d' ' -f2)" "$4")" "$3" "$i" "$TRIES" >&2 ;;
            drift)
                last=drift
                rem=$(echo "$val" | cut -d' ' -f3)
                if [ "$rem" = "?" ]; then
                    # No usable asymptote - a still-linear transient, i.e. early.
                    printf '  still settling: %s %s per window, no asymptote yet (try %d/%d)...\n' \
                        "$(disp "$(echo "$val" | cut -d' ' -f2)" "$4")" "$3" "$i" "$TRIES" >&2
                else
                    printf '  still settling: %s %s per window, ~%s %s to go (try %d/%d)...\n' \
                        "$(disp "$(echo "$val" | cut -d' ' -f2)" "$4")" "$3" \
                        "$(disp "$rem" "$4")" "$3" "$i" "$TRIES" >&2
                fi ;;
            *)
                # A steady window. Hold it and require STEADY_RUNS in a row
                # whose two halves agree - see STEADY_RUNS above for why one
                # window is not enough, and why 4 of them were not either.
                # Sliding, not resetting: once the probe stops moving the oldest
                # window ages out and the run passes, so a long settle costs
                # exactly as long as it takes and no extra penalty at the end.
                runs="${runs:+$runs }$val"
                nruns=$((nruns + 1))
                if [ "$nruns" -gt "$STEADY_RUNS" ]; then
                    runs="${runs#* }"
                    nruns=$STEADY_RUNS
                fi
                if [ "$nruns" -lt "$STEADY_RUNS" ]; then
                    last=settle
                    printf '  steady window %d/%d - holding to confirm it is not still settling (try %d/%d)...\n' \
                        "$nruns" "$STEADY_RUNS" "$i" "$TRIES" >&2
                    continue
                fi
                read -r step avg <<<"$(half_step "$runs")"
                if [ "$(awk -v a="$step" -v t="$5" 'BEGIN { print (a > t) ? "yes" : "no" }')" = "yes" ]; then
                    last=settle
                    printf '  steady but still MOVING: %s %s between the halves of the last %d windows (try %d/%d)...\n' \
                        "$(disp "$step" "$4")" "$3" "$STEADY_RUNS" "$i" "$TRIES" >&2
                    continue
                fi
                # Mean of all STEADY_RUNS windows, not one: with the MCU's
                # 10-deep average a single window mean is barely quieter than a
                # single sample, so this longer average is where the fit's input
                # precision actually comes from.
                echo "$avg"; return ;;
        esac
    done

    # Name the fault that actually kept happening - they have opposite fixes.
    echo >&2
    if [ "$last" = "short" ]; then
        echo "never received a full window of samples. The bridge is publishing, but" >&2
        echo "not fast enough to judge - check the Nexus log for a reconnect loop and" >&2
        echo "'ros2 topic hz $TOPIC_PKT' for the real rate (expected ~1.6 Hz)." >&2
    elif [ "$last" = "settle" ]; then
        echo "individual windows were steady, but they never AGREED with each other" >&2
        echo "across ${STEADY_RUNS} in a row. That is slow settling: too gradual to show up" >&2
        echo "inside one 7.5 s window, plainly visible over the ~1 min the streak spans." >&2
        echo "It is the failure" >&2
        echo "mode that used to pass silently and land in the fit as a residual." >&2
        echo "Soak the probe for a few hours - a dry-stored electrode's gel layer" >&2
        echo "takes that long to form - and rinse from a near-neighbour buffer" >&2
        echo "rather than jumping across the range." >&2
    elif [ "$last" = "drift" ]; then
        echo "reading never stopped drifting. The electrode is still equilibrating, which" >&2
        echo "after this long usually means it was stored dry and its gel layer is still" >&2
        echo "forming - soak it for a few hours and start again. A probe moved between" >&2
        echo "buffers of very different pH also takes longer than one that was rinsed" >&2
        echo "from a near neighbour." >&2
    else
        echo "reading never settled - check the probe is fully immersed, the solution is" >&2
        echo "still, and the BNC shell is connected (a floating input wanders forever)." >&2
        echo "At ~340 MOhm source impedance a damp or dirty BNC insulator leaks enough" >&2
        echo "to swamp the signal; clean it with IPA and dry it completely." >&2
    fi
    exit 1
}

# Leaving the MCU on the identity transform is a silent trap: the topic keeps
# publishing, but it publishes volts labelled as pH. Say so on every abnormal exit
# - but ONLY while that is actually true. The window opens when step 1 sends the
# identity and closes when the last step sends the fit, so this is gated on its own
# flag rather than on "did we finish": an exit at the span check has sent the MCU
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

# Join the buffer list for display: "4.01, 6.86, 9.18"
ph_list=$(printf '%s, ' "${PH[@]}"); ph_list=${ph_list%, }
echo "== pH calibration: ${NPOINTS}-point, buffers ${ph_list} =="
if [ "$NPOINTS" -eq 2 ]; then
    echo "   two points fit a line exactly, so this run cannot tell you whether the"
    echo "   electrode is healthy - only that it responds at all. Use three buffers"
    echo "   when the answer matters."
fi

# Reject buffers too close together before asking the operator to do any work: the
# fit is a division by the volt span, so a narrow span amplifies noise without bound.
span_ok=$(printf '%s\n' "${PH[@]}" | awk -v m="$PH_SPAN_MIN" '
    NR == 1 { lo = $1; hi = $1 }
    { if ($1 < lo) lo = $1; if ($1 > hi) hi = $1 }
    END { print (hi - lo >= m) ? "yes" : "no" }')
if [ "$span_ok" != "yes" ]; then
    echo "buffers span less than $PH_SPAN_MIN pH - a fit over that range is noise." >&2
    echo "Use a spread set, e.g. 4.01 6.86 9.18 (or 4.01 9.18 for a 2-point run)." >&2
    exit 1
fi

# Source the ROS environment if the shell we were handed has none.
#
# There are only two places the container ever sources it, and a script started
# with `docker exec` hits neither: /ros_entrypoint.sh is the image ENTRYPOINT, so
# it runs for `docker run` and is skipped by `docker exec`; ~/.bashrc (Dockerfile)
# is read by bash only when INTERACTIVE, and running a script is not, -it or not.
# On the laptop that never shows, because attach.sh opens an interactive shell and
# the script inherits its env. On the RP it always shows: erc_run_avionics.sh
# leaves no shell to attach to, so `docker exec <ctr> ~/scripts/calibrate_ph.sh`
# is the normal way to run this - and it arrives here with no ros2 at all.
#
# set +u around the sourcing: the ROS/colcon setup files read unbound variables
# (AMENT_TRACE_SETUP_FILES and friends), which under this script's -u aborts the
# run inside setup.bash with an error that points nowhere near the real cause.
if ! command -v ros2 >/dev/null 2>&1; then
    set +u
    if [ -f /opt/ros/humble/setup.bash ]; then . /opt/ros/humble/setup.bash; fi
    if [ -f "$HOME/dev_ws/install/setup.bash" ]; then . "$HOME/dev_ws/install/setup.bash"; fi
    set -u
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

echo "[1/$TOTAL_STEPS] slope := 1.0, offset := 0.0 (topic now shows raw volts)"
send_cal 1.0 0.0
IDENTITY_LOADED=1

# Wait out the MCU's moving average before believing anything on the topic.
#
# There IS a filter window to flush here, and assuming otherwise is what made this
# check fail on the first run of the day and pass on the second. pHMeterThread.cpp
# folds every sample into the same PH_AVG_SIZE=10 ring and never clears it when a
# calibration arrives, so for 6.25 s after the switch the published mean is a BLEND
# of pH-domain samples (~6.86) and volt-domain ones (~0.016).
#
# The blend does not cross the 0.6 V gate until the window is COMPLETELY flushed -
# at 9 of 10 samples converted it still reads 0.70, and a real failure looked
# exactly like a reading of 0.89 (8.7 samples in). The old `sleep 2` was racing
# 6.25 s of flush plus however long `ros2 topic echo` needs to discover the
# publisher, and lost roughly half the time. The second run then passed for the
# wrong reason: the MCU was still holding the identity from the aborted first run,
# so the window was already full of volts.
sleep "$PH_AVG_FLUSH_S"

# Confirm the MCU APPLIED it, rather than trusting a publish that structurally
# cannot report delivery (see send_cal). If the identity never lands, the dips
# sample pH while believing they are sampling volts, and the fit is a line through
# the wrong quantity: buffer pH against itself, so slope 1.0 pH/V, offset 0, and
# residuals of exactly zero. The residual guard is blind to it.
#
# It is caught eventually - the health check reports ~1690% of theoretical and the
# read-back fails - but only after the operator has rinsed and settled every
# buffer for nothing. Failing here costs half a minute instead of ten minutes, and
# names the actual cause instead of the two symptoms.
#
# The test is a range check, because the two domains cannot overlap: under the
# identity the topic carries volts, which the ADS1114's +-0.512 V PGA bounds, and
# any usable calibration buffer reads pH 4-9. One sample separates them.
#
# POLLED, not a single sample. The flush above is sized from the firmware's own
# constants, but a sleep alone is still a guess about discovery latency - so the
# sleep gets the common case there in one shot, and this loop is what makes the
# answer independent of timing. Each pass reads the MOST RECENT sample, which is
# the most flushed one in the batch.
identity_ok=no
first_v=""
for ((i = 1; i <= IDENTITY_TRIES; i++)); do
    # One sample: this is a domain test, not a measurement. Nothing here cares
    # what the probe is sitting in or whether it has settled - only whether the
    # number is small enough to be volts rather than large enough to be a pH.
    batch=$(grab 1)
    [ -z "$batch" ] && continue
    first_v=${batch##*$'\n'}   # last line = newest sample
    if [ "$(awk -v v="$first_v" 'BEGIN { if (v < 0) v = -v; print (v <= 0.6) ? "yes" : "no" }')" = "yes" ]; then
        identity_ok=yes
        break
    fi
    echo "  MCU average still flushing (reads ${first_v}, try $i/$IDENTITY_TRIES)..."
done

if [ -z "$first_v" ]; then
    echo "no packets on $TOPIC_PKT - is the bridge running, and does this board carry the probe?" >&2
    exit 1
fi
if [ "$identity_ok" != "yes" ]; then
    echo >&2
    echo "CALIBRATION FAILED: the identity calibration was not applied." >&2
    echo "$TOPIC_PKT still reads ${first_v}, which is a pH value - under slope 1.0 /" >&2
    echo "offset 0.0 it would be volts, bounded by the ADC's +-0.512 V range." >&2
    echo >&2
    echo "This is not the averaging window: that flushes in 6.25 s and this check" >&2
    echo "waited it out, so the reading above is settled, not a blend." >&2
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
for ph in "${PH[@]}"; do
    i=$((i + 1))
    read -rp "[$((i + 1))/$TOTAL_STEPS] rinse the probe, immerse it in the pH ${ph} buffer, then press Enter... "
    echo "  waiting for a steady reading: ${STEADY_RUNS} windows that agree end to end,"
    echo "  so the electrode sets the pace (expect 2-5 min on a well-soaked probe)..."
    v=$(measure_steady "$TOL_V" "$TOL_DRIFT_V" mV 1000 "$SETTLE_STEP_V")   # VOLTS here (identity calibration)
    printf '  pH %-6s -> %s V\n' "$ph" "$v"
    readings+="$v $ph"$'\n'
done

echo "[$STEP_FIT/$TOTAL_STEPS] fitting ph = slope * volts + offset through $NPOINTS points"

# Least squares rather than solving through two of the points: with three points
# the line that minimises total error is the honest answer, and the residuals it
# leaves are the diagnostic. Picking two points would hide the third's disagreement.
# With two points least squares degenerates to the line through both, which is the
# right answer there too - the same code covers both cases.
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
    echo "the readings are all at the same voltage: the electrode is not responding." >&2
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
# This one works at two points as well as three, which is why it is the main guard
# left standing on a 2-point run.
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

# --- offset gate ------------------------------------------------------------
#
# The asymmetry potential: the EMF this electrode gives in a pH 7 buffer, which
# for an ideal one is 0 mV. Checked BEFORE the fit is sent, so a rig that fails
# here never has the bad pair applied to the MCU at all.
asym=$(awk -v s="$slope" -v o="$offset" 'BEGIN { printf "%.1f", (7.00 - o)/s*1000 }')
echo "  asymmetry potential: ${asym} mV at pH 7 (ideal 0, healthy within +-30)"

if [ "$(awk -v a="$asym" -v m="$ASYM_MAX_MV" 'BEGIN { if (a < 0) a = -a; print (a > m) ? "yes" : "no" }')" = "yes" ]; then
    echo >&2
    echo "CALIBRATION FAILED: asymmetry potential ${asym} mV exceeds +-${ASYM_MAX_MV}." >&2
    echo "The electrode's zero is too far from where any glass electrode sits." >&2
    echo >&2
    echo "The slope can look fine while this is wrong, which is why it is checked" >&2
    echo "separately: a resistive leak on the input attenuates the slope only a" >&2
    echo "little but drags the offset a lot, so a leaky rig passes every response" >&2
    echo "and residual test and still calibrates to the wrong zero." >&2
    echo >&2
    echo "To tell a leak from a genuinely odd electrode: dry the BNC connector" >&2
    echo "thoroughly and refit. If the asymmetry moves substantially toward zero," >&2
    echo "it was the connector. If it does not, it is the probe - and if that probe" >&2
    echo "is otherwise healthy, raise ASYM_MAX_MV deliberately rather than by" >&2
    echo "accident." >&2
    echo "Do not hardcode these values." >&2
    exit 1
fi

echo "[$TOTAL_STEPS/$TOTAL_STEPS] sending the fitted calibration"
send_cal "$slope" "$offset"

# IDENTITY_LOADED stays 1 until the read-back PROVES the MCU took the new line.
# Clearing it here assumed the send landed, and send_cal structurally cannot tell
# you that (see its note on best-effort QoS). If the fit never arrives, the board
# is still on the identity and still publishing volts - exactly when the trap's
# warning matters - and the old ordering suppressed it precisely then.

# Same flush as step 1, and for the same reason: the MCU's ring still holds
# volt-domain samples from the identity, so for 6.25 s the topic publishes a
# blend of volts and pH. Without this the read-back's first window spans that
# blend and reports a multi-pH "spread" that is not noise at all - it is the
# filter emptying. measure_steady would retry past it, but it names the wrong
# fault while doing so.
sleep "$PH_AVG_FLUSH_S"

# --- check the line back against every buffer -------------------------------
#
# Two different checks, and they catch different things. The residuals are the fit
# disagreeing with the buffers, computed from readings already taken - that is a
# statement about the ELECTRODE, and it only exists at three points. The live
# read-back is the MCU disagreeing with the fit - that is a statement about the
# LINK, it works at any number of points, and it is the one that catches firmware
# that silently ignored the request.

echo
if [ "$NPOINTS" -ge 3 ]; then
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
else
    # Deliberately NOT printing near-zero residuals here. They would be a measure of
    # awk's floating point, not of the electrode, and a table of +0.000 reads as
    # evidence of a good calibration to anyone skimming the log.
    echo "residuals: not checked - a line through two points passes through both by"
    echo "  construction, so there is no disagreement left to measure. The electrode"
    echo "  response above (${pct}% of theoretical) is the only health signal this run"
    echo "  produced; re-run with three buffers to get a residual check."
fi

echo
echo "live read-back (probe is still in the pH ${PH_LAST} buffer):"
# TOL_PH, not TOL_V: the MCU is publishing pH again as of the last step. This one
# is a hard failure rather than a skip - an unverifiable read-back is exactly what
# a board that ignored the request looks like, so passing the run anyway would
# defeat the only check that can tell the difference.
verif=$(measure_steady "$TOL_PH" "$TOL_DRIFT_PH" pH 1 "$SETTLE_STEP_PH")

# Compare against the FITTED value at this buffer, not its nominal pH. Those are
# the same number at two points and differ by that buffer's residual at three, so
# using the nominal silently folds the residual into the read-back budget and
# leaves no tolerance to spend on what this check is actually for.
v_last=$(printf '%s' "$readings" | tail -1 | cut -d' ' -f1)
pred_last=$(awk -v m="$slope" -v b="$offset" -v v="$v_last" 'BEGIN { printf "%.4f", m*v + b }')
dev=$(awk -v a="$verif" -v b="$pred_last" 'BEGIN { d = a - b; printf "%.4f", (d < 0) ? -d : d }')
echo "  reads ${verif} pH; the fit predicts ${pred_last} for this buffer (nominal ${PH_LAST})"
echo "  deviation: ${dev} pH"

# Two thresholds on one measurement, because the size of the miss names the fault.
# A board that ignored the request is wrong by whole pH units; an electrode that
# moved between the dip and the read-back is wrong by tenths. Collapsing them into
# a single 0.30 pH gate reported the second as a pass.
if [ "$(awk -v d="$dev" -v t="$VERIFY_TOL" 'BEGIN { print (d > t) ? "yes" : "no" }')" = "yes" ]; then
    echo >&2
    echo "CALIBRATION FAILED: read-back is ${dev} pH from the fitted value, over ${VERIFY_TOL}." >&2
    echo "A miss this large is the MCU never applying the request - firmware/bridge" >&2
    echo "wire-format mismatch on the 9-byte PhRequest. Reflash the firmware." >&2
    echo "Do not hardcode these values." >&2
    exit 1
fi

# Past the link check: whatever else is wrong, the MCU is applying the fitted
# line, not the identity, so the trap must stop claiming the topic carries volts.
IDENTITY_LOADED=0

if [ "$(awk -v d="$dev" -v t="$STABILITY_TOL" 'BEGIN { print (d > t) ? "yes" : "no" }')" = "yes" ]; then
    echo >&2
    echo "CALIBRATION FAILED: read-back is ${dev} pH from the fitted value (limit ${STABILITY_TOL})." >&2
    echo "The MCU DID apply the calibration - a miss this small could not survive a" >&2
    echo "board that ignored it. The electrode moved between the dip and the read-back," >&2
    echo "so the voltage the fit was built on is not the voltage it reads now, and every" >&2
    echo "buffer in this run is suspect for the same reason." >&2
    echo >&2
    echo "Watch $TOPIC_PKT for a few minutes before re-running. A reading that creeps" >&2
    echo "one way and slows is equilibrating: soak the probe for hours and try again." >&2
    echo "A reading that WANDERS - down, then back up - is not settling at all, it is" >&2
    echo "an unstable input: clean and dry the BNC, and check the shell connection." >&2
    echo "Do not hardcode these values." >&2
    exit 1
fi
echo "  within ${STABILITY_TOL} pH of the fit - calibration applied and the electrode held still."
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
