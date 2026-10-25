package org.firstinspires.ftc.teamcode.Subsystems.turret;

import com.bylazar.configurables.annotations.Configurable;

/**
 * TurretCrt — the CRT (Chinese Remainder Theorem) angle math for the turret's
 * TWO absolute encoders, broken out of {@link TurretPivot} so the tracking
 * subsystem stays readable.
 *
 * <p><b>The problem it solves:</b> a single absolute encoder only knows its
 * angle within ONE turn of the ENCODER shaft. Because the encoder pinion
 * turns several times per turret turn, many different turret angles produce
 * the same encoder reading. With a SECOND encoder on a pinion with a
 * DIFFERENT tooth count, only one turret angle makes both readings agree at
 * once — so at boot we can compute the turret's TRUE multi-turn angle from
 * two single-turn readings. (Same trick as sharing a secret between two
 * clocks that repeat at different rates — hence "Chinese Remainder Theorem".)
 *
 * <p><b>This robot's gearing:</b> the 176T turret ring spins a 64T idler, and
 * the idler drives BOTH encoder pinions: 31T (encoder A) and 32T (encoder B).
 * Each pinion turns {@code 176 / pinionTeeth} per turret turn:
 * A = 176/31 ≈ 5.677, B = 176/32 = 5.5. The (A, B) pair is unique over
 * lcm(31, 32)/176 ≈ 5.64 turret turns — far more than the ±180° travel — so
 * the boot solve always has exactly one true answer.
 *
 * <p><b>Frame convention (SESSION 5b):</b> the zero offsets below are RAW
 * magnet-frame values, subtracted from the raw reading BEFORE the direction
 * flips are applied. That ordering makes the calibration immune to flipping
 * {@link #ENCODER_A_REVERSED} (or the tracking encoder's flip): a flip only
 * mirrors the rotation direction, never the zero.
 *
 * <p>Everything here is static and stateless — {@link TurretPivot} calls into
 * it at boot (the solve) and every loop (the encoder-A cross-check).
 */
@Configurable
public final class TurretCrt {

    private TurretCrt() {
        // static-only class
    }

    // =========================================================================
    // Calibration + tuning (Panels -> TurretCrt)
    // =========================================================================

    /**
     * Teeth on encoder A's pinion (the CRT partner). Encoder B's pinion is the
     * 32T one in TurretPivot's geometry; A must differ from B or the CRT pair
     * is not unique.
     */
    public static final double CRT_ENCODER_A_TEETH = 31.0;

    /**
     * CALIBRATION (do this ONCE at the bench, same idea as FRC
     * CRT1ZeroOffset/CRT2ZeroOffset): park the turret at FORWARD (0 deg),
     * read the two raw dashboards ("Turret CRT A raw us" / "Turret abs raw
     * us"), and enter the raw ENCODER revolutions [0,1) for each (raw us / 1024).
     *
     * <p>RAW magnet-frame values, subtracted from the readings BEFORE the
     * direction flips are applied — so flipping a reversal flag (e.g. after
     * seeing the dashboard rotate the wrong way) only mirrors the direction
     * and NEVER invalidates the calibration.
     */
    public static double CRT_ZERO_OFFSET_A_REV = 657.0 / 1024.0;  // "Turret CRT A raw us": 657
    public static double CRT_ZERO_OFFSET_B_REV = 776.0 / 1024.0;  // "Turret abs raw us": 776

    /** True once both CRT zero offsets have been calibrated at the bench. */
    public static boolean CRT_ZERO_CALIBRATED = true;

    /**
     * CRT match tolerance in ENCODER revolutions: a candidate turret angle is
     * accepted when the measured partner reading is within this many encoder
     * revolutions of its prediction. Widen until the solve never comes up
     * empty at boot, but keep it below half the candidate spacing
     * (1/(2*32) = 0.0156 enc-revs on B) or two candidates can both "match".
     */
    public static double CRT_MATCH_TOL_ENC_REVS = 0.03;

    /** Documentation constant: the soft-limit travel window plus a small overshoot margin (turns). */
    public static double CRT_TRAVEL_MARGIN_TURNS = 0.25;

    /** Documentation constant: turns over which the (A, B) pair is unique (31*32/176 ≈ 5.64). */
    public static final double CRT_UNIQUE_RANGE_TURNS = 31.0 * 32.0 / 176.0;

    // ---- Every-loop encoder-A cross-check (slipped-magnet / drift watchdog) ----

    /** Base cross-check tolerance (turret deg) at standstill. */
    public static double CRT_CROSSCHECK_TOL_DEG = 3.0;

    /** Consecutive failed cross-checks (below the cutoff speed) before latching the fault. */
    public static int CRT_CROSSCHECK_FAIL_LIMIT = 5;

    /**
     * Speed-aware tolerance growth (s). Encoder A's PWM refreshes at ~10 Hz,
     * so while MOVING its reading is up to ~0.1 s stale vs the tracked stream;
     * the tolerance grows as {@code base + CRT_CROSSCHECK_STALE_S * |vel|} so
     * the check keeps running at speed without latching on legitimate
     * staleness. A real slip is a persistent offset on top of this.
     */
    public static double CRT_CROSSCHECK_STALE_S = 0.10;

    /**
     * Skip the cross-check entirely above this speed (deg/s) — above it a
     * full-power slew outruns the ~10 Hz refresh many times over. 0 = always
     * check (tolerance fully speed-scaled).
     */
    public static double CRT_CHECK_MAX_SPEED_DPS = 120.0;

    /**
     * Software sign flip for encoder A (the CRT partner). Each Through Bore's
     * magnet orientation is independent, so this is tuned per encoder. Flip it
     * if the boot solve comes up empty with a big predicted-vs-measured
     * mismatch. Flipping only mirrors A's direction — the zero offsets are
     * RAW-frame and never need re-entry.
     */
    public static boolean ENCODER_A_REVERSED = true;

    // =========================================================================
    // Frame helpers
    // =========================================================================

    /** Wrap any value into [0, 1). */
    public static double mod1(double v) {
        double r = v % 1.0;
        if (r < 0) {
            r += 1.0;
        }
        return r;
    }

    /**
     * Wrap-aware difference of two [0,1) fractions, centered on 0: result in [-0.5, 0.5).
     * CONVENTION: {@code fracDiff(x, y) = x - y} (shortest path) — "measured minus
     * predicted". For accumulating MOTION between two readings use
     * {@link #forwardDiffRevs} instead (its argument-swapped sibling); mixing the
     * two up silently REVERSES the tracked stream (see TURRET_PORTING_NOTES §12.20).
     */
    public static double fracDiff(double x, double y) {
        return mod1(x - y + 0.5) - 0.5;
    }

    /**
     * Shortest signed distance FROM {@code from} TO {@code to} in [0,1) fractions,
     * wrapped to [-0.5, 0.5) — i.e. {@code to - from}, seam-crossing safe. This is
     * the one to use for "how much did it move since last loop" accumulation.
     */
    public static double forwardDiffRevs(double from, double to) {
        return mod1(to - from + 0.5) - 0.5;
    }

    /**
     * Convert a RAW encoder fractional revolution into the SOLVE frame: the
     * zero offset is subtracted from the RAW reading FIRST (0 = turret
     * forward), THEN the direction flip mirrors the rotation. See the class
     * javadoc for why that ordering matters.
     */
    public static double solveFrameRevs(double rawRevs, double zeroOffsetRev, boolean reversed) {
        double zeroed = mod1(rawRevs - zeroOffsetRev);   // RAW frame: 0 at turret-forward
        return reversed ? mod1(1.0 - zeroed) : zeroed;   // THEN the direction flip
    }

    // =========================================================================
    // The boot solve
    // =========================================================================

    /** Result of {@link #solve}: always returned, check {@link #solved}. */
    public static final class SolveResult {
        /** Solved turret angle in TURNS (not degrees, not wrapped) — NaN when unsolved. */
        public final double turretTurns;
        /** Match diff (encoder-A revs, ±0.5-centered) of the winning candidate; NaN when unsolved. */
        public final double matchDiffEncRevs;
        /** How many turret-turn candidates were considered (telemetry/debug). */
        public final int candidates;
        /** True when a candidate matched within tolerance. */
        public final boolean solved;

        SolveResult(double turretTurns, double matchDiffEncRevs, int candidates, boolean solved) {
            this.turretTurns = turretTurns;
            this.matchDiffEncRevs = matchDiffEncRevs;
            this.candidates = candidates;
            this.solved = solved;
        }
    }

    /**
     * The CRT boot solve — the same algorithm as FRC EasyCRT, hand-ported.
     *
     * <p>Given both encoders' RAW fractional revolutions (pre-reversal, as read
     * straight off the pulse-width train):
     * <ol>
     *   <li>{@link #solveFrameRevs} puts both fractions relative to
     *       turret-forward and sign-aligned to "positive turret rotation reads
     *       positive".</li>
     *   <li>Enumerate every turret turn {@code t} in the travel window that
     *       satisfies encoder B: {@code t = (bFrac + k) / R_B} for integer k
     *       (5–6 candidates in a ±180° window).</li>
     *   <li>For each candidate predict encoder A: {@code aPred = frac(R_A * t)}
     *       and take the wrap-aware difference to the measured A fraction.</li>
     *   <li>The candidate with the smallest difference wins, IF it is within
     *       {@link #CRT_MATCH_TOL_ENC_REVS}; otherwise {@code solved == false}.</li>
     * </ol>
     *
     * @param aRawRevs       encoder A RAW fractional revolution [0, 1) (pre-reversal)
     * @param bRawRevs       encoder B RAW fractional revolution [0, 1) (pre-reversal)
     * @param reversedB      encoder B's direction flip (from the tracking source)
     * @param revsPerTurretB encoder-B revolutions per ONE turret revolution (176/32 = 5.5)
     * @param minTurns       travel window low end, turret turns (e.g. -180/360)
     * @param maxTurns       travel window high end, turret turns (e.g. +180/360)
     * @return the solve result — check {@link SolveResult#solved}
     */
    public static SolveResult solve(double aRawRevs, double bRawRevs, boolean reversedB,
                                    double revsPerTurretB, double minTurns, double maxTurns) {
        // SESSION 5b: offsets are RAW-frame — subtract them from the raw
        // readings BEFORE the direction flips (see solveFrameRevs).
        double b = solveFrameRevs(bRawRevs, CRT_ZERO_OFFSET_B_REV, reversedB);
        double a = solveFrameRevs(aRawRevs, CRT_ZERO_OFFSET_A_REV, ENCODER_A_REVERSED);

        final double rA = encoderARevsPerTurretRev();

        // Candidates: t = (b + k) / rB for every integer k that lands in the window.
        // k range: minTurns <= (b + k)/rB <= maxTurns  =>  k in [minTurns*rB - b, maxTurns*rB - b].
        int kStart = (int) Math.ceil(minTurns * revsPerTurretB - b);
        int kEnd = (int) Math.floor(maxTurns * revsPerTurretB - b);
        int candidateCount = Math.max(0, kEnd - kStart + 1);

        double bestT = Double.NaN;
        double bestDiff = Double.POSITIVE_INFINITY;
        for (int k = kStart; k <= kEnd; k++) {
            double t = (b + k) / revsPerTurretB;
            if (t < minTurns || t > maxTurns) {
                continue;
            }
            double aPred = mod1(rA * t);
            double diff = Math.abs(fracDiff(a, aPred));
            if (diff < bestDiff) {
                bestDiff = diff;
                bestT = t;
            }
        }
        boolean solved = !Double.isNaN(bestT) && bestDiff <= CRT_MATCH_TOL_ENC_REVS;
        return new SolveResult(solved ? bestT : Double.NaN,
                Double.isNaN(bestT) ? Double.NaN : bestDiff, candidateCount, solved);
    }

    /** Encoder-A revolutions per ONE turret revolution (176/31 ≈ 5.677). */
    public static double encoderARevsPerTurretRev(double turretRingTeeth) {
        return turretRingTeeth / CRT_ENCODER_A_TEETH;
    }

    /** Same, using this robot's 176T ring (convenience overload). */
    public static double encoderARevsPerTurretRev() {
        return encoderARevsPerTurretRev(176.0);
    }

    /**
     * Every-loop cross-check: how far encoder A's solve-frame reading is from
     * what the tracked turret angle predicts, in TURRET degrees (wrap-aware).
     * Positive/negative is a signed disagreement, not a direction.
     *
     * @param aFrac         encoder A's solve-frame fraction [0,1) (see {@link #solveFrameRevs})
     * @param turretTurns   the tracked turret angle in turns
     * @param revsPerTurretA encoder-A revolutions per ONE turret revolution (176/31 here)
     */
    public static double crosscheckDiffDeg(double aFrac, double turretTurns, double revsPerTurretA) {
        double aPred = mod1(revsPerTurretA * turretTurns);
        double diffEncRevs = fracDiff(aFrac, aPred);
        return diffEncRevs / revsPerTurretA * 360.0;
    }
}
