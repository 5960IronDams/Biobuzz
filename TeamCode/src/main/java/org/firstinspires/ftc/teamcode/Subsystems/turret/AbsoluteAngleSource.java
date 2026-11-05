package org.firstinspires.ftc.teamcode.Subsystems.turret;

import java.util.OptionalDouble;

/**
 * A single-turn absolute angle source for the turret: returns the ENCODER-shaft
 * angle as revolutions in [0, 1), in the SAME direction convention as the
 * tracked angle after the source's direction flip is applied. Empty when the
 * source has no fresh valid data this loop.
 *
 * <p>The production source is {@link OctoQuadAbsoluteSource} (an OctoQuad
 * pulse-width channel reading a REV Through Bore's absolute PWM output) — the
 * Control Hub itself cannot decode that PWM train.
 */
public interface AbsoluteAngleSource {

    /** Latest absolute reading (encoder revolutions [0,1)), or empty on a bad/missing read. */
    OptionalDouble readRevs();

    /**
     * The CRT partner encoder's (encoder A) RAW fractional revolution, decoded
     * from the SAME transaction as the last {@link #readRevs()}. Empty when the
     * source is single-encoder, the read failed, or encoder A was invalid.
     * Never performs its own I/O — always pair with a fresh {@link #readRevs()}.
     */
    OptionalDouble readCrossRevs();

    /** Human-readable identity for logs/telemetry. */
    String describe();
}
