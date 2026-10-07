namespace Apex.Vehicles;

/// <summary>
/// How much acceleration the tyres can produce. Speed is an input because
/// aerodynamic downforce pushes the car into the road harder as it goes
/// faster, which raises the grip limit.
/// </summary>
public interface IGripModel
{
    /// <summary>Maximum lateral (cornering) accelerationn at this speed, m/s^2.</summary>
    double LateralLimit(double speed);

    /// <summary>Maximum tyre-limited longitudinal acceleration (driving or braking) at this speed, m/s^2.</summary>
    double LongitudinalLimit(double speed);

    /// <summary>
    /// The speed at which cornering on this curvature uses all the lateral
    /// grip: the root of v^2 * |k| = LateralLimit(v), capper at topSpeed.
    /// The default implementation bisects, which works for any model whose
    /// grip grows no faster than v^2 |k|. Models with a closed form override it.
    /// </summary>
    double CorneringSpeed(double absCurvature, double topSpeed) =>
        CorneringSolver.Bisect(this, absCurvature, topSpeed);
}
