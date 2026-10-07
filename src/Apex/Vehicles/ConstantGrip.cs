namespace Apex.Vehicles;

/// <summary>Grip that does not depend on speed: a car with no downforce.</summary>
public sealed class ConstantGrip(double lateral, double longitudinal) : IGripModel
{
    public double Lateral { get; } = lateral > 0 ? lateral : throw new ArgumentOutOfRangeException(nameof(lateral));

    public double Longitudinal { get; } = longitudinal > 0 ? longitudinal : throw new ArgumentOutOfRangeException(nameof(longitudinal));

    public double LateralLimit(double speed) => Lateral;

    public double LongitudinalLimit(double speed) => Longitudinal;

    /// <summary>Closed form: v = sqrt(a_y / |k|). No iteration needed.</summary>
    public double CorneringSpeed(double absCurvature, double topSpeed) =>
        absCurvature <= Lateral / (topSpeed * topSpeed)
            ? topSpeed
            : Math.Sqrt(Lateral / absCurvature);
}
