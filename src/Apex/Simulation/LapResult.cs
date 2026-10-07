using System.Collections.Immutable;

namespace Apex.Simulation;

/// <summary>The full outcome of one simulated lap, per station</summary>
public sealed record LapResult(
    double LapTime,
    ImmutableArray<double> Speed,
    ImmutableArray<double> Curvature,
    ImmutableArray<double> SegmentLength)
{
    public double Distance => SegmentLength.Sum();
    public double MaxAbsCurvature => Curvature.Max(Math.Abs);
}
