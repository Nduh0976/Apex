using Apex.Geometry;

namespace Apex.Simulation;

/// <summary>
/// Every buffer one lap-time evaluation needs, allocated once. A workspace is
/// not thread-safe: give each thread its own.
/// </summary>
public sealed class LapWorkspace
{
    public LapWorkspace(int count)
    {
        Count = count;
        Points = new Vec2[count];
        SegmentLength = new double[count];
        Curvature = new double[count];
        Ceiling = new double[count];
        Forward = new double[count];
        Backward = new double[count];
        Spline = new SplineWorkspace(count);
    }

    public int Count { get; }

    /// <summary>The racing line's points. Callers fill this before evaluating.</summary>
    public Vec2[] Points { get; }

    internal double[] SegmentLength { get; }
    internal double[] Curvature { get; }
    internal double[] Ceiling { get; }
    internal double[] Forward { get; }
    internal double[] Backward { get; }
    internal SplineWorkspace Spline { get; }
}
