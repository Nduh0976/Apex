using Apex.Tracks;

namespace Apex.Optimisation;

/// <summary>Box bounds on the lateral offset at every station</summary>
public sealed class OffsetBounds
{
    public OffsetBounds(Track track, double margin)
    {
        ArgumentNullException.ThrowIfNull(track);
        ArgumentOutOfRangeException.ThrowIfNegative(margin);

        Lower = new double[track.Count];
        Upper = new double[track.Count];

        for (var i = 0; i < track.Count; i++)
        {
            Lower[i] = track.LowerBound(i, margin);
            Upper[i] = track.UpperBound(i, margin);
        }
    }

    public double[] Lower { get; }
    public double[] Upper { get; }

    /// <summary>Projection onto the box: the closest feasible point is a per-element clamp</summary>
    public void Project(Span<double> offsets)
    {
        for (var i = 0; i < offsets.Length; i++)
        {
            offsets[i] = Math.Clamp(offsets[i], Lower[i], Upper[i]);
        }
    }
}
