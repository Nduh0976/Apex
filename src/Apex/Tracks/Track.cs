using System.Collections.Immutable;
using Apex.Geometry;

namespace Apex.Tracks;

/// <summary>
/// A closed circuit: centreline points, the unit left-normal at each point,
/// and the usable width either side. Immutable, so one instance can be shared
/// across every thread in the optimiser without locks
/// </summary>
public sealed class Track
{
    public Track(
        string name,
        ImmutableArray<Vec2> centre,
        ImmutableArray<double> widthRight,
        ImmutableArray<double> widthLeft)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(name);

        if (centre.Length < 3)
        {
            throw new ArgumentException("A track must have at least three points.", nameof(centre));
        }

        if (widthRight.Length != centre.Length || widthLeft.Length != centre.Length)
        {
            throw new ArgumentException("Width arrays must match the number of centre points.");
        }

        Name = name;
        Centre = centre;
        WidthRight = widthRight;
        WidthLeft = widthLeft;

        var n = centre.Length;
        var normals = ImmutableArray.CreateBuilder<Vec2>(n);
        var length = 0.0;

        for (var i = 0; i < n; i++)
        {
            var prev = centre[i == 0 ? n - 1 : i - 1];
            var next = centre[i == n - 1 ? 0 : i + 1];

            normals.Add((next - prev).Normalized().PerpLeft);
            length += Vec2.Distance(centre[i], next);
        }

        Normal = normals.MoveToImmutable();
        Length = length;
    }

    public string Name { get; }
    public ImmutableArray<Vec2> Centre { get; }
    public ImmutableArray<Vec2> Normal { get; }
    public ImmutableArray<double> WidthRight { get; }
    public ImmutableArray<double> WidthLeft { get; }
    public int Count => Centre.Length;
    public double Length { get; }

    /// <summary> Most negative (rightward) offset allowed at station i.</summary>
    public double LowerBound(int i, double margin) => Math.Min(0.0, WidthRight[i] - margin);

    /// <summary> Most positive (leftward) offset allowed at station i.</summary>
    public double UpperBound(int i, double margin) => Math.Max(0.0, WidthLeft[i] - margin);

    /// <summary> p[i] = centre[i] + offset[i] * Normal[i]. Allocation-free</summary>
    public void PointsAt(ReadOnlySpan<double> offsets, Span<Vec2> destination)
    {
        var centre = Centre.AsSpan();
        var normal = Normal.AsSpan();

        for (var i = 0; i < centre.Length; i++)
        {
            destination[i] = centre[i] + offsets[i] * normal[i];
        }
    }

    /// <summary>
    /// Returns a copy of the track which stations exactly <paramref name="spacing"/>
    /// metres apart (to width rounding), sampled from a periodic spline through
    /// the original centreline. Widths are interpolated linearly.
    /// </summary>
    public Track Resample(double spacing)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(spacing);

        var spline = new PeriodicSpline(Centre.AsSpan());

        // Dense table of (parameter, arc length) so we can invert s -> t.
        const int subdivisions = 20;
        var denseCount = Count * subdivisions;
        var tDense = new double[denseCount + 1];
        var sDense = new double[denseCount + 1];
        var previous = spline.Evaluate(0);
        
        for (var k = 1; k <= denseCount; k++)
        {
            tDense[k] = spline.Period * k / denseCount;
            var point = spline.Evaluate(tDense[k]);
            sDense[k] = sDense[k - 1] + Vec2.Distance(previous, point);
            previous = point;
        }

        var totalLength = sDense[^1];
        var count = Math.Max(3, (int)Math.Round(totalLength / spacing));

        var centre = ImmutableArray.CreateBuilder<Vec2>(count);
        var right = ImmutableArray.CreateBuilder<double>(count);
        var left = ImmutableArray.CreateBuilder<double>(count);

        for (var j = 0; j < count; j++)
        {
            var s = totalLength * j / count;
            var t = Interpolate(sDense, tDense, s);
            centre.Add(spline.Evaluate(t));

            var (knot, fraction) = Locate(spline, t);
            var nextKnot = (knot + 1) % Count;
            right.Add(WidthRight[knot] + fraction * (WidthRight[nextKnot] - WidthRight[knot]));
            left.Add(WidthLeft[knot] + fraction * (WidthLeft[nextKnot] - WidthLeft[knot]));
        }

        return new Track(Name, centre.MoveToImmutable(), right.MoveToImmutable(), left.MoveToImmutable());
    } 

    private static double Interpolate(double[] xs, double[] ys, double x)
    {
        var i = Array.BinarySearch(xs, x);
        if (i >= 0)
        {
            return ys[i];
        }

        i = ~i;
        if (i == 0)
        {
            return ys[0];
        }

        if (i >= xs.Length)
        {
            return ys[^1];
        }

        var f = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
        return ys[i - 1] + f * (ys[i] - ys[i - 1]);
    }

    private static (int Knot, double Fraction) Locate(PeriodicSpline spline, double t)
    {
        // Linear scan is fine here: resampling runs once per track load.
        for (var i = 0; i < spline.Count; i++)
        {
            var start = spline.KnotAt(i);
            var end = spline.KnotAt(i + 1);

            if (t < end || i == spline.Count - 1)
            {
                return (i, Math.Clamp((t - start) / (end - start), 0.0, 1.0));
            }
        }

        return (spline.Count - 1, 1.0);
    }
}
