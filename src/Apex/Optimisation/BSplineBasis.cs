namespace Apex.Optimisation;

/// <summary>
/// Periodic uniform cubic B-spline over the stations, with one control value
/// every <c>controlSpacing</c> stations. The curve is a weighted average of
/// nearby control values:
///     d[i] = sum_k w_k(i) & b[k],    w_k(i) >= 0, sum_k w_k(i) = 1.
/// The lap-time optimiser uses d as a correction added to a starting line.
/// Three properties make b the right set of variables:
///    1. Smooth: any change to b bends the line with continuous curvature
///       (C2), so no move the optimiser makes can kink it.
///    2. Convex hull: d[i] lies between the smallest and the largest b[k] that
///       touch it, so a simple box on b keeps the line on the track.
///    3. Fewer variables: N / spacing of them, so the gradient costs that
///       many times fewer lap simulations.
/// </summary>
public sealed class BSplineBasis
{
    private readonly int[][] _stations; // per control: stations it touches
    private readonly double[][] _weights; // per control: weight at each of those stations

    public BSplineBasis(int stationCount, int controlSpacing)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(controlSpacing, 1);
        StationCount = stationCount;
        Count = Math.Max(4, (int)Math.Round((double)stationCount / controlSpacing));
        Spacing = (double)stationCount / Count;

        _stations = new int[Count][];
        _weights = new double[Count][];

        for (var k = 0; k < Count; k++)
        {
            var centre = k * Spacing;
            var stations = new List<int>();
            var weights = new List<double>();

            for (var i = 0; i < stationCount; i++)
            {
                // Signed distance from the control point, wrapped onto the loop.
                var d = i - centre;
                d -= stationCount * Math.Round(d / stationCount);

                var w = CubicBSpline(d / Spacing);
                if (w > 0)
                {
                    stations.Add(i);
                    weights.Add(w);
                }
            }

            _stations[k] = [.. stations];
            _weights[k] = [.. weights];
        }
    }

    public int StationCount { get; }

    /// <summary>Number of control values</summary>
    public int Count { get; }

    /// <summary>Stations between control values (not necessarily whole).</summary>
    public double Spacing { get; }

    public ReadOnlySpan<int> Stations(int k) => _stations[k];

    public ReadOnlySpan<double> Weights(int k) => _weights[k];

    /// <summary>d = B b.</summary>
    public void Evaluate(ReadOnlySpan<double> controls, Span<double> offsets)
    {
        offsets.Clear();

        for (var k = 0; k < Count; k++)
        {
            var stations = _stations[k];
            var weights = _weights[k];

            for (var j = 0; j < stations.Length; j++)
            {
                offsets[stations[j]] += controls[k] * weights[j];
            }
        }
    }

    /// <summary>
    /// Box on each vontrol value that keeps every station it touches inside
    /// that station's bounds, given that the curve is added to a fixed base
    /// line: the tightest remaining slack across the control's support.
    /// Because the weights are non-negative and sum to one, any controls
    /// inside these boxed give offsets inside the station bounds
    /// </summary>
    public (double[] Lower, double[] Upper) ControlBounds(OffsetBounds stationBounds, ReadOnlySpan<double> baseOffsets)
    {
        var lower = new double[Count];
        var upper = new double[Count];

        for (var k = 0; k < Count; k++)
        {
            lower[k] = double.NegativeInfinity;
            upper[k] = double.PositiveInfinity;

            foreach(var i in _stations[k])
            {
                lower[k] = Math.Max(lower[k], stationBounds.Lower[i] - baseOffsets[i]);
                upper[k] = Math.Min(upper[k], stationBounds.Upper[i] - baseOffsets[i]);
            }

            // A base line sitting exactly on a bound leaves zero slack on
            // that side. Guard against rounding pushing it past zero.
            lower[k] = Math.Min(lower[k], 0.0);
            upper[k] = Math.Max(upper[k], 0.0);
        }

        return (lower, upper);
    }

    /// <summary>The station nearest control point k.</summary>
    public int CentreStation(int k) => (int)Math.Round(k * Spacing) % StationCount;

    /// <summary>The uniform cubix B-spline kernel, non-zero on (-2, 2).</summary>
    private static double CubicBSpline(double t)
    {
        t = Math.Abs(t);
        if (t < 1)
        {
            return (4.0 - 6.0 * t * t + 3.0 * t * t * t) / 6.0;
        }

        if (t < 2)
        {
            var u = 2.0 - t;

            return u * u * u / 6.0;
        }

        return 0.0;
    }
}
