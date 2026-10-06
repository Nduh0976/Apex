namespace Apex.Geometry;

/// <summary>
/// A fitted periodic spline you can evaluate anywhere along its length.
/// Used for loading and resampling tracks, where convenience beats speed.
/// The hot path uses <see cref="SplineMath"/> with a reused workspace instead.
/// </summary>
public sealed class PeriodicSpline
{
    private readonly Vec2[] _points;
    private readonly double[] _knots; // cumulative chord length, length n + 1
    private readonly double[] _h;
    private readonly double[] _mx;
    private readonly double[] _my;

    public PeriodicSpline(ReadOnlySpan<Vec2> points)
    {
        var ws = new SplineWorkspace(points.Length);
        SplineMath.Fit(points, ws);

        _points = points.ToArray();
        _h = ws.H;
        _mx = ws.Mx;
        _my = ws.My;

        _knots = new double[points.Length + 1];

        for (var i = 0; i < points.Length; i++)
        {
            _knots[i + 1] = _knots[i] + _h[i];
        }
    }

    public int Count => _points.Length;

    /// <summary>Total chord length of the closed loop; t runs from 0 to Period.</summary>
    public double Period => _knots[^1];

    /// <summary>Parameter value at knot i.</summary>
    public double KnotAt(int i) => _knots[i];

    public Vec2 Evaluate(double t)
    {
        t %= Period;

        if (t < 0)
        {
            t += Period;
        }

        // Binary search for the segment containing t.
        var i = Array.BinarySearch(_knots, t);

        if (i < 0)
        {
            i = ~i - 1;
        }

        if (i >= _points.Length)
        {
            i = _points.Length - 1;
        }

        var next = i == _points.Length - 1 ? 0 : i + 1;
        var h = _h[i];
        var a = (_knots[i + 1] - t) / h;
        var b = 1.0 - a;
        var w = h * h / 6.0;

        var x = a * _points[i].X + b * _points[next].X + ((a * a * a - a) * _mx[i] + (b * b * b - b) * _mx[next]) * w;
        var y = a * _points[i].Y + b * _points[next].Y + ((a * a * a - a) * _my[i] + (b * b * b - b) * _my[next]) * w;

        return new Vec2(x, y);
    }
}
