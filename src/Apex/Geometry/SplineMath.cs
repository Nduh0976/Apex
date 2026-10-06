namespace Apex.Geometry;

/// <summary>
/// Periodic cubic spline through a closed loop of points, parameterised by
/// cummulative chord length. Each coordinate is a separate 1D spline x(t),
/// y(t) sharing the same knots, so both use the same tridiagonal matrix
/// </summary>
public static class SplineMath
{
    /// <summary>
    /// Fits the spline: fills ws.H with chord lengths and ws.Mx / ws.My with
    /// the second derivatives at each knot. C2 continuity at every knot gives,
    /// for each i (indices wrap):
    ///     h[i-1] M[i-1] + 2 (h[i-1] + h[i]) M[i] + h[i] M[i+1]
    ///      = 6 ((y[i+1] - y[i]) / h[i] - (y[i] - y[i-1]) / h[i-1])
    /// </summary>
    public static void Fit(ReadOnlySpan<Vec2> points, SplineWorkspace ws)
    {
        var n = points.Length;

        if (n != ws.Count)
        {
            throw new ArgumentException($"Workspace is sized for {ws.Count} points, got {n}.", nameof(points));
        }

        var h = ws.H;
        for (var i = 0; i < n; i++)
        {
            h[i] = Vec2.Distance(points[i], points[i == n - 1 ? 0 : i + 1]);

            if (h[i] <= 0)
            {
                throw new ArgumentException($"Points {i} and {(i + 1) % n} coincide.", nameof(points));
            }
        }

        for (var i = 0; i < n; i++)
        {
            var prev = i == 0 ? n - 1 : i - 1;
            var next = i == n - 1 ? 0 : i + 1;

            ws.Sub[i] = h[prev];
            ws.Diag[i] = 2.0 * (h[prev] + h[i]);
            ws.Sup[i] = h[i];

            var p = points[prev];
            var c = points[i];
            var q = points[next];

            ws.Mx[i] = 6.0 * ((q.X - c.X) / h[i] - (c.X - p.X) / h[prev]); 
            ws.My[i] = 6.0 * ((q.Y - c.Y) / h[i] - (c.Y - p.Y) / h[prev]); 
        }

        Tridiagonal.SolveCyclic(ws.Sub, ws.Diag, ws.Sup, ws.Mx, ws.Solver);
        Tridiagonal.SolveCyclic(ws.Sub, ws.Diag, ws.Sup, ws.My, ws.Solver);
    }

    /// <summary>
    /// Signed curvature at every knot: positive turning left, in 1/m.
    /// Allocation-free given a workspace.
    /// </summary>
    public static void CurvatureAtKnots(ReadOnlySpan<Vec2> points, Span<double> curvature, SplineWorkspace ws)
    {
        Fit(points, ws);

        var n = points.Length;

        for (var i = 0; i < n; i++)
        {
            var next = i == n - 1 ? 0 : i + 1;
            var h = ws.H[i];

            // First derivative at the left end of the segment i.
            var dx = (points[next].X - points[i].X) / h - h * (2.0 * ws.Mx[i] + ws.Mx[next]) / 6.0;
            var dy = (points[next].Y - points[i].Y) / h - h * (2.0 * ws.My[i] + ws.My[next]) / 6.0;

            var speedSquared = dx * dx + dy * dy;
            curvature[i] = (dx * ws.My[i] - dy * ws.Mx[i]) / (speedSquared * Math.Sqrt(speedSquared));
        }
    }
}
 