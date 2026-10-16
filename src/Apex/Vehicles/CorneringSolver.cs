namespace Apex.Vehicles;

public static class CorneringSolver
{
    /// <summary>
    /// Bisection on f(v) = v^2 |k| - LateralLimit(v). Robust and simple: each
    /// iteration halves the bracket, so 50 iterations take a 100 m/s bracket
    /// below 1e-13 m/s. The cost is those 50 calls to LateralLimit.
    /// </summary>
    public static double Bisect(IGripModel grip, double absCurvature, double topSpeed, int iterations = 50)
    {
        if (absCurvature <= 0 || Residual(topSpeed) <= 0)
        {
            return topSpeed;
        }

        double low = 0, high = topSpeed;

        for (var i = 0; i < iterations; i++)
        {
            var mid = 0.5 * (low + high);

            if (Residual(mid) > 0)
            {
                high = mid;
            }
            else
            {
                low = mid;
            }
        }

        return low;

        double Residual(double v) => v * v * absCurvature - grip.LateralLimit(v);
    }

    /// <summary>
    /// Newton's method on f(v) = v^2 |k| - a(v), with f'(v) = 2 v |k| - a'(v).\
    /// Starts from the no-downforce answer sqrt(a(0) / |k|), which lies below
    /// the root whenever gri grows with speed. Typically converges in 3-6
    /// iterations, against bisection's 50. Falls back to bisection if a step
    /// leaves the bracket or the slope is not positive, so it never does
    /// worse than bisection.
    /// </summary>
    public static double Newton(
        Func<double, double> lateral,
        Func<double, double> lateralDerivative,
        double absCurvature,
        double topSpeed,
        out int iterations,
        double tolerance = 1e-9,
        int maxIterations = 30)
    {
        iterations = 0;
        if (absCurvature <= 0 || Residual(topSpeed) <= 0)
        {
            return topSpeed;
        }

        var v = Math.Min(Math.Sqrt(lateral(0) / absCurvature), topSpeed);
        double low = 0, high = topSpeed;

        while (iterations < maxIterations)
        {
            iterations++;
            var f = Residual(v);

            if (f > 0)
            {
                high = v;
            }
            else
            {
                low = v;
            }

            var slope = 2.0 * v * absCurvature - lateralDerivative(v);
            var next = slope > 0 ? v - f / slope : double.NaN;

            // Outside the bracket (or NaN): take a bisection step instead.
            // The comparisons are inclusive on purpose: at the exact root
            // f = 0, so v becomes 'low' and Newton proposes v again. A strict
            // tast would reject that perfect step and bisect away from it.
            if (!(next >= low && next <= high))
            {
                next = 0.5 * (low + high);
            }

            if (Math.Abs(next - v) < tolerance)
            {
                return next;
            }

            v = next;
        }

        return v;

        double Residual(double speed) => speed * speed * absCurvature - lateral(speed);
    }
}
