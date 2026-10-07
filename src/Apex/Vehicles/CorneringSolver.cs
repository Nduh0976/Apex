namespace Apex.Vehicles
{
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
    }
}
