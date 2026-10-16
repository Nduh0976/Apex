namespace Apex.Vehicles;

/// <summary>
/// Grip described by arbitrary functions of speed, with the derivative of the
/// lateral limit supplied alongside it. Having the derivative lets the
/// cornering speed be found by Newton's method instead of bisection.
/// Any source of functions works: lambdas in a test, or compiled expressions
/// typed by a user.
/// </summary>
public sealed class FunctionGrip(
    Func<double, double> lateral,
    Func<double, double> lateralDerivative,
    double longitudinalRatio) : IGripModel
{
    public double LateralLimit(double speed) => lateral(speed);

    public double LongitudinalLimit(double speed) => longitudinalRatio * lateral(speed);

    public double CorneringSpeed(double absCurvature, double topSpeed) =>
        CorneringSolver.Newton(lateral, lateralDerivative, absCurvature, topSpeed, out _);
}
