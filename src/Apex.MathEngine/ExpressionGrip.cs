using Apex.Vehicles;
using MathEngine.Ast;
using MathEngine.Calculus;
using MathEngine.Compilation;
using MathEngine.Evaluation;
using MathEngine.Rewriting;

namespace Apex.MathEngine;

/// <summary>
/// Adapter from MathEngine's expression engine to Apex's grip model. The
/// user types the lateral grip limit as a formula in speed; we parse it,
/// differentiate it symbolically, and compile both to delegates, The core
/// library never sees MathEngine: it only knows <see cref="FunctionGrip"/>,
/// which takes two Func&lt;double, double&gt; values.
/// </summary>
public static class ExpressionGrip
{
    // One argument buffer per thread. The compiled delegates take a double[]
    // allocating one per call would cost an allocation per station per lap,
    // and a single shared array would be a data racew the moment the
    // optimiser's Parallel.For calls two delegates on two threads at once.
    [ThreadStatic]
    private static double[]? t_arguments;

    /// <param name="lateralFormula">Lateral grip limit in m/s^2, e.g. "mu * (g + 0.5 * rho * cla * v^2 / m)".</param>
    /// <param name="constants">Values for every name in the formula except the speed variable.</param>
    /// <param name="longitudinalRatio">Longitudinal limit as a fraction of the lateral limit.</param>
    /// <param name="speedVariable">The name the formula uses for speed.</param>
    public static FunctionGrip FromFormula(
        string lateralFormula,
        IReadOnlyDictionary<string, double> constants,
        double longitudinalRatio,
        string speedVariable = "v")
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(lateralFormula);
        ArgumentNullException.ThrowIfNull(constants);

        var functions = FunctionRegistry.CreateDefault();
        var lateral = Simplifier.Simplify(Parser.Parse(lateralFormula));
        var derivative = Simplifier.Simplify(Differentiator.Differentiate(lateral, speedVariable, functions));

        return new FunctionGrip(
            Bind(lateral, speedVariable, functions, constants),
            Bind(derivative, speedVariable, functions, constants),
            longitudinalRatio);
    }

    private static Func<double, double> Bind(
        Expr expr,
        string speedVariable,
        FunctionRegistry functions,
        IReadOnlyDictionary<string, double> constants)
    {
        // Constants are folded into the compiled code; only speed stays a parameter.
        var compiled = ExpressionCompiler.Compile(expr, [speedVariable], functions, constants);

        return speed =>
        {
            var arguments = t_arguments ??= new double[1];
            arguments[0] = speed;

            return compiled(arguments);
        };
    }
}
