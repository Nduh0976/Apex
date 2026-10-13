using System.Numerics;
using System.Runtime.InteropServices;

namespace Apex.Optimisation;

/// <summary>
/// Element-wise loops of the projected-gradient solver, written with
/// <see cref="Vector{T}"/> so the JIT emits SIMD instructions: on a machine
/// with AVX2, four doubles per instruction. Each nethid has a scalar tail
/// for the elements left over when the length is not a multiple of
/// Vector&lt;double&gt;.Count.
/// </summary>
internal static class VectorKernels
{
    /// <summary>y = x + momentum * (x - previous).</summary>
    public static void Extrapolate(ReadOnlySpan<double> x, ReadOnlySpan<double> previous, double momentum, Span<double> y)
    {
        var vx = MemoryMarshal.Cast<double, Vector<double>>(x);
        var vp = MemoryMarshal.Cast<double, Vector<double>>(previous);
        var vy = MemoryMarshal.Cast<double, Vector<double>>(y);
        var m = new Vector<double>(momentum);

        for (var i = 0; i < vx.Length; i++)
        {
            vy[i] = vx[i] + m * (vx[i] - vp[i]);
        }

        for (var i = vx.Length * Vector<double>.Count; i < x.Length; i++)
        {
            y[i] = x[i] + momentum * (x[i] - previous[i]);
        }
    }

    /// <summary>x = clamp(y - step * gradient, lowe, upper): the projected gradient step.</summary>
    public static void StepAndProject(
        ReadOnlySpan<double> y,
        ReadOnlySpan<double> gradient,
        double step,
        ReadOnlySpan<double> lower,
        ReadOnlySpan<double> upper,
        Span<double> x)
    {
        var vy = MemoryMarshal.Cast<double, Vector<double>>(y);
        var vg = MemoryMarshal.Cast<double, Vector<double>>(gradient);
        var vl = MemoryMarshal.Cast<double, Vector<double>>(lower);
        var vu = MemoryMarshal.Cast<double, Vector<double>>(upper);
        var vx = MemoryMarshal.Cast<double, Vector<double>>(x);
        var s = new Vector<double>(step);

        for (var i = 0; i < vy.Length; i++)
        {
            vx[i] = Vector.Min(Vector.Max(vy[i] - s * vg[i], vl[i]), vu[i]);
        }

        for (var i = vy.Length * Vector<double>.Count; i < y.Length; i++)
        {
            x[i] = Math.Clamp(y[i] - step * gradient[i], lower[i], upper[i]);
        }
    }

    /// <summary>
    /// Restart and convergence statistics in one pass:
    ///     uphill       = sum (y - x)(x - previous)    (positive: momentum is hurting)
    ///     stationarity =  max |y - x|                 (the size of the projected step)
    /// </summary>
    public static (double Uphill, double Stationarity) Diagnostics(
        ReadOnlySpan<double> y,
        ReadOnlySpan<double> x,
        ReadOnlySpan<double> previous)
    {
        var vy = MemoryMarshal.Cast<double, Vector<double>>(y);
        var vx = MemoryMarshal.Cast<double, Vector<double>>(x);
        var vp = MemoryMarshal.Cast<double, Vector<double>>(previous);
        var dot = Vector<double>.Zero;
        var max = Vector<double>.Zero;

        for (var i = 0; i < vy.Length; i++)
        {
            var step = vy[i] - vx[i];
            dot += step * (vx[i] - vp[i]);
            max = Vector.Max(max, Vector.Abs(step));
        }

        var uphill = Vector.Dot(dot, Vector<double>.One);
        var stationarity = 0.0;

        for (var lane = 0; lane < Vector<double>.Count; lane++)
        {
            stationarity = Math.Max(stationarity, max[lane]);
        }

        for (var i = vy.Length * Vector<double>.Count; i < y.Length; i++)
        {
            uphill += (y[i] - x[i]) * (x[i] - previous[i]);
            stationarity = Math.Max(stationarity, Math.Abs(y[i] - x[i]));
        }

        return (uphill, stationarity);
    }
}
