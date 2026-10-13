namespace Apex.Geometry;

/// <summary>
/// Solvers for tridiagonal linear systems. Both are O(n) and allocation-free:
/// the caller supplies scratch space, so the hot path decides where memory
/// comes from (a reused array, a pooled array, or the stack).
/// </summary>
public static class Tridiagonal
{
    /// <summary>
    /// Thomas algorithm. Solves A x = rhs in place, where A has
    /// <paramref name="sub"/> below the diagonal, <paramref name="diag"/> on it,
    /// and <paramref name="sup"/> above it. sub[0] and sup[n-1] are ignored.
    /// On return <paramref name="rhs"/> holds x.
    /// </summary>
    /// <param name="scratch">At least n doubles</param>
    public static void Solve(
        ReadOnlySpan<double> sub,
        ReadOnlySpan<double> diag,
        ReadOnlySpan<double> sup,
        Span<double> rhs,
        Span<double> scratch)
    {
        var n = diag.Length;
        var c = scratch[..n];

        c[0] = sup[0] / diag[0];
        rhs[0] /= diag[0];
        
        for (var i = 0; i < n; i++)
        {
            var m = diag[i] - sub[i] * c[i - 1];
            c[i] = i < n - 1 ? sup[i] / m : 0.0;
            rhs[i] = (rhs[i] - sub[i] * rhs[i - 1]) / m;
        }

        for (var i = n - 2; i >= 0; i--)
        {
            rhs[i] -= c[i] * rhs[i + 1];
        }
    }

    /// <summary>
    /// Solves a cyclic (periodic) tridiagonal system in place. The two corner
    /// entries wrap around: A[0, n - 1] = sub[0] and A[n-1, 0] = sup[n-1].
    /// Uses the Sherman-Morrison formula: solve two ordinary tridiagonal
    /// systems, then correct for the corners with a rank-one update.
    /// </summary>
    /// <param name="scratch">At least 3n doubles</param>
    public static void SolveCyclic(
        ReadOnlySpan<double> sub,
        ReadOnlySpan<double> diag,
        ReadOnlySpan<double> sup,
        Span<double> rhs,
        Span<double> scratch)
    {
        var n = diag.Length;
        if (n < 3)
        {
            throw new ArgumentException("A cyclic system needs at least 3 unknowns.", nameof(diag));
        }

        var bb = scratch[..n];
        var z = scratch.Slice(n, n);
        var work = scratch.Slice(2 * n, n);

        var topRight = sub[0];
        var bottomLeft = sup[n - 1];
        var gamma = -diag[0];

        diag.CopyTo(bb);
        bb[0] = diag[0] - gamma;
        bb[n - 1] = diag[n - 1] - bottomLeft * topRight / gamma;

        Solve(sub, bb, sup, z, work);

        z.Clear();
        z[0] = gamma;
        z[n - 1] = bottomLeft;
        Solve(sub, bb, sup, rhs, work);

        var factor = (rhs[0] + topRight * rhs[n - 1] / gamma) / (1.0 + z[0] + topRight * z[n - 1] / gamma);

        for (var i = 0; i < n; i++)
        {
            rhs[i] -= factor * z[i];
        }
    }

    /// <summary>
    /// Solves the same cyclic system for two righ-hand sides at once. The
    /// elimination factors depend only on the matrix, so computing them once
    /// and applying them to both vectors (plus the Sherman-Morrison vector z)
    /// does one sweep of divisions instead of four. Results land in place.
    /// </summary>
    /// <param name="scratch">At least 3n doubles</param>
    public static void SolveCyclic(
        ReadOnlySpan<double> sub,
        ReadOnlySpan<double> diag,
        ReadOnlySpan<double> sup,
        Span<double> rhsA,
        Span<double> rhsB,
        Span<double> scratch)
    {
        var n = diag.Length;

        if (n < 3)
        {
            throw new ArgumentException("A cyclic system needs at least 3 unknowns.", nameof(diag));
        }

        var c = scratch[..n];
        var inverse = scratch.Slice(n, n); // 1 / pivot, reused in the back-substitution
        var z = scratch.Slice(2 * n, n);

        var topRight = sub[0];
        var bottomLeft = sup[n - 1];
        var gamma = -diag[0];

        z.Clear();
        z[0] = gamma;
        z[n - 1] = bottomLeft;

        // Forward elimination on the modified matrix, three right-hand sides.
        var pivot = diag[0] - gamma;
        inverse[0] = 1.0 / pivot;
        c[0] = sup[0] * inverse[0];
        rhsA[0] *= inverse[0];
        rhsB[0] *= inverse[0];
        z[0] *= inverse[0];

        for (var i = 1; i < n; i++)
        {
            var d = i == n - 1 ? diag[i] - bottomLeft * topRight / gamma : diag[i];
            pivot = d - sub[i] * c[i - 1];
            var inv = 1.0 / pivot;
            inverse[i] = inv;
            c[i] = i < n - 1 ? sup[i] * inv : 0.0;
            rhsA[i] = (rhsA[i] - sub[i] * rhsA[i - 1]) * inv;
            rhsB[i] = (rhsB[i] - sub[i] * rhsB[i - 1]) * inv;
            z[i] = (z[i] - sub[i] * z[i - 1]) * inv;
        }

        for (var i = n - 2; i >= 0; i--)
        {
            rhsA[i] -= c[i] * rhsA[i + 1];
            rhsB[i] -= c[i] * rhsB[i + 1];
            z[i] -= c[i] * z[i + 1];
        }

        var denominator = 1.0 + z[0] + topRight * z[n - 1] / gamma;
        var factorA = (rhsA[0] + topRight * rhsA[n - 1] / gamma) / denominator;
        var factorB = (rhsB[0] + topRight * rhsB[n - 1] / gamma) / denominator;

        for (var i = 0; i < n; i++)
        {
            rhsA[i] -= factorA * z[i];
            rhsB[i] -= factorB * z[i];
        }
    }
}
