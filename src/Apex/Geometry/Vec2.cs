namespace Apex.Geometry;

/// <summary>
/// An immutable 2D vector in metres. A readonly struct gives value
/// equality for free and lives inline in arrays, so a Vec2[] is one block
/// of memory rather than an array of pointers to heap objects.
/// </summary>
public record struct Vec2(double X, double Y)
{
    public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);
    public static Vec2 operator -(Vec2 a, Vec2 b) => new(a.X - b.X, a.Y - b.Y);
    public static Vec2 operator *(double s, Vec2 v) => new(s * v.X, s * v.Y);
    public static Vec2 operator *(Vec2 v, double s) => new(s * v.X, s * v.Y);

    public double Length => Math.Sqrt(X * X + Y * Y);

    public double LengthSquared => X * X + Y * Y;

    /// <summary> The vector rotated 90 degrees counter-clockwise. </summary>
    public Vec2 PerpLeft => new(-Y, X);

    public Vec2 Normalized()
    {
        var length = Length;

        return length > 0
            ? new Vec2(X / length, Y / length)
            : throw new InvalidOperationException("Cannot normalize a zero-length vector.");
    }

    public static double Dot(Vec2 a, Vec2 b) => a.X * b.X + a.Y * b.Y;

    /// <summary> The z-component of the 3D cross product. Positive when b is anticlockwise of a. </summary>
    public static double Cross(Vec2 a, Vec2 b) => a.X * b.Y - a.Y * b.X;

    public static double Distance(Vec2 a, Vec2 b) => (b - a).Length;
}
