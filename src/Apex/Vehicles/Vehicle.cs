namespace Apex.Vehicles;

/// <summary>
/// A point-mass car. Units: kg, W, m^2, kg/m^3, m/s
/// </summary>
public sealed record Vehicle(
    double Mass,
    double Power,
    double DragArea,
    double AirDensity,
    double TopSpeed,
    IGripModel Grip)
{
    /// <summary>Deceleration from aerodynamic drag at this speed, m/s^2.</summary>

    public double DragDeceleration(double speed) => 0.5 * AirDensity * DragArea * speed * speed / Mass;

    /// <summary>
    /// Roughly a road-legal track car with no meaningful downforce:
    /// 1,300 kg, 300 KW, CdA 0.7 m^2, 1.3 g lateral and 1.2 g longitudinal grip.
    /// </summary>
    public static Vehicle TrackCar() => new(
        Mass: 1300,
        Power: 300_000,
        DragArea: 0.7,
        AirDensity: 1.2,
        TopSpeed: 90,
        Grip: new ConstantGrip(lateral: 1.3 * 9.81, longitudinal: 1.2 * 9.81));
}
