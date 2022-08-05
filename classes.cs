using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

public class Point3
{
    public float X;
    public float Y;
    public float Z;

    public Point3(float X, float Y, float Z)
    {
        this.X = X;
        this.Y = Y;
        this.Z = Z;
    }

}

public class Phys
{
    public const float g = 9.81f; // Gravitational acceleration
    public const float R = 8.314f; // Universal gas constant
    public const float Lb = -0.0065f; // Temperature lapse rate (K/m) in ISA
    public const float M = 0.029f; // Molar mass of dry air
    public const float Mv = 0.018f; // Molar mass of water vapor
}

public class environment
{
    public float air_pressure_asl = 103250;
    public float air_temperature_asl = 288;

    public float air_pressure;
    public float air_density;
    public float air_temperature;

    public environment(float p_asl, float t_asl)
    {
        air_pressure_asl = p_asl;
        air_temperature_asl = t_asl;
    }

    public void recalc(float alt)
    {
        air_temperature = air_temperature_asl - (Phys.Lb * alt);
        air_pressure = air_pressure_asl * MathF.Pow(((air_temperature_asl + alt * Phys.Lb) / air_temperature_asl), (-Phys.g * Phys.M / (Phys.R * Phys.Lb)));
        air_density = air_pressure * Phys.M / (Phys.R * air_temperature);

        //saturation_vapor_pressure = 6.108f * MathF.Pow(10, ((7.5f * air_temperature) / (air_temperature - 237.3f)));
        //water_vapor_pressure = saturation_vapor_pressure * air_humidity;
    }


}