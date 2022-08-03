using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

public class Point3
{
    public float x;
    public float y;
    public float z;

    public Point3( float x, float y, float z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void RelfectionAboutPoint( Point3 p )
    {
    
    
    }

    public void RotationAboutPoint(Point3 p)
    {


    }

}




public class physical_constants
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

    
}