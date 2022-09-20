using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

public struct Point3
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

    public Point3(Vector3 vec)
    {
        X = vec.X;
        Y = vec.Y;
        Z = vec.Z;
    }

    public static implicit operator Vector3(Point3 p)
    {
        return new Vector3(p.X, p.Y, p.Z);
    }

    public static explicit operator Point3(Vector3 vec)
    {
        return new Point3(vec);
    }

    public static Point3 operator +(Point3 p) => p;
    public static Point3 operator -(Point3 p) => new Point3(-p.X, -p.Y, -p.Z);

    public static Point3 operator +(Point3 p1, Point3 p2) => new Point3(p1.X + p2.X, p1.Y + p2.Y, p1.Z + p2.Z);
    public static Point3 operator -(Point3 p1, Point3 p2) => new Point3(p1.X - p2.X, p1.Y - p2.Y, p1.Z - p2.Z);


    public static Point3 operator *(Point3 p, float v) => new Point3(p.X * v, p.Y * v, p.Z * v);
    public static Point3 operator /(Point3 p, float v) => new Point3(p.X / v, p.Y / v, p.Z / v);


    public static Point3 operator +(Point3 p, Vector3 v) => new Point3(p.X + v.X, p.Y + v.Y, p.Z + v.Z);
    public static Point3 operator -(Point3 p, Vector3 v) => new Point3(p.X - v.X, p.Y - v.Y, p.Z - v.Z);
    public static Point3 operator *(Point3 p, Vector3 v) => new Point3(p.X * v.X, p.Y * v.Y, p.Z * v.Z);
    public static Point3 operator /(Point3 p, Vector3 v) => new Point3(p.X / v.X, p.Y / v.Y, p.Z / v.Z);

    public static Point3 operator +(Vector3 v, Point3 p) => new Point3(p.X + v.X, p.Y + v.Y, p.Z + v.Z);
    public static Point3 operator -(Vector3 v, Point3 p) => new Point3(p.X - v.X, p.Y - v.Y, p.Z - v.Z);
    public static Point3 operator *(Vector3 v, Point3 p) => new Point3(p.X * v.X, p.Y * v.Y, p.Z * v.Z);
    public static Point3 operator /(Vector3 v, Point3 p) => new Point3(v.X / p.X, v.Y / p.Y, v.Z / p.Z);

}

public struct PolarCrds
{
    public float azimuth
    { 
        get => azimuth;  
        set => azimuth = value % 360f; 
    }

    public float elevation 
    { 
        get 
        { 
            return elevation;  
        } 
        set 
        { 
            if (MathF.Abs(value) > 90)
            {
                elevation = 180 - value;
                azimuth = (azimuth - 180) % 360;
            }
        } 
    }

    public float eq_roll
    {
        get => eq_roll; 
        set => eq_roll = value % 360f; 
    }

    public PolarCrds(float azimuth, float elevation)
    {
        this.azimuth = azimuth;
        this.elevation = elevation;
    }

    public PolarCrds(float azimuth, float elevation, float eq_roll)
    {
        this.azimuth = azimuth;
        this.elevation = elevation;
        this.eq_roll = eq_roll;
    }

    public PolarCrds(Vector3 nvec)
    {
        azimuth = MathF.Asin(nvec.Y / nvec.Z);
        elevation = MathF.Atan(MathF.Sqrt(nvec.X * nvec.X + nvec.Y * nvec.Y) / nvec.Z);
    }

    public static PolarCrds operator +(PolarCrds p) => p;
    public static PolarCrds operator -(PolarCrds p) => new PolarCrds(-p.azimuth, -p.elevation, -p.eq_roll);

    public static PolarCrds operator +(PolarCrds p1, PolarCrds p2)
    {
        return new PolarCrds(p1.azimuth + p2.azimuth, p1.elevation + p2.elevation, p1.eq_roll + p2.eq_roll);
    }

    public static PolarCrds operator -(PolarCrds p1, PolarCrds p2)
    {
        return new PolarCrds(p1.azimuth - p2.azimuth, p1.elevation - p2.elevation, p1.eq_roll - p2.eq_roll);
    }

    public Vector3 ConvertToNVec()
    {
        return new Vector3(
            MathF.Sin(azimuth / 180 * MathF.PI) * MathF.Cos(elevation), 
            MathF.Sin((azimuth + 90) / 180 * MathF.PI) * MathF.Cos(elevation), 
            MathF.Sin(elevation / 180 * MathF.PI));
    }
}

