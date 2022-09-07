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

    public Vector3 ConvertToVector3()
    {
        return new Vector3(X, Y, Z);
    }

    public static explicit operator Vector3(Point3 p)
    {
        return new Vector3(p.X, p.Y, p.Z);
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
        get { return azimuth;  }
        set { value %= 360f; } 
    
    }
    public float elevation 
    { 
        get { return elevation;  } 
        set 
        { 
            if (MathF.Abs(value) > 90)
            {
                elevation = 180 - value;
                azimuth = (azimuth - 180) % 360;
            }
        } 
    }

    public PolarCrds(float azimuth, float elevation)
    {
        this.azimuth = azimuth;
        this.elevation = elevation;
    }

    public PolarCrds(Vector3 vec)
    {
        azimuth = MathF.Asin(vec.Y / vec.Z);
        elevation = MathF.Atan(MathF.Sqrt(vec.X * vec.X + vec.Y * vec.Y) / vec.Z);
    }

    public static PolarCrds operator +(PolarCrds p) => p;
    public static PolarCrds operator -(PolarCrds p) => new PolarCrds(-p.azimuth, -p.elevation);

    public static PolarCrds operator +(PolarCrds p1, PolarCrds p2) => new PolarCrds(p1.azimuth + p2.azimuth, p1.elevation + p2.elevation);
    public static PolarCrds operator -(PolarCrds p1, PolarCrds p2)

    {
        return new PolarCrds(p1.azimuth - p2.azimuth, p1.elevation - p2.elevation);
    }

    public Vector3 ConvertToNVec()
    {
        return new Vector3(
            MathF.Sin(azimuth / 180 * MathF.PI) * MathF.Cos(elevation), 
            MathF.Sin((azimuth + 90) / 180 * MathF.PI) * MathF.Cos(elevation), 
            MathF.Sin(elevation / 180 * MathF.PI));
    }
}

