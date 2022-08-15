using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

public class engine_class
{
    public Vector3 thrust_nvec;
    public Point3 thrust_app_point;

    public float thrust_max;
    public float thrust_min;
    public float fuel_cons_specific;

    public float thrust;
    public float fuel_cons;

    public engine_class(float thrust_max, float thrust_min, float fuel_cons_specific)
    {
        this.thrust_max = thrust_max;
        this.thrust_min = thrust_min;
        this.fuel_cons_specific = fuel_cons_specific;
    }

    public void recalc_fuel_cons(environment env)
    {
        fuel_cons = thrust * fuel_cons_specific;
    }
}

