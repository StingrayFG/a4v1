using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

public class control_surface_functions
{
    public float decrease_speed;
    public float decrease_k;

    public virtual float get_max_decrease(float speed) { return 1; }
}

public class c_surf_func_linear: control_surface_functions
{
    public c_surf_func_linear(float decrease_speed, float decrease_k)
    {
        this.decrease_speed = decrease_speed;
        this.decrease_k = decrease_k;
    }

    public override float get_max_decrease(float speed)
    {
        if (speed < decrease_speed)
        {
            return 1;
        }
        else
        {
            return MathF.Max(0, 1 - ((speed - decrease_speed) * decrease_k));
        }
    }
}