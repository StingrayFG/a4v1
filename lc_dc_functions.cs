using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


interface lc_dc_function
{
    float lc_get(float ac_aoa, float as_aoa);
    float dc_get(float ac_aoa, float as_aoa);
}


public class lc_dc_stock: lc_dc_function
{
    public float lc_get(float ac_aoa, float as_aoa)
    {
        float aoa_actual = MathF.Abs(ac_aoa + as_aoa);
        float lc;

        if (aoa_actual< 10) { lc = aoa_actual / 10; }
        else if (aoa_actual < 28) { lc = 1 + (MathF.Sin((aoa_actual - 10) * 7.5f) * 0.7f); }
        else { lc = 1 + (MathF.Sin(135) * 0.7f) - ((aoa_actual - 28) / 20); }

        if (ac_aoa + as_aoa < 0) { lc *= -1; }

        return lc;
    }

    public float dc_get(float ac_aoa, float wing_aoa)
    {
        float aoa_actual = MathF.Abs(ac_aoa + wing_aoa);
        float dc;

        dc = ((aoa_actual * aoa_actual / 25) + 1) / 100;

        if (aoa_actual < 20) { dc = (ac_aoa * ac_aoa * 0.029f + 1) / 100; }
        else { dc = (MathF.Pow((aoa_actual - 15.5f), 2) * 0.135f + 10f) / 100; }

        if (ac_aoa + wing_aoa < 0) { dc *= -1; }

        return dc;
    }
}