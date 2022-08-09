using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


public interface coefficients_functions
{
    float lc_get(float aoa_true);
    float dc_get(float aoa_true);
}


public class lc_dc_stock: coefficients_functions
{
    public float lc_get(float aoa_true)
    {
        float lc;

        if (MathF.Abs(aoa_true) < 10) { lc = aoa_true / 10; }
        else if (MathF.Abs(aoa_true) < 28) { lc = 1 + (MathF.Sin(((aoa_true - 10) * 7.5f) / 180 * MathF.PI) * 0.7f); }
        else { lc = 1 + (MathF.Sin(135 / 180 * MathF.PI) * 0.7f) - ((aoa_true - 28) / 20); }

        if (aoa_true < 0) { lc *= -1; }

        return lc;
    }

    public float dc_get(float aoa_true)
    {
        float dc;

        if (MathF.Abs(aoa_true) < 20) { dc = (aoa_true * aoa_true * 0.029f + 1) / 100; }
        else { dc = (MathF.Pow((aoa_true - 15.5f), 2) * 0.135f + 10f) / 100; }

        if (aoa_true < 0) { dc *= -1; }

        return dc;
    }
}