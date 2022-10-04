using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;



public interface actns_coeff_functions
{
    public void coeffs_get(actions_class actns, PolarCrds delta) { }
}

public class actns_coeff_stock : actns_coeff_functions
{
    public void coeffs_get(actions_class actns, PolarCrds delta)
    {
        actns.values["roll"] = actns.activation_coeffs["roll"] * delta.eq_roll / 180;
        if (actns.values["roll"] > 1) { actns.values["roll"] = 0; }

        actns.values["pitch"] = actns.activation_coeffs["pitch"] * delta.elevation / 180;
        if (actns.values["pitch"] > 1) { actns.values["pitch"] = 0; }

        actns.values["yaw"] = actns.activation_coeffs["yaw"] * delta.azimuth / 180;
        if (actns.values["yaw"] > 1) { actns.values["yaw"] = 0; }

    }
}