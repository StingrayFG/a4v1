using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

public interface actns_coeff_functions
{
    float roll_coeff_get(float delta);

    void calc(PolarCrds delta);

}

public class actions_class
{
    public Dictionary<string, float> values = new Dictionary<string, float>()
    {
        {"none", 0f },
        {"roll", 0f },
        {"pitch", 0f },
        {"yaw", 0f },
        {"flaps", 0f },
        {"slats", 0f },
        {"airbrake", 0f }
    };

    public Dictionary<string, float> true_angles = new Dictionary<string, float>()
    {
        {"none", 0f },
        {"roll", 0f },
        {"pitch", 0f },
        {"yaw", 0f },
        {"flaps", 0f },
        {"slats", 0f },
        {"airbrake", 0f }
    };

    public actns_coeff_functions coeff_functions;

    public void SetMainActions(PolarCrds delta)
    {
        coeff_functions.calc(delta);
    }
}



class aircraft_controls
{
    actions_class actions_main = new actions_class(); 

    public PolarCrds activation_coeff;

    public PolarCrds cr_polar_delta;

    public PolarCrds cr_global_polar;

    public void recalc_delta(PolarCrds ac_global_polar)
    {
        cr_polar_delta = cr_global_polar - ac_global_polar;
    }

    public void set_actions()
    {
        actions_main.SetMainActions(cr_polar_delta);
    }

    public void recalc_main(aircraft ac)
    {
        recalc_delta(ac.ac_axis_global_polar);
    }

}

