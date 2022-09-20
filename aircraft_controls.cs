using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

class aircraft_controls
{
    public Vector3 activation_coeff;

    public PolarCrds cr_polar_delta;

    public PolarCrds cr_polar_global;

    public void recalc_delta(PolarCrds ac_polar_global)
    {
        cr_polar_delta = cr_polar_global - ac_polar_global;
    }

    public void recalc_main(aircraft ac)
    {
        recalc_delta(ac.ac_polar_global);
    }

}

