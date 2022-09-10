using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

class aircraft_controls
{
    Vector3 activation_coeff;

    Vector3 c_ring_delta;

    PolarCrds c_ring_global;


    public void get_delta(aircraft ac)
    {
        c_ring_delta.Z = c_ring_global.azimuth - ac.ac_axis_global.azimuth;
        c_ring_delta.Y = c_ring_global.elevation - ac.ac_axis_global.elevation;
        c_ring_delta.X = MathF.Atan(c_ring_delta.Z / c_ring_delta.Y) * 180 / MathF.PI - ac.rotation.X;
    }
}

