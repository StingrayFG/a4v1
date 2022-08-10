using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;



public class aircraft
{
    public float aircraft_aoa;
    public float course_deviation; // left to right
    public float speed;
    public float roll;
    public float altitude;
    public float main_axis_aoa;

    public Vector3 main_axis_local_nvec;
    public Vector3 velocity_local_nvec;
    public Vector3 rotational_speeds;

    public Vector3 main_axis_global_nvec;
    public Vector3 velocity_global_nvec;
    public Vector3 rotation_global;
    public Vector3 position_global;

    //public float zfw;
    //public float auw;
    //public float ffw;
    //public float fuel_weight;
    //public float fuel_weight_max;

    //public float fuel_consumption;
    //public float fuel_consumption_min;
    //public float fuel_consumption_max;

    //public float throttle;
    //public float thrust;
    //public float thrust_max;

    //public float[] thrust_point = new float[3];
    //public float[] thrust_nvec = new float[3];

    //public float[] center_of_mass_zfw = new float[3];
    //public float[] center_of_mass_auw = new float[3];
    //public float[] center_of_mass_ffw = new float[3];

    public List<aerodynamic_surface> surfaces = new List<aerodynamic_surface>();

    // X - roll, parallel to aircraft central axis, directed forward; Y - pitch, horizontal, perpendicular to aircraft central axis, directed left; Z - rudder, vertical, directed up; 

    public void recalc_surfaces(environment env)
    {
        foreach(aerodynamic_surface surf in surfaces)
        {
            surf.recalc_main(this, env);
        }
    }


    //public void initial_precalc()
    //{
    //    ffw = zfw + fuel_weight_max;
    //}


    //public void recalc_aircraft_parameters()
    //{
    //    auw = zfw + fuel_weight;
    //    for (int i = 0; i < 3; i++)
    //    {
    //        center_of_mass_auw[0] = center_of_mass_zfw[0] + ((center_of_mass_ffw[0] - center_of_mass_zfw[0]) * (fuel_weight / fuel_weight_max));
    //    }

    //    thrust = throttle * thrust_max;
    //    fuel_consumption = fuel_consumption_min + ((fuel_consumption_max - fuel_consumption_min) * throttle);
    //    fuel_weight -= fuel_consumption * phys_time_step;
    //}

    //public void recalc_forces()
    //{

    //}

    //public void recalc_main(float[] vec1, float[] vec2)
    //{
    //    main_axis_aoa = (MathF.Atan(vec1[2] / vec1[0]) + MathF.Atan(vec2[2] / vec2[0]) * 180 / MathF.PI);

    //    recalc_aircraft_parameters();

    //}

}









