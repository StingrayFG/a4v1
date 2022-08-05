using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

public class wing_cross_section
{
    public wing_functions functions;

    public float aoa_base;
    public float aoa_no_sideslip;
    public float aoa_true;

    public float chord_len;
    public float span_half_fraction;

    public Point3 ad_center;

    public float lift_coeff;
    public float drag_coeff;
}

public class control_surface
{
    public float affected_area;

    public float chord_fraction_avg;
    public float chord_len_avg;
    public float span_half_fraction;

    public float angle;
    public float min_angle;
    public float max_angle;

    public Point3 forces_app_point;

    public Vector3 lift_force_nvec;
    public Vector3 drag_force_nvec;
    public Vector3 forces_sum_nvec;
}

public class aerodynamic_surface
{
    public wing_cross_section root = new wing_cross_section();
    public wing_cross_section tip = new wing_cross_section();

    public float area;
    public float span_half;

    public float ad_center_line_slope;
    public Vector3 ad_center_line_nvec; // used for determination of on which side of aircraft surface is
    

    public float lift_force;
    public float drag_force;

    public Point3 forces_app_point;

    public Vector3 lift_force_nvec; 
    public Vector3 drag_force_nvec;
    public Vector3 forces_sum_nvec;

    public List<control_surface> control_surfaces = new List<control_surface>();

    public void recalc_cross_sections(float aircraft_aoa, float course_deviation, float roll, float speed)
    {
        if(ad_center_line_nvec.Y < 0) { course_deviation *= -1; } // if aerodynamic surface is on the left half of the aircraft, course deviation is inverted

        root.aoa_no_sideslip = root.aoa_base + aircraft_aoa + (MathF.Atan((span_half * root.span_half_fraction * roll * 180 / MathF.PI) / speed) * 180 / MathF.PI);
        root.aoa_true = root.aoa_no_sideslip * MathF.Cos(course_deviation / 180 * MathF.PI) + ad_center_line_slope * MathF.Sin(course_deviation / 180 * MathF.PI);

        tip.aoa_no_sideslip = tip.aoa_base + aircraft_aoa + (MathF.Atan((span_half * tip.span_half_fraction * roll * 180 / MathF.PI) / speed) * 180 / MathF.PI);
        tip.aoa_true = tip.aoa_no_sideslip * MathF.Cos(course_deviation / 180 * MathF.PI) + ad_center_line_slope * MathF.Sin(course_deviation / 180 * MathF.PI);

        root.lift_coeff = root.functions.lc_get(root.aoa_true);
        root.drag_coeff = root.functions.dc_get(root.aoa_true);

        tip.lift_coeff = tip.functions.lc_get(tip.aoa_true);
        tip.drag_coeff = tip.functions.dc_get(root.aoa_true);
    }

    public void recalc_forces(Vector3 velocity_nvec, float air_density, float speed)
    {
        lift_force = ((root.lift_coeff + tip.lift_coeff) / 2) * air_density * speed * speed * area / 2;
        drag_force = ((root.drag_coeff + tip.drag_coeff) / 2) * air_density * speed * speed * area / 2;

        float forces_coeff = (tip.chord_len * tip.lift_coeff / (root.chord_len * root.lift_coeff + tip.chord_len * tip.lift_coeff));

        forces_app_point.X = root.ad_center.X + (root.ad_center.X + tip.ad_center.X) * forces_coeff;
        forces_app_point.Y = root.ad_center.Y + (root.ad_center.Y + tip.ad_center.Y) * forces_coeff;
        forces_app_point.Z = root.ad_center.Z + (root.ad_center.Z + tip.ad_center.Z) * forces_coeff;

        drag_force_nvec.X = -velocity_nvec.X;
        drag_force_nvec.Y = -velocity_nvec.Y;
        drag_force_nvec.Z = -velocity_nvec.Z;


    }



}

class Program
{
    static void Main(string[] args)
    {
        //aerodynamic_surface ad = new aerodynamic_surface();
        //ad.ad_center_line_slope = 5;
        //ad.root = new wing_cross_section();
        //ad.tip = new wing_cross_section();
        //ad.root.aoa_base = 5;
        //ad.tip.aoa_base = 5;

        ////Console.WriteLine(ad.recalc_cross_sections(5, 15));

    }
}


//public class aircraft
//{


//    // aircraft parameters

//    public float wingspan;

//    public float[] main_axis_local_nvec = new float[3];
//    public float[] velocity_local_nvec = new float[3];    
//    public float[] rotational_speeds = new float[3];

//    public float[] main_axis_global_nvec = new float[3];  
//    public float[] velocity_global_nvec = new float[3];
//    public float[] rotation_global = new float[3];
//    public float[] position_global = new float[3];

//    public float velocity;
//    public float altitude;
//    public float main_axis_aoa;

//    public float zfw;
//    public float auw;
//    public float ffw;
//    public float fuel_weight;
//    public float fuel_weight_max;

//    public float fuel_consumption;
//    public float fuel_consumption_min;
//    public float fuel_consumption_max;

//    public float throttle;
//    public float thrust;
//    public float thrust_max;

//    public float[] thrust_point = new float[3];
//    public float[] thrust_nvec = new float[3];

//    public float[] center_of_mass_zfw = new float[3];
//    public float[] center_of_mass_auw = new float[3];
//    public float[] center_of_mass_ffw = new float[3];

//    // wings and tail

//    public float hor_stab_area;
//    public float hor_stab_aoa;
//    public float hor_stab_slope;
//    public float hor_stab_lc;
//    public float hor_stab_lf;
//    public float hor_stab_df;
//    public float hor_stab_dc;
//    public float[] hor_stab_lf_point = new float[3];
//    public float[] hor_stab_lf_nvec = new float[3];

//    public float elevator_affected_area;
//    public float elevator_avg_chord_fraction;
//    public float elevator_lf;
//    public float elevator_df;
//    public float[] elevator_lf_point = new float[3];
//    public float[] elevator_lf_nvec = new float[3];
//    public float elevator_angle;
//    public float[] elevator_max_angles = new float[2];

//    // functions

//    // X - roll, parallel to aircraft central axis, directed forward; Y - pitch, horizontal, perpendicular to aircraft central axis, directed left; Z - rudder, vertical, directed up; 

//    public void initial_precalc()
//    {
//        ffw = zfw + fuel_weight_max;
//    }


//    public void recalc_aircraft_parameters()
//    {
//        auw = zfw + fuel_weight;
//        for(int i = 0; i < 3; i++)
//        {
//            center_of_mass_auw[0] = center_of_mass_zfw[0] + ((center_of_mass_ffw[0] - center_of_mass_zfw[0]) * (fuel_weight / fuel_weight_max));
//        }

//        thrust = throttle * thrust_max;
//        fuel_consumption = fuel_consumption_min + ((fuel_consumption_max - fuel_consumption_min) * throttle);
//        fuel_weight -= fuel_consumption * phys_time_step;
//    }

//    public void recalc_forces()
//    {

//    }

//    public void recalc_main(float[] vec1, float[] vec2)
//    {
//        main_axis_aoa = (MathF.Atan(vec1[2] / vec1[0]) + MathF.Atan(vec2[2] / vec2[0]) * 180 / MathF.PI);

//        recalc_aircraft_parameters();

//    }

//}




