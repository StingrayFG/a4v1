using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;



public class control_surface
{
    public float affected_area;
    public float avg_chord_fraction;
    public float lift_coeff;
    public float lift_force;
    public float drag_coeff;
    public float drag_force;
    public float[] elevator_lf_point = new float[3];
    public float[] elevator_lf_nvec = new float[3];
    public float angle;
    public float min_angle;
    public float max_angle;
}

public class aerodynamic_surface
{
    public float area;

    public float lift_coeff;
    public float lift_force;
    public float drag_coeff;
    public float drag_force;

    public Point3 lift_force_point;

    public Vector3 lift_force_nvec;
    public Vector3 drag_force_nvec;

    public List<control_surface> control_surfaces = new List<control_surface>();

    public void

}


public class aircraft
{

    // physical parameters and constants 



    public float phys_time_step = 0.01f;

    // aircraft parameters

    public float[] main_axis_local_nvec = new float[3];
    public float[] velocity_local_nvec = new float[3];    
    public float[] rotational_speeds = new float[3];

    public float[] main_axis_global_nvec = new float[3];  
    public float[] velocity_global_nvec = new float[3];
    public float[] rotation_global = new float[3];
    public float[] position_global = new float[3];

    public float velocity;
    public float altitude;
    public float main_axis_aoa;

    public float zfw;
    public float auw;
    public float ffw;
    public float fuel_weight;
    public float fuel_weight_max;

    public float fuel_consumption;
    public float fuel_consumption_min;
    public float fuel_consumption_max;

    public float throttle;
    public float thrust;
    public float thrust_max;

    public float[] thrust_point = new float[3];
    public float[] thrust_nvec = new float[3];

    public float[] center_of_mass_zfw = new float[3];
    public float[] center_of_mass_auw = new float[3];
    public float[] center_of_mass_ffw = new float[3];

    // wings and tail

    public float hor_stab_area;
    public float hor_stab_aoa;
    public float hor_stab_slope;
    public float hor_stab_lc;
    public float hor_stab_lf;
    public float hor_stab_df;
    public float hor_stab_dc;
    public float[] hor_stab_lf_point = new float[3];
    public float[] hor_stab_lf_nvec = new float[3];

    public float elevator_affected_area;
    public float elevator_avg_chord_fraction;
    public float elevator_lf;
    public float elevator_df;
    public float[] elevator_lf_point = new float[3];
    public float[] elevator_lf_nvec = new float[3];
    public float elevator_angle;
    public float[] elevator_max_angles = new float[2];


    public float wing_area;
    public float wing_aoa;
    public float wing_slope;
    public float wing_lc;
    public float wing_lf;
    public float wing_df;
    public float wing_dc;
    public float[] wing_lf_point = new float[3];
    public float[] wing_lf_nvec = new float[3];

    public float aileron_affected_area;
    public float aileron_avg_chord_fraction;
    public float aileron_lf;
    public float aileron_df;
    public float[] aileron_lf_point = new float[3];
    public float[] aileron_lf_nvec = new float[3];
    public float aileron_angle;
    public float[] aileron_max_angles = new float[2];


    public float ver_stab_area;
    public float ver_stab_aoa;
    public float ver_stab_slope;
    public float ver_stab_lc;
    public float ver_stab_lf;
    public float ver_stab_dc;
    public float ver_stab_df;
    public float[] ver_stab_lf_point = new float[3];
    public float[] ver_stab_lf_nvec = new float[3];

    public float rudder_affected_area;
    public float rudder_avg_chord_fraction;
    public float rudder_lf;
    public float rudder_df;
    public float[] rudder_lf_point = new float[3];
    public float[] rudder_lf_nvec = new float[3];
    public float rudder_angle;
    public float[] rudder_max_angles = new float[2];

    // functions

    // X - roll, parallel to aircraft central axis, directed forward; Y - pitch, horizontal, perpendicular to aircraft central axis, directed left; Z - rudder, vertical, directed up; 

    public float[,] rotation_matrix_calc(float[] u, float phi) // u - normalized axis vector, phi - angle
    {
        float[,] res = new float[3, 3];

        phi *= -1;

        if (phi < 0) { phi = 360 + phi; }

        res[0, 0] = MathF.Cos(phi) + u[0] * u[0] * (1 - MathF.Cos(phi));
        res[0, 1] = u[0] * u[1] * (1 - MathF.Cos(phi)) - u[2] * MathF.Sin(phi);
        res[0, 2] = u[0] * u[2] * (1 - MathF.Cos(phi)) + u[1] * MathF.Sin(phi);

        res[1, 0] = u[1] * u[0] * (1 - MathF.Cos(phi)) + u[2] * MathF.Sin(phi);
        res[1, 1] = MathF.Cos(phi) + u[1] * u[1] * (1 - MathF.Cos(phi));
        res[1, 2] = u[1] * u[2] * (1 - MathF.Cos(phi)) - u[0] * MathF.Sin(phi);

        res[2, 0] = u[2] * u[0] * (1 - MathF.Cos(phi)) - u[1] * MathF.Sin(phi);
        res[2, 1] = u[2] * u[1] * (1 - MathF.Cos(phi)) + u[0] * MathF.Sin(phi);
        res[2, 2] = MathF.Cos(phi) + u[2] * u[2] * (1 - MathF.Cos(phi));

        return res;
    }

    public float[,] rotation_matrix_apply(float[,] r, float[,] dvec) // r - rotation matrix, nr_dvec, returns rotated copy of dvec vector
    {
        float[,] res = dvec;
        float[] svec = new float[3];

        for (int i = 0; i < 3; i++) { svec[i] = dvec[1, i] - dvec[0, i]; }

        for (int i = 0; i < 3; i++)
        {
            float sum = 0;
            for (int j = 0; j < 3; j++) { sum += r[i, j] * svec[j]; }
            res[1, i] = res[0, i] + sum;
        }

        return res;
    }

    public void initial_precalc()
    {
        ffw = zfw + fuel_weight_max;
    }

    public void recalc_air_parameters()
    {
        air_temperature = air_temperature_asl - (Lb * altitude);  
        air_pressure = air_pressure_asl * MathF.Pow(((air_temperature_asl + altitude * Lb) / air_temperature_asl), (-g * M / (R * Lb)));
        air_density = air_pressure * M / (R * air_temperature);

        //saturation_vapor_pressure = 6.108f * MathF.Pow(10, ((7.5f * air_temperature) / (air_temperature - 237.3f)));
        //water_vapor_pressure = saturation_vapor_pressure * air_humidity;
    }

    public void recalc_surfaces()
    {
        hor_stab_lc = get_lift_coefficient(main_axis_aoa, hor_stab_aoa);
        wing_lc = get_lift_coefficient(main_axis_aoa, wing_aoa);
        ver_stab_lc = get_lift_coefficient(main_axis_aoa, ver_stab_aoa);

        hor_stab_dc = get_lift_coefficient(main_axis_aoa, hor_stab_aoa);
        wing_dc = get_lift_coefficient(main_axis_aoa, wing_aoa);
        ver_stab_dc = get_lift_coefficient(main_axis_aoa, ver_stab_aoa);

        hor_stab_lf = hor_stab_lc * air_density * velocity * velocity * (hor_stab_area - elevator_affected_area) / 2;
        elevator_lf = hor_stab_lc * MathF.Pow(elevator_avg_chord_fraction, 0.5f) * MathF.Sin(elevator_angle * MathF.PI / 180 ) * air_density * velocity * velocity * elevator_affected_area / 2;

        wing_lf = wing_lc * air_density * velocity * velocity * (hor_stab_area - aileron_affected_area) / 2;
        aileron_lf = wing_lc * MathF.Pow(aileron_avg_chord_fraction, 0.5f) * MathF.Sin(aileron_angle * MathF.PI / 180) * air_density * velocity * velocity * aileron_affected_area / 2;

        ver_stab_lf = ver_stab_lc * air_density * velocity * velocity * (ver_stab_area - rudder_affected_area) / 2;
        rudder_lf = ver_stab_lc * MathF.Pow(elevator_avg_chord_fraction, 0.5f) * MathF.Sin(rudder_angle * MathF.PI / 180) * air_density * velocity * velocity * rudder_affected_area / 2;
    }

    public void recalc_aircraft_parameters()
    {
        auw = zfw + fuel_weight;
        for(int i = 0; i < 3; i++)
        {
            center_of_mass_auw[0] = center_of_mass_zfw[0] + ((center_of_mass_ffw[0] - center_of_mass_zfw[0]) * (fuel_weight / fuel_weight_max));
        }

        thrust = throttle * thrust_max;
        fuel_consumption = fuel_consumption_min + ((fuel_consumption_max - fuel_consumption_min) * throttle);
        fuel_weight -= fuel_consumption * phys_time_step;
    }

    public void recalc_forces()
    {
        
    }

    public void recalc_main(float[] vec1, float[] vec2)
    {
        main_axis_aoa = (MathF.Atan(vec1[2] / vec1[0]) + MathF.Atan(vec2[2] / vec2[0]) * 180 / MathF.PI);

        recalc_air_parameters();
        recalc_aircraft_parameters();
        
    }

}


class Program
{
    static void Main(string[] args)
    {
        

    }
}


