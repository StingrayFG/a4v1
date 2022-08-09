using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

public class ad_surface_section
{
    public coefficients_functions root_functions;
    public coefficients_functions tip_functions;

    public float aoa_base;
    public float aoa_no_sideslip;
    public float aoa_true;

    public bool aoa_is_defined = false;

    public float chord_len;
    public float length_fraction;

    public Point3 ad_center;
    public Point3 chord_center;

    public float lift_coeff;
    public float drag_coeff;

    public ad_surface_section(float aoa_base, float chord_len, float length_fraction, Point3 ad_center, Point3 chord_center)
    {
        this.aoa_base = aoa_base;
        this.chord_len = chord_len;
        this.length_fraction = length_fraction;
        this.ad_center = ad_center;
        this.chord_center = chord_center;
        aoa_is_defined = true;
    }

    public ad_surface_section(float chord_len, float length_fraction)
    {
        this.chord_len = chord_len;
        this.length_fraction = length_fraction;
        aoa_is_defined = false;
    }

    public void lc_get()
    {
        lift_coeff = (root_functions.lc_get(aoa_true) * length_fraction) + (tip_functions.lc_get(aoa_true) * (1 - length_fraction));
    }

    public void dc_get()
    {
        drag_coeff = (root_functions.dc_get(aoa_true) * length_fraction) + (tip_functions.dc_get(aoa_true) * (1 - length_fraction));
    }



}

public class control_surface
{
    public float affected_area;

    public float chord_fraction_avg;
    public float chord_len_avg;
    public float length_fraction;
    public float length_offset;

    public float angle;
    public float min_angle;
    public float max_angle;

    public float lift_force;
    public float drag_force;

    public int[] sections_nums = new int[2];

    public Point3 forces_app_point;

    public Vector3 lift_force_nvec;
    public Vector3 drag_force_nvec;
    public Vector3 add_force_sum_nvec;
}

public class aerodynamic_surface
{
    public List<ad_surface_section> sections_main = new List<ad_surface_section>();
    public List<ad_surface_section> sections_all = new List<ad_surface_section>();

    public List<control_surface> control_surfaces = new List<control_surface>();

    public coefficients_functions root_functions;
    public coefficients_functions tip_functions;

    public float sections_step;

    public float area;
    public float slope;
    public float length;
    public float length_offset;
    
    //public Vector3 ad_center_line_nvec; // used for determination of on which side of aircraft surface is

    public float lift_force;
    public float drag_force;

    public Point3 forces_app_point;

    public Vector3 lift_force_nvec; 
    public Vector3 drag_force_nvec;
    public Vector3 force_sum_nvec;

    public void generate_sections_main_aoa()
    {
        for (int i = 1; i < (sections_main.Count - 1); i++)
        {
            if (sections_main[i].aoa_is_defined == false)
            {
                for (int j = i; j < (sections_main.Count - 1); j++)
                {
                    if (sections_main[j].aoa_is_defined == true)
                    {
                        sections_main[j].aoa_base = sections_main[i - 1].aoa_base + ((sections_main[i].aoa_base - sections_main[i].aoa_base) * ((float)j + 1));
                        sections_main[j].aoa_is_defined = true;
                    }
                }
                if (sections_main[i].aoa_is_defined == false) { throw new InvalidOperationException("one of sections_main sections's aoa_base cant be calculated, not enough sections with determined aoa_base"); }
            }
        }
    }

    public void generate_sections_all()
    {
        for (int i = 0; i < (sections_main.Count - 1); i++)
        {
            int divisions = (int)((sections_main[i + 1].length_fraction * length - sections_main[i + 1].length_fraction * length) % sections_step);

            sections_all.Add(sections_main[i]);

            for (int j = 0; i < divisions; j++)
            {
                sections_all.Add(
                new ad_surface_section(
                    (sections_main[i].aoa_base + ((sections_main[i + 1].aoa_base - sections_main[i].aoa_base) * ((float)j + 1) / divisions)),
                    (sections_main[i].chord_len + ((sections_main[i + 1].chord_len - sections_main[i].chord_len) * ((float)j + 1) / divisions)),
                    (sections_main[i].length_fraction + ((sections_main[i + 1].length_fraction - sections_main[i].length_fraction) * ((float)j + 1) / divisions)),
                    new Point3(
                        (sections_main[i].ad_center.X + (sections_main[i + 1].ad_center.X - sections_main[i].ad_center.X) * ((float)j + 1) / divisions),
                        (sections_main[i].ad_center.Y + (sections_main[i + 1].ad_center.Y - sections_main[i].ad_center.Y) * ((float)j + 1) / divisions),
                        (sections_main[i].ad_center.Z + (sections_main[i + 1].ad_center.Z - sections_main[i].ad_center.Z) * ((float)j + 1) / divisions)),
                    new Point3(
                        (sections_main[i].chord_center.X + (sections_main[i + 1].chord_center.X - sections_main[i].chord_center.X) * ((float)j + 1) / divisions),
                        (sections_main[i].chord_center.Y + (sections_main[i + 1].chord_center.Y - sections_main[i].chord_center.Y) * ((float)j + 1) / divisions),
                        (sections_main[i].chord_center.Z + (sections_main[i + 1].chord_center.Z - sections_main[i].chord_center.Z) * ((float)j + 1) / divisions))
                    ));
            }
        }
    }

    //public void recalc_sections_main_aoa(float aircraft_aoa, float course_deviation, float roll, float speed)
    //{
    //    if (sections_main[sections_main.Count - 1].ad_center.Y < 0) { course_deviation *= -1; }

    //    foreach (ad_surface_section section in sections_main)
    //    {
    //        section.aoa_no_sideslip = section.aoa_base + aircraft_aoa + (MathF.Atan(((length_offset + length * section.length_fraction) * roll / 180 * MathF.PI) / speed) * 180 / MathF.PI);
    //        section.aoa_true = section.aoa_no_sideslip * MathF.Cos(course_deviation / 180 * MathF.PI) + slope * MathF.Sin(course_deviation / 180 * MathF.PI);
    //        Console.WriteLine(section.aoa_true);
    //    }
    //}

    public void recalc_sections_all_coefficients(float aircraft_aoa, float course_deviation, float roll, float speed, int[] sections_nums)
    {
        for (int i = sections_nums[0]; i < sections_nums[1]; i++)
        {
            int divisions = (int)((sections_main[i + 1].length_fraction * length - sections_main[i + 1].length_fraction * length) % sections_step);

            int sections_iterator = 0;

            for (int j = 0; i < divisions; j++)
            {
                sections_all[sections_iterator].aoa_no_sideslip = sections_all[sections_iterator].aoa_base + aircraft_aoa + (MathF.Atan(((length_offset + length * sections_all[sections_iterator].length_fraction) * roll / 180 * MathF.PI) / speed) * 180 / MathF.PI);
                sections_all[sections_iterator].aoa_true = sections_all[sections_iterator].aoa_no_sideslip * MathF.Cos(course_deviation / 180 * MathF.PI) + slope * MathF.Sin(course_deviation / 180 * MathF.PI);
                sections_all[sections_iterator].lc_get();
                sections_all[sections_iterator].dc_get();

                sections_iterator++;
            }
        }
    }

    public void recalc_surface_coefficients(float aircraft_aoa, float course_deviation, float roll, float speed)
    {
        foreach (control_surface surface in control_surfaces)
        {
            
        }    
    }




    public void recalc_forces(Vector3 velocity_nvec, float air_density, float speed)
    {
        

        drag_force_nvec.X = -velocity_nvec.X;
        drag_force_nvec.Y = -velocity_nvec.Y;
        drag_force_nvec.Z = -velocity_nvec.Z;
    }



}

class Program
{
    static void Main(string[] args)
    {
        aerodynamic_surface ad = new aerodynamic_surface();
        ad.slope = 5;
        ad.length = 4;
        ad.length_offset = 0.5f;

        ad.root_functions = new lc_dc_stock();
        ad.tip_functions = new lc_dc_stock();

        ad.sections_main.Add(new ad_surface_section(5, 4.56f, 0, new Point3(6, 0.5f, 0.2f), new Point3(4, 0.5f, 0.2f)));

        ad.sections_main.Add(new ad_surface_section(10, 0.4f, 0.5f, new Point3(5, 2f, 0.5f), new Point3(4, 2f, 0.5f)));

        ad.sections_main.Add(new ad_surface_section(15, 0.4f, 1, new Point3(4, 4.5f, 0.5f), new Point3(3.5f, 4.5f, 0.5f)));

        
        //ad.sections_main[1].functions = new lc_dc_stock();
        //ad.sections_main[2].functions = new lc_dc_stock();

        //ad.generate_sections_all();

        //ad.recalc_sections(0, 10, 0, 100);
        //ad.recalc_forces(new Vector3(1, 1, 1), 0.001f, 100);

        ////Console.WriteLine(ad.recalc_sections(5, 15));

    }
}


public class aircraft
{


    // aircraft parameters

    public float aicraft_aoa;
    public float course_deviation; // left to right

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

    // functions

    // X - roll, parallel to aircraft central axis, directed forward; Y - pitch, horizontal, perpendicular to aircraft central axis, directed left; Z - rudder, vertical, directed up; 

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




