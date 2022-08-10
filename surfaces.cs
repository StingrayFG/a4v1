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



public class surface
{
    public float area;

    public float lift_coeff;
    public float drag_coeff;

    public float lift_force;
    public float drag_force;

    public float aoa_avg;

    public Point3 forces_app_point;

    public Vector3 lift_force_nvec;
    public Vector3 drag_force_nvec;
    public Vector3 forces_sum_nvec;
}

public class control_surface : surface // 'forces_sum_nvec' is a vector of additional force, not a total force applied to aerodynamic surface, 'area' of control surface is area affected by coefficients changing.
{
    public float chord_fraction_avg;
    public float chord_len_avg;
    public float length_fraction;
    public float root_offset;

    public float angle;
    public float min_angle;
    public float max_angle;

    public int[] sections_nums = new int[2];
}



public class aerodynamic_surface : surface
{
    public List<ad_surface_section> sections_main = new List<ad_surface_section>();
    public List<ad_surface_section> sections_all = new List<ad_surface_section>();

    public List<control_surface> control_surfaces = new List<control_surface>();

    public coefficients_functions root_functions;
    public coefficients_functions tip_functions;

    public Vector3 one_axis_nvec;

    public float sections_step;
    public float slope;
    public float length;
    public float root_offset;



    public void calc_sections_main_aoa() // calculates 'aoa_base' for sections of 'sections_main' which 'aoa_base' isnt determined
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

    public void calc_sections_all() // generates additional sections so distance between them will be less than 'sections_step'
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

    public void calc_lift_force_nvec()
    {
        lift_force_nvec = new Vector3(
            -MathF.Asin(aoa_avg / 180 * MathF.PI), 
            -MathF.Asin(slope / 180 * MathF.PI), 
            MathF.Sqrt(-MathF.Pow(MathF.Asin(aoa_avg / 180 * MathF.PI), 2) - MathF.Pow(MathF.Asin(slope / 180 * MathF.PI), 2) + 1));
    }



    public void recalc_sections_all_coefficients(aircraft ac) // recalculates coefficients for each section in 'sections_all'
    {
        for (int i = 0; i < sections_all.Count; i++)
        {
            int divisions = (int)((sections_main[i + 1].length_fraction * length - sections_main[i + 1].length_fraction * length) % sections_step);

            int sections_iterator = 0;

            for (int j = 0; i < divisions; j++)
            {
                sections_all[sections_iterator].aoa_no_sideslip = sections_all[sections_iterator].aoa_base + ac.aircraft_aoa + (MathF.Atan(((root_offset + length * sections_all[sections_iterator].length_fraction) * ac.roll / 180 * MathF.PI) / ac.speed) * 180 / MathF.PI);
                sections_all[sections_iterator].aoa_true = sections_all[sections_iterator].aoa_no_sideslip * MathF.Cos(ac.course_deviation / 180 * MathF.PI) + slope * MathF.Sin(ac.course_deviation / 180 * MathF.PI);
                sections_all[sections_iterator].lc_get();
                sections_all[sections_iterator].dc_get();

                sections_iterator++;
            }
        }
    }

    public void recalc_surface_coefficients(int[] sections_nums, surface surf) // calculates average coefficients for given sections of aerodynamic or control surface
    {
        float chords_sum = 0;

        float lc_res = 0;
        float dc_res = 0;
        float aoa_res = 0;

        int sections_iterator = 0;

        for (int i = sections_nums[0]; i < sections_nums[1]; i++)
        {
            int divisions = (int)((sections_main[i + 1].length_fraction * length - sections_main[i + 1].length_fraction * length) % sections_step);

            for (int j = 0; i < divisions; j++)
            {
                chords_sum += sections_all[sections_iterator].chord_len;
                sections_iterator++;
            }
        }

        for (int k = 0; k < sections_iterator; k++)
        {
            float coeff = sections_all[k].chord_len / chords_sum;
            lc_res += sections_all[k].lift_coeff * coeff;
            dc_res += sections_all[k].drag_coeff * coeff;
            aoa_res += sections_all[k].aoa_base * coeff;
        }

        surf.lift_coeff = lc_res;
        surf.drag_coeff = dc_res;
        surf.aoa_avg = aoa_res;
    }

    public void recalc_surface_forces(aircraft ac, environment env, surface surf) // calculates forces coefficients for given surface
    {
        surf.lift_force = surf.lift_coeff * env.air_density * ac.speed * ac.speed * surf.area / 2;
    }



    public void calc_initial()
    {
        calc_sections_main_aoa();
        calc_sections_all();
        calc_lift_force_nvec();
    }

    public void recalc_main(aircraft ac, environment env)
    {
        recalc_sections_all_coefficients(ac);

        recalc_surface_coefficients(new int[2] { 0, (sections_all.Count - 1) }, this);
        recalc_surface_forces(ac, env, this);
        drag_force_nvec = Vector3.Negate(ac.velocity_local_nvec);

        foreach (control_surface c_surf in control_surfaces)
        {
            recalc_surface_coefficients(c_surf.sections_nums, c_surf);
            recalc_surface_forces(ac, env, c_surf);
            c_surf.drag_force_nvec = Vector3.Negate(ac.velocity_local_nvec);
        }



        //drag_force_nvec.X = -velocity_nvec.X;
        //drag_force_nvec.Y = -velocity_nvec.Y;
        //drag_force_nvec.Z = -velocity_nvec.Z;
    }

    //public void recalc_sections_main_aoa(float aircraft_aoa, float course_deviation, float roll, float speed)
    //{
    //    if (sections_main[sections_main.Count - 1].ad_center.Y < 0) { course_deviation *= -1; }

    //    foreach (ad_surface_section section in sections_main)
    //    {
    //        section.aoa_no_sideslip = section.aoa_base + aircraft_aoa + (MathF.Atan(((root_offset + length * section.length_fraction) * roll / 180 * MathF.PI) / speed) * 180 / MathF.PI);
    //        section.aoa_true = section.aoa_no_sideslip * MathF.Cos(course_deviation / 180 * MathF.PI) + slope * MathF.Sin(course_deviation / 180 * MathF.PI);
    //        Console.WriteLine(section.aoa_true);
    //    }
    //}

}
