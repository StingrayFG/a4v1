using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;



public class ad_surface_section
{
    public surf_coeff_functions root_functions;
    public surf_coeff_functions tip_functions;

    public float aoa_base;
    public float aoa_rotated;
    public float aoa_no_sideslip;
    public float aoa_true;

    public bool aoa_is_defined = false;

    public float chord_len;
    public float length_fraction;

    public Vector3 ad_center;
    public Vector3 chord_center;

    public float lift_coeff;
    public float drag_coeff;

    public ad_surface_section(float aoa_base, float chord_len, float length_fraction, Vector3 ad_center, Vector3 chord_center)
    {
        this.aoa_base = aoa_base;
        aoa_rotated = aoa_base;

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

    public void lc_get_for_extension(float coeff)
    {

    }

    public void dc_get()
    {
        drag_coeff = (root_functions.dc_get(aoa_true) * length_fraction) + (tip_functions.dc_get(aoa_true) * (1 - length_fraction));
    }
}



public class surface
{

    public ad_surface_section avg_section;

    public float area;

    public float root_offset;

    public float lift_force;
    public float drag_force;

    public Vector3 forces_app_point;
    public Vector3 chord_center_point;

    public Vector3 lift_force_nvec;
    public Vector3 drag_force_nvec;
    public Vector3 forces_sum_nvec;

    public virtual void recalc_coefficients() 
    {
        avg_section.lc_get();
        avg_section.dc_get();
    }

    public void recalc_surface_forces(aircraft ac, environment env) // calculates forces coefficients for given surface
    {
        lift_force = avg_section.lift_coeff * env.air_density * ac.speed * ac.speed * area / 2;
        drag_force = avg_section.drag_coeff * env.air_density * ac.speed * ac.speed * area / 2;
    }
}



public interface moving_object
{
    public string action_key { get => action_key; set => action_key = value; }

    public virtual void set_position(aircraft ac, environment env, ref actions_class actns) { }
}

public interface rotation_object: moving_object
{

    public virtual float angle { get => angle; set => angle = value; }
    public virtual float min_angle { get => angle; set => angle = value; }
    public virtual float max_angle { get => angle; set => angle = value; }
    public virtual float rotation_speed { get => angle; set => angle = value; }

    new public void set_position(aircraft ac, environment env, ref actions_class actns) 
    {
        if (MathF.Abs(actns.values[action_key] * min_angle - angle) > rotation_speed * physics.Ts)
        {
            if (actns.values[action_key] > 0)
            {
                if (actns.values[action_key] * max_angle > angle)
                {
                    angle += rotation_speed * physics.Ts;
                }
                else if (actns.values[action_key] * max_angle < angle)
                {
                    angle -= rotation_speed * physics.Ts;
                }
            }
            else if (actns.values[action_key] < 0)
            {
                if (actns.values[action_key] * min_angle > angle)
                {
                    angle += rotation_speed * physics.Ts;
                }
                else if (actns.values[action_key] * min_angle < angle)
                {
                    angle -= rotation_speed * physics.Ts;
                }
            }

            if (MathF.Abs(actns.values[action_key] * min_angle - angle) < rotation_speed * physics.Ts)
            {
                angle = actns.values[action_key] * min_angle;
            }
        }
    }
}

public interface extension_object: moving_object
{

    public virtual float extension { get => extension; set => extension  = value; }
    public virtual float min_extension { get => min_extension; set => min_extension = value; }
    public virtual float max_extension { get => max_extension; set => max_extension = value; }
    public virtual float extension_speed { get => extension_speed; set => extension_speed = value; }

    new public void set_position(aircraft ac, environment env, ref actions_class actns) 
    {
        if (MathF.Abs(actns.values[action_key] - extension) > extension_speed * physics.Ts)
        {
            if (actns.values[action_key] > extension)
            {
                extension += extension_speed * physics.Ts;
            }
            else if (actns.values[action_key] < extension)
            {
                extension -= extension_speed * physics.Ts;
            }

            if (MathF.Abs(actns.values[action_key] - extension) < extension_speed * physics.Ts)
            {
                extension = actns.values[action_key] * max_extension;
            }
        }
    }
}

public class control_surface : surface, moving_object // 'forces_sum_nvec' is a vector of additional force, not a total force applied to aerodynamic surface, 'area' of control surface is area affected by coefficients changing.
{
    public string action_key { get; set; }

    public control_surface_functions c_surf_func;

    public float length_of_surf;
    
    public float forces_app_point_offset;

    public int[] sections_nums = new int[2];

    public virtual void recalc_forces_app_point() { }

    public void set_position(aircraft ac, ref actions_class actns) { }
}

public class plain_control_surface: control_surface, rotation_object
{
    public virtual float angle { get; set; }
    public virtual float min_angle { get; set; }
    public virtual float max_angle { get; set; }
    public virtual float rotation_speed { get; set; }

    new public void set_position(aircraft ac, ref actions_class actns)
    {
        if (MathF.Abs(actns.values[action_key] * min_angle - angle) > rotation_speed * physics.Ts)
        {
            if (actns.values[action_key] > 0)
            {
                if (actns.values[action_key] * max_angle > angle)
                {
                    angle += rotation_speed * physics.Ts;
                }
                else if (actns.values[action_key] * max_angle < angle)
                {
                    angle -= rotation_speed * physics.Ts;
                }
            }
            else if (actns.values[action_key] < 0)
            {
                if (actns.values[action_key] * min_angle > angle)
                {
                    angle += rotation_speed * physics.Ts;
                }
                else if (actns.values[action_key] * min_angle < angle)
                {
                    angle -= rotation_speed * physics.Ts;
                }
            }

            if (MathF.Abs(actns.values[action_key] * min_angle - angle) < rotation_speed * physics.Ts)
            {
                angle = actns.values[action_key] * min_angle;
            }
        }

        avg_section.aoa_rotated = avg_section.aoa_base + angle;
    }

    public override void recalc_coefficients()
    {
        avg_section.lift_coeff = MathF.Sqrt(length_of_surf / avg_section.chord_len) * avg_section.lift_coeff * MathF.Sin(angle / 180 * MathF.PI);
        // drag coefficient calculation here...
    }
}

public class split_control_surface: control_surface, rotation_object
{

    public float angle { get; set; }
    public float min_angle { get; set; }
    public float max_angle { get; set; }
    public float rotation_speed { get; set; }


    public override void recalc_coefficients()
    {
        avg_section.lift_coeff = MathF.Sqrt(length_of_surf / avg_section.chord_len) * avg_section.lift_coeff * MathF.Sin(angle / 180 * MathF.PI);
        // drag coefficient calculation here...
    }

}

public class slat_rot_control_surface: control_surface, rotation_object
{

    public float angle { get; set; }
    public float min_angle { get; set; }
    public float max_angle { get; set; }
    public float rotation_speed { get; set; }

    public override void recalc_coefficients()
    {
        avg_section.lift_coeff = 0;
        // drag coefficient calculation here...
    }

    new public void set_position(aircraft ac, ref actions_class actns)
    {
        if (MathF.Abs(actns.values[action_key] - angle) > rotation_speed * physics.Ts)
        {
            if (actns.values[action_key] > angle)
            {
                angle += rotation_speed * physics.Ts;
            }
            else if (actns.values[action_key] < angle)
            {
                angle -= rotation_speed * physics.Ts;
            }

            if (MathF.Abs(actns.values[action_key] - angle) < rotation_speed * physics.Ts)
            {
                angle = actns.values[action_key] * max_angle;
            }
        }

        avg_section.aoa_rotated = avg_section.aoa_base + angle;
    }
}


public class aerodynamic_surface: surface
{
    public List<ad_surface_section> sections_main = new List<ad_surface_section>();
    public List<ad_surface_section> sections_all = new List<ad_surface_section>();

    public List<control_surface> control_surfaces = new List<control_surface>();

    public Vector3 one_axis_nvec;

    public float sections_step;
    public float slope;
    public float length;



    public aerodynamic_surface get_mirrored_copy()
    {
        aerodynamic_surface copy = this;
        copy.one_axis_nvec.Y *= -1;
        copy.lift_force_nvec.Y *= -1;

        foreach (ad_surface_section section in sections_main)
        {
            section.ad_center.Y *= -1;
            section.chord_center.Y *= -1;
        }

        foreach (ad_surface_section section in sections_all)
        {
            section.ad_center.Y *= -1;
            section.chord_center.Y *= -1;
        }

        return copy;
    }

    public void calc_initial()
    {
        calc_sections_main_aoa();
        calc_sections_all();
    }

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
                        sections_main[j].aoa_rotated = sections_main[j].aoa_base;
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
                    new Vector3(
                        (sections_main[i].ad_center.X + (sections_main[i + 1].ad_center.X - sections_main[i].ad_center.X) * ((float)j + 1) / divisions),
                        (sections_main[i].ad_center.Y + (sections_main[i + 1].ad_center.Y - sections_main[i].ad_center.Y) * ((float)j + 1) / divisions),
                        (sections_main[i].ad_center.Z + (sections_main[i + 1].ad_center.Z - sections_main[i].ad_center.Z) * ((float)j + 1) / divisions)),
                    new Vector3(
                        (sections_main[i].chord_center.X + (sections_main[i + 1].chord_center.X - sections_main[i].chord_center.X) * ((float)j + 1) / divisions),
                        (sections_main[i].chord_center.Y + (sections_main[i + 1].chord_center.Y - sections_main[i].chord_center.Y) * ((float)j + 1) / divisions),
                        (sections_main[i].chord_center.Z + (sections_main[i + 1].chord_center.Z - sections_main[i].chord_center.Z) * ((float)j + 1) / divisions))
                    ));
            }
        }
    }



    public void recalc_lift_force_nvec(surface surf)
    {
        surf.lift_force_nvec = new Vector3(
            MathF.Asin(avg_section.aoa_rotated / 180 * MathF.PI) * (one_axis_nvec.Z - 1), 
            -MathF.Asin(slope / 180 * MathF.PI) * one_axis_nvec.Y + one_axis_nvec.Z, 
            MathF.Sqrt(-MathF.Pow(MathF.Asin(avg_section.aoa_rotated / 180 * MathF.PI), 2) - MathF.Pow(MathF.Asin(slope / 180 * MathF.PI), 2) + 1));
    }

    public void recalc_sections_all_coefficients(aircraft ac) // recalculates coefficients for each section in 'sections_all'
    {
        for (int i = 0; i < sections_all.Count; i++)
        {
            int divisions = (int)((sections_main[i + 1].length_fraction * length - sections_main[i + 1].length_fraction * length) % sections_step);

            int sections_iterator = 0;

            for (int j = 0; i < divisions; j++)
            {
                sections_all[sections_iterator].aoa_no_sideslip = sections_all[sections_iterator].aoa_rotated + ac.aircraft_aoa + (MathF.Atan(((root_offset + length * sections_all[sections_iterator].length_fraction) * ac.roll / 180 * MathF.PI) / ac.speed) * 180 / MathF.PI);
                sections_all[sections_iterator].aoa_true = sections_all[sections_iterator].aoa_no_sideslip * MathF.Cos(ac.course_deviation / 180 * MathF.PI) + slope * MathF.Sin(ac.course_deviation / 180 * MathF.PI);
                sections_all[sections_iterator].lc_get();
                sections_all[sections_iterator].dc_get();

                sections_iterator++;
            }
        }
    }

    public void recalc_avg_section(int[] sections_nums, surface surf) // calculates average coefficients, chords lengths and forces application point for given sections of aerodynamic or control surface
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

        for (int k = sections_nums[0]; k < (sections_iterator + sections_nums[0]); k++)
        {
            float coeff = sections_all[k].chord_len / chords_sum;
            lc_res += sections_all[k].lift_coeff * coeff;
            dc_res += sections_all[k].drag_coeff * coeff;
            aoa_res += sections_all[k].aoa_true * coeff;

            surf.forces_app_point += sections_all[k].ad_center * coeff;

            surf.chord_center_point += sections_all[k].chord_center * coeff;

            surf.avg_section.chord_len += sections_all[k].chord_len * coeff;
        }

        surf.avg_section.lift_coeff = lc_res;
        surf.avg_section.drag_coeff = dc_res;
        surf.avg_section.aoa_true = aoa_res;
    }

    public virtual void recalc_main(aircraft ac, environment env, ref actions_class actns)
    {
        recalc_sections_all_coefficients(ac);

        recalc_avg_section(new int[2] { 0, (sections_all.Count - 1) }, this);
        recalc_lift_force_nvec(this);

        recalc_surface_forces(ac, env);
        drag_force_nvec = Vector3.Negate(ac.velocity_local_nvec);

        foreach (control_surface c_surf in control_surfaces)
        {
            recalc_avg_section(c_surf.sections_nums, c_surf);
            recalc_lift_force_nvec(c_surf);

            c_surf.set_position(ac, ref actns);
            c_surf.recalc_coefficients();

            c_surf.recalc_surface_forces(ac, env);
            c_surf.drag_force_nvec = Vector3.Negate(ac.velocity_local_nvec);
        }

    }
}

class ad_monosurface: aerodynamic_surface, rotation_object
{
    public string action_key { get; set; }

    public float angle { get; set; }
    public float min_angle { get; set; }
    public float max_angle { get; set; }
    public float rotation_speed { get; set; }

    public void set_position(aircraft ac, actions_class actns)
    {
        if (MathF.Abs(actns.values[action_key] * min_angle - angle) > rotation_speed * physics.Ts)
        {
            if (actns.values[action_key] > 0)
            {
                if (actns.values[action_key] * max_angle > angle)
                {
                    angle += rotation_speed * physics.Ts;
                }
                else if (actns.values[action_key] * max_angle < angle)
                {
                    angle -= rotation_speed * physics.Ts;
                }
            }
            else if (actns.values[action_key] < 0)
            {
                if (actns.values[action_key] * min_angle > angle)
                {
                    angle += rotation_speed * physics.Ts;
                }
                else if (actns.values[action_key] * min_angle < angle)
                {
                    angle -= rotation_speed * physics.Ts;
                }
            }

            if (MathF.Abs(actns.values[action_key] * min_angle - angle) < rotation_speed * physics.Ts)
            {
                angle = actns.values[action_key] * min_angle;
            }
        }

        avg_section.aoa_rotated = avg_section.aoa_base + angle;
    }

    

    public override void recalc_main(aircraft ac, environment env, ref actions_class actns) 
    {
        set_position(ac, actns);

        foreach (ad_surface_section section in sections_main)
        {
            section.aoa_rotated = section.aoa_base + angle;
        }

        foreach (ad_surface_section section in sections_all)
        {
            section.aoa_rotated = section.aoa_base + angle;
        }

        recalc_sections_all_coefficients(ac);

        recalc_avg_section(new int[2] { 0, (sections_all.Count - 1) }, this);
        recalc_lift_force_nvec(this);

        recalc_coefficients();
        recalc_surface_forces(ac, env);
        drag_force_nvec = Vector3.Negate(ac.velocity_local_nvec);
    }
}
