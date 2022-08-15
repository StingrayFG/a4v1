using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;



public class aircraft
{
    public List<aerodynamic_surface> surfaces = new List<aerodynamic_surface>();

    public engine_class engine;

    public float aircraft_aoa;
    public float course_deviation; // left to right
    
    public float roll;
    public float altitude;

    public Vector3 ac_axis_local_nvec;
    public Vector3 ac_axis_global_nvec;

    public Vector3 forces_local;
    public Vector3 acceleration_local_vec;

    public Vector3 velocity_local_vec;
    public Vector3 velocity_local_nvec;

    public Vector3 velocity_global_vec;
    public Vector3 velocity_global_nvec;
    public float speed;
    public Point3 position;

    public Vector3 moment_of_inertia;
    public Vector3 torque;
    public Vector3 angular_acceleration;
    public Vector3 angular_velocity;
    public Vector3 rotation;

    public Vector3 gravity_nvec;

    public float zfw;
    public float auw;
    public float ffw;

    public float fuel_weight;
    public float fuel_weight_max;

    public Point3 center_of_mass_zfw;
    public Point3 center_of_mass_auw;
    public Point3 center_of_mass_ffw;

    actions_class ac_actions = new actions_class();

    // X - roll, parallel to aircraft central axis, directed forward; Y - pitch, horizontal, perpendicular to aircraft central axis, directed left; Z - rudder, vertical, directed up; 

    public void calc_initial()
    {
        ffw = zfw + fuel_weight_max;
    }



    public void recalc_surfaces(environment env, actions_class actns)
    {
        foreach (aerodynamic_surface surf in surfaces)
        {
            surf.recalc_main(this, env, ref actns);
        }
    }

    public void recalc_com(environment env)
    {
        float coeff = fuel_weight / fuel_weight_max;

        auw = zfw + fuel_weight;

        center_of_mass_auw.X = center_of_mass_zfw.X + ((center_of_mass_ffw.X - center_of_mass_zfw.X) * coeff);
        center_of_mass_auw.Y = center_of_mass_zfw.Y + ((center_of_mass_ffw.Y - center_of_mass_zfw.Y) * coeff);
        center_of_mass_auw.Z = center_of_mass_zfw.Z + ((center_of_mass_ffw.X - center_of_mass_zfw.Z) * coeff);

        fuel_weight -= engine.fuel_cons * env.physics_step;
    }

    public void recalc_torque()
    {
        foreach (aerodynamic_surface surf in surfaces)
        {
            torque.X += surf.lift_force * surf.lift_force_nvec.X * (surf.forces_app_point.X - center_of_mass_auw.X);
            torque.Y += surf.lift_force * surf.lift_force_nvec.Y * (surf.forces_app_point.Y - center_of_mass_auw.Y);
            torque.Z += surf.lift_force * surf.lift_force_nvec.Z * (surf.forces_app_point.Z - center_of_mass_auw.Z);

            torque.X += surf.drag_force * surf.drag_force_nvec.X * (surf.forces_app_point.X - center_of_mass_auw.X);
            torque.Y += surf.drag_force * surf.drag_force_nvec.Y * (surf.forces_app_point.Y - center_of_mass_auw.Y);
            torque.Z += surf.drag_force * surf.drag_force_nvec.Z * (surf.forces_app_point.Z - center_of_mass_auw.Z);

            foreach (control_surface c_surf in surf.control_surfaces)
            {
                torque.X += c_surf.lift_force * c_surf.lift_force_nvec.X * (c_surf.forces_app_point.X - center_of_mass_auw.X);
                torque.Y += c_surf.lift_force * c_surf.lift_force_nvec.Y * (c_surf.forces_app_point.Y - center_of_mass_auw.Y);
                torque.Z += c_surf.lift_force * c_surf.lift_force_nvec.Z * (c_surf.forces_app_point.Z - center_of_mass_auw.Z);

                torque.X += c_surf.drag_force * c_surf.drag_force_nvec.X * (c_surf.forces_app_point.X - center_of_mass_auw.X);
                torque.Y += c_surf.drag_force * c_surf.drag_force_nvec.Y * (c_surf.forces_app_point.Y - center_of_mass_auw.Y);
                torque.Z += c_surf.drag_force * c_surf.drag_force_nvec.Z * (c_surf.forces_app_point.Z - center_of_mass_auw.Z);
            }
        }
    }

    public void recalc_rotation(environment env)
    {
        angular_acceleration += torque / moment_of_inertia * 180 / MathF.PI;
        angular_velocity += angular_acceleration * env.physics_step;
        rotation += angular_velocity * env.physics_step;

        rotation.X = rotation.X % 360;
        rotation.Y = rotation.Y % 360;
        rotation.Z = rotation.Z % 360;

        ac_axis_global_nvec.X = MathF.Cos(rotation.Z / 180 * MathF.PI) * MathF.Cos(rotation.Y / 180 * MathF.PI);
        ac_axis_global_nvec.Y = MathF.Sin(rotation.Z / 180 * MathF.PI) * MathF.Cos(rotation.Y / 180 * MathF.PI);
        ac_axis_global_nvec.Z = MathF.Sin(rotation.Y / 180 * MathF.PI);
    }

    public void recalc_forces()
    {
        foreach (aerodynamic_surface surf in surfaces)
        {
            if (surf.one_axis_nvec.Z != 1)
            {
                forces_local += surf.lift_force * surf.lift_force_nvec;
                forces_local += surf.drag_force * surf.drag_force_nvec;

                foreach(control_surface c_surf in surf.control_surfaces)
                {
                    forces_local += c_surf.lift_force * c_surf.lift_force_nvec;
                    forces_local += c_surf.drag_force * c_surf.drag_force_nvec;
                }
            }

            forces_local += engine.thrust * engine.thrust_nvec;
            forces_local += gravity_nvec * physics.g * auw;
        }
    }

    public void recalc_velocity_and_position(environment env)
    {
        acceleration_local_vec += forces_local / auw;
        velocity_local_vec += acceleration_local_vec * env.physics_step;
        speed = velocity_local_vec.Length();
        velocity_local_nvec = velocity_local_vec / speed;

        velocity_global_nvec.X = MathF.Cos(MathF.Asin(ac_axis_global_nvec.Z / 180 * MathF.PI) + MathF.Asin(velocity_local_nvec.Z / 180 * MathF.PI) - MathF.Asin(ac_axis_local_nvec.Z / 180 * MathF.PI)) * 
            MathF.Cos(MathF.Asin(velocity_local_nvec.Y / 180 * MathF.PI) - MathF.Asin(ac_axis_local_nvec.Y / 180 * MathF.PI)); ;
        velocity_global_nvec.Y = MathF.Sin(MathF.Asin(ac_axis_global_nvec.Z / 180 * MathF.PI) + MathF.Asin(velocity_local_nvec.Z / 180 * MathF.PI) - MathF.Asin(ac_axis_local_nvec.Z / 180 * MathF.PI)) * 
            MathF.Cos(MathF.Asin(velocity_local_nvec.Y / 180 * MathF.PI) - MathF.Asin(ac_axis_local_nvec.Y / 180 * MathF.PI)); ;
        velocity_global_nvec.Z = MathF.Sin(MathF.Asin(velocity_local_nvec.Y / 180 * MathF.PI) - MathF.Asin(ac_axis_local_nvec.Y / 180 * MathF.PI));

        velocity_global_vec = velocity_global_nvec * speed;

        position.X += velocity_global_vec.X * env.physics_step;
        position.Y += velocity_global_vec.Y * env.physics_step;
        position.Z += velocity_global_vec.Z * env.physics_step;
    }

    public void recalc_main(environment env)
    {
        recalc_surfaces(env, ac_actions);
        engine.recalc_fuel_cons(env);
        recalc_com(env);

        recalc_torque();
        recalc_rotation(env);

        recalc_forces();
        recalc_velocity_and_position(env);    
    }
}









