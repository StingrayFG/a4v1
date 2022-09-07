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



    public Vector3 get_rotated_vector(Vector3 vec, Vector3 axis_nvec, float angle)
    {
        Vector3 res;

        float[,] R = new float[3, 3]
        {
            {
                MathF.Cos(angle) + axis_nvec.X * axis_nvec.X * (1 - MathF.Cos(angle)),
                axis_nvec.X * axis_nvec.Y * (1 - MathF.Cos(angle)) - axis_nvec.Z * MathF.Sin(angle),
                axis_nvec.X * axis_nvec.Z * (1 - MathF.Cos(angle)) + axis_nvec.Y * MathF.Sin(angle)
            },
            {
                axis_nvec.Y * axis_nvec.X * (1 - MathF.Cos(angle)) + axis_nvec.Z * MathF.Sin(angle),
                MathF.Cos(angle) + axis_nvec.Y  * axis_nvec.Y * (1 - MathF.Cos(angle)),
                axis_nvec.Y * axis_nvec.Z * (1 - MathF.Cos(angle)) - axis_nvec.X * MathF.Sin(angle)
            },
            {
                axis_nvec.Z * axis_nvec.X * (1 - MathF.Cos(angle)) - axis_nvec.Y * MathF.Sin(angle),
                axis_nvec.Z * axis_nvec.Y * (1 - MathF.Cos(angle)) + axis_nvec.X * MathF.Sin(angle),
                MathF.Cos(angle) + axis_nvec.Z * axis_nvec.Z * (1 - MathF.Cos(angle))
            }
        };

        res.X = R[0, 0] * vec.X + R[0, 1] * vec.Y + R[0, 2] * vec.Z;
        res.Y = R[1, 0] * vec.X + R[1, 1] * vec.Y + R[1, 2] * vec.Z;
        res.Z = R[2, 0] * vec.X + R[2, 1] * vec.Y + R[2, 2] * vec.Z;

        return res;
    }



    public void recalc_surfaces(environment env, actions_class actns)
    {
        foreach (aerodynamic_surface surf in surfaces)
        {
            surf.recalc_main(this, env, ref actns);
        }
    }

    public void recalc_com()
    {
        float coeff = fuel_weight / fuel_weight_max;

        auw = zfw + fuel_weight;

        center_of_mass_auw = center_of_mass_zfw + ((center_of_mass_ffw - center_of_mass_zfw) * coeff);

        fuel_weight -= engine.fuel_cons * physics.Ts;
    }

    public void recalc_torque()
    {
        foreach (aerodynamic_surface surf in surfaces)
        {
            torque += (Vector3)(surf.lift_force * surf.lift_force_nvec * (surf.forces_app_point - center_of_mass_auw));

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

    public void recalc_rotation()
    {
        angular_acceleration += torque / moment_of_inertia * 180 / MathF.PI;
        angular_velocity += angular_acceleration * physics.Ts;
        rotation += angular_velocity * physics.Ts;

        rotation.X = rotation.X % 360;
        rotation.Y = rotation.Y % 360;
        rotation.Z = rotation.Z % 360;

        ac_axis_global_nvec.X = MathF.Sin((rotation.Z + 90) / 180 * MathF.PI) * MathF.Cos(rotation.Y / 180 * MathF.PI);
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

    public void recalc_velocity_and_position()
    {
        acceleration_local_vec += forces_local / auw;
        velocity_local_vec += acceleration_local_vec * physics.Ts;
        speed = velocity_local_vec.Length();
        velocity_local_nvec = velocity_local_vec / speed;

        //velocity_global_nvec.X = MathF.Cos(MathF.Asin(ac_axis_global_nvec.Z / 180 * MathF.PI) + MathF.Asin(velocity_local_nvec.Z / 180 * MathF.PI) - MathF.Asin(ac_axis_local_nvec.Z / 180 * MathF.PI)) * 
        //    MathF.Cos(MathF.Asin(velocity_local_nvec.Y / 180 * MathF.PI) - MathF.Asin(ac_axis_local_nvec.Y / 180 * MathF.PI)); ;
        //velocity_global_nvec.Y = MathF.Sin(MathF.Asin(ac_axis_global_nvec.Z / 180 * MathF.PI) + MathF.Asin(velocity_local_nvec.Z / 180 * MathF.PI) - MathF.Asin(ac_axis_local_nvec.Z / 180 * MathF.PI)) * 
        //    MathF.Cos(MathF.Asin(velocity_local_nvec.Y / 180 * MathF.PI) - MathF.Asin(ac_axis_local_nvec.Y / 180 * MathF.PI)); ;
        //velocity_global_nvec.Z = MathF.Sin(MathF.Asin(velocity_local_nvec.Y / 180 * MathF.PI) - MathF.Asin(ac_axis_local_nvec.Y / 180 * MathF.PI));

        ////velocity_global_nvec.X = MathF.Acos(rotation.Z + (MathF.Cos(velocity_local_nvec.X / velocity_local_nvec.Z / 180 * MathF.PI) * ((MathF.Round((rotation.Z - 180) / 360) * -2) + 1))) *
        ////((MathF.Round((rotation.Z + (MathF.Cos(velocity_local_nvec.X / 180 * MathF.PI) * ((MathF.Round((rotation.Z - 180) / 360) * -2) + 1)) - 180) / 360) * -2) + 1);
        ////velocity_global_nvec.Y = MathF.Asin(rotation.Z + (MathF.Cos(velocity_local_nvec.X / velocity_local_nvec.Z / 180 * MathF.PI) * ((MathF.Round((rotation.Z - 180) / 360) * -2) + 1)));
        ////velocity_global_nvec.Z = rotation.Y + MathF.Asin(velocity_local_nvec.Z);

        velocity_global_nvec.X = MathF.Sin((rotation.Z + MathF.Asin(velocity_local_nvec.Y) + 90) / 180 * MathF.PI) * MathF.Cos(rotation.Y);
        velocity_global_nvec.Y = MathF.Sin((rotation.Z + MathF.Asin(velocity_local_nvec.Y)) / 180 * MathF.PI) * MathF.Cos(rotation.Y);
        velocity_global_nvec.Z = MathF.Sin((rotation.Y + MathF.Asin(velocity_local_nvec.Z)) / 180 * MathF.PI);

        velocity_global_nvec = get_rotated_vector(velocity_global_nvec, ac_axis_global_nvec, -rotation.X);

        velocity_global_vec = velocity_global_nvec * speed;

        position.X += velocity_global_vec.X * physics.Ts;
        position.Y += velocity_global_vec.Y * physics.Ts;
        position.Z += velocity_global_vec.Z * physics.Ts;
    }

    public void recalc_main(environment env)
    {
        recalc_surfaces(env, ac_actions);
        engine.recalc_fuel_cons();
        recalc_com();

        recalc_torque();
        recalc_rotation();

        recalc_forces();
        recalc_velocity_and_position();    
    }
}









