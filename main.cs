using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;



class Program
{
    static void Main(string[] args)
    {
        //aerodynamic_surface ad = new aerodynamic_surface();
        //ad.slope = 5;
        //ad.length = 4;
        //ad.root_offset = 0.5f;

        //ad.root_functions = new lc_dc_stock();
        //ad.tip_functions = new lc_dc_stock();

        //ad.sections_main.Add(new ad_surface_section(5, 4.56f, 0, new Point3(6, 0.5f, 0.2f), new Point3(4, 0.5f, 0.2f)));

        //ad.sections_main.Add(new ad_surface_section(10, 0.4f, 0.5f, new Point3(5, 2f, 0.5f), new Point3(4, 2f, 0.5f)));

        //ad.sections_main.Add(new ad_surface_section(15, 0.4f, 1, new Point3(4, 4.5f, 0.5f), new Point3(3.5f, 4.5f, 0.5f)));

        //aircraft ac_new = new aircraft();
        //environment env_new = new environment(103250, 290);

        //ac_new.surfaces.Add(ad);

        //ad.calc_initial();



        //float rotationX = 30;
        //float rotationY = 90;
        //float rotationZ = 45;

        //Vector3 ac_axis_local_nvec = new Vector3(
        //    MathF.Cos(rotationZ / 180 * MathF.PI) * MathF.Cos(rotationY / 180 * MathF.PI),
        //    MathF.Sin(rotationZ / 180 * MathF.PI) * MathF.Cos(rotationY / 180 * MathF.PI),
        //    MathF.Sin(rotationY / 180 * MathF.PI));

        //Console.WriteLine(ac_axis_local_nvec.X);
        //Console.WriteLine(ac_axis_local_nvec.Y);
        //Console.WriteLine(ac_axis_local_nvec.Z);
        //Console.WriteLine(ac_axis_local_nvec.Length());


        //ad.sections_main[1].functions = new lc_dc_stock();
        //ad.sections_main[2].functions = new lc_dc_stock();

        //ad.generate_sections_all();

        //ad.recalc_sections(0, 10, 0, 100);
        //ad.recalc_forces(new Vector3(1, 1, 1), 0.001f, 100);

        ////Console.WriteLine(ad.recalc_sections(5, 15));
        ///

        //Vector3 vec1 = new Vector3(1, 2, 4);

        //Point3 p1 = new Point3(3, 2, 1);

        //Point3 p2 = (Point3)vec1;

        //Vector3 vec2 = (Vector3)p1;

        //Console.WriteLine(p2.X.ToString() + " " + p2.Y.ToString() + " " + p2.Z.ToString());
        //Console.WriteLine(vec2.X.ToString() + " " + vec2.Y.ToString() + " " + vec2.Z.ToString());

        Point3 p1 = new Point3(2, 2, 4);
        Point3 p2 = new Point3(3, 2, 1);
        Point3 p3 = p1 + p2;
    }
}