/*
 * RealSense Tools
 * A toolkit for working with RealSense depth cameras.
 * Copyright 2020 CITA
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

using Intel.RealSense;
using Volvox_Cloud;

using Rhino.Geometry;
using Grasshopper.Kernel;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Drawing;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Threading;
using System.Runtime.InteropServices;

namespace RsTools.GH.Components
{
    public class Cmpt_RsCamera : GH_Component
    {
        public Cmpt_RsCamera()
          : base("RsCamera", "RsCam",
              "Get data from RealSense depth camera.",
              "RsTools", "Main")
        {
        }

        private Thread _thread = null;
        private volatile bool IsOn = false;

        private Rhino.Geometry.PointCloud pointcloud = new Rhino.Geometry.PointCloud();
        private Transform xform = Transform.Identity;
        private Box clipping_box = Box.Empty;

        private int DecimationMagnitude = 2;

        private int SpatialMagnitude = 2;
        private double SpatialSmoothAlpha = 1.0f;
        private double SpatialSmoothDelta = 2;

        private double TemporalSmoothAlpha = 0.1f;
        private double TemporalSmoothDelta = 100;

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("ON", "ON", "Turn the RealSense grabber on and off.", GH_ParamAccess.item, false);
            pManager.AddTransformParameter("Transform", "T", "Transform the sensor data.", GH_ParamAccess.item);
            pManager.AddBoxParameter("Clipping Box", "CP", "Clipping box for the sensor data.", GH_ParamAccess.item, Box.Empty);

            pManager.AddIntegerParameter("Dec Mag", "DM", "Decimation filter magnitude", GH_ParamAccess.item, DecimationMagnitude);
            pManager.AddIntegerParameter("Spat Mag", "SM", "Spatial smoothing filter magnitude", GH_ParamAccess.item, SpatialMagnitude);
            pManager.AddNumberParameter("Spat Alpha", "SA", "Spatial smoothing filter smooth alpha", GH_ParamAccess.item, SpatialSmoothAlpha);
            pManager.AddNumberParameter("Spat Delta", "SD", "Spatial smoothing filter smooth delta", GH_ParamAccess.item, SpatialSmoothDelta);

            pManager.AddNumberParameter("Temp Alpha", "TA", "Temporal smoothing filter smooth alpha", GH_ParamAccess.item, TemporalSmoothAlpha);
            pManager.AddNumberParameter("Temp Delta", "TD", "Temporal smoothing filter smooth delta", GH_ParamAccess.item, TemporalSmoothDelta);

            for (int i = 1; i < pManager.ParamCount; ++i)
                pManager[i].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Output", "O", "Generic output stuff.", GH_ParamAccess.item);
        }

        public Color[] GetPointColors(VideoFrame vf, Point2f[] tx)
        {
            Color[] colors = new Color[tx.Length];

            byte[] bytedata = new byte[vf.DataSize];
            Marshal.Copy(vf.Data, bytedata, 0, vf.DataSize);

            int width = vf.Width, height = vf.Height;
            int psize = vf.BitsPerPixel / 8;

            Parallel.For(0, tx.Length - 1, (i) =>
            {
                Point2f t = new Point2f(tx[i].X * (width - 1), tx[i].Y * (height - 1));
                double fraction_x, fraction_y, one_minus_x, one_minus_y;
                int ceil_x, ceil_y, floor_x, floor_y;
                byte c1, c2, c3, c4;
                byte[] rgb = new byte[3];
                byte b1, b2;

                floor_x = Math.Max(0, (int)Math.Floor(t.X));
                floor_y = Math.Max(0, (int)Math.Floor(t.Y));
                ceil_x = floor_x + 1;
                if (ceil_x >= width) ceil_x = floor_x;
                ceil_y = floor_y + 1;
                if (ceil_y >= height) ceil_y = floor_y;
                fraction_x = t.X - floor_x;
                fraction_y = t.Y - floor_y;
                one_minus_x = 1.0 - fraction_x;
                one_minus_y = 1.0 - fraction_y;

                for (int j = 0; j < 3; ++j)
                {
                    c1 = bytedata[floor_x * psize + floor_y * width * psize + j];
                    c2 = bytedata[ceil_x * psize + floor_y * width * psize + j];
                    c3 = bytedata[floor_x * psize + ceil_y * width * psize + j];
                    c4 = bytedata[ceil_x * psize + ceil_y * width * psize + j];

                    b1 = (byte)(one_minus_x * c1 + fraction_x * c2);
                    b2 = (byte)(one_minus_x * c3 + fraction_x * c4);

                    rgb[j] = (byte)(one_minus_y * (double)(b1) + fraction_y * (double)(b2));
                }

                colors[i] = Color.FromArgb(255, rgb[0], rgb[1], rgb[2]);
            });

            return colors;
        }

        private Rhino.Geometry.PointCloud CropCloud(Rhino.Geometry.PointCloud cloud, Box clipping_box)
        {
            var new_cloud = new Rhino.Geometry.PointCloud();
            var points = cloud.Where(x => clipping_box.Contains(x.Location));
            
            new_cloud.AddRange(points.Select(x => x.Location), points.Select(x => x.Color));
            return new_cloud;
        }

        private void RunThread()
        {
            Intel.RealSense.PointCloud pc = new Intel.RealSense.PointCloud();

            DecimationFilter dec_filter = new DecimationFilter();
            SpatialFilter spat_filter = new SpatialFilter();
            TemporalFilter temp_filter = new TemporalFilter();

            dec_filter.Options[Option.FilterMagnitude].Value = DecimationMagnitude;

            spat_filter.Options[Option.FilterMagnitude].Value = SpatialMagnitude;
            spat_filter.Options[Option.FilterSmoothAlpha].Value = Math.Min(1.0f, (float)SpatialSmoothAlpha);
            spat_filter.Options[Option.FilterSmoothDelta].Value = (float)SpatialSmoothDelta;

            temp_filter.Options[Option.FilterSmoothAlpha].Value = Math.Min(1.0f, (float)TemporalSmoothAlpha);
            temp_filter.Options[Option.FilterSmoothDelta].Value = (float)TemporalSmoothDelta;

            List<ProcessingBlock> filters = new List<ProcessingBlock> { dec_filter, spat_filter, temp_filter };
            Align align_to_depth = new Align(Stream.Depth);

            var cfg = new Config();
            cfg.EnableStream(Stream.Depth, 640, 480);
            cfg.EnableStream(Stream.Color, 1280, 720, Format.Rgb8);

            var pipeline = new Pipeline();
            var pp = pipeline.Start(cfg);

            while (IsOn)
            {
                using (var frames = pipeline.WaitForFrames())
                {
                    var aligned = align_to_depth.Process<FrameSet>(frames).DisposeWith(frames);
                    var color = aligned.ColorFrame.DisposeWith(frames);

                    pc.MapTexture(color);

                    var filtered = aligned[Stream.Depth].DisposeWith(frames);

                    foreach (var filter in filters)
                        filtered = filter.Process(filtered).DisposeWith(frames);

                    Points points = pc.Process<Points>(filtered);

                    var vertices = new Point3f[points.Count];
                    var tex_coords = new Point2f[points.Count];

                    points.CopyVertices<Point3f>(vertices);
                    points.CopyTextureCoords<Point2f>(tex_coords);

                    Debug.Assert(vertices.Length == tex_coords.Length);

                    // ======== CULL INVALID POINTS ======== 

                    if (true)
                    {

                        var flags = new bool[vertices.Length];
                        int new_size = 0;
                        for (int i = 0; i < vertices.Length; ++i)
                        {
                            if (vertices[i].Z > 0.1)
                            {
                                flags[i] = true;
                                new_size++;
                            }
                        }

                        var new_vertices = new Point3f[new_size];
                        var new_tex_coords = new Point2f[new_size];

                        for (int i = 0, j = 0; i < vertices.Length; ++i)
                        {
                            if (flags[i])
                            {
                                new_vertices[j] = vertices[i];
                                new_tex_coords[j] = tex_coords[i];
                                ++j;
                            }
                        }

                        vertices = new_vertices;
                        tex_coords = new_tex_coords;
                    }

                    // ======== TRANSFORM ======== 

                    if (xform.IsValid)
                    {
                        Parallel.For(0, vertices.Length - 1, (i) =>
                        {
                            vertices[i].Transform(xform);
                        });
                    }
                    
                    // ======== CLIP TO BOX ======== 
                    if (clipping_box.IsValid &&
                        clipping_box.X.Length > 0 &&
                        clipping_box.Y.Length > 0 &&
                        clipping_box.Z.Length > 0)
                    {
                        Point3d box_centre = clipping_box.Plane.Origin;
                        double minx = clipping_box.X.Min + box_centre.X, maxx = clipping_box.X.Max + box_centre.X;
                        double miny = clipping_box.Y.Min + box_centre.Y, maxy = clipping_box.Y.Max + box_centre.Y;
                        double minz = clipping_box.Z.Min + box_centre.Z, maxz = clipping_box.Z.Max + box_centre.Z;

                        var flags = new bool[vertices.Length];
                        int new_size = 0;
                        for (int i = 0; i < vertices.Length; ++i)
                        {
                            if (
                                vertices[i].X < maxx && vertices[i].X > minx &&
                                vertices[i].Y < maxy && vertices[i].Y > miny &&
                                vertices[i].Z < maxz && vertices[i].Z > minz
                                )
                            {
                                flags[i] = true;
                                new_size++;
                            }
                        }

                        var new_vertices = new Point3f[new_size];
                        var new_tex_coords = new Point2f[new_size];

                        for (int i = 0, j = 0; i < vertices.Length; ++i)
                        {
                            if (flags[i])
                            {
                                new_vertices[j] = vertices[i];
                                new_tex_coords[j] = tex_coords[i];
                                ++j;
                            }
                        }
                        
                        vertices = new_vertices;
                        tex_coords = new_tex_coords;
                    }

                    Debug.Assert(vertices.Length == tex_coords.Length);
                    
                    var point_colors = GetPointColors(color, tex_coords);

                    Rhino.Geometry.PointCloud new_pointcloud = new Rhino.Geometry.PointCloud();
                    new_pointcloud.AddRange(vertices.Select(x => new Point3d(x)), point_colors);

                    lock (pointcloud)
                        pointcloud = new_pointcloud;
                }
            }

            if (pipeline != null)
                pipeline.Stop();
        }

        private bool CheckFiltersChanged(IGH_DataAccess DA)
        {
            bool changed = false;
            double tempf = 0.0;
            int tempi = 0;

            if (DA.GetData("Dec Mag", ref tempi))
                if (tempi != DecimationMagnitude)
                {
                    DecimationMagnitude = tempi;
                    changed = true;
                }

            if (DA.GetData("Spat Mag", ref tempi))
                if (tempi != SpatialMagnitude)
                {
                    SpatialMagnitude = tempi;
                    changed = true;
                }

            if (DA.GetData("Spat Alpha", ref tempf))
                if (tempf != SpatialSmoothAlpha)
                {
                    SpatialSmoothAlpha = tempf;
                    changed = true;
                }

            if (DA.GetData("Spat Delta", ref tempf))
                if (tempf != SpatialSmoothDelta)
                {
                    SpatialSmoothDelta = tempf;
                    changed = true;
                }

            if (DA.GetData("Temp Alpha", ref tempf))
                if (tempf != TemporalSmoothAlpha)
                {
                    TemporalSmoothAlpha = tempf;
                    changed = true;
                }

            if (DA.GetData("Temp Delta", ref tempf))
                if (tempf != TemporalSmoothAlpha)
                {
                    TemporalSmoothDelta = tempf;
                    changed = true;
                }

            return changed;
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //if (_thread != null && _thread.IsAlive && filters_changed)
            //{
            //    IsOn = false;
            //    _thread.Join();
            //}

            DA.GetData("ON", ref IsOn);
            DA.GetData("Transform", ref xform);
            DA.GetData("Clipping Box", ref clipping_box);

            if (_thread == null || !_thread.IsAlive)
                _thread = new Thread(new ThreadStart(RunThread));

            if (IsOn && _thread.ThreadState == System.Threading.ThreadState.Unstarted)
            {
                bool filters_changed = CheckFiltersChanged(DA);
                _thread.Start();
            }
            else if (!IsOn)
            {
                if (_thread != null && _thread.IsAlive)
                    _thread.Join();
            }

            lock (pointcloud)
                DA.SetData("Output", new GH_Cloud(pointcloud));
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return null;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("753148CD-0ECF-492E-A614-64F5AD0FF0B6"); }
        }
    }
}