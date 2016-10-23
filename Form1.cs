using DEASL.Core.Rendering;
using MobilEyeInterface;
//using Rendering;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using UrbanChallenge.Pose;
using UrbanChallenge.Common;
using DEASL.Components.CommonRenderables;
using DEASL.Core.Mathematics;
using DEASL.Core.GeneralStructures;

namespace asl
{

    public partial class Form1 : Form
    {
        Renderer renderer;
        PoseRenderable leftPathRenderable, rightpPathRenderable;
        PoseRenderable poseRenderable;

        PoseClient poseClient = new PoseClient(IPAddress.Parse("239.132.1.33"), 4839);
        MobilEyeInterfaceRX mobileyeInterface = new MobilEyeInterfaceRX();

        public Form1()
        {
            InitializeComponent();
            poseClient.Start();

            // initializing the leftPathRenderable
            leftPathRenderable = new PoseRenderable();
            leftPathRenderable.PointColor = Color.Blue;
            // initializing the rightPathRenderable
            rightpPathRenderable = new PoseRenderable();
            rightpPathRenderable.PointColor = Color.Black;
            // initialzing the ECEF poseRenderable of the vehicle
            poseRenderable = new PoseRenderable();
            poseRenderable.PointColor = Color.Red;
            // starting the event handlers
            poseClient.PoseAbsReceived += PoseClient_PoseAbsReceived; 
            mobileyeInterface.GotMobileyeRoadInformation += MobileyeInterface_GotMobileyeRoadInformation;
        }

        bool gotFirstPosePacket = false;
        double initialX = 0, initialY = 0, initialZ = 0;

        private Single disToLeftLaneMark;
        private Single disToRightLaneMark;

        private void PoseClient_PoseAbsReceived(object sender, PoseAbsReceivedEventArgs e)
        {
            if (disToLeftLaneMark == disToRightLaneMark && disToLeftLaneMark == 0 && disToRightLaneMark == 0) return;
            PoseYPR leftPath, rightPath;
            double cons_left, cons_right;
            DEASL.Core.Mathematics.Vector3 new_x, new_y, new_z;
            DEASL.Core.Mathematics.Vector3 leftMarkVec, rightMarkVec;

            // plotting the pose
            if (!gotFirstPosePacket)
            {
                initialX = e.PoseAbsData.ecef_px;
                initialY = e.PoseAbsData.ecef_py;
                initialZ = e.PoseAbsData.ecef_pz;
                gotFirstPosePacket = true;
            }

            double X = e.PoseAbsData.ecef_px - initialX;
            double Y = e.PoseAbsData.ecef_py - initialY;
            double Z = e.PoseAbsData.ecef_pz - initialZ;

            PoseYPR toPlot = new PoseYPR(X, Y, Z, (Angle)e.PoseAbsData.yaw, (Angle)e.PoseAbsData.pitch, (Angle)e.PoseAbsData.roll);
            poseRenderable.AddPose(toPlot);

            // plotting the left lane mark
            double vx = e.PoseAbsData.ecef_vx;
            double vy = e.PoseAbsData.ecef_vy;
            double vz = e.PoseAbsData.ecef_vz;

            new_x = new DEASL.Core.Mathematics.Vector3(vx, vy, vz);
            new_z = new DEASL.Core.Mathematics.Vector3(X, Y, Z);
            new_y = new_x.Cross(new_z);
            new_y = new_y.GetNormalizedVector();

            leftMarkVec = new DEASL.Core.Mathematics.Vector3(0, 0, 0);
            rightMarkVec = new DEASL.Core.Mathematics.Vector3(0, 0, 0);

            cons_left = Math.Sqrt(Math.Abs(disToLeftLaneMark));
    
            leftMarkVec.X = new_y.X * cons_left + X;
            leftMarkVec.Y = new_y.Y * cons_left + Y;
            leftMarkVec.Z = new_y.Z * cons_left + Z;

            leftPath = new PoseYPR(leftMarkVec.X, leftMarkVec.Y, leftMarkVec.Z, (Angle)0, (Angle)0, (Angle)0);
            leftPathRenderable.AddPose(leftPath);
            // plotting the right lane mark
            cons_right = -Math.Sqrt(Math.Abs(disToRightLaneMark));
            rightMarkVec.X = new_y.X * cons_right + X;
            rightMarkVec.Y = new_y.Y * cons_right + Y;
            rightMarkVec.Z = new_y.Z * cons_right + Z;

            rightPath = new PoseYPR(rightMarkVec.X, rightMarkVec.Y, rightMarkVec.Z, (Angle)0, (Angle)0, (Angle)0);
            rightpPathRenderable.AddPose(rightPath);
            // the end of plotting
        }

        private void MobileyeInterface_GotMobileyeRoadInformation(object sender, MobileyeRoadInformationEventArgs e)
        {
            // reading the distance to both left and right lane marks
            disToLeftLaneMark = e.info.distToLeftMark;
            disToRightLaneMark = e.info.distToRightMark;

            if (!(disToLeftLaneMark == 0 && disToRightLaneMark == 0))
            {
                print(richTextBox1, e);
            }
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            renderer = new Renderer(this.drawingPad, 50);

            renderer.AddRenderable(leftPathRenderable);
            renderer.AddRenderable(rightpPathRenderable);
            renderer.AddRenderable(poseRenderable);

            renderer.OnFormShown();
        }




        // =========================================================================================
        // =========================================================================================
        // print for debug use
        public void print(RichTextBox r, PoseAbsReceivedEventArgs e)
        {
            //MethodInvoker action = delegate
            //{ r.AppendText(e.PoseAbsData.yaw + "  " + e.PoseAbsData.pitch + "  " + e.PoseAbsData.roll + "\n"); };
            //r.Invoke(action);
            MethodInvoker action = delegate
            { r.AppendText(e.PoseAbsData.ecef_vx + "  " + e.PoseAbsData.ecef_vy + "  " + e.PoseAbsData.ecef_vz + "\n"); };
            r.Invoke(action);
        }

        public void print(RichTextBox r, MobilEyeWorldObstacle e)
        {
            MethodInvoker action = delegate
                                    { r.AppendText(e.obstacleWidth.ToString()); };
            r.Invoke(action);
        }

        public void print(RichTextBox r, MobileyeRoadInformationEventArgs e)
        {
            MethodInvoker action = delegate
            { r.AppendText(e.info.distToLeftMark.ToString() + " " + e.info.distToRightMark.ToString() + "\n"
                ); };
            r.Invoke(action);
        }

        private void drawingPad_Load(object sender, EventArgs e)
        {

        }

        private void richTextBox1_TextChanged(object sender, EventArgs e)
        {

        }
    }
}
