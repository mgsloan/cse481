using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Aldebaran.Proxies;
using System.Collections;
using Microsoft.Xna.Framework;
using System.Threading;

namespace KinectViewer
{
    class Balancer
    {
        NaoSimulator naoSim;
        NaoProxy nao;
        private float speed = 0.2f;

        public Balancer(NaoSimulator naoSim)
        {
            this.naoSim = naoSim;
            this.nao = naoSim.proxy;
        }

        public void Balance(int feet, List<LabelledVector> ls, Vector3 torso)
        {
            NaoFoot targetFoot;
            string prefix;

            if (feet == 2) {
                targetFoot = naoSim.GetRightFoot();
                prefix = "R";
            } else {
                targetFoot = naoSim.GetLeftFoot();
                prefix = "L";
            }

            /*
            Matrix groundRef = targetFoot.pfl.transform;
            groundRef.Translation = Vector3.Zero;
            
            NaoPos kneePos = naoSim.GetPosition(prefix ++ "KneePitch");

            naoSim.UpdateAngle(prefix ++ "HipRoll", );
            */
            
            // Center of mass, and center of target, both in torso space.
            Vector3 com = naoSim.GetCOM();
            Vector3 target = targetFoot.GetCenter();

            // Balance vector.  We need it to be vertical.
            Vector3 delta = Vector3.Subtract(com, target);

            // Transform into lower leg local space.
            Matrix mat = Matrix.Invert(MathUtils.ExtractRotation(naoSim.GetTransform(prefix + "KneePitch")));
            Vector3 local = Vector3.Transform(delta, mat);

            // Take the angle of the vector to be the angle we need to rotate
            // the ground plane in order to achieve balance.
            float roll  = (float) Math.Atan2(local.X, local.Y);
            float pitch = (float) Math.Atan2(local.Z, local.Y);

            // Use Force sensors to tweak result.
            float forwardBias = MathUtils.Average(targetFoot.ffl - targetFoot.frl, targetFoot.ffr - targetFoot.frr) * 0.01f;
            float leftwardBias = MathUtils.Average(targetFoot.ffl - targetFoot.ffr, targetFoot.frl - targetFoot.frr) * 0.01f;
            //Console.WriteLine("Biases: " + forwardBias.ToString() + " " + leftwardBias.ToString());

            Vector3 offset = new Vector3(0, 0, 3f);
            ls.Add(new LabelledVector(offset, Vector3.Add(offset, local), Color.Black, ""));
            ls.Add(new LabelledVector(offset, new Vector3(leftwardBias, 1f, 3f + forwardBias), Color.Green, ""));
            
            // Foot commands with experimental fudge factors
            if (feet == 2)
            {
                pitch += forwardBias;
                roll += leftwardBias;
                naoSim.RAUpdate(pitch + 0.05f, -roll);
            }
            else
            {
                pitch += forwardBias;
                roll += leftwardBias;
                naoSim.LAUpdate(pitch - 0.05f, 0.05f - roll);
            }
        }

        public void doEveryting(NaoSimulator sim)
        {
            // Get foot objects
            //NaoFoot leftFoot = proxy.GetLeftFoot();
            //NaoFoot rightFoot = proxy.GetRightFoot();
            NaoFoot leftFootsim = sim.GetLeftFoot();
            NaoFoot rightFootsim = sim.GetRightFoot();

            // Get foot and COM locations
            Vector3 LFPsim = sim.GetPosition("LAnkleRoll");
            Vector3 RFPsim = sim.GetPosition("RAnkleRoll");
            Vector3 COMsim = sim.GetCOM();
            //Vector3 LFP = proxy.GetPosition("LAnkleRoll").position;
            //Vector3 RFP = proxy.GetPosition("RAnkleRoll").position;
            //Vector3 COM = NaoPos.Convert(proxy.GetCOM());
            //Vector3 leftFootPos = proxy.GetPos("LAnkleRoll");
            //Vector3 rightFootPos = proxy.GetPos("RAnkleRoll");
            //Vector3 COM2 = proxy.GetCOM();

            // Update feet for offset calculations
            //leftFoot.updateFoot(COM);
            //rightFoot.updateFoot(COM);
            leftFootsim.updateFoot(COMsim);
            rightFootsim.updateFoot(COMsim);
            //updateFoot(leftFoot, COM2);
            //updateFoot(rightFoot, COM2);

            // Obtain the offset parameter
            //float offsetL = leftFoot.GetOffset();
            //float offsetR = rightFoot.GetOffset();
            //float offset = OffsetParameter(offsetL, offsetR);
            float offsetLsim = leftFootsim.GetOffset();
            float offsetRsim = rightFootsim.GetOffset();
            float offsetsim = OffsetParameter(offsetLsim, offsetRsim);

            // Get Egoal, project feet onto Egoal plane
            Tuple<Vector3, Vector3> Egoal = GetEgoal(offsetsim, LFPsim, RFPsim);
            Tuple<Vector3, Vector3> footProj = GetFootProj(Egoal, LFPsim, RFPsim);
            //Tuple<Vector3,Vector3> Egoal = GetEgoal(offset, leftFootPos, rightFootPos);
            //Tuple<Vector3, Vector3> footProj = GetFootProj(Egoal, leftFootPos, rightFootPos);

            //**********************************************************
            // TODO: determine which foot(feet) the robot is standing on
            //**********************************************************
            //float leftDiff = footProj.Item1.Z - leftFootPos.Z;
            //float rightDiff = footProj.Item2.Z - rightFootPos.Z;
            float leftDiff = footProj.Item1.Y - LFPsim.Y;
            float rightDiff = footProj.Item2.Y - RFPsim.Y;


            // Rotate Egoal so COM is within support polygon
            Tuple<Vector3, Vector3> EgoalNew = RotateEgoal(Egoal, footProj, offsetsim, 2, COMsim);
            
            // reproject feet onto rotated EgoalNew to get the final foot positions
            Tuple<Vector3, Vector3> footProjNew = GetFootProj2(EgoalNew, LFPsim, RFPsim);

            // determine if feet actually need to be moved
            float leftlength = Vector3.Subtract(footProjNew.Item1, LFPsim).Length();
            float rightlength = Vector3.Subtract(footProjNew.Item2, RFPsim).Length();

            if ((leftlength > 0.005) || (rightlength > 0.005))
            {
                // move feet and ankles
                //footIK(footProjNew, LFPsim, RFPsim);
                //RotateAnkles(EgoalNew);
            }
        }

        // Calculate offset
        // From paper Algorithm 6.2
        public float OffsetParameter(float offsetL, float offsetR)
        {
            if (offsetL < offsetR)
            {
                return (offsetL / offsetR) / 2;
            }
            else if (offsetR < offsetL)
            {
                return 1 - ((offsetR / offsetL) / 2);
            }
            else
            {
                return 0.5f;
            }
        }

        /*UPDATE INITGRATING */
        /* moved into foot class--NOT-- NEEDS INTIGRATION WORK */
        // Calculate innerEdge and outerEdge for given foot
        // From paper section 6.2.1
        /*
        public void updateFoot(NaoFoot foot, Vector3 com)
        {
            Vector3 fr = proxy.GetPos(foot.name + "FsrFR"),
                    rr = proxy.GetPos(foot.name + "FsrRR"),
                    fl = proxy.GetPos(foot.name + "FsrFL"),
                    rl = proxy.GetPos(foot.name + "FsrRL");
 
            Vector3 leftSide = Vector3.Subtract(fl, rl);
            Vector3 rightSide = Vector3.Subtract(fr, rr);

            Vector3 COM = com; // proxy.GetCOM();

            Plane footPlane = new Plane(fr, fl, rr);
            Vector3 planeNormal = footPlane.Normal;

            // BEST PROJECTION FOUND
            float COMoffset = Vector3.Dot((fl - COM), planeNormal) / (Vector3.Dot(planeNormal, planeNormal));
            Vector3 COMPROJ = COMoffset * planeNormal + COM;
            // TEST PROJECTION
            float zero = Vector3.Dot((COMPROJ - fl), planeNormal);


            // ORIGINAL PROJECTION
            //A || B = B x (A x B) / |B|^2 
            //Vector3 COMproj = Vector3.Cross(planeNormal, (Vector3.Cross(COM, planeNormal))) / (planeNormal.LengthSquared());

            //(AB x AC)/|AB|
            // AB = leftSide, rightSide
            // A = rl, rr
            Vector3 tempL = Vector3.Subtract(COMPROJ, rl);
            Vector3 tempR = Vector3.Subtract(COMPROJ, rr);

            // use sine to get the distance from COMproj to the foot edges
            double distance1 = Math.Sin(Math.Acos((double)(Vector3.Dot(leftSide, tempL) / (leftSide.Length() * tempL.Length())))) * tempL.Length();
            double distance2 = Math.Sin(Math.Acos((double)(Vector3.Dot(rightSide, tempR) / (rightSide.Length() * tempR.Length())))) * tempR.Length();


            float width = Vector3.Distance(fr, fl);

            //if (distance1 < distance2)
            if (foot.name == "R")
            {
                foot.innerEdge = (float) distance1;
                foot.outerEdge = (float) distance2;
            }
            else
            {
                foot.innerEdge = (float) distance2;
                foot.outerEdge = (float) distance1;
            }
            foot.width = width;
        }
        */

        // Calculates the goal plane
        // Equations from paper section 6.2.2
        public Tuple<Vector3,Vector3> GetEgoal(float offset, Vector3 LARpos, Vector3 RARpos)
        {
            float LAnkleRoll = nao.GetData("LAnkleRoll");
            float RAnkleRoll = nao.GetData("RAnkleRoll");

            float Rgoal = ((1 - offset) * LAnkleRoll + (offset * RAnkleRoll)) / 2;
            Vector3 normal = new Vector3(0, Rgoal, 0);
            //Vector3 normal = new Vector3(0, 0, Rgoal);                               // Z was up/down - now its Y
            Vector3 position = LARpos + offset*(RARpos - LARpos);

            return new Tuple<Vector3, Vector3>(normal, position);
        }

        // Vertical projection of feet onto Egoal plane
        public Tuple<Vector3, Vector3> GetFootProj(Tuple<Vector3, Vector3> Egoal, Vector3 leftFoot, Vector3 rightFoot)
        {
            // TODO : what position do I need to use for this projection?
            // switch to use average position from foot sensors???????????
            //Vector3 leftFoot = leftFootPos; // proxy.GetPos("LAnkleRoll");
            //Vector3 rightFoot = rightFootPos; // proxy.GetPos("RAnkleRoll");

            // Vertical projection onto plane
            // SHOULD I DO AN ACTUAL PROJECTION HERE?????????????????
            Vector3 planePos = Egoal.Item2;
            Vector3 leftFootProj, rightFootProj;
            leftFootProj.X = leftFoot.X; leftFootProj.Y = planePos.Y; leftFootProj.Z = leftFoot.Z;          // now using planePos.Y
            rightFootProj.X = rightFoot.X; rightFootProj.Y = planePos.Y; rightFootProj.Z = rightFoot.Z;
            //leftFootProj.X = leftFoot.X; leftFootProj.Y = leftFoot.Y; leftFootProj.Z = planePos.Z;        // used planPos.Z
            //rightFootProj.X = rightFoot.X; rightFootProj.Y = rightFoot.Y; rightFootProj.Z = planePos.Z;

            return new Tuple<Vector3, Vector3>(leftFootProj, rightFootProj);
        }

        // rotate Egoal plane so that robot COM is over the support polygon
        // equations from paper section 6.3
        public Tuple<Vector3, Vector3> RotateEgoal(Tuple<Vector3, Vector3> Egoal, Tuple<Vector3, Vector3> footProj, float offset, int support, Vector3 COM)
        {
            Vector3 COMgoalProj;
            /*
            if (support == 0)           // left foot - position vector center of left foot -- -- -- -- -- -- -- -- should this be the bottom of the foot?
            {
                COMgoalProj = footProj.Item1;
            }
            else if (support == 1)      // right foot - position vector center of right foot
            {
                COMgoalProj = footProj.Item2;
            }
            else                        // both feet
            */
            {
                COMgoalProj = footProj.Item1 + offset * (footProj.Item2 - footProj.Item1);
            }

            Vector3 normal = COM - COMgoalProj;

            return new Tuple<Vector3, Vector3>(normal, COMgoalProj);
        }

        // Project the feet onto the rotated Egoal plane
        public Tuple<Vector3, Vector3> GetFootProj2(Tuple<Vector3, Vector3> plane, Vector3 LARpos, Vector3 RARpos)
        {
            Vector3 normal = plane.Item1;
            Vector3 position = plane.Item2;

            // Project left foot onto rotated goal plane
            float leftOffset = Vector3.Dot((position - LARpos), normal) / (Vector3.Dot(normal, normal));
            Vector3 leftProj = leftOffset * normal + LARpos;
            // TEST PROJECTION
            float zero = Vector3.Dot((leftProj - position), normal);

            // Project right foot onto rotated goal plane
            float rightOffset = Vector3.Dot((position - RARpos), normal) / (Vector3.Dot(normal, normal));
            Vector3 rightProj = rightOffset * normal + RARpos;
            // TEST PROJECTION
            float zero2 = Vector3.Dot((rightProj - position), normal);

            return new Tuple<Vector3, Vector3>(leftProj, rightProj);
        }

        // WARNING
        // ROTATE ANKLES HAS NOT BEEN UPDATED TO USE KINECT COORDINATES -- STILL USING UNCONVERTED NAO COORDINATES
        // Move feet to new positions using NAO's positionInterpolation
        public void footIK(Tuple<Vector3,Vector3> feet, Vector3 LARpos, Vector3 RARpos)
        {
            Vector3 leftFootProj = feet.Item1;
            Vector3 rightFootProj = feet.Item2;
            // effector, space, path, axisMask, times, isAbsolute

            string effector1 = "LLeg";
            string effector2 = "RLeg";
            int space = 0;                      // torso
            int axisMask = 7;                   // x, y, z
            float time = 2.0f;
            bool isAblosute = false;

            // move the difference between the two points
            float[] path1 = { leftFootProj.X - LARpos.X, leftFootProj.Y - LARpos.Y, leftFootProj.Z - LARpos.Z, 0.0f, 0.0f, 0.0f };
            nao.positionInterpolation(effector1, space, path1, axisMask, time, isAblosute);

            float[] path2 = { rightFootProj.X - RARpos.X, rightFootProj.Y - RARpos.Y, rightFootProj.Z - RARpos.Z, 0.0f, 0.0f, 0.0f };
            nao.positionInterpolation(effector2, space, path2, axisMask, time, isAblosute);
        }

        // WARNING
        // ROTATE ANKLES HAS NOT BEEN UPDATED TO USE KINECT COORDINATES -- STILL USING UNCONVERTED NAO COORDINATES
        // Rotate ankles onto Egoal plane - equation from paper section 6.5
        public void RotateAnkles(Tuple<Vector3, Vector3> Egoal)
        {
            Vector3 EgoalNorm = Egoal.Item1;

            // left ankle calculations
            float LAnklePitch = nao.GetData("LAnklePitch");
            float LAnkleRoll = nao.GetData("LAnkleRoll");

            Vector3 Leftlengthwise = new Vector3(LAnklePitch, 0, 0);
            Vector3 normalLeftAnklePitch = new Vector3(0, LAnklePitch, 0);

            Vector3 v_h = Vector3.Cross(normalLeftAnklePitch, EgoalNorm);
            float d = Vector3.Dot(Leftlengthwise, v_h) / (Leftlengthwise.Length() * v_h.Length());
            float LAnklePitchNew = (float) Math.Acos(d);

            Vector3 normalLeftAnkleRoll = new Vector3(0, 0, LAnkleRoll);
            float d2 = Vector3.Dot(normalLeftAnkleRoll, EgoalNorm) / (normalLeftAnkleRoll.Length() * EgoalNorm.Length());
            float LAnkleRollNew = (float)Math.Acos(d2);

            // right ankle calculations
            float RAnklePitch = nao.GetData("RAnklePitch");
            float RAnkleRoll = nao.GetData("RAnkleRoll");

            Vector3 Rightlengthwise = new Vector3(RAnklePitch, 0, 0);
            Vector3 normalRightAnklePitch = new Vector3(0, RAnklePitch, 0);

            Vector3 v_h_r = Vector3.Cross(normalRightAnklePitch, EgoalNorm);
            float d_r = Vector3.Dot(Rightlengthwise, v_h_r) / (Rightlengthwise.Length() * v_h_r.Length());
            float RAnklePitchNew = (float)Math.Acos(d_r);

            Vector3 normalRightAnkleRoll = new Vector3(0, 0, RAnkleRoll);
            float d2_r = Vector3.Dot(normalRightAnkleRoll, EgoalNorm) / (normalRightAnkleRoll.Length() * EgoalNorm.Length());
            float RAnkleRollNew = (float)Math.Acos(d2_r);

            // set up own SetAngles call
            ArrayList joints = new ArrayList(new String[] {
                "RAnklePitch",
                "RAnkleRoll",
                "LAnklePitch",
                "LAnkleRoll" });

            double larn2 = (double)(LAnkleRoll + LAnkleRollNew - Math.PI) % Math.PI;
            double lapn2 = (double)(LAnklePitch + LAnklePitchNew) % Math.PI; 
            double rarn2 = (double)(RAnkleRoll + RAnkleRollNew - Math.PI) % Math.PI;
            double rapn2 = (double)(RAnklePitch + RAnklePitchNew) % Math.PI;

            ArrayList angles = new ArrayList(new float[] {
                RAnklePitch + RAnklePitchNew,
                RAnkleRoll - RAnkleRollNew,
                LAnklePitch + LAnklePitchNew,
                LAnkleRoll - LAnkleRollNew });
            angles[0] = RAnklePitchNew;
            angles[1] = RAnkleRollNew;
            angles[2] = LAnklePitchNew;
            angles[3] = LAnkleRollNew;

            nao.SetAngles(joints, angles, speed);
        }
    }
}