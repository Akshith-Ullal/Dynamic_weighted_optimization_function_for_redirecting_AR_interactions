using UnityEngine;
using System.Collections.Generic;
using System;
using System.Linq;

namespace BioIK {
	public class Model {
        //Reference to the character
        //private BioIK Character;
        public BioIK Character;

        //Reference to root
        private BioSegment Root;

		//Offset to world
		private double OPX, OPY, OPZ;								//Offset rosition to world frame
		private double ORX, ORY, ORZ, ORW;							//Offset rotation to world frame
		private double OSX, OSY, OSZ;								//Offset scale to world Frame
        public double maxpos = Double.MinValue;
        public double maxori = Double.MinValue;
		//Linked list of nodes in the model
		public Node[] Nodes = new Node[0];
        

        //Global pointers to the IK setup
        public MotionPtr[] MotionPtrs = new MotionPtr[0];
		public ObjectivePtr[] ObjectivePtrs = new ObjectivePtr[0];

		//Assigned Configuraton
		private double[] Configuration;
		private double[] Gradient;
		private double[] Losses;

		//Simulated Configuration
		private double[] PX,PY,PZ,RX,RY,RZ,RW;
		private double[] SimulatedLosses;
        public double[] refSolution = null;
        public double cosanglescore;
        public static List<config> configpoints = new List<config>();
        public static List<config> refconfigpoints = new List<config>();



        //Degree of Freedom
        private int DoF;
        private bool temp = false;
        private bool temploop = false;
        public static bool priorsec;
        //public bool refconfigset = false;
        private int testcount =0;
        public int tempcount=0;
        double[] vectorArray1;
        double[] vectorArray2;
        public double totalcos;
        public double totalcos_c;
        public static double maxweightg = 0;
        public static double alphag = 0;
        public static double  secondaryweightg = 0;
        public int nodenum = 0;
        public bool final;
        public bool finalfinal = false;
        public double cosloss = 0;
        public static int primaryobjectivelength = 0;
        public static int secondaryobjectivelength = 0;
        static List<int>  primarylist = new List<int>();
        static List<int> secondarylist = new List<int>();





        public Model(BioIK character) {
            //Debug.Log("Model");
			Character = character;

			//Set Root
			Root = Character.FindSegment(Character.transform);
            //Debug.Log(Root.Transform.name);
			//Create Root
			AddNode(Root);
			
			//Build Model
			BioObjective[] objectives = CollectObjectives(Root, new List<BioObjective>());
            //Debug.Log("objective length:" +objectives.Length);
            for (int i=0; i<objectives.Length; i++) {
				List<BioSegment> chain = Character.GetChain(Root, objectives[i].Segment);
                //if (Root.Transform.name == "Root")
                if(Character.cosloss == false)
                //if(finalfinal == true)
                {
                    //Debug.Log(objectives[i].Segment.Transform.name);

                    if (objectives[i].Segment.Transform.name == "Left_Hand_IK" || objectives[i].Segment.Transform.name == "Right_Hand_IK" || objectives[i].Segment.Transform.name == "Left_Foot_IK" || objectives[i].Segment.Transform.name == "Right_Foot_IK" || objectives[i].Segment.Transform.name == "Eyes")
                    {
                        
                        primarylist.Add(i);
                        //primaryobjectivelength++;
                        //Debug.Log("primary objective: " + objectives[i].Segment.Transform.name);
                    }
                    else
                    {
                        secondarylist.Add(i);
                        //secondaryobjectivelength++;
                        //Debug.Log("Secondary objective: " + objectives[i].Segment.Transform.name);

                    }
                }
                //Debug.Log(i);
                // Debug.Log("objective"+objectives[i].Segment.Transform.name);

                //objectives[i].SetWeight(objectives[i].GetWeight() + 1);
                //Debug.Log(objectives[i].GetWeight().ToString());
                //Debug.Log("chain length:" + chain.Count);
                for (int j=1; j<chain.Count; j++) {
					AddNode(chain[j]);
				}
                 
            }
            //if (Root.Transform.name == "Root")
            //if (finalfinal == true)
            //{
            //    Debug.Log("objectives length" + objectives.Length);
            //    Debug.Log("primary length" + primaryobjectivelength);
            //    Debug.Log("secondary length" + secondaryobjectivelength);
            //}
            //Debug.Log("node length:" + Nodes.Length);
            //Assign DoF
            DoF = MotionPtrs.Length; // MotionPtrs are added in the AddNode() function of both root and chain in Model

            //Initialise arrays for single transform modifications
            //Debug.Log("nodes length:" + Nodes.Length);
			for(int i=0; i<Nodes.Length; i++) {
				Nodes[i].ObjectiveImpacts = new bool[ObjectivePtrs.Length];
			}
			PX = new double[ObjectivePtrs.Length];
			PY = new double[ObjectivePtrs.Length];
			PZ = new double[ObjectivePtrs.Length];
			RX = new double[ObjectivePtrs.Length];
			RY = new double[ObjectivePtrs.Length];
			RZ = new double[ObjectivePtrs.Length];
			RW = new double[ObjectivePtrs.Length];
			Configuration = new double[MotionPtrs.Length];
			Gradient = new double[MotionPtrs.Length];
			Losses = new double[ObjectivePtrs.Length];
			SimulatedLosses = new double[ObjectivePtrs.Length];

			//Assigns references to all objective nodes that are affected by a parenting node
			for(int i=0; i<ObjectivePtrs.Length; i++) {
				Node node = ObjectivePtrs[i].Node;
				while(node != null) {
					node.ObjectiveImpacts[i] = true;
					node = node.Parent;
				}
			}

			Refresh();

			//DebugSetup();
		}

		public int GetDoF() {
			return DoF;
		}

		public BioIK GetCharacter() {
			return Character;
		}

		public void Refresh() {
			//Updates configuration
			for(int i=0; i<Configuration.Length; i++) {
				Configuration[i] = MotionPtrs[i].Motion.GetTargetValue(true);
			}

			//Update offset from world to root
			if(Root.Transform.root == Character.transform) {
				OPX = OPY = OPZ = ORX = ORY = ORZ = 0.0;
				ORW = OSX = OSY = OSZ = 1.0;
			} else {
				Vector3 p = Root.Transform.parent.position;
				Quaternion r = Root.Transform.parent.rotation;
				Vector3 s = Root.Transform.parent.lossyScale;
				OPX = p.x; OPY = p.y; OPZ = p.z;
				ORX = r.x; ORY = r.y; ORZ = r.z; ORW = r.w;
				OSX = s.x; OSY = s.y; OSZ = s.z;
			}

			//Updates the nodes
			Nodes[0].Refresh();
		}

		public void CopyFrom(Model model) {
			OPX = model.OPX;
			OPY = model.OPY;
			OPZ = model.OPZ;
			ORX = model.ORX;
			ORY = model.ORY;
			ORZ = model.ORZ;
			ORW = model.ORW;
			OSX = model.OSX;
			OSY = model.OSY;
			OSZ = model.OSZ;
			for(int i=0; i<DoF; i++) {
				Configuration[i] = model.Configuration[i];
				Gradient[i] = model.Gradient[i];
			}
			for(int i=0; i<ObjectivePtrs.Length; i++) {
				PX[i] = model.PX[i];
				PY[i] = model.PY[i];
				PZ[i] = model.PZ[i];
				RX[i] = model.RX[i];
				RY[i] = model.RY[i];
				RZ[i] = model.RZ[i];
				RW[i] = model.RW[i];
				Losses[i] = model.Losses[i];
				SimulatedLosses[i] = model.SimulatedLosses[i];
			}
			for(int i=0; i<Nodes.Length; i++) {
				Nodes[i].WPX = model.Nodes[i].WPX;
				Nodes[i].WPY = model.Nodes[i].WPY;
				Nodes[i].WPZ = model.Nodes[i].WPZ;
				Nodes[i].WRX = model.Nodes[i].WRX;
				Nodes[i].WRY = model.Nodes[i].WRY;
				Nodes[i].WRZ = model.Nodes[i].WRZ;
				Nodes[i].WRW = model.Nodes[i].WRW;
				Nodes[i].WSX = model.Nodes[i].WSX;
				Nodes[i].WSY = model.Nodes[i].WSY;
				Nodes[i].WSZ = model.Nodes[i].WSZ;

				Nodes[i].LPX = model.Nodes[i].LPX;
				Nodes[i].LPY = model.Nodes[i].LPY;
				Nodes[i].LPZ = model.Nodes[i].LPZ;
				Nodes[i].LRX = model.Nodes[i].LRX;
				Nodes[i].LRY = model.Nodes[i].LRY;
				Nodes[i].LRZ = model.Nodes[i].LRZ;
				Nodes[i].LRW = model.Nodes[i].LRW;

				//Nodes[i].RootX = model.Nodes[i].RootX;
				//Nodes[i].RootY = model.Nodes[i].RootY;
				//Nodes[i].RootZ = model.Nodes[i].RootZ;
				Nodes[i].XValue = model.Nodes[i].XValue;
				Nodes[i].YValue = model.Nodes[i].YValue;
				Nodes[i].ZValue = model.Nodes[i].ZValue;
			}
		}

        //Computes the loss as the RMSE over all objectives
        //public double ComputeLoss(double[] configuration) {
        public double ComputeLoss(double[] configuration, double[] refsolution) {
            //         foreach (var item in refsolution.Where(n => n == null))
            //         {
            //             Debug.Log("refsol compute loss = " + String.Join("",
            //new List<double>(refsolution)
            //.ConvertAll(j => j.ToString())
            //.ToArray()));
            //             Debug.Log("calling method"+(new System.Diagnostics.StackTrace()).GetFrame(1).GetMethod().Name);
            //         }


            //if(Character.cosloss == false)
            // {
            //     Debug.Log("wassup");
            // }
            //double before = L2Norm(refsolution);

            //FK(configuration);
            //configpoints.Clear();
            //refconfigpoints.Clear();
            
            if(final == true)
            {
                FKmod(refsolution, true);
               
            }
           
            FKmod(configuration,false);
            //Debug.Log("config length:" + configpoints.Count);
            //if (finalfinal == true && configpoints.Count !=0 && refconfigpoints.Count != 0)
           // if (configpoints.Count == 26 && refconfigpoints.Count != 0)
            //if (finalfinal == true)
            //{
               

                //Debug.Log("config length:" + configpoints.Count);
                //Debug.Log("reconfig length:" + refconfigpoints.Count);
                //cosloss = 0;
                //for (int i = 0; i < configpoints.Count; i++)
                //{
                    
                      
                    
                   // Vector3 configroot = new Vector3((float)configpoints[0].X, (float)configpoints[0].Y, (float)configpoints[0].Z);
                   // Vector3 refconfigroot = new Vector3((float)refconfigpoints[0].X, (float)refconfigpoints[0].Y, (float)refconfigpoints[0].Z);
                    //Vector3 configtemp = new Vector3((float)configpoints[i].X, (float)configpoints[i].Y, (float)configpoints[i].Z);
                    //Vector3 refconfigtemp = new Vector3((float)refconfigpoints[i].X, (float)refconfigpoints[i].Y, (float)refconfigpoints[i].Z);
                    //cosloss = cosloss + configpoints[i].Weight * (Math.Abs((configtemp - configroot).magnitude - (refconfigtemp - refconfigroot).magnitude));
                    //cosloss = cosloss + configpoints[i].Weight*(Vector3.Distance(refconfigroot, configroot));
                    //cosloss = cosloss + (Vector3.Distance(refconfigroot, configroot));
                    // Debug.Log("index: " + i+"value: " + Vector3.Distance(refconfigtemp, configtemp));

                //}
               // Debug.Log("cosloss:" + cosloss);
                //Debug.Log("weight:" + configpoints[0].Weight);
           // }
            //FKmod(refsolution, true);
            
            //Character.testv = new Vector3((float)refconfigpoints[0].X, (float)refconfigpoints[0].Y,(float)refconfigpoints[0].Z);
            ////Debug.Log("X:" + (float)refconfigpoints[1].X + "Y:" + (float)refconfigpoints[1].Y + "Z:"+ (float)refconfigpoints[1].Z);
            //if (configpoints.Count > 0)
            //{
            //    // Debug.Log("X:" + (float)refconfigpoints[1].X + "Y:" + (float)refconfigpoints[1].Y + "Z:"+ (float)refconfigpoints[1].Z);
            //   // Debug.Log("X:" + (float)configpoints[0].X + "Y:" + (float)configpoints[0].Y + "Z:" + (float)configpoints[0].Z);
            //    //Debug.Log("wassup");
            //}
            //Debug.Log("final node num:" +nodenum);
            //nodenum = 0;
            //Debug.Log("no of elements config:" + configpoints.Count);
            //Debug.Log("no of elements refconfig:" + refconfigpoints.Count);

            double loss = 0.0;
            double primaryloss = 0.0;
            double secondaryloss = 0.0;
             double beta = 1;
            bool flip = false;
        //double cosinesim = 0.0;
        //double maxweight = 10.0;

        double cosinesimnum = 0.0;
            double refvectomag = 0.0;
            //double[] refvectarray = new double[ObjectivePtrs.Length * 3];
            double curvectomag = 0.0;
            for (int i = 0; i < ObjectivePtrs.Length; i++)
            {
                //Debug.Log(ObjectivePtrs.Length);
                Node node = ObjectivePtrs[i].Node;
                Node rootnode = ObjectivePtrs[0].Node;

                //Debug.Log(ObjectivePtrs[i].Node.Transform.name);
                //if (ObjectivePtrs[i].Objective.GetObjectiveType().ToString() == "Position")
                //{
                //if (!temp)
                //{
                //    for (int j = 0; j < ObjectivePtrs.Length; j++)
                //    {

                //        Node nodeo = ObjectivePtrs[j].Node;

                //        if (!(double.IsNaN(nodeo.WPX * nodeo.WPX + nodeo.WPY * nodeo.WPY + nodeo.WPZ * nodeo.WPZ)))
                //        {
                //            refvectomag = refvectomag + nodeo.WPX * nodeo.WPX + nodeo.WPY * nodeo.WPY + nodeo.WPZ * nodeo.WPZ;
                //            refvectarray[j * 3] = nodeo.WPX;
                //            refvectarray[(j * 3) + 1] = nodeo.WPY;
                //            refvectarray[(j * 3) + 2] = nodeo.WPZ;

                //        }
                //    }
                //    refvectomag = System.Math.Pow(refvectomag, 1.0 / (ObjectivePtrs.Length * 3));

                // Debug.Log(node.Transform.position);
                //    temp = true;
                //}
                //}
                //if (ObjectivePtrs[i].Objective.GetObjectiveType().ToString() == "Displacement")
                //{
                //     Debug.Log("x disp:" + node.WPX);
                //}
                //if (ObjectivePtrs[i].Objective.GetObjectiveType().ToString() == "Position")
                //{
                //    Debug.Log("x pos:" + node.WPX);
                //}
                //if (!(double.IsNaN(node.WPX * node.WPX + node.WPY * node.WPY + node.WPZ * node.WPZ)))
                //{
                // Debug.Log("number:" + i);
                //Debug.Log("y:" + node.WPY);
                //Debug.Log("z:" + node.WPZ);

                //if (!temp)
                //    {
                //        refvectarray[i * 3] = node.WPX;
                //        refvectarray[(i * 3) + 1] = node.WPY;
                //        refvectarray[(i * 3) + 2] = node.WPZ;

                //    }
                //    curvectomag = curvectomag + node.WPX * node.WPX + node.WPY * node.WPY + node.WPZ * node.WPZ;
                //    cosinesimnum = cosinesimnum + refvectarray[i * 3] * node.WPX + refvectarray[(i * 3) + 1] * node.WPY + refvectarray[(i * 3) + 2] * node.WPZ;
                //}
                //if (ObjectivePtrs[i].Objective.GetObjectiveType().ToString() == "Position")
                //{
                //    Losses[i] = ObjectivePtrs[i].Objective.ComputeLoss(node.WPX, node.WPY, node.WPZ, node.WRX, node.WRY, node.WRZ, node.WRW, node, Configuration);
                //    //if (Losses[i] > maxpos)
                //    //{
                //    //    maxpos = Losses[i];
                //    //}
                //    //Debug.Log("pos loss:" + Losses[i]);
                //    loss += Losses[i];
                //}
                //else if(ObjectivePtrs[i].Objective.GetObjectiveType().ToString() == "Orientation")
                //{

                //    Losses[i] = ObjectivePtrs[i].Objective.ComputeLoss(node.WPX, node.WPY, node.WPZ, node.WRX, node.WRY, node.WRZ, node.WRW, node, Configuration);
                //    if (Losses[i] > maxori)
                //    {
                //        maxori = Losses[i];
                //    }
                //    loss += Losses[i];

                //}
                //else
                {
                    //Losses[i] = ObjectivePtrs[i].Objective.ComputeLoss(node.WPX, node.WPY, node.WPZ, node.WRX, node.WRY, node.WRZ, node.WRW, node, Configuration);
                    if (primarylist.Contains(i))
                    {
                        priorsec = true;
                    }
                    else
                    {
                        priorsec = false;
                    }
                    //double alpha = 5.0;
                    //double secondaryweight = maxweight - alpha;
                    //maxweightg = maxweight;
                    //alphag = Character.primaryweight;
                    //secondaryweightg = Character.secondaryweight;
                    alphag = Character.primaryweight;
                    secondaryweightg = Character.secondaryweight;
                    
                    Losses[i] = ObjectivePtrs[i].Objective.ComputeLoss(node.WPX, node.WPY, node.WPZ, node.WRX, node.WRY, node.WRZ, node.WRW, node, Configuration, rootnode.WPX, rootnode.WPY, rootnode.WPZ, rootnode.WRX, rootnode.WRY, rootnode.WRZ, rootnode.WRW, priorsec,alphag,secondaryweightg);
                    //if (final == true)
                    //{
                    //    if (i == 5)
                    //    {


                    //        if (Character.cosloss != true)
                    //        {

                    //            Debug.Log("left hand loss_c:" + Math.Sqrt(Losses[i]) / 10);
                    //        }
                    //        else
                    //        {
                    //            Debug.Log("left hand loss:" + Math.Sqrt(Losses[i]) / 10);
                    //        }
                    //    }
                    //}

                    //if (Root.Transform.name == "Root")
                    //if(finalfinal == true)
                    //if (Character.cosloss != true)
                    //{    
                    if (primarylist.Contains(i))
                    //    if (ObjectivePtrs[i].Objective.transform.name == "Left_Hand_IK" || ObjectivePtrs[i].Objective.transform.name == "Right_Hand_IK" || ObjectivePtrs[i].Objective.transform.name == "Left_Foot_IK" || ObjectivePtrs[i].Objective.transform.name == "Right_Foot_IK" || ObjectivePtrs[i].Objective.transform.name == "Eyes")
                    {
                        primaryloss += Losses[i];
                    }
                    else
                    {
                        secondaryloss += Losses[i];
                    }
                    //maxweightg = 10;
                    //alphag = 9;

                    //    // alpha = 1 / (1 + Math.Pow(2.71828, -beta * (primaryloss / primaryobjectivelength)));
                    //if (primaryobjectivelength != 0)
                    //{
                    //    alphag = maxweightg / (1 + Math.Pow(2.71828, -beta * (primaryloss / primaryobjectivelength)));
                    //}
                    //else
                    //{
                    //    alphag = maxweightg / (1 + Math.Pow(2.71828, -beta * 0));
                    //}
                    //System.Random rnd = new System.Random();
                    //alphag = rnd.Next(1, 10000);

                    //alphag = maxweightg / (1 + Math.Pow(2.71828, -0.25*rnd.Next(1, 20)));
                    //if (flip)
                    //{
                    //    alphag = 100000;
                    //    flip = !flip;
                    //}
                    //else
                    //{
                    //    alphag = 0;
                    //    flip = !flip;
                    //}
                   // secondaryweightg = maxweightg - alphag;
                    //alpha = 0.5;
                    //}
                    //else
                    {
                        loss += Losses[i];
                    }
                    //Debug.Log("loss:" + loss);
                }
            }
            if (Character.cosloss != true)
            {
                //cosanglescore = Score(refsolution, configuration);
                ////Debug.Log("max pos:" +maxpos);
                ////Debug.Log("max ori:" +maxori);

               // if (!(Double.IsNaN(cosanglescore) || cosanglescore == 0))
                //{
                    //Debug.Log("loss before:" + loss);
                    //Debug.Log("cosanglescore:" + (cosanglescore*10000)+ " loss:" +loss);
                   // loss += (Math.Abs(1/(cosanglescore*10000000)));
                    //Debug.Log("loss:" + System.Math.Sqrt(loss / ((double)ObjectivePtrs.Length) + 1));
                    //return System.Math.Sqrt(loss / ((double)ObjectivePtrs.Length)+1);
               // }
            }
            //if (Character.cosloss == true)
            //{

            //}


            //if (!temp)
            //{
            //    refvectomag = curvectomag;
            //    refvectomag = System.Math.Pow(refvectomag, 1.0 / (ObjectivePtrs.Length * 3));
            //    temp = true;

            //}
            //curvectomag = System.Math.Pow(curvectomag, 1.0 / (ObjectivePtrs.Length * 3));
            //cosinesim = cosinesimnum / (curvectomag * refvectomag);
            //if (!(double.IsNaN(loss)))
            //{
            //    Debug.Log("loss:" + System.Math.Sqrt(loss / (double)ObjectivePtrs.Length));
            //}

            //Debug.Log("loss:" +loss);
            //Debug.Log("loss:" + System.Math.Sqrt(loss / (double)ObjectivePtrs.Length));
            //Debug.Log("sol diff:" + (before-L2Norm(refsolution)));
            //if (Root.Transform.name == "Root")
            //if(finalfinal == true)
            //if (Character.cosloss != true)
            {
                // Debug.Log("primary loss:" + primaryloss);
                //Debug.Log("primary objective length:" + primaryobjectivelength);
                //Debug.Log("secondary objective length:" +secondaryobjectivelength);
                //alpha = 5;
                // alpha = maxweight / (1 + Math.Pow(2.71828, -beta * (primaryloss / primaryobjectivelength)));
                // Debug.Log("alpha:" + alpha);
                //return System.Math.Sqrt((alpha * primaryloss + (maxweight - alpha) * secondaryloss) / (double)ObjectivePtrs.Length);
                //return System.Math.Sqrt(( primaryloss + ( secondaryloss)) / (double)ObjectivePtrs.Length);
                return System.Math.Sqrt(loss / (double)ObjectivePtrs.Length);
            }
            //else
            //{
            //    return System.Math.Sqrt(loss / (double)ObjectivePtrs.Length);
            //}

        }

        private double Score(double[] vectorArray1, double[] vectorArray2)
        {
            List<double> v1 = new List<double>(vectorArray1);
            List<double> v2 = new List<double>(vectorArray2);

            if (v1.Count == 0 || v2.Count == 0)
            {
                //throw new System.ArgumentException("Cosine Similarity: Vectors cannot be zero length");
            }
            if (v1.Count != v2.Count)
            {
               // throw new System.ArgumentException("Cosine Similarity: Vectors must be the same length");
            }

            double dotProduct = 0.0;
            double sumsq1 = 0;
            for (int i = 0; i < v1.Count; i++)
            {
                sumsq1 += v1[i] * v1[i];
            }
            double l2norm1 = System.Math.Sqrt(sumsq1);
            //Debug.Log("refsol norm:" + l2norm1);

            double sumsq2 = 0;
            for (int i = 0; i < v2.Count; i++)
            {
                sumsq2 += v2[i] * v2[i];
            }
            double l2norm2 = System.Math.Sqrt(sumsq2);
            //Debug.Log("sol norm:" + l2norm2);


            if (l2norm1 == 0 || l2norm2 == 0)
            {
               // throw new System.ArgumentException("Cosine Similarity: A vector cannot be a zero vector");
            }
            for (int i = 0; i < v1.Count; i++)
            {
                dotProduct += v1[i] * v2[i];
            }
            return dotProduct / (l2norm1 * l2norm2);
        }

        public static double L2Norm(List<double> v)
        {
            double sumsq = 0;
            for (int i = 0; i < v.Count; i++)
            {
                sumsq += v[i] * v[i];
            }
            return System.Math.Sqrt(sumsq);
        }

        public static double L2Norm(double[] vArray)
        {
            List<double> v = new List<double>(vArray);
            return L2Norm(v);
        }

      

        //Computes the gradient
        public double[] ComputeGradient(double[] configuration,double[] refSolution, double resolution) {
            //double oldLoss = ComputeLoss(configuration); //objectivefunctionupdaterequired
            // Debug.Log("gradient calc");
            //Debug.Log("refsol beforeloss model.cs 379  = " + String.Join("",
            // new List<double>(refSolution)
            // .ConvertAll(i => i.ToString())
            // .ToArray()));
            //Debug.Log("calling method" + (new System.Diagnostics.StackTrace()).GetFrame(1).GetMethod().Name);
            double oldLoss = ComputeLoss(configuration,refSolution);
            for (int j=0; j<DoF; j++) {
				Configuration[j] += resolution;
				MotionPtrs[j].Node.SimulateModification(Configuration);
				Configuration[j] -= resolution;
				double newLoss = 0.0;
				for(int i=0; i<ObjectivePtrs.Length; i++) {
					newLoss += SimulatedLosses[i];
				}
				newLoss = System.Math.Sqrt(newLoss / (double)ObjectivePtrs.Length);
				Gradient[j] = (newLoss - oldLoss) / resolution;
			}
			return Gradient;
		}

		//Returns whether the model converges for a particular configuration
		public bool CheckConvergence(double[] configuration) {
			FK(configuration);
			for(int i=0; i<ObjectivePtrs.Length; i++) {
				Model.Node node = ObjectivePtrs[i].Node;
				if(!ObjectivePtrs[i].Objective.CheckConvergence(node.WPX, node.WPY, node.WPZ, node.WRX, node.WRY, node.WRZ, node.WRW, node, configuration)) {
					return false;
				}
			}
			return true;
		}

        //Applies a forward kinematics pass to the model

        private void FK(double[] configuration)
        {
            for (int i = 0; i < Configuration.Length; i++)
            {
                Configuration[i] = configuration[i];
            }
            Nodes[0].FeedForwardConfiguration(configuration,false);
        }

        public void FKmod(double[] configuration, bool status)
        {
            if (status == true)
            {
               
                refconfigpoints.Clear();
                
                //nodenum = 0;

            }
            configpoints.Clear();
            for (int i = 0; i < Configuration.Length; i++)
            {
                Configuration[i] = configuration[i];
            }
            Nodes[0].FeedForwardConfiguration(configuration,status);
            //if (status == true)
            //{

               
            //    Debug.Log("refconfig length:" + refconfigpoints.Count);

            //    //nodenum = 0;

            //}
            
        }

        //Adds a segment node into the model
        private void AddNode(BioSegment segment) {
			if(FindNode(segment.Transform) == null) {
				Node node = new Node(this, FindNode(segment.Transform.parent), segment);

				if(node.Joint != null) {
					if(node.Joint.GetDoF() == 0 || !node.Joint.enabled) {
						node.Joint = null;
					} else {
						if(node.Joint.X.IsEnabled()) {
							MotionPtr motionPtr = new MotionPtr(node.Joint.X, node, MotionPtrs.Length);
							System.Array.Resize(ref MotionPtrs, MotionPtrs.Length+1);
							MotionPtrs[MotionPtrs.Length-1] = motionPtr;
							node.XEnabled = true;
							node.XIndex = motionPtr.Index;
						}
						if(node.Joint.Y.IsEnabled()) {
							MotionPtr motionPtr = new MotionPtr(node.Joint.Y, node, MotionPtrs.Length);
							System.Array.Resize(ref MotionPtrs, MotionPtrs.Length+1);
							MotionPtrs[MotionPtrs.Length-1] = motionPtr;
							node.YEnabled = true;
							node.YIndex = motionPtr.Index;
						}
						if(node.Joint.Z.IsEnabled()) {
							MotionPtr motionPtr = new MotionPtr(node.Joint.Z, node, MotionPtrs.Length);
							System.Array.Resize(ref MotionPtrs, MotionPtrs.Length+1);
							MotionPtrs[MotionPtrs.Length-1] = motionPtr;
							node.ZEnabled = true;
							node.ZIndex = motionPtr.Index;
						}
					}
				}

				BioObjective[] objectives = segment.Objectives;
				for(int i=0; i<objectives.Length; i++) {
					if(objectives[i].enabled) {
						System.Array.Resize(ref ObjectivePtrs, ObjectivePtrs.Length+1);
						ObjectivePtrs[ObjectivePtrs.Length-1] = new ObjectivePtr(objectives[i], node, ObjectivePtrs.Length);
					}
				}

				System.Array.Resize(ref Nodes, Nodes.Length+1);
                //if (node.Transform.name == "Root")
                //{
                //    node.weight = 1;
                //}
                //else if (node.Transform.name == "Hip")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Thigh_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Knee_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Ankle_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Toe_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Foot_Ik")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Thigh_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Knee_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Ankle_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Toe_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Foot_Ik")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Ribs")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Shoulder_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Upper_Arm_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Forearm_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Wrist_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Index_Finger_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Index_Finger_Joint_01b")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Index_Finger_Joint_01c")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Middle_Finger_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Middle_Finger_Joint_01b")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Middle_Finger_Joint_01c")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Pinky_Finger_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Pinky_Finger_Joint_01b")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Pinky_Finger_Joint_01c")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Ring_Finger_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Ring_Finger_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Ring_Finger_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Thumb_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Thumb_Joint_01b")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Left_Hand_IK")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Neck")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Head")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Eyes")
                //{
                //    node.weight =0;
                //}
                //else if (node.Transform.name == "Right_Shoulder_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Upper_Arm_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Forearm_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Wrist_Joint_01")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Index_Finger_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Index_Finger_Joint_01b")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Index_Finger_Joint_01c")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Middle_Finger_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Middle_Finger_Joint_01b")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Middle_Finger_Joint_01c")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Pinky_Finger_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Pinky_Finger_Joint_01b")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Pinky_Finger_Joint_01c")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Ring_Finger_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Ring_Finger_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Ring_Finger_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Thumb_Joint_01a")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Thumb_Joint_01b")
                //{
                //    node.weight = 0;
                //}
                //else if (node.Transform.name == "Right_Hand_IK")
                //{
                //    node.weight = 0;
                //}
                Nodes[Nodes.Length-1] = node;
                //Debug.Log("index:" + Nodes.Length);
                //Debug.Log("transform position:" +segment.Transform.position);
            }
		}

		//Returns all objectives which are childs in the hierarcy, beginning from the root
		private BioObjective[] CollectObjectives(BioSegment segment, List<BioObjective> objectives) {
			for(int i=0; i<segment.Objectives.Length; i++) {
				if(segment.Objectives[i].enabled) {
					objectives.Add(segment.Objectives[i]);
				}
			}
			for(int i=0; i<segment.Childs.Length; i++) {
				CollectObjectives(segment.Childs[i], objectives);
			}
			return objectives.ToArray();
		}

		//Returns a node in the model
		private Node FindNode(Transform t) {
			for(int i=0; i<Nodes.Length; i++) {
				if(Nodes[i].Transform == t) {
					return Nodes[i];
				}
			}
			return null;
		}

		//Returns the pointer to the motion
		public MotionPtr FindMotionPtr(BioJoint.Motion motion) {
			for(int i=0; i<MotionPtrs.Length; i++) {
				if(MotionPtrs[i].Motion == motion) {
					return MotionPtrs[i];
				}
			}
			return null;
		}

		//Returns the pointer to the objective
		public ObjectivePtr FindObjectivePtr(BioObjective objective) {
			for(int i=0; i<ObjectivePtrs.Length; i++) {
				if(ObjectivePtrs[i].Objective == objective) {
					return ObjectivePtrs[i];
				}
			}
			return null;
		}

		//Subclass representing the single nodes for the OFKT data structure.
		//Values are stored using primitive data types for faster access and efficient computation.
		public class Node {
			public Model Model;							//Reference to the kinematic model
			public Node Parent;							//Reference to the parent of this node
			public Node[] Childs = new Node[0];			//Reference to all child nodes
			public Transform Transform;					//Reference to the transform
			public BioJoint Joint;						//Reference to the joint
			public Transform[] Chain;
            public bool Weight;
            public double alpha;

			public double WPX, WPY, WPZ;				//World position
			public double WRX, WRY, WRZ, WRW;			//World rotation
			public double WSX, WSY, WSZ;				//World scale
			public double LPX, LPY, LPZ;				//Local position
			public double LRX, LRY, LRZ, LRW;			//Local rotation
			//public double RootX, RootY, RootZ;		//World position of root joint

			public bool XEnabled = false;
			public bool YEnabled = false;
			public bool ZEnabled = false;
			public int XIndex = -1;
			public int YIndex = -1;
			public int ZIndex = -1;
			public double XValue = 0.0;					//
			public double YValue = 0.0;					//
			public double ZValue = 0.0;					//
            public double weight;
		
			public bool[] ObjectiveImpacts;				//Boolean values to represent which objective indices in the whole kinematic tree are affected (TODO: Refactor this)

			//Setup for the node
			public Node(Model model, Node parent, BioSegment segment) {
				Model = model;
				Parent = parent;
				if(Parent != null) {
					Parent.AddChild(this);
				}
				Transform = segment.Transform;
				Joint = segment.Joint;

				List<Transform> reverseChain = new List<Transform>();
				reverseChain.Add(Transform);
				Node p = parent;
				while(p != null) {
					reverseChain.Add(p.Transform);
					p = p.Parent;
				}
				reverseChain.Reverse();
				Chain = reverseChain.ToArray();
			}

			//Adds a child to this node
			public void AddChild(Node child) {
				System.Array.Resize(ref Childs, Childs.Length+1);
				Childs[Childs.Length-1] = child;
			}

			//Recursively refreshes the current transform data
			public void Refresh() {
				//Local
				if(Joint == null) {
					Vector3 lp = Transform.localPosition;
					Quaternion lr = Transform.localRotation;
					LPX = lp.x;
					LPY = lp.y;
					LPZ = lp.z;
					LRX = lr.x;
					LRY = lr.y;
					LRZ = lr.z;
					LRW = lr.w;
				} else {
					XValue = Joint.X.GetTargetValue(true);
					YValue = Joint.Y.GetTargetValue(true);
					ZValue = Joint.Z.GetTargetValue(true);
					Joint.ComputeLocalTransformation(XValue, YValue, ZValue, out LPX, out LPY, out LPZ, out LRX, out LRY, out LRZ, out LRW);
				}
				Vector3 ws = Transform.lossyScale;
				WSX = ws.x;
				WSY = ws.y;
				WSZ = ws.z;

				//World
				ComputeWorldTransformation(true,false);

				//Feed Forward
				foreach(Node child in Childs) {
					child.Refresh();
				}
			}

			//Updates local and world transform, and feeds the joint variable configuration forward to all childs
			public void FeedForwardConfiguration(double[] configuration, bool status,bool updateWorld = false) {
				//Assume no local update is required
				bool updateLocal = false;

				if(XEnabled && configuration[XIndex] != XValue) {
					XValue = configuration[XIndex];
					updateLocal = true;
				}
				if(YEnabled && configuration[YIndex] != YValue) {
					YValue = configuration[YIndex];
					updateLocal = true;
				}
				if(ZEnabled && configuration[ZIndex] != ZValue) {
					ZValue = configuration[ZIndex];
					updateLocal = true;
				}
				
				//Only update local transformation if a joint value has changed
				if(updateLocal) {
					Joint.ComputeLocalTransformation(XValue, YValue, ZValue, out LPX, out LPY, out LPZ, out LRX, out LRY, out LRZ, out LRW);
					updateWorld = true;
				}
                if (status == true)
                {
                    //Model.nodenum++;

                    config refconfigtemp = new config(WRX, WRY, WRZ, WRW, WPX, WPY, WPZ,weight);
                    refconfigpoints.Add(refconfigtemp);
                }
                config configtemp = new config(WRX, WRY, WRZ, WRW, WPX, WPY, WPZ,weight);
                configpoints.Add(configtemp);
                //Only update world transformation if local transformation (in this or parent node) has changed
                if (updateWorld) {
					ComputeWorldTransformation(false,status);
				}
                //Model.nodenum++;
               
                //Feed forward the joint variable configuration
                foreach (Node child in Childs) {
                   
                    child.FeedForwardConfiguration(configuration, status,updateWorld);
				}
                //if (status == true)
                //{
                //    //Debug.Log("reconfig length:" + refconfigpoints.Count);
                //    Debug.Log("X:" + Model.Nodes[0].WPX + "Y:" + Model.Nodes[0].WPY + "Z:" + Model.Nodes[0].WPZ);
                   
                //}


            }

			//Simulates a single transform modification while leaving the whole data structure unchanged
			//Returns the resulting Cartesian posture transformations in the out values
			public void SimulateModification(
				double[] configuration
			) {
				double[] px=Model.PX; double[] py=Model.PY; double[] pz=Model.PZ;
				double[] rx=Model.RX; double[] ry=Model.RY; double[] rz=Model.RZ; double[] rw=Model.RW;
				for(int i=0; i<Model.ObjectivePtrs.Length; i++) {
					Node node = Model.ObjectivePtrs[i].Node;
					if(ObjectiveImpacts[i]) {
						//WorldPosition = ParentPosition + ParentRotation * (LocalPosition . ParentScale) + ParentRotation * LocalRotation * WorldRotation^-1 * (ObjectivePosition - WorldPosition)
						//WorldRotation = ParentRotation * LocalRotation * WorldRotation^-1 * ObjectiveRotation
						double lpX, lpY, lpZ, lrX, lrY, lrZ, lrW;
						Joint.ComputeLocalTransformation(
							XEnabled ? configuration[XIndex] : XValue,
							YEnabled ? configuration[YIndex] : YValue, 
							ZEnabled ? configuration[ZIndex] : ZValue, 
							out lpX, out lpY, out lpZ, out lrX, out lrY, out lrZ, out lrW
						);
						double Rx, Ry, Rz, Rw, X, Y, Z;
						if(Parent == null) {
							px[i] = Model.OPX;
							py[i] = Model.OPY;
							pz[i] = Model.OPZ;
							Rx = Model.ORX;
							Ry = Model.ORY;
							Rz = Model.ORZ;
							Rw = Model.ORW;
							X = Model.OSX*lpX;
							Y = Model.OSY*lpY;
							Z = Model.OSZ*lpZ;
						} else {
							px[i] = Parent.WPX;
							py[i] = Parent.WPY;
							pz[i] = Parent.WPZ;
							Rx = Parent.WRX;
							Ry = Parent.WRY;
							Rz = Parent.WRZ;
							Rw = Parent.WRW;
							X = Parent.WSX*lpX;
							Y = Parent.WSY*lpY;
							Z = Parent.WSZ*lpZ;
						}
						double qx = Rx * lrW + Ry * lrZ - Rz * lrY + Rw * lrX;
						double qy = -Rx * lrZ + Ry * lrW + Rz * lrX + Rw * lrY;
						double qz = Rx * lrY - Ry * lrX + Rz * lrW + Rw * lrZ;
						double qw = -Rx * lrX - Ry * lrY - Rz * lrZ + Rw * lrW;
						double dot = WRX*WRX + WRY*WRY + WRZ*WRZ + WRW*WRW;
						double x = qx/dot; double y = qy/dot; double z = qz/dot; double w = qw/dot;
						qx = x * WRW + y * -WRZ - z * -WRY + w * -WRX;
						qy = -x * -WRZ + y * WRW + z * -WRX + w * -WRY;
						qz = x * -WRY - y * -WRX + z * WRW + w * -WRZ;
						qw = -x * -WRX - y * -WRY - z * -WRZ + w * WRW;
						px[i] +=
								+ 2.0 * ((0.5 - Ry * Ry - Rz * Rz) * X + (Rx * Ry - Rw * Rz) * Y + (Rx * Rz + Rw * Ry) * Z)
								+ 2.0 * ((0.5 - qy * qy - qz * qz) * (node.WPX-WPX) + (qx * qy - qw * qz) * (node.WPY-WPY) + (qx * qz + qw * qy) * (node.WPZ-WPZ));
						py[i] += 
								+ 2.0 * ((Rx * Ry + Rw * Rz) * X + (0.5 - Rx * Rx - Rz * Rz) * Y + (Ry * Rz - Rw * Rx) * Z)
								+ 2.0 * ((qx * qy + qw * qz) * (node.WPX-WPX) + (0.5 - qx * qx - qz * qz) * (node.WPY-WPY) + (qy * qz - qw * qx) * (node.WPZ-WPZ));
						pz[i] += 
								+ 2.0 * ((Rx * Rz - Rw * Ry) * X + (Ry * Rz + Rw * Rx) * Y + (0.5 - (Rx * Rx + Ry * Ry)) * Z)
								+ 2.0 * ((qx * qz - qw * qy) * (node.WPX-WPX) + (qy * qz + qw * qx) * (node.WPY-WPY) + (0.5 - qx * qx - qy * qy) * (node.WPZ-WPZ));
						rx[i] = qx * node.WRW + qy * node.WRZ - qz * node.WRY + qw * node.WRX;
						ry[i] = -qx * node.WRZ + qy * node.WRW + qz * node.WRX + qw * node.WRY;
						rz[i] = qx * node.WRY - qy * node.WRX + qz * node.WRW + qw * node.WRZ;
						rw[i] = -qx * node.WRX - qy * node.WRY - qz * node.WRZ + qw * node.WRW;
                        //Model.SimulatedLosses[i] = Model.ObjectivePtrs[i].Objective.ComputeLoss(px[i], py[i], pz[i], rx[i], ry[i], rz[i], rw[i], node, configuration);
                        //if (Model.Character.Root.Transform.name == "Root")
                        //{
                        //    Weight = false;
                        //}
                        //else
                        //{
                        //    Weight = true;
                        //}
                        if (primarylist.Contains(i))
                        {
                            priorsec = true;
                        }
                        else
                        {
                            priorsec = false;
                        }
                        //double alpha = 2.874;
                        //double secondaryweight = maxweightg - 5.328;
                        Model.SimulatedLosses[i] = Model.ObjectivePtrs[i].Objective.ComputeLoss(px[i], py[i], pz[i], rx[i], ry[i], rz[i], rw[i], node, configuration, px[0], py[0], pz[0], rx[0], ry[0], rz[0], rw[0],priorsec,alphag,secondaryweightg);
                    } else {
						px[i] = node.WPX;
						py[i] = node.WPY;
						pz[i] = node.WPZ;
						rx[i] = node.WRX;
						ry[i] = node.WRY;
						rz[i] = node.WRZ;
						rw[i] = node.WRW;
						Model.SimulatedLosses[i] = Model.Losses[i];
					}
				}
			}

			//Computes the world transformation using the current joint variable configuration
			private void ComputeWorldTransformation(bool refresh, bool status) {
				//WorldPosition = ParentPosition + ParentRotation*LocalPosition;
				//WorldRotation = ParentRotation*LocalRotation;
				double RX,RY,RZ,RW,X,Y,Z;
				if(Parent == null) {
					WPX = Model.OPX;
					WPY = Model.OPY;
					WPZ = Model.OPZ;
					RX = Model.ORX;
					RY = Model.ORY;
					RZ = Model.ORZ;
					RW = Model.ORW;
					X = Model.OSX*LPX;
					Y = Model.OSY*LPY;
					Z = Model.OSZ*LPZ;
				} else {
					WPX = Parent.WPX;
					WPY = Parent.WPY;
					WPZ = Parent.WPZ;
					RX = Parent.WRX;
					RY = Parent.WRY;
					RZ = Parent.WRZ;
					RW = Parent.WRW;
					X = Parent.WSX*LPX;
					Y = Parent.WSY*LPY;
					Z = Parent.WSZ*LPZ;
				}
				WPX += 2.0 * ((0.5 - RY * RY - RZ * RZ) * X + (RX * RY - RW * RZ) * Y + (RX * RZ + RW * RY) * Z);
				WPY += 2.0 * ((RX * RY + RW * RZ) * X + (0.5 - RX * RX - RZ * RZ) * Y + (RY * RZ - RW * RX) * Z);
				WPZ += 2.0 * ((RX * RZ - RW * RY) * X + (RY * RZ + RW * RX) * Y + (0.5 - RX * RX - RY * RY) * Z);
				WRX = RX * LRW + RY * LRZ - RZ * LRY + RW * LRX;
				WRY = -RX * LRZ + RY * LRW + RZ * LRX + RW * LRY;
				WRZ = RX * LRY - RY * LRX + RZ * LRW + RW * LRZ;
				WRW = -RX * LRX - RY * LRY - RZ * LRZ + RW * LRW;

                //if (!refresh)
                //{

                //    if (status)
                //    {

                //        config refconfigtemp = new config(WRX, WRY, WRZ, WRW, WPX, WPY, WPZ);
                //        refconfigpoints.Add(refconfigtemp);
                //        //if (refconfigpoints.Count > 1 && refconfigpoints.Count < 3)
                //        ////if (refconfigpoints.Count < 2)
                //        //{
                //        //    Debug.Log("X:" + refconfigtemp.X + "Y:" + refconfigtemp.Y + "Z:" + refconfigtemp.Z);
                //        //    //Model.Character.testv = new Vector3((float)refconfigtemp.X, (float)refconfigtemp.Y, (float)refconfigtemp.Z);
                //        //}
                //    }
                //    else
                //    {
                //        //config configtemp = new config(WRX, WRY, WRZ, WRW, WPX, WPY, WPZ);
                //        //configpoints.Add(configtemp);
                //if (configpoints.Count < 2)
                //{
                //    Debug.Log("X:" + configtemp.X + "Y:" + configtemp.Y + "Z:" + configtemp.Z);
                //}
                //    }
                //}
            }
        }

		//Data class to store pointers to the objectives
		public class ObjectivePtr {
			public BioObjective Objective;
			public Node Node;
			public int Index;
			public ObjectivePtr(BioObjective objective, Node node, int index) {
				Objective = objective;
				Node = node;
				Index = index;
			}
		}

        //Data class to store pointers to the joint motions
        public class MotionPtr
        {
            public BioJoint.Motion Motion;
            public Node Node;
            public int Index;
            public MotionPtr(BioJoint.Motion motion, Node node, int index)
            {
                Motion = motion;
                Node = node;
                Index = index;
            }
        }

            public class config
            {
                public double RX, RY, RZ, RW, X, Y, Z, Weight;
                public config(double rx,double ry,double rz,double rw, double x, double y, double z, double weight)
                {
                    RX = rx;
                    RY = ry;
                    RZ = rz;
                    RW = rw;
                    X = x;
                    Y = y;
                    Z = z;
                Weight = weight;
                }
            }

            

        
	}
}