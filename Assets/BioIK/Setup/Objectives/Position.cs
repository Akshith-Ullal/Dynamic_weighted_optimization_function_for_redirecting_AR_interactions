using UnityEngine;
using System.Collections.Generic;
using System;

namespace BioIK {

	//This objective aims to minimise the translational distance between the transform and the target.
	[AddComponentMenu("")]
	public class Position : BioObjective {
		
		[SerializeField] private Transform Target;
		[SerializeField] private double TPX, TPY, TPZ;
		[SerializeField] private double MaximumError = 0.001;

		private double ChainLength;
		private double Rescaling;
		//private Vector3 Root;

		public override ObjectiveType GetObjectiveType() {
			return ObjectiveType.Position;
		}

		public override void UpdateData() {
			if(Segment.Character.Evolution == null) {
				return;
			}
			ChainLength = 0.0;
			Transform[] chain = Segment.Character.Evolution.GetModel().FindObjectivePtr(this).Node.Chain;;
			for(int i=0; i<chain.Length-1; i++) {
				ChainLength += Vector3.Distance(chain[i].position, chain[i+1].position);
			}
			Rescaling = (Utility.PI * Utility.PI) / (ChainLength * ChainLength);
			//Root = chain[0].position;
			if(Target != null) {
				Vector3 position = Target.position;
				TPX = position.x;
				TPY = position.y;
				TPZ = position.z;
			}
		}

        //public override double ComputeLoss(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
        public override double ComputeLoss(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration, double rootWPX, double rootWPY, double rootWPZ, double rootWRX, double rootWRY, double rootWRZ, double rootWRW,bool compare, double alpha, double secondaryweight)
        {
            //Adaptive
            //if (compare != true)
            //{
          
            double loss = System.Math.Sqrt((TPX - WPX) * (TPX - WPX) + (TPY - WPY) * (TPY - WPY) + (TPZ - WPZ) * (TPZ - WPZ));
                double s = System.Math.Sqrt(
                    (node.Chain.Length + loss)
                    *
                //(System.Math.Sqrt((WPX - Root.x) * (WPX - Root.x) + (WPY - Root.y) * (WPY - Root.y) + (WPZ - Root.z) * (WPZ - Root.z)) + loss)
                //);
                (System.Math.Sqrt((WPX - rootWPX) * (WPX - rootWPX) + (WPY - rootWPY) * (WPY - rootWPY) + (WPZ - rootWPZ) * (WPZ - rootWPZ)) + loss)
                    );
                loss = Utility.PI * loss / s;
            //return Weight * loss * loss;
            //if (Segment.Character.Root.Transform.name == "Root")
                if (compare == false)
                {
                    return secondaryweight * loss * loss;
                    //return  loss * loss;
                }
                else
                {
                    return alpha * loss * loss;
                    // return loss * loss;
                }
            //return Weight * loss * loss;
                //return 100 * loss * loss;
            //}

            //List<double> v1 = new List<double>(configuration);
            //List<double> v2 = new List<double>(configuration);

            //if (v1.Count == 0 || v2.Count == 0)
            //{

            //    //throw new System.ArgumentException("Cosine Similarity: Vectors cannot be zero length");
            //}
            //if (v1.Count != v2.Count)
            //{
            //   // throw new System.ArgumentException("Cosine Similarity: Vectors must be the same length");
            //}

            //double dotProduct = 0.0;
            //double sumsq1 = 0;
            //for (int i = 0; i < v1.Count; i++)
            //{
            //    sumsq1 += v1[i] * v1[i];
            //}
            //double l2norm1 = System.Math.Sqrt(sumsq1);

            //double sumsq2 = 0;
            //for (int i = 0; i < v2.Count; i++)
            //{
            //    sumsq2 += v2[i] * v2[i];
            //}
            //double l2norm2 = System.Math.Sqrt(sumsq2);


            //if (l2norm1 == 0 || l2norm2 == 0)
            //{
            //    //throw new System.ArgumentException("Cosine Similarity: A vector cannot be a zero vector");
            //}
            //for (int i = 0; i < v1.Count; i++)
            //{
            //    dotProduct += v1[i] * v2[i];
            //}
            ////return dotProduct / (l2norm1 * l2norm2);
            //Debug.Log(dotProduct / (l2norm1 * l2norm2));
            //Fast and slightly adaptive
            //Debug.Log("position loss: " + Rescaling * ((TPX - WPX) * (TPX - WPX) + (TPY - WPY) * (TPY - WPY) + (TPZ - WPZ) * (TPZ - WPZ)));
            //else
            //{
            //    return Weight * Rescaling * ((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ));
            //}
            //Fastest but not adaptive
            //return Weight * ((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ));
        }

		public override bool CheckConvergence(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			return System.Math.Sqrt((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ)) <= MaximumError;
		}

		public override double ComputeValue(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			return System.Math.Sqrt((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ));
		}

		public void SetTargetTransform(Transform target) {
			Target = target;
			if(Target != null) {
				SetTargetPosition(Target.position);
			}
		}

		public Transform GetTargetTransform() {
			return Target;
		}

		public void SetTargetPosition(Vector3 position) {
			TPX = position.x;
			TPY = position.y;
			TPZ = position.z;
		}

		public Vector3 GetTargetPosition() {
			return new Vector3((float)TPX, (float)TPY, (float)TPZ);
		}
		
		public void SetMaximumError(double units) {
			MaximumError = units;
		}

		public double GetMaximumError() {
			return MaximumError;
		}

	}

}