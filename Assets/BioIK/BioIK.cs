using UnityEngine;
using System.Collections.Generic;
using System;
using System.IO;
using Newtonsoft.Json;

namespace BioIK {

    //[ExecuteInEditMode]
    [DisallowMultipleComponent]
    public class BioIK : MonoBehaviour {

        //public bool SolveInEditMode = false;

        [SerializeField] private bool UseThreading = true;

        [SerializeField] private int Generations = 2;
        [SerializeField] private int PopulationSize = 50;
        [SerializeField] private int Elites = 2;

        public float Smoothing = 0.5f;
        public float AnimationWeight = 0f;
        public float Errorthreshold = 0.1f;
        public float AnimationBlend = 0f;
        public MotionType MotionType = MotionType.Instantaneous;
        public float MaximumVelocity = 3f;
        public float MaximumAcceleration = 3f;
        public double currentanimationcyclecompare;


        public List<BioSegment> Segments = new List<BioSegment>();
        public List<GameObject> jointo = new List<GameObject>();
        public List<GameObject> jointc = new List<GameObject>();
        List<bool> dynamiclist = new List<bool>();
        List<double> thresholdlist = new List<double>();
        List<string> filenames = new List<string>();
        private int temp = 0;
        private bool tempangle = false;
        //float refvectomag = 0.0f;

        public List<float> weights = new List<float>();
        public List<GameObject> lefthandchain = new List<GameObject>();
        public List<GameObject> righthandchain = new List<GameObject>();
        public List<GameObject> leftlegchain = new List<GameObject>();
        public List<GameObject> rightlegchain = new List<GameObject>();
        public List<GameObject> neckchain = new List<GameObject>();
        public List<Vector3> frontspherelocations = new List<Vector3>();
        public List<Vector3> backspherelocations = new List<Vector3>();
        public List<Vector3> reconvertedlistfront = new List<Vector3>();
        public List<Vector3> reconvertedlistback = new List<Vector3>();
        float[] refvectarray = new float[63];
        public BioSegment Root = null;
        public Evolution Evolution = null;
        public double[] Solution = null;
        public double[] refSolution = null;
        public Vector3[] refcord = null;
        public double cosanglescore;
        public double posescore;
        public double cosposscore = 0;
        public double cosposscore_c = 0;
        public int totalcount = 0;
        public int dynamiclistcount;
        public int thresholdlistcount;
        public int totalcount_c = 0;
        public int targetcyclecount = 0;
        public int targetcyclecountfront = 0;
        public int targetcyclecountback = 0;
        private bool Destroyed = false;
        public bool twotarget = false;
        public bool threetarget = false;
        public bool writejson = false;
        public bool frontcsvwrite = true;
        public bool backcsvwrite = false;
        public bool humantesting;
        public bool dynamic;
        public double finalfitness;
        public Vector3 testv;
        public bool cosloss = false;
        Vector3 origtargtposition;
        Vector3 origtargtpositionrandom;
        Vector3 finaltargetposition;
        Vector3 dir;
        Vector3 dirrandomforward;
        public int longnum = 0;
        public int latnum = 0;
        public int numcirclelayers;
        public int currentlayer;

        public GameObject plane;
        public GameObject test_sphere;
        public GameObject test_sphere_;
        public GameObject test_sphere__;
        public GameObject Right_Hand_Target;
        public GameObject Right_Hand_IK;
        public double Right_Hand_IK_error;
        public GameObject Right_Hand_IK_c;
        //public Vector3 Right_Hand_Target_ini;
        //public Vector3 Right_Hand_Target_corr;
        public GameObject Eyes_Target;
        public GameObject Eyes;
        public double Eyes_error;
        public GameObject Eyes_c;
        //public Vector3 Eyes_Target_ini;
        //public Vector3 Eyes_Target_corr;
        public GameObject Left_Hand_Target;
        public GameObject Left_Hand_Target_Actual;
        public GameObject Left_Hand_IK;
        public double Left_Hand_IK_error;
        public GameObject Left_Hand_IK_c;
        //public Vector3 Left_Hand_Target_ini;
        //public Vector3 Left_Hand_Target_corr;
        public GameObject Right_Foot_Target;
        public GameObject Right_Foot_IK;
        public double Right_Foot_IK_error;
        public GameObject Right_Foot_IK_c;
        //public Vector3 Right_Foot_Target_ini;
        //public Vector3 Right_Foot_Target_corr;
        public GameObject Left_Foot_Target;
        public GameObject Left_Foot_IK;
        public double Left_Foot_IK_error;
        public GameObject Left_Foot_IK_c;
        //public Vector3 Left_Foot_Target_ini;
        //public Vector3 Left_Foot_Target_corr;
        public GameObject Right_Hand_Target_c;
        //public Vector3 Right_Hand_Target_c_ini;
        //public Vector3 Right_Hand_Target_c_corr;
        public GameObject Eyes_Target_c;
        //public Vector3 Eyes_Target_c_ini;
        //public Vector3 Eyes_Target_c_corr;
        public GameObject Left_Hand_Target_c;
        //public Vector3 Left_Hand_Target_c_ini;
        //public Vector3 Left_Hand_Target_c_corr;
        public GameObject Right_Foot_Target_c;
        //public Vector3 Right_Foot_Target_c_ini;
        //public Vector3 Right_Foot_Target_c_corr;
        public GameObject Left_Foot_Target_c;
        //public Vector3 Left_Foot_Target_c_ini;
        //public Vector3 Left_Foot_Target_c_corr;



        public GameObject Left_Forearm_Joint_remote;
        public GameObject Left_Forearm_Joint_target_remote;
        public GameObject Line_Renderer_remote;
        public GameObject testsphere_remote;
        public GameObject Left_Forearm_Joint_ref_remote;
        public Vector3 gesturerefvector_remote;
        public Quaternion Left_Forearm_Joint_01_correct_remote;

        public Vector3 rotation;
        public GameObject Robot_Kyle_ref;
        public GameObject Root_c;
        public GameObject Root_target;
        public Vector3 rootVector;
        public Vector3 radiusVector;
        public GameObject Root_;
        public GameObject Root_dummy;
        public double Root_error;

        public GameObject Hip_c;
        public GameObject Hip_target;
        public GameObject Hip;
        public GameObject Hip_dummy;
        public double Hip_error;

        public GameObject Left_Thigh_Joint_01;
        public GameObject Left_Thigh_Joint_01_c;
        public GameObject Left_Thigh_Joint_01_target;
        public GameObject Left_Thigh_Joint_01_dummy;
        public double Left_Thigh_Joint_01_error;
        public GameObject Left_Knee_Joint_01;
        public GameObject Left_Knee_Joint_01_c;
        public GameObject Left_Knee_Joint_01_target;
        public GameObject Left_Knee_Joint_01_dummy;
        public double Left_Knee_Joint_01_error;
        public GameObject Left_Ankle_Joint_01;
        public GameObject Left_Ankle_Joint_01_c;
        public GameObject Left_Ankle_Joint_01_target;
        public GameObject Left_Ankle_Joint_01_dummy;
        public GameObject Left_Toe_Joint_01;
        public GameObject Left_Toe_Joint_01_c;
        public GameObject Left_Toe_Joint_01_target;
        public GameObject Left_Toe_Joint_01_dummy;
        public double Left_Ankle_Joint_01_error;
        public double Left_Toe_Joint_01_error;

        public GameObject Right_Thigh_Joint_01;
        public GameObject Right_Thigh_Joint_01_c;
        public GameObject Right_Thigh_Joint_01_target;
        public GameObject Right_Thigh_Joint_01_dummy;
        public double Right_Thigh_Joint_01_error;
        public GameObject Right_Knee_Joint_01;
        public GameObject Right_Knee_Joint_01_c;
        public GameObject Right_Knee_Joint_01_target;
        public GameObject Right_Knee_Joint_01_dummy;
        public double Right_Knee_Joint_01_error;
        public GameObject Right_Ankle_Joint_01;
        public GameObject Right_Ankle_Joint_01_c;
        public GameObject Right_Ankle_Joint_01_target;
        public GameObject Right_Ankle_Joint_01_dummy;
        public GameObject Right_Toe_Joint_01;
        public GameObject Right_Toe_Joint_01_c;
        public GameObject Right_Toe_Joint_01_target;
        public GameObject Right_Toe_Joint_01_dummy;
        public double Right_Ankle_Joint_01_error;
        public double Right_Toe_Joint_01_error;

        public GameObject Left_Shoulder_Joint_01;
        public GameObject Left_Shoulder_Joint_01_c;
        public GameObject Left_Shoulder_Joint_01_target;
        public GameObject Left_Shoulder_Joint_01_dummy;
        public double Left_Shoulder_Joint_01_error;
        public GameObject Left_Forearm_Joint_01;
        public GameObject Left_Forearm_Joint_01_c;
        public GameObject Left_Forearm_Joint_01_target;
        public GameObject Left_Forearm_Joint_01_dummy;
        public double Left_Forearm_Joint_01_error;
        public Quaternion Left_Forearm_Joint_01_angle_error;
        public GameObject Left_Wrist_Joint_01;
        public GameObject Left_Wrist_Joint_01_c;
        public GameObject Left_Wrist_Joint_01_target;
        public GameObject Left_Wrist_Joint_01_dummy;
        public double Left_Wrist_Joint_01_error;
        public GameObject Left_Middle_Finger_Joint_01;

        public GameObject Right_Upper_Arm_Joint_01;
        public GameObject Right_Upper_Arm_Joint_01_c;
        public GameObject Right_Upper_Arm_Joint_01_target;
        public GameObject Right_Upper_Arm_Joint_01_dummy;
        public double Right_Upper_Arm_Joint_01_error;
        public GameObject Right_Shoulder_Joint_01;
        public GameObject Right_Shoulder_Joint_01_c;
        public GameObject Right_Shoulder_Joint_01_target;
        public GameObject Right_Shoulder_Joint_01_dummy;
        public double Right_Shoulder_Joint_01_error;
        public GameObject Right_Forearm_Joint_01;
        public GameObject Right_Forearm_Joint_01_c;
        public GameObject Right_Forearm_Joint_01_target;
        public GameObject Right_Forearm_Joint_01_dummy;
        public double Right_Forearm_Joint_01_error;
        public GameObject Right_Wrist_Joint_01;
        public GameObject Right_Wrist_Joint_01_c;
        public GameObject Right_Wrist_Joint_01_target;
        public GameObject Right_Wrist_Joint_01_dummy;
        public double Right_Wrist_Joint_01_error;


        //Custom Inspector Helpers
        public BioSegment SelectedSegment = null;
        public Vector2 Scroll = Vector2.zero;
        public Vector3 correcvec;
        public Vector3 correcvecstatic;
        public Vector3 initialtargetpos;
        public double Maxweight = 10;
        public double primaryweight = 10;
        public double primaryweightdummy = 10;
        public double primaryweightif;
        public double secondaryweight = 0;
        public double primaryerror;
        public double secondaryerror;
        public double primaryerrortotal = 0;
        public double primaryerrortotalfront = 0;
        public double primaryerrortotalback = 0;
        public double secondaryerrortotal = 0;
        public double secondaryerrortotalfront = 0;
        public double secondaryerrortotalback = 0;
        public double thresholdfactor;
        public double shiftingfactor = 1;
        public int targetanglecounterlat = 0;
        public int targetanglecounterlong = 0;
        public int randomseed;
        public System.Random Rand;
        public string filename;


       
    void Awake() {
            //Debug.Log("Awake");
            Refresh();
        }

        void Start() {

            //Left_Forearm_Joint_remote = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01");
            // //Left_Forearm_Joint_target_remote = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01_target");
            // Left_Forearm_Joint_ref_remote = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01/Left_Forearm_Joint_01_remote_refgameobject");
            // //Line_Renderer_remote = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01/Left_Forearm_Joint_01_remote_line_renderer");
            // testsphere_remote = GameObject.Find("test_sphere_local");
            // ////lineRenderer = Line_Renderer.GetComponent<LineRenderer>();
            // Left_Forearm_Joint_01_correct_remote = Left_Forearm_Joint_remote.transform.rotation * Quaternion.Inverse(Left_Forearm_Joint_ref_remote.transform.rotation);



            //rootVector = new Vector3(0.002187f, 1.062831f, 0.035129f);
            //radiusVector = new Vector3(-0.274f, 1.341f, 1.344f);
            filename = "P08_two_hand_angry";
            bool[] dynamicboollist = { false, true,true};
            dynamiclist.AddRange(dynamicboollist);
            double[] thresholddoublelist = { 1.0, 1.0, 13.18 };
            thresholdlist.AddRange(thresholddoublelist);
            Debug.Log("dynamic list size: " + dynamiclist.Count);
            Debug.Log("threshold list size: " + thresholdlist.Count);
            humantesting = true;
            twotarget = true;
            currentlayer = 1;
            numcirclelayers = 2;
            dynamiclistcount = 0;
            thresholdlistcount = 0;
            //dynamic = dynamiclist[dynamiclistcount];
            //thresholdfactor = thresholdlist[thresholdlistcount];
            dynamic = false;
            thresholdfactor = 13.18;
            if (!dynamic)
            {
                primaryweight = 10;
            }
            else
            {
                primaryweightdummy = 10;
            }

            plane = GameObject.Find("Plane");
            test_sphere_ = GameObject.Find("test_sphere");
            test_sphere__ = GameObject.Find("sphere_green");

            test_sphere = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root_target");
            Root_ = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root");
            //Root_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root_target");
            //Root_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c");
            Root_dummy = GameObject.Find("Robot Kyle/Root");
            Robot_Kyle_ref = GameObject.Find("Robot Kyle");

            jointo.Add(Root_);
            jointc.Add(Root_c);
            weights.Add(1.0f);
            righthandchain.Add(Root_c);
            lefthandchain.Add(Root_c);
            rightlegchain.Add(Root_c);
            leftlegchain.Add(Root_c);
            neckchain.Add(Root_c);

            Hip = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip");
            Hip_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Hip");
            Hip_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip_target");
            Hip_dummy = GameObject.Find("Robot Kyle/Root/Hip");
            jointo.Add(Hip);
            jointc.Add(Hip_c);
            weights.Add(1.0f);
            leftlegchain.Add(Hip_c);
            rightlegchain.Add(Hip_c);
            Left_Thigh_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Left_Thigh_Joint_01");
            Left_Thigh_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Left_Thigh_Joint_01_target");
            Left_Thigh_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Left_Thigh_Joint_01");
            Left_Thigh_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root/Hip/Left_Thigh_Joint_01");
            jointo.Add(Left_Thigh_Joint_01);
            jointc.Add(Left_Thigh_Joint_01_c);
            weights.Add(1.0f);
            leftlegchain.Add(Left_Thigh_Joint_01_c);
            Left_Knee_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01");
            Left_Knee_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01_target");
            Left_Knee_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01");
            Left_Knee_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01");
            jointo.Add(Left_Knee_Joint_01);
            jointc.Add(Left_Knee_Joint_01_c);
            weights.Add(1.0f);
            leftlegchain.Add(Left_Knee_Joint_01_c);
            Left_Ankle_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01/Left_Ankle_Joint_01");
            Left_Ankle_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01/Left_Ankle_Joint_01_target");
            Left_Ankle_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01/Left_Ankle_Joint_01");
            Left_Ankle_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01/Left_Ankle_Joint_01");
            jointo.Add(Left_Ankle_Joint_01);
            jointc.Add(Left_Ankle_Joint_01_c);
            weights.Add(1.0f);
            leftlegchain.Add(Left_Ankle_Joint_01_c);
            Left_Toe_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01/Left_Ankle_Joint_01/Left_Toe_Joint_01");
            Left_Toe_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01/Left_Ankle_Joint_01/Left_Toe_Joint_01_target");
            Left_Toe_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01/Left_Ankle_Joint_01/Left_Toe_Joint_01");
            Left_Toe_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01/Left_Ankle_Joint_01/Left_Toe_Joint_01");
            jointo.Add(Left_Toe_Joint_01);
            jointc.Add(Left_Toe_Joint_01_c);
            weights.Add(1.0f);
            leftlegchain.Add(Left_Toe_Joint_01_c);



            Right_Thigh_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Right_Thigh_Joint_01");
            Right_Thigh_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Right_Thigh_Joint_01_target");
            Right_Thigh_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Right_Thigh_Joint_01");
            Right_Thigh_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Hip/Right_Thigh_Joint_01");
            jointo.Add(Right_Thigh_Joint_01);
            jointc.Add(Right_Thigh_Joint_01_c);
            weights.Add(1.0f);
            rightlegchain.Add(Right_Thigh_Joint_01_c);
            Right_Knee_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01");
            Right_Knee_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01_target");
            Right_Knee_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01");
            Right_Knee_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01");
            jointo.Add(Right_Knee_Joint_01);
            jointc.Add(Right_Knee_Joint_01_c);
            weights.Add(1.0f);
            rightlegchain.Add(Right_Knee_Joint_01_c);
            Right_Ankle_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01/Right_Ankle_Joint_01");
            Right_Ankle_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01/Right_Ankle_Joint_01_target");
            Right_Ankle_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01/Right_Ankle_Joint_01");
            Right_Ankle_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01/Right_Ankle_Joint_01");
            jointo.Add(Right_Ankle_Joint_01);
            jointc.Add(Right_Ankle_Joint_01_c);
            weights.Add(1.0f);
            rightlegchain.Add(Right_Ankle_Joint_01_c);
            Right_Toe_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01/Right_Ankle_Joint_01/Right_Toe_Joint_01");
            Right_Toe_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01/Right_Ankle_Joint_01/Right_Toe_Joint_01_target");
            Right_Toe_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01/Right_Ankle_Joint_01/Right_Toe_Joint_01");
            Right_Toe_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01/Right_Ankle_Joint_01/Right_Toe_Joint_01");
            jointo.Add(Right_Toe_Joint_01);
            jointc.Add(Right_Toe_Joint_01_c);
            weights.Add(1.0f);
            rightlegchain.Add(Right_Toe_Joint_01);

            GameObject Ribs = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs");
            GameObject Ribs_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs");
            jointo.Add(Ribs);
            jointc.Add(Ribs_c);
            weights.Add(1.0f);
            righthandchain.Add(Ribs_c);
            lefthandchain.Add(Ribs_c);
            neckchain.Add(Ribs_c);

            Left_Shoulder_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01");
            Left_Shoulder_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01_target");
            Left_Shoulder_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Left_Shoulder_Joint_01");
            Left_Shoulder_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Left_Shoulder_Joint_01");
            jointo.Add(Left_Shoulder_Joint_01);
            jointc.Add(Left_Shoulder_Joint_01_c);
            weights.Add(1.0f);
            lefthandchain.Add(Left_Shoulder_Joint_01_c);
            GameObject Left_Upper_Arm_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01");
            GameObject Left_Upper_Arm_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01");
            jointo.Add(Left_Upper_Arm_Joint_01);
            jointc.Add(Left_Upper_Arm_Joint_01_c);
            weights.Add(1.0f);
            lefthandchain.Add(Left_Upper_Arm_Joint_01_c);
            Left_Forearm_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01");
            Left_Forearm_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01_target");
            Left_Forearm_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01");
            Left_Forearm_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01");
            jointo.Add(Left_Forearm_Joint_01);
            jointc.Add(Left_Forearm_Joint_01_c);
            weights.Add(1.0f);
            lefthandchain.Add(Left_Forearm_Joint_01_c);
            Left_Wrist_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01/Left_Wrist_Joint_01/Left_Middle_Finger_Joint_01a/Left_Middle_Finger_Joint_01b/Left_Middle_Finger_Joint_01c");
            Left_Middle_Finger_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01/Left_Wrist_Joint_01/");
            Left_Wrist_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01/Left_Wrist_Joint_01_target");
            Left_Wrist_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01/Left_Wrist_Joint_01");
            Left_Wrist_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01/Left_Wrist_Joint_01");

            jointo.Add(Left_Wrist_Joint_01);
            jointc.Add(Left_Wrist_Joint_01_c);
            weights.Add(1.0f);
            lefthandchain.Add(Left_Wrist_Joint_01_c);




            Right_Shoulder_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Right_Shoulder_Joint_01");
            Right_Shoulder_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Right_Shoulder_Joint_01_target");
            Right_Shoulder_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Right_Shoulder_Joint_01");
            Right_Shoulder_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Right_Shoulder_Joint_01");
            jointo.Add(Right_Shoulder_Joint_01);
            jointc.Add(Right_Shoulder_Joint_01_c);
            weights.Add(1.0f);
            righthandchain.Add(Right_Shoulder_Joint_01_c);
            Right_Upper_Arm_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01");
            Right_Upper_Arm_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01_target");
            Right_Upper_Arm_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01");
            Right_Upper_Arm_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01");
            jointo.Add(Right_Upper_Arm_Joint_01);
            jointc.Add(Right_Upper_Arm_Joint_01_c);
            weights.Add(1.0f);
            righthandchain.Add(Right_Upper_Arm_Joint_01_c);
            Right_Forearm_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01/Right_Forearm_Joint_01");
            Right_Forearm_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01/Right_Forearm_Joint_01_target");
            Right_Forearm_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01/Right_Forearm_Joint_01");
            Right_Forearm_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01/Right_Forearm_Joint_01");
            jointo.Add(Right_Forearm_Joint_01);
            jointc.Add(Right_Forearm_Joint_01_c);
            weights.Add(1.0f);
            righthandchain.Add(Right_Forearm_Joint_01_c);
            Right_Wrist_Joint_01 = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01/Right_Forearm_Joint_01/Right_Wrist_Joint_01");
            Right_Wrist_Joint_01_target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01/Right_Forearm_Joint_01/Right_Wrist_Joint_01_target");
            Right_Wrist_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01/Right_Forearm_Joint_01/Right_Wrist_Joint_01");
            Right_Wrist_Joint_01_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01/Right_Forearm_Joint_01/Right_Wrist_Joint_01");
            jointo.Add(Right_Wrist_Joint_01);
            jointc.Add(Right_Wrist_Joint_01_c);
            weights.Add(1.0f);
            righthandchain.Add(Right_Wrist_Joint_01_c);




            GameObject Neck = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Neck");
            GameObject Neck_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Neck");
            jointo.Add(Neck);
            jointc.Add(Neck_c);
            weights.Add(1.0f);
            neckchain.Add(Neck_c);
            GameObject Head = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Neck/Head");
            GameObject Head_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Neck/Head");
            jointo.Add(Head);
            jointc.Add(Head_c);
            weights.Add(1.0f);
            neckchain.Add(Head_c);






            Right_Hand_Target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Right_Hand_Target");
            Right_Hand_IK = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01/Right_Forearm_Joint_01/Right_Wrist_Joint_01/Right_Hand_IK");
            //Right_Hand_Target_ini = Right_Hand_Target.transform.position;

            initialtargetpos = Right_Hand_Target.transform.position;

            Eyes_Target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Eyes_Target");
            Eyes = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Neck/Head/Eyes");

            //Eyes_Target_ini = Eyes_Target.transform.position;
            Left_Hand_Target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Left_Hand_Target");
            Left_Hand_Target_Actual = GameObject.Find("Full Body/Robot Kyle (Full Body)/Left_Hand_Target_Actual");
            Left_Hand_IK = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01/Left_Wrist_Joint_01/Left_Hand_IK");

            //Left_Hand_Target_ini = Left_Hand_Target.transform.position;
            Right_Foot_Target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Right_Foot_Target");
            Right_Foot_IK = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01/Right_Ankle_Joint_01/Right_Toe_Joint_01/Right_Foot_IK");

            //Right_Foot_Target_ini = Right_Foot_Target.transform.position;
            Left_Foot_Target = GameObject.Find("Full Body/Robot Kyle (Full Body)/Left_Foot_Target");
            Left_Foot_IK = GameObject.Find("Full Body/Robot Kyle (Full Body)/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01/Left_Ankle_Joint_01/Left_Toe_Joint_01/Left_Foot_IK");

            //Left_Foot_Target_ini = Left_Foot_Target.transform.position;
            Right_Hand_Target_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Right_Hand_Target");
            Right_Hand_IK_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01/Right_Forearm_Joint_01/Right_Wrist_Joint_01/Right_Hand_IK");
            //Right_Hand_Target_c_ini = Right_Hand_Target_c.transform.position;
            Eyes_Target_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Eyes_Target");
            Eyes_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Neck/Head/Eyes");
            //Eyes_Target_c_ini = Eyes_Target_c.transform.position;
            Left_Hand_Target_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Left_Hand_Target");
            Left_Hand_IK_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01/Left_Wrist_Joint_01/Left_Hand_IK");
            //Left_Hand_Target_c_ini = Left_Hand_Target_c.transform.position;
            Right_Foot_Target_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Right_Foot_Target");
            Right_Foot_IK_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01/Right_Ankle_Joint_01/Right_Toe_Joint_01/Right_Foot_IK");
            //Right_Foot_Target_c_ini = Right_Foot_Target_c.transform.position;
            Left_Foot_Target_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Left_Foot_Target");
            Left_Foot_IK_c = GameObject.Find("Full Body (1)/Robot Kyle (Full Body)/Root_c/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01/Left_Ankle_Joint_01/Left_Toe_Joint_01/Left_Foot_IK");
            //Left_Foot_Target_c_ini = Left_Hand_Target_c.transform.position;

            correcvecstatic = Root.transform.position - Root_dummy.transform.position;

            origtargtposition = Left_Hand_Target.transform.position;
            finaltargetposition = Left_Hand_Target.transform.position;

            //Left_Hand_Target.transform.position = Root_.transform.position;
            origtargtpositionrandom = Root_.transform.position;

            //Vector3 pos = test_sphere.transform.position;
            dir = (Left_Hand_Target.transform.position - test_sphere.transform.position).normalized;
            dirrandomforward = (finaltargetposition - origtargtpositionrandom).normalized;
            randomseed = 10;
            // Initialize the random number generator for two and three targets.

            Rand = new System.Random(randomseed);

        

            //Reading all front and back positions in case of 2 and 3 targets
            if (twotarget)
            {


                reconvertedlistfront = JsonConvert.DeserializeObject<List<Vector3>>(File.ReadAllText(@"C:/Users/ullala/Desktop/bioik_testing_csv/fronthemispherelocationsjson.json"));
                //Debug.Log("Number of elements in front hemisphere:" + reconvertedlistfront.Count);
                reconvertedlistback = JsonConvert.DeserializeObject<List<Vector3>>(File.ReadAllText(@"C:/Users/ullala/Desktop/bioik_testing_csv/backhemispherelocationsjson.json"));
                //Debug.Log("Number of elements in back hemisphere:" + reconvertedlistback.Count);

            }

            //This is for custom rotation of goal target (left hand in this case)
          

            //Left_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(0.0f, 1.0f, 0.0f), 0);
            //Left_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(Mathf.Cos(latnum * Mathf.Deg2Rad), 0.0f, -Mathf.Sin(latnum * Mathf.Deg2Rad)), 18);

            //refcord = new Vector3[jointo.Count];

            //for (int j = 0; j < jointo.Count; j++)
            //{
            //    refcord[j] = jointo[j].transform.position;


            //}

            //Debug.Log("wrist finger distance :" + Vector3.Distance(Left_Middle_Finger_Joint_01.transform.position, Left_Wrist_Joint_01.transform.position));
            //Debug.Log("forearm distance :" + Vector3.Distance(Left_Wrist_Joint_01.transform.position, Left_Forearm_Joint_01.transform.position));
            //Debug.Log("bicep distance distance :" + Vector3.Distance(Left_Forearm_Joint_01.transform.position, Left_Shoulder_Joint_01.transform.position));
            //Debug.Log("body distance :" + Vector3.Distance(Left_Shoulder_Joint_01.transform.position, Hip.transform.position));


            //Debug.Log("ankle distance :" + Vector3.Distance(Right_Toe_Joint_01.transform.position, Right_Ankle_Joint_01.transform.position));
            //Debug.Log("knee distance :" + Vector3.Distance(Right_Ankle_Joint_01.transform.position, Right_Knee_Joint_01.transform.position));
            //Debug.Log("thigh distance :" + Vector3.Distance(Right_Thigh_Joint_01.transform.position, Right_Knee_Joint_01.transform.position));

        }

        void OnDestroy() {
            Destroyed = true;
            DeInitialise();
            Utility.Cleanup(transform);
        }

        void OnEnable() {
            //Debug.Log("OnEnable");
            Initialise();
        }

        void OnDisable() {
            DeInitialise();
        }

        private void Initialise() {
            if (Evolution == null) {
                //Evolution = new Evolution(new Model(this), PopulationSize, Elites, UseThreading);
                // Debug.Log("before first call");
                Evolution = new Evolution(new Model(this), PopulationSize, Elites, UseThreading, refSolution);
                // Debug.Log("after first call");
                //    Debug.Log("refsol before = " + String.Join("",
                //new List<double>(refSolution)
                //.ConvertAll(i => i.ToString())
                //.ToArray()));
            }
        }

        private void DeInitialise() {
            if (Evolution != null) {
                // Debug.Log("DeInitialize");
                Evolution.Kill();
                Evolution = null;
            }
        }

        void Update() {


            // all measures related to gestural optmization

            //Line_Renderer_remote.GetComponent<LineRenderer>().SetPosition(0, Left_Forearm_Joint_remote.transform.position);
            //Line_Renderer_remote.GetComponent<LineRenderer>().SetPosition(1, testsphere_remote.transform.position);
            // gesturerefvector_remote = testsphere_remote.transform.position - Left_Forearm_Joint_remote.transform.position ;
            //Left_Forearm_Joint_ref_remote.transform.rotation = Quaternion.LookRotation(gesturerefvector_remote, Vector3.up)* Left_Forearm_Joint_01_correct_remote;
            ////Left_Forearm_Joint_ref.transform.rotation = Line_Renderer.transform.rotation;
            //Left_Forearm_Joint_remote.transform.rotation = Line_Renderer_remote.transform.rotation * Left_Forearm_Joint_01_correct_remote;
            //Debug.Log("left forearm rotation before: " + Left_Forearm_Joint_01.transform.rotation.eulerAngles);
            //Left_Forearm_Joint_01.transform.rotation = Left_Forearm_Joint_01.transform.rotation * Quaternion.Euler(0, 1, 0);
            //Debug.Log("left forearm rotation after: " + Left_Forearm_Joint_01.transform.rotation.eulerAngles);




            // all measures realted to positional optmization



            //Right_Hand_Target_c.transform.localPosition = Right_Hand_Target.transform.localPosition;
            //Eyes_Target_c.transform.localPosition = Eyes_Target.transform.localPosition;
            //Left_Hand_Target_c.transform.localPosition = Left_Hand_Target.transform.localPosition;
            //Right_Foot_Target_c.transform.localPosition = Right_Foot_Target.transform.localPosition;
            //Left_Foot_Target_c.transform.localPosition = Left_Foot_Target.transform.localPosition;



            //Right_Hand_Target_ini = Right_Hand_Target.transform.position;
            //correcvec = Root.transform.position - Root_c.transform.position;
            //correcvec = Root.transform.position - Root_dummy.transform.position;
            //Root_target.transform.position = Root_dummy.transform.position + correcvec;
            //Debug.Log("left thigh position: " + Left_Thigh_Joint_01_dummy.transform.position);

            //Left_Thigh_Joint_01_target.transform.position = Left_Thigh_Joint_01_c.transform.position + correcvec;
            //Left_Knee_Joint_01_target.transform.position = Left_Knee_Joint_01_c.transform.position + correcvec;
            //Left_Ankle_Joint_01_target.transform.position = Left_Ankle_Joint_01_c.transform.position + correcvec;

            Hip_target.transform.position = Hip_dummy.transform.position + correcvecstatic;
            Hip_error = Vector3.Distance(Hip_target.transform.position, Hip.transform.position);

            Left_Thigh_Joint_01_target.transform.position = Left_Thigh_Joint_01_dummy.transform.position + correcvecstatic;
            Left_Thigh_Joint_01_error = Vector3.Distance(Left_Thigh_Joint_01_target.transform.position, Left_Thigh_Joint_01.transform.position);
            Left_Knee_Joint_01_target.transform.position = Left_Knee_Joint_01_dummy.transform.position + correcvecstatic;
            Left_Knee_Joint_01_error = Vector3.Distance(Left_Knee_Joint_01_target.transform.position, Left_Knee_Joint_01.transform.position);
            Left_Ankle_Joint_01_target.transform.position = Left_Ankle_Joint_01_dummy.transform.position + correcvecstatic;
            Left_Ankle_Joint_01_error = Vector3.Distance(Left_Ankle_Joint_01_target.transform.position, Left_Ankle_Joint_01.transform.position);
            Left_Toe_Joint_01_target.transform.position = Left_Toe_Joint_01_dummy.transform.position + correcvecstatic;
            Left_Toe_Joint_01_error = Vector3.Distance(Left_Toe_Joint_01_target.transform.position, Left_Toe_Joint_01.transform.position);

            //Right_Thigh_Joint_01_target.transform.position = Right_Thigh_Joint_01_c.transform.position + correcvec;
            //Right_Knee_Joint_01_target.transform.position = Right_Knee_Joint_01_c.transform.position + correcvec;
            //Right_Ankle_Joint_01_target.transform.position = Right_Ankle_Joint_01_c.transform.position + correcvec;

            Right_Thigh_Joint_01_target.transform.position = Right_Thigh_Joint_01_dummy.transform.position + correcvecstatic;
            Right_Thigh_Joint_01_error = Vector3.Distance(Right_Thigh_Joint_01_target.transform.position, Right_Thigh_Joint_01.transform.position);
            Right_Knee_Joint_01_target.transform.position = Right_Knee_Joint_01_dummy.transform.position + correcvecstatic;
            Right_Knee_Joint_01_error = Vector3.Distance(Right_Knee_Joint_01_target.transform.position, Right_Knee_Joint_01.transform.position);
            Right_Ankle_Joint_01_target.transform.position = Right_Ankle_Joint_01_dummy.transform.position + correcvecstatic;
            Right_Ankle_Joint_01_error = Vector3.Distance(Right_Ankle_Joint_01_target.transform.position, Right_Ankle_Joint_01.transform.position);
            Right_Toe_Joint_01_target.transform.position = Right_Toe_Joint_01_dummy.transform.position + correcvecstatic;
            Right_Toe_Joint_01_error = Vector3.Distance(Right_Toe_Joint_01_target.transform.position, Right_Toe_Joint_01.transform.position);

            //Left_Shoulder_Joint_01_target.transform.position = Left_Shoulder_Joint_01_c.transform.position + correcvec;
            //Left_Forearm_Joint_01_target.transform.position = Left_Forearm_Joint_01_c.transform.position + correcvec;
            //Left_Wrist_Joint_01_target.transform.position = Left_Wrist_Joint_01_c.transform.position + correcvec;

            Left_Shoulder_Joint_01_target.transform.position = Left_Shoulder_Joint_01_dummy.transform.position + correcvecstatic;
            Left_Shoulder_Joint_01_error = Vector3.Distance(Left_Shoulder_Joint_01_target.transform.position, Left_Shoulder_Joint_01.transform.position);
            Left_Forearm_Joint_01_target.transform.position = Left_Forearm_Joint_01_dummy.transform.position + correcvecstatic;
            Left_Forearm_Joint_01_error = Vector3.Distance(Left_Forearm_Joint_01_target.transform.position, Left_Forearm_Joint_01.transform.position);

            //Debug.Log("left forearm joint error"+ Left_Forearm_Joint_01_angle_error);
            Left_Wrist_Joint_01_target.transform.position = Left_Wrist_Joint_01_dummy.transform.position + correcvecstatic;
            Left_Wrist_Joint_01_error = Vector3.Distance(Left_Wrist_Joint_01_target.transform.position, Left_Wrist_Joint_01.transform.position);

            //Right_Shoulder_Joint_01_target.transform.position = Right_Shoulder_Joint_01_c.transform.position + correcvec;
            //Right_Forearm_Joint_01_target.transform.position = Right_Forearm_Joint_01_c.transform.position + correcvec;
            //Right_Wrist_Joint_01_target.transform.position = Right_Wrist_Joint_01_c.transform.position + correcvec;

            Right_Shoulder_Joint_01_target.transform.position = Right_Shoulder_Joint_01_dummy.transform.position + correcvecstatic;
            Right_Shoulder_Joint_01_error = Vector3.Distance(Right_Shoulder_Joint_01_target.transform.position, Right_Shoulder_Joint_01.transform.position);
            Right_Forearm_Joint_01_target.transform.position = Right_Forearm_Joint_01_dummy.transform.position + correcvecstatic;
            Right_Forearm_Joint_01_error = Vector3.Distance(Right_Forearm_Joint_01_target.transform.position, Right_Forearm_Joint_01.transform.position);
            Right_Wrist_Joint_01_target.transform.position = Right_Wrist_Joint_01_dummy.transform.position + correcvecstatic;
            Right_Wrist_Joint_01_error = Vector3.Distance(Right_Wrist_Joint_01_target.transform.position, Right_Wrist_Joint_01.transform.position);



            Right_Hand_IK_error = Vector3.Distance(Right_Hand_Target.transform.position, Right_Hand_IK.transform.position);
            Eyes_error = Vector3.Distance(Eyes_Target.transform.position, Eyes.transform.position);
            Left_Hand_IK_error = Vector3.Distance(Left_Hand_Target.transform.position, Left_Hand_IK.transform.position);
            //Left_Hand_IK_error = Vector3.Distance(Left_Hand_Target_Actual.transform.position, Left_Hand_IK.transform.position);
            Right_Foot_IK_error = Vector3.Distance(Right_Foot_Target.transform.position, Right_Foot_IK.transform.position);
            Left_Foot_IK_error = Vector3.Distance(Left_Foot_Target.transform.position, Left_Foot_IK.transform.position);


         
          

            /*  if (writejson)
              {
                  //var json = JsonConvert.SerializeObject(backspherelocations); // To Serialise
                  var json = JsonConvert.SerializeObject(frontspherelocations, Formatting.None,
                          new JsonSerializerSettings()
                          {
                              ReferenceLoopHandling = ReferenceLoopHandling.Ignore
                          });
                  File.WriteAllText(@"C:/Users/ullala/Desktop/bioik_testing_csv/fronthemispherelocationsjson.json", json);
                  var reconvertedlist = JsonConvert.DeserializeObject<List<Vector3>>(File.ReadAllText(@"C:/Users/ullala/Desktop/bioik_testing_csv/fronthemispherelocationsjson.json")); // To Deserialise
                  Debug.Log("Number of elements:" +reconvertedlist.Count);



                  //Debug.Log("target cycle count:" + targetcyclecount);
              }*/

            ////Error calculations
            //primaryerror = Right_Hand_IK_error + Left_Hand_IK_error + Right_Foot_IK_error + Left_Foot_IK_error;



            //primaryweight = 9.0;
            //secondaryweight = 1.0;




            //if (Input.GetKeyDown(KeyCode.C))
            {
                if (currentlayer < numcirclelayers)  // This if condition is used for goal target in locations of spherical form with several layers  
                {
                   
                    //Debug.Log("current layer : " + currentlayer + "numcirclelayers : " + numcirclelayers);
                    Maxweight = 10;
                    //primaryweight = Maxweight;
                    //if (primaryweight <= 0)
                    //{
                    //    primaryweight = Maxweight;
                    //}
                    //thresholdfactor = (1 / (double)Errorthreshold) * Math.Log(((0.9999) / (Maxweight * (1 - 0.9999))));
                  

                   
                    
                    if (threetarget)
                    {
                        primaryerror = Left_Hand_IK_error + Right_Hand_IK_error + Right_Foot_IK_error;
                    }
                    else if (twotarget)
                    {
                        primaryerror = Left_Hand_IK_error + Right_Hand_IK_error;
                    }
                    else
                    {
                        //primaryerror = Left_Hand_IK_error;
                        primaryerror = Right_Hand_IK_error;
                    }


                    primaryerrortotal = primaryerrortotal + primaryerror;
                    secondaryerrortotal = secondaryerrortotal + secondaryerror;

                    //if (frontcsvwrite)
                    //{
                    //    primaryerrortotalfront = primaryerrortotalfront + primaryerror;
                    //    secondaryerrortotalfront = secondaryerrortotalfront + secondaryerror;
                    //}

                    //if (backcsvwrite)
                    //{
                    //    primaryerrortotalback = primaryerrortotalback + primaryerror;
                    //    secondaryerrortotalback = secondaryerrortotalback + secondaryerror;
                    //}
                    
                    //secondaryerror = Left_Shoulder_Joint_01_error + Left_Forearm_Joint_01_error + Left_Wrist_Joint_01_error;
                    secondaryerror = Hip_error + Left_Thigh_Joint_01_error + Left_Knee_Joint_01_error + Left_Ankle_Joint_01_error + Left_Toe_Joint_01_error+Right_Thigh_Joint_01_error + Right_Knee_Joint_01_error + Right_Ankle_Joint_01_error + Right_Toe_Joint_01_error+Left_Shoulder_Joint_01_error + Left_Forearm_Joint_01_error + Left_Wrist_Joint_01_error + Right_Shoulder_Joint_01_error + Right_Forearm_Joint_01_error + Right_Wrist_Joint_01_error;
                    //System.Random rnd = new System.Random();
                    //double alphag = rnd.Next(1, (int)maxweight);
                    if (dynamic)
                    {
                        if (twotarget)
                        {
                            primaryweight = (Maxweight) / (1 + (Math.Pow(2.71828, -(thresholdfactor * (primaryerror) / 2))));
                        }
                        else
                        {
                            primaryweight = (Maxweight) / (1 + (Math.Pow(2.71828, -(thresholdfactor * (primaryerror) / 1))));
                        }
                    }
                    //Debug.Log(Math.Truncate(Robot_Kyle_ref.GetComponent<Animator>().GetCurrentAnimatorStateInfo(0).normalizedTime));
                    //Debug.Log(Robot_Kyle_ref.GetComponent<Animator>().GetCurrentAnimatorStateInfo(0).normalizedTime);
                    secondaryweight = Maxweight - primaryweight;

                    if (Math.Truncate(Robot_Kyle_ref.GetComponent<Animator>().GetCurrentAnimatorStateInfo(0).normalizedTime) > currentanimationcyclecompare)
                    {


                        ////StreamWriter sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/ARtestingpaperdynamicsprintingsecondtest.csv", true);
                        //StreamWriter sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/paper_fronthemisphere/ARtestingpaperdynamicwalking.csv", true);
                        ////StreamWriter sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/ARtestingpaperstaticsprinting.csv", true);
                        //var line = String.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}", Root_.transform.position.x, Root_.transform.position.y, Root_.transform.position.z, Left_Hand_Target.transform.position.x, Left_Hand_Target.transform.position.y, Left_Hand_Target.transform.position.z, Errorthreshold.ToString(), targetcyclecount.ToString(), targetcyclecountfront.ToString(),primaryweight.ToString(), secondaryweight.ToString(), primaryerrortotal.ToString(), secondaryerrortotal.ToString());
                        //sw.WriteLine(line);
                        //sw.Close();

                        //if (frontcsvwrite)
                        //{
                        //    string linefront;
                        //    StreamWriter sw;

                        //    if (twotarget)
                        //    {


                        //        sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/paper_threshold_2target/ARtestingpaperfrontdynamicrunning6.0thresholdfactor.csv", true);

                        //        linefront = String.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15}", Root_.transform.position.x, Root_.transform.position.y, Root_.transform.position.z, Left_Hand_Target.transform.position.x, Left_Hand_Target.transform.position.y, Left_Hand_Target.transform.position.z, Right_Hand_Target.transform.position.x, Right_Hand_Target.transform.position.y, Right_Hand_Target.transform.position.z, Errorthreshold.ToString(), targetcyclecount.ToString(), targetcyclecountfront.ToString(), primaryweight.ToString(), secondaryweight.ToString(), primaryerrortotalfront.ToString(), secondaryerrortotalfront.ToString());

                        //    }
                        //    else
                        //    {


                        //         sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/paper_threshold_2target/ARtestingpaperfrontdynamicrunning6.0thresholdfactor.csv", true);

                        //        linefront = String.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}", Root_.transform.position.x, Root_.transform.position.y, Root_.transform.position.z, Left_Hand_Target.transform.position.x, Left_Hand_Target.transform.position.y, Left_Hand_Target.transform.position.z, Errorthreshold.ToString(), targetcyclecount.ToString(), targetcyclecountfront.ToString(), primaryweight.ToString(), secondaryweight.ToString(), primaryerrortotalfront.ToString(), secondaryerrortotalfront.ToString());
                        //    }
                        //    sw.WriteLine(linefront);
                        //    sw.Close();
                        //}

                        //if (backcsvwrite)

                        //{
                        //    string lineback;
                        //    StreamWriter sw;

                        //    if (twotarget)
                        //    {

                        //        sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/paper_threshold_2target/ARtestingpaperbackdynamicrunning6.0thresholdfactor.csv", true);

                        //        lineback = String.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15}", Root_.transform.position.x, Root_.transform.position.y, Root_.transform.position.z, Left_Hand_Target.transform.position.x, Left_Hand_Target.transform.position.y, Left_Hand_Target.transform.position.z, Right_Hand_Target.transform.position.x, Right_Hand_Target.transform.position.y, Right_Hand_Target.transform.position.z, Errorthreshold.ToString(), targetcyclecount.ToString(), targetcyclecountback.ToString(), primaryweight.ToString(), secondaryweight.ToString(), primaryerrortotalback.ToString(), secondaryerrortotalback.ToString());
                        //    }
                        //    else
                        //    {
                        //        sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/paper_threshold_2target/ARtestingpaperbackdynamicrunning6.0thresholdfactor.csv", true);
                        //        lineback = String.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}", Root_.transform.position.x, Root_.transform.position.y, Root_.transform.position.z, Left_Hand_Target.transform.position.x, Left_Hand_Target.transform.position.y, Left_Hand_Target.transform.position.z, Errorthreshold.ToString(), targetcyclecount.ToString(), targetcyclecountback.ToString(), primaryweight.ToString(), secondaryweight.ToString(), primaryerrortotalback.ToString(), secondaryerrortotalback.ToString());
                        //    }
                        //    sw.WriteLine(lineback);
                        //    sw.Close();
                        //} 


                        if (humantesting)

                        {
                            string lineback;
                            StreamWriter sw;

                            if (twotarget)
                            {

                                //sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/humantestingresults/IJSCjournalpaper2fulltarget1layer_staticAkshithangry.csv", true);

                                if (dynamic)
                                {
                                    sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/humantestingresults/IJSCjournalpaper_two_target" + filename + "_dynamic_" + thresholdfactor + "_.csv", true);
                                }
                                else
                                {
                                    sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/humantestingresults/IJSCjournalpaper_two_target" + filename + "_static_.csv", true);
                                }

                                lineback = String.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}", Root_.transform.position.x, Root_.transform.position.y, Root_.transform.position.z, Left_Hand_Target.transform.position.x, Left_Hand_Target.transform.position.y, Left_Hand_Target.transform.position.z, Right_Hand_Target.transform.position.x, Right_Hand_Target.transform.position.y, Right_Hand_Target.transform.position.z, thresholdfactor.ToString(), targetcyclecount.ToString(), primaryweight.ToString(), secondaryweight.ToString(), primaryerrortotal.ToString(), secondaryerrortotal.ToString());
                            }
                            else
                            {
                                // sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/humantestingresults/IJSCjournalpaper1fulltarget1layer_staticAkshithangry.csv", true);
                                if (dynamic)
                                {
                                    sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/humantestingresults/IJSCjournalpaper_" + filename + "_dynamic_" + thresholdfactor + "_.csv", true);
                                }
                                else
                                {
                                    sw = new StreamWriter(@"C:/Users/ullala/Desktop/bioik_testing_csv/humantestingresults/IJSCjournalpaper_" + filename + "_static_.csv", true);
                                }

                                lineback = String.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11}", Root_.transform.position.x, Root_.transform.position.y, Root_.transform.position.z, Left_Hand_Target.transform.position.x, Left_Hand_Target.transform.position.y, Left_Hand_Target.transform.position.z, thresholdfactor.ToString(), targetcyclecount.ToString(), primaryweight.ToString(), secondaryweight.ToString(), primaryerrortotal.ToString(), secondaryerrortotal.ToString());
                            }
                            sw.WriteLine(lineback);
                            sw.Close();
                        }

                        if (!dynamic)
                        {
                            primaryweight = primaryweight - 1.0;
                            Debug.Log("Primary weight : " + primaryweight);
                        }
                        else
                        {
                            primaryweightdummy = primaryweightdummy - 1.0;
                            Debug.Log("Primary weight : " + primaryweight);
                            Debug.Log("primary weight dummy:" + primaryweightdummy);
                        }



                    }


                    currentanimationcyclecompare = Math.Truncate(Robot_Kyle_ref.GetComponent<Animator>().GetCurrentAnimatorStateInfo(0).normalizedTime);
                    if (dynamic)
                    {
                        primaryweightif = primaryweightdummy;
                    }
                    else
                    {
                        primaryweightif = primaryweight;
                    }
                    // below code is used for moving goal target 360 degree in a spherical fashion with several layers 

                   //if (primaryweightdummy <= 0.0)
                   if (primaryweightif <= 0.0 )
                    {
                        if (!dynamic)
                        {
                            primaryweight = 10.0;
                        }
                        else
                        {
                            primaryweightdummy = 10.0;
                        }

                       

                        longnum += 1;
                        //if (longnum <= 20 && latnum <= 180)
                        // if (longnum <= 8 && latnum <= 180) //for 45 degree
                        if (longnum <= 4 && latnum <= 180) //for 90 degree
                        
                        {
                            //Left_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(Mathf.Cos(latnum * Mathf.Deg2Rad), 0.0f, -Mathf.Sin(latnum * Mathf.Deg2Rad)), 18);
                            //Right_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(Mathf.Cos(latnum * Mathf.Deg2Rad), 0.0f, -Mathf.Sin(latnum * Mathf.Deg2Rad)), 18);
                            //Right_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(Mathf.Cos(latnum * Mathf.Deg2Rad), 0.0f, -Mathf.Sin(latnum * Mathf.Deg2Rad)), 45);
                            Right_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(Mathf.Cos(latnum * Mathf.Deg2Rad), 0.0f, -Mathf.Sin(latnum * Mathf.Deg2Rad)), 90);
                            //this.transform.RotateAround(new Vector3(0.0f, 0.0f, 0.0f), new Vector3(0.0f, 1.0f, 0.0f), 18);

                            if (latnum <= 90)
                            {
                                //if ((longnum >= 1 && longnum < 5) || (longnum > 15 && longnum <= 20))
                                if ((longnum >= 1 && longnum < 2) || (longnum > 2 && longnum <= 4)) //for 90 degree

                                {
                                    frontcsvwrite = true;
                                    backcsvwrite = false;
                                    //Instantiate(test_sphere_, Left_Hand_Target.transform.position, Left_Hand_Target.transform.rotation);
                                    //Left_Hand_Target_Actual.transform.position = Left_Hand_Target.transform.position;
                                    //frontspherelocations.Add(Left_Hand_Target.transform.position);
                                }

                                //if (longnum >= 5 && longnum <= 15)
                                    if (longnum >= 2 && longnum <= 3) // for 90 degree

                                    {
                                    backcsvwrite = true;
                                    frontcsvwrite = false;
                                    //Instantiate(test_sphere_, Left_Hand_Target.transform.position, Left_Hand_Target.transform.rotation);
                                    //Left_Hand_Target_Actual.transform.position = Left_Hand_Target.transform.position;
                                    //backspherelocations.Add(Left_Hand_Target.transform.position);
                                }


                            }
                            if (latnum > 90 && latnum <= 180)
                            {
                                if (longnum > 5 && longnum < 15)

                                {
                                    frontcsvwrite = true;
                                    backcsvwrite = false;
                                    //Instantiate(test_sphere_, Left_Hand_Target.transform.position, Left_Hand_Target.transform.rotation);
                                    //Left_Hand_Target_Actual.transform.position = Left_Hand_Target.transform.position;
                                    //frontspherelocations.Add(Left_Hand_Target.transform.position);
                                }
                                if ((longnum >= 1 && longnum <= 5) || (longnum >= 15 && longnum <= 20))

                                {
                                    backcsvwrite = true;
                                    frontcsvwrite = false;
                                    //Instantiate(test_sphere_, Left_Hand_Target.transform.position, Left_Hand_Target.transform.rotation);
                                    //Left_Hand_Target_Actual.transform.position = Left_Hand_Target.transform.position;
                                    //backspherelocations.Add(Left_Hand_Target.transform.position);
                                }


                            }
                        }
                        else
                        {
                            if (latnum > 180)
                            {
                                latnum = 0;
                                currentlayer += 1;
                                //Debug.Log("position before: " + this.transform.position);
                                //Left_Hand_Target.transform.position = origtargtposition - dir * (currentlayer) * (Vector3.Distance(origtargtposition, test_sphere.transform.position) / numcirclelayers);
                                Right_Hand_Target.transform.position = origtargtposition - dir * (currentlayer) * (Vector3.Distance(origtargtposition, test_sphere.transform.position) / numcirclelayers);
                                //Debug.Log("currentlayer: " + currentlayer);
                                //Debug.Log("original after: " + origtargtposition);
                                //Debug.Log("position after: " + this.transform.position);
                            }
                            //latnum += 18;
                            //latnum += 45;
                            latnum += 90;
                            //Left_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(0.0f, 1.0f, 0.0f), 18);
                            //Right_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(0.0f, 1.0f, 0.0f), 45);
                            Right_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(0.0f, 1.0f, 0.0f), 90);
                            //Instantiate(test_sphere, this.transform.position, this.transform.rotation);
                            longnum = 0;
                        }

                       


                        if (frontcsvwrite)
                        {
                            //Instantiate(test_sphere_, Left_Hand_Target.transform.position, Left_Hand_Target.transform.rotation);
                            Instantiate(test_sphere_, Right_Hand_Target.transform.position, Right_Hand_Target.transform.rotation);
                            //Right_Hand_Target.transform.position = reconvertedlistfront[targetcyclecountfront];
                            if (twotarget)
                            {
                                //Right_Hand_Target.transform.position = reconvertedlistfront[Rand.Next(0, reconvertedlistfront.Count)];
                                Left_Hand_Target.transform.position = reconvertedlistfront[Rand.Next(0, reconvertedlistfront.Count)];
                            }
                            // Debug.Log("Random number front: " + Rand.Next(0, reconvertedlistfront.Count));
                            targetcyclecountfront++;
                            targetcyclecount++;
                        }
                        if (backcsvwrite)
                        {
                            //Instantiate(test_sphere__, Left_Hand_Target.transform.position, Left_Hand_Target.transform.rotation);
                            Instantiate(test_sphere__, Right_Hand_Target.transform.position, Right_Hand_Target.transform.rotation);
                            //Right_Hand_Target.transform.position = reconvertedlistback[targetcyclecountback];
                            if (twotarget)
                            {
                                //Right_Hand_Target.transform.position = reconvertedlistback[Rand.Next(0, reconvertedlistback.Count)];
                                Left_Hand_Target.transform.position = reconvertedlistback[Rand.Next(0, reconvertedlistback.Count)];
                            }
                            //Debug.Log("Random number back: " + Rand.Next(0, reconvertedlistback.Count));
                            targetcyclecountback++;
                            targetcyclecount++;
                        }
                    }

                    // below code is used when the target is in the front hemishpere 
                    /*  if (primaryweightdummy <= 0)
                          //if (primaryweight <= 0)
                      {
                          //primaryweight = 10.0;
                         // Debug.Log("targetcount :" + targetcyclecount);
                          primaryweightdummy = 10.0;
                          targetcyclecount++;
                          if (frontcsvwrite)
                          {
                              targetcyclecountfront++;
                             // targetcyclecountback++;
                          }
                          longnum += 1;
                          if (longnum <= 20 && latnum <= 180)
                          {

                              frontcsvwrite = false;
                              Left_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(Mathf.Cos(latnum * Mathf.Deg2Rad), 0.0f, -Mathf.Sin(latnum * Mathf.Deg2Rad)), 18);

                              //this.transform.RotateAround(new Vector3(0.0f, 0.0f, 0.0f), new Vector3(0.0f, 1.0f, 0.0f), 18);
                          }
                          else
                          {
                              frontcsvwrite = false;
                              if (latnum > 180)
                              {
                                  latnum = 0;
                                  currentlayer += 1;
                                  //Debug.Log("position before: " + this.transform.position);
                                  Left_Hand_Target.transform.position = origtargtposition - dir * (currentlayer) * (Vector3.Distance(origtargtposition, test_sphere.transform.position) / numcirclelayers);
                                  //Debug.Log("currentlayer: " + currentlayer);
                                  //Debug.Log("original after: " + origtargtposition);
                                  //Debug.Log("position after: " + this.transform.position);
                              }
                              latnum += 18;
                              Left_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(0.0f, 1.0f, 0.0f), 18);
                              //Instantiate(test_sphere, this.transform.position, this.transform.rotation);
                              longnum = 0;
                          }
                          if (latnum <= 90)
                          {
                              if ((longnum >= 1 && longnum <= 5) || (longnum >= 15 && longnum <= 20))
                              {
                                  frontcsvwrite = true;
                                  Instantiate(test_sphere_, Left_Hand_Target.transform.position, Left_Hand_Target.transform.rotation);
                                  Left_Hand_Target_Actual.transform.position = Left_Hand_Target.transform.position;
                                  //frontspherelocations.Add(Left_Hand_Target.transform.position);
                              }
                          }
                          if (latnum > 90 && latnum <= 180)
                          {
                              if (longnum > 5 && longnum <= 15)
                              {
                                  frontcsvwrite = true;
                                  Instantiate(test_sphere_, Left_Hand_Target.transform.position, Left_Hand_Target.transform.rotation);
                                  Left_Hand_Target_Actual.transform.position = Left_Hand_Target.transform.position;
                                  //frontspherelocations.Add(Left_Hand_Target.transform.position);
                              }
                          }

                         /* if(targetcyclecount == 654)
                          {
                              writejson = true;
                              Debug.Log("targetcount :" + targetcyclecount);
                          }
                      } */



                    // below code is used when the target is in the back hemishpere  

                    /*   if (targetcyclecount <= 655)
                       {
                           //primaryweight = 10.0;
                           primaryweightdummy = 10.0;
                          

                       targetcyclecount++;

                       longnum += 1;
                       if (longnum <= 20 && latnum <= 180)
                       {
                           Left_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(Mathf.Cos(latnum * Mathf.Deg2Rad), 0.0f, -Mathf.Sin(latnum * Mathf.Deg2Rad)), 18);

                           //this.transform.RotateAround(new Vector3(0.0f, 0.0f, 0.0f), new Vector3(0.0f, 1.0f, 0.0f), 18);
                       }
                       else
                       {
                           if (latnum > 180)
                           {
                               latnum = 0;
                               currentlayer += 1;
                               //Debug.Log("position before: " + this.transform.position);
                               Left_Hand_Target.transform.position = origtargtposition - dir * (currentlayer) * (Vector3.Distance(origtargtposition, test_sphere.transform.position) / numcirclelayers);
                               //Debug.Log("currentlayer: " + currentlayer);
                               //Debug.Log("original after: " + origtargtposition);
                               //Debug.Log("position after: " + this.transform.position);
                           }
                           latnum += 18;
                           Left_Hand_Target.transform.RotateAround(test_sphere.transform.position, new Vector3(0.0f, 1.0f, 0.0f), 18);
                           //Instantiate(test_sphere, this.transform.position, this.transform.rotation);
                           longnum = 0;
                       }
                        if (latnum <= 90)
                           {
                               //if ((longnum >= 1 && longnum <= 5) || (longnum >= 15 && longnum <= 20))
                               if (longnum > 5 && longnum <= 15)
                               {
                                   Instantiate(test_sphere_, Left_Hand_Target.transform.position, Left_Hand_Target.transform.rotation);
                                   Left_Hand_Target_Actual.transform.position = Left_Hand_Target.transform.position;
                                   backspherelocations.Add(Left_Hand_Target.transform.position);
                               }
                           }
                           if (latnum > 90 && latnum <= 180)
                               if ((longnum >= 1 && longnum <= 5) || (longnum >= 15 && longnum <= 20))
                               //if (longnum > 5 && longnum <= 15)
                               {
                                   Instantiate(test_sphere_, Left_Hand_Target.transform.position, Left_Hand_Target.transform.rotation);
                                   Left_Hand_Target_Actual.transform.position = Left_Hand_Target.transform.position;
                                   backspherelocations.Add(Left_Hand_Target.transform.position);
                               }
                       }
                   }
                   else
                   {
                       writejson = true;
                   }*/


                   

                }
               // else
                //{
                //    if(dynamiclistcount < dynamiclist.Count-1)
                //    {
                //        currentlayer = 1;
                //        numcirclelayers = 2;

                //        dynamiclistcount++;
                //        thresholdlistcount++;
                //        dynamic = dynamiclist[dynamiclistcount];
                //        thresholdfactor = thresholdlist[thresholdlistcount];
                //        Debug.Log("dynamic " + dynamic);
                //        Debug.Log("threshold " + thresholdfactor);
                //        if (!dynamic)
                //        {
                //            primaryweight = 10;
                //        }
                //        else
                //        {
                //            primaryweightdummy = 10;
                //        }

                //        Right_Hand_Target.transform.position = initialtargetpos;

                //    }
                //    else
                //    {
                //        // dynamiclistcount = 0;
                       
                //    }
                    

                //}
            }
        
            float cosinesim = 0.0f;
            float cosinesimnum = 0.0f;
            float curvectomag = 0.0f;
            PrecaptureAnimation(Root);
            //if (Input.GetKeyDown(KeyCode.A))
            //if (!temp)
            //{

            //    for (int j = 0; j < 20; j++)
            //    {




            //        refvectomag = refvectomag + (jointo[j].transform.position.x - jointo[20].transform.position.x) * (jointo[j].transform.position.x - jointo[20].transform.position.x) + (jointo[j].transform.position.y - jointo[20].transform.position.y) * (jointo[j].transform.position.y - jointo[20].transform.position.y) + (jointo[j].transform.position.z - jointo[20].transform.position.z) * (jointo[j].transform.position.z - jointo[20].transform.position.z);
            //        refvectarray[j * 3] = (jointo[j].transform.position.x - jointo[20].transform.position.x);
            //        refvectarray[(j * 3) + 1] = (jointo[j].transform.position.y - jointo[20].transform.position.y);
            //        refvectarray[(j * 3) + 2] = (jointo[j].transform.position.z - jointo[20].transform.position.z);




            //    }
            //    refvectomag = (float)System.Math.Pow((double)refvectomag, 1.0 / (2));
            //    temp = true;

            //}
            //for (int j = 0; j < jointc.Count; j++)
            //{
            //    curvectomag = curvectomag + (jointo[j].transform.position.x - jointo[20].transform.position.x) * (jointo[j].transform.position.x - jointo[20].transform.position.x) + (jointo[j].transform.position.y - jointo[20].transform.position.y) * (jointo[j].transform.position.y - jointo[20].transform.position.y) + (jointo[j].transform.position.z - jointo[20].transform.position.z) * (jointo[j].transform.position.z - jointo[20].transform.position.z);
            //    cosinesimnum = cosinesimnum + refvectarray[j * 3] * (jointo[j].transform.position.x - jointo[20].transform.position.x) + refvectarray[(j * 3) + 1] * (jointo[j].transform.position.y - jointo[20].transform.position.y) + refvectarray[(j * 3) + 2] * (jointo[j].transform.position.z - jointo[20].transform.position.z);
            //    jointc[j].transform.rotation = jointo[j].transform.rotation;
            //    //Debug.Log(jointc[j].transform.name+"position: " + jointc[21].transform.position);
            //}
            //curvectomag = (float)System.Math.Pow((double)curvectomag, 1.0 / (2));
            //cosinesim = cosinesimnum / (curvectomag * refvectomag);
            //Debug.Log("neck position:" + jointc[21].transform.position);
            //Debug.Log("cosine similarity:" + cosinesim);
            //Debug.Log("root parent:" + jointc[20].transform.parent);
            //for (int j = 0; j < leftlegchain.Count; j++)
            //{
            //   GameObject gameobject = GameObject.Find("Joint_targets/"+ leftlegchain[j].name.ToString());

            //    Debug.Log(leftlegchain[j]);
            //}
            //for (int j = 0; j < rightlegchain.Count; j++)
            //{
            //    Debug.Log(rightlegchain[j]);
            //}
            //for (int j = 0; j < lefthandchain.Count; j++)
            //{
            //    Debug.Log(lefthandchain[j]);
            //}
            //for (int j = 0; j < righthandchain.Count; j++)
            //{
            //    Debug.Log(righthandchain[j]);
            //}
            //for (int j = 0; j < neckchain.Count; j++)
            //{
            //    Debug.Log(neckchain[j]);
            //}

        }

        void LateUpdate()
        {


            if (gameObject.name == "Root_c")
            {

                //print(gameObject.name);
                cosloss = true;


            }

            else
            {
                //print(gameObject.name);
                cosloss = false;

                //Debug.Log(elbow_pos.ToString());
            }

            PostcaptureAnimation(Root);

            UpdateData(Root);
            //Debug.Log("Solution length:" + Solution.Length);
            //if (Input.GetKeyDown(KeyCode.A))
            //if (!tempangle)
            //{

            for (int i = 0; i < Solution.Length; i++)
            {// solution is initialized in the Initialize() function
                Solution[i] = Evolution.GetModel().MotionPtrs[i].Motion.GetTargetValue(true); //MotionPtrs is a Data class to store pointers to the joint motions
                                                                                              //Debug.Log(+i);
                                                                                              //Debug.Log("joint angle" + Solution[i]);

                //refSolution[i] = Solution[i];
                //Debug.Log("ref sol:" + refSolution[i]*Utility.Rad2Deg*100000);
            }
            //tempangle = true;
            //}
            //else
            //{

            //}
            //Solution = Evolution.Optimise(Generations, Solution); 
            if (temp <= 10)
            {

                refSolution = new double[Solution.Length];
                for (int i = 0; i < Solution.Length; i++)
                {
                    refSolution[i] = Solution[i];
                }
                // Debug.Log("refsol  = " + String.Join("",
                //new List<double>(refSolution)
                //.ConvertAll(i => i.ToString())
                //.ToArray()));
                Solution = Evolution.Optimise(Generations, Solution, refSolution);


                temp++;
            }
            // Debug.Log("refsol after = " + String.Join("",
            //new List<double>(refSolution)
            //.ConvertAll(i => i.ToString())
            //.ToArray()));

            if (Input.GetKeyDown(KeyCode.Space))
            {
                Solution = Evolution.Optimise(Generations, Solution, refSolution);

                for (int i = 0; i < Solution.Length; i++)
                {
                    refSolution[i] = Solution[i];
                }
                Evolution.Model.final = true;
                Evolution.Model.ComputeLoss(Solution, refSolution);
                Evolution.Model.final = false;

            }
            //if (Input.GetKeyDown(KeyCode.I) && gameObject.name == "Root")
            //{
            //    //Debug.Log("i pressed");

            //    for (int i = 0; i < Evolution.Model.ObjectivePtrs.Length; i++)
            //    {
            //        if (Evolution.Model.ObjectivePtrs[i].Objective.transform.name == "Left_Hand_IK")
            //        {
            //            Evolution.Model.ObjectivePtrs[i].Objective.SetWeight(10);
            //            Debug.Log("weight: " + Evolution.Model.ObjectivePtrs[i].Objective.GetWeight());
            //        }
            //    }

            //}
            //if (Input.GetKeyDown(KeyCode.D) && gameObject.name == "Root")
            //{

            //    //Debug.Log("D pressed");
            //    for (int i = 0; i < Evolution.Model.ObjectivePtrs.Length; i++)
            //    {
            //        if (Evolution.Model.ObjectivePtrs[i].Objective.transform.name == "Left_Hand_IK")
            //        {
            //            Evolution.Model.ObjectivePtrs[i].Objective.SetWeight(0);
            //            Debug.Log("weight: " + Evolution.Model.ObjectivePtrs[i].Objective.GetWeight());
            //        }
            //    }
            //}

            Solution = Evolution.Optimise(Generations, Solution, refSolution);


            if (!(L2Norm(Solution) == 0 || L2Norm(refSolution) == 0))
            {
                cosanglescore = Score(refSolution, Solution);
                if (cosloss == true)
                {
                    totalcount_c += 1;
                    cosposscore_c += cosanglescore;
                    //Debug.Log("average cos score_c" + (cosposscore_c/ totalcount_c));
                    //Debug.Log("cos score_c" + cosanglescore);
                }
                else
                {
                    totalcount += 1;
                    cosposscore += cosanglescore;
                    //Debug.Log("average cos score:" + (cosposscore/totalcount));
                    //Debug.Log("cos score:" + cosanglescore);
                }
                //Debug.Log(cosloss);
                //cosloss = false;
            }
            //GameObject test = GameObject.Find("Joint_targets/Root");
            //test.transform.position = new Vector3(testv.x, testv.y, testv.z);
            for (int i = 0; i < Solution.Length; i++)
            {

                BioJoint.Motion motion = Evolution.GetModel().MotionPtrs[i].Motion;
                motion.SetTargetValue(Solution[i], true);
                /*
                if(motion.Joint.GetJointType() == JointType.Revolute) {
                    motion.SetTargetValue((float)Solution[i]);
                } else if(motion.Joint.GetJointType() == JointType.Continuous) {
                    motion.SetTargetValue(motion.GetTargetValue() + Mathf.Deg2Rad*Mathf.DeltaAngle(Mathf.Rad2Deg*motion.GetTargetValue(), Mathf.Rad2Deg*(float)Solution[i]));
                } else if(motion.Joint.GetJointType() == JointType.Prismatic) {
                    motion.SetTargetValue((float)Solution[i]);
                } else if(motion.Joint.GetJointType() == JointType.Floating) {
                    motion.SetTargetValue((float)Solution[i]);
                }
                */
            }

            ProcessMotion(Root);

            //Debug.Log("check:" + Left_Hand_IK_c.transform.position);
            //Debug.Log("left hand error:" + Vector3.Distance(Left_Hand_Target.transform.position, Left_Hand_IK.transform.position) + " left hand error_c:" + Vector3.Distance(Left_Hand_Target_c.transform.position, Left_Hand_IK_c.transform.position));
            //Debug.Log("right hand error:" + Vector3.Distance(Right_Hand_Target.transform.position, Right_Hand_IK.transform.position) + " right hand error_c:" + Vector3.Distance(Right_Hand_Target_c.transform.position, Right_Hand_IK_c.transform.position));
            //Debug.Log("left leg error:" + Vector3.Distance(Left_Foot_Target.transform.position, Left_Foot_IK.transform.position) + " left leg error_c:" + Vector3.Distance(Left_Foot_Target_c.transform.position, Left_Foot_IK_c.transform.position));
            //Debug.Log("right leg error:" + Vector3.Distance(Right_Foot_Target.transform.position, Right_Foot_IK.transform.position) + " right leg error_c:" + Vector3.Distance(Right_Foot_Target_c.transform.position, Right_Foot_IK_c.transform.position));
            //Debug.Log("eyes error:" + Vector3.Distance(Eyes_Target.transform.position, Eyes.transform.position) + " eyes error_c:" + Vector3.Distance(Eyes_Target_c.transform.position, Eyes_c.transform.position));

            //Debug.Log(cosloss);

            // Pose comparison method
            //    if (Input.GetKeyDown(KeyCode.C))
            //    {
            //        for (int j = 0; j < jointo.Count; j++)
            //        {
            //            refcord[j] = jointo[j].transform.position;
            //        }

            //    }
            //    posescore = 0;
            //    for (int j = 0; j < jointc.Count; j++)
            //    {
            //        //Vector3 tempvector = refcord[j]-refcord[0];
            //        float tempvector = (refcord[j]-refcord[0]).magnitude;
            //        //posescore = posescore + Vector3.Distance(Vector3.Distance(refcord[j],Vector3.Distance(jointo[0].transform.position,refcord[0])), jointc[j].transform.position ) *weights[j];
            //        posescore = posescore + ((refcord[j] - refcord[0]).magnitude - (jointc[j].transform.position - jointc[0].transform.position).magnitude) * weights[j];
            //        // Debug.Log(jointc[j].transform.position);
            //       // Debug.Log("pose similarity score: 1" + (refcord[j] - refcord[0]).magnitude);
            //        //Debug.Log("pose similarity score: 2" + (jointc[j].transform.position - jointc[0].transform.position).magnitude);

            //    }
            //    Debug.Log("pose similarity score: " + posescore);
            }
            public static double Score(double[] vectorArray1, double[] vectorArray2)
        {
            List<double> v1 = new List<double>(vectorArray1);
            List<double> v2 = new List<double>(vectorArray2);

            if (v1.Count == 0 || v2.Count == 0)
            {
                throw new System.ArgumentException("Cosine Similarity: Vectors cannot be zero length");
            }
            if (v1.Count != v2.Count)
            {
                throw new System.ArgumentException("Cosine Similarity: Vectors must be the same length");
            }

            double dotProduct = 0.0;
            double l2norm1 = L2Norm(v1);
            double l2norm2 = L2Norm(v2);
            if (l2norm1 == 0 || l2norm2 == 0)
            {
                throw new System.ArgumentException("Cosine Similarity: A vector cannot be a zero vector");
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

        public void SetThreading(bool enabled) {
			if(UseThreading != enabled) {
				UseThreading = enabled;
				if(Application.isPlaying) {
					Refresh();
				}
			}
		}

		public bool GetThreading() {
			return UseThreading;
		}

		public void SetGenerations(int generations) {
			Generations = generations;
		}

		public int GetGenerations() {
			return Generations;
		}

		public void SetPopulationSize(int populationSize) {
			if(PopulationSize != populationSize) {
				PopulationSize = System.Math.Max(1, populationSize);
				Elites = System.Math.Min(populationSize, Elites);
				if(Application.isPlaying) {
					Refresh();
				}
			}
		}

		public int GetPopulationSize() {
			return PopulationSize;
		}

		public void SetElites(int elites) {
			if(Elites != elites) {
				Elites = System.Math.Max(1, elites);
				if(Application.isPlaying) {
					Refresh();
				}
			}
		}

		public int GetElites() {
			return Elites;
		}

		public void ResetPosture(BioSegment segment) {
			if(segment.Joint != null) {
				segment.Joint.X.SetTargetValue(0f);
				segment.Joint.Y.SetTargetValue(0f);
				segment.Joint.Z.SetTargetValue(0f);
				if(!Application.isPlaying) {
					segment.Joint.PrecaptureAnimation();
					segment.Joint.PostcaptureAnimation();
					segment.Joint.UpdateData();
					segment.Joint.ProcessMotion();
				}
			}
			for(int i=0; i<segment.Childs.Length; i++) {
				ResetPosture(segment.Childs[i]);
			}
		}

		public BioSegment FindSegment(Transform t) {
			for(int i=0; i<Segments.Count; i++) {
				if(Segments[i].Transform == t) {
					return Segments[i];
				}
			}
			return null;
		}

		public BioSegment FindSegment(string name) {
			for(int i=0; i<Segments.Count; i++) {
				if(Segments[i].Transform.name == name) {
					return Segments[i];
				}
			}
			return null;
		}

		public List<BioSegment> GetChain(Transform start, Transform end) {
			BioSegment a = FindSegment(start);
			BioSegment b = FindSegment(end);
			if(a == null || b == null) {
				Debug.Log("Could not generate chain for given transforms");
				return null;
			}
			return GetChain(a, b);
		}

		public List<BioSegment> GetChain(BioSegment start, BioSegment end) {
			List<BioSegment> chain = new List<BioSegment>();
			BioSegment segment = end;
			while(true) {
				chain.Add(segment);
				if(segment.Transform == transform || segment.Parent == null) {
					break;
				} else {
					segment = segment.Parent;
				}
			}
			chain.Reverse();
			return chain;
		}

		public void UpdateData(BioSegment segment) {
			if(segment.Joint != null) {
				if(segment.Joint.enabled) {
					segment.Joint.UpdateData();
				}
			}
			for(int i=0; i<segment.Objectives.Length; i++) {
				if(segment.Objectives[i].enabled) {
					segment.Objectives[i].UpdateData();
				}
			}
			for(int i=0; i<segment.Childs.Length; i++) {
				UpdateData(segment.Childs[i]);
			}
		}

		public void Refresh(bool evolution = true) {
            if (Destroyed) {
				return;
			}
            //Debug.Log("Segment count:" + Segments.Count);
			for(int i=0; i<Segments.Count; i++) {
				if(Segments[i] == null) {
					Segments.RemoveAt(i);
					i--;
				}
			}
			Refresh(transform);
			Root = FindSegment(transform);

			if(evolution && Application.isPlaying) {
				DeInitialise();
				Initialise();
				Solution = new double[Evolution.GetModel().GetDoF()];
			}
		}

		private void Refresh(Transform t) {
			BioSegment segment = FindSegment(t);
			if(segment == null) {
				segment = Utility.AddBioSegment(this, t);
				Segments.Add(segment);
			}
			segment.Character = this;
			segment.RenewRelations();
			
			for(int i=0; i<t.childCount; i++) {
				Refresh(t.GetChild(i));
			}
		}

		private void PrecaptureAnimation(BioSegment segment) {
			if(segment.Joint != null) {
				if(segment.Joint.enabled) {
					segment.Joint.PrecaptureAnimation();
				}
			}
			for(int i=0; i<segment.Childs.Length; i++) {
				PrecaptureAnimation(segment.Childs[i]);
			}
		}

		private void PostcaptureAnimation(BioSegment segment) {
			if(segment.Joint != null) {
				if(segment.Joint.enabled) {
					segment.Joint.PostcaptureAnimation();
				}
			}
			for(int i=0; i<segment.Childs.Length; i++) {
				PostcaptureAnimation(segment.Childs[i]);
			}
		}

		private void ProcessMotion(BioSegment segment) {
			if(segment.Joint != null) {
				if(segment.Joint.enabled) {
					segment.Joint.ProcessMotion();
				}
			}
			for(int i=0; i<segment.Childs.Length; i++) {
				ProcessMotion(segment.Childs[i]);
			}
		}

	}

}