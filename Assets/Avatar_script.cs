using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;
using Newtonsoft.Json.Serialization;
//using UnityEngine.XR.MagicLeap;
using WebSocketSharp;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

public struct timestampedBodyData
{
    public Avatar_script.BodyData bodydata;

    public float timestamp;
}

[RequireComponent(typeof(Animator))]
public class Avatar_script : MonoBehaviour
{

    private float timer = 0.0f;
    WebSocket ws;
    public String wsipAddress = "129.59.79.27";
    public String wsport = "4969";
    public String wsservice = "AvatarData";

    [Tooltip("How high above the ground is the sensor, in meters.")]
    public float sensorHeight = 1.0f;

    [Tooltip("Index of the player, tracked by this component. 0 means the 1st player, 1 - the 2nd one, 2 - the 3rd one, etc.")]
    public int playerIndex = 0;

    [Tooltip("Whether the avatar is facing the player or not.")]
    public bool mirroredMovement = false;

    [Tooltip("Whether the avatar is allowed to move vertically or not.")]
    public bool verticalMovement = false;

    [Tooltip("Whether the avatar's root motion is applied by other component or script.")]
    public bool externalRootMotion = false;

    [Tooltip("Whether the head rotation is controlled externally (e.g. by VR-headset).")]
    public bool externalHeadRotation = false;

    [Tooltip("Whether the hand and finger rotations are controlled externally (e.g. by LeapMotion controller)")]
    public bool externalHandRotations = false;

    [Tooltip("Whether the finger orientations are allowed or not.")]
    public bool fingerOrientations = false;

    [Tooltip("Rate at which the avatar will move through the scene.")]
    public float moveRate = 1f;

    [Tooltip("Smooth factor used for avatar movements and joint rotations.")]
    public float smoothFactor = 10f;

    [Tooltip("Game object this transform is relative to (optional).")]
    public GameObject offsetNode;

    [Tooltip("If enabled, makes the avatar position relative to this camera to be the same as the player's position to the sensor.")]
    public Camera posRelativeToCamera;

    [Tooltip("Whether the avatar's position should match the color image (in Pos-rel-to-camera mode only).")]
    public bool posRelOverlayColor = false;

    [Tooltip("Plane used to render the color camera background to overlay.")]
    public Transform backgroundPlane;

    [Tooltip("Whether z-axis movement needs to be inverted (Pos-Relative mode only).")]
    public bool posRelInvertedZ = false;

    [Tooltip("Whether the avatar's feet must stick to the ground.")]
    public bool groundedFeet = false;

    [Tooltip("Whether to apply the humanoid model's muscle limits or not.")]
    public bool applyMuscleLimits = false;

    [Tooltip("Whether to flip left and right, relative to the sensor.")]
    public bool flipLeftRight = false;


    [Tooltip("Horizontal offset of the avatar with respect to the position of user's spine-base.")]
    [Range(-0.5f, 0.5f)]
    public float horizontalOffset = 0f;

    [Tooltip("Vertical offset of the avatar with respect to the position of user's spine-base.")]
    [Range(-0.5f, 0.5f)]
    public float verticalOffset = 0f;

    [Tooltip("Forward offset of the avatar with respect to the position of user's spine-base.")]
    [Range(-0.5f, 0.5f)]
    public float forwardOffset = 0f;

    [Tooltip("Whether to utilize only the really tracked joints (and ignore the inferred ones) or not.")]
    public bool ignoreInferredJoints = false;

    // userId of the player
    [NonSerialized]
    public Int64 playerId = 0;


    timestampedBodyData bodyframe;

    public bool walk = false;
    public static bool humantestfinishstatus = false;
    public bool temp = false;
    public bool scriptforbioik = true;
    public static bool updateready = false;
    public int curIndex = 0;
    List<string> reconvertedlist;
    List<timestampedBodyData> playbackFrames = new List<timestampedBodyData>();

    int num = 0;

    public enum HandEventType : int
    {
        None = 0,
        Grip = 1,
        Release = 2
    }

    // The body root node
    protected Transform bodyRoot;

    // Variable to hold all them bones. It will initialize the same size as initialRotations.
    protected Transform[] bones;
    protected Transform[] prevbones;
    protected Transform prevhead;
    protected Transform[] fingerBones;


    // Rotations of the bones when the Kinect tracking starts.
    protected Quaternion[] initialRotations;
    protected Quaternion[] localRotations;
    protected bool[] isBoneDisabled;

    // Local rotations of finger bones
    protected Dictionary<HumanBodyBones, Quaternion> fingerBoneLocalRotations = new Dictionary<HumanBodyBones, Quaternion>();
    protected Dictionary<HumanBodyBones, Vector3> fingerBoneLocalAxes = new Dictionary<HumanBodyBones, Vector3>();

    // Initial position and rotation of the transform
    protected Vector3 initialPosition;
    protected Quaternion initialRotation;
    protected Vector3 initialHipsPosition;
    protected Quaternion initialHipsRotation;

    protected Vector3 offsetNodePos;
    protected Quaternion offsetNodeRot;
    protected Vector3 bodyRootPosition;
    private Vector3 _heading;
    public Component[] allcomponents;

    // Calibration Offset Variables for Character Position.
    [NonSerialized]
    public bool offsetCalibrated = false;
    protected Vector3 offsetPos = Vector3.zero;
    //protected float xOffset, yOffset, zOffset;
    //private Quaternion originalRotation;

    private Animator animatorComponent = null;
    private HumanPoseHandler humanPoseHandler = null;
    private HumanPose humanPose = new HumanPose();

    // whether the parent transform obeys physics
    protected bool isRigidBody = false;

    // private instance of the KinectManager
    //protected KinectManager kinectManager;

    // last hand events
    private HandEventType lastLeftHandEvent = HandEventType.Release;
    private HandEventType lastRightHandEvent = HandEventType.Release;

    // fist states
    private bool bLeftFistDone = false;
    private bool bRightFistDone = false;

    // grounder constants and variables
    private const int raycastLayers = ~2;  // Ignore Raycast
    private const float maxFootDistanceGround = 0.02f;  // maximum distance from lower foot to the ground
    private const float maxFootDistanceTime = 0.2f; // 1.0f;  // maximum allowed time, the lower foot to be distant from the ground
    private Transform leftFoot, rightFoot;

    private float fFootDistanceInitial = 0f;
    private float fFootDistance = 0f;
    private float fFootDistanceTime = 0f;

    public float speed = 1.0f;

    // background plane rectangle
    private Rect planeRect = new Rect();
    private bool planeRectSet = false;

    public GameObject indexlefttip;
    public GameObject indexlefttop;
    public GameObject indexleftmiddle;
    public GameObject leftwristpoint;
    public GameObject wristleft;
    public GameObject wristright;
    public GameObject indextiptest;
    public GameObject indextipprevtest;
    //public GameObject lefteyewoman;
    //public GameObject righteyewoman;
    //public GameObject lefteyewomana;
    //public GameObject righteyewomana;
    public GameObject lefteyeman;
    public GameObject righteyeman;
    public GameObject lefteyemana;
    public GameObject righteyemana;
    //public GameObject womanneck;
    public GameObject manneck;
    public GameObject womanlineleft;
    public GameObject womanlineright;
    public GameObject dummytarget;
    public GameObject dummytargetprev;


    public Vector3 previndextip;
    public Vector3 previndexmiddle;
    public Vector3 previndexbottom;
    public Quaternion leftcorrectwoman;
    public Quaternion rightcorrectwoman;
    public Quaternion leftcorrectman;
    public Quaternion rightcorrectman;
    //public Quaternion neckcorrectwoman;
    //public Quaternion neckinitialwoman;
    public Quaternion neckinitialman;
    public GameObject rightIKhint;
    public GameObject rightIKposition;
    public GameObject rightelbow;
    public GameObject righthand;
    public Vector3 correctionvec;
    public Vector3 correctposvec;




    public bool initialset = false;

    BodyData bodyData;

    public bool Hips = false;
    public bool Spine = false;
    public bool Neck = false;
    public bool LeftUpperArm = false;
    public bool LeftLowerArm = false;
    public bool LeftHand = false;
    public bool RightUpperArm = false;
    public bool RightLowerArm = false;
    public bool RightHand = false;
    public bool LeftShoulder = false;
    public bool RightShoulder = false;
    public bool LeftIndexProximal = false;
    public bool RightIndexProximal = false;
    public bool LeftThumbProximal = false;
    public bool RightThumbProximal = false;


    // public static Transform bonetransformtemp;
    public Transform prev;
    public float[] floatx;
    public float[] floaty;
    public float[] floatz;
    public float[] floatw;


    public int update = 0;
    public int angle = 0;
    // Start is called before the first frame update


    public enum TrackingState
    {
        NotTracked = 0,
        Inferred = 1,
        Tracked = 2
    }

    public enum HandState
    {
        Unknown = 0,
        NotTracked = 1,
        Open = 2,
        Closed = 3,
        Lasso = 4
    }

    public enum TrackingConfidence
    {
        Low = 0,
        High = 1
    }
    /// <summary>
    /// Container for the body-joint data.
    /// </summary>
    public struct JointData
    {
        // parameters filled in by the sensor interface
        //public JointType jointType;
        public TrackingState trackingState;
        public Vector3 kinectPos;
        public Vector3 position;
        public Quaternion orientation;  // deprecated

        public Vector3 posPrev;
        public Vector3 posRel;
        public Vector3 posVel;

        // KM calculated parameters
        public Vector3 direction;
        public Quaternion normalRotation;
        public Quaternion mirroredRotation;

        // Constraint parameters
        public float lastAngle;



        public JointData(int size)
        {
            this.trackingState = new TrackingState();
            this.kinectPos = new Vector3();
            this.position = new Vector3();
            this.orientation = new Quaternion();
            this.posPrev = new Vector3();
            this.posRel = new Vector3();
            this.posVel = new Vector3();
            this.direction = new Vector3();
            this.normalRotation = new Quaternion();
            this.mirroredRotation = new Quaternion();
            this.lastAngle = new float();

        }
    }

    public enum JointType : int
    {
        SpineBase = 0,
        SpineMid = 1,
        Neck = 2,
        Head = 3,
        ShoulderLeft = 4,
        ElbowLeft = 5,
        WristLeft = 6,
        HandLeft = 7,
        ShoulderRight = 8,
        ElbowRight = 9,
        WristRight = 10,
        HandRight = 11,
        HipLeft = 12,
        KneeLeft = 13,
        AnkleLeft = 14,
        FootLeft = 15,
        HipRight = 16,
        KneeRight = 17,
        AnkleRight = 18,
        FootRight = 19,
        SpineShoulder = 20,
        HandTipLeft = 21,
        ThumbLeft = 22,
        HandTipRight = 23,
        ThumbRight = 24
        //Count = 25
    }


    /// <summary>
    /// Container for the body data.
    /// </summary>
    public struct BodyData
    {
        // parameters filled in by the sensor interface
        public Int64 liTrackingID;
        public Vector3 position;
        public Quaternion orientation;  // deprecated

        public JointData[] joint;

        public BodyData(int size)
        {
            this.joint = new JointData[size];
            this.liTrackingID = new Int64();
            this.position = new Vector3();
            this.orientation = new Quaternion();
            this.normalRotation = new Quaternion();
            this.mirroredRotation = new Quaternion();
            this.hipsDirection = new Vector3();
            this.shouldersDirection = new Vector3();
            this.bodyTurnAngle = new float();
            this.isTurnedAround = new bool();
            this.turnFaceLastTrackedTime = new float();
            this.turnLeftShoulderTrackedTime = new float();
            this.turnRightShoulderTrackedTime = new float();
            this.turnShoulderDistTrackedTime = new float();
            this.leftHandOrientation = new Quaternion();
            this.rightHandOrientation = new Quaternion();
            this.headOrientation = new Quaternion();
            this.leftHandState = new HandState();
            this.leftHandConfidence = new TrackingConfidence();
            this.rightHandState = new HandState();
            this.rightHandConfidence = new TrackingConfidence();
            this.dwClippedEdges = new uint();
            this.bIsTracked = new short();
            this.bIsRestricted = new short();






        }

        // KM calculated parameters
        public Quaternion normalRotation;
        public Quaternion mirroredRotation;

        public Vector3 hipsDirection;
        public Vector3 shouldersDirection;
        public float bodyTurnAngle;
        //public float bodyFullAngle;
        //public float turnAroundFactor;
        public bool isTurnedAround;
        public float turnFaceLastTrackedTime;
        public float turnLeftShoulderTrackedTime;
        public float turnRightShoulderTrackedTime;
        public float turnShoulderDistTrackedTime;

        public Quaternion leftHandOrientation;
        public Quaternion rightHandOrientation;

        public Quaternion headOrientation;

        //		public Vector3 leftArmDirection;
        //		public Vector3 leftThumbForward;
        //		public Vector3 leftThumbDirection;
        //		//public float leftThumbAngle;
        //
        //		public Vector3 rightArmDirection;
        //		public Vector3 rightThumbForward;
        //		public Vector3 rightThumbDirection;
        //		//public float rightThumbAngle;

        //public Vector3 leftLegDirection;
        //public Vector3 leftFootDirection;
        //public Vector3 rightLegDirection;
        //public Vector3 rightFootDirection;

        public HandState leftHandState;
        public TrackingConfidence leftHandConfidence;
        public HandState rightHandState;
        public TrackingConfidence rightHandConfidence;

        public uint dwClippedEdges;
        public short bIsTracked;
        public short bIsRestricted;
    }

    /// <summary>
    /// Gets the number of bone transforms (array length).
    /// </summary>
    /// <returns>The number of bone transforms.</returns>
    public int GetBoneTransformCount()
    {
        return bones != null ? bones.Length : 0;
    }

    /// <summary>
    /// Gets the bone transform by index.
    /// </summary>
    /// <returns>The bone transform.</returns>
    /// <param name="index">Index</param>
    public Transform GetBoneTransform(int index)
    {
        if (index >= 0 && bones != null && index < bones.Length)
        {
            return bones[index];
        }

        return null;
    }

    /// <summary>
    /// Get joint position with respect of player world and kinect offsets   ( //!!still some problems with accurate Y pos, probably connected with kinect sensor height estimation ) 
    /// </summary>
    /// <param name="jointType"></param>
    /// <returns></returns>
    public Vector3 GetJointWorldPos(JointType jointType)
    {
        //if (!kinectManager)
        //{
        //    return Vector3.zero;
        //}

        Vector3 jointPosition = GetJointPosition(playerId, (int)jointType);
        Vector3 worldPosition = new Vector3(
            jointPosition.x - offsetPos.x,
        //            jointPosition.y - offsetPos.y + kinectManager.sensorHeight,  //!! this should be better investigated .. 
        //jointPosition.y + offsetPos.y - kinectManager.sensorHeight,  //!! this workds better on my example 
        //!mirroredMovement && !posRelativeToCamera ? (-jointPosition.z - offsetPos.z) : (jointPosition.z - offsetPos.z));
        jointPosition.y + offsetPos.y - sensorHeight,  //!! this workds better on my example 
            !mirroredMovement && !posRelativeToCamera ? (-jointPosition.z - offsetPos.z) : (jointPosition.z - offsetPos.z));

        Quaternion posRotation = mirroredMovement ? Quaternion.Euler(0f, 180f, 0f) * initialRotation : initialRotation;
        worldPosition = posRotation * worldPosition;

        return bodyRootPosition + worldPosition;
    }

    public Vector3 GetJointPosition(Int64 userId, int joint)
    {
        //if (dictUserIdToIndex.ContainsKey(userId))
        //{
        //    int index = dictUserIdToIndex[userId];

        //    if (index >= 0 && index < sensorData.bodyCount &&
        //        bodyFrame.bodyData[index].bIsTracked != 0)
        //    {
        if (joint >= 0 && joint < 25)
        {
            JointData jointData = bodyData.joint[joint];
            return jointData.position;
        }
        //}
        //}

        return Vector3.zero;
    }

    /// <summary>
    /// Disables the bone and optionally resets its orientation.
    /// </summary>
    /// <param name="index">Bone index.</param>
    /// <param name="resetBone">If set to <c>true</c> resets bone orientation.</param>
    public void DisableBone(int index, bool resetBone)
    {
        if (index >= 0 && index < bones.Length)
        {
            isBoneDisabled[index] = true;

            if (resetBone && bones[index] != null)
            {
                bones[index].rotation = localRotations[index];
            }
        }
    }

    /// <summary>
    /// Enables the bone, so AvatarController could update its orientation.
    /// </summary>
    /// <param name="index">Bone index.</param>
    public void EnableBone(int index)
    {
        if (index >= 0 && index < bones.Length)
        {
            isBoneDisabled[index] = false;
        }
    }

    /// <summary>
    /// Determines whether the bone orientation update is enabled or not.
    /// </summary>
    /// <returns><c>true</c> if the bone update is enabled; otherwise, <c>false</c>.</returns>
    /// <param name="index">Bone index.</param>
    public bool IsBoneEnabled(int index)
    {
        if (index >= 0 && index < bones.Length)
        {
            return !isBoneDisabled[index];
        }

        return false;
    }

    /// <summary>
    /// Gets the bone index by joint type.
    /// </summary>
    /// <returns>The bone index.</returns>
    /// <param name="joint">Joint type</param>
    /// <param name="bMirrored">If set to <c>true</c> gets the mirrored joint index.</param>
    public int GetBoneIndexByJoint(JointType joint, bool bMirrored)
    {
        int boneIndex = -1;

        if (jointMap2boneIndex.ContainsKey(joint))
        {
            boneIndex = !bMirrored ? jointMap2boneIndex[joint] : mirrorJointMap2boneIndex[joint];
        }

        return boneIndex;
    }

    /// <summary>
    /// Gets the special index by two joint types.
    /// </summary>
    /// <returns>The spec index by joint.</returns>
    /// <param name="joint1">Joint 1 type.</param>
    /// <param name="joint2">Joint 2 type.</param>
    /// <param name="bMirrored">If set to <c>true</c> gets the mirrored joint index.</param>
    public int GetSpecIndexByJoint(JointType joint1, JointType joint2, bool bMirrored)
    {
        int boneIndex = -1;

        if ((joint1 == JointType.ShoulderLeft && joint2 == JointType.SpineShoulder) ||
           (joint2 == JointType.ShoulderLeft && joint1 == JointType.SpineShoulder))
        {
            return (!bMirrored ? 25 : 26);
        }
        else if ((joint1 == JointType.ShoulderRight && joint2 == JointType.SpineShoulder) ||
                (joint2 == JointType.ShoulderRight && joint1 == JointType.SpineShoulder))
        {
            return (!bMirrored ? 26 : 25);
        }
        else if ((joint1 == JointType.HandTipLeft && joint2 == JointType.HandLeft) ||
                (joint2 == JointType.HandTipLeft && joint1 == JointType.HandLeft))
        {
            return (!bMirrored ? 27 : 28);
        }
        else if ((joint1 == JointType.HandTipRight && joint2 == JointType.HandRight) ||
                (joint2 == JointType.HandTipRight && joint1 == JointType.HandRight))
        {
            return (!bMirrored ? 28 : 27);
        }
        else if ((joint1 == JointType.ThumbLeft && joint2 == JointType.HandLeft) ||
                (joint2 == JointType.ThumbLeft && joint1 == JointType.HandLeft))
        {
            return (!bMirrored ? 29 : 30);
        }
        else if ((joint1 == JointType.ThumbRight && joint2 == JointType.HandRight) ||
                (joint2 == JointType.ThumbRight && joint1 == JointType.HandRight))
        {
            return (!bMirrored ? 30 : 29);
        }

        return boneIndex;
    }

    /// <summary>
    /// Gets the number of finger bone transforms (array length).
    /// </summary>
    /// <returns>The number of finger bone transforms.</returns>
    public int GetFingerTransformCount()
    {
        return fingerBones != null ? fingerBones.Length : 0;
    }

    /// <summary>
    /// Gets the finger bone transform by index.
    /// </summary>
    /// <returns>The finger bone transform.</returns>
    /// <param name="index">Index</param>
    public Transform GetFingerTransform(int index)
    {
        if (index >= 0 && index < fingerBones.Length)
        {
            return fingerBones[index];
        }

        return null;
    }


    // transform caching gives performance boost since Unity calls GetComponent<Transform>() each time you call transform 
    private Transform _transformCache;
    public new Transform transform
    {
        get
        {
            if (!_transformCache)
            {
                _transformCache = base.transform;
            }

            return _transformCache;
        }
    }

    public void Awake()
    {
        ws = new WebSocket("ws://" + wsipAddress + ":" + wsport + "/" + wsservice);
        // check for double start
        if (bones != null)
            return;
        if (!gameObject.activeInHierarchy)
            return;

        // inits the bones array
        bones = new Transform[31];
        prevbones = new Transform[31];
        floatx = new float[31];
        floaty = new float[31];
        floatz = new float[31];
        floatw = new float[31];

        // get the animator reference
        //if (!(this.gameObject == GameObject.Find("BusinessWomanPFB")))
        {
            animatorComponent = GetComponent<Animator>();

        }
        // Map bones to the points the Kinect tracks
        MapBones();

        // Set model's arms to be in T-pose, if needed
        SetModelArmsInTpose();

        // Initial rotations and directions of the bones.
        initialRotations = new Quaternion[bones.Length];
        localRotations = new Quaternion[bones.Length];
        isBoneDisabled = new bool[bones.Length];

        // Get initial bone rotations
        GetInitialRotations();

        // enable all bones
        for (int i = 0; i < bones.Length; i++)
        {
            isBoneDisabled[i] = false;
        }

        // get initial distance to ground
        fFootDistanceInitial = GetDistanceToGround();
        fFootDistance = 0f;
        fFootDistanceTime = 0f;

        // if parent transform uses physics
        isRigidBody = (gameObject.GetComponent<Rigidbody>() != null);

        // get the pose handler reference
        if (animatorComponent && animatorComponent.avatar && animatorComponent.avatar.isHuman)
        {
            //Transform hipsTransform = animator.GetBoneTransform(HumanBodyBones.Hips);
            //Transform rootTransform = hipsTransform.parent;
            Transform rootTransform = transform;

            humanPoseHandler = new HumanPoseHandler(animatorComponent.avatar, rootTransform);
            humanPoseHandler.GetHumanPose(ref humanPose);

            initialHipsPosition = (humanPose.bodyPosition - rootTransform.position);  // hipsTransform.position
            initialHipsRotation = humanPose.bodyRotation;
        }

        //Debug.Log("bones_length"+bones.Length);
    }


    // Capture the initial rotations of the bones
    protected void GetInitialRotations()
    {
        // save the initial rotation
        if (offsetNode != null)
        {
            offsetNodePos = offsetNode.transform.position;
            offsetNodeRot = offsetNode.transform.rotation;
        }

        initialPosition = transform.position;
        initialRotation = transform.rotation;

        //		initialHipsPosition = bones[0] ? bones[0].localPosition : Vector3.zero;
        //		initialHipsRotation = bones[0] ? bones[0].localRotation : Quaternion.identity;

        //		if(offsetNode != null)
        //		{
        //			initialRotation = Quaternion.Inverse(offsetNodeRot) * initialRotation;
        //		}

        transform.rotation = Quaternion.identity;

        // save the body root initial position
        if (bodyRoot != null)
        {
            bodyRootPosition = bodyRoot.position;
        }
        else
        {
            bodyRootPosition = transform.position;
        }

        if (offsetNode != null)
        {
            bodyRootPosition = bodyRootPosition - offsetNodePos;
        }

        // save the initial bone rotations
        for (int i = 0; i < bones.Length; i++)
        {
            if (bones[i] != null)
            {
                initialRotations[i] = bones[i].rotation;
                localRotations[i] = bones[i].localRotation;
            }
        }

        // get finger bones' local rotations
        //Animator animatorComponent = GetComponent<Animator>();

        foreach (int boneIndex in specialIndex2MultiBoneMap.Keys)
        {
            List<HumanBodyBones> alBones = specialIndex2MultiBoneMap[boneIndex];
            //Transform handTransform = animatorComponent.GetBoneTransform((boneIndex == 27 || boneIndex == 29) ? HumanBodyBones.LeftHand : HumanBodyBones.RightHand);

            for (int b = 0; b < alBones.Count; b++)
            {
                HumanBodyBones bone = alBones[b];
                Transform boneTransform = animatorComponent ? animatorComponent.GetBoneTransform(bone) : null;

                // get the finger's 1st transform
                Transform fingerBaseTransform = animatorComponent ? animatorComponent.GetBoneTransform(alBones[b - (b % 3)]) : null;
                //Vector3 vBoneDirParent = handTransform && fingerBaseTransform ? (handTransform.position - fingerBaseTransform.position).normalized : Vector3.zero;

                // get the finger's 2nd transform
                Transform baseChildTransform = fingerBaseTransform && fingerBaseTransform.childCount > 0 ? fingerBaseTransform.GetChild(0) : null;
                Vector3 vBoneDirChild = baseChildTransform && fingerBaseTransform ? (baseChildTransform.position - fingerBaseTransform.position).normalized : Vector3.zero;
                Vector3 vOrthoDirChild = Vector3.Cross(vBoneDirChild, Vector3.up).normalized;

                if (boneTransform)
                {
                    fingerBoneLocalRotations[bone] = boneTransform.localRotation;

                    if (vBoneDirChild != Vector3.zero)
                    {
                        fingerBoneLocalAxes[bone] = boneTransform.InverseTransformDirection(vOrthoDirChild).normalized;
                    }
                    else
                    {
                        fingerBoneLocalAxes[bone] = Vector3.zero;
                    }

                    //					Transform bparTransform = boneTransform ? boneTransform.parent : null;
                    //					Transform bchildTransform = boneTransform && boneTransform.childCount > 0 ? boneTransform.GetChild(0) : null;
                    //
                    //					// get the finger base transform (1st joint)
                    //					Transform fingerBaseTransform = animatorComponent.GetBoneTransform(alBones[b - (b % 3)]);
                    //					Vector3 vBoneDir2 = (handTransform.position - fingerBaseTransform.position).normalized;
                    //
                    //					// set the fist rotation
                    //					if(boneTransform && fingerBaseTransform && handTransform)
                    //					{
                    //						Vector3 vBoneDir = bchildTransform ? (bchildTransform.position - boneTransform.position).normalized :
                    //							(bparTransform ? (boneTransform.position - bparTransform.position).normalized : Vector3.zero);
                    //
                    //						Vector3 vOrthoDir = Vector3.Cross(vBoneDir2, vBoneDir).normalized;
                    //						fingerBoneLocalAxes[bone] = boneTransform.InverseTransformDirection(vOrthoDir);
                    //					}
                }
            }
        }

        // Restore the initial rotation
        transform.rotation = initialRotation;
    }



    // Set model's arms to be in T-pose
    protected void SetModelArmsInTpose()
    {
        Vector3 vTposeLeftDir = transform.TransformDirection(Vector3.left);
        Vector3 vTposeRightDir = transform.TransformDirection(Vector3.right);

        Transform transLeftUarm = GetBoneTransform(GetBoneIndexByJoint(JointType.ShoulderLeft, false)); // animator.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        Transform transLeftLarm = GetBoneTransform(GetBoneIndexByJoint(JointType.ElbowLeft, false)); // animator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
        Transform transLeftHand = GetBoneTransform(GetBoneIndexByJoint(JointType.WristLeft, false)); // animator.GetBoneTransform(HumanBodyBones.LeftHand);

        if (transLeftUarm != null && transLeftLarm != null)
        {
            Vector3 vUarmLeftDir = transLeftLarm.position - transLeftUarm.position;
            float fUarmLeftAngle = Vector3.Angle(vUarmLeftDir, vTposeLeftDir);

            if (Mathf.Abs(fUarmLeftAngle) >= 5f)
            {
                Quaternion vFixRotation = Quaternion.FromToRotation(vUarmLeftDir, vTposeLeftDir);
                transLeftUarm.rotation = vFixRotation * transLeftUarm.rotation;
            }

            if (transLeftHand != null)
            {
                Vector3 vLarmLeftDir = transLeftHand.position - transLeftLarm.position;
                float fLarmLeftAngle = Vector3.Angle(vLarmLeftDir, vTposeLeftDir);

                if (Mathf.Abs(fLarmLeftAngle) >= 5f)
                {
                    Quaternion vFixRotation = Quaternion.FromToRotation(vLarmLeftDir, vTposeLeftDir);
                    transLeftLarm.rotation = vFixRotation * transLeftLarm.rotation;
                }
            }
        }

        Transform transRightUarm = GetBoneTransform(GetBoneIndexByJoint(JointType.ShoulderRight, false)); // animator.GetBoneTransform(HumanBodyBones.RightUpperArm);
        Transform transRightLarm = GetBoneTransform(GetBoneIndexByJoint(JointType.ElbowRight, false)); // animator.GetBoneTransform(HumanBodyBones.RightLowerArm);
        Transform transRightHand = GetBoneTransform(GetBoneIndexByJoint(JointType.WristRight, false)); // animator.GetBoneTransform(HumanBodyBones.RightHand);

        if (transRightUarm != null && transRightLarm != null)
        {
            Vector3 vUarmRightDir = transRightLarm.position - transRightUarm.position;
            float fUarmRightAngle = Vector3.Angle(vUarmRightDir, vTposeRightDir);

            if (Mathf.Abs(fUarmRightAngle) >= 5f)
            {
                Quaternion vFixRotation = Quaternion.FromToRotation(vUarmRightDir, vTposeRightDir);
                transRightUarm.rotation = vFixRotation * transRightUarm.rotation;
            }

            if (transRightHand != null)
            {
                Vector3 vLarmRightDir = transRightHand.position - transRightLarm.position;
                float fLarmRightAngle = Vector3.Angle(vLarmRightDir, vTposeRightDir);

                if (Mathf.Abs(fLarmRightAngle) >= 5f)
                {
                    Quaternion vFixRotation = Quaternion.FromToRotation(vLarmRightDir, vTposeRightDir);
                    transRightLarm.rotation = vFixRotation * transRightLarm.rotation;
                }
            }
        }

    }

    // If the bones to be mapped have been declared, map that bone to the model.
    protected virtual void MapBones()
    {
        //		// make OffsetNode as a parent of model transform.
        //		offsetNode = new GameObject(name + "Ctrl") { layer = transform.gameObject.layer, tag = transform.gameObject.tag };
        //		offsetNode.transform.position = transform.position;
        //		offsetNode.transform.rotation = transform.rotation;
        //		offsetNode.transform.parent = transform.parent;

        //		// take model transform as body root
        //		transform.parent = offsetNode.transform;
        //		transform.localPosition = Vector3.zero;
        //		transform.localRotation = Quaternion.identity;

        //bodyRoot = transform;

        // get bone transforms from the animator component
        //Animator animatorComponent = GetComponent<Animator>();
        //prevhead = animatorComponent ? animatorComponent.GetBoneTransform(boneIndex2MecanimMap[3]) : null;
        for (int boneIndex = 0; boneIndex < bones.Length; boneIndex++)
        {
            if (!boneIndex2MecanimMap.ContainsKey(boneIndex))
                continue;

            bones[boneIndex] = animatorComponent ? animatorComponent.GetBoneTransform(boneIndex2MecanimMap[boneIndex]) : null;
            prevbones[boneIndex] = animatorComponent ? animatorComponent.GetBoneTransform(boneIndex2MecanimMap[boneIndex]) : null;
        }

        // map finger bones, too
        fingerBones = new Transform[fingerIndex2MecanimMap.Count];

        for (int boneIndex = 0; boneIndex < fingerBones.Length; boneIndex++)
        {
            if (!fingerIndex2MecanimMap.ContainsKey(boneIndex))
                continue;

            fingerBones[boneIndex] = animatorComponent ? animatorComponent.GetBoneTransform(fingerIndex2MecanimMap[boneIndex]) : null;
            //prevfingerBones[boneIndex] = animatorComponent ? animatorComponent.GetBoneTransform(fingerIndex2MecanimMap[boneIndex]) : null;
        }
    }
    void Start()
    {
        //MLEyes.Start();
        //MLHands.Start();
        scriptforbioik = true;
        if (scriptforbioik)
        {
            reconvertedlist = JsonConvert.DeserializeObject<List<string>>(File.ReadAllText(@"C:/Users/ullala/Desktop/2021plan/human_bioik_ismar_paper_test/kinectrecordedposejson_akshith_partcipant.json")); // To Deserialise
            Debug.Log("reconverted list : " + reconvertedlist.Count);
            for (int i = 0; i < reconvertedlist.Count; i++)
            {
                playbackFrames.Add(processJSON(reconvertedlist[i]));
            }
            Debug.Log("Total body frames to play back: " + playbackFrames.Count);
        }
        if (GameObject.Find("SkinnyGuy") != null)
        {

            leftcorrectman = lefteyeman.transform.rotation * Quaternion.Inverse(lefteyemana.transform.rotation);
            rightcorrectman = righteyeman.transform.rotation * Quaternion.Inverse(righteyemana.transform.rotation);
            neckinitialman = Camera.main.transform.rotation * Quaternion.Inverse(manneck.transform.rotation);
            GameObject indextipright = GameObject.Find("SkinnyGuy/Group/Main/DeformationSystem/Root_M/Spine1_M/Chest_M/Scapula_R/Shoulder_R/Elbow_R/Wrist_R/IndexFinger1_R/IndexFinger2_R/IndexFinger3_R");
            GameObject elbowright = GameObject.Find("SkinnyGuy/Group/Main/DeformationSystem/Root_M/Spine1_M/Chest_M/Scapula_R/Shoulder_R/Elbow_R");
            //Debug.Log("index tip:" + indextipright.transform.position);
            float sphereradius = Vector3.Distance(indextipright.transform.position, elbowright.transform.position);
            SphereCollider elbowCollider = elbowright.GetComponent<SphereCollider>();
            elbowCollider.radius = sphereradius;
            //Debug.Log("collider radius:"+sphereradius);


        }

        //manavatar.GetComponents<MonoBehaviour>;


        //if (GameObject.Find("BusinessWomanPFB") != null)
        //{
        //     leftcorrectwoman = lefteyewoman.transform.rotation * Quaternion.Inverse(lefteyewomana.transform.rotation);
        //     rightcorrectwoman = righteyewoman.transform.rotation * Quaternion.Inverse(righteyewomana.transform.rotation);
        //     neckinitialwoman = Camera.main.transform.rotation * Quaternion.Inverse(womanneck.transform.rotation);
        //}

        //Debug.Log("left correction start:" + leftcorrectman);
        //lefteyewoman = GameObject.Find("BusinessWomanPFB/root/pelvis/spine_01/spine_02/spine_03/neck_01/head/eye_l/leye");
        //righteyewoman = GameObject.Find("BusinessWomanPFB/root/pelvis/spine_01/spine_02/spine_03/neck_01/head/eye_r/reye");
        //lefteyewoman.transform.LookAt(Camera.main.transform);
        //righteyewoman.transform.LookAt(Camera.main.transform);
        //lefteyewomana = GameObject.Find("BusinessWomanPFB/root/pelvis/spine_01/spine_02/spine_03/neck_01/head/eye_l");
        //righteyewomana = GameObject.Find("BusinessWomanPFB/root/pelvis/spine_01/spine_02/spine_03/neck_01/head/eye_r");

        //leftcorrectman = lefteyewoman.transform.rotation * Quaternion.Inverse(lefteyewomana.transform.rotation);
        //rightcorrectman = righteyewoman.transform.rotation * Quaternion.Inverse(righteyewomana.transform.rotation);
        //lefteyewomana.transform.rotation = lefteyewoman.transform.rotation;
        //righteyewomana.transform.rotation = righteyewoman.transform.rotation;
        //if (MLHands.Left.IsVisible) {
        //    previndextip.position= MLHands.Left.Index.KeyPoints[0].Position;
        //    previndexmiddle.position = MLHands.Left.Index.KeyPoints[1].Position;
        //    previndexbottom.position = MLHands.Left.Index.KeyPoints[2].Position;
        //    initialset = true;
        //}

    }

    // Update is called once per frame
    void Update()
    {

        //var fps = 1.0 / Time.deltaTime;
        //Debug.Log("frame rate : " + fps);
        //GameObject originalGameObject = GameObject.Find("U_CharacterFront/U/joint_Char/joint_Pelvis/joint_TorsoA/joint_TorsoB/joint_TorsoC/joint_Neck");
        //GameObject womenneck = GameObject.Find("BusinessWomanPFB/root/pelvis/spine_01/spine_02/spine_03/neck_01");
        //GameObject manneck = GameObject.Find("BlueCollarGuy_LOD0/Group/Main/DeformationSystem/Root_M/Spine1_M/Chest_M/Neck_M");
        //GameObject head = originalGameObject.transform.FindChild("U/joint_Char/joint_Pelvis/joint_TorsoA/joint_TorsoB/joint_TorsoC/joint_Neck").gameObject;
        //Debug.Log("before:" + head.transform.rotation);
        //originalGameObject.transform.rotation = Quaternion.Euler(Camera.main.transform.rotation.eulerAngles.x, Camera.main.transform.rotation.eulerAngles.y-90, Camera.main.transform.rotation.eulerAngles.z);
        //womanneck.transform.rotation = Quaternion.Euler(Camera.main.transform.rotation.eulerAngles.x, Camera.main.transform.rotation.eulerAngles.y - 90, Camera.main.transform.rotation.eulerAngles.z);
        //manneck.transform.rotation = Quaternion.Euler(Camera.main.transform.rotation.eulerAngles.x, Camera.main.transform.rotation.eulerAngles.y - 90, Camera.main.transform.rotation.eulerAngles.z);
        //originalGameObject.transform.rotation = Quaternion.Euler(0, -90, 0);
        //head.transform.rotation = Camera.main.transform.rotation;
        //Debug.Log("after:" + head.transform.rotation);
        //head.transform.rotation = Quaternion.Slerp(prevhead.rotation, Camera.main.transform.rotation, smoothFactor * Time.deltaTime);
        // prevhead.rotation = head.transform.rotation;
        //Debug.Log(head.transform.rotation);
        //Debug.Log(Camera.main.velocity);
        //if ((Camera.main.velocity.x > 0) || (Camera.main.velocity.y > 0) || (Camera.main.velocity.z > 0))
        //if (Input.GetKeyDown(KeyCode.Space))
        //{
        //    walk = true;
        //}
        //if (Input.GetKeyDown(KeyCode.KeypadEnter))
        //{
        //    walk = false;
        //}
        //if (walk)
        //{
        //    animatorComponent.SetBool("walk", true);

        //}
        //if(walk == false)
        //{
        //    animatorComponent.SetBool("walk", false);
        //}
        //Debug.Log(walk);
        //animatorComponent.SetBool("walk", true);
        //animatorComponent.SetFloat("speedx",1.0f);
        //float offset = GameObject.Find("joint_EyeLT").transform.position.y - GameObject.Find("U_CharacterFront").transform.position.y;
        //gameObject.transform.position = new Vector3(Camera.main.transform.position.x, Camera.main.transform.position.y - offset, Camera.main.transform.position.z); ;
        //gameObject.transform.rotation = new Quaternion(Camera.main.transform.rotation.x, Camera.main.transform.rotation.y, Camera.main.transform.rotation.z, Camera.main.transform.rotation.w); 
        //gameObject.transform.rotation = Quaternion.Euler(0, Camera.main.transform.rotation.eulerAngles.y, 0);

        //Transform avatar = gameObject.transform;
        //update++;
        //Debug.Log(Camera.main.velocity);

        //if ((Math.Abs(Camera.main.velocity.x) > 0) || (Math.Abs(Camera.main.velocity.y) > 0) || (Math.Abs(Camera.main.velocity.z) > 0))
        //{
        //    //angle = angle + 90;
        //    //gameObject.transform.rotation = Quaternion.Euler(0, angle, 0);
        //    animatorComponent.SetBool("walk", true);
        //    //update = 0;
        //}
        //if ((Camera.main.velocity.x == 0) && (Camera.main.velocity.y == 0) && (Camera.main.velocity.z == 0))
        //{
        //    animatorComponent.SetBool("walk", false);
        //    //update = 0;
        //}

       
        // updateready = false;

    }
    private void OnAnimatorIK()
    {
        //Debug.Log("right IKhint before" + rightIKhint.transform.position);
        //Debug.Log("right IKposition before" + rightIKposition.transform.position);
        //if (RightLowerArm == true)
        //{
        //    bones[12].rotation = Quaternion.Slerp(prevbones[12].rotation, new Quaternion(floatx[12], floaty[12], floatz[12], floatw[12]), smoothFactor * Time.deltaTime);
        //    prevbones[12].rotation = new Quaternion(floatx[12], floaty[12], floatz[12], floatw[12]);
        //    //bones[12].rotation = new Quaternion(floatx, floaty, floatz, floatw);
        //    //animatorComponent.SetBoneLocalRotation(HumanBodyBones.RightLowerArm, bonetransformtemp.rotation);
        //    RightLowerArm = false;
        //    //Debug.Log(bonetransformtemp.rotation);
        //}
        //if (RightHand == true)
        //{
        //    bones[13].rotation = Quaternion.Slerp(prevbones[13].rotation, new Quaternion(floatx[13], floaty[13], floatz[13], floatw[13]), smoothFactor * Time.deltaTime);
        //    prevbones[13].rotation = new Quaternion(floatx[13], floaty[13], floatz[13], floatw[13]);
        //    //bones[13].rotation = new Quaternion(floatx, floaty, floatz, floatw);
        //    //animatorComponent.SetBoneLocalRotation(HumanBodyBones.RightHand, bonetransformtemp.rotation);
        //    RightHand = false;
        //    //Debug.Log(bonetransformtemp.rotation);
        //}
        //rightIKhint.transform.position = rightelbow.transform.position;
        //rightIKposition.transform.position = righthand.transform.position;
        //Debug.Log("right IKhint after" + rightIKhint.transform.position);
        //Debug.Log("right IKposition after" + rightIKposition.transform.position);
        //if (GameObject.Find("SkinnyGuy") != null)
        //{
        //    animatorComponent.SetIKHintPosition(AvatarIKHint.RightElbow, rightIKhint.transform.position);
        //    animatorComponent.SetIKHintPositionWeight(AvatarIKHint.RightElbow, 1);
        //    animatorComponent.SetIKPosition(AvatarIKGoal.RightHand, rightIKposition.transform.position);
        //    animatorComponent.SetIKPositionWeight(AvatarIKGoal.RightHand, 1);

        //}
    }

    void LateUpdate()
    {

        timer += Time.deltaTime;

        if (curIndex >= playbackFrames.Count)
        {
            curIndex = 0;
            timer = 0.0f;
            humantestfinishstatus = true;
            Debug.Log("humantestfinishstatus :" + humantestfinishstatus);
        }

        //if (updateready && timer >= playbackFrames[curIndex].timestamp)
        if (timer >= playbackFrames[curIndex].timestamp)
        {
            //Debug.Log("current index: " + curIndex);
            UpdateAvatar(playbackFrames[curIndex].bodydata);

            curIndex++;
        }


        //GameObject eye_fixation = GameObject.Find("ML_eyefixation");
        //eye_fixation.transform.position = MLEyes.FixationPoint;
        //Camera.main.GetComponent<LineRenderer>().SetPosition(1, MLEyes.FixationPoint);
        RaycastHit hit;
        ////GameObject neckskinny = GameObject.Find("SkinnyGuy/Group/Main/DeformationSystem/Root_M/Spine1_M/Chest_M/Neck_M");
        ////Debug.Log("Neck bone transform:"+animatorComponent.GetBoneTransform(HumanBodyBones.Neck).position);
        //GameObject hit_avatar = GameObject.Find("hit_avatar");
        //GameObject hit_avatar1 = GameObject.Find("hit_avatar_1");
        //hit_avatar.transform.position = animatorComponent.GetBoneTransform(HumanBodyBones.RightUpperArm).position;
        //hit_avatar1.transform.position = animatorComponent.GetBoneTransform(HumanBodyBones.RightShoulder).position;
        ////Debug.Log("Neck transform:"+neckskinny.transform.position);
        ////var ray = Camera.main.ScreenPointToRay(MLEyes.FixationPoint);

        ////if (Physics.Raycast(ray, out hit))
        ////if(Physics.Raycast(Camera.main.transform.position,transform.forward,out hit))
        ////Debug.Log(Physics.Raycast(MLEyes.FixationPoint, transform.forward, out hit));
        if (GameObject.Find("SkinnyGuy") != null)
        {
            GameObject manavatar = GameObject.Find("SkinnyGuy");
            //manavatar.GetComponents<MonoBehaviour>;


            //allcomponents = gameObject.GetComponents(typeof(MonoBehaviour));
            //foreach (Component component in allcomponents)
            //{
            //    Debug.Log(component.name);
            //}

            //manavatar.transform.position = new Vector3(Camera.main.transform.position.x - 1, -1.1f, Camera.main.transform.position.z - 1);
            //manavatar.transform.rotation = Quaternion.Euler(0, Camera.main.transform.rotation.eulerAngles.y+180, 0);
            //GameObject hitavatar = GameObject.Find("hit_avatar_sphere");
            //if (temp)
            //{
            //    //manavatar.transform.position = Vector3.Slerp(manavatar.transform.position, new Vector3(0, 0, 0),   (smoothFactor * Time.deltaTime));
            //     manavatar.transform.position = new Vector3(0, 0, 0);
            //    temp = !temp;
            //    //num = 0;



            //}
            //else
            //{
            //    manavatar.transform.position = new Vector3(1, 0, 1);
            //    //manavatar.transform.position = Vector3.Slerp(manavatar.transform.position, new Vector3(1, 0, 1), (smoothFactor * Time.deltaTime));
            //    temp = !temp;
            //    //num++;
            //    //Debug.Log("num:" + num);
            //}
            ////manneck.transform.rotation = Camera.main.transform.rotation * Quaternion.Inverse(neckinitialman);
            ////_heading = MLEyes.FixationPoint - Camera.main.transform.position;
            //lefteyeman.transform.LookAt(Camera.main.transform);
            //righteyeman.transform.LookAt(Camera.main.transform);
            //lefteyemana.transform.rotation = lefteyeman.transform.rotation * Quaternion.Inverse(leftcorrectman);//*Quaternion.Euler(90,0,0);
            //righteyemana.transform.rotation = righteyeman.transform.rotation * Quaternion.Inverse(rightcorrectman);//* Quaternion.Euler(270, 0, 0);
            //Debug.Log("left correction update:" + leftcorrectman);
            //_heading = MLEyes.FixationPoint - Camera.main.transform.position;
            // Use the proper material
            //if (Physics.Raycast(Camera.main.transform.position, _heading, out hit, Mathf.Infinity))
            //{

            //    //Debug.Log("hit object: "+hit.collider.gameObject.name);
            //    if (hit.rigidbody != null)
            //    {
            //        if (hit.collider.gameObject.name == "Scapula_L")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftShoulder).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftShoulder).position, animatorComponent.GetBoneTransform(HumanBodyBones.LeftUpperArm).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            //Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Scapula_L" + ":" + percent.ToString());

            //        }
            //        if (hit.collider.gameObject.name == "Scapula_R")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightShoulder).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightShoulder).position, animatorComponent.GetBoneTransform(HumanBodyBones.RightUpperArm).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            //Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Scapula_R" + ":" + percent.ToString());
            //        }
            //        if (hit.collider.gameObject.name == "Hip_R")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightUpperLeg).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightUpperLeg).position, animatorComponent.GetBoneTransform(HumanBodyBones.RightLowerLeg).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            //Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Hip_R" + ":" + percent.ToString());
            //        }
            //        if (hit.collider.gameObject.name == "Hip_L")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftUpperLeg).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftUpperLeg).position, animatorComponent.GetBoneTransform(HumanBodyBones.LeftLowerLeg).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            //Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Hip_L" + ":" + percent.ToString());
            //        }
            //        if (hit.collider.gameObject.name == "Knee_L")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftLowerLeg).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftLowerLeg).position, animatorComponent.GetBoneTransform(HumanBodyBones.LeftFoot).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            //Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Knee_L" + ":" + percent.ToString());
            //        }
            //        if (hit.collider.gameObject.name == "Knee_R")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightLowerLeg).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightLowerLeg).position, animatorComponent.GetBoneTransform(HumanBodyBones.RightFoot).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            //Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Knee_R" + ":" + percent.ToString());
            //        }
            //        if (hit.collider.gameObject.name == "Ankle_R")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightFoot).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightFoot).position, animatorComponent.GetBoneTransform(HumanBodyBones.RightToes).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            // Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Ankle_R" + ":" + percent.ToString());
            //        }
            //        if (hit.collider.gameObject.name == "Ankle_L")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftFoot).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftFoot).position, animatorComponent.GetBoneTransform(HumanBodyBones.LeftToes).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            // Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Ankle_L" + ":" + percent.ToString());
            //        }
            //        if (hit.collider.gameObject.name == "Shoulder_L")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftUpperArm).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftUpperArm).position, animatorComponent.GetBoneTransform(HumanBodyBones.LeftLowerArm).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            // Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Shoulder_L" + ":" + percent.ToString());
            //        }
            //        if (hit.collider.gameObject.name == "Elbow_L")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftLowerArm).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftLowerArm).position, animatorComponent.GetBoneTransform(HumanBodyBones.LeftHand).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            //Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Elbow_L" + ":" + percent.ToString());
            //        }
            //        if (hit.collider.gameObject.name == "Wrist_L")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftHand).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.LeftHand).position, animatorComponent.GetBoneTransform(HumanBodyBones.LeftMiddleDistal).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            //Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Wrist_L" + ":" + percent.ToString());
            //        }
            //        if (hit.collider.gameObject.name == "Shoulder_R")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightUpperArm).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightUpperArm).position, animatorComponent.GetBoneTransform(HumanBodyBones.RightLowerArm).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            // Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Shoulder_R" + ":" + percent.ToString());

            //        }
            //        if (hit.collider.gameObject.name == "Elbow_R")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightLowerArm).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightLowerArm).position, animatorComponent.GetBoneTransform(HumanBodyBones.RightHand).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            //Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Elbow_R" + ":" + percent.ToString());
            //        }
            //        if (hit.collider.gameObject.name == "Wrist_R")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightHand).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.RightHand).position, animatorComponent.GetBoneTransform(HumanBodyBones.RightMiddleDistal).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            //Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Wrist_R" + ":" + percent.ToString());
            //        }
            //        if (hit.collider.gameObject.name == "Chest_M")
            //        {
            //            GameObject subject_fix = GameObject.Find("eye_hitpoint");
            //            subject_fix.transform.position = hit.point;
            //            float dist1 = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.Chest).position, hit.point);
            //            float totaldist = Vector3.Distance(animatorComponent.GetBoneTransform(HumanBodyBones.Chest).position, animatorComponent.GetBoneTransform(HumanBodyBones.Hips).position);
            //            GameObject collide = hit.collider.gameObject;
            //            float dist2 = collide.GetComponent<CapsuleCollider>().radius;
            //            Debug.Log("object name:" + hit.collider.gameObject.name);
            //            Debug.Log("distance:" + dist2);
            //            float actualdist = Mathf.Sqrt(dist1 * dist1 - dist2 * dist2);
            //            float percent = actualdist / totaldist;
            //            //Debug.Log("percent dist:" + percent);
            //            if (!ws.IsAlive)
            //            {
            //                Debug.Log("Attempting to Connect");
            //                ws.Connect();
            //            }
            //            ws.Send("#" + "Chest_M" + ":" + percent.ToString());
            //        }





            //    }
            //}
        }

        //GameObject originalGameObject = GameObject.Find("U_CharacterFront/U/joint_Char/joint_Pelvis/joint_TorsoA/joint_TorsoB/joint_TorsoC/joint_Neck");
        //        if (GameObject.Find("BusinessWomanPFB") != null)
        //{
        //    GameObject womanavatar = GameObject.Find("BusinessWomanPFB");
        //    womanavatar.transform.position = new Vector3(Camera.main.transform.position.x+1, 0, Camera.main.transform.position.z);
        //    womanavatar.transform.rotation = Quaternion.Euler(0, Camera.main.transform.rotation.eulerAngles.y+180, 0);
        //   // womanneck.transform.rotation = Camera.main.transform.rotation * Quaternion.Inverse(neckinitialwoman);
        //    //_heading = MLEyes.FixationPoint - Camera.main.transform.position;
        //    //lefteyewoman.transform.LookAt(MLEyes.FixationPoint);
        //    //righteyewoman.transform.LookAt(MLEyes.FixationPoint);
        //    //lefteyewomana.transform.rotation = lefteyewoman.transform.rotation * Quaternion.Inverse(leftcorrectwoman);//*Quaternion.Euler(90,0,0);
        //    //righteyewomana.transform.rotation = righteyewoman.transform.rotation * Quaternion.Inverse(rightcorrectwoman);//* Quaternion.Euler(270, 0, 0);

        //                                                                                         //womanlineleft.GetComponent<LineRenderer>().SetVertexCount(2);
        //                                                                                                                 //womanlineright.GetComponent<LineRenderer>().SetVertexCount(2);
        //                                                                                                                 //womanlineleft.GetComponent<LineRenderer>().SetPosition(1, MLEyes.FixationPoint);
        //                                                                                                                 //womanlineright.GetComponent<LineRenderer>().SetPosition(1, MLEyes.FixationPoint);
        //}

        //if (MLHands.Left.IsVisible)
        //{
        //wristtest.transform.position = MLHands.Left.Wrist.KeyPoints[0].Position;
        //Debug.Log("indexbottom:" + MLHands.Left.Index.KeyPoints[2].Position);
        //indexlefttip.transform.position = MLHands.Left.Thumb.KeyPoints[0].Position;
        //indexleftmiddle.transform.position = MLHands.Left.Thumb.KeyPoints[1].Position;
        //indexlefttop.transform.position = MLHands.Left.Thumb.KeyPoints[2].Position;

        //GameObject indexlefttop = GameObject.Find("U_CharacterFront/U/joint_Char/joint_Pelvis/joint_TorsoA/joint_TorsoB/joint_TorsoC/joint_ClavicleLT/joint_ShoulderLT/joint_ElbowLT/joint_HandLT/joint_CapitateLT/joint_IndexALT");
        //GameObject indexleftmiddle = GameObject.Find("U_CharacterFront/U/joint_Char/joint_Pelvis/joint_TorsoA/joint_TorsoB/joint_TorsoC/joint_ClavicleLT/joint_ShoulderLT/joint_ElbowLT/joint_HandLT/joint_CapitateLT/joint_IndexALT/joint_IndexBLT");
        //GameObject indexlefttip = GameObject.Find("U_CharacterFront/U/joint_Char/joint_Pelvis/joint_TorsoA/joint_TorsoB/joint_TorsoC/joint_ClavicleLT/joint_ShoulderLT/joint_ElbowLT/joint_HandLT/joint_CapitateLT/joint_IndexALT/joint_IndexBLT/joint_IndexCLT");
        //Debug.Log("initialset:"+initialset);
        //if (!initialset)
        //{
        //    previndextip = MLHands.Left.Index.KeyPoints[0].Position;
        //    //previndextip.position = new Vector3(0,0,0);
        //    previndexmiddle = MLHands.Left.Index.KeyPoints[1].Position;
        //    //previndexmiddle.position = new Vector3(0, 0, 0);
        //    previndexbottom = MLHands.Left.Index.KeyPoints[2].Position;
        //    //previndexbottom.position = new Vector3(0, 0, 0);
        //    initialset = true;

        //}
        //float step = speed * Time.deltaTime; // calculate distance to move
        //if (wristtest.transform.position != transform.TransformPoint(leftwristpoint.transform.position))
        //{
        //    //Vector3 tempposition = transform.TransformPoint(leftwristpoint.transform.position);
        //    Vector3 addposition = leftwristpoint.transform.position - MLHands.Left.Wrist.KeyPoints[0].Position;
        //    wristtest.transform.position = wristtest.transform.position + addposition;
        //}
        //Debug.Log("position" + transform.TransformPoint(leftwristpoint.transform.position));
        //if (initialset)
        //{
        //var rotationt0 = Quaternion.LookRotation(MLHands.Left.Index.KeyPoints[0].Position - indexlefttip.transform.position);
        //Debug.Log("indextip:" + previndextip);
        //Debug.Log("wristtip:" + MLHands.Left.Wrist.KeyPoints[0].Position);
        //indextiptest.transform.position = MLHands.Left.Index.KeyPoints[0].Position;
        //wristtest.transform.position = MLHands.Left.Wrist.KeyPoints[0].Position;
        //indextipprevtest.transform.position = previndextip;
        //previndextip= MLHands.Left.Index.KeyPoints[0].Position;
        //Vector3 indextiptestc = new Vector3(0.01090663f, -0.3688616f, 0.2063677f);
        //Vector3 wristtestc = new Vector3(-0.01152904f, -0.4040097f, 0.1320198f);
        //Vector3 previndextipc = new Vector3(0.006946956f, -0.377946f, 0.2074568f);
        //var rotationt0 = Quaternion.FromToRotation(previndextipc - wristtestc, indextiptestc - wristtestc);
        //Debug.Log("rotation"+rotationt0.eulerAngles);



        //indexlefttip.transform.rotation = indexlefttip.transform.rotation*Quaternion.FromToRotation(previndextip - MLHands.Left.Wrist.KeyPoints[0].Position, MLHands.Left.Index.KeyPoints[0].Position - MLHands.Left.Wrist.KeyPoints[0].Position);
        //Debug.Log("indextip:" + indexlefttip.transform.rotation.eulerAngles);
        ////indexlefttip.transform.rotation= Quaternion.FromToRotation(previndextip, MLHands.Left.Index.KeyPoints[0].Position);
        ////indexlefttip.transform.rotation = Quaternion.Slerp(indexlefttip.transform.rotation, rotationt0, smoothFactor * Time.deltaTime);
        //previndextip = MLHands.Left.Index.KeyPoints[0].Position;
        //indexleftmiddle.transform.rotation = Quaternion.FromToRotation(previndexmiddle - MLHands.Left.Wrist.KeyPoints[0].Position, MLHands.Left.Index.KeyPoints[1].Position - MLHands.Left.Wrist.KeyPoints[0].Position);
        //Debug.Log("indexniddle:" + indexlefttip.transform.rotation.eulerAngles);
        //previndexmiddle = MLHands.Left.Index.KeyPoints[1].Position;
        //indexlefttop.transform.rotation = indexlefttop.transform.rotation*Quaternion.FromToRotation(previndexbottom - MLHands.Left.Wrist.KeyPoints[0].Position, MLHands.Left.Index.KeyPoints[2].Position - MLHands.Left.Wrist.KeyPoints[0].Position);
        //Debug.Log("indexbottom:" + indexlefttip.transform.rotation.eulerAngles);
        //previndexbottom = MLHands.Left.Index.KeyPoints[2].Position;


        //}


        //indexlefttip.transform.position = Vector3.Slerp(indexlefttip.transform.position, MLHands.Left.Index.KeyPoints[0].Position, smoothFactor * Time.deltaTime);
        //previndextip = MLHands.Left.Index.KeyPoints[0].Position;

        //indexlefttip.transform.position = Vector3.Slerp(indexleftmiddle.transform.position, MLHands.Left.Index.KeyPoints[1].Position, smoothFactor * Time.deltaTime);
        //previndexmiddle = MLHands.Left.Index.KeyPoints[1].Position;

        //indexlefttip.transform.position = Vector3.Slerp(indexlefttop.transform.position, MLHands.Left.Index.KeyPoints[2].Position, smoothFactor * Time.deltaTime);
        //previndexbottom = MLHands.Left.Index.KeyPoints[2].Position;

        //}
        //if (MLHands.Right.IsVisible)
        //{

        //}
        ////originalGameObject.transform.rotation = Camera.main.transform.rotation;
        //if (Hips == true)
        //{
        //    bones[0].rotation = Quaternion.Slerp(prevbones[0].rotation, new Quaternion(floatx[0], floaty[0], floatz[0], floatw[0]), smoothFactor * Time.deltaTime);
        //    prevbones[0].rotation = new Quaternion(floatx[0], floaty[0], floatz[0], floatw[0]);

        //    //animatorComponent.SetBoneLocalRotation(HumanBodyBones.Hips, bonetransformtemp.rotation);
        //    Hips = false;
        //    //Debug.Log(bonetransformtemp.rotation);
        //}
        if (Spine == true)
        {
            bones[1].rotation = Quaternion.Slerp(prevbones[1].rotation, new Quaternion(floatx[1], floaty[1], floatz[1], floatw[1]), smoothFactor * Time.deltaTime);
            prevbones[1].rotation = new Quaternion(floatx[1], floaty[1], floatz[1], floatw[1]);
            //bones[1].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.Spine, bonetransformtemp.rotation);
            Spine = false;
            //Debug.Log(bonetransformtemp.rotation);
        }
        //if (Neck == true)

        //{
        //    bones[3].rotation = Quaternion.Slerp(prevbones[3].rotation, new Quaternion(floatx[3], floaty[3], floatz[3], floatw[3]), smoothFactor * Time.deltaTime);
        //    prevbones[3].rotation = new Quaternion(floatx[3], floaty[3], floatz[3], floatw[3]);
        //    // bones[3].rotation = new Quaternion(floatx, floaty, floatz, floatw);
        //    //bonetransformtemp.rotation = new Quaternion(floatx, floaty, floatz, floatw);
        //    //animatorComponent.SetBoneLocalRotation(HumanBodyBones.Neck, new Quaternion(floatx, floaty, floatz, floatw));
        //    Neck = false;
        //    //Debug.Log(new Quaternion(floatx, floaty, floatz, floatw));
        //    //Debug.Log("neck IK" + new Quaternion(floatx[3], floaty[3], floatz[3], floatw[3]));
        //}
        if (LeftUpperArm == true)
        {
            bones[5].rotation = Quaternion.Slerp(prevbones[5].rotation, new Quaternion(floatx[5], floaty[5], floatz[5], floatw[5]), smoothFactor * Time.deltaTime);
            prevbones[5].rotation = new Quaternion(floatx[5], floaty[5], floatz[5], floatw[5]);
            //bones[5].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.LeftUpperArm, bonetransformtemp.rotation);
            LeftUpperArm = false;
            //Debug.Log(bonetransformtemp.rotation);
        }
        if (LeftLowerArm == true)
        {
            bones[6].rotation = Quaternion.Slerp(prevbones[6].rotation, new Quaternion(floatx[6], floaty[6], floatz[6], floatw[6]), smoothFactor * Time.deltaTime);
            prevbones[6].rotation = new Quaternion(floatx[6], floaty[6], floatz[6], floatw[6]);
            //bones[6].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.LeftLowerArm, bonetransformtemp.rotation);
            LeftLowerArm = false;
            //Debug.Log("leftlowerarm IK"+new Quaternion(floatx[6], floaty[6], floatz[6], floatw[6]));
        }
        if (LeftHand == true)
        {
            bones[7].rotation = Quaternion.Slerp(prevbones[7].rotation, new Quaternion(floatx[7], floaty[7], floatz[7], floatw[7]), smoothFactor * Time.deltaTime);
            prevbones[7].rotation = new Quaternion(floatx[7], floaty[7], floatz[7], floatw[7]);
            //bones[7].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.LeftHand, bonetransformtemp.rotation);
            LeftHand = false;
            //Debug.Log(bonetransformtemp.rotation);
        }
        if (RightUpperArm == true)
        {
            bones[11].rotation = Quaternion.Slerp(prevbones[11].rotation, new Quaternion(floatx[11], floaty[11], floatz[11], floatw[11]), smoothFactor * Time.deltaTime);
            prevbones[11].rotation = new Quaternion(floatx[11], floaty[11], floatz[11], floatw[11]);
            //bones[11].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.RightUpperArm, bonetransformtemp.rotation);
            RightUpperArm = false;
            //Debug.Log(bonetransformtemp.rotation);
        }
        if (RightLowerArm == true)
        {
            bones[12].rotation = Quaternion.Slerp(prevbones[12].rotation, new Quaternion(floatx[12], floaty[12], floatz[12], floatw[12]), smoothFactor * Time.deltaTime);
            prevbones[12].rotation = new Quaternion(floatx[12], floaty[12], floatz[12], floatw[12]);
            //bones[12].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.RightLowerArm, bonetransformtemp.rotation);
            RightLowerArm = false;
            //Debug.Log(bonetransformtemp.rotation);
        }
        if (RightHand == true)
        {
            bones[13].rotation = Quaternion.Slerp(prevbones[13].rotation, new Quaternion(floatx[13], floaty[13], floatz[13], floatw[13]), smoothFactor * Time.deltaTime);
            prevbones[13].rotation = new Quaternion(floatx[13], floaty[13], floatz[13], floatw[13]);
            //bones[13].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.RightHand, bonetransformtemp.rotation);
            RightHand = false;
            //Debug.Log(bonetransformtemp.rotation);
        }
        if (LeftShoulder == true)
        {
            bones[25].rotation = Quaternion.Slerp(prevbones[25].rotation, new Quaternion(floatx[25], floaty[25], floatz[25], floatw[25]), smoothFactor * Time.deltaTime);
            prevbones[25].rotation = new Quaternion(floatx[25], floaty[25], floatz[25], floatw[25]);
            //bones[25].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.LeftShoulder, bonetransformtemp.rotation);
            LeftShoulder = false;
            //Debug.Log(bonetransformtemp.rotation);
        }
        if (RightShoulder == true)
        {
            bones[26].rotation = Quaternion.Slerp(prevbones[26].rotation, new Quaternion(floatx[26], floaty[26], floatz[26], floatw[26]), smoothFactor * Time.deltaTime);
            prevbones[26].rotation = new Quaternion(floatx[26], floaty[26], floatz[26], floatw[26]);
            //bones[26].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.RightShoulder, bonetransformtemp.rotation);
            RightShoulder = false;
            //Debug.Log(bonetransformtemp.rotation);
        }
        if (LeftIndexProximal == true)
        {
            bones[27].rotation = Quaternion.Slerp(prevbones[27].rotation, new Quaternion(floatx[27], floaty[27], floatz[27], floatw[27]), smoothFactor * Time.deltaTime);
            prevbones[27].rotation = new Quaternion(floatx[27], floaty[27], floatz[27], floatw[27]);
            //bones[27].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.LeftIndexProximal, bonetransformtemp.rotation);
            LeftIndexProximal = false;
            //Debug.Log(bonetransformtemp.rotation);
        }
        if (RightIndexProximal == true)
        {
            bones[28].rotation = Quaternion.Slerp(prevbones[28].rotation, new Quaternion(floatx[28], floaty[28], floatz[28], floatw[28]), smoothFactor * Time.deltaTime);
            prevbones[28].rotation = new Quaternion(floatx[28], floaty[28], floatz[28], floatw[28]);
            //bones[28].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.RightIndexProximal, bonetransformtemp.rotation);
            RightIndexProximal = false;
            //Debug.Log(bonetransformtemp.rotation);
        }
        if (LeftThumbProximal == true)
        {
            bones[29].rotation = Quaternion.Slerp(prevbones[29].rotation, new Quaternion(floatx[29], floaty[29], floatz[29], floatw[29]), smoothFactor * Time.deltaTime);
            prevbones[29].rotation = new Quaternion(floatx[29], floaty[29], floatz[29], floatw[29]);
            //bones[29].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.LeftThumbProximal, bonetransformtemp.rotation);
            LeftThumbProximal = false;
            //Debug.Log(bonetransformtemp.rotation);
        }
        if (RightThumbProximal == true)
        {
            bones[30].rotation = Quaternion.Slerp(prevbones[30].rotation, new Quaternion(floatx[30], floaty[30], floatz[30], floatw[30]), smoothFactor * Time.deltaTime);
            prevbones[30].rotation = new Quaternion(floatx[30], floaty[30], floatz[30], floatw[30]);
            //bones[30].rotation = new Quaternion(floatx, floaty, floatz, floatw);
            //animatorComponent.SetBoneLocalRotation(HumanBodyBones.RightThumbProximal, bonetransformtemp.rotation);
            RightThumbProximal = false;
            //Debug.Log(bonetransformtemp.rotation);
        }

        //animatorComponent.SetIKPosition(AvatarIKGoal.RightHand, Vector3.zero);
        //animatorComponent.SetIKPositionWeight(AvatarIKGoal.RightHand, 1);
        GameObject manavatarr = GameObject.Find("SkinnyGuy");
        //if (!temp)
        //{
        //dummytarget.transform.position = new Vector3(1, 2, 1);
        //correctionvec = manavatarr.transform.position - indextiptest.transform.position;
        //correctposvec = dummytarget.transform.position + correctionvec;
        //manavatarr.transform.position = new Vector3(correctposvec.x, 0, correctposvec.z);
        //temp = !temp;
        //}
        //else
        //{
        //dummytarget.transform.position = new Vector3(1.1f, 2, 1);
        //correctionvec = manavatarr.transform.position - indextiptest.transform.position;
        //correctposvec = dummytarget.transform.position + correctionvec;
        //manavatarr.transform.position = new Vector3(correctposvec.x, 0, correctposvec.z);
        ////temp = !temp;
        //}
        //dummytarget.transform.position = indextiptest.transform.position;
        //RaycastHit hitt;
        //if (Physics.Raycast(dummytarget.transform.position, (rightelbow.transform.position- dummytarget.transform.position), out hitt, Mathf.Infinity))
        //{
        //    Debug.Log(hitt.collider.gameObject.name);
        //    GameObject objecttoplace = GameObject.Find("eye_hitpoint");
        //    objecttoplace.transform.position = hitt.point;


        //}
        //RaycastHit[] hits;
        //hits = Physics.RaycastAll(dummytarget.transform.position, (rightelbow.transform.position - dummytarget.transform.position), Mathf.Infinity);

        //for (int i = 0; i < hits.Length; i++)
        //{
        //    RaycastHit hitt = hits[i];


        //    if (hitt.collider.gameObject.name == "Elbow_R")
        //    {
        //        //Debug.Log(hitt.collider.GetType().ToString());
        //        if (hitt.collider.GetType().ToString() == "UnityEngine.SphereCollider")
        //        {
        //            // Change the material of all hit colliders
        //            // to use a transparent shader.
        //            GameObject objecttoplace = GameObject.Find("eye_hitpoint");
        //            objecttoplace.transform.position = hitt.point;

        //        }
        //    }
        //}
    }

    public void UpdateAvatar(BodyData bodydatapoint)
    {


        bodyData = bodydatapoint;
        Int64 UserID = bodydatapoint.liTrackingID;


        //if (!gameObject.activeInHierarchy)
        //    return;

        //// Get the KinectManager instance
        ////if (kinectManager == null)
        ////{
        ////    kinectManager = KinectManager.Instance;
        ////}

        //// get the background plane rectangle if needed 
        if (backgroundPlane && !planeRectSet)
        {
            planeRectSet = true;

            planeRect.width = 10f * Mathf.Abs(backgroundPlane.localScale.x);
            planeRect.height = 10f * Mathf.Abs(backgroundPlane.localScale.z);
            planeRect.x = backgroundPlane.position.x - planeRect.width / 2f;
            planeRect.y = backgroundPlane.position.y - planeRect.height / 2f;
        }


        //// move the avatar to its Kinect position
        if (!externalRootMotion)
        {
            MoveAvatar(UserID, bodydatapoint);

        }

        // get the left hand state and event
        //if (kinectManager && kinectManager.GetJointTrackingState(UserID, (int)KinectInterop.JointType.HandLeft) != KinectInterop.TrackingState.NotTracked)
        if (GetJointTrackingState(UserID, (int)JointType.HandLeft, bodydatapoint) != TrackingState.NotTracked)
        {

            HandState leftHandState = GetLeftHandState(UserID, bodydatapoint);
            //Debug.Log(leftHandState.ToString());
            HandEventType leftHandEvent = HandStateToEvent(leftHandState, lastLeftHandEvent);

            if (leftHandEvent != HandEventType.None)
            {
                lastLeftHandEvent = leftHandEvent;
            }
        }

        // get the right hand state and event
        //if (kinectManager && kinectManager.GetJointTrackingState(UserID, (int)KinectInterop.JointType.HandRight) != KinectInterop.TrackingState.NotTracked)
        if (GetJointTrackingState(UserID, (int)JointType.HandRight, bodydatapoint) != TrackingState.NotTracked)
        {
            HandState rightHandState = GetRightHandState(UserID, bodydatapoint);
            HandEventType rightHandEvent = HandStateToEvent(rightHandState, lastRightHandEvent);

            if (rightHandEvent != HandEventType.None)
            {
                lastRightHandEvent = rightHandEvent;
            }
        }

        //Debug.Log("bones.Length"+bones.Length);
        //rotate the avatar bones
        for (var boneIndex = 0; boneIndex < bones.Length; boneIndex++)
        {
            if (!bones[boneIndex] || isBoneDisabled[boneIndex])
                continue;

            if (boneIndex2JointMap.ContainsKey(boneIndex))
            {

                JointType joint = !(mirroredMovement ^ flipLeftRight) ?
                    boneIndex2JointMap[boneIndex] : boneIndex2MirrorJointMap[boneIndex];

                if (externalHeadRotation && joint == JointType.Head)   // skip head if moved externally
                {
                    continue;
                }

                if (externalHandRotations &&    // skip hands if moved externally
                    (joint == JointType.WristLeft || joint == JointType.WristRight ||
                        joint == JointType.HandLeft || joint == JointType.HandRight))
                {
                    continue;
                }
                //if (boneIndex != 0 && boneIndex < 17 && boneIndex !=3)
                {

                    TransformBone(UserID, joint, boneIndex, !(mirroredMovement ^ flipLeftRight), bodydatapoint);

                }


            }
            else if (specIndex2JointMap.ContainsKey(boneIndex))
            {
                // special bones (clavicles)
                List<JointType> alJoints = !(mirroredMovement ^ flipLeftRight) ?
                    specIndex2JointMap[boneIndex] : specIndex2MirrorMap[boneIndex];

                if (alJoints.Count >= 2)
                {
                    //Debug.Log(alJoints[0].ToString());
                    Vector3 baseDir = alJoints[0].ToString().EndsWith("Left") ? Vector3.left : Vector3.right;
                    TransformSpecialBone(UserID, alJoints[0], alJoints[1], boneIndex, baseDir, !(mirroredMovement ^ flipLeftRight));
                }
            }


        }

        //if (applyMuscleLimits && kinectManager && kinectManager.IsUserTracked(UserID))
        if (applyMuscleLimits)
        {
            // check for limits
            CheckMuscleLimits();
        }



    }

    /// <summary>
	/// Gets the right hand state for the specified user.
	/// </summary>
	/// <returns>The right hand state.</returns>
	/// <param name="userId">User ID</param>
	public HandState GetRightHandState(Int64 userId, BodyData bodydata)
    {
        //if (dictUserIdToIndex.ContainsKey(userId))
        //{
        //int index = dictUserIdToIndex[userId];

        //if (index >= 0 && index < sensorData.bodyCount &&
        //    bodyFrame.bodyData[index].bIsTracked != 0)
        if (bodydata.bIsTracked != 0)
        {
            return bodydata.rightHandState;
        }
        //}

        return HandState.NotTracked;
    }

    // converts hand state to hand event type
    public static HandEventType HandStateToEvent(HandState handState, HandEventType lastEventType)
    {
        switch (handState)
        {
            case HandState.Open:
                return HandEventType.Release;

            case HandState.Closed:
            case HandState.Lasso:
                return HandEventType.Grip;

            case HandState.Unknown:
                return lastEventType;
        }

        return HandEventType.None;
    }

    /// <summary>
	/// Gets the tracking state of the joint.
	/// </summary>
	/// <returns>The joint tracking state.</returns>
	/// <param name="userId">User ID</param>
	/// <param name="joint">Joint index</param>
	public TrackingState GetJointTrackingState(Int64 userId, int joint, BodyData bodydata)
    {
        // if (dictUserIdToIndex.ContainsKey(userId))
        //{
        //int index = dictUserIdToIndex[userId];

        //if (index >= 0 && index < sensorData.bodyCount &&
        //bodyFrame.bodyData[index].bIsTracked != 0)
        //{
        if (joint >= 0 && joint < 25)
        {
            return bodydata.joint[joint].trackingState;
        }
        //}
        //}

        return TrackingState.NotTracked;
    }

    /// <summary>
	/// Gets the left hand state for the specified user.
	/// </summary>
	/// <returns>The left hand state.</returns>
	/// <param name="userId">User ID</param>
	public HandState GetLeftHandState(Int64 userId, BodyData bodydata)
    {
        //if (dictUserIdToIndex.ContainsKey(userId))
        //{
        //    int index = dictUserIdToIndex[userId];

        //if (index >= 0 && index < sensorData.bodyCount &&
        //    bodyFrame.bodyData[index].bIsTracked != 0)
        if (bodydata.bIsTracked != 0)
        {
            return bodydata.leftHandState;
        }
        //}

        return HandState.NotTracked;
    }

    // Apply the rotations tracked by kinect to the joints.
    protected void TransformBone(Int64 userId, JointType joint, int boneIndex, bool flip, BodyData bodydata)
    {

        Transform boneTransform = bones[boneIndex];
        //if (boneTransform == null || kinectManager == null)
        if (boneTransform == null)
            return;

        int iJoint = (int)joint;
        //if (iJoint < 0 || !kinectManager.IsJointTracked(userId, iJoint))
        if (iJoint < 0 || !IsJointTracked(userId, iJoint, bodydata))
            return;

        // Get Kinect joint orientation
        //Quaternion jointRotation = kinectManager.GetJointOrientation(userId, iJoint, flip);
        Quaternion jointRotation = GetJointOrientation(userId, iJoint, flip, bodydata);
        if (jointRotation == Quaternion.identity && !IsLegJoint(joint))
            return;

        // calculate the new orientation
        Quaternion newRotation = Kinect2AvatarRot(jointRotation, boneIndex);
        //if (boneIndex == 3)
        //{
        //    newRotation = Camera.main.transform.rotation;
        //    Debug.Log("rotation: "+newRotation);
        //}

        if (externalRootMotion)
        {
            newRotation = transform.rotation * newRotation;
        }

        // Smoothly transition to the new rotation
        if (smoothFactor != 0f)
        {

            boneTransform.rotation = Quaternion.Slerp(boneTransform.rotation, newRotation, smoothFactor * Time.deltaTime);
        }
        else
        {

            boneTransform.rotation = newRotation;
        }

        /*  if (boneIndex == 0)
          {
              Hips = true;
              floatx[0] = boneTransform.rotation.x;
              floaty[0] = boneTransform.rotation.y;
              floatz[0] = boneTransform.rotation.z;
              floatw[0] = boneTransform.rotation.w;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }
          //else if (boneIndex == 1)
          //{
          //    Spine = true;
          //    floatx[1] = boneTransform.rotation.x;
          //    floaty[1] = boneTransform.rotation.y;
          //    floatz[1] = boneTransform.rotation.z;
          //    floatw[1] = boneTransform.rotation.w;
          //    //bonetransformtemp = boneTransform;
          //    //bonetransformtemp.rotation = boneTransform.rotation;
          //}
          else if (boneIndex == 3)
          {
              Neck = true;
              floatx[3] = boneTransform.rotation.x;
              floaty[3] = boneTransform.rotation.y;
              floatz[3] = boneTransform.rotation.z;
              floatw[3] = boneTransform.rotation.w;
              //bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }
          else if (boneIndex == 5)
          {
              LeftUpperArm = true;
              floatx[5] = boneTransform.rotation.x;
              floaty[5] = boneTransform.rotation.y;
              floatz[5] = boneTransform.rotation.z;
              floatw[5] = boneTransform.rotation.w;
              //bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }
          else if (boneIndex == 6)
          {
              LeftLowerArm = true;
              floatx[6] = boneTransform.rotation.x;
              floaty[6] = boneTransform.rotation.y;
              floatz[6] = boneTransform.rotation.z;
              floatw[6] = boneTransform.rotation.w;
              // bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
              //Debug.Log("original lowerarm" + boneTransform.rotation);
          }
          else if (boneIndex == 7)
          {
              LeftHand = true;
              floatx[7] = boneTransform.rotation.x;
              floaty[7] = boneTransform.rotation.y;
              floatz[7] = boneTransform.rotation.z;
              floatw[7] = boneTransform.rotation.w;
              //bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }
          else if (boneIndex == 11)
          {
              RightUpperArm = true;
              floatx[11] = boneTransform.rotation.x;
              floaty[11] = boneTransform.rotation.y;
              floatz[11] = boneTransform.rotation.z;
              floatw[11] = boneTransform.rotation.w;




              //bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }
          else if (boneIndex == 12)
          {

              //Debug.Log("right IKhint before" + rightIKhint.transform.position);
              RightLowerArm = true;
              floatx[12] = boneTransform.rotation.x;
              floaty[12] = boneTransform.rotation.y;
              floatz[12] = boneTransform.rotation.z;
              floatw[12] = boneTransform.rotation.w;

              //rightIKhint.transform.position = boneTransform.position;

              //Debug.Log("right IKhint after" + rightIKhint.transform.position);

              //bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }
          else if (boneIndex == 13)
          {
              //Debug.Log("right IKposition before" + rightIKposition.transform.position);
              RightHand = true;
              floatx[13] = boneTransform.rotation.x;
              floaty[13] = boneTransform.rotation.y;
              floatz[13] = boneTransform.rotation.z;
              floatw[13] = boneTransform.rotation.w;
             // rightIKposition.transform.position = boneTransform.position;
              //Debug.Log("right IKposition after" + rightIKposition.transform.position);
              //bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }
          else if (boneIndex == 25)
          {
              LeftShoulder = true;
              floatx[25] = boneTransform.rotation.x;
              floaty[25] = boneTransform.rotation.y;
              floatz[25] = boneTransform.rotation.z;
              floatw[25] = boneTransform.rotation.w;
              //bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }
          else if (boneIndex == 26)
          {
              RightShoulder = true;
              floatx[26] = boneTransform.rotation.x;
              floaty[26] = boneTransform.rotation.y;
              floatz[26] = boneTransform.rotation.z;
              floatw[26] = boneTransform.rotation.w;
              //bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }
          else if (boneIndex == 27)
          {
              LeftIndexProximal = true;
              floatx[27] = boneTransform.rotation.x;
              floaty[27] = boneTransform.rotation.y;
              floatz[27] = boneTransform.rotation.z;
              floatw[27] = boneTransform.rotation.w;
              //bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }
          else if (boneIndex == 28)
          {
              RightIndexProximal = true;
              floatx[28] = boneTransform.rotation.x;
              floaty[28] = boneTransform.rotation.y;
              floatz[28] = boneTransform.rotation.z;
              floatw[28] = boneTransform.rotation.w;
              //bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }
          else if (boneIndex == 29)
          {
              LeftThumbProximal = true;
              floatx[29] = boneTransform.rotation.x;
              floaty[29] = boneTransform.rotation.y;
              floatz[29] = boneTransform.rotation.z;
              floatw[29] = boneTransform.rotation.w;
              //bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }
          else if (boneIndex == 30)
          {
              RightThumbProximal = true;
              floatx[30] = boneTransform.rotation.x;
              floaty[30] = boneTransform.rotation.y;
              floatz[30] = boneTransform.rotation.z;
              floatw[30] = boneTransform.rotation.w;
              //bonetransformtemp = boneTransform;
              //bonetransformtemp.rotation = boneTransform.rotation;
          }*/
    }

    /// <summary>
    /// Gets the joint orientation of the specified user.
    /// </summary>
    /// <returns>The joint rotation.</returns>
    /// <param name="userId">User ID</param>
    /// <param name="joint">Joint index</param>
    /// <param name="flip">If set to <c>true</c>, this means non-mirrored rotation</param>
    public Quaternion GetJointOrientation(Int64 userId, int joint, bool flip, BodyData bodydata)
    {
        //if (dictUserIdToIndex.ContainsKey(userId))
        //{
        //    int index = dictUserIdToIndex[userId];

        if (bodydata.bIsTracked != 0)
        {
            if (flip)
                return bodydata.joint[joint].normalRotation;
            else
                return bodydata.joint[joint].mirroredRotation;
        }
        //}

        return Quaternion.identity;
    }


    // Checks if the given joint is part of the legs
    protected bool IsLegJoint(JointType joint)
    {
        return ((joint == JointType.HipLeft) || (joint == JointType.HipRight) ||
                (joint == JointType.KneeLeft) || (joint == JointType.KneeRight) ||
                (joint == JointType.AnkleLeft) || (joint == JointType.AnkleRight));
    }


    // Converts kinect joint rotation to avatar joint rotation, depending on joint initial rotation and offset rotation
    protected Quaternion Kinect2AvatarRot(Quaternion jointRotation, int boneIndex)
    {
        Quaternion newRotation = jointRotation * initialRotations[boneIndex];
        //newRotation = initialRotation * newRotation;

        //		if(offsetNode != null)
        //		{
        //			newRotation = offsetNode.transform.rotation * newRotation;
        //		}
        //		else
        if (!externalRootMotion)  // fix by Mathias Parger
        {
            newRotation = initialRotation * newRotation;
        }

        return newRotation;
    }

    // Apply the rotations tracked by kinect to a special joint
    protected void TransformSpecialBone(Int64 userId, JointType joint, JointType jointParent, int boneIndex, Vector3 baseDir, bool flip)
    {
        Transform boneTransform = bones[boneIndex];
        //if (boneTransform == null || kinectManager == null)
        if (boneTransform == null)
            return;

        // get joint tracking state
        //bool isJointTracked = kinectManager.IsJointTracked(userId, (int)joint);
        bool isJointTracked = IsJointTracked(userId, (int)joint, bodyData);

        if (boneIndex >= 27 && boneIndex <= 30)
        {
            // fingers or thumbs
            if (fingerOrientations && !externalHandRotations)
            {
                TransformSpecialBoneFingers(userId, (int)joint, boneIndex, flip, isJointTracked);
            }

            return;
        }

        if (!isJointTracked || !IsJointTracked(userId, (int)jointParent, bodyData))
        {
            return;
        }

        // if the user is turned, tracking of special bones may be incorrect 
        bool userTurned = IsUserTurnedAround(userId);
        if (userTurned)
        {
            return;
        }

        Vector3 jointDir = GetJointDirection(userId, (int)joint, userTurned, true).normalized;
        Quaternion jointRotation = jointDir != Vector3.zero ? Quaternion.FromToRotation(baseDir, jointDir) : Quaternion.identity;

        if (!flip)
        {
            Vector3 mirroredAngles = jointRotation.eulerAngles;
            mirroredAngles.y = -mirroredAngles.y;
            mirroredAngles.z = -mirroredAngles.z;

            jointRotation = Quaternion.Euler(mirroredAngles);
        }

        if (jointRotation != Quaternion.identity)
        {
            // Smoothly transition to the new rotation
            Quaternion newRotation = Kinect2AvatarRot(jointRotation, boneIndex);

            if (externalRootMotion)
            {
                newRotation = transform.rotation * newRotation;
            }

            if (smoothFactor != 0f)
                boneTransform.rotation = Quaternion.Slerp(boneTransform.rotation, newRotation, smoothFactor * Time.deltaTime);
            else
                boneTransform.rotation = newRotation;
        }

    }

    public Vector3 GetJointDirection(Int64 userId, int joint, bool flipX, bool flipZ)
    {
        //if (dictUserIdToIndex.ContainsKey(userId))
        //{
        //    int index = dictUserIdToIndex[userId];

        //if (index >= 0 && index < sensorData.bodyCount &&
        //    bodyFrame.bodyData[index].bIsTracked != 0)
        if (bodyData.bIsTracked != 0)
        {
            if (joint >= 0 && joint < 25)
            {
                JointData jointData = bodyData.joint[joint];
                Vector3 jointDir = jointData.direction;

                if (flipX)
                    jointDir.x = -jointDir.x;

                if (flipZ)
                    jointDir.z = -jointDir.z;

                return jointDir;
            }
        }
        //}

        return Vector3.zero;
    }

    // Apply the rotations tracked by kinect to fingers (one joint = multiple bones)
    protected void TransformSpecialBoneFingers(Int64 userId, int joint, int boneIndex, bool flip, bool isJointTracked)
    {
        // check for hand grips
        if (joint == (int)JointType.HandTipLeft || joint == (int)JointType.ThumbLeft)
        {
            if (lastLeftHandEvent == HandEventType.Grip)
            {
                if (!bLeftFistDone && !IsUserTurnedAround(userId))
                {
                    float angleSign = !mirroredMovement /**(boneIndex == 27 || boneIndex == 29)*/ ? -1f : -1f;
                    float angleRot = angleSign * 60f;

                    TransformSpecialBoneFist(boneIndex, angleRot);
                    bLeftFistDone = (boneIndex >= 29);
                }

                return;
            }
            else if (bLeftFistDone && lastLeftHandEvent == HandEventType.Release)
            {
                TransformSpecialBoneUnfist(boneIndex);
                bLeftFistDone = !(boneIndex >= 29);
            }
        }
        else if (joint == (int)JointType.HandTipRight || joint == (int)JointType.ThumbRight)
        {
            if (lastRightHandEvent == HandEventType.Grip)
            {
                if (!bRightFistDone && !IsUserTurnedAround(userId))
                {
                    float angleSign = !mirroredMovement /**(boneIndex == 27 || boneIndex == 29)*/ ? -1f : -1f;
                    float angleRot = angleSign * 60f;

                    TransformSpecialBoneFist(boneIndex, angleRot);
                    bRightFistDone = (boneIndex >= 29);
                }

                return;
            }
            else if (bRightFistDone && lastRightHandEvent == HandEventType.Release)
            {
                TransformSpecialBoneUnfist(boneIndex);
                bRightFistDone = !(boneIndex >= 29);
            }
        }

        // get the animator component
        //Animator animatorComponent = GetComponent<Animator>();
        if (!animatorComponent || !isJointTracked)
            return;

        // Get Kinect joint orientation
        Quaternion jointRotation = GetJointOrientation(userId, joint, flip);
        if (jointRotation == Quaternion.identity)
            return;

        // calculate the new orientation
        Quaternion newRotation = Kinect2AvatarRot(jointRotation, boneIndex);

        if (externalRootMotion)
        {
            newRotation = transform.rotation * newRotation;
        }

        // get the list of bones
        //List<HumanBodyBones> alBones = flip ? specialIndex2MultiBoneMap[boneIndex] : specialIndex2MirrorBoneMap[boneIndex];
        List<HumanBodyBones> alBones = specialIndex2MultiBoneMap[boneIndex];

        // Smoothly transition to the new rotation
        for (int i = 0; i < alBones.Count; i++)
        {
            Transform boneTransform = animatorComponent.GetBoneTransform(alBones[i]);
            if (!boneTransform)
                continue;

            if (smoothFactor != 0f)
                boneTransform.rotation = Quaternion.Slerp(boneTransform.rotation, newRotation, smoothFactor * Time.deltaTime);
            else
                boneTransform.rotation = newRotation;
        }
    }

    public bool IsUserTurnedAround(Int64 userId)
    {
        //if (dictUserIdToIndex.ContainsKey(userId))
        //{
        //    int index = dictUserIdToIndex[userId];

        //    if (index >= 0 && index < sensorData.bodyCount &&
        //       bodyFrame.bodyData[index].bIsTracked != 0)
        if (bodyData.bIsTracked != 0)
        {
            return bodyData.isTurnedAround;
        }
        //}

        return false;
    }


    public Quaternion GetJointOrientation(Int64 userId, int joint, bool flip)
    {
        //if (dictUserIdToIndex.ContainsKey(userId))
        //{
        //    int index = dictUserIdToIndex[userId];

        //    if (index >= 0 && index < sensorData.bodyCount &&
        //       bodyFrame.bodyData[index].bIsTracked != 0)
        if (bodyData.bIsTracked != 0)
        {
            if (flip)
                return bodyData.joint[joint].normalRotation;
            else
                return bodyData.joint[joint].mirroredRotation;
        }
        //}

        return Quaternion.identity;
    }

    // applies the muscle limits for humanoid avatar
    private void CheckMuscleLimits()
    {
        if (humanPoseHandler == null)
            return;

        humanPoseHandler.GetHumanPose(ref humanPose);

        //Debug.Log(playerId + " - Trans: " + transform.position + ", body: " + humanPose.bodyPosition);

        bool isPoseChanged = false;

        float muscleMin = -1f;
        float muscleMax = 1f;

        for (int i = 0; i < humanPose.muscles.Length; i++)
        {
            if (float.IsNaN(humanPose.muscles[i]))
            {
                //humanPose.muscles[i] = 0f;
                continue;
            }

            if (humanPose.muscles[i] < muscleMin)
            {
                humanPose.muscles[i] = muscleMin;
                isPoseChanged = true;
            }
            else if (humanPose.muscles[i] > muscleMax)
            {
                humanPose.muscles[i] = muscleMax;
                isPoseChanged = true;
            }
        }

        if (isPoseChanged)
        {
            //Quaternion localBodyRot = Quaternion.Inverse(transform.rotation) * humanPose.bodyRotation;
            Quaternion localBodyRot = Quaternion.Inverse(initialHipsRotation) * humanPose.bodyRotation;

            // recover the body position & orientation
            //humanPose.bodyPosition = Vector3.zero;
            //humanPose.bodyPosition.y = initialHipsPosition.y;
            humanPose.bodyPosition = initialHipsPosition;
            humanPose.bodyRotation = localBodyRot; // Quaternion.identity;

            humanPoseHandler.SetHumanPose(ref humanPose);
            //Debug.Log("  Human pose updated.");
        }

    }
    // Apply the rotations needed to transform fingers to fist
    protected void TransformSpecialBoneFist(int boneIndex, float angle)
    {
        //		// do fist only for fingers
        //		if(boneIndex != 27 && boneIndex != 28)
        //			return;

        // get the animator component
        //Animator animatorComponent = GetComponent<Animator>();
        if (!animatorComponent)
            return;

        // get the list of bones
        List<HumanBodyBones> alBones = specialIndex2MultiBoneMap[boneIndex];

        for (int i = 0; i < alBones.Count; i++)
        {
            if (i < 1 && (boneIndex == 29 || boneIndex == 30))  // skip the first two thumb bones
                continue;

            HumanBodyBones bone = alBones[i];
            Transform boneTransform = animatorComponent.GetBoneTransform(bone);

            // set the fist rotation
            if (boneTransform && fingerBoneLocalAxes[bone] != Vector3.zero)
            {
                Quaternion qRotFinger = Quaternion.AngleAxis(angle, fingerBoneLocalAxes[bone]);
                boneTransform.localRotation = fingerBoneLocalRotations[bone] * qRotFinger;
            }
        }

    }

    // Apply the initial rotations fingers
    protected void TransformSpecialBoneUnfist(int boneIndex)
    {
        //		// do fist only for fingers
        //		if(boneIndex != 27 && boneIndex != 28)
        //			return;

        // get the animator component
        //Animator animatorComponent = GetComponent<Animator>();
        if (!animatorComponent)
            return;

        // get the list of bones
        List<HumanBodyBones> alBones = specialIndex2MultiBoneMap[boneIndex];

        for (int i = 0; i < alBones.Count; i++)
        {
            HumanBodyBones bone = alBones[i];
            Transform boneTransform = animatorComponent.GetBoneTransform(bone);

            // set the initial rotation
            if (boneTransform)
            {
                boneTransform.localRotation = fingerBoneLocalRotations[bone];
            }
        }
    }

    protected void MoveAvatar(Int64 UserID, BodyData bodydata)
    {

        if ((moveRate == 0f) || IsJointTracked(UserID, (int)JointType.SpineBase, bodydata))
        {

            return;
        }

        //get the position of user's spine base
        //Vector3 trans = GetUserPosition(UserID, bodydata);
        //if (flipLeftRight)
        //    trans.x = -trans.x;



        ////use the color overlay position if needed
        //if (posRelativeToCamera && posRelOverlayColor)
        //    {
        //        if (backgroundPlane && planeRectSet)
        //        {
        //            // get the plane overlay position
        //            trans = GetJointPosColorOverlay(UserID, (int)KinectInterop.JointType.SpineBase, planeRect, bodydata);
        //            trans.z = backgroundPlane.position.z - posRelativeToCamera.transform.position.z - 0.1f;  // 10cm offset
        //        }
        //        else
        //        {
        //            Rect backgroundRect = posRelativeToCamera.pixelRect;
        //            PortraitBackground portraitBack = PortraitBackground.Instance;

        //            if (portraitBack && portraitBack.enabled)
        //            {
        //                backgroundRect = portraitBack.GetBackgroundRect();
        //            }

        //            trans = GetJointPosColorOverlayy(UserID, (int)KinectInterop.JointType.SpineBase, posRelativeToCamera, backgroundRect, bodydata);
        //        }

        //        if (flipLeftRight)
        //            trans.x = -trans.x;
        //    }

        //// invert the z-coordinate, if needed
        //if (posRelativeToCamera && posRelInvertedZ)
        //{
        //    trans.z = -trans.z;
        //}
        //using (System.IO.StreamWriter file =
        //    new System.IO.StreamWriter(@"C:\Users\ullala\Desktop\testreceived.csv", true))
        //{
        //    //file.WriteLine("joint received: " + bodydatar.joint[1].kinectPos.ToString());
        //    file.WriteLine("avatar spine position:" + trans);

        //}
        //Debug.Log("avatar spine position:" + trans);
        //if (!offsetCalibrated)
        //{
        //    offsetCalibrated = true;

        //    offsetPos.x = trans.x;  // !mirroredMovement ? trans.x * moveRate : -trans.x * moveRate;
        //    offsetPos.y = trans.y;  // trans.y * moveRate;
        //    offsetPos.z = !mirroredMovement && !posRelativeToCamera ? -trans.z : trans.z;  // -trans.z * moveRate;

        //    if (posRelativeToCamera)
        //    {
        //        Vector3 cameraPos = posRelativeToCamera.transform.position;
        //        Vector3 bodyRootPos = bodyRoot != null ? bodyRoot.position : transform.position;
        //        Vector3 hipCenterPos = bodyRoot != null ? bodyRoot.position : (bones != null && bones.Length > 0 && bones[0] != null ? bones[0].position : Vector3.zero);

        //        float yRelToAvatar = 0f;
        //        if (verticalMovement)
        //        {
        //            yRelToAvatar = (trans.y - cameraPos.y) - (hipCenterPos - bodyRootPos).magnitude;
        //        }
        //        else
        //        {
        //            yRelToAvatar = bodyRootPos.y - cameraPos.y;
        //        }

        //        Vector3 relativePos = new Vector3(trans.x, yRelToAvatar, trans.z);
        //        Vector3 newBodyRootPos = cameraPos + relativePos;

        //        //				if(offsetNode != null)
        //        //				{
        //        //					newBodyRootPos += offsetNode.transform.position;
        //        //				}

        //        if (bodyRoot != null)
        //        {
        //            bodyRoot.position = newBodyRootPos;
        //        }
        //        else
        //        {
        //            transform.position = newBodyRootPos;
        //        }

        //        bodyRootPosition = newBodyRootPos;
        //    }
        //}

        //// transition to the new position
        //Vector3 targetPos = bodyRootPosition + Kinect2AvatarPos(trans, verticalMovement);

        //if (isRigidBody && !verticalMovement)
        //{
        //    // workaround for obeying the physics (e.g. gravity falling)
        //    targetPos.y = bodyRoot != null ? bodyRoot.position.y : transform.position.y;
        //}

        //// added by r618
        //if (horizontalOffset != 0f &&
        //    bones[5] != null && bones[11] != null)
        //{
        //    // { 5, HumanBodyBones.LeftUpperArm},
        //    // { 11, HumanBodyBones.RightUpperArm},
        //    //Vector3 dirSpine = bones[5].position - bones[11].position;
        //    Vector3 dirShoulders = bones[11].position - bones[5].position;
        //    targetPos += dirShoulders.normalized * horizontalOffset;
        //}

        //if (verticalMovement && verticalOffset != 0f &&
        //    bones[0] != null && bones[3] != null)
        //{
        //    Vector3 dirSpine = bones[3].position - bones[0].position;
        //    targetPos += dirSpine.normalized * verticalOffset;
        //}

        //if (forwardOffset != 0f &&
        //    bones[0] != null && bones[3] != null && bones[5] != null && bones[11] != null)
        //{
        //    Vector3 dirSpine = (bones[3].position - bones[0].position).normalized;
        //    Vector3 dirShoulders = (bones[11].position - bones[5].position).normalized;
        //    Vector3 dirForward = Vector3.Cross(dirShoulders, dirSpine).normalized;

        //    targetPos += dirForward * forwardOffset;
        //}

        //if (groundedFeet)
        //{
        //    // keep the current correction
        //    float fLastTgtY = targetPos.y;
        //    targetPos.y += fFootDistance;

        //    float fNewDistance = GetDistanceToGround();
        //    float fNewDistanceTime = Time.time;

        //    //			Debug.Log(string.Format("PosY: {0:F2}, LastY: {1:F2},  TgrY: {2:F2}, NewDist: {3:F2}, Corr: {4:F2}, Time: {5:F2}", bodyRoot != null ? bodyRoot.position.y : transform.position.y,
        //    //				fLastTgtY, targetPos.y, fNewDistance, fFootDistance, fNewDistanceTime));

        //    if (Mathf.Abs(fNewDistance) >= 0.01f && Mathf.Abs(fNewDistance - fFootDistanceInitial) >= maxFootDistanceGround)
        //    {
        //        if ((fNewDistanceTime - fFootDistanceTime) >= maxFootDistanceTime)
        //        {
        //            fFootDistance += (fNewDistance - fFootDistanceInitial);
        //            fFootDistanceTime = fNewDistanceTime;

        //            targetPos.y = fLastTgtY + fFootDistance;

        //            //					Debug.Log(string.Format("   >> change({0:F2})! - Corr: {1:F2}, LastY: {2:F2},  TgrY: {3:F2} at time {4:F2}", 
        //            //								(fNewDistance - fFootDistanceInitial), fFootDistance, fLastTgtY, targetPos.y, fFootDistanceTime));
        //        }
        //    }
        //    else
        //    {
        //        fFootDistanceTime = fNewDistanceTime;
        //    }
        //}

        //if (bodyRoot != null)
        //{
        //    bodyRoot.position = smoothFactor != 0f ?
        //        Vector3.Lerp(bodyRoot.position, targetPos, smoothFactor * Time.deltaTime) : targetPos;
        //}
        //else
        //{
        //    transform.position = smoothFactor != 0f ?
        //        Vector3.Lerp(transform.position, targetPos, smoothFactor * Time.deltaTime) : targetPos;
        //}
    }

    public bool IsJointTracked(Int64 userId, int joint, BodyData bodydata)
    {
        //if (dictUserIdToIndex.ContainsKey(userId))
        //{
        //    int index = dictUserIdToIndex[userId];

        //    if (index >= 0 && index < sensorData.bodyCount &&
        //        bodyFrame.bodyData[index].bIsTracked != 0)
        //    {
        if (joint >= 0 && joint < 25)
        {
            JointData jointData = bodyData.joint[joint];

            return ignoreInferredJoints ? (jointData.trackingState == TrackingState.Tracked) :
                (jointData.trackingState != TrackingState.NotTracked);
        }
        //}
        //}

        return false;
        //}
    }

    public Vector3 GetUserPosition(Int64 userId, BodyData bodydata)
    {
        //if (dictUserIdToIndex.ContainsKey(userId))
        //{
        //    int index = dictUserIdToIndex[userId];

        if (bodydata.bIsTracked != 0)
        {
            return bodydata.position;
        }
        //}

        return Vector3.zero;
    }

    public Vector2 GetJointPosColorOverlay(Int64 userId, int joint, Rect imageRect, BodyData bodydata)
    {
        //if (dictUserIdToIndex.ContainsKey(userId))
        //{
        //    int index = dictUserIdToIndex[userId];

        //if (index >= 0 && index < sensorData.bodyCount &&
        //    bodyFrame.bodyData[index].bIsTracked != 0)
        //{
        if (joint >= 0 && joint < 25)
        {
            JointData jointData = bodydata.joint[joint];
            Vector3 posJointRaw = jointData.kinectPos;

            if (posJointRaw != Vector3.zero)
            {
                // 3d position to depth
                Vector2 posDepth = MapSpacePointToDepthCoords(posJointRaw);
                ushort depthValue = GetDepthForPixel((int)posDepth.x, (int)posDepth.y);

                //if (posDepth != Vector2.zero && depthValue > 0 && sensorData != null)
                if (posDepth != Vector2.zero && depthValue > 0)
                {
                    // depth pos to color pos
                    Vector2 posColor = MapDepthPointToColorCoords(posDepth, depthValue);

                    if (!float.IsInfinity(posColor.x) && !float.IsInfinity(posColor.y))
                    {
                        //float xScaled = (float)posColor.x * imageRect.width / sensorData.colorImageWidth;
                        //float yScaled = (float)posColor.y * imageRect.height / sensorData.colorImageHeight;

                        float xScaled = (float)posColor.x * imageRect.width / 1920;
                        float yScaled = (float)posColor.y * imageRect.height / 1080;

                        float xImage = imageRect.x + xScaled;
                        float yImage = imageRect.y + imageRect.height - yScaled;

                        return new Vector2(xImage, yImage);
                    }
                }
            }
        }
        //}
        //}
        return Vector2.zero;
    }

    public Vector2 MapSpacePointToDepthCoords(Vector3 posPoint)
    {
        Vector2 posDepth = Vector2.zero;

        //if (kinectInitialized)
        //{
        posDepth = MapSpacePointToDepthCoordss(posPoint);
        //}

        return posDepth;
    }

    // returns depth frame coordinates for the given 3d Kinect-space point
    public static Vector2 MapSpacePointToDepthCoordss(Vector3 kinectPos)
    {
        Vector2 vPoint = Vector2.zero;

        //if (sensorData.sensorInterface != null)
        //{
        //vPoint = sensorData.sensorInterface.MapSpacePointToDepthCoords(sensorData, kinectPos);
        //vPoint = DepthSensorInterface.MapSpacePointToDepthCoords(sensorData, kinectPos);

        //}

        return vPoint;
    }


    public ushort GetDepthForPixel(int x, int y)
    {
        //if (sensorData != null && sensorData.depthImage != null)
        //{
        //    int index = y * sensorData.depthImageWidth + x;

        //    if (index >= 0 && index < sensorData.depthImage.Length)
        //    {
        //        return sensorData.depthImage[index];
        //    }
        //}

        return 0;
    }

    public Vector2 MapDepthPointToColorCoords(Vector2 posPoint, ushort depthValue)
    {
        Vector2 posColor = Vector2.zero;

        //if (kinectInitialized)
        //{
        //    posColor = MapDepthPointToColorCoords(sensorData, posPoint, depthValue);
        //}

        return posColor;
    }


    // Converts Kinect position to avatar skeleton position, depending on initial position, mirroring and move rate
    protected Vector3 Kinect2AvatarPos(Vector3 jointPosition, bool bMoveVertically)
    {
        float xPos = (jointPosition.x - offsetPos.x) * moveRate;
        float yPos = (jointPosition.y - offsetPos.y) * moveRate;
        float zPos = !mirroredMovement && !posRelativeToCamera ? (-jointPosition.z - offsetPos.z) * moveRate : (jointPosition.z - offsetPos.z) * moveRate;

        Vector3 newPosition = new Vector3(xPos, bMoveVertically ? yPos : 0f, zPos);

        Quaternion posRotation = mirroredMovement ? Quaternion.Euler(0f, 180f, 0f) * initialRotation : initialRotation;
        newPosition = posRotation * newPosition;

        if (offsetNode != null)
        {
            //newPosition += offsetNode.transform.position;
            newPosition = offsetNode.transform.position;
        }

        return newPosition;
    }

    public Vector3 GetJointPosColorOverlayy(Int64 userId, int joint, Camera camera, Rect imageRect, BodyData bodydata)
    {
        //if (dictUserIdToIndex.ContainsKey(userId) && camera != null)
        //{
        //    int index = dictUserIdToIndex[userId];

        //    if (index >= 0 && index < sensorData.bodyCount &&
        //       bodyFrame.bodyData[index].bIsTracked != 0)
        //    {
        if (joint >= 0 && joint < 25)
        {
            JointData jointData = bodydata.joint[joint];
            Vector3 posJointRaw = jointData.kinectPos;

            if (posJointRaw != Vector3.zero)
            {
                // 3d position to depth
                Vector2 posDepth = MapSpacePointToDepthCoords(posJointRaw);
                ushort depthValue = GetDepthForPixel((int)posDepth.x, (int)posDepth.y);

                //if (posDepth != Vector2.zero && depthValue > 0 && sensorData != null)
                if (posDepth != Vector2.zero && depthValue > 0)
                {
                    // depth pos to color pos
                    Vector2 posColor = MapDepthPointToColorCoords(posDepth, depthValue);

                    if (!float.IsInfinity(posColor.x) && !float.IsInfinity(posColor.y))
                    {
                        //float xScaled = (float)posColor.x * imageRect.width / sensorData.colorImageWidth;
                        //float yScaled = (float)posColor.y * imageRect.height / sensorData.colorImageHeight;

                        float xScaled = (float)posColor.x * imageRect.width / 1920;
                        float yScaled = (float)posColor.y * imageRect.height / 1080;

                        float xScreen = imageRect.x + xScaled;
                        //float yScreen = camera.pixelHeight - (imageRect.y + yScaled);
                        float yScreen = imageRect.y + imageRect.height - yScaled;

                        Plane cameraPlane = new Plane(camera.transform.forward, camera.transform.position);
                        float zDistance = cameraPlane.GetDistanceToPoint(posJointRaw);
                        //float zDistance = (jointData.kinectPos - camera.transform.position).magnitude;

                        //Vector3 vPosJoint = camera.ViewportToWorldPoint(new Vector3(xNorm, yNorm, zDistance));
                        Vector3 vPosJoint = camera.ScreenToWorldPoint(new Vector3(xScreen, yScreen, zDistance));

                        return vPosJoint;
                    }
                }
            }
            //}
            //}
        }

        return Vector3.zero;
    }

    protected virtual float GetDistanceToGround()
    {
        if (leftFoot == null && rightFoot == null)
        {
            //			Animator animatorComponent = GetComponent<Animator>();
            //
            //			if(animatorComponent)
            //			{
            //				leftFoot = animatorComponent.GetBoneTransform(HumanBodyBones.LeftFoot);
            //				rightFoot = animatorComponent.GetBoneTransform(HumanBodyBones.RightFoot);
            //			}

            leftFoot = GetBoneTransform(GetBoneIndexByJoint(JointType.FootLeft, false));
            rightFoot = GetBoneTransform(GetBoneIndexByJoint(JointType.FootRight, false));

            if (leftFoot == null || rightFoot == null)
            {
                leftFoot = GetBoneTransform(GetBoneIndexByJoint(JointType.AnkleLeft, false));
                rightFoot = GetBoneTransform(GetBoneIndexByJoint(JointType.AnkleRight, false));
            }
        }

        float fDistMin = 1000f;
        float fDistLeft = leftFoot ? GetTransformDistanceToGround(leftFoot) : fDistMin;
        float fDistRight = rightFoot ? GetTransformDistanceToGround(rightFoot) : fDistMin;
        fDistMin = Mathf.Abs(fDistLeft) < Mathf.Abs(fDistRight) ? fDistLeft : fDistRight;

        if (fDistMin == 1000f)
        {
            fDistMin = 0f; // fFootDistanceInitial;
        }

        //		Debug.Log (string.Format ("LFootY: {0:F2}, Dist: {1:F2}, RFootY: {2:F2}, Dist: {3:F2}, Min: {4:F2}", leftFoot ? leftFoot.position.y : 0f, fDistLeft,
        //						rightFoot ? rightFoot.position.y : 0f, fDistRight, fDistMin));

        return fDistMin;
    }

    protected virtual float GetTransformDistanceToGround(Transform trans)
    {
        if (!trans)
            return 0f;

        //		RaycastHit hit;
        //		if(Physics.Raycast(trans.position, Vector3.down, out hit, 2f, raycastLayers))
        //		{
        //			return -hit.distance;
        //		}
        //		else if(Physics.Raycast(trans.position, Vector3.up, out hit, 2f, raycastLayers))
        //		{
        //			return hit.distance;
        //		}
        //		else
        //		{
        //			if (trans.position.y < 0)
        //				return -trans.position.y;
        //			else
        //				return 1000f;
        //		}

        return -trans.position.y;
    }


    protected static readonly Dictionary<JointType, int> jointMap2boneIndex = new Dictionary<JointType, int>
    {
        {JointType.SpineBase, 0},
        {JointType.SpineMid, 1},
        {JointType.SpineShoulder, 2},
        {JointType.Neck, 3},
        {JointType.Head, 4},

        {JointType.ShoulderLeft, 5},
        {JointType.ElbowLeft, 6},
        {JointType.WristLeft, 7},
        {JointType.HandLeft, 8},

        {JointType.HandTipLeft, 9},
        {JointType.ThumbLeft, 10},

        {JointType.ShoulderRight, 11},
        {JointType.ElbowRight, 12},
        {JointType.WristRight, 13},
        {JointType.HandRight, 14},

        {JointType.HandTipRight, 15},
        {JointType.ThumbRight, 16},

        {JointType.HipLeft, 17},
        {JointType.KneeLeft, 18},
        {JointType.AnkleLeft, 19},
        {JointType.FootLeft, 20},

        {JointType.HipRight, 21},
        {JointType.KneeRight, 22},
        {JointType.AnkleRight, 23},
        {JointType.FootRight, 24},
    };

    protected static readonly Dictionary<JointType, int> mirrorJointMap2boneIndex = new Dictionary<JointType, int>
    {
        {JointType.SpineBase, 0},
        {JointType.SpineMid, 1},
        {JointType.SpineShoulder, 2},
        {JointType.Neck, 3},
        {JointType.Head, 4},

        {JointType.ShoulderRight, 5},
        {JointType.ElbowRight, 6},
        {JointType.WristRight, 7},
        {JointType.HandRight, 8},

        {JointType.HandTipRight, 9},
        {JointType.ThumbRight, 10},

        {JointType.ShoulderLeft, 11},
        {JointType.ElbowLeft, 12},
        {JointType.WristLeft, 13},
        {JointType.HandLeft, 14},

        {JointType.HandTipLeft, 15},
        {JointType.ThumbLeft, 16},

        {JointType.HipRight, 17},
        {JointType.KneeRight, 18},
        {JointType.AnkleRight, 19},
        {JointType.FootRight, 20},

        {JointType.HipLeft, 21},
        {JointType.KneeLeft, 22},
        {JointType.AnkleLeft, 23},
        {JointType.FootLeft, 24},
    };

    protected static readonly Dictionary<int, HumanBodyBones> boneIndex2MecanimMap = new Dictionary<int, HumanBodyBones>
    {
        {0, HumanBodyBones.Hips},
        {1, HumanBodyBones.Spine},
        {2, HumanBodyBones.Chest},
        {3, HumanBodyBones.Neck},
        {4, HumanBodyBones.Head},

        {5, HumanBodyBones.LeftUpperArm},
        {6, HumanBodyBones.LeftLowerArm},
        {7, HumanBodyBones.LeftHand},
        {8, HumanBodyBones.LeftIndexProximal},
        {9, HumanBodyBones.LeftIndexIntermediate},
        {10, HumanBodyBones.LeftThumbProximal},

        {11, HumanBodyBones.RightUpperArm},
        {12, HumanBodyBones.RightLowerArm},
        {13, HumanBodyBones.RightHand},
        {14, HumanBodyBones.RightIndexProximal},
        {15, HumanBodyBones.RightIndexIntermediate},
        {16, HumanBodyBones.RightThumbProximal},

        {17, HumanBodyBones.LeftUpperLeg},
        {18, HumanBodyBones.LeftLowerLeg},
        {19, HumanBodyBones.LeftFoot},
        {20, HumanBodyBones.LeftToes},

        {21, HumanBodyBones.RightUpperLeg},
        {22, HumanBodyBones.RightLowerLeg},
        {23, HumanBodyBones.RightFoot},
        {24, HumanBodyBones.RightToes},

        {25, HumanBodyBones.LeftShoulder},
        {26, HumanBodyBones.RightShoulder},
        {27, HumanBodyBones.LeftIndexProximal},
        {28, HumanBodyBones.RightIndexProximal},
        {29, HumanBodyBones.LeftThumbProximal},
        {30, HumanBodyBones.RightThumbProximal},
    };

    protected static readonly Dictionary<int, JointType> boneIndex2JointMap = new Dictionary<int, JointType>
    {
        {0, JointType.SpineBase},
        {1, JointType.SpineMid},
        {2, JointType.SpineShoulder},
        {3, JointType.Neck},
        {4, JointType.Head},

        {5, JointType.ShoulderLeft},
        {6, JointType.ElbowLeft},
        {7, JointType.WristLeft},
        {8, JointType.HandLeft},

        {9, JointType.HandTipLeft},
        {10, JointType.ThumbLeft},

        {11, JointType.ShoulderRight},
        {12, JointType.ElbowRight},
        {13, JointType.WristRight},
        {14, JointType.HandRight},

        {15, JointType.HandTipRight},
        {16, JointType.ThumbRight},

        {17, JointType.HipLeft},
        {18, JointType.KneeLeft},
        {19, JointType.AnkleLeft},
        {20, JointType.FootLeft},

        {21, JointType.HipRight},
        {22, JointType.KneeRight},
        {23, JointType.AnkleRight},
        {24, JointType.FootRight},
    };


    protected static readonly Dictionary<int, List<HumanBodyBones>> specialIndex2MultiBoneMap = new Dictionary<int, List<HumanBodyBones>>
    {
        {27, new List<HumanBodyBones> {  // left fingers
				HumanBodyBones.LeftIndexProximal,
                HumanBodyBones.LeftIndexIntermediate,
                HumanBodyBones.LeftIndexDistal,
                HumanBodyBones.LeftMiddleProximal,
                HumanBodyBones.LeftMiddleIntermediate,
                HumanBodyBones.LeftMiddleDistal,
                HumanBodyBones.LeftRingProximal,
                HumanBodyBones.LeftRingIntermediate,
                HumanBodyBones.LeftRingDistal,
                HumanBodyBones.LeftLittleProximal,
                HumanBodyBones.LeftLittleIntermediate,
                HumanBodyBones.LeftLittleDistal,
            }},
        {28, new List<HumanBodyBones> {  // right fingers
				HumanBodyBones.RightIndexProximal,
                HumanBodyBones.RightIndexIntermediate,
                HumanBodyBones.RightIndexDistal,
                HumanBodyBones.RightMiddleProximal,
                HumanBodyBones.RightMiddleIntermediate,
                HumanBodyBones.RightMiddleDistal,
                HumanBodyBones.RightRingProximal,
                HumanBodyBones.RightRingIntermediate,
                HumanBodyBones.RightRingDistal,
                HumanBodyBones.RightLittleProximal,
                HumanBodyBones.RightLittleIntermediate,
                HumanBodyBones.RightLittleDistal,
            }},
        {29, new List<HumanBodyBones> {  // left thumb
				HumanBodyBones.LeftThumbProximal,
                HumanBodyBones.LeftThumbIntermediate,
                HumanBodyBones.LeftThumbDistal,
            }},
        {30, new List<HumanBodyBones> {  // right thumb
				HumanBodyBones.RightThumbProximal,
                HumanBodyBones.RightThumbIntermediate,
                HumanBodyBones.RightThumbDistal,
            }},
    };


    protected static readonly Dictionary<int, List<JointType>> specIndex2JointMap = new Dictionary<int, List<JointType>>
    {
        {25, new List<JointType> {JointType.ShoulderLeft, JointType.SpineShoulder} },
        {26, new List<JointType> {JointType.ShoulderRight, JointType.SpineShoulder} },
        {27, new List<JointType> {JointType.HandTipLeft, JointType.HandLeft} },
        {28, new List<JointType> {JointType.HandTipRight, JointType.HandRight} },
        {29, new List<JointType> {JointType.ThumbLeft, JointType.HandLeft} },
        {30, new List<JointType> {JointType.ThumbRight, JointType.HandRight} },
    };

    protected static readonly Dictionary<int, JointType> boneIndex2MirrorJointMap = new Dictionary<int, JointType>
    {
        {0, JointType.SpineBase},
        {1, JointType.SpineMid},
        {2, JointType.SpineShoulder},
        {3, JointType.Neck},
        {4, JointType.Head},

        {5, JointType.ShoulderRight},
        {6, JointType.ElbowRight},
        {7, JointType.WristRight},
        {8, JointType.HandRight},

        {9, JointType.HandTipRight},
        {10, JointType.ThumbRight},

        {11, JointType.ShoulderLeft},
        {12, JointType.ElbowLeft},
        {13, JointType.WristLeft},
        {14, JointType.HandLeft},

        {15, JointType.HandTipLeft},
        {16, JointType.ThumbLeft},

        {17, JointType.HipRight},
        {18, JointType.KneeRight},
        {19, JointType.AnkleRight},
        {20, JointType.FootRight},

        {21, JointType.HipLeft},
        {22, JointType.KneeLeft},
        {23, JointType.AnkleLeft},
        {24, JointType.FootLeft},
    };

    protected static readonly Dictionary<int, List<JointType>> specIndex2MirrorMap = new Dictionary<int, List<JointType>>
    {
        {25, new List<JointType> {JointType.ShoulderRight, JointType.SpineShoulder} },
        {26, new List<JointType> {JointType.ShoulderLeft, JointType.SpineShoulder} },
        {27, new List<JointType> {JointType.HandTipRight, JointType.HandRight} },
        {28, new List<JointType> {JointType.HandTipLeft, JointType.HandLeft} },
        {29, new List<JointType> {JointType.ThumbRight, JointType.HandRight} },
        {30, new List<JointType> {JointType.ThumbLeft, JointType.HandLeft} },
    };

    protected static readonly Dictionary<int, HumanBodyBones> fingerIndex2MecanimMap = new Dictionary<int, HumanBodyBones>
    {
        {0, HumanBodyBones.LeftThumbProximal},
        {1, HumanBodyBones.LeftThumbIntermediate},
        {2, HumanBodyBones.LeftThumbDistal},

        {3, HumanBodyBones.LeftIndexProximal},
        {4, HumanBodyBones.LeftIndexIntermediate},
        {5, HumanBodyBones.LeftIndexDistal},

        {6, HumanBodyBones.LeftMiddleProximal},
        {7, HumanBodyBones.LeftMiddleIntermediate},
        {8, HumanBodyBones.LeftMiddleDistal},

        {9, HumanBodyBones.LeftRingProximal},
        {10, HumanBodyBones.LeftRingIntermediate},
        {11, HumanBodyBones.LeftRingDistal},

        {12, HumanBodyBones.LeftLittleProximal},
        {13, HumanBodyBones.LeftLittleIntermediate},
        {14, HumanBodyBones.LeftLittleDistal},

        {15, HumanBodyBones.RightThumbProximal},
        {16, HumanBodyBones.RightThumbIntermediate},
        {17, HumanBodyBones.RightThumbDistal},

        {18, HumanBodyBones.RightIndexProximal},
        {19, HumanBodyBones.RightIndexIntermediate},
        {20, HumanBodyBones.RightIndexDistal},

        {21, HumanBodyBones.RightMiddleProximal},
        {22, HumanBodyBones.RightMiddleIntermediate},
        {23, HumanBodyBones.RightMiddleDistal},

        {24, HumanBodyBones.RightRingProximal},
        {25, HumanBodyBones.RightRingIntermediate},
        {26, HumanBodyBones.RightRingDistal},

        {27, HumanBodyBones.RightLittleProximal},
        {28, HumanBodyBones.RightLittleIntermediate},
        {29, HumanBodyBones.RightLittleDistal}
    };


    public struct BodyDataserialize
    {
        public float timestamp;

        // parameters filled in by the sensor interface
        public Int64 liTrackingID;
        public Vector3 position;
        public Quaternion orientation;  // deprecated

        //public JointData[] joint;
        public TrackingState[] trackingState;
        //public Vector3[] kinectPos;
        public float[] kinectPosx;
        public float[] kinectPosy;
        public float[] kinectPosz;
        //public Vector3[] positionj;
        public float[] positionjx;
        public float[] positionjy;
        public float[] positionjz;
        //public Quaternion[] orientationj;  // deprecated
        public float[] orientationjx;
        public float[] orientationjy;
        public float[] orientationjz;
        public float[] orientationjw;

        //public Vector3[] posPrev;
        public float[] posPrevx;
        public float[] posPrevy;
        public float[] posPrevz;
        //public Vector3[] posRel;
        public float[] posRelx;
        public float[] posRely;
        public float[] posRelz;
        //public Vector3[] posVel;
        public float[] posVelx;
        public float[] posVely;
        public float[] posVelz;

        // KM calculated parameters
        //public Vector3[] direction;
        public float[] directionx;
        public float[] directiony;
        public float[] directionz;
        //public Quaternion[] normalRotationj;
        public float[] normalRotationjx;
        public float[] normalRotationjy;
        public float[] normalRotationjz;
        public float[] normalRotationjw;
        //public Quaternion[] mirroredRotationj;
        public float[] mirroredRotationjx;
        public float[] mirroredRotationjy;
        public float[] mirroredRotationjz;
        public float[] mirroredRotationjw;

        // Constraint parameters
        public float[] lastAngle;

        // KM calculated parameters
        public Quaternion normalRotation;
        public Quaternion mirroredRotation;

        public Vector3 hipsDirection;
        public Vector3 shouldersDirection;
        public float bodyTurnAngle;
        //public float bodyFullAngle;
        //public float turnAroundFactor;
        public bool isTurnedAround;
        public float turnFaceLastTrackedTime;
        public float turnLeftShoulderTrackedTime;
        public float turnRightShoulderTrackedTime;
        public float turnShoulderDistTrackedTime;

        public Quaternion leftHandOrientation;
        public Quaternion rightHandOrientation;

        public Quaternion headOrientation;

        public HandState leftHandState;
        public TrackingConfidence leftHandConfidence;
        public HandState rightHandState;
        public TrackingConfidence rightHandConfidence;

        public uint dwClippedEdges;
        public short bIsTracked;
        public short bIsRestricted;
    }


    timestampedBodyData processJSON(string content)
    {
        try
        {
            //Debug.Log(content);
            //Message(commands[0]);
            //logstring = commands[0];
            //debuglog = true;


            //string jsontext = datar.Split('#', '$')[1];
            //int startIndex = datar.LastIndexOf('#');
            //int endIndex = datar.LastIndexOf('$');
            //if (startIndex != -1 && endIndex != -1)
            //{
            //    string jsontext = datar.Substring(startIndex + 1, endIndex - startIndex - 1);

            if (IsValidJson(content))
            {
                bodyframe = FromJson(content);
                //logstring = "validjson";
                //debuglog = true;
                updateready = true;
                //logstring = content;
                //debuglog = true;
                //using (System.IO.StreamWriter file =
                //    new System.IO.StreamWriter(@"C:\Users\ullala\Desktop\testreceived.csv", true))
                ////new System.IO.StreamWriter(@"C:\Users\ullala\Desktop\testreceived.csv"))
                //{
                //    file.WriteLine("joint received: " + bodydatar.joint[1].kinectPos.ToString());
                //    //file.WriteLine(commands[0]);

                //}



            }

            ////}
            else
            {
                //logstring = "not valid json";
                Debug.Log("not valid JSON");
                //debuglog = true;
                //logstring = content;
                //debuglog = true;
                //using (System.IO.StreamWriter file =
                //   new System.IO.StreamWriter(@"C:\Users\ullala\Desktop\testreceived_bad.csv", true))
                ////new System.IO.StreamWriter(@"C:\Users\ullala\Desktop\testreceived.csv"))
                //{
                //    //file.WriteLine("joint received: " + bodydatar.joint[1].kinectPos.ToString());
                //    file.WriteLine(content + "____________________________");

                //}
            }
            //script.UpdateAvatar(bodydatar);
        }
        catch (Exception e)
        {
            Debug.Log(e.ToString());
            //continue;
        }
        return bodyframe;
    }

    private static bool IsValidJson(string strInput)
    {
        strInput = strInput.Trim();
        if ((strInput.StartsWith("{") && strInput.EndsWith("}")) || //For object
            (strInput.StartsWith("[") && strInput.EndsWith("]"))) //For array
        {
            try
            {
                var obj = JToken.Parse(strInput);
                return true;
            }
            catch (JsonReaderException jex)
            {
                //Exception in parsing json
                Console.WriteLine(jex.Message);
                return false;
            }
            catch (Exception ex) //some other exception
            {
                Console.WriteLine(ex.ToString());
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    public static timestampedBodyData FromJson(string jsonstring)
    {

        timestampedBodyData frame;

        BodyDataserialize bodydatatemp = JsonConvert.DeserializeObject<BodyDataserialize>(jsonstring);

        frame.timestamp = bodydatatemp.timestamp;

        BodyData bodydatar = new BodyData(25);

        bodydatar.liTrackingID = bodydatatemp.liTrackingID;


        bodydatar.position.x = bodydatatemp.position.x;
        bodydatar.position.y = bodydatatemp.position.y;
        bodydatar.position.z = bodydatatemp.position.z;

        bodydatar.orientation.x = bodydatatemp.orientation.x;
        bodydatar.orientation.y = bodydatatemp.orientation.y;
        bodydatar.orientation.z = bodydatatemp.orientation.z;
        bodydatar.orientation.w = bodydatatemp.orientation.w;



        for (int i = 0; i < 25; i++)
        {

            bodydatar.joint[i].trackingState = bodydatatemp.trackingState[i];
            bodydatar.joint[i].kinectPos.x = bodydatatemp.kinectPosx[i];
            bodydatar.joint[i].kinectPos.y = bodydatatemp.kinectPosy[i];
            bodydatar.joint[i].kinectPos.z = bodydatatemp.kinectPosz[i];
            bodydatar.joint[i].position.x = bodydatatemp.positionjx[i];
            bodydatar.joint[i].position.y = bodydatatemp.positionjy[i];
            bodydatar.joint[i].position.z = bodydatatemp.positionjz[i];
            bodydatar.joint[i].orientation.x = bodydatatemp.orientationjx[i];
            bodydatar.joint[i].orientation.y = bodydatatemp.orientationjy[i];
            bodydatar.joint[i].orientation.z = bodydatatemp.orientationjz[i];
            bodydatar.joint[i].orientation.w = bodydatatemp.orientationjw[i];
            bodydatar.joint[i].posPrev.x = bodydatatemp.posPrevx[i];
            bodydatar.joint[i].posPrev.y = bodydatatemp.posPrevy[i];
            bodydatar.joint[i].posPrev.z = bodydatatemp.posPrevz[i];
            bodydatar.joint[i].posRel.x = bodydatatemp.posRelx[i];
            bodydatar.joint[i].posRel.y = bodydatatemp.posRely[i];
            bodydatar.joint[i].posRel.z = bodydatatemp.posRelz[i];
            bodydatar.joint[i].posVel.x = bodydatatemp.posVelx[i];
            bodydatar.joint[i].posVel.y = bodydatatemp.posVely[i];
            bodydatar.joint[i].posVel.z = bodydatatemp.posVelz[i];
            bodydatar.joint[i].direction.x = bodydatatemp.directionx[i];
            bodydatar.joint[i].direction.y = bodydatatemp.directiony[i];
            bodydatar.joint[i].direction.z = bodydatatemp.directionz[i];
            bodydatar.joint[i].normalRotation.x = bodydatatemp.normalRotationjx[i];
            bodydatar.joint[i].normalRotation.y = bodydatatemp.normalRotationjy[i];
            bodydatar.joint[i].normalRotation.z = bodydatatemp.normalRotationjz[i];
            bodydatar.joint[i].normalRotation.w = bodydatatemp.normalRotationjw[i];
            bodydatar.joint[i].mirroredRotation.x = bodydatatemp.mirroredRotationjx[i];
            bodydatar.joint[i].mirroredRotation.y = bodydatatemp.mirroredRotationjy[i];
            bodydatar.joint[i].mirroredRotation.z = bodydatatemp.mirroredRotationjz[i];
            bodydatar.joint[i].mirroredRotation.w = bodydatatemp.mirroredRotationjw[i];
            bodydatar.joint[i].lastAngle = bodydatatemp.lastAngle[i];


        }

        bodydatar.normalRotation.x = bodydatatemp.normalRotation.x;
        bodydatar.normalRotation.y = bodydatatemp.normalRotation.y;
        bodydatar.normalRotation.z = bodydatatemp.normalRotation.z;
        bodydatar.normalRotation.w = bodydatatemp.normalRotation.w;

        bodydatar.mirroredRotation.x = bodydatatemp.mirroredRotation.x;
        bodydatar.mirroredRotation.y = bodydatatemp.mirroredRotation.y;
        bodydatar.mirroredRotation.z = bodydatatemp.mirroredRotation.z;
        bodydatar.mirroredRotation.w = bodydatatemp.mirroredRotation.w;

        bodydatar.hipsDirection.x = bodydatatemp.hipsDirection.x;
        bodydatar.hipsDirection.y = bodydatatemp.hipsDirection.y;
        bodydatar.hipsDirection.z = bodydatatemp.hipsDirection.z;

        bodydatar.shouldersDirection.x = bodydatatemp.shouldersDirection.x;
        bodydatar.shouldersDirection.y = bodydatatemp.shouldersDirection.y;
        bodydatar.shouldersDirection.z = bodydatatemp.shouldersDirection.z;

        bodydatar.bodyTurnAngle = bodydatatemp.bodyTurnAngle;

        bodydatar.isTurnedAround = bodydatatemp.isTurnedAround;

        bodydatar.turnFaceLastTrackedTime = bodydatatemp.turnFaceLastTrackedTime;

        bodydatar.turnLeftShoulderTrackedTime = bodydatatemp.turnLeftShoulderTrackedTime;

        bodydatar.turnShoulderDistTrackedTime = bodydatatemp.turnShoulderDistTrackedTime;

        bodydatar.leftHandOrientation.x = bodydatatemp.leftHandOrientation.x;
        bodydatar.leftHandOrientation.y = bodydatatemp.leftHandOrientation.y;
        bodydatar.leftHandOrientation.z = bodydatatemp.leftHandOrientation.z;
        bodydatar.leftHandOrientation.w = bodydatatemp.leftHandOrientation.w;

        bodydatar.rightHandOrientation.x = bodydatatemp.rightHandOrientation.x;
        bodydatar.rightHandOrientation.y = bodydatatemp.rightHandOrientation.y;
        bodydatar.rightHandOrientation.z = bodydatatemp.rightHandOrientation.z;
        bodydatar.rightHandOrientation.w = bodydatatemp.rightHandOrientation.w;

        bodydatar.headOrientation.x = bodydatatemp.headOrientation.x;
        bodydatar.headOrientation.y = bodydatatemp.headOrientation.y;
        bodydatar.headOrientation.z = bodydatatemp.headOrientation.z;
        bodydatar.headOrientation.w = bodydatatemp.headOrientation.w;

        bodydatar.leftHandState = bodydatatemp.leftHandState;

        bodydatar.leftHandConfidence = bodydatatemp.leftHandConfidence;

        bodydatar.rightHandState = bodydatatemp.rightHandState;

        bodydatar.rightHandConfidence = bodydatatemp.rightHandConfidence;

        bodydatar.dwClippedEdges = bodydatatemp.dwClippedEdges;

        bodydatar.bIsTracked = bodydatatemp.bIsTracked;

        bodydatar.bIsRestricted = bodydatatemp.bIsRestricted;






        frame.bodydata = bodydatar;
        return frame;
    }

}
