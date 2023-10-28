using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class angletest : MonoBehaviour
{
    // Start is called before the first frame update

    public GameObject Left_Forearm_Joint;
    public GameObject Left_Wrist_Joint;
    public GameObject Line_Renderer;
    public GameObject testsphere;
    public GameObject Left_Forearm_Joint_ref;
    public Vector3 gesturerefvector;
    public Quaternion Left_Forearm_Joint_01_correct;
    //public LineRenderer lineRenderer;
    void Start()
    {
        Left_Forearm_Joint = GameObject.Find("Robot Kyle/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01");
        Left_Wrist_Joint = GameObject.Find("Robot Kyle/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01/Left_Wrist_Joint_01");
        Left_Forearm_Joint_ref = GameObject.Find("Robot Kyle/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01/Left_Forearm_Joint_01_local_refgameobject");
        testsphere = GameObject.Find("test_sphere_local");
        //Line_Renderer = GameObject.Find("Robot Kyle/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01_local_line_renderer");
        //lineRenderer = Line_Renderer.GetComponent<LineRenderer>();
        Left_Forearm_Joint_01_correct = Left_Forearm_Joint.transform.rotation*Quaternion.Inverse(Left_Forearm_Joint_ref.transform.rotation);
       // gesturerefvector = Left_Forearm_Joint.transform.position - Left_Wrist_Joint.transform.position;
       // Left_Forearm_Joint_ref.transform.rotation = Quaternion.LookRotation(-gesturerefvector, Vector3.up);
      
    }

    // Update is called once per frame
    void Update()
    {
        ////Left_Forearm_Joint.transform.position = testsphere.transform.position;
        ////Left_Forearm_Joint.transform.position = new Vector3(5.0f,5.0f,5.0f);
        //Line_Renderer.GetComponent<LineRenderer>().SetPosition(0, Left_Forearm_Joint.transform.position);
        //Line_Renderer.GetComponent<LineRenderer>().SetPosition(1, testsphere.transform.position);
        gesturerefvector = testsphere.transform.position - Left_Forearm_Joint.transform.position;
        Left_Forearm_Joint_ref.transform.rotation = Quaternion.LookRotation(gesturerefvector, Vector3.up);
        ////Left_Forearm_Joint_ref.transform.rotation = Line_Renderer.transform.rotation;
        Left_Forearm_Joint.transform.rotation = Left_Forearm_Joint_ref.transform.rotation * Left_Forearm_Joint_01_correct;


        //Debug.Log("line renderer rotation:"+ Line_Renderer.transform.rotation);
        //Debug.Log("left forearm rotation:" + Left_Forearm_Joint.transform.rotation);
    }
}
