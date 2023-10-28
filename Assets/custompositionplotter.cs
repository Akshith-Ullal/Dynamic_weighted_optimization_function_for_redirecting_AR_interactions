using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using Newtonsoft.Json;

public class custompositionplotter : MonoBehaviour
{
    // Start is called before the first frame update

    public List<GameObject> jointc = new List<GameObject>();
    public List<Quaternion> reconvertedlist = new List<Quaternion>();

    public GameObject Root_dummy;
    public GameObject Hip_dummy;
    public GameObject Ribs_dummy;
    public GameObject Neck_dummy;
    public GameObject Head_dummy;

    public GameObject Left_Thigh_Joint_01_dummy;
    public GameObject Left_Knee_Joint_01_dummy;
    public GameObject Left_Ankle_Joint_01_dummy;
    public GameObject Left_Toe_Joint_01_dummy;


    public GameObject Right_Thigh_Joint_01_dummy;
    public GameObject Right_Knee_Joint_01_dummy;
    public GameObject Right_Ankle_Joint_01_dummy;
    public GameObject Right_Toe_Joint_01_dummy;


    public GameObject Left_Upper_Arm_Joint_01_dummy;
    public GameObject Left_Shoulder_Joint_01_dummy;
    public GameObject Left_Forearm_Joint_01_dummy;
    public GameObject Left_Wrist_Joint_01_dummy;
   
    public GameObject Right_Upper_Arm_Joint_01_dummy;
    public GameObject Right_Shoulder_Joint_01_dummy;
    public GameObject Right_Forearm_Joint_01_dummy;
    public GameObject Right_Wrist_Joint_01_dummy;
   
    void Start()
    {
        

        Root_dummy = GameObject.Find("Robot Kyle/Root");
        jointc.Add(Root_dummy);
        Hip_dummy = GameObject.Find("Robot Kyle/Root/Hip");
        jointc.Add(Hip_dummy);
        Ribs_dummy = GameObject.Find("Robot Kyle/Root/Ribs");
        jointc.Add(Ribs_dummy);
        Neck_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Neck");
        jointc.Add(Neck_dummy);
        Head_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Neck/Head");
        jointc.Add(Head_dummy);


        Left_Thigh_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Left_Thigh_Joint_01");
        jointc.Add(Left_Thigh_Joint_01_dummy);
        Left_Knee_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01");
        jointc.Add(Left_Knee_Joint_01_dummy);
        Left_Ankle_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01/Left_Ankle_Joint_01");
        jointc.Add(Left_Ankle_Joint_01_dummy);
        Left_Toe_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Left_Thigh_Joint_01/Left_Knee_Joint_01/Left_Ankle_Joint_01/Left_Toe_Joint_01");
        jointc.Add(Left_Toe_Joint_01_dummy);
      

        Right_Thigh_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Right_Thigh_Joint_01");
        jointc.Add(Right_Thigh_Joint_01_dummy);
        Right_Knee_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01");
        jointc.Add(Right_Knee_Joint_01_dummy);
        Right_Ankle_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01/Right_Ankle_Joint_01");
        jointc.Add(Right_Ankle_Joint_01_dummy);
        Right_Toe_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Hip/Right_Thigh_Joint_01/Right_Knee_Joint_01/Right_Ankle_Joint_01/Right_Toe_Joint_01");
        jointc.Add(Right_Toe_Joint_01_dummy);
       

       
        Left_Shoulder_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Left_Shoulder_Joint_01");
        jointc.Add(Left_Shoulder_Joint_01_dummy);
        Left_Upper_Arm_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01");
        jointc.Add(Left_Upper_Arm_Joint_01_dummy);
        Left_Forearm_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01");
        jointc.Add(Left_Forearm_Joint_01_dummy);
        Left_Wrist_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Left_Shoulder_Joint_01/Left_Upper_Arm_Joint_01/Left_Forearm_Joint_01/Left_Wrist_Joint_01");
        jointc.Add(Left_Wrist_Joint_01_dummy);
       

       
        Right_Shoulder_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Right_Shoulder_Joint_01");
        jointc.Add(Right_Shoulder_Joint_01_dummy);
        Right_Upper_Arm_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01");
        jointc.Add(Right_Upper_Arm_Joint_01_dummy);
        Right_Forearm_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01/Right_Forearm_Joint_01");
        jointc.Add(Right_Forearm_Joint_01_dummy);
        Right_Wrist_Joint_01_dummy = GameObject.Find("Robot Kyle/Root/Ribs/Right_Shoulder_Joint_01/Right_Upper_Arm_Joint_01/Right_Forearm_Joint_01/Right_Wrist_Joint_01");
        jointc.Add(Right_Wrist_Joint_01_dummy);
     

        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.P))
        {

            reconvertedlist = JsonConvert.DeserializeObject<List<Quaternion>>(File.ReadAllText(@"C:/Users/ullala/Desktop/IEEE_VR_journal_paper/front1stquadrantposelocations.json"));

            for (int i = 0; i < reconvertedlist.Count; i++)
            {
                jointc[i].transform.rotation = reconvertedlist[i];
            }
        }
    }
}
