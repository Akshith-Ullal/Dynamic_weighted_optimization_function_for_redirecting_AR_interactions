using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class rotobj : MonoBehaviour
{
    // Start is called before the first frame update
    //void Start()
    //{

    //}

    //// Update is called once per frame
    //void Update()
    //{

    //}
    float rotSpeed = 20;
    void OnMouseDrag()
    {
        float rotx = Input.GetAxis("Mouse X") * rotSpeed * Mathf.Deg2Rad;
        float roty = Input.GetAxis("Mouse Y") * rotSpeed * Mathf.Deg2Rad;
        float rotz = Input.GetAxis("Mouse Z") * rotSpeed * Mathf.Deg2Rad;

        transform.RotateAround(Vector3.up, -rotx);
        transform.RotateAround(Vector3.right, roty);
    }
}
