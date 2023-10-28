using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class printTextureSize : MonoBehaviour
{
    // Print texture size to the Console
    public Texture texture;
    void Start()
    {
        print("Size is " + texture.width + " by " + texture.height);
    }
}
