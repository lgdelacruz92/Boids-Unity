using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpawnBoids : MonoBehaviour
{
    public GameObject boidPrefab;
    // Start is called before the first frame update
    void Start()
    {

        for (int i = 0; i < 100; i++)
        {
            GameObject newBoid = Instantiate(boidPrefab);
            newBoid.transform.position = new Vector3(1, 2, 2);
        }
    }
}
