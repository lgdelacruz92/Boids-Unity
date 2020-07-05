using UnityEngine;
using System.Collections.Generic;

public class BoidMovement : MonoBehaviour
{
    public Rigidbody boidRigidBody;
    public Transform boidTransform;
    public List<Vector3> rays;
    public Vector3 originalDir;
    public GameObject[] otherBoids;
    public bool avoidObstacle;

    private void Start()
    {
        initializeRays();
        originalDir = new Vector3(0, 0, 1);
        boidRigidBody.velocity = new Vector3(Random.value, Random.value, Random.value) * 2;
        otherBoids = GameObject.FindGameObjectsWithTag("Boid");
        avoidObstacle = true;
    }

    private void initializeRays()
    {
        for (float x = -1; x <= 1; x += 0.5f)
        {
            for (float y = -1; y <= 1; y += 0.5f)
            {
                for (float z = 0; z <= 1; z += 0.5f)
                {
                    rays.Add(new Vector3(x, y, z));
                }
            }
        }
    }

    void FixedUpdate()
    {
        RotateToDirection();


        Vector3 totalForce = new Vector3(0, 0, 0);

        if (OutOfBounds())
        {
            totalForce = GoToCenter();
        } else
        {
            totalForce = AvoidObstacles() + Alignment() + Cohesion() + Separation();
        }

        boidRigidBody.AddForce(totalForce);
        ScaleVelocity();
    }

    private Vector3 Alignment()
    {
        List<GameObject> neighbors = GetNeighbors();

        Vector3 alignmentForce = new Vector3(0, 0, 0);
        foreach (GameObject neighbor in neighbors)
        {
            alignmentForce += neighbor.GetComponent<Rigidbody>().velocity;
        }

        if (neighbors.Count > 0)
        {
            alignmentForce /= neighbors.Count;

        }

        return alignmentForce;
    }

    private Vector3 Cohesion()
    {
        List<GameObject> neighbors = GetNeighbors();

        Vector3 cohesionForce = new Vector3(0, 0, 0);
        foreach (GameObject neighbor in neighbors)
        {
            cohesionForce += neighbor.transform.position;
        }

        if (neighbors.Count > 0)
        {
            cohesionForce /= neighbors.Count;
        }
        return cohesionForce;
    }

    private Vector3 Separation()
    {
        List<GameObject> neighbors = GetNeighbors();

        Vector3 separationForce = new Vector3(0, 0, 0);
        foreach (GameObject neighbor in neighbors)
        {
            Vector3 separationDir = -1 * (neighbor.transform.position - boidTransform.position);
            if (separationDir.magnitude > 0)
            {
                float separationScalar = 1 / separationDir.magnitude;
                separationDir = (separationDir) * (separationScalar / separationDir.magnitude);
                separationForce += separationDir;
            }
        }

        return separationForce;
    }


    private Vector3 AvoidObstacles()
    {
        Vector3 avoidObstacleForce = new Vector3(0, 0, 0);
        if (avoidObstacle)
        {
            Vector3 startPoint = boidTransform.position;
            foreach (Vector3 ray in rays)
            {
                RaycastHit hit;
                // Does the ray intersect any objects excluding the player layer
                if (Physics.Raycast(boidTransform.position, boidTransform.TransformDirection(ray), out hit, 1))
                {
                    Vector3 direction = boidTransform.TransformDirection(ray);
                    //Debug.DrawRay(boidTransform.position, direction * hit.distance, Color.yellow);

                    Vector3 singleForce = (-1 * direction * hit.distance);
                    avoidObstacleForce += singleForce;
                }
                else
                {
                    Vector3 direction = boidTransform.TransformDirection(ray);
                    //Debug.DrawRay(boidTransform.position, direction * 1, Color.white);
                }
            }
        }

        return avoidObstacleForce;
    }

    private List<GameObject> GetNeighbors()
    {
        List<GameObject> neighbors = new List<GameObject>();
        foreach (GameObject otherBoid in otherBoids)
        {
            RaycastHit hit;
            Vector3 rayToNeighbor = boidTransform.TransformDirection(otherBoid.transform.position);
            if (otherBoid != this && Physics.Raycast(boidTransform.position, rayToNeighbor, out hit, 1))
            {
                neighbors.Add(otherBoid);
            }
        }

        return neighbors;
    }

    private void ScaleVelocity()
    {
        const int velocityScale = 2;
        Vector3 boidVel = boidRigidBody.velocity;
        float vellMag = boidVel.magnitude;
        boidRigidBody.velocity = boidVel * (velocityScale / vellMag);
    }

    private void RotateToDirection()
    {
        Vector3 currentDir = boidRigidBody.velocity;
        boidTransform.LookAt((currentDir * (1 / currentDir.magnitude)) + boidTransform.position);
    }

    private bool OutOfBounds()
    {
        float zPos = boidTransform.position.z;
        float yPos = boidTransform.position.y;
        float xPos = boidTransform.position.x;
        return (zPos < -5 || zPos > 5) || (yPos < 0 || yPos > 5) || (xPos < -5 || xPos > 5);
    }

    private Vector3 GoToCenter()
    {
        Vector3 center = new Vector3(0.0f, 2.5f, 0.0f);
        Vector3 centerDirection = center - boidTransform.position;
        
        return centerDirection;
    }
}
