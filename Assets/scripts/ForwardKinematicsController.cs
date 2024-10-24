using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ForwardKinematicsController : MonoBehaviour
{
    public List<ArticulationBody> Joints;

    [Range(-180.0f, 180.0f)] public float Joint1Angle;
    [Range(-180.0f, 180.0f)] public float Joint2Angle;
    [Range(-180.0f, 180.0f)] public float Joint3Angle;
    [Range(-180.0f, 180.0f)] public float Joint4Angle;
    [Range(-180.0f, 180.0f)] public float GripperAngle;

    readonly float l1 = 1.0f * scalingFactor;
    readonly float l2 = 0.3f * scalingFactor;
    readonly float l3 = 0.3f * scalingFactor;
    readonly float l4 = 0.3f * scalingFactor;
    static readonly float scalingFactor = 1.0f;

    public GameObject FKTracker;


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // UpdateAllJointTargets();

        Matrix4x4 T = Matrix4x4.identity;
        // Matrix4x4 T01 = MakeDHParameter(Joint1Angle * Mathf.Deg2Rad, Mathf.PI/2.0f, l1, 0);
        // Matrix4x4 T12 = MakeDHParameter(Joint2Angle * Mathf.Deg2Rad, 0, 0, l2);
        // Matrix4x4 T23 = MakeDHParameter(Joint3Angle * Mathf.Deg2Rad, 0, 0, l3);
        // Matrix4x4 T34 = MakeDHParameter(Joint4Angle * Mathf.Deg2Rad, 0, 0, l4);
        // T *= T01 * T12 * T23 * T34;

        // T *= TXYZ(-0.07f, 1.35f, 0.1f);
        // T *= RY((-Joint1Angle - 20.0f) * Mathf.Deg2Rad);
        // T *= RX((-Joint2Angle + 30.0f) * Mathf.Deg2Rad);
        // T *= TXYZ(0, 0, -0.8f);
        // T *= RX((-Joint3Angle - 30.0f) * Mathf.Deg2Rad);
        // T *= RX((-Joint4Angle - 30.0f) * Mathf.Deg2Rad);
        // T *= TXYZ(0, -0.05f, -0.45f);

        T *= TXYZ(0, 0, 0.333f);
        T *= RY((-Joint1Angle) * Mathf.Deg2Rad);
        T *= TXYZ(-0.08f,0.525f,0.1f - 0.333f);
        T *= RX((-Joint2Angle) * Mathf.Deg2Rad);
        T *= TXYZ(0, 1.36f - 0.525f, 0f);
        T *= RX((-Joint3Angle) * Mathf.Deg2Rad);

        var xyz = new Vector3(-0.0799999982f,0.912f,-0.540000021f) - new Vector3(-0.0799999982f,1.36000001f,0.100000001f); // offset = new - old
        T *= TXYZ(xyz.x, xyz.y, xyz.z);
        T *= RX((-Joint4Angle) * Mathf.Deg2Rad);
        xyz = new Vector3(-0.0799999982f,0.894999981f,-1.25800002f) - new Vector3(-0.0799999982f,0.912f,-0.540000081f);
        T *= TXYZ(xyz.x, xyz.y, xyz.z);

        FKTracker.transform.position = ExtractPosition(T);
        FKTracker.transform.rotation = T.rotation;

        UpdateAllJointTargets();
    }

    void SetArticulationBodyTarget(ArticulationBody a, float degrees)
    {
        var drive = a.xDrive;
        drive.target = degrees;
        a.xDrive = drive;
    }

    void UpdateAllJointTargets()
    {
        SetArticulationBodyTarget(Joints[0], Joint1Angle);
        SetArticulationBodyTarget(Joints[1], Joint2Angle);
        SetArticulationBodyTarget(Joints[2], Joint3Angle);
        SetArticulationBodyTarget(Joints[3], Joint4Angle);
        SetArticulationBodyTarget(Joints[4], GripperAngle);
        SetArticulationBodyTarget(Joints[5], -GripperAngle);
    }

    Matrix4x4 MakeDHParameter(float theta, float alpha, float d, float a)
    {
        return new Matrix4x4(
            new Vector4(cos(theta), sin(theta), 0, 0),
            new Vector4(-sin(theta)*cos(alpha), cos(theta)*cos(alpha), sin(alpha), 0),
            new Vector4(sin(theta)*sin(alpha), -cos(theta)*sin(alpha),cos(alpha)),
            new Vector4(a*cos(theta), a*sin(theta),d,1)
        );
    }

    Vector3 ExtractPosition(Matrix4x4 T)
    {
        float x = T.m03;
        float y = T.m13;
        float z = T.m23;

        return new Vector3(x, y, z);
    }

    float cos(float x) => Mathf.Cos(x);

    float sin(float x) => Mathf.Sin(x);

    Matrix4x4 RX(float theta)
    {
        return new Matrix4x4(
            new Vector4(1,0,0,0),
            new Vector4(0,cos(theta),-sin(theta),0),
            new Vector4(0,sin(theta),cos(theta),0),
            new Vector4(0, 0, 0, 1)
        );
    }

    Matrix4x4 RY(float theta)
    {
        return new Matrix4x4
        (
            new Vector4(cos(theta),0,-sin(theta),0),
            new Vector4(0,1,0,0),
            new Vector4(sin(theta),0,cos(theta),0),
            new Vector4(0, 0, 0, 1)
        );
    }

    Matrix4x4 RZ(float theta)
    {
        return new Matrix4x4
        (
            new Vector4(cos(theta),-sin(theta),0,0),
            new Vector4(-sin(theta),cos(theta),0,0),
            new Vector4(0,0,1,0),
            new Vector4(0, 0, 0, 1)
        );
    }

    Matrix4x4 TXYZ(float x, float y, float z)
    {
        return new Matrix4x4(
            new Vector4(1, 0, 0, 0),
            new Vector4(0, 1, 0, 0),
            new Vector4(0, 0, 1, 0),
            new Vector4(x, y, z, 1)
        );
    }
}
