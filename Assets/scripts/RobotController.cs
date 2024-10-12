using UnityEngine;

public class RobotController : MonoBehaviour
{
    public ArticulationBody[] joints; // Array of ArticulationBody components

    // Update is called once per frame
    void Update()
    {
        ControlJoints();
    }

    private void ControlJoints()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            float input = Input.GetAxis($"Joint{i + 1}"); // Use input mapping for each joint
            UpdateJointAngle(joints[i], input);
        }
    }

    private void UpdateJointAngle(ArticulationBody joint, float input)
    {
        var drive = joint.xDrive; // Get the drive settings for the joint

        // Calculate new angle based on input
        float newAngle = drive.target + input * Time.deltaTime;

        // Clamp the new angle between lower and upper limits defined in the joint's drive
        newAngle = Mathf.Clamp(newAngle, drive.lowerLimit, drive.upperLimit);

        drive.target = newAngle; // Update the drive target
        joint.xDrive = drive; // Apply the updated drive settings
    }
}
