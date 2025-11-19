using UnityEngine;
using UnityEngine.XR;
using System.Net.Sockets;
using System.Text;
using System;
using static System.Math;


public class ControllerTracker : MonoBehaviour
{   
    TcpClient client;
    NetworkStream stream;

    private InputDevice rightController;

    void Start()
    {
        // Connect to TCP server
        client = new TcpClient("127.0.0.1", 5001); // Connects Local Host (Local Machine)
        stream = client.GetStream();
    }


    // Called very frame in Unity
    void Update()
    {

        // Try to acquire the controllers if they aren't valid yet
        if (!rightController.isValid)
            rightController = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);

        // Only print if the device is valid
        TrackController(rightController, "Right");
    }

    private void TrackController(InputDevice controller, string name)
    {
        if (!controller.isValid) return; // skip if not ready

        bool hasPos = controller.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 handPos);
        bool hasRot = controller.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion handRot);

        if (hasPos && hasRot)
        {
            // Controller angles
            Vector3 handEuler = handRot.eulerAngles;
            float MP = (float)Math.Round(handEuler.x, 1);
            float MY = (float)Math.Round(handEuler.y, 1);
            float MR = (float)Math.Round(handEuler.z, 1);

            // Controller position
            float HX = (float)Math.Round(handPos.x, 1);
            float HY = (float)Math.Round(handPos.y, 1);
            float HZ = (float)Math.Round(handPos.z, 1);

            // Headset rotation
            InputDevice head = InputDevices.GetDeviceAtXRNode(XRNode.Head);
            head.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion headRot);
            Vector3 headEuler = headRot.eulerAngles;

            float KP = (float)Math.Round(headEuler.x, 1);
            float KY = (float)Math.Round(headEuler.y, 1);
            float KR = (float)Math.Round(headEuler.z, 1);

            // Final String Format
            string outgoingMsg =
                $"MP{MP}MY{MY}MR{MR}" +
                $"HX{HX}HY{HY}HZ{HZ}" +
                $"KP{KP}KY{KY}KR{KR}\n";
            
            byte[] bytes = Encoding.ASCII.GetBytes(outgoingMsg); // Convert to bytes
            stream.Write(bytes, 0, bytes.Length); // Send Message

            // Debug.Log($"{name} Controller - Position: {position}, Rotation: {rotation.eulerAngles}");
            Debug.Log($"Outgoing: {outgoingMsg}");
        }
        else
        {
            Debug.Log($"{name} Controller detected but position/rotation not ready yet");
        }
    }
}
