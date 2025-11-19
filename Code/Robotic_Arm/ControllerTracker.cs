using UnityEngine;
using UnityEngine.XR;
using System.Net.Sockets;
using System.Text;

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

        if (controller.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position) &&
            controller.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation))
        {
            Debug.Log($"{name} Controller - Position: {position}, Rotation: {rotation.eulerAngles}");
            
            //Send over controller tracking info
            string outgoingMsg = $"Hand:{position}\n";
            byte[] bytes = Encoding.ASCII.GetBytes(outgoingMsg); // Convert to bytes
            stream.Write(bytes, 0, bytes.Length); // Send Message

            Debug.Log($"Outgoing: {outgoingMsg}");
        }
        else
        {
            Debug.Log($"{name} Controller detected but position/rotation not ready yet");
        }
    }
}
