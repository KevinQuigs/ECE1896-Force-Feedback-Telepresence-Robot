using UnityEngine;
using UnityEngine.XR;
using System.IO.Ports;
using System.Threading;

public class ControllerTracker : MonoBehaviour
{   
    Thread IOThread = new Thread(DataThread);
    private static SerialPort sp;
    private static string incomingMsg = "";
    private static string outgoingMsg = "";





    private InputDevice leftController;
    private InputDevice rightController;


    private static void DataThread()
    {
        sp = new SerialPort("COM4", 9600);
        sp.Open();

        while(true) {
            if (outgoingMsg != "")
            {
                sp.Write(outgoingMsg);
                outgoingMsg = ""; // otherwise will keep sending data
            }

            incomingMsg = sp.ReadExisting();

            Thread.Sleep(200);
        } 
    }

    private void OnDestroy()
    {
        IOThread.Abort();
        sp.Close();
    }

    void Start()
    {
        IOThread.Start();
    }


    // Called very frame in Unity
    void Update()
    {
        // if (incomingMsg != "")
        // {
        //     Debug.Log($"Incoming {incomingMsg}");
        // }

        // Try to acquire the controllers if they aren't valid yet
        if (!leftController.isValid)
        {
            leftController = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        }
        if (!rightController.isValid)
        {
            rightController = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        }

        // Only print if the device is valid
        // TrackController(leftController, "Left");
        TrackController(rightController, "Right");
    }

    private void TrackController(InputDevice controller, string name)
    {
        if (!controller.isValid) return; // skip if not ready

        if (controller.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position) &&
            controller.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation))
        {
            Debug.Log($"{name} Controller - Position: {position}, Rotation: {rotation.eulerAngles}");
            
            if (position.x > 0.0)
                outgoingMsg = "1\n";
            else
                outgoingMsg = "0\n";

            Debug.Log($"Outgoing: {outgoingMsg}");
        }
        else
        {
            Debug.Log($"{name} Controller detected but position/rotation not ready yet");
        }
    }
}
