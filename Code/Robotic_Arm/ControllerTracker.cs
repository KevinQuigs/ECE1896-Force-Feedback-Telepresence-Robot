using UnityEngine;
using UnityEngine.XR;
using System.Net.Sockets;
using System.Text;
using System;
using UnityEngine.UI;

public class ControllerTracker : MonoBehaviour
{   
    TcpClient client;
    NetworkStream stream;
    private InputDevice rightController;
    
    private WebCamTexture webcamTexture;

    void Start()
    {
        client = new TcpClient("127.0.0.1", 5001);
        stream = client.GetStream();

        webcamTexture = new WebCamTexture("OBS Virtual Camera");
        
        // Create Canvas for VR
        GameObject canvasObj = new GameObject("DesktopCanvas");
        Canvas canvas = canvasObj.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        
        // Position and scale canvas
        canvasObj.transform.position = new Vector3(0, 0, 2f);
        canvasObj.transform.localScale = new Vector3(0.01f, 0.01f, 0.01f);
        
        // Set canvas size
        RectTransform canvasRect = canvasObj.GetComponent<RectTransform>();
        canvasRect.sizeDelta = new Vector2(1920, 1080);
        
        // Create RawImage to display webcam
        GameObject imageObj = new GameObject("DesktopImage");
        imageObj.transform.SetParent(canvasObj.transform, false);
        
        RawImage rawImage = imageObj.AddComponent<RawImage>();
        rawImage.texture = webcamTexture;
        
        RectTransform imageRect = imageObj.GetComponent<RectTransform>();
        imageRect.anchorMin = new Vector2(0, 0);
        imageRect.anchorMax = new Vector2(1, 1);
        imageRect.sizeDelta = new Vector2(0, 0);
        imageRect.anchoredPosition = new Vector2(0, 0);
        
        webcamTexture.Play();
        
        Debug.Log("Available cameras:");
        foreach(var device in WebCamTexture.devices)
            Debug.Log(device.name);
    }

    void Update()
    {
        if (!rightController.isValid)
            rightController = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);

        TrackController(rightController, "Right");
    }

    private void TrackController(InputDevice controller, string name)
    {
        float MP = 0.0f;
        float MY = 0.0f;
        float MR = 0.0f;
        float HX = 0.0f;
        float HY = 0.0f;
        float HZ = 0.0f;
        float KP = 0.0f;
        float KY = 0.0f;
        float KR = 0.0f;

        if (controller.isValid)
        {
            bool hasPos = controller.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 handPos);
            bool hasRot = controller.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion handRot);

            if (hasPos && hasRot)
            {
                Vector3 handEuler = handRot.eulerAngles;
                MP = (float)Math.Round(handEuler.x, 3);
                MY = (float)Math.Round(handEuler.y, 3);
                MR = (float)Math.Round(handEuler.z, 3);

                HX = (float)Math.Round(handPos.x, 3);
                HY = (float)Math.Round(handPos.y, 3);
                HZ = (float)Math.Round(handPos.z, 3);
            }
        }

        InputDevice head = InputDevices.GetDeviceAtXRNode(XRNode.Head);
        if (head.isValid && head.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion headRot))
        {
            Vector3 headEuler = headRot.eulerAngles;
            KP = (float)Math.Round(headEuler.x, 3);
            KY = (float)Math.Round(headEuler.y, 3);
            KR = (float)Math.Round(headEuler.z, 3);
        }

        string outgoingMsg =
            $"{MP:0.000},{MY:0.000},{MR:0.000}," +
            $"{HX:0.000},{HY:0.000},{HZ:0.000}," +
            $"{KP:0.000},{KY:0.000},{KR:0.000}\n";
        
        byte[] bytes = Encoding.ASCII.GetBytes(outgoingMsg);
        stream.Write(bytes, 0, bytes.Length);
        Debug.Log($"Outgoing: {outgoingMsg}");
    }

    void OnDestroy()
    {
        if (webcamTexture != null)
            webcamTexture.Stop();
        stream?.Close();
        client?.Close();
    }
}