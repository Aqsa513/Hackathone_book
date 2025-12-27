---
sidebar_position: 17
title: Unity High-Fidelity Rendering Exercise
---

# Unity for High-Fidelity Rendering - Hands-on Exercise

## Exercise: Creating a High-Fidelity Robot Visualization in Unity

In this exercise, you'll create a Unity scene with high-fidelity robot visualization and basic human-robot interaction.

### Prerequisites

- Unity 2021.3 LTS or newer installed
- Basic understanding of Unity interface and C# scripting
- (Optional) Unity Robotics Hub package

### Task 1: Setting Up the Unity Scene

1. Create a new 3D project in Unity
2. Create a basic environment:
   - Add a plane as the ground
   - Add a directional light to simulate sunlight
   - Add an environment package (or create basic objects)

### Task 2: Creating a Robot Model

Create a simple robot model using primitive shapes:

```csharp
// RobotController.cs
using UnityEngine;

public class RobotController : MonoBehaviour
{
    public float moveSpeed = 5f;
    public float turnSpeed = 100f;
    public Transform[] wheels; // Array to hold wheel transforms

    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
            rb.mass = 5f;
        }
    }

    void Update()
    {
        // Get input
        float moveInput = Input.GetAxis("Vertical");
        float turnInput = Input.GetAxis("Horizontal");

        // Calculate movement direction
        Vector3 moveDirection = transform.forward * moveInput;

        // Move the robot
        transform.Translate(moveDirection * moveSpeed * Time.deltaTime, Space.World);

        // Rotate the robot
        transform.Rotate(Vector3.up, turnInput * turnSpeed * Time.deltaTime);

        // Rotate wheels visually
        if (wheels != null)
        {
            foreach (Transform wheel in wheels)
            {
                wheel.Rotate(Vector3.right, moveInput * moveSpeed * 60 * Time.deltaTime);
            }
        }
    }
}
```

### Task 3: Building the Robot in Unity Editor

1. Create an empty GameObject named "Robot"
2. Add the RobotController script to it
3. Add a Capsule as the main body:
   - Position: (0, 0.5, 0)
   - Scale: (0.5, 0.5, 0.5)
4. Add a Cube as the head:
   - Position: (0, 1.2, 0.2)
   - Scale: (0.3, 0.3, 0.3)
5. Add 4 Cylinders as wheels:
   - Position: (0.3, 0.2, 0.3), (0.3, 0.2, -0.3), (-0.3, 0.2, 0.3), (-0.3, 0.2, -0.3)
   - Scale: (0.2, 0.1, 0.2)
   - Rotation: (0, 0, 0)

### Task 4: Setting Up Materials and Lighting

Create realistic materials for your robot:

```csharp
// MaterialChanger.cs
using UnityEngine;

public class MaterialChanger : MonoBehaviour
{
    public Material robotBodyMaterial;
    public Material wheelMaterial;
    public Material headMaterial;

    void Start()
    {
        // Find robot parts and assign materials
        Transform body = transform.Find("RobotBody");
        Transform head = transform.Find("RobotHead");
        Transform[] wheels = new Transform[4];

        for (int i = 0; i < transform.childCount; i++)
        {
            Transform child = transform.GetChild(i);
            if (child.name.Contains("Wheel"))
            {
                if (child.GetComponent<Renderer>() != null)
                {
                    child.GetComponent<Renderer>().material = wheelMaterial;
                }
            }
            else if (child.name.Contains("Head"))
            {
                if (child.GetComponent<Renderer>() != null)
                {
                    child.GetComponent<Renderer>().material = headMaterial;
                }
            }
            else if (child.name.Contains("Body"))
            {
                if (child.GetComponent<Renderer>() != null)
                {
                    child.GetComponent<Renderer>().material = robotBodyMaterial;
                }
            }
        }
    }
}
```

### Task 5: Creating Lighting Setup

Set up realistic lighting in your scene:

1. Select the Directional Light
2. Set the following properties:
   - Intensity: 1.0
   - Color: White (RGB: 255, 255, 255)
   - Rotation: (50, -120, 0) to simulate sun
3. Add additional lighting:
   - Create a Point Light near the robot as a headlight
   - Add a Reflection Probe for realistic reflections

### Task 6: Adding Human-Robot Interaction

Create a simple UI for robot control:

```csharp
// RobotUIController.cs
using UnityEngine;
using UnityEngine.UI;

public class RobotUIController : MonoBehaviour
{
    public RobotController robot;
    public Slider speedSlider;
    public Text speedDisplay;
    public Button forwardButton;
    public Button backwardButton;
    public Button leftButton;
    public Button rightButton;

    void Start()
    {
        if (speedSlider != null)
        {
            speedSlider.onValueChanged.AddListener(OnSpeedChanged);
        }

        if (forwardButton != null)
        {
            forwardButton.onClick.AddListener(() => MoveRobot(Vector3.forward));
        }
        if (backwardButton != null)
        {
            backwardButton.onClick.AddListener(() => MoveRobot(Vector3.back));
        }
        if (leftButton != null)
        {
            leftButton.onClick.AddListener(() => RotateRobot(-1));
        }
        if (rightButton != null)
        {
            rightButton.onClick.AddListener(() => RotateRobot(1));
        }
    }

    void OnSpeedChanged(float value)
    {
        if (robot != null)
        {
            robot.moveSpeed = value;
        }
        if (speedDisplay != null)
        {
            speedDisplay.text = "Speed: " + value.ToString("F1");
        }
    }

    void MoveRobot(Vector3 direction)
    {
        if (robot != null)
        {
            robot.transform.Translate(direction * robot.moveSpeed * Time.deltaTime, Space.Self);
        }
    }

    void RotateRobot(float direction)
    {
        if (robot != null)
        {
            robot.transform.Rotate(Vector3.up, direction * robot.turnSpeed * Time.deltaTime);
        }
    }
}
```

### Task 7: Creating Post-Processing Effects

Add high-fidelity visual effects:

1. Go to Window → Package Manager
2. Install "Post Processing" package
3. Create a Post-process Volume in your scene
4. Add effects like:
   - Bloom: For realistic light scattering
   - Color Grading: For enhanced visual appearance
   - Ambient Occlusion: For realistic shadows in corners

### Task 8: Setting Up Camera System

Create multiple camera views:

```csharp
// CameraController.cs
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public Transform target; // Robot to follow
    public float distance = 10f;
    public float height = 5f;
    public float smoothSpeed = 12f;
    public Vector3 offset = new Vector3(0, 2, -5);

    void LateUpdate()
    {
        if (target != null)
        {
            Vector3 desiredPosition = target.position + offset;
            Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed * Time.deltaTime);
            transform.position = smoothedPosition;

            transform.LookAt(target);
        }
    }
}
```

### Task 9: Testing the Simulation

1. Run the scene in Unity
2. Test robot movement using keyboard controls (WASD)
3. Verify that wheels rotate as the robot moves
4. Test the UI controls
5. Observe the lighting and material effects

### Exercise Questions

1. How does the RobotController script handle movement differently from physics-based movement?
2. What is the purpose of the Reflection Probe in the scene?
3. How do post-processing effects enhance the visual quality?
4. Why might you want multiple camera views in a robot simulation?
5. How would you add realistic sensor visualization to your robot?

### Advanced Task

Enhance your robot simulation by:
1. Adding realistic materials with metallic and smoothness maps
2. Implementing a more complex robot model using imported 3D assets
3. Adding a first-person camera mode for immersive interaction
4. Creating a more sophisticated UI with robot status indicators
5. Adding particle effects for environmental interactions (dust, etc.)

### Summary

This exercise demonstrates fundamental concepts of high-fidelity rendering in Unity:
- Creating 3D objects and hierarchies
- Implementing basic robot control systems
- Setting up realistic materials and lighting
- Creating human-robot interaction interfaces
- Adding post-processing effects for enhanced visuals