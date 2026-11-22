# Unitree G1 SDK and MuJoCo simulation

The original demo needed to exercise the G1 arm interface, `rt/arm_sdk`, with live joint feedback in Docker. The official Unitree MuJoCo bridge accepts motor commands on `rt/lowcmd`. This project uses Unitree's G1 model, MuJoCo physics, and the Unitree Python SDK, with a custom runner that accepts the arm topic directly.

## Command and feedback paths

| | Official Unitree MuJoCo | This project simulation |
|---|---|---|
| Command input | Receives SDK `LowCmd_` on `rt/lowcmd` and applies commands to the loaded model's actuators. | Receives the same G1 message type on `rt/arm_sdk`. It reads the 14 arm targets and the arm-authority weight. |
| Physics | The official bridge computes actuator inputs and the official runner steps MuJoCo. | `g1_control/simulator.py` blends commanded arm torques with a local hold, writes directly to MuJoCo's `data.ctrl`, and calls `mj_step()`. No message is published to `rt/lowcmd`. |
| Feedback | Publishes `rt/lowstate`. The documented bridge interfaces also include base position and velocity and, for the G1 C++ bridge, torso IMU. | Publishes positions and velocities for all 29 modeled joints on `rt/lowstate`. The web app uses those angles for its 3D view. |
| Scene and config | Uses the upstream scene and runner configuration. Its G1 scene adds a floor and visual settings around the robot model. | Loads the pinned `g1_29dof.xml` model directly, adds a pelvis weld, and holds the legs and waist near their initial pose. The upstream config, terrain, and joystick integration are outside this run path. |



## Why the project needed a custom runner

### Supported Unitree sdk2 Messages:

> **Current version only supports low-level development, mainly used for sim to real verification of controller**
>
> - `LowCmd`: Motor control commands
> - `LowState`: Motor state information
> - `SportModeState`: Robot position and velocity data
> - `IMUState`: Torso IMU state at `rt/secondary_imu` topic (G1 only)



### Why the G1 arm interface needs `rt/arm_sdk`

For the G1, `LowCmd` names the DDS message type. The DDS topic tells the simulator which command stream to receive. The stock bridge subscribes to `rt/lowcmd` and applies motor commands to the modeled actuators. The other messages in the list provide feedback. This is enough for a controller that targets the G1's low-level motor interface. 

The web app controls the 14 arm joints on the 29-DoF G1. Unitree's G1 arm example sends the same G1 `LowCmd_` type on `rt/arm_sdk`, sets an arm-authority weight in a reserved motor slot, and reads joint feedback on `rt/lowstate`. It lowers the weight to release arm control. A command published to `rt/arm_sdk` needs a subscriber on that topic. Sending the same payload on `rt/lowcmd` would exercise the stock motor-control path and bypass the arm-authority behavior the project needs to test. [

### What the custom runner verifies

The custom runner receives `rt/arm_sdk` directly and uses the weight to blend arm commands with a local hold. It publishes measured joint positions and velocities on `rt/lowstate`, so the web app and SDK examples can send arm targets and observe the resulting motion through the same DDS topics. The pelvis weld and lower-body hold keep the G1 supported so the test can focus on the arms. 

When the sender releases arm authority or goes 250 ms without a fresh command, the runner captures and holds the measured arm pose. This gives the project a way to test command release and command loss in simulation. The physical G1's firmware handoff and timeout behavior still require hardware verification. 

## G1 SDK and simulator coverage

 Each SDK interface needs a matching topic or service and an implementation on the robot or simulator. On a physical G1, availability depends on the model and firmware. 

| Feature | G1 SDK or hardware interface | Official Unitree MuJoCo |
|---|---|---|
| **Low-level motor control** | `unitree_hg.LowCmd_` carries position, velocity, gains, and feedforward torque. Hardware control mode and joint layout matter. | Supports `rt/lowcmd` for actuators in the loaded model. To exercise the `rt/arm_sdk` authority blend or the `rt/user_lowcmd` handoff, use an implementation of the corresponding interface. |
| **Joint state, IMU, and base telemetry** | `LowState_` carries joint and onboard state. The G1 also exposes torso IMU and sport-state interfaces. | Publishes modeled motor feedback and available sensor data. The C++ bridge supplies `rt/secondary_imu` for the G1. The Python bridge uses the `unitree_go` sport-state type. Hardware-specific fields can be defaults or simulated values. |
| **Factory gestures and custom actions** | Python exposes preset execution and listing. C++ also exposes named custom action execution and stopping. | Executing factory action IDs requires the G1 firmware's `arm` service. See [G1 factory gestures](#g1-factory-gestures). |
| **High-level locomotion and mode changes** | G1 `LocoClient` provides velocity, standing, balance, and mode requests. The motion-switcher runs as a robot-side service. | The bridge supplies sport-state telemetry. Walking in MuJoCo requires a controller or policy that supplies commands to the modeled joints. Testing calls such as `LocoClient.Move()` or firmware mode changes also requires a compatible service. [ |
| **Audio and RGB LED** | G1 `AudioClient` calls the `voice` service for TTS, PCM playback, volume, and RGB LED control. Unitree's microphone example receives raw PCM over UDP multicast. | Testing these interfaces requires a separate voice service or microphone stream and a compatible audio or LED backend. |
| **Camera and video** | The Python `VideoClient.GetImageSample()` example targets Go2. G1 compatibility with that client remains unverified. Unitree's G1 teleoperation stack documents a separate image server. | The MuJoCo viewport renders the scene. Testing a camera client requires a compatible image server. Unitree Isaac Lab streams simulated camera frames through Teleimager over ZMQ or WebRTC. |
| **Hands and grippers** | SDK message types and Unitree's Dex3 example expose finger commands and feedback. Hand features depend on the fitted device. | Testing Dex3 commands requires a hand-equipped scene and a bridge that implements the corresponding DDS topics. Unitree Isaac Lab documents both. |

The web app's simulation mode uses the custom runner described above. Mock mode exchanges joint feedback through direct Python calls. The browser renders the robot from joint positions, and the optional noVNC viewer displays the MuJoCo scene. 

## G1 factory gestures

### Why factory action IDs cannot run in MuJoCo

`G1ArmActionClient.ExecuteAction(id)` sends an RPC request containing the action ID to the G1 firmware's `arm` service. The service looks up the preset motion and drives the arms through `rt/arm_sdk`. `GetActionList()` queries that same service for the actions available on the connected robot. The public Python client exposes execution and listing, with the motion definitions held by the service. 

The stock MuJoCo bridge receives motor commands on `rt/lowcmd`. This project's runner receives arm setpoints on `rt/arm_sdk`. Both depend on a caller supplying joint commands. For `ExecuteAction(id)` to work in simulation, a compatible `arm` service would have to receive the request, resolve the ID to the factory motion, and generate those setpoints. The current setup requires a compatible physical G1 to execute a factory action by ID. 

### Actions listed in the Python SDK

The Python client's `action_map` lists these IDs. The project-pinned SDK contains the same map. Use `GetActionList()` to check which actions a particular robot's firmware reports. [Unitree Python SDK source on GitHub](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/g1/arm/g1_arm_action_client.py)

| ID | Action |
|---:|---|
| 11 | Two-hand kiss |
| 12 | Left kiss |
| 13 | Right kiss |
| 15 | Hands up |
| 17 | Clap |
| 18 | High five |
| 19 | Hug |
| 20 | Heart |
| 21 | Right heart |
| 22 | Reject |
| 23 | Right hand up |
| 24 | X-ray |
| 25 | Face wave |
| 26 | High wave |
| 27 | Shake hand |

The map also lists **Release arm** (ID `99`), which releases an arm action. [C++ arm action documentation](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/arm/g1_arm_action_client.hpp)

### How the web app plays gestures

The web app uses its own keyframes. Selecting **Face wave**, for example, plans a trajectory from the local catalog and streams the resulting joint targets as `LowCmd_` messages on `rt/arm_sdk`. The catalog's factory ID is metadata. If a physical G1 is available, recording `rt/lowstate` during a factory action and replaying the measured joint positions as setpoints can approximate that particular run. 

For differences between the hardware SDKs, see the [C++ and Python SDK2 comparison](unitree_g1_sdk2_comparison.md).
