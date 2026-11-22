# Unitree SDK2 vs SDK2 Python — G1 feature comparison

**Initial finding: Both SDKs support the Unitree G1, including high-level locomotion, low-level joint control, and DDS communication. However, they are not completely feature-equivalent at the API and example level.**

The repositories are:

- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) — C++
- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) — Python

Both use Unitree's DDS communication architecture, and the Python repository explicitly states that its interfaces are intended to remain consistent with the C++ SDK.

The important distinction is between **robot capability**, **SDK implementation**, and **included examples**. A missing Python convenience wrapper does not necessarily mean the G1 feature is inaccessible from Python.

## 1. G1 feature matrix

*Assessment of the current default branches: C++ `main`, Python `master`. ✓ = direct support; ◐ = partial support or lower-level implementation required; — = no equivalent first-party interface found.*

| G1 capability | C++ SDK | Python SDK |
|---|---|---|
| DDS publish/subscribe | ✓ | ✓ |
| Low-level motor position, velocity, torque control | ✓ | ✓ |
| G1 joint states, IMU, motor feedback | ✓ | ✓ |
| High-level walking / velocity commands | ✓ | ✓ |
| Stand, damp, stop, squat and balance operations | ✓ | ✓, different wrappers |
| 5-DOF and 7-DOF arm control through `rt/arm_sdk` | ✓ | ✓ |
| Motion switching / release of motion service | ✓ | ✓ |
| Full set of G1 locomotion query methods | ✓ | ◐ |
| Locomotion mimic-motion discovery | ✓ | — |
| Preset arm actions | ✓ | ✓, payload differs |
| Custom named arm actions and stopping them | ✓ | — |
| Audio TTS, PCM playback, volume, RGB LED | ✓ | ✓ |
| Dex3 dexterous hands | ✓, example | ◐, DDS types available |
| `rt/hand_sdk` control | ✓, example | ◐, generic DDS |
| `rt/user_lowcmd` whole-body user control | ✓, example | ◐, generic DDS |
| G1-D wheeled base and height adjustment | ✓ | —, no dedicated client found |
| G1-D waist-pitch control | ✓, example | ◐, DDS types available |
| Built-in G1 orientation safety helper | ✓ | — |

The low-level foundation is closely aligned: Python's `unitree_hg` message definitions include 35 motor slots in `LowCmd_`/`LowState_`, with the standard 29-joint G1 examples using the first 29. They also include `HandCmd_`, `HandState_`, and motor command messages.

**The main advantage of C++ is broader first-party coverage**, particularly newer G1 controls, custom actions, G1-D interfaces, and ready-made examples. Python can reproduce many of the topic-level capabilities, but some functionality needs additional code.

## 2. Confirmed API differences

### A. G1 locomotion

The C++ `g1::LocoClient` exposes functions missing from Python's `LocoClient`.

| Function | C++ | Python |
|---|---|---|
| `GetFsmId()` | ✓ | ✓ |
| `GetFsmMode()` | ✓ | — |
| `GetBalanceMode()` | ✓ | — |
| `GetSwingHeight()` | ✓ | — |
| `GetStandHeight()` | ✓ | — |
| `GetPhase()` (deprecated) | ✓ | — |
| `SetSwingHeight()` | ✓ | — |
| `GetMimicMotion()` | ✓ | — |
| `ContinuousGait()` | ✓ | Via `SetBalanceMode()` |
| `SwitchMoveMode()` | ✓ | Via `Move(..., continous_move=...)` |

There is an important nuance: Python registers several of the underlying RPC IDs in `Init()`, despite not providing callable public methods for them. Consequently, many of these are **missing convenience wrappers, not necessarily unsupported robot features**. The newer C++ `GetMimicMotion()` functionality also uses API ID `7100`, which the Python G1 client does not register.

The standing and squatting convenience methods also differ. C++ uses `Squat()` with FSM 2 and `StandUp()` with FSM 4; Python exposes `Squat2StandUp()` and `StandUp2Squat()`, both currently calling FSM 706. These should not be assumed interchangeable without checking G1 firmware behavior.

### B. Arm actions

C++ includes additional support for executing named custom actions and stopping them. Python exposes only numeric `ExecuteAction()` and `GetActionList()`.

More importantly, there is a concrete serialization discrepancy:

**C++ payload**

```json
{"action_id": 27}
```

**Python payload**

```json
{"data": 27}
```

Both call service `arm`, API `7106`, and advertise version `1.0.0.14`, but use different JSON keys. This is a genuine compatibility concern; without testing against the G1 firmware, it is not possible to say whether both payload formats are accepted.

### C. Audio

Both SDKs provide TTS, stream playback, stopping playback, volume control and RGB LED control. However, Python currently contains:

```python
self.tts_index += self.tts_index
```

Because `tts_index` starts at zero, this never increments it. C++ correctly uses `tts_index++`. This is a Python implementation defect worth fixing before using repeated TTS requests.

## 3. Dexterous hands and whole-body control

### Dex3 hands

The C++ repository includes a complete [G1 Dex3 example](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/dex3/g1_dex3_example.cpp) demonstrating joint position commands, state feedback, and finger control.

The Python repository includes the necessary DDS types (`HandCmd_`, `HandState_`), but I did not find a comparable standalone Dex3 example in its G1 examples.

The important DDS topics are:

```text
rt/dex3/left/cmd
rt/dex3/left/state
rt/dex3/right/cmd
rt/dex3/right/state
```

This means **Dex3 control is possible using Python** without introducing the C++ SDK. Unitree's separate `unitree_lerobot` project already demonstrates controlling Dex3 hands through the Python SDK.

### Arm SDK and whole-body user control

Both repositories demonstrate arm commands through `rt/arm_sdk`.

C++ additionally has a dedicated example for `rt/user_lowcmd`, which allows user-defined whole-body control after switching to the corresponding control mode.

Python already provides `SwitchToUserCtrl()`, so this is largely an example-level gap rather than a fundamental inability to access the control topic.

### G1-D is a more substantial difference

If your target is the wheeled G1-D rather than the standard walking G1, C++ has a clearer advantage.

It includes a dedicated `AgvClient` supporting base movement and height-column adjustment, as well as G1-D examples for waist pitch, arm control and height feedback. The Python repository does not currently provide equivalent G1-D client wrappers.




