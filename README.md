# JR3 Mbed firmware (UART interface)

An Arm Mbed OS 6 application that performs data acquisition from a JR3 force-torque sensor and streams it through a UART channel.

Refer to [roboticslab-uc3m/jr3-mbed-firmware](https://github.com/roboticslab-uc3m/jr3-mbed-firmware/) for the underlying board-sensor communication. This repository focuses on the serial (UART) interface layer with an external PC. Refer to [roboticslab-uc3m/jr3-mbed-firmware-can](https://github.com/roboticslab-uc3m/jr3-mbed-firmware-can/) for the CAN interface variant.

## Installation

Since the Mbed Online Compiler has been discontinued and the Keil web compiler will be shut down on July 2026, the preferred method is to use the [Mbed CE (Community Edition)](https://mbed-ce.dev/) CMake-based build system. Once built, plug in the Mbed to a USB port of your PC and drag-and-drop the downloaded .bin file into it.

## Communication Protocol

The communication protocol for this project is based on the UART structure, organized in frames. Each frame represents a message that follows a predefined structure.

### Frame Structure

Each message in this protocol follows the structure below:

1. **Start Character ('<')**: Marks the beginning of a new message.
2. **Operation Field ('OP')**: Specifies the task or command being executed. There are 10 possible operations. To maintain consistency, each command is represented by two bytes, with a leading '0' for single-digit commands.
3. **Data Field ('DATA')**: Contains the specific data required for each command. This field has a capacity of 14 bytes, but not all commands use the full capacity. Some commands do not require additional data, leaving this field empty.
4. **End Character ('>')**: Marks the end of the message. The receiver stops reading the message when it encounters this character.

### Example Frame Structure

Here is an example of the message structure in the protocol:

| Field               | Description                                |
|---------------------|--------------------------------------------|
| **Start Character**  | `<` (Start of message)                     |
| **Operation Field**  | Operation code indicating the task (2 bytes) |
| **Data Field**       | Optional command-specific data (up to 14 bytes) |
| **End Character**    | `>` (End of message)                       |

### Message Table

The following table outlines the different messages in the protocol:

| **Message Name**       | **Function**                             | **Op Code** | **Direction** | **Payload (Bytes)** | **Details** |
|------------------------|------------------------------------------|-------------|---------------|---------------------|-------------|
| `JR3_ACK`              | Acknowledge                              | `<01>`      | Outgoing      | 1                   | Status: 0x00 (ready), 0x01 (not initialized) |
| **`JR3_START`**        | Start (Asynchronous)                     | `<02>`      | Incoming      | 6                   | 2 LSB bytes: cutoff frequency [0.01\*Hz], 4 MSB bytes: read period [us] |
| **`JR3_STOP`**         | Stop                                     | `<03>`      | Incoming      | 0                   | -           |
| **`JR3_ZERO_OFFS`**    | Zero offsets                             | `<04>`      | Incoming      | 0                   | -           |
| **`JR3_SET_FILTER`**   | Set filter                               | `<05>`      | Incoming      | 2                   | Cutoff frequency [0.01\*Hz] |
| **`JR3_GET_STATE`**    | Get state                                | `<06>`      | Incoming      | 0                   | -           |
| **`JR3_GET_FS`**       | Get full scales (forces and moments)     | `<07>`      | Incoming      | 0                   | Acknowledge message contains force/moment range in 6 bytes (Fx, Fy, Fz, Mx, My, Mz) |
| **`JR3_RESET`**        | Reset                                    | `<08>`      | Incoming      | 0                   | -           |
| `JR3_READ`             | Read (force and moment data)             | `<09>`      | Outgoing      | 14                  | 6 bytes for forces/moments (Fx, Fy, Fz, Mx, My, Mz) and 2 bytes for frame counter |
| `JR3_BOOTUP`           | Bootup                                   | `<10>`      | Outgoing      | 0                   | Indicates that the sensor has started up |

*Messages in bold* automatically receive an acknowledgment (`JR3_ACK`) from the Mbed device.

### Message Descriptions

Below is a brief explanation of the operation code (opcode) for each message:

- **JR3_ACK**: Sent by the Mbed, informs the current state of the JR3 sensor:
  - `JR3_READY (0x00)`: The JR3 sensor is correctly connected.
  - `JR3_NOT_INITIALIZED (0x01)`: The JR3 sensor is not correctly connected or initialized.

- **JR3_START**: Initiates the asynchronous mode and begins the data acquisition process. The message includes the cutoff frequency and acquisition period for configuring the sensor:
  - Cutoff frequency: Specifies the frequency at which the sensor filter should attenuate signals.
  - Acquisition period: Defines the interval between readings sent by the JR3 sensor.

- **JR3_STOP**: Stops the force and moment data transmission.

- **JR3_ZERO_OFFS**: Calibrates the sensor offsets.

- **JR3_SET_FILTER**: Sets the cutoff frequency for the sensor filter.

- **JR3_GET_STATE**: Requests the current state of the JR3 sensor.

- **JR3_GET_FS**: Requests the constant force and moment data experienced by the sensor when no external forces are applied. The acknowledgment message includes this data in its payload.

- **JR3_RESET**: Reinitializes the JR3 sensor.

- **JR3_READ**: Sends the force and moment data registered by the JR3 sensor every specified acquisition period. The structure of the message is as follows:
  - 6 bytes for the force and moment data (Fx, Fy, Fz, Mx, My, Mz)
  - 2 bytes for the frame counter.

- **JR3_BOOTUP**: Sent by the Mbed when the JR3 sensor has started up, indicating it is ready to receive commands.

### Design Considerations

This protocol design was inspired by the [CAN protocol implementation](https://github.com/roboticslab-uc3m/jr3-mbed-firmware-can) from Bartosz Piotr Lukawski. Unlike CAN, which divides force and moment data into two separate commands due to byte limitations, BufferedSerial has 14 bytes available, allowing both force and moment data to be sent in a single message. This design simplifies communication and ensures coherence between force and moment readings.

The frame counter is retained from the CAN protocol and helps ensure consistency in force and moment readings. While not strictly necessary in BufferedSerial due to the larger byte capacity, the frame counter can still be useful for verifying communication integrity and performance analysis.

Additionally, the command related to gripper control (PMW) was removed, as it is not relevant to this project. The synchronous mode was also eliminated since synchronization between multiple devices (as used in the TEO humanoid) is unnecessary here. This led to the unification of the `JR3_START` command into a single, streamlined version, improving overall performance and reducing unnecessary functions.

## Configuration

See [mbed-app.json5](mbed_app.json5) for a list of configurable parameters and their description. The project should be recompiled after any changes to this file.

## Additional Tools

### `abb_motion_program_exec` Library
- Used for **Pose Target** mode (`EgmPoseTargetConfig`), sending position and orientation targets to the ABB robot.

### `abb_robot_client` Library
- Only the **`egm`** module is used to control the robot's movements through the Externally Guided Motion (EGM) interface.

### KDL Library
- Utilized the **Motion** module for trajectory planning, though it was partially re-implemented in Python due to incomplete `PyKDL`.

## Project Overview

This project aims to integrate a JR3 force-torque sensor with an ABB robot to modify its tool's trajectory along the Z-axis when encountering an object. The goal is to control the ABB robot externally using the **Externally Guided Motion (EGM)** feature, which allows real-time path correction.

The project is motivated by the need to modify the trajectories of GoFa cobots in the RoboticsLab research team from Universidad Carlos III de Madrid using an external controller. The ABB RobotStudio simulator is used to verify this functionality with a connected JR3 sensor and an Mbed LPC1768 microcontroller.

A Python-based ROS 2 driver has been developed to interface with the Mbed from an external source through serial communications. Refer to [roboticslab-uc3m/jr3-driver](https://github.com/roboticslab-uc3m/jr3-driver) for more details.

## Citation

If you found this project useful, please consider citing the following work:

Alba Olano Díaz, *Integración de un sensor fuerza-par en un robot industrial ABB*, Trabajo Fin de Grado, Universidad Carlos III de Madrid, 2024. https://github.com/albaod/jr3-mbed-firmware

```bibtex
@inproceedings{
    author={Olano Díaz, Alba},
    title={{Integración de un sensor fuerza-par en un robot industrial ABB. Universidad Carlos III de Madrid.}},
    year={2024},
    publisher={Universidad Carlos III de Madrid},
}
```

## See also

- [roboticslab-uc3m/yarp-devices#263](https://github.com/roboticslab-uc3m/yarp-devices/issues/263)
- [roboticslab-uc3m/jr3-mbed-firmware](https://github.com/roboticslab-uc3m/jr3-mbed-firmware)
- [roboticslab-uc3m/jr3-mbed-firmware-can](https://github.com/roboticslab-uc3m/jr3-mbed-firmware-can)
- [roboticslab-uc3m/jr3-driver](https://github.com/roboticslab-uc3m/jr3-driver)
- [roboticslab-uc3m/jr3pci-linux](https://github.com/roboticslab-uc3m/jr3pci-linux)
