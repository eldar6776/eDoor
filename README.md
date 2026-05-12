# eDoor

<p align="center">
  <img alt="Platform" src="https://img.shields.io/badge/platform-STM32F0-blue">
  <img alt="RFID" src="https://img.shields.io/badge/RFID-125%20kHz-success">
  <img alt="Protocol" src="https://img.shields.io/badge/bus-RS485-informational">
  <img alt="Desktop App" src="https://img.shields.io/badge/desktop-Windows%20Forms-5C2D91">
  <img alt="Framework" src="https://img.shields.io/badge/.NET-Framework%204.8-512BD4">
  <img alt="Hardware" src="https://img.shields.io/badge/hardware-Altium%20Designer-A5915F">
  <img alt="Firmware" src="https://img.shields.io/badge/firmware-Keil%20MDK--ARM-red">
  <img alt="Config" src="https://img.shields.io/badge/config-STM32CubeMX-03234B">
</p>

Operational access control system based on a **125 kHz RFID reader**, **STM32 firmware**, and an **RS485 control bus**. The repository contains production-oriented firmware, hardware design files, a Windows configuration utility, and supporting technical documentation used to develop and maintain the eDoor platform.

---

## Overview

`eDoor` is a field-deployed door access control platform designed for RFID-based identification, local decision making, event logging, and remote management over an RS485 network.

Based on the repository contents, the system includes:

- embedded firmware for an **STM32F0** microcontroller platform
- **125 kHz RFID** reader logic based on the **HTRC110 / EM4102** approach
- **EEPROM-backed** storage for cards, configuration, and event logs
- an **RS485 command protocol** for monitoring and control
- a **Windows desktop utility** for communication, testing, and administration
- complete **hardware design files** for PCB, schematic, and fabrication outputs
- supporting **datasheets, reference designs, and development documentation**

## Core Capabilities

- reading 125 kHz RFID cards/tags
- validating cards against locally stored data
- door lock control
- buzzer control
- real-time and stored event handling
- time configuration from the host application
- adding and removing RFID cards
- reading and deleting stored event logs
- restarting one controller or multiple controllers on the RS485 bus
- PC-based service and configuration workflow

## System in the Field

```text
                    +-----------------------------------+
                    |          Supervisory PC           |
                    |  Windows Forms Service Utility    |
                    +-----------------+-----------------+
                                      |
                                      | RS485 bus
                                      |
          ---------------------------------------------------------------
             |                         |                         |
             |                         |                         |
+------------v-----------+ +-----------v------------+ +----------v------------+
|   eDoor Controller     | |   eDoor Controller     | |   eDoor Controller    |
|   Door / Entry Point   | |   Door / Entry Point   | |   Door / Entry Point  |
+------------+-----------+ +-----------+------------+ +----------+------------+
             |                         |                         |
   +---------+---------+     +---------+---------+     +---------+---------+
   | 125 kHz RFID      |     | 125 kHz RFID      |     | 125 kHz RFID      |
   | reader front-end  |     | reader front-end  |     | reader front-end  |
   +-------------------+     +-------------------+     +-------------------+
             |                         |                         |
   +---------+---------+     +---------+---------+     +---------+---------+
   | EEPROM / logs /   |     | EEPROM / logs /   |     | EEPROM / logs /   |
   | local card store  |     | local card store  |     | local card store  |
   +-------------------+     +-------------------+     +-------------------+
             |                         |                         |
   +---------+---------+     +---------+---------+     +---------+---------+
   | Lock output /     |     | Lock output /     |     | Lock output /     |
   | buzzer / signaling|     | buzzer / signaling|     | buzzer / signaling|
   +-------------------+     +-------------------+     +-------------------+
```

## Repository Structure

```text
.
├── doc/   # Datasheets, reference design material, images, protocol notes
├── fw/    # Firmware projects, revisions, build outputs, and archives
├── hw/    # Altium hardware projects, PCB, schematic, BOM, fabrication files
├── sw/    # Desktop software for communication and device configuration
└── README.md
```

## Firmware

The `fw/` directory contains multiple project revisions, including:

- `DE-071124`
- `DE-100924`
- `DE-290624`
- `eDoor-main.zip`

The most complete and recent firmware structure is visible in `fw/DE-100924/`, which contains:

- `Core/` – main application logic
- `Drivers/` – STM32 HAL and CMSIS components
- `Common/` – shared support code
- `MDK-ARM/` – Keil project and build outputs
- `DE-100924.ioc` – STM32CubeMX configuration

From the code and project files, the firmware stack includes:

- **STM32F0** MCU family
- **Keil MDK-ARM** build environment
- **STM32CubeMX** project configuration
- **I2C EEPROM** storage for cards and logs
- dedicated modules such as:
  - `rfid.c/.h`
  - `eeprom.c/.h`
  - `logger.*`
  - `main.*`

### Firmware Functional Blocks

#### RFID Processing
The firmware contains an RFID state machine in `rfid.c` / `rfid.h` for initialization and tag processing. The implementation references the **HTRC110** operating model and is aligned with a 125 kHz RFID reader design.

#### EEPROM Storage
The `eeprom.c/.h` module manages external EEPROM access over I2C. The memory map defines dedicated areas for:

- system configuration
- door timing configuration
- RFID card list
- event log list

#### Event Handling
The firmware structure and command interface indicate support for:

- event counting
- retrieval of the last logged event
- deletion of the latest event
- full event memory clearing

This is consistent with a deployed access-control workflow where event history is retained locally on the controller.

## Hardware

The `hw/` directory contains multiple hardware revisions and manufacturing outputs. The latest visible revision includes files such as:

- `.SchDoc` – schematics
- `.PcbDoc` – PCB layout
- `.PrjPcb` – Altium project
- `.BomDoc` – bill of materials documentation
- `.pdf` and `_3D.pdf` – exported design views
- fabrication output archives

This indicates that the hardware has been developed in **Altium Designer** and maintained through multiple board revisions.

Based on the repository structure and documentation set, the hardware platform includes:

- STM32 microcontroller section
- 125 kHz RFID analog front-end / reader section
- RS485 communication interface
- lock control output stage
- buzzer / status signaling
- external EEPROM
- supporting power and protection circuitry

## Desktop Application

The `sw/rfid/` directory contains a **C# Windows Forms** application used to communicate with controllers over a serial/RS485 interface.

### Application Stack

- solution file: `sw/rfid/rfid.sln`
- main project: `sw/rfid/RubiconCtrlConf/`
- target framework: **.NET Framework 4.8**
- UI technology: **Windows Forms**
- communication: serial COM port with RS485-connected devices

### Main Functions

The application supports:

- selecting COM port and baud rate
- selecting controller and application addresses
- sending control and diagnostic commands
- displaying controller responses
- displaying real-time event notifications

Commands visible in the desktop application include:

- `GET_SYS_FLAG`
- `GET_CARD_CNT`
- `GET_CARD_PRESENT`
- `GET_EVENT_CNT`
- `GET_EVENT_LAST`
- `SET_SYS_TIME`
- `SET_SYS_RESTART`
- `SET_CARD_ONE`
- `SET_DOOR_OPEN`
- `SET_DOOR_TIME`
- `SET_DOOR_ENABLE`
- `SET_DOOR_DISABLE`
- `SET_BUZZER_ENABLE`
- `SET_BUZZER_DISABLE`
- `DELETE_CARD_ONE`
- `DELETE_CARD_ALL`
- `DELETE_EVENT_LAST`
- `DELETE_EVENT_ALL`
- `RESTART_ONE`
- `RESTART_ALL`

## RS485 Protocol

The repository includes a protocol note in `doc/rs485_protokol.txt`, while the Windows utility provides an implementation-oriented view of the packet format and command set.

### Packet Layout

The documented packet structure is centered around:

- `SOH` (`0x01`) as start-of-header
- receiver/controller address
- payload length
- command byte
- optional parameter bytes
- 16-bit checksum
- `EOT` (`0x04`) as end-of-transmission

### Documented Commands

The protocol documentation includes commands such as:

- `GET_SYS_FLAG`
- `GET_CARD_CNT`
- `GET_CARD_ALL`
- `GET_EVENT_CNT`
- `GET_EVENT_LAST`
- `SET_SYS_FLAG`
- `SET_SYS_TIME`
- `SET_SYS_RESTART`
- `SET_CARD_ONE`
- `DELETE_CARD_ONE`
- `DELETE_CARD_ALL`
- `DELETE_EVENT_LAST`
- `DELETE_EVENT_ALL`

### Commands Implemented in the PC Utility

The Windows application also shows an actively used command model with operations for:

- card presence lookup
- door opening
- door enable/disable
- buzzer enable/disable
- single/all controller restart
- event retrieval and event deletion workflow

The protocol appears to have evolved across firmware and software revisions, so practical deployment should pair the desktop tool with the intended firmware revision.

## Documentation Set

The `doc/` directory contains supporting engineering material, including references related to:

- **HTRC110**
- 125 kHz RFID design and antenna tuning
- RDM6300 reference material
- MOSFET and protection components
- power-related supporting devices
- photos and design notes
- RS485 protocol description

This documentation base is useful both for maintenance of the current platform and for future hardware/firmware revisions.

## Development Environment

### Firmware Tools

To work with the firmware, the repository is structured around:

- **Keil MDK-ARM**
- **STM32CubeMX**
- optionally **STM32CubeProgrammer** for device programming

Suggested entry points:

1. Open `fw/DE-100924/DE-100924.ioc` in STM32CubeMX to inspect peripheral configuration.
2. Open the Keil project inside `fw/DE-100924/MDK-ARM/` for build and debug.
3. Review `Core/` and `Common/` for the main application logic.

### Hardware Tools

To inspect and modify the hardware design, use:

- **Altium Designer**

Start with one of the hardware revisions under `hw/`, for example `hw/DE-150824/`.

### Desktop Tools

To build the service/configuration utility, use:

- **Visual Studio**
- **.NET Framework 4.8 Developer Pack**

Suggested steps:

1. Open `sw/rfid/rfid.sln`
2. Build the `RubiconCtrlConf` project
3. Connect to the correct COM port / RS485 interface
4. Set the desired baud rate and node addresses
5. Send and monitor controller commands

## Repository Contents at a Glance

This repository is maintained as a real engineering project and includes:

- multiple firmware revisions
- multiple hardware revisions
- vendor driver packages
- build outputs and archives
- desktop tooling
- board documentation and fabrication data
- protocol and component reference material

## Notes

This README is written to reflect the repository as an operational access-control platform rather than a demonstration project. Device photos and installation images can be added later to complement the technical documentation and field presentation of the system.
