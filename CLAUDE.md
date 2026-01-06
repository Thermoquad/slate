# Slate Project - AI Assistant Guide

> **Note:** This file documents the Slate project specifically.
> Always read the [Thermoquad Organization CLAUDE.md](../../CLAUDE.md) first
> for organization-wide structure and conventions.

## Project Overview

**Slate** is a multi-mode controller firmware for the Helios liquid fuel burner system. It acts as the master controller in the Helios serial protocol, providing user interface and system integration capabilities.

**Operating Modes:**

1. **Normal Mode:** Direct user interface with display and physical controls
   - OLED LCD display with LVGL UI framework
   - Quadrature encoder (rotary wheel) for navigation
   - Direct control of Helios burner via serial protocol
   - Can operate as portable wireless controller

2. **Node Mode:** Bridge/gateway for system integration
   - Serial bridge between normal mode Slate and Helios ICU
   - Bluetooth interface for smartphone control
   - CAN bus integration for larger systems
   - Network-based remote control interfaces

**Hardware:**
- **Development Board:** Raspberry Pi Pico 2W (RP2350A with WiFi/Bluetooth)
- **Production Target:** Custom board with RP2354A (planned)
- **Current Status:** Early prototype - input testing phase (quadrature encoder functional)

**Communication:**
- **Master Role:** Sends commands to Helios ICU (slave)
- **Protocol:** Helios serial protocol (via helios_serial module)
- **Transports:** UART, WiFi, Bluetooth, CAN bus (depending on mode)

---

## Project Status

**Current State:** 🟡 Greenfield / Early Prototype

**What's Working:**
- ✅ Quadrature encoder input (GPIO-based QDEC driver)
- ✅ Button input with short/long press detection
- ✅ Shell interface for debugging
- ✅ CAN bus hardware configured (MCP2515 bridge)
- ✅ WiFi/networking stack configured
- ✅ Build system and development workflow

**What's Implemented:**
- ✅ LVGL display integration with OLED LCD
- ✅ Helios telemetry display (state, temperature, RPM)
- ✅ Serial communication with Helios (via Fusain library)
- ✅ Packet validation to reject corrupted telemetry

**What's Planned:**
- 🔲 Helios command transmission (full master implementation)
- 🔲 UI framework (menu system, interactive controls)
- 🔲 Mode switching (normal vs node mode)
- 🔲 Full threading model (currently using display thread only)
- 🔲 Wireless communication (WiFi/Bluetooth)
- 🔲 CAN bus messaging
- 🔲 State management and persistence

**Development Hardware (Prototype):**
- ✅ OLED LCD display (SSD1306, I2C, working with LVGL)
- ✅ Quadrature encoder (working)
- ✅ Push button (working)
- ✅ UART1 serial connection to Helios (working, receiving telemetry)
- ✅ MCP2515 CAN bridge (configured, not used)

---

## Architecture

### Design Philosophy

Slate will adopt conventions from Helios where practical:
- Zephyr RTOS threading model
- Zbus message bus for inter-thread communication
- State machine for mode management
- Modular controller design
- Shell commands for testing and debugging

### Key Differences from Helios

| Aspect | Helios (ICU) | Slate (Controller) |
|--------|--------------|-------------------|
| **Role** | Slave (receives commands) | Master (sends commands) |
| **Serial Protocol** | Responds to commands, sends telemetry | Initiates commands, receives telemetry |
| **User Interface** | None (headless) | LVGL display + encoder/buttons |
| **Operating Modes** | Single mode (burner control) | Dual mode (normal/node) |
| **State Machine** | Burner lifecycle (IDLE, PREHEAT, HEATING, etc.) | UI/connection state, mode switching |
| **Real-Time Control** | Direct hardware control (motor, pump, glow) | High-level orchestration via commands |
| **Communication** | UART slave only | UART master + WiFi/BT/CAN |

### Planned Threading Model

**Pattern:** One thread per major subsystem (following Helios convention)

**Proposed Threads:**

| Thread | Purpose | Rate | Priority |
|--------|---------|------|----------|
| `display_thread` | LVGL display updates and rendering | 50ms | 4 |
| `input_thread` | Encoder/button event processing | Event-driven | 3 |
| `serial_master_thread` | Helios serial protocol (TX/RX) | 100ms | 2 |
| `ui_state_thread` | UI state machine and navigation | 10ms | 5 |
| `network_thread` | WiFi/Bluetooth communication | Variable | 6 |
| `can_thread` | CAN bus messaging (node mode) | Variable | 6 |

**Note:** Thread model is subject to change based on development needs.

### Communication: Zbus Message Bus

Following Helios pattern, use Zephyr Zbus for inter-thread messaging:

**Proposed Channel Pairs:**
- `ui_command_chan` / `ui_data_chan` - UI state and events
- `display_command_chan` / `display_data_chan` - Display control
- `helios_command_chan` / `helios_data_chan` - Burner commands and telemetry
- `network_command_chan` / `network_data_chan` - Network status and data
- `mode_command_chan` / `mode_data_chan` - Operating mode control

**Message Structures:** To be defined in `include/slate/zbus.h` (not yet created)

---

## Current Implementation

### File Structure

```
slate/
├── CLAUDE.md                    # This file
├── CMakeLists.txt              # Build configuration
├── prj.conf                    # Zephyr Kconfig
├── Taskfile.dist.yml           # Task runner commands
├── boards/
│   └── rpi_pico2_rp2350a_m33.overlay  # Board-specific device tree
├── src/
│   └── main.c                  # Minimal prototype (input testing)
└── build/                      # Build artifacts (gitignored)
```

**Note:** Directory structure will expand as development progresses. Expected additions:
- `include/slate/` - Header files
- `src/ui/` - UI components and LVGL integration
- `src/communications/` - Serial, WiFi, Bluetooth, CAN handlers
- `src/controllers/` - Thread entry points and logic
- `docs/` - Architecture diagrams and specifications

### Current Code (Prototype)

**File:** `src/main.c`

**Purpose:** Input hardware validation (quadrature encoder + button)

**Implementation:**
```c
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>

static void test_cb(struct input_event *evt, void *p) {
    if (evt->code == INPUT_REL_WHEEL) {
        printk("z event %d\n", evt->value);  // Encoder rotation
    }
    if (evt->code == INPUT_BTN_1 && evt->value == 1) {
        printk("short press %d\n", evt->value);  // Short press
    }
    if (evt->code == INPUT_BTN_2 && evt->value == 1) {
        printk("long press %d\n", evt->value);  // Long press
    }
}

INPUT_CALLBACK_DEFINE(NULL, test_cb, NULL);

int main(void) {
    printk("Hello World! %s\n", CONFIG_BOARD);
    k_sleep(K_FOREVER);  // Sleep indefinitely, events handled by callback
    return 0;
}
```

**Input Events:**
- `INPUT_REL_WHEEL` - Quadrature encoder rotation (value = step count/direction)
- `INPUT_BTN_1` - Short button press (< 420ms)
- `INPUT_BTN_2` - Long button press (≥ 420ms)

**Callback Pattern:** Current implementation uses Zephyr input callbacks. This will be replaced with a dedicated input thread when the UI framework is implemented.

---

## Hardware Configuration

### Device Tree (Board Overlay)

**File:** `boards/rpi_pico2_rp2350a_m33.overlay`

#### Quadrature Encoder (GPIO-based)

```devicetree
qdec {
    compatible = "gpio-qdec";
    status = "okay";
    gpios = <&pico_header 14 (GPIO_PULL_UP | GPIO_ACTIVE_HIGH)>,
            <&pico_header 15 (GPIO_PULL_UP | GPIO_ACTIVE_HIGH)>;
    steps-per-period = <4>;
    zephyr,axis = <INPUT_REL_WHEEL>;
    sample-time-us = <1000>;
    idle-timeout-ms = <100>;
};
```

- **Pins:** GPIO 14 (A), GPIO 15 (B)
- **Resolution:** 4 steps per detent
- **Sampling:** 1ms
- **Idle timeout:** 100ms

#### Button Input

```devicetree
buttons: buttons {
    compatible = "gpio-keys";
    debounce-interval-ms = <50>;
    qdec_btn: btn_pin {
        gpios = <&pico_header 13 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
        label = "wheel";
        zephyr,code = <INPUT_BTN_0>;
    };
};

qdec_btn {
    input = <&buttons>;
    compatible = "zephyr,input-longpress";
    input-codes = <INPUT_BTN_0>;
    short-codes = <INPUT_BTN_1>;
    long-codes = <INPUT_BTN_2>;
    long-delay-ms = <420>;
};
```

- **Pin:** GPIO 13 (active low, pulled up)
- **Debounce:** 50ms
- **Long press threshold:** 420ms
- **Events:** Short press (BTN_1), Long press (BTN_2)

#### CAN Bus (MCP2515 SPI Bridge)

```devicetree
&spi0 {
    status = "okay";
    cs-gpios = <&pico_header 20 GPIO_ACTIVE_LOW>;

    can0: can@0 {
        compatible = "microchip,mcp2515";
        spi-max-frequency = <1000000>;
        int-gpios = <&pico_header 21 GPIO_ACTIVE_LOW>;
        status = "okay";
        reg = <0x0>;
        osc-freq = <8000000>;
        max-bitrate = <500000>;
    };
};
```

- **Interface:** SPI0 at 1 MHz
- **CS Pin:** GPIO 20
- **Interrupt Pin:** GPIO 21
- **CAN Bitrate:** 500 kbps max
- **Oscillator:** 8 MHz

#### Pin Summary

| GPIO | Function | Direction | Pull | Notes |
|------|----------|-----------|------|-------|
| 13 | Button | Input | Pull-up | Encoder button (active low) |
| 14 | Encoder A | Input | Pull-up | Quadrature channel A |
| 15 | Encoder B | Input | Pull-up | Quadrature channel B |
| 20 | CAN CS | Output | - | MCP2515 chip select |
| 21 | CAN INT | Input | - | MCP2515 interrupt |
| TBD | UART1 TX | Output | - | Helios serial (planned) |
| TBD | UART1 RX | Input | - | Helios serial (planned) |
| TBD | I2C SDA | Bidir | Pull-up | OLED display (planned) |
| TBD | I2C SCL | Output | Pull-up | OLED display (planned) |

---

## Zephyr Configuration (prj.conf)

### Core Kernel

```kconfig
CONFIG_EARLY_CONSOLE=y
CONFIG_INIT_STACKS=y
CONFIG_STACK_SENTINEL=y
CONFIG_MAIN_STACK_SIZE=5200
CONFIG_SHELL_STACK_SIZE=5200
```

### Input Subsystem

```kconfig
CONFIG_INPUT=y
CONFIG_INPUT_GPIO_QDEC=y
```

- GPIO-based quadrature decoder driver
- Input event framework at priority 90

### Networking Stack

```kconfig
CONFIG_NETWORKING=y
CONFIG_NET_IPV4=y
CONFIG_NET_IPV6=n
CONFIG_NET_TCP=y
CONFIG_NET_DHCPV4=y
CONFIG_WIFI=y
CONFIG_NET_SHELL=y
CONFIG_NET_STATISTICS=y
```

- Full IPv4 TCP/IP stack
- DHCP client for IP assignment
- WiFi with 802.11 management (Pico 2W)
- Network shell for debugging

### CAN Bus

```kconfig
CONFIG_CAN=y
CONFIG_CAN_SHELL=y
CONFIG_CAN_STATS=y
CONFIG_CAN_MAX_FILTER=5
```

- CAN driver (MCP2515)
- CAN shell commands
- Statistics tracking
- Max 5 message filters

### Shell & Debugging

```kconfig
CONFIG_SHELL=y
CONFIG_DEVICE_SHELL=y
CONFIG_DEBUG_COREDUMP=y
CONFIG_DEBUG_COREDUMP_BACKEND_LOGGING=y
```

---

## Planned Features

### 1. Display Integration (LVGL)

**Display Hardware:** OLED LCD (exact model TBD)

**LVGL Integration:**
- Enable `CONFIG_LVGL=y` in prj.conf
- Configure display driver (I2C or SPI based on hardware)
- Create UI widgets in `src/ui/`
- Implement display thread for rendering

**Planned UI Screens:**
- Home screen (burner status, temperature, mode)
- Menu navigation (encoder-driven)
- Settings screen (pump rate, target temperature, etc.)
- Status/error display
- Network configuration (node mode)

### 2. Helios Serial Protocol Integration

**Library:** `modules/lib/fusain` (Fusain Protocol implementation)

**Protocol Documentation:** `../../origin/docs/protocols/serial_protocol.md` (Fusain Protocol v2.0)

**Integration Steps:**
1. Add fusain module to CMakeLists.txt (like Helios app does)
2. Enable `CONFIG_FUSAIN=y`
3. Implement master-side serial handler in `src/communications/`
4. Create TX thread for command sending
5. Create RX thread for telemetry reception
6. Integrate with UI for display and control

**Command Types (Master → Helios ICU):**
- SET_MODE (idle, fan, heat, emergency)
- SET_PUMP_RATE
- SET_TARGET_RPM
- PING_REQUEST (keepalive)
- EMERGENCY_STOP

**Data Types (Helios ICU → Master):**
- TELEMETRY_BUNDLE (state, error, motor data, temperatures)
- PING_RESPONSE
- Error messages

### 3. Mode Switching (Normal vs Node)

**Configuration:** Runtime mode selection or compile-time configuration

**Normal Mode:**
- Enable display thread
- Enable input thread
- Enable Helios serial master (UART)
- Optional: WiFi for wireless operation

**Node Mode:**
- Disable display/input (optional: minimal status display)
- Enable Helios serial bridge
- Enable secondary communication interface:
  - Bluetooth server for phone app
  - CAN bus for system integration
  - Network server for remote control

### 4. Threading Model Implementation

**Pattern:** Follow Helios convention with `K_THREAD_DEFINE`

**Example (from future implementation):**
```c
K_THREAD_DEFINE(display_id, 4096, display_thread, NULL, NULL, NULL, 4, 0, 0);
K_THREAD_DEFINE(input_id, 2048, input_thread, NULL, NULL, NULL, 3, 0, 0);
K_THREAD_DEFINE(serial_master_id, 2048, serial_master_thread, NULL, NULL, NULL, 2, 0, 0);
K_THREAD_DEFINE(ui_state_id, 2048, ui_state_thread, NULL, NULL, NULL, 5, 0, 0);
```

**Thread Declarations:** Create `include/slate/threads.h` (like Helios)

### 5. State Machine (UI/Mode State)

**Framework:** Zephyr SMF (like Helios state machine)

**Proposed States:**
- INITIALIZING - Startup, hardware validation
- DISCONNECTED - No connection to Helios
- CONNECTED - Active connection to Helios
- MENU - User navigating menu
- MONITORING - Displaying burner status
- ERROR - Communication or system error
- NODE_BRIDGE - Node mode bridging

**State Transitions:** Driven by connection status, user input, errors

### 6. Wireless Communication

**WiFi (Pico 2W):**
- Access point mode for direct connection
- Station mode for network integration
- mDNS/DNS-SD for service discovery

**Bluetooth:**
- BLE GATT server for phone app
- Custom service UUID for Helios control
- Characteristics for commands and telemetry

**Implementation:** Create `src/communications/wireless.c`

### 7. CAN Bus Integration (Node Mode)

**Use Cases:**
- RV/Marine system integration
- Industrial automation
- Multi-heater systems

**CAN Message IDs:** TBD (define message protocol)

**Implementation:** Create `src/communications/can_handler.c`

---

## Building & Development

### Build Commands (via Taskfile)

**IMPORTANT:** Always use the Taskfile commands for building. Never run `west build` directly.

```bash
task build-firmware    # Build firmware (ALWAYS use this)
task flash-firmware    # Flash to device (USER ONLY - see safety note below)
task rebuild-firmware  # Clean and rebuild in one command
task menuconfig        # Zephyr Kconfig menu
task serial-terminal   # Connect via minicom
```

**SAFETY REQUIREMENT:** AI assistants must NEVER automatically execute `task flash-firmware`. See [Thermoquad Organization CLAUDE.md](../../CLAUDE.md) "Firmware Flashing Safety" section. After building firmware, always ask the user to manually flash it.

### Serial Console

```bash
minicom -D /dev/ttyACM0 -b 115200
```

**Note:** Serial port may vary based on system. Use `task serial-terminal` for automatic detection.

---

## Code Style & Formatting

**Formatter:** clang-format (config in `.clang-format` at project root)

**Conventions:**
- Use `snake_case` for functions and variables
- Use `UPPER_CASE` for constants and macros
- Thread entry point functions: `<subsystem>_thread()`
- Helper functions are `static` unless needed externally
- All multi-byte values use **little-endian** byte order

**Logging:**
- `LOG_ERR()` - Errors requiring attention
- `LOG_WRN()` - Warnings about unusual conditions
- `LOG_INF()` - Important state changes
- `LOG_DBG()` - Debug information

**Comments:**
- Group code into logical blocks with comment headers
- Document non-obvious behavior
- Explain "why" not "what" for complex algorithms

---

## Testing Strategy

**Current Testing:** Manual input validation via shell

**Planned Testing:**

1. **Input Testing:**
   - Encoder rotation accuracy
   - Short/long press detection
   - Debouncing effectiveness

2. **Display Testing:**
   - LVGL rendering performance
   - Screen transitions
   - Menu navigation

3. **Serial Protocol Testing:**
   - Command transmission
   - Telemetry reception
   - CRC validation
   - Timeout handling

4. **Integration Testing:**
   - Slate → Helios command/response loop
   - UI responsiveness during communication
   - Error recovery

5. **Wireless Testing:**
   - WiFi connection stability
   - Bluetooth pairing and communication
   - Range testing (normal mode wireless operation)

---

## Common Tasks for AI Assistants

### Adding Display Support

1. Identify OLED hardware (I2C address, driver compatibility)
2. Add display driver to device tree overlay
3. Enable `CONFIG_LVGL=y` and related options in prj.conf
4. Create `src/ui/display.c` with display thread
5. Create LVGL UI screens in `src/ui/screens/`
6. Integrate encoder input with LVGL navigation

### Adding Helios Serial Protocol

**Protocol Documentation:** `../../origin/docs/protocols/serial_protocol.md` (Fusain Protocol v2.0)

1. Add fusain module to CMakeLists.txt:
   ```cmake
   list(APPEND EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/fusain)
   ```
2. Enable in prj.conf: `CONFIG_FUSAIN=y`
3. Create `src/communications/serial_master.c`
4. Implement command sending functions
5. Implement telemetry parsing
6. Create Zbus channels for burner state
7. Integrate with UI for status display and control

### Implementing Mode Switching

1. Define mode enum in `include/slate/mode.h`
2. Create mode state machine
3. Implement mode selection UI
4. Conditionally start/stop threads based on mode
5. Configure communication interfaces per mode

### Adding Network Features

1. Configure WiFi in device tree (may require additional overlay)
2. Implement network configuration UI
3. Create network server thread
4. Define protocol for remote control (HTTP, WebSocket, custom TCP, etc.)
5. Integrate with Helios serial master for command forwarding

---

## Relationship to Other Projects

### Helios ICU (apps/helios/)

**Relationship:** Slate is the master controller, Helios is the slave ICU

**Shared Components:**
- helios_serial protocol library
- Build conventions (Taskfile, clang-format)
- Threading patterns (K_THREAD_DEFINE, Zbus)
- Code style and documentation standards

**Differences:**
- Slate has UI, Helios is headless
- Slate sends commands, Helios executes them
- Slate monitors telemetry, Helios generates it

### Fusain Protocol Library (modules/lib/fusain/)

**Relationship:** Slate uses this library as the master controller

**Integration:**
- Link fusain module via CMake
- Implement master-side command encoding
- Implement telemetry decoding
- Handle CRC validation and error recovery

**Protocol Specification:** `../../origin/docs/protocols/serial_protocol.md` (Fusain Protocol v2.0)

**Library Documentation:** `../../modules/lib/fusain/CLAUDE.md`

---

## Future Enhancements

**Planned:**
1. 🔲 LVGL display integration
2. 🔲 Helios serial protocol master implementation
3. 🔲 UI framework (menus, status screens)
4. 🔲 Mode switching (normal/node)
5. 🔲 WiFi wireless operation (normal mode)
6. 🔲 Bluetooth phone app interface
7. 🔲 CAN bus integration (node mode)
8. 🔲 Settings persistence (NVS storage)
9. 🔲 Firmware updates over network
10. 🔲 Data logging and diagnostics

**Considerations:**
- Battery power support (portable wireless controller)
- Sleep modes for power efficiency
- Security (authentication for network/BT access)
- Multi-language UI support
- Configurable units (°C/°F, etc.)

---

## Troubleshooting

### Build Errors

- **Missing CONFIG_***: Add to `prj.conf`
- **Device tree errors**: Check board overlay syntax
- **Module not found**: Verify helios_serial path in CMakeLists.txt

### Runtime Issues

- **Encoder not responding**: Check GPIO pins in overlay, verify pull-ups
- **Button not detected**: Check debounce settings, verify active-low configuration
- **CAN not working**: Verify MCP2515 SPI wiring, check interrupt pin
- **Display blank**: Check I2C/SPI connection, verify driver configuration
- **Network not connecting**: Check WiFi credentials, verify DHCP client

### Common Mistakes

- Not reading org CLAUDE.md first (development conventions)
- Forgetting to update device tree when adding hardware
- Not testing input events before implementing UI
- Mixing Helios (slave) patterns with Slate (master) patterns
- Incorrect byte order (must be little-endian)

---

## Git Workflow

**IMPORTANT: This project follows the organization git workflow documented in the [Thermoquad Organization CLAUDE.md](../../CLAUDE.md#git-workflow).**

**Summary:**
1. Show changes with `git diff` or `git diff --staged`
2. Explain modifications and rationale
3. Show proposed commit message
4. Wait for explicit approval
5. Only then commit

**Never commit without showing changes first.**

**Commit Style:** Conventional Commits (see org CLAUDE.md)

**Scopes for Slate:**
- `ui` - User interface and display
- `input` - Encoder and button handling
- `serial` - Helios serial protocol integration
- `network` - WiFi/Bluetooth communication
- `can` - CAN bus integration
- `mode` - Mode switching logic
- `state` - State machine

---

## Resources

### Zephyr Documentation
- **Zephyr RTOS:** https://docs.zephyrproject.org/
- **LVGL Integration:** https://docs.zephyrproject.org/latest/samples/modules/lvgl/index.html
- **Input Subsystem:** https://docs.zephyrproject.org/latest/services/input/index.html
- **Networking:** https://docs.zephyrproject.org/latest/connectivity/networking/index.html
- **CAN Bus:** https://docs.zephyrproject.org/latest/connectivity/can/index.html

### Hardware Documentation
- **RP2350 Datasheet:** https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
- **Pico 2 Documentation:** https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html

### Project Documentation
- **Organization:** `../../CLAUDE.md` - Thermoquad organization conventions
- **Helios ICU:** `../helios/CLAUDE.md` - Slave ICU firmware reference
- **Helios Serial:** `../../modules/lib/helios_serial/CLAUDE.md` - Protocol library

---

## Project History

This project has progressed from early prototype to functional telemetry display:

**Completed Milestones:**
- ✅ Input hardware validation (quadrature encoder, button detection)
- ✅ LVGL display integration with SSD1306 OLED
- ✅ Helios serial protocol integration (RX only, via Fusain library)
- ✅ Telemetry display implementation (state, temperature, RPM)
- ✅ Packet validation to prevent display corruption

**In Progress:**
- Full master implementation (command transmission)
- Interactive UI with menu system

**Development with AI Assistance:**
All development is documented in this CLAUDE.md file to guide AI assistants. Follow the "ask questions, don't assume" philosophy documented in the organization CLAUDE.md.

**Last Updated:** 2026-01-04

---

## Contact & Support

This is part of the Thermoquad project. For questions or issues, refer to the project repository.

**Remember:**
- Read org CLAUDE.md first
- Ask questions when requirements are unclear
- Follow Helios patterns where practical
- Test on real hardware frequently
- Document architecture decisions
