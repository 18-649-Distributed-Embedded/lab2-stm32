### Phase 1: Repository & RTOS Scaffolding

Establish a modular Zephyr application structure to isolate hardware drivers from control logic.

* **Directory Structure:**
* `/boards`: `nucleo_f401re.overlay` (all hardware overrides, pinmuxing, and devicetree aliases).
* `/src/hw`: Hardware abstraction layers (e.g., `encoders.c`, `l298n.c`, `servo.c`).
* `/src/sys`: RTOS threads and FSMs (e.g., `network.c`, `drivetrain.c`, `steering.c`).
* `/src/main.c`: RTOS initialization and thread spawning.


* **RTOS Configuration (`prj.conf`):** Enable `CONFIG_SERIAL`, `CONFIG_UART_INTERRUPT_DRIVEN`, `CONFIG_PWM`, `CONFIG_ADC`, and `CONFIG_EVENTS` for RTOS synchronization.
* **Shared Data Structures:** Define a central `struct vehicle_cmd` containing target steering angle, throttle/brake setpoints, and blinker states.

### Phase 2: UART Parser & Inter-Thread Communication

Isolate the asynchronous Raspberry Pi communication from the synchronous control loops.

* **Hardware Setup:** Configure USART2 (PA2/PA3) for 115200 baud communication.
* **Interrupt Service Routine (ISR):** Implement `uart_rx_isr()` to read incoming bytes from the hardware FIFO.
* **Frame Parsing:** Buffer incoming bytes until a valid packet delimiter is found. Verify the frame structure (length and checksum/bounds) to reject malformed data.


* **Message Queue (`k_msgq`):** Upon validating a frame, pack the data into a `vehicle_cmd` struct and push it to a Zephyr message queue (`k_msgq_put`). This immediately hands the data off to the RTOS without blocking the ISR.
* **Test Point Integration:** Toggle the `CMD_RX` GPIO pin inside the ISR to validate reception timing on the 13-point test breadboard.



### Phase 3: Network Health Manager

Manage system safety and telemetry transmission.

* **Thread Configuration:** Spawn `network_health_thread` with a medium priority and a strict 20 ms loop delay (`k_msleep(20)`).
* **Heartbeat Transmission:** Read the three ADC current sensors (PA0, PA1, PB0) and transmit the status frame back to the Pi over USART2.


* **Fail-Safe Watchdog:** Monitor the timestamp of the last valid command pulled from the `k_msgq`. If `k_uptime_get()` exceeds the 150 ms threshold (three missed commands), transition the global atomic `system_state` to `FAIL_SAFE`.



### Phase 4: Actuator FSMs (Drivetrain & Steering)

Execute physical control loops decoupled from the network parser.

* **Setpoint Synchronization:** Use a Zephyr Mutex (`k_mutex`) to safely read the latest `vehicle_cmd` struct updated by the UART parser.
* **Drivetrain Thread (2 ms Deadline):**
* Run at the highest RTOS priority.
* Read raw ticks directly from TIM1 (Right, PA8/PA9) and TIM2 (Left, PA5/PA1) hardware registers.
* Calculate current velocity and execute the PID control loop against the throttle setpoint.
* *Safety Override:* If `system_state == FAIL_SAFE` or the brake command is active, bypass the PID loop and immediately assert dynamic braking on the L298N pins.


* *Test Point:* Toggle `PWM_SET` immediately after writing the new L298N duty cycle to measure software response time against `CMD_RX`.




* **Steering Thread (50 ms Deadline):**
* Run at a high priority.
* Map the commanded steering axis to the 1.0ms–2.0ms servo PWM limits on TIM4 (PB6).
* Evaluate the physical turn threshold to set the auto-cancel flag for the blinker FSM.





### Phase 5: Auxiliary FSMs & Hardware Sync

Manage low-priority indicators and physical inputs.

* **Blinker Thread (100 ms Deadline):**
* Run at low priority with a 100 ms sleep loop.
* If `system_state == NORMAL`, execute the 1 Hz (50% duty cycle) state machine for left/right turns based on the proxy input.


* If `system_state == FAIL_SAFE`, override standard blinkers and flash all four LEDs simultaneously at 2 Hz as hazards.




* **Self-Test Button (ISR + Workqueue):**
* Configure a physical wheel button index to trigger a Zephyr GPIO interrupt.
* Defer the debouncing logic to a Zephyr Workqueue (`k_work_submit`) to avoid blocking the hardware interrupt.
* Upon a valid press, force `system_state = FAIL_SAFE` within the 10 ms deadline.