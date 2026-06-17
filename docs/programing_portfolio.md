# Team Xeo #14278 - Programming Portfolio

Source: extracted from `14278_teamxeo_tportfolio.pdf` and reformatted into a programming-focused document.

## Programming Team

### Programming Lead

**Tudor N.**

- Lead Programming
- Second season
- Skills: code structuring and documentation

### Programming Member

**Alex B.**

- Programming
- First season
- Skills: Kotlin coroutines and dependency injection

## Programming Goals

The programming work supported the robot's main competitive goals:

- Execute fast and precise movements.
- Launch artifacts along a consistent trajectory while following the detected pattern.
- Automatically adjust launching speed based on the robot's field position.
- Complete as many artifact cycles as possible according to the match motif.
- Open the gate during autonomous to prevent artifact overflow.
- Fully park in the End Game period.

## Engineering Process

The technical department used a Gantt chart to plan and track tasks across 3D modeling, assembly, programming, and evaluation. Each task had a responsible member and a clear deadline, and progress was continuously monitored to ensure the robot met its goals and performance targets.

The robot development process followed the V-model:

- Requirements definition
- Architecture
- Detailed design
- Implementation
- Unit testing
- Full system testing
- System testing
- System validation

If problems occurred during testing or validation, the process returned to earlier stages for correction.

The team also aligned its process with the NASA Project Life Cycle:

- Phase A: Concept studies and brainstorming
- Phase B: Preliminary design and virtual prototyping
- Phase C: Final design and manufacturing
- Phase D: Integration and full testing
- Phase E: Operations, support, practice, and League Meets

## Game Objectives Supported by Software

### Autonomous

- Sort artifacts according to the match motif.
- Exit completely from the launch zone by the end of autonomous.

### TeleOp

- Score an average of 48 artifacts.
- Keep the turret constantly oriented toward the target.
- Shoot quickly, regardless of motif.
- Open the gate quickly and collect immediately.
- Automatically adjust shooter angle and firing speed based on each launch zone.

### Endgame

- Shoot according to the match motif.
- Complete a full parking maneuver.

## Kotlin Architecture

The team chose Kotlin not just as an alternative to Java, but as a strategic choice for improving reliability and development speed.

| Functionality | Issue in Java | Kotlin solution | Competitive impact |
| --- | --- | --- | --- |
| Coroutines | Asynchronous handling requires complex threading. | `launch { ... }` provides simple, low-overhead concurrency. | Subsystems are decoupled and do not block the main loop. |
| Lambdas and higher-order functions | Callbacks require anonymous classes of several lines. | `runAfter(2.0) { claw.open() }` creates a compact one-line callback. | Asynchronous programming is clearer and faster to write. |
| Properties | Getter and setter functions are verbose, such as `robot.drive.setPower(1.0)`. | Direct access, such as `robot.drive.power = 1.0`. | Code is simpler and easier to read. |
| Null safety | Unchecked null values can crash the robot program. | Safe-call operators such as `?.` force null handling. | Failure cases are explicit in the code. |

### Additional Kotlin Benefits

- Kotlin syntax is similar to Python, which helped new members coming from FLL learn the codebase faster.
- Kotlin data classes reduce boilerplate. For example, `data class Point(val x: Double)` automatically generates functions that would require many lines in Java.

### Java Interoperability

The transition to Kotlin required minor compatibility work with Java-based FTC tools, including the SDK, FTC Dashboard, and PedroPathing.

One example was the use of `@JvmField` for variables controlled through FTC Dashboard. Since Dashboard expects Java-written code, this annotation exposes the variable in a Java-compatible way. A similar compatibility adjustment was needed for PedroPathing.

## Coroutines

The main advantage of adopting Kotlin was the use of coroutines. They changed how robot logic was structured by enabling real multitasking that is difficult to achieve cleanly in Java.

In a traditional FTC architecture, robot code runs in a single `loop()`. If a complex mathematical computation takes 10 ms, the entire robot, including joystick responsiveness, freezes for that duration.

The team's solution was to run subsystems in independent coroutine-based loops:

- **Lane 1: Driver control** - Top priority; keeps the robot responsive to the driver.
- **Lane 2: Automation** - Uses sensor feedback to detect and sort artifacts.
- **Lane 3: Calculations** - Performs real-time shooter trajectory calculations.

All lanes run simultaneously. Even if trajectory calculations are heavy, driver control remains responsive.

### Reactive Data Streams

Coroutines allowed the team to use reactive flows instead of manual sensor polling. The code remains idle until a sensor event occurs, then reacts immediately.

This enabled:

- More efficient sensor handling
- Cleaner background processing
- Real-time shooter trajectory calculations
- Autonomous routines written as readable linear sequences, such as "Move -> Wait -> Shoot", instead of large state machines

## PedroPathing DSL

The Pedro Pathing library was selected because its reactive vector-based Drive Vectoring architecture gave the robot greater defensive robustness than more rigid trajectory systems.

The team then built a custom domain-specific language, `PedroPathingDSL`, to make autonomous path writing faster and easier during competitions.

### DSL Definition

A domain-specific language is a "language within a language" designed for one purpose. In this case, the DSL describes robot trajectories.

### Community Contribution

`PedroPathingDSL` was developed by the team and released as open source to help other FTC teams write cleaner autonomous code.

### Java vs. Kotlin DSL

| Java standard Pedro | Kotlin DSL |
| --- | --- |
| Requires 20+ lines of code to define 3 points. | Defines the same structure in about 5 lines. |
| Requires manual object instantiation, such as `new Point()`. | Points are added through logical composition, such as `+Pose()`. |
| Difficult to visualize quickly. | Reads like a narrative of movement. |

## Coroutine Integration

The key innovation was the engine behind the visual DSL, not only the DSL syntax itself.

Everything starts with `followSuspend`, the fundamental function that replaces the classic loop-based approach. It runs the underlying update loop until the trajectory is complete, while exposing the behavior through a coroutine-based API.

The result is that the processor remains free to handle other data and subsystem logic simultaneously.

### `followAndIntake`

`followAndIntake` simultaneously launches robot movement and the intake mechanism using `select`.

Logic:

> Move toward the balls, but if they are picked up along the way, stop immediately.

If the intake reports that it is full halfway through a path, the function automatically cancels the remaining trajectory. The robot switches immediately to scoring, saving valuable seconds.

### `holdSuspend`

`holdSuspend` anchors the robot to a position using PedroPathing vectors. This helps the robot resist external pushes while scoring mechanisms continue to run in the background.

## Dependency Injection

The team used Metro for dependency injection to decouple robot logic from hardware.

The OpMode requests high-level components, such as `Intake`, while the library automatically composes them from subsystem and hardware layers.

Example structure:

```text
Sorter -> SorterImpl -> Transfer -> TransferImpl -> Motor -> Hardware
```

Because each component can be replaced with a fake implementation, the architecture enables unit testing even when the physical robot is not available.

## Robot Systems Programming

### TeleOp Strategy

In TeleOp, the driver chooses between the Close cycle and the Far cycle.

Close cycle:

- The robot opens the gate using bumpers.
- The intake is placed directly into the opening.
- Balls are collected immediately as they are released.

Far cycle:

- The robot cycles from the distant corner.
- This is coordinated with the alliance partner, who opens the gate on their side.

The choice depends on the alliance partner's cycle time. If the partner cycles quickly in Close, Team Xeo switches to Far to avoid blocking them.

During the final 30 seconds, the robot automatically switches to Pattern Mode:

- The shooter aligns precisely.
- Balls are placed in the required order.
- The alliance pattern is completed.

### Autonomous Strategy

The team uses two standard autonomous routines:

- Close Auto
- Far Auto

If the alliance partner cannot use their autonomous routine, Team Xeo also has a Solo Auto routine that places all balls in a pattern.

The Close/Far selection is designed to cover the partner's weaknesses. The robot takes over the slower side to maximize alliance efficiency.

Both primary autonomous routines quickly score the first balls and keep the last 6 for the pattern, with the partner contributing the other 3.

Far Auto depends on the partner opening the gate. The balls roll back into the far corner, where the robot collects them.

### Autonomous Path Plans

Close Auto:

```text
preload -> row 2 -> gate x (n - 1) -> final gate -> row 1
```

Far Auto:

```text
preload -> row 3 -> corner x (n - 2) -> final corner x 2
```

Solo Auto:

```text
preload -> row 1 -> row 2 -> row 3
```

In these plans, `n` continuously increases as travel time, callback timing, and path velocity are optimized to maximize cycles within the 30-second autonomous period.

### Istanbul Premier Event Improvements

Far Auto:

- Added camera-based path selection.
- Right before cycling to the corner, the camera chooses between two collection paths:
  - one tight along the wall
  - one about one robot length away
- The robot automatically selects the path that maximizes the chance of collecting all available artifacts.

Close Auto:

- Added one additional full cycle compared with the previous version.

## Codebase Practices

The codebase follows standard practices:

- High reuse
- Red and Blue autonomous code share the same class through mirroring
- Clear naming conventions
- Feature-based Git branching
- Code review before merging

## Subsystem Programming

### Drive

The drive system uses:

- 4 mecanum wheels
- 4 motors
- GoBilda Pinpoint localization
- PedroPathing follower control in both Autonomous and TeleOp

The follower allows the driver to switch between field-centric and robot-centric movement.

Field-centric:

```text
joystick up = north, always
```

Robot-centric:

```text
joystick up = robot front, variable based on robot heading
```

### Intake

Intake versions:

- **v1** - Basic roller motor tied to a button input.
- **v2** - Added rollers at the sorter input to retain artifacts that the sorter's centrifugal force would otherwise eject.
- **v3** - Added two side bumpers used to open the gate during the Close cycle.

### Sorter

The sorter is a 3-slot disc actuated by a geared servo. The transmission ratio causes multiple full rotations within the servo's range, enabling multiple valid slot positions and optimal path selection.

Sorter versions:

- **v1** - Each slot had a fixed intake and shooter position. The disc always moved to a predefined angle, independent of current position.
- **v2** - Added REV Color Sensor V3 on the intake ramp for color detection. Integrated the secondary roller mechanism from Intake v2. In idle mode, the intake rotates slowly and continuously to prevent ball loss. The shooting preparation algorithm computes all valid slot positions and selects the closest one, guaranteeing minimum rotation.
- **v3** - Relocated the color sensor above the first sorter slot. The sensor reads the ball only after it enters the disc, improving detection reliability.
- **v4** - Added side-mounted color sensors on each slot for per-slot validation. The Axon servo's fourth wire is used for load sensing, enabling automatic jam detection and response.

### Transfer

The transfer subsystem transports artifacts from the sorter to the shooter.

Transfer versions:

- **v1** - Used a continuous-rotation servo.
- **v2** - Switched from a servo to a motor for faster operation, with negligible programming changes.

### Parking

The parking mechanism is controlled by a single motor. It controls the endgame tilt mechanism, allowing both alliance robots to be parked. The turret rotates to reposition the center of mass to the rear.

## Shooter Programming

The shooter consists of:

- Flywheel driven by a motor
- Servo-controlled hood for launch angle
- Turret servo for horizontal rotation

Shooter versions:

- **v1** - Included a flywheel motor, servo-controlled hood, and turret servo for horizontal aiming.
- **v2** - Combined motor encoder feedback with AprilTag-based distance estimation. The system computes a target velocity from distance and uses PID plus feedforward control to maintain flywheel speed.
- **v3** - Used two motors and distance-based lookup tables for hood position and motor speed. Camera-based vision and odometry independently compute turret angle and range, then validate each other to prevent single-source measurement errors.
- **v4** - Eliminated flywheel speed recovery delays between artifacts. The robot computes the ball trajectory from current flywheel velocity and dynamically adjusts launch angle. While rear-shooting, it continuously adjusts the guide flap to maintain cycle speed, allowing a single-motor flywheel design without sacrificing performance.

### Shooter Control Flow

```text
Odometry -> Distance -> Speed LUT -> PID + FF -> Flywheel
Camera -> Turret aiming
Trajectory calculation -> Hood
canShoot? -> Sorter timing -> Successful shot x 3
```

### V1-V3 vs. V4

V1-V3:

- Constant hood position
- Wait for flywheel acceleration
- Shoot

V4:

- Set flywheel speed setpoint
- Dynamically adjust hood based on flywheel speed
- Continuous shooting

## AutoShoot

`AutoShoot` is the convergence of all major subsystems:

- Drive
- Vision
- Speed and angle control
- Turret
- Sorter

These systems run independently and in parallel through coroutines.

## Vision, Sensors, and Feedback

The robot software uses:

- REV Color Sensor V3 for artifact color detection
- Side-mounted color sensors for per-slot sorter validation
- Axon servo load sensing for jam detection
- AprilTag-based distance estimation
- Camera-based turret alignment
- Camera-based Far Auto path selection
- GoBilda Pinpoint localization
- Motor encoder feedback for flywheel speed control

The shooter can use camera data to reorient the turret if odometry fails or becomes unreliable.

## Testing and Validation

### Collection Test

| Iteration | Collection time | Successfully collected artifacts |
| --- | --- | --- |
| V1 | about 10 s | 2/3 |
| V2 | about 5 s | 3/3 |
| V3 | 2-3 s | 3/3 |

Across the mechanical and programming iterations, collection performance improved significantly. The first iteration collected only 2 of 3 artifacts due to intake limitations. The second fixed reliability issues, and the third reduced execution time to 2-3 seconds.

### Rapid Shooting Test

| Iteration | Time from first to last artifact | Successfully shot artifacts |
| --- | --- | --- |
| V1 | about 1.2 s | 2/3 |
| V2 | about 0.7 s | 3/3 |
| V3 | about 0.4 s | 3/3 |

Rapid shooting lifts the entire sorter simultaneously, releasing all three artifacts in a very short time. Iterations reduced the time between the first and last artifact to about 0.4 seconds.

### Sorted Shooting Test

| Iteration | Time from first to last artifact | Pattern accuracy |
| --- | --- | --- |
| V1 | about 2 s | 1/3 |
| V2 | about 1.2 s | 3/3 |
| V3 | about 0.76 s | 3/3 |

Sorted shooting launches artifacts sequentially with 270 ms preparation intervals and 110 ms lifting time per artifact. The current version reaches 3/3 pattern accuracy with a first-to-last artifact time of about 0.76 seconds.

## Maintenance Relevant to Programming

The team regularly recalibrates:

- Color sensors
- Camera
- Cable connections
- Hub connectors
- Electronic components

Before and after competitions, the team checks cable continuity, recalibrates sensors and the camera, and verifies electronics. Before each match, the team replaces the battery, checks hub connector cables, and checks for screws at risk of loosening.

## Electronics Reliability

The portfolio emphasizes that software reliability depends on electronics reliability. A single disconnected cable can halt the robot during a match.

Programming-relevant electronics practices:

- Control Hub, Expansion Hub, battery, and power distributors are mounted for fast access.
- Cables are grouped by subsystem to reduce debugging time under pressure.
- Exposed cable runs are braided or placed in textile sleeves.
- Cables are secured with zip ties to reduce fatigue at connection points.
- Data cables, especially the GoBilda Pinpoint sensor cable, are wrapped in aluminum foil to reduce electromagnetic interference from motors.
- Control Hub and Expansion Hub are mounted on non-metallic standoffs to isolate them from the metal chassis.
- The power system includes a grounding wire from the power distribution hub to provide a stable ground reference and prevent sporadic hard-to-diagnose errors.
