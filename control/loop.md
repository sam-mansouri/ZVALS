``` mermaid
flowchart TD
    A[Start: Receive Target Position Vector via RF] --> B[Acquire Current Drone State: Position, Velocity, Attitude from Sensors GPS, IMU, Altimeter]
    B --> C[Compute Error: Difference Between Target and Current Position/Velocity]
    C --> D[Outer Loop Controller: Position Control e.g., PID for X,Y,Z]
    D --> E[Generate Reference Commands: Desired Velocity/Attitude for Inner Loops]
    E --> F[Inner Loop Controller: Velocity/Attitude Control e.g., PID or Model Predictive Control]
    F --> G[Generate Actuator Commands: Thrust, Torque for Motors/Propellers]
    G --> H[Apply to Drone Dynamics: Update Position, Velocity, Attitude]
    H --> I[Check Landing Criteria: Proximity to Target, Low Velocity, Touchdown Detection?]
    I -->|No| B
    I -->|Yes| J[End: Safe Landing Achieved]
    subgraph "Feedback Loop"
        B --> C --> D --> E --> F --> G --> H --> B
    end
```