# tod_static_entities {#tod_static_entities_docs}

Static Entities are part of the tod_graphics_library and represent fixed elements in the scene. Unlike dynamic entities that update every frame, static entities are typically created once and remain unchanged throughout the runtime. They provide essential visual context and reference points in the environment.

Static entities include components such as:

- **Camera**: Manages the camera component and updates its state based on vehicle data.
- **CoordinateSystem**: Visualizes coordinate frames (e.g., World Frame, BaseFootPrint).
- **Display**: A 3D Car HUD that displays vehicle information using text.
- **Floor**: Represents the floor of the scene.
- **Grid**: Renders an equally spaced grid on the floor.
- **OrbitalPoint**: Creates an orbital point entity using a circle mesh attached to a parent entity.

---

| Entity           | Description                                                                      |
|------------------|----------------------------------------------------------------------------------|
| **Camera**       | Holds the camera component and updates based on gear changes.                  |
| **CoordinateSystem** | Visualizes coordinate frames in the scene.                          |
| **Display**      | Provides a 3D Car HUD for displaying vehicle information.                      |
| **Floor**        | Creates a static floor entity in the scene.                                    |
| **Grid**         | Renders an equally spaced grid on the floor.                                   |
| **OrbitalPoint** | Generates an orbital point using a circle mesh attached to a parent entity.      |
