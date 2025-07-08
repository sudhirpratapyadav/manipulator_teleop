# WebXR API Server Diagrams

## Architecture Diagram

```mermaid
graph TD
    subgraph Client Side
        A[Mobile Device<br>WebXR Client]
        B[Web Browser]
        L[DOM Overlay UI]
    end
    
    subgraph Server Side
        C[FastAPI Server]
        D[Connection Manager]
        E[REST API]
        F[WebSocket API]
        G[Static File Server]
        V[3D Visualization GUI]
        J[Latest Pose Memory]
    end
    
    subgraph HTTPS Layer
        H[Uvicorn HTTPS]
        I[Self-signed Certs]
    end

    A -- WebXR API --> B
    B -- DOM Overlay --> L
    B -- HTTPS/WebSocket --> H
    H -- forwards --> C
    C -- serves --> G
    C -- REST /ws --> F
    C -- REST /server-info --> E
    C -- REST /latest_pose --> E
    C -- manages --> D
    D -- stores --> J
    D -- broadcasts to --> V
    I -- enables --> H
```

## Sequence Diagram

```mermaid
sequenceDiagram
    participant User as Mobile User
    participant Browser as Web Browser
    participant Server as FastAPI Server
    participant Manager as Connection Manager
    participant GUI as 3D GUI Client

    User->>Browser: Open Web App
    Browser->>Server: HTTPS Request (index.html)
    Server->>Browser: Serve Static Files
    
    GUI->>Server: Connect to WebSocket
    Server->>Manager: Register GUI Connection
    
    User->>Browser: Click Connect
    Browser->>Server: WebSocket Connect (/ws)
    Server->>Manager: Register Mobile Connection
    
    User->>Browser: Start AR Session
    Browser->>Server: Send Pose Data (WebSocket)
    Server->>Manager: Update Latest Pose
    Manager->>GUI: Broadcast Pose Data
    GUI->>GUI: Update 3D Visualization
    
    Browser->>Server: REST Request (/latest_pose)
    Server->>Manager: Get Latest Pose
    Server->>Browser: Return Pose JSON
```

## Data Flow Diagram

```mermaid
flowchart LR
    A[WebXR Client] --> |1 - Establish WebSocket| B[FastAPI Server]
    A --> |2 - Send Pose Data| B
    B --> |3 - Store| C[Latest Pose]
    B --> |4 - Broadcast| D[Connected Clients]
    E[Robot/App] --> |5 - Request Latest Pose| B
    B --> |6 - Return Pose| E
    D --> |7 - Update Visualization| G[3D GUI]
```

## Error Handling Flow

```mermaid
flowchart TD
    A[Client Connection] --> B{WebSocket Connected?}
    B -->|Yes| C[Normal Operation]
    B -->|No| D[Retry Connection]
    
    C --> E{XR Session Active?}
    E -->|Yes| F[Send Pose Data]
    E -->|No| G[Display XR Not Available]
    
    F --> H{Connection Lost?}
    H -->|Yes| I[End XR Session]
    H -->|No| F
    
    I --> J[Update UI]
    J --> B
```

## Notes

- **HTTPS** is required for WebXR API access on most devices.
- **WebSocket** provides real-time pose streaming and broadcasting to all connected clients.
- **Connection Manager** tracks clients, broadcasts data, and maintains latest pose.
- **Static files** serve the web client, 3D visualization, and test pages.
- **Self-signed certificates** are used for local development.
- **DOM Overlay** provides UI controls within the AR experience.
- **3D Visualization** renders pose data in a three-dimensional scene with interactive controls.
- **Data Format:**
  - JSON with timestamp, position (x, y, z), orientation (x, y, z, w quaternion).
- **Error Handling:**
  - Connection failures are detected and reported
  - WebXR compatibility is checked before enabling AR
  - WebSocket connections are monitored and cleaned up if disconnected