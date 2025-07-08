import json
import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, RedirectResponse
from typing import Dict, List
import socket
import logging
from contextlib import asynccontextmanager

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from manip_teleop.msg import WebXRControl  # Import our custom message
import threading

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('webxr_pose_publisher')
        
        # Create publisher for WebXR control data with all needed information
        self.control_publisher = self.create_publisher(
            WebXRControl,
            '/webxr/control',
            10
        )
        
        # Timer for periodic tasks (optional)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('WebXR Pose Publisher Node initialized')
        
    def publish_pose(self, pose_data: Dict):
        """Publish pose data with control states to ROS topic"""
        try:
            self.get_logger().info(f"Attempting to publish pose data: {json.dumps(pose_data)[:100]}...")
            
            # Create WebXRControl message with all data
            control_msg = WebXRControl()
            
            # Set header
            control_msg.header = Header()
            control_msg.header.stamp = self.get_clock().now().to_msg()
            control_msg.header.frame_id = "webxr_frame"
            
            # Set position
            if "position" in pose_data:
                pos = pose_data["position"]
                control_msg.pose.position.x = float(pos.get('x', 0.0))
                control_msg.pose.position.y = float(pos.get('y', 0.0))
                control_msg.pose.position.z = float(pos.get('z', 0.0))
            
            # Set orientation (quaternion)
            if "orientation" in pose_data:
                ori = pose_data["orientation"]
                control_msg.pose.orientation.x = float(ori.get('x', 0.0))
                control_msg.pose.orientation.y = float(ori.get('y', 0.0))
                control_msg.pose.orientation.z = float(ori.get('z', 0.0))
                control_msg.pose.orientation.w = float(ori.get('w', 1.0))
            else:
                # Default orientation (no rotation)
                control_msg.pose.orientation.w = 1.0
            
            # Set control states
            if "control" in pose_data:
                control = pose_data["control"]
                control_msg.gripper_open = bool(control.get('gripperOpen', False))
                control_msg.move_enabled = bool(control.get('moveEnabled', False))
            else:
                control_msg.gripper_open = False
                control_msg.move_enabled = False
            
            # Publish the WebXRControl message
            self.control_publisher.publish(control_msg)
            
            self.get_logger().info(
                f'Published to ROS topic: pos({control_msg.pose.position.x:.3f}, '
                f'{control_msg.pose.position.y:.3f}, {control_msg.pose.position.z:.3f}) '
                f'ori({control_msg.pose.orientation.x:.3f}, {control_msg.pose.orientation.y:.3f}, '
                f'{control_msg.pose.orientation.z:.3f}, {control_msg.pose.orientation.w:.3f}) '
                f'control(gripper_open={control_msg.gripper_open}, move_enabled={control_msg.move_enabled})'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing pose with control: {str(e)}')
    
    def timer_callback(self):
        """Optional periodic callback for housekeeping"""
        pass

class AsyncROSManager:
    def __init__(self):
        self.node = None
        self.executor = None
        self.ros_task = None
        
    async def start_ros(self):
        """Initialize and start ROS 2 node in async context"""
        try:
            # Initialize ROS 2
            rclpy.init()
            
            # Create node
            self.node = PosePublisherNode()
            
            # Create single-threaded executor
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            
            # Start ROS spinning in background task
            self.ros_task = asyncio.create_task(self._spin_ros())
            
            logger.info("ROS 2 node initialized and started")
            
        except Exception as e:
            logger.error(f"Error starting ROS: {str(e)}")
            raise
    
    async def _spin_ros(self):
        """Run ROS executor in async context"""
        try:
            loop = asyncio.get_event_loop()
            
            while rclpy.ok():
                # Spin once with timeout, then yield control back to asyncio
                await loop.run_in_executor(
                    None, 
                    lambda: self.executor.spin_once(timeout_sec=0.01)
                )
                
                # Yield control to other async tasks
                await asyncio.sleep(0.001)
                
        except asyncio.CancelledError:
            logger.info("ROS spinning task cancelled")
        except Exception as e:
            logger.error(f"Error in ROS spinning: {str(e)}")
    
    async def publish_pose(self, pose_data: Dict):
        """Publish pose data asynchronously"""
        if self.node:
            # Run the publish operation in executor to avoid blocking
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(
                None, 
                self.node.publish_pose, 
                pose_data
            )
    
    async def shutdown_ros(self):
        """Shutdown ROS 2 gracefully"""
        try:
            if self.ros_task:
                self.ros_task.cancel()
                try:
                    await self.ros_task
                except asyncio.CancelledError:
                    pass
            
            if self.executor:
                self.executor.shutdown()
            
            if self.node:
                self.node.destroy_node()
            
            rclpy.shutdown()
            
            logger.info("ROS 2 shutdown complete")
            
        except Exception as e:
            logger.error(f"Error during ROS shutdown: {str(e)}")

# Global ROS manager
ros_manager = AsyncROSManager()

# Store for connected clients
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.latest_pose: Dict = {}
        self.connection_count: int = 0

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        self.connection_count += 1
        logger.info(f"New client connected. Total clients: {self.connection_count}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            self.connection_count -= 1
            logger.info(f"Client disconnected. Total clients: {self.connection_count}")

    async def broadcast(self, message: Dict):
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except RuntimeError:
                disconnected.append(connection)
        
        for conn in disconnected:
            self.disconnect(conn)
    
    def update_pose(self, pose_data: Dict):
        self.latest_pose = pose_data

    def get_latest_pose(self) -> Dict:
        return self.latest_pose
        
    async def disconnect_all(self):
        logger.info(f"Disconnecting all {len(self.active_connections)} clients")
        for connection in self.active_connections[:]:
            try:
                await connection.close()
            except Exception:
                pass
            self.disconnect(connection)
        logger.info("All clients disconnected")

manager = ConnectionManager()

# Async context manager for FastAPI lifespan
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Starting FastAPI with ROS 2 integration...")
    await ros_manager.start_ros()
    
    yield
    
    # Shutdown
    logger.info("Shutting down FastAPI and ROS 2...")
    await manager.disconnect_all()
    await ros_manager.shutdown_ros()

# Create FastAPI app with lifespan
app = FastAPI(lifespan=lifespan)

# Serve static files
app.mount("/static", StaticFiles(directory="static"), name="static")

# Get local IP address
def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception as e:
        logger.error(f"Error getting local IP: {e}")
        return "127.0.0.1"

@app.get("/", response_class=HTMLResponse)
async def get(request: Request):
    return RedirectResponse(url="/static/index.html")

@app.get("/favicon.ico")
async def favicon():
    return RedirectResponse(url="/static/favicon.ico")

@app.get("/server-info")
async def server_info():
    local_ip = get_local_ip()
    return {
        "message": "WebXR Teleop Server with ROS 2",
        "local_ip": local_ip,
        "clients_connected": manager.connection_count,
        "server_status": "running",
        "ros_status": "active" if ros_manager.node else "inactive"
    }

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    client_ip = websocket.client.host
    logger.info(f"WebSocket connection attempt from: {client_ip}")
    await manager.connect(websocket)
    
    try:
        while True:
            # Receive pose data from client
            data = await websocket.receive_text()
            logger.info(f"Raw data received: {data[:100]}...") # Log the first 100 chars of raw data
            
            pose_data = json.loads(data)
            
            # Update the latest pose
            manager.update_pose(pose_data)
            
            # Publish to ROS 2 topic asynchronously
            await ros_manager.publish_pose(pose_data)
            
            # Log pose data
            if "position" in pose_data and "orientation" in pose_data:
                pos = pose_data["position"]
                ori = pose_data["orientation"]
                ctrl = pose_data.get("control", {})
                logger.info(f"Received and published AR pose:")
                logger.info(f"  Position: x={pos['x']}, y={pos['y']}, z={pos['z']}")
                logger.info(f"  Quaternion: x={ori['x']}, y={ori['y']}, z={ori['z']}, w={ori['w']}")
                logger.info(f"  Control: gripperOpen={ctrl.get('gripperOpen', False)}, moveEnabled={ctrl.get('moveEnabled', False)}")
            elif "position" in pose_data:
                pos = pose_data["position"]
                logger.info(f"Received and published AR pose: x={pos['x']}, y={pos['y']}, z={pos['z']}")
            
            # Broadcast to other WebSocket clients
            await manager.broadcast(pose_data)
            
    except WebSocketDisconnect:
        manager.disconnect(websocket)
        logger.info(f"Client {client_ip} disconnected")
    except Exception as e:
        logger.error(f"Error with client {client_ip}: {str(e)}")
        manager.disconnect(websocket)
        logger.info(f"Client {client_ip} disconnected due to error")

@app.get("/latest_pose")
async def get_latest_pose():
    return manager.get_latest_pose()

# Additional ROS info endpoint
@app.get("/ros-info")
async def ros_info():
    if ros_manager.node:
        return {
            "node_name": ros_manager.node.get_name(),
            "topics": ["/webxr/control"],
            "status": "active"
        }
    else:
        return {"status": "inactive"}