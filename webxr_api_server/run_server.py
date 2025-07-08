import uvicorn
import socket
import argparse
import os
import sys
from pathlib import Path

def get_local_ip():
    """Get the local IP address of the machine"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('8.8.8.8', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def check_ros2_environment():
    """Check if ROS2 environment is properly set up"""
    ros_distro = os.environ.get('ROS_DISTRO')
    if not ros_distro:
        print("⚠️  WARNING: ROS_DISTRO environment variable not found!")
        print("   Make sure to source your ROS2 setup before running:")
        print("   source /opt/ros/humble/setup.bash  # or your ROS2 distro")
        print()
        return False
    
    try:
        import rclpy
        print(f"✅ ROS2 environment detected: {ros_distro}")
        return True
    except ImportError:
        print("❌ ERROR: rclpy not found!")
        print("   Install ROS2 Python dependencies:")
        print("   sudo apt install python3-rclpy python3-geometry-msgs")
        return False

def main():
    parser = argparse.ArgumentParser(description='Run FastAPI server with HTTPS and ROS2 integration')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=8443, help='Port to bind to')
    args = parser.parse_args()

    # Check ROS2 environment
    if not check_ros2_environment():
        response = input("Continue anyway? (y/N): ")
        if response.lower() != 'y':
            sys.exit(1)

    local_ip = get_local_ip()
    
    # SSL setup
    cert_path = Path('certs/cert.pem')
    key_path = Path('certs/key.pem')
    
    if not cert_path.exists() or not key_path.exists():
        print("❌ ERROR: Certificate files not found. Run the setup script first.")
        return
    
    print("="*60)
    print(f"🚀 Starting HTTPS server with ROS2 integration")
    print(f"   Address: {local_ip}:{args.port}")
    print("="*60)
    print(f"📱 Access URLs:")
    print(f"   - Main App: https://{local_ip}:{args.port}/static/index.html")
    print(f"   - WebXR Test: https://{local_ip}:{args.port}/static/webxr_test.html")
    print(f"   - Pose Monitor: https://{local_ip}:{args.port}/static/gui.html")
    print(f"   - Server Info: https://{local_ip}:{args.port}/server-info")
    print(f"   - ROS Info: https://{local_ip}:{args.port}/ros-info")
    print("="*60)
    print(f"🤖 ROS2 Integration:")
    print(f"   - Topic: /webxr/pose (geometry_msgs/PoseStamped)")
    print(f"   - Node: webxr_pose_publisher")
    print(f"   - Test with: ros2 topic echo /webxr/pose")
    print("="*60)
    print("⚠️  SSL Certificate Warnings:")
    print("   You will see security warnings about the self-signed certificate.")
    print("   You need to accept these warnings to proceed.")
    print("="*60)
    
    print("🔧 Controls:")
    print("   - Press Ctrl+C once to gracefully shut down")
    print("   - ROS2 node will shutdown automatically")
    print("="*60)
    
    try:
        uvicorn.run(
            "server:app",  # Make sure this matches your server file name
            host=args.host,
            port=args.port,
            reload=False,  # Changed from True - reload can cause issues with ROS2
            ssl_keyfile=str(key_path),
            ssl_certfile=str(cert_path)
        )
    except KeyboardInterrupt:
        print("\n🛑 Shutting down server...")
        print("   ROS2 cleanup handled by FastAPI lifespan manager")
    except Exception as e:
        print(f"❌ Error starting server: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()