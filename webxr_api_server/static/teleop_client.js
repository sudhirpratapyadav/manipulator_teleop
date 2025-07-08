// DOM Elements
const connectBtn = document.getElementById('connect-btn');
const startXRBtn = document.getElementById('start-xr-btn');
const connectionStatus = document.getElementById('connection-status');
const connectionIndicator = document.getElementById('connection-indicator');

// Global variables
let socket = null;
let xrSession = null;
let referenceSpace = null;
let animationFrameId = null;
// Control state variables for AR session
let gripperOpen = false;
let moveEnabled = false;

// Check WebXR support
function checkXRSupport() {
    // Display HTTPS warning if needed
    if (window.location.protocol !== 'https:' && 
        window.location.hostname !== 'localhost' && 
        !window.location.hostname.startsWith('192.168.') && 
        !window.location.hostname.startsWith('10.') &&
        !window.location.hostname.match(/^127\.\d+\.\d+\.\d+$/)) {
        console.warn('WebXR typically requires HTTPS (except on localhost)');
    }
    
    if ('xr' in navigator) {
        console.log('WebXR is available in navigator');
        
        // Check for AR support
        navigator.xr.isSessionSupported('immersive-ar')
            .then((supported) => {
                if (supported) {
                    console.log('✅ WebXR immersive-ar supported');
                    // Only enable the Start AR button if we're connected to the server
                    startXRBtn.disabled = !(socket && socket.readyState === WebSocket.OPEN);
                    startXRBtn.textContent = 'Start AR Session';
                } else {
                    console.log('❌ AR not supported on this device');
                    startXRBtn.disabled = true;
                }
            })
            .catch(err => {
                console.error('Error checking WebXR support:', err);
            });
    } else {
        console.log('WebXR not supported by this browser');
        startXRBtn.disabled = true;
    }
}

// Connect to WebSocket server
function connectToServer() {
    // Get the current location to build the WebSocket URL dynamically
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.host;
    const wsUrl = `${protocol}//${host}/ws`;
    
    console.log(`Connecting to WebSocket at: ${wsUrl}`);
    socket = new WebSocket(wsUrl);
    
    socket.onopen = () => {
        connectionStatus.textContent = 'WebSocket: Connected';
        connectionIndicator.classList.add('connected');
        connectionIndicator.classList.add('pulse');
        document.getElementById('connection-card').classList.add('connected');
        connectBtn.textContent = 'Disconnect';
        connectBtn.classList.add('connected');
        // Check WebXR support and enable the button if supported
        checkXRSupport();
    };
    
    socket.onclose = () => {
        connectionStatus.textContent = 'WebSocket: Disconnected';
        connectionIndicator.classList.remove('connected');
        connectionIndicator.classList.remove('pulse');
        document.getElementById('connection-card').classList.remove('connected');
        connectBtn.textContent = 'Connect to Server';
        connectBtn.classList.remove('connected');
        startXRBtn.disabled = true;
        
        // If XR session is active, stop it
        if (xrSession) {
            stopXRSession();
        }
    };
    
    socket.onerror = (error) => {
        console.error('WebSocket error:', error);
        connectionStatus.textContent = 'WebSocket: Error';
    };
}

// Disconnect from WebSocket server
function disconnectFromServer() {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.close();
    }
}

// Toggle WebSocket connection
function toggleConnection() {
    if (!socket || socket.readyState === WebSocket.CLOSED || socket.readyState === WebSocket.CLOSING) {
        connectToServer();
    } else {
        disconnectFromServer();
    }
}

// Start WebXR session
async function startXRSession() {
    try {
        if ('xr' in navigator) {
            console.log('Starting AR session...');
            
            // Create a DOM overlay container for AR mode
            const overlayElement = document.createElement('div');
            overlayElement.id = 'xr-overlay';
            overlayElement.style.width = '100%';
            overlayElement.style.height = '100%';
            overlayElement.style.position = 'fixed';
            overlayElement.style.top = '0';
            overlayElement.style.left = '0';
            overlayElement.style.zIndex = '999999';
            overlayElement.style.pointerEvents = 'none'; // Allow clicks to pass through except for our button
            document.body.appendChild(overlayElement);
            
            // Reset control state variables
            gripperOpen = false;
            moveEnabled = false;
            
            // Create button styling function
            function createARButton(id, text, bottomPosition, leftPosition) {
                const button = document.createElement('button');
                button.innerText = text;
                button.style.position = 'absolute';
                button.style.bottom = bottomPosition;
                button.style.left = leftPosition;
                button.style.transform = 'translateX(-50%)';
                button.style.padding = '16px 20px';
                button.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                button.style.color = 'white';
                button.style.border = 'none';
                button.style.borderRadius = '30px';
                button.style.fontSize = '16px';
                button.style.fontWeight = 'bold';
                button.style.boxShadow = '0 4px 8px rgba(0, 0, 0, 0.3)';
                button.style.width = 'auto';
                button.style.minWidth = 'unset';
                button.style.maxWidth = 'unset';
                button.style.alignItems = 'center';
                button.style.justifyContent = 'center';
                button.style.pointerEvents = 'auto';
                button.style.flex = 'none';
                button.style.userSelect = 'none';
                button.id = id;
                return button;
            }
            
            // Create an exit button in the overlay
            const overlayExitBtn = createARButton('xr-exit-btn', 'Exit', '30px', '50%');
            
            // Add click event before appending to DOM
            overlayExitBtn.addEventListener('click', () => {
                console.log('Exit button clicked');
                if (xrSession) {
                    xrSession.end();
                }
            });
            
            // Create gripper toggle button
            const gripperBtn = createARButton('xr-gripper-btn', 'Open Gripper', '30px', '20%');
            gripperBtn.addEventListener('click', () => {
                gripperOpen = !gripperOpen;
                gripperBtn.innerText = gripperOpen ? 'Close Gripper' : 'Open Gripper';
                console.log(`Gripper state changed to: ${gripperOpen ? 'Open' : 'Closed'}`);
            });
            
            // Create move momentary button (active only while pressed)
            const moveBtn = createARButton('xr-move-btn', 'Hold to Move', '30px', '80%');
            
            // Add touch/mouse down event (activate when pressed)
            moveBtn.addEventListener('mousedown', () => {
                moveEnabled = true;
                // Keep text consistent, only change color
                moveBtn.style.backgroundColor = 'rgba(46, 204, 113, 0.8)'; // Green background when active
                console.log('Move state changed to: Enabled');
            });
            moveBtn.addEventListener('touchstart', () => {
                moveEnabled = true;
                // Keep text consistent, only change color
                moveBtn.style.backgroundColor = 'rgba(46, 204, 113, 0.8)'; // Green background when active
                console.log('Move state changed to: Enabled');
            });
            
            // Add touch/mouse up and leave events (deactivate when released)
            const deactivateMove = () => {
                moveEnabled = false;
                moveBtn.style.backgroundColor = 'rgba(0, 0, 0, 0.7)'; // Reset to original color
                console.log('Move state changed to: Disabled');
            };
            
            moveBtn.addEventListener('mouseup', deactivateMove);
            moveBtn.addEventListener('mouseleave', deactivateMove);
            moveBtn.addEventListener('touchend', deactivateMove);
            moveBtn.addEventListener('touchcancel', deactivateMove);
            
            // Add all buttons to overlay
            overlayElement.appendChild(overlayExitBtn);
            overlayElement.appendChild(gripperBtn);
            overlayElement.appendChild(moveBtn);
            
            // Set up AR session
            let sessionOptions = {
                requiredFeatures: ['local']
            };
            
            // Add DOM Overlay as optional feature
            sessionOptions.optionalFeatures = ['dom-overlay'];
            sessionOptions.domOverlay = { root: overlayElement };
            console.log('DOM Overlay configuration:', sessionOptions);
            
            // Request AR session
            xrSession = await navigator.xr.requestSession('immersive-ar', sessionOptions);
            console.log('AR session started with DOM overlay');
            
            // Set up session events
            xrSession.addEventListener('end', onXRSessionEnded);
            
            // Get the canvas for WebXR rendering (required even if we don't render anything)
            const canvas = document.createElement('canvas');
            document.body.appendChild(canvas);
            canvas.style.display = 'none'; // Hide the canvas as we don't need to see it
            
            // Get WebGL context
            const gl = canvas.getContext('webgl', { xrCompatible: true });
            if (!gl) {
                throw new Error('WebGL not supported');
            }
            
            // Initialize WebGL for XR
            const xrGlLayer = new XRWebGLLayer(xrSession, gl);
            await xrSession.updateRenderState({
                baseLayer: xrGlLayer
            });
            
            // Get reference space for pose tracking
            try {
                // First try local-floor for better tracking if available
                referenceSpace = await xrSession.requestReferenceSpace('local-floor');
                console.log('Using local-floor reference space');
            } catch (e) {
                // Fallback to local reference space
                console.log('local-floor not available, using local reference space');
                referenceSpace = await xrSession.requestReferenceSpace('local');
            }
            
            // Start the XR animation loop
            xrSession.requestAnimationFrame(onXRAnimationFrame);
            
            // Update UI
            startXRBtn.disabled = true;
        } else {
            console.error('WebXR not supported');
        }
    } catch (error) {
        console.error('Error starting AR session:', error);
    }
}



// XR Animation Frame callback
function onXRAnimationFrame(time, frame) {
    if (!xrSession) return;
    
    // Request the next frame
    animationFrameId = xrSession.requestAnimationFrame(onXRAnimationFrame);
    
    // Get the pose
    const pose = frame.getViewerPose(referenceSpace);
    
    if (pose) {
        // Extract position and orientation from pose
        const position = pose.transform.position;
        const orientation = pose.transform.orientation;
        
        // Create a pose data object with control states
        const poseData = {
            timestamp: new Date().toISOString(),
            position: {
                x: position.x.toFixed(4),
                y: position.y.toFixed(4),
                z: position.z.toFixed(4)
            },
            orientation: {
                x: orientation.x.toFixed(4),
                y: orientation.y.toFixed(4),
                z: orientation.z.toFixed(4),
                w: orientation.w.toFixed(4)
            },
            control: {
                gripperOpen: gripperOpen,
                moveEnabled: moveEnabled
            }
        };
        
        // Send the pose data via WebSocket if connected
        if (socket && socket.readyState === WebSocket.OPEN) {
            const jsonData = JSON.stringify(poseData);
            console.log(`Sending data to server: ${jsonData.substring(0, 100)}...`);
            socket.send(jsonData);
        } else {
            console.warn('WebSocket not connected, cannot send pose data');
        }
    } else {
        console.log('No pose available in this frame');
    }
}

// Handle XR session end
function onXRSessionEnded() {
    console.log('WebXR session ended');
    
    // Clean up session variables
    xrSession = null;
    referenceSpace = null;
    
    // Remove the overlay if it exists
    const overlayElement = document.getElementById('xr-overlay');
    if (overlayElement) {
        document.body.removeChild(overlayElement);
    }
    
    // Update UI
    startXRBtn.disabled = false;
    
    // If there was a canvas created for the session, remove it
    const tempCanvas = document.querySelector('canvas[style*="display: none"]');
    if (tempCanvas) {
        tempCanvas.remove();
    }
}

// Stop XR session
function stopXRSession() {
    if (xrSession) {
        xrSession.end();
    }
}

// Add server status check before connecting
function checkServerStatus() {
    // Get the current location to build the URL dynamically
    const url = `${window.location.protocol}//${window.location.host}/server-info`;
    
    console.log(`Checking server status at: ${url}`);
    fetch(url)
        .then(response => response.json())
        .then(data => {
            console.log('Server status:', data);
            if (data.server_status === 'running') {
                console.log('Server is running, can connect to WebSocket');
            } else {
                console.warn('Server status check failed or server not running');
            }
        })
        .catch(error => {
            console.error('Error checking server status:', error);
        });
}

// Call status check on page load
document.addEventListener('DOMContentLoaded', checkServerStatus);

// Event listeners
connectBtn.addEventListener('click', toggleConnection);
startXRBtn.addEventListener('click', startXRSession);

// Initialize
checkXRSupport();
