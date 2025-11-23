"""
Python WebSocket Client for Telepresence Robot
Connects to Pi WebSocket server
-Sends tracking data from Unity/Glove ESP32 to Pi
-Receives sensor data (force feedback) from Pi
"""

import asyncio
import websockets
import json
from datetime import datetime

# Configuration
PI_WEBSOCKET_URL = 'ws://192.168.1.250:8080'  

class TeleopClient:
    def __init__(self, pi_url=PI_WEBSOCKET_URL):
        self.pi_url = pi_url
        self.websocket = None
        self.connected = False
        self.receive_callback = None  # Callback for incoming sensor data
        
    async def connect(self):
        """Connect to Pi WebSocket server"""
        try:
            self.websocket = await websockets.connect(self.pi_url)
            self.connected = True
            print(f"✓ Connected to Pi at {self.pi_url}")
            
            # Send ping to verify
            await self.send_ping()
            
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            self.connected = False
            return False
    
    async def disconnect(self):
        """Disconnect from Pi"""
        if self.websocket:
            await self.websocket.close()
            self.connected = False
            print("Disconnected from Pi")
    
    async def send_ping(self):
        """Send ping to verify connection"""
        if self.websocket:
            await self.websocket.send(json.dumps({'type': 'ping'}))
    
    async def send_tracking_data(self, finger_data, hand_position, hand_rotation, head_rotation):
        """
        Send tracking data to Pi
        
        Args:
            finger_data: dict with keys t, i, m, r, p (thumb, index, middle, ring, pinky)
            hand_position: dict with keys x, y, z
            hand_rotation: dict with keys p, y, r (pitch, yaw, roll)
            head_rotation: dict with keys p, y, r (pitch, yaw, roll)
        """
        if not self.websocket or not self.connected:
            return False
        
        try:
            data = {
                'type': 'tracking',
                'rotationFinger': finger_data,
                'positionHand': hand_position,
                'rotationHand': hand_rotation,
                'rotationHead': head_rotation
            }
            
            await self.websocket.send(json.dumps(data))
            return True
            
        except Exception as e:
            print(f"Error sending tracking data: {e}")
            self.connected = False
            return False
    
    async def send_raw(self, data_dict):
        """Send raw dictionary data"""
        if not self.websocket or not self.connected:
            return False
        
        try:
            await self.websocket.send(json.dumps(data_dict))
            return True
        except Exception as e:
            print(f"Error sending data: {e}")
            self.connected = False
            return False
    
    async def receive_loop(self):
        """Loop to receive data from Pi (run as task)"""
        while self.connected:
            try:
                message = await self.websocket.recv()
                data = json.loads(message)
                
                print(f"Pi -> PC: {message}")
                
                # Call callback if set
                if self.receive_callback:
                    self.receive_callback(data)
                    
            except websockets.exceptions.ConnectionClosed:
                print("Connection to Pi closed")
                self.connected = False
                break
            except Exception as e:
                print(f"Error receiving: {e}")
                await asyncio.sleep(0.1)
    
    def set_receive_callback(self, callback):
        """Set callback function for incoming sensor data"""
        self.receive_callback = callback


# =============================================================================
# Helper functions for simple integration
# =============================================================================

# Global client instance
_client = None
_loop = None
_receive_task = None

def init_teleop_client(pi_ip='192.168.1.100', on_receive=None):
    """
    Initialize the teleop client (call once at startup)
    
    Args:
        pi_ip: IP address of the Raspberry Pi
        on_receive: Callback function for incoming sensor data
    """
    global _client, _loop, _receive_task
    
    _client = TeleopClient(f'ws://{pi_ip}:8080')
    
    if on_receive:
        _client.set_receive_callback(on_receive)
    
    # Create event loop
    _loop = asyncio.new_event_loop()
    asyncio.set_event_loop(_loop)
    
    # Connect
    connected = _loop.run_until_complete(_client.connect())
    
    if connected:
        # Start receive loop in background
        _receive_task = _loop.create_task(_client.receive_loop())
    
    return connected

def send_tracking(thumb, index, middle, ring, pinky,
                  hand_x, hand_y, hand_z,
                  hand_pitch, hand_yaw, hand_roll,
                  head_pitch, head_yaw, head_roll):
    """
    Send tracking data to Pi (call this from your main loop)
    
    All angles in degrees, positions in whatever units you prefer
    """
    global _client, _loop
    
    if not _client or not _client.connected:
        return False
    
    finger_data = {'t': thumb, 'i': index, 'm': middle, 'r': ring, 'p': pinky}
    hand_position = {'x': hand_x, 'y': hand_y, 'z': hand_z}
    hand_rotation = {'p': hand_pitch, 'y': hand_yaw, 'r': hand_roll}
    head_rotation = {'p': head_pitch, 'y': head_yaw, 'r': head_roll}
    
    # Run async send in the event loop
    future = asyncio.run_coroutine_threadsafe(
        _client.send_tracking_data(finger_data, hand_position, hand_rotation, head_rotation),
        _loop
    )
    
    return future.result(timeout=0.1)

def process_incoming():
    """Process any incoming messages (call periodically)"""
    global _loop
    if _loop:
        _loop.run_until_complete(asyncio.sleep(0))

def cleanup_teleop():
    """Cleanup (call on shutdown)"""
    global _client, _loop, _receive_task
    
    if _receive_task:
        _receive_task.cancel()
    
    if _client and _loop:
        _loop.run_until_complete(_client.disconnect())
    
    if _loop:
        _loop.close()

