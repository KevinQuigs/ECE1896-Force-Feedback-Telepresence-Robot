"""
Python WebSocket Client for Telepresence Robot
Connects to Pi WebSocket server
-Sends tracking data from Unity/Glove ESP32 to Pi
-Receives sensor data (force feedback) from Pi
"""

import asyncio
import websockets
import json
import threading
from queue import Queue

# Configuration
PI_WEBSOCKET_URL = 'ws://192.168.1.60:8080'  

class TeleopClient:
    def __init__(self, pi_url=PI_WEBSOCKET_URL):
        self.pi_url = pi_url
        self.websocket = None
        self.connected = False
        self.receive_callback = None
        self.send_queue = Queue()
        self.loop = None
        self.thread = None
        self._running = False
        
    def start(self):
        """Start the async event loop in a separate thread"""
        self._running = True
        self.thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.thread.start()
        
        # Wait for connection
        import time
        timeout = 5
        start = time.time()
        while not self.connected and time.time() - start < timeout:
            time.sleep(0.1)
        
        return self.connected
    
    def _run_async_loop(self):
        """Run the asyncio event loop in this thread"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        try:
            self.loop.run_until_complete(self._async_main())
        except Exception as e:
            print(f"Async loop error: {e}")
        finally:
            self.loop.close()
    
    async def _async_main(self):
        """Main async function that handles connection and loops"""
        # Connect
        if not await self.connect():
            return
        
        # Run both send and receive loops concurrently
        await asyncio.gather(
            self._send_loop(),
            self._receive_loop(),
            return_exceptions=True
        )
    
    async def connect(self):
        """Connect to Pi WebSocket server"""
        try:
            self.websocket = await websockets.connect(
                self.pi_url,
                ping_interval=20,  # Send ping every 20 seconds
                ping_timeout=10    # Wait 10 seconds for pong
            )
            self.connected = True
            print(f"✓ Connected to Pi at {self.pi_url}")
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            self.connected = False
            return False
    
    async def _send_loop(self):
        """Loop to send queued messages"""
        while self._running and self.connected:
            try:
                # Check if there's data to send (non-blocking)
                if not self.send_queue.empty():
                    data = self.send_queue.get_nowait()
                    
                    if self.websocket and self.connected:
                        await self.websocket.send(json.dumps(data))
                else:
                    # Small sleep to prevent busy waiting
                    await asyncio.sleep(0.001)
                    
            except Exception as e:
                print(f"Error in send loop: {e}")
                self.connected = False
                break
    
    async def _receive_loop(self):
        """Loop to receive data from Pi"""
        while self._running and self.connected:
            try:
                message = await asyncio.wait_for(self.websocket.recv(), timeout=1.0)
                data = json.loads(message)
                
                print(f"Pi -> PC: {message}")
                
                # Call callback if set
                if self.receive_callback:
                    self.receive_callback(data)
                    
            except asyncio.TimeoutError:
                # No message received, continue
                continue
            except websockets.exceptions.ConnectionClosed:
                print("Connection to Pi closed")
                self.connected = False
                break
            except Exception as e:
                print(f"Error receiving: {e}")
                self.connected = False
                break
    
    def send_tracking_data(self, finger_data, hand_position, hand_rotation, head_rotation):
        """
        Queue tracking data to be sent to Pi
        
        Args:
            finger_data: dict with keys t, i, m, r, p (thumb, index, middle, ring, pinky)
            hand_position: dict with keys x, y, z
            hand_rotation: dict with keys p, y, r (pitch, yaw, roll)
            head_rotation: dict with keys p, y, r (pitch, yaw, roll)
        """
        if not self.connected:
            return False
        
        try:
            data = {
                'type': 'tracking',
                'rotationFinger': finger_data,
                'positionHand': hand_position,
                'rotationHand': hand_rotation,
                'rotationHead': head_rotation
            }
            
            self.send_queue.put(data)
            return True
            
        except Exception as e:
            print(f"Error queueing tracking data: {e}")
            return False
    
    def set_receive_callback(self, callback):
        """Set callback function for incoming sensor data"""
        self.receive_callback = callback
    
    def stop(self):
        """Stop the client and cleanup"""
        self._running = False
        self.connected = False
        
        if self.websocket and self.loop:
            # Schedule websocket close in the event loop
            asyncio.run_coroutine_threadsafe(self.websocket.close(), self.loop)
        
        if self.thread:
            self.thread.join(timeout=2)
        
        print("Teleop client stopped")


# =============================================================================
# Helper functions for simple integration
# =============================================================================

# Global client instance
_client = None

def init_teleop_client(pi_ip='192.168.1.100', on_receive=None):
    """
    Initialize the teleop client (call once at startup)
    
    Args:
        pi_ip: IP address of the Raspberry Pi
        on_receive: Callback function for incoming sensor data
    """
    global _client
    
    _client = TeleopClient(f'ws://{pi_ip}:8080')
    
    if on_receive:
        _client.set_receive_callback(on_receive)
    
    # Start the client (this will run asyncio in a background thread)
    return _client.start()

def send_tracking(thumb, index, middle, ring, pinky,
                  hand_x, hand_y, hand_z,
                  hand_pitch, hand_yaw, hand_roll,
                  head_pitch, head_yaw, head_roll):
    """
    Send tracking data to Pi (call this from your main loop)
    
    All angles in degrees, positions in whatever units you prefer
    """
    global _client
    
    if not _client or not _client.connected:
        return False
    
    finger_data = {'t': thumb, 'i': index, 'm': middle, 'r': ring, 'p': pinky}
    hand_position = {'x': hand_x, 'y': hand_y, 'z': hand_z}
    hand_rotation = {'p': hand_pitch, 'y': hand_yaw, 'r': hand_roll}
    head_rotation = {'p': head_pitch, 'y': head_yaw, 'r': head_roll}
    
    return _client.send_tracking_data(finger_data, hand_position, hand_rotation, head_rotation)

def process_incoming():
    """Process any incoming messages (call periodically) - No longer needed but kept for compatibility"""
    pass

def cleanup_teleop():
    """Cleanup (call on shutdown)"""
    global _client
    
    if _client:
        _client.stop()
        _client = None