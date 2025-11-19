/**
 * Node.js WebSocket Bridge Server for Telepresence Robot
 * Bridges between browser (WebSocket) and ESP32 (Serial)
 */

const WebSocket = require('ws');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

// Configuration
const WEBSOCKET_PORT = 8080;
const SERIAL_PORT = '/dev/ttyUSB0'; 
const SERIAL_BAUD = 115200;

// Track connected clients
const clients = new Set();

// Serial connection
let serialPort = null;
let parser = null;

// Setup serial connection
function setupSerial() {
    try {
        serialPort = new SerialPort({
            path: SERIAL_PORT,
            baudRate: SERIAL_BAUD
        });

        parser = serialPort.pipe(new ReadlineParser({ delimiter: '\n' }));

        serialPort.on('open', () => {
            console.log(`✓ Serial connected: ${SERIAL_PORT} @ ${SERIAL_BAUD} baud`);
        });

        serialPort.on('error', (err) => {
            console.error(`✗ Serial error: ${err.message}`);
        });

        // Handle incoming data from ESP32
        parser.on('data', (line) => {
            line = line.trim();
            if (line) {
                console.log(`ESP32 -> Pi: ${line}`);
                
                // Parse the data
                const sensorData = parseESP32Data(line);
                
                if (sensorData && clients.size > 0) {
                    const message = JSON.stringify(sensorData);
                    console.log(`Pi -> Browser (${clients.size} clients): ${message}`);
                    
                    // Broadcast to all connected clients
                    clients.forEach((client) => {
                        if (client.readyState === WebSocket.OPEN) {
                            client.send(message);
                        }
                    });
                }
            }
        });

    } catch (err) {
        console.error(`✗ Failed to setup serial: ${err.message}`);
        console.log('  Check: ls /dev/ttyUSB* or ls /dev/ttyACM*');
    }
}

// Parse ESP32 data
function parseESP32Data(line) {
    const data = {
        type: 'sensor',
        timestamp: new Date().toISOString()
    };

    try {
        // Simple analog test: "ANALOG:1234"
        if (line.startsWith('ANALOG:')) {
            const value = parseFloat(line.replace('ANALOG:', '').trim());
            data.analog = value;
            console.log(`Parsed analog value: ${value}`);
            return data;
        }
        
        // Full force sensors: "FORCE:12.5,34.6,56.7,78.9,90.1"
        if (line.startsWith('FORCE:')) {
            const values = line.replace('FORCE:', '').trim().split(',');
            if (values.length >= 5) {
                data.thumb = parseFloat(values[0]);
                data.index = parseFloat(values[1]);
                data.middle = parseFloat(values[2]);
                data.ring = parseFloat(values[3]);
                data.pinky = parseFloat(values[4]);
                console.log(`Parsed force sensors: T=${data.thumb}, I=${data.index}, M=${data.middle}, R=${data.ring}, P=${data.pinky}`);
                return data;
            }
        }

        // Legacy format: "THUMB:12.5,INDEX:34.6,..."
        if (line.includes(':')) {
            const parts = line.split(',');
            parts.forEach((part) => {
                if (part.includes(':')) {
                    const [key, value] = part.split(':');
                    const keyLower = key.trim().toLowerCase();
                    const val = parseFloat(value.trim());
                    
                    if (keyLower === 'thumb') data.thumb = val;
                    else if (keyLower === 'index') data.index = val;
                    else if (keyLower === 'middle') data.middle = val;
                    else if (keyLower === 'ring') data.ring = val;
                    else if (keyLower === 'pinky') data.pinky = val;
                }
            });
            
            if (Object.keys(data).length > 2) {
                return data;
            }
        }

    } catch (err) {
        console.error(`Error parsing ESP32 data: ${line} - ${err.message}`);
    }

    return null;
}

// Handle incoming WebSocket messages
function handleClientMessage(message) {
    try {
        const data = JSON.parse(message);
        const msgType = data.type || 'unknown';

        if (msgType === 'tracking') {
            
	    //console.log('Head rotation received: ', data.rotationHead);
	    
	    // Extract tracking data
            const rotFinger = data.rotationFinger || {};
            const posHand = data.positionHand || {};
            const rotHand = data.rotationHand || {};
            const rotHead = data.rotationHead || {};

            // Pack data for ESP32 (binary format)
            // Format: [msg_type(1)][thumb(4)][index(4)][middle(4)][ring(4)][pinky(4)]
            //         [hand_x(4)][hand_y(4)][hand_z(4)][hand_p(4)][hand_y(4)][hand_r(4)]
            //         [head_p(4)][head_y(4)][head_r(4)]
            const buffer = Buffer.alloc(1 + 14 * 4); // 1 byte + 14 floats
            let offset = 0;
	    
	    //console.log('Packing values:', {
		//fingers: [rotFinger.t, rotFinger.i, rotFinger.m, rotFinger.r, rotFinger.p],
		//handPos: [posHand.x, posHand.y, posHand.z],
		//handRot: [rotHand.p, rotHand.y, rotHand.r],
		//headRot: [rotHead.p, rotHead.y, rotHead.r]
	    //});
            buffer.writeUInt8(0x01, offset); offset += 1;  // Message type
            
            // Fingers
            buffer.writeFloatLE(rotFinger.t || 0, offset); offset += 4;
            buffer.writeFloatLE(rotFinger.i || 0, offset); offset += 4;
            buffer.writeFloatLE(rotFinger.m || 0, offset); offset += 4;
            buffer.writeFloatLE(rotFinger.r || 0, offset); offset += 4;
            buffer.writeFloatLE(rotFinger.p || 0, offset); offset += 4;
            
            // Hand position
            buffer.writeFloatLE(posHand.x || 0, offset); offset += 4;
            buffer.writeFloatLE(posHand.y || 0, offset); offset += 4;
            buffer.writeFloatLE(posHand.z || 0, offset); offset += 4;
            
            // Hand rotation
            buffer.writeFloatLE(rotHand.p || 0, offset); offset += 4;
            buffer.writeFloatLE(rotHand.y || 0, offset); offset += 4;
            buffer.writeFloatLE(rotHand.r || 0, offset); offset += 4;
            
            // Head rotation
            buffer.writeFloatLE(rotHead.p || 0, offset); offset += 4;
            buffer.writeFloatLE(rotHead.y || 0, offset); offset += 4;
            buffer.writeFloatLE(rotHead.r || 0, offset); offset += 4;

            // Send to ESP32
            if (serialPort && serialPort.isOpen) {
                serialPort.write(buffer);
            }

	    

        } else if (msgType === 'command') {
            const command = data.command || '';
            if (serialPort && serialPort.isOpen) {
                serialPort.write(`CMD:${command}\n`);
            }

        } else if (msgType === 'ping') {
            return {
                type: 'pong',
                timestamp: new Date().toISOString()
            };
        }

    } catch (err) {
        console.error(`Error handling message: ${err.message}`);
    }

    return null;
}

// Main function
function main() {
    // Setup serial connection
    setupSerial();

    // Create WebSocket server
    const wss = new WebSocket.Server({ port: WEBSOCKET_PORT });

    console.log('\n' + '='.repeat(50));
    console.log('WebSocket Bridge Server Running (Node.js)');
    console.log('='.repeat(50));
    console.log(`WebSocket: ws://0.0.0.0:${WEBSOCKET_PORT}`);
    console.log(`Serial: ${SERIAL_PORT} @ ${SERIAL_BAUD} baud`);
    console.log('='.repeat(50));
    console.log('\nExpected ESP32 formats:');
    console.log('  Simple test:  ANALOG:1234');
    console.log('  Force sensors: FORCE:12.5,34.6,56.7,78.9,90.1');
    console.log('  Legacy format: THUMB:12.5,INDEX:34.6,...');
    console.log('='.repeat(50) + '\n');

    // Handle new connections
    wss.on('connection', (ws, req) => {
        const clientAddress = req.socket.remoteAddress;
        console.log(`[WebSocket] Client connected from ${clientAddress}`);
        clients.add(ws);
        console.log(`Total clients: ${clients.size}`);

        // Handle messages from client
        ws.on('message', (message) => {
            const msgStr = message.toString();
            console.log(`[WebSocket] Received: ${msgStr.substring(0, 100)}...`);
            
            const response = handleClientMessage(msgStr);
            if (response) {
                ws.send(JSON.stringify(response));
            }
        });

        // Handle client disconnect
        ws.on('close', () => {
            clients.delete(ws);
            console.log(`[WebSocket] Client disconnected. Total clients: ${clients.size}`);
        });

        // Handle errors
        ws.on('error', (err) => {
            console.error(`[WebSocket] Error: ${err.message}`);
            clients.delete(ws);
        });
    });

    // Handle server errors
    wss.on('error', (err) => {
        console.error(`[WebSocket Server] Error: ${err.message}`);
    });
}

// Run the server
main();
