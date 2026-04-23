import asyncio
import json
import websockets

async def test_toggle():
    uri = "ws://localhost:8000/ws"
    async with websockets.connect(uri) as websocket:
        # Wait for initial state
        msg = await websocket.recv()
        print(f"Initial state: {msg}")
        
        # Toggle PBV101
        command = {"action": "TOGGLE_VALVE", "name": "PBV101"}
        print(f"Sending: {command}")
        await websocket.send(json.dumps(command))
        
        # Wait for update
        resp = await websocket.recv()
        print(f"Response: {resp}")

asyncio.run(test_toggle())
