"""End-to-end test of RobotLink against a FAKE rosbridge websocket server (test-only; the GUI never fakes data).

Checks: real roslibpy subscribe path, JSON null/inf handling, TF-derived pose, rosapi discovery,
staleness after the server stops, and automatic recovery when it comes back.
Run:  python -m unittest tests.test_integration -v   (from gui/; needs roslibpy + websockets)
"""
import asyncio
import json
import math
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

import websockets

from jetbot_gui.config import Config
from jetbot_gui.link import CONNECTED, RobotLink
from jetbot_gui.status import OK, WARN, assess
from test_core import grid_msg, tfmsg

PORT = 19090


class FakeRosbridge:
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.server = None
        self.subscribed = []
        self.client_publishes = []              # every `publish` op the GUI sends to the (fake) robot
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()

    def start(self):
        fut = asyncio.run_coroutine_threadsafe(self._start(), self.loop)
        fut.result(5)

    async def _start(self):
        self.server = await websockets.serve(self._handler, "127.0.0.1", PORT)

    def stop(self):
        async def _stop():
            self.server.close()
            await self.server.wait_closed()
        asyncio.run_coroutine_threadsafe(_stop(), self.loop).result(5)

    async def _handler(self, ws, *_):
        pubs = {}
        async def push(topic, msg):
            await ws.send(json.dumps({"op": "publish", "topic": topic, "msg": msg}))

        async def stream():
            while True:
                t = time.time()
                for topic in list(pubs):
                    if topic == "/tf":
                        await push(topic, {"transforms": [tfmsg("map", "odom", 1.0, 2.0, 0.0),
                                                          tfmsg("odom", "base_footprint", 0.5, 0.0, math.pi / 2),
                                                          tfmsg("base_footprint", "laser_frame", 0, 0, math.pi)]})
                    elif topic == "/move_base/status":
                        await push(topic, {"status_list": [{"goal_id": {"id": "g1", "stamp": {"secs": 0, "nsecs": 0}},
                                                            "status": 3, "text": "Goal reached."}]})
                    elif topic == "/scan":
                        await push(topic, {"header": {"frame_id": "laser_frame"}, "angle_min": 0.0,
                                           "angle_increment": math.pi / 2, "range_min": 0.15, "range_max": 12.0,
                                           "ranges": [2.0, None, "Infinity", 1.0]})
                await asyncio.sleep(0.1)
        task = None
        try:
            async for raw in ws:
                m = json.loads(raw)
                if m["op"] == "subscribe":
                    self.subscribed.append(m["topic"])
                    pubs[m["topic"]] = True
                    if m["topic"] == "/map":
                        await push("/map", grid_msg(10, 10, 0.1, (0, 0), 0, {(1, 1): 100, (2, 2): 0}))
                    if task is None:
                        task = asyncio.ensure_future(stream())
                elif m["op"] == "publish":
                    self.client_publishes.append(m)
                elif m["op"] == "call_service":
                    vals = {"/rosapi/nodes": {"nodes": ["/amcl", "/move_base", "/rosbridge_websocket"]},
                            "/rosapi/topics": {"topics": ["/scan"], "types": ["sensor_msgs/LaserScan"]}}
                    await ws.send(json.dumps({"op": "service_response", "service": m["service"], "id": m.get("id"),
                                              "values": vals.get(m["service"], {}), "result": True}))
        finally:
            if task:
                task.cancel()


def wait_for(cond, timeout=8.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.1)
    return False


class IntegrationTest(unittest.TestCase):
    def test_full_cycle(self):
        cfg = Config(host="127.0.0.1", port=PORT, scan_stale_s=0.6, pose_stale_s=0.6, discovery_period_s=1.0)
        server = FakeRosbridge()
        link = RobotLink(cfg)
        try:
            link.connect("127.0.0.1", PORT)           # server is NOT up yet: must not block or raise
            time.sleep(1.0)
            s = link.snapshot()
            self.assertNotEqual(s.conn_state, CONNECTED)
            self.assertIsNone(s.map)

            server.start()
            self.assertTrue(wait_for(lambda: link.snapshot().conn_state == CONNECTED), "did not connect/reconnect")
            self.assertTrue(wait_for(lambda: link.snapshot().scan is not None and link.snapshot().map is not None
                                     and link.snapshot().nodes is not None))
            s = link.snapshot()
            a = assess(s, cfg)
            self.assertEqual(len(s.scan.points()), 2)              # None and "Infinity" dropped
            self.assertTrue(a.pose_live_in_map)
            x, y, th = s.pose_map
            self.assertAlmostEqual(x, 1.5)                          # map->odom (1,2) + odom->base (0.5,0)
            self.assertAlmostEqual(y, 2.0)
            self.assertAlmostEqual(th, math.pi / 2)
            self.assertTrue(a.scan_on_map)
            self.assertEqual(a.localizer.level, OK)
            self.assertIn("AMCL", a.localizer.text)
            self.assertIn("move_base running", a.nav.text)

            server.stop()                                           # robot / network goes away
            self.assertTrue(wait_for(lambda: not assess(link.snapshot(), cfg).scan_live, 5), "scan stayed 'live'")
            a = assess(link.snapshot(), cfg)
            self.assertFalse(a.pose_live_in_map)                    # stale pose must not be drawn as live
            self.assertNotEqual(link.snapshot().conn_state, CONNECTED)

            server = FakeRosbridge(); server.start()                # robot comes back
            self.assertTrue(wait_for(lambda: assess(link.snapshot(), cfg).scan_live, 20), "did not recover")
        finally:
            link.disconnect()
            try:
                server.stop()                       # also the second server started after the 'robot comes back' step
            except Exception:
                pass


if __name__ == "__main__":
    unittest.main()
