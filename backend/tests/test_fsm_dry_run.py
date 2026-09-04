import os
import time
import logging

os.environ.setdefault("MAVLINK20", "1")
os.environ["FSM_DRY_RUN"] = "1"
os.environ["FSM_PREFLIGHT_TIMEOUT_S"] = "2.0"

from config import config
import state
from models import GeofenceStatus
import fsm
from mavlink_bus import MavlinkBus

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

def main():
    print("--- Starting Tier 2: FSM DRY-RUN Acceptance Test ---")
    
    # Use a dummy UDP endpoint for bus just to initialize it
    bus = MavlinkBus(device="udp:127.0.0.1:14550", timeout_s=1.0)
    # We won't start the bus thread for dry-run since we just want to test logic without SITL
    # But wait, without bus thread, we won't get heartbeat.
    # We can mock state and let FSM run.
    
    # Initialize fake state
    fence = GeofenceStatus(loaded=True, armable=True, reason="DRY RUN OK")
    state.set(fence)
    
    mission = fsm.init_fsm(bus)
    mission._last_heartbeat = time.time()  # Fake healthy link
    
    mission.start()
    
    print("Waiting for FSM to transition through states...")
    try:
        while True:
            status = mission.get_status()
            print(f"Current State: {status.state} | Reason: {status.reason}")
            
            # Keep link alive
            mission._last_heartbeat = time.time()
            
            if status.state == "MISSION_COMPLETE":
                print("Test passed: Reached MISSION_COMPLETE successfully!")
                break
            elif status.state == "FAILSAFE":
                print(f"Test failed: Entered FAILSAFE. Reason: {status.reason}")
                break
                
            time.sleep(1.0)
    except KeyboardInterrupt:
        mission.abort()
        
if __name__ == "__main__":
    main()
