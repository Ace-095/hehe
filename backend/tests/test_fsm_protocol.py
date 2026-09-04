import time
import pytest
from unittest.mock import MagicMock, patch

from config import config
from models import GeofenceStatus
import fsm
import state

@pytest.fixture
def mock_bus():
    bus = MagicMock()
    bus.conn = MagicMock()
    bus.conn.target_system = 1
    bus.conn.target_component = 1
    return bus

def test_fsm_initial_state(mock_bus):
    mission_fsm = fsm.MissionFSM(mock_bus)
    status = mission_fsm.get_status()
    assert status.state == "INIT"

def test_fsm_abort(mock_bus):
    mission_fsm = fsm.MissionFSM(mock_bus)
    status = mission_fsm.abort()
    assert status.state == "FAILSAFE"

@patch('fsm.state.get')
def test_fsm_pre_flight_check_fail_no_fence(mock_state_get, mock_bus):
    mock_state_get.return_value = None
    config.FSM_PREFLIGHT_TIMEOUT_S = 0.5
    
    mission_fsm = fsm.MissionFSM(mock_bus)
    mission_fsm.start()
    
    time.sleep(1.0)
    status = mission_fsm.get_status()
    
    assert status.state == "FAILSAFE"
    assert "timeout" in status.reason.lower()

@patch('fsm.state.get')
def test_fsm_pre_flight_check_pass(mock_state_get, mock_bus):
    fence_status = GeofenceStatus(loaded=True, armable=True, reason="ok")
    mock_state_get.return_value = fence_status
    
    mission_fsm = fsm.MissionFSM(mock_bus)
    # Simulate a very recent heartbeat
    mission_fsm._last_heartbeat = time.time()
    
    mission_fsm.start()
    time.sleep(0.5)
    
    status = mission_fsm.get_status()
    assert status.state in ["ARM", "TAKEOFF"]

