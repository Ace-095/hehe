"""
Unit and streak edge-case test suite for QR perception pipeline (Phase 7).

Run from backend/ directory:
    ./.venv/bin/python3 tests/test_qr_pipeline.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from camera_source import SyntheticQRSource
from models import BBox
from qr_pipeline import ConsensusBuffer, PlaceholderColorShapeDetector, decode_qr


def test_consensus_buffer_streak():
    buf = ConsensusBuffer(required_consecutive=3)

    # Frame 1
    newly_confirmed, res = buf.update("42", BBox(x=10, y=10, w=50, h=50))
    assert not newly_confirmed and not res.confirmed and res.streak == 1

    # Frame 2
    newly_confirmed, res = buf.update("42", BBox(x=10, y=10, w=50, h=50))
    assert not newly_confirmed and not res.confirmed and res.streak == 2

    # Frame 3 -> Should confirm
    newly_confirmed, res = buf.update("42", BBox(x=10, y=10, w=50, h=50))
    assert newly_confirmed and res.confirmed and res.streak == 3 and res.payload == "42"

    print("PASS: ConsensusBuffer streak reaching N=3 confirmed payload.")


def test_consensus_buffer_corrupted_frame_edge_case():
    buf = ConsensusBuffer(required_consecutive=3)

    # Frame 1 & 2 valid
    buf.update("42", BBox(x=10, y=10, w=50, h=50))
    _, res2 = buf.update("42", BBox(x=10, y=10, w=50, h=50))
    assert res2.streak == 2 and not res2.confirmed

    # Frame 3 corrupted (None decode) -> Must NOT confirm or false-trigger
    newly_confirmed, res3 = buf.update(None, None)
    assert not newly_confirmed and not res3.confirmed and res3.streak == 0

    # Frames 4, 5, 6 valid decodes
    buf.update("42", BBox(x=10, y=10, w=50, h=50))
    buf.update("42", BBox(x=10, y=10, w=50, h=50))
    newly_confirmed, res6 = buf.update("42", BBox(x=10, y=10, w=50, h=50))

    assert newly_confirmed and res6.confirmed and res6.streak == 3 and res6.payload == "42"
    print("PASS: Corrupted frame edge case correctly reset streak without false positive trigger.")


def test_consensus_buffer_mismatch_reset():
    buf = ConsensusBuffer(required_consecutive=3)

    buf.update("TARGET_A", BBox(x=10, y=10, w=50, h=50))
    buf.update("TARGET_A", BBox(x=10, y=10, w=50, h=50))

    # Mismatched payload
    _, res_b = buf.update("TARGET_B", BBox(x=10, y=10, w=50, h=50))
    assert res_b.streak == 1 and not res_b.confirmed

    buf.update("TARGET_B", BBox(x=10, y=10, w=50, h=50))
    newly_confirmed, res_b3 = buf.update("TARGET_B", BBox(x=10, y=10, w=50, h=50))

    assert newly_confirmed and res_b3.confirmed and res_b3.payload == "TARGET_B"
    print("PASS: Mismatched payload cleanly reset streak and required 3 matching frames.")


def test_synthetic_qr_source_end_to_end():
    source = SyntheticQRSource(payload="SYNTHETIC_TARGET_99", fps=30.0)
    source.start()

    detector = PlaceholderColorShapeDetector()
    buf = ConsensusBuffer(required_consecutive=3)

    confirmed_payload = None

    try:
        for _ in range(15):
            frame = source.get_frame()
            assert frame is not None and frame.shape == (480, 640, 3)

            bbox = detector.detect_cache(frame)
            assert bbox is not None, "Detector failed to return candidate BBox"

            payload = decode_qr(frame, bbox)
            if payload:
                newly_confirmed, res = buf.update(payload, bbox)
                if newly_confirmed:
                    confirmed_payload = res.payload
                    break

        assert confirmed_payload == "SYNTHETIC_TARGET_99", f"Expected 'SYNTHETIC_TARGET_99', got '{confirmed_payload}'"
        print(f"PASS: End-to-end SyntheticQRSource decoding reached consensus for '{confirmed_payload}'.")

    finally:
        source.stop()


def main():
    failures = []
    tests = [
        ("test_consensus_buffer_streak", test_consensus_buffer_streak),
        ("test_consensus_buffer_corrupted_frame_edge_case", test_consensus_buffer_corrupted_frame_edge_case),
        ("test_consensus_buffer_mismatch_reset", test_consensus_buffer_mismatch_reset),
        ("test_synthetic_qr_source_end_to_end", test_synthetic_qr_source_end_to_end),
    ]

    for name, fn in tests:
        try:
            fn()
        except Exception as e:
            print(f"FAIL  {name}: {e}")
            import traceback
            traceback.print_exc()
            failures.append(name)

    if failures:
        print(f"\n{len(failures)} test(s) FAILED: {failures}")
        sys.exit(1)

    print("\nAll QR Perception Pipeline unit & streak edge-case tests passed!")


if __name__ == "__main__":
    main()
