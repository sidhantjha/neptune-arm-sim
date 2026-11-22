"""Repeatable headless M4 autonomous pick-and-place integration checks."""

import math

from src.pick_and_place.demo import run_episode


TEST_MASSES = (0.03, 0.05, 0.08)
REPEATS = 2
MAX_PLACEMENT_ERROR = 0.04  # meters

EXPECTED_STATES = [
    "HOME", "OPEN", "APPROACH", "ALIGN", "GRASP",
    "LIFT", "TRANSFER", "PLACE", "RELEASE",
    "RETREAT", "SUCCESS",
]


def main():
    outcomes = []

    print("\n=== M4 PICK-AND-PLACE VALIDATION ===\n")

    for mass in TEST_MASSES:
        for trial in range(1, REPEATS + 1):
            try:
                success, error, history = run_episode(mass=mass, gui=False)
                passed = (
                    success
                    and history == EXPECTED_STATES
                    and math.isfinite(error)
                    and error < MAX_PLACEMENT_ERROR
                )
            except Exception as exc:
                passed = False
                error = float("nan")
                history = ["ERROR"]
                print(f"Trial exception: {exc!r}")

            outcomes.append(passed)

            print(
                f"Mass={mass:.3f} kg | Trial={trial} "
                f"| Error={error:.4f} m "
                f"| Final={history[-1]} "
                f"| {'PASS' if passed else 'FAIL'}\n"
            )

    passed_count = sum(outcomes)
    print(f"=== M4 VALIDATION: {passed_count}/{len(outcomes)} PASSED ===")

    if not all(outcomes):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
