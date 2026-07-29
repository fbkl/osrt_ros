#!/usr/bin/env python3
"""Numeric check for whether a generated iiwa .osim model's body tree actually
chains joint offsets, or whether every body collapses to (~0,0,0).

No visualizer involved - reads each body's transform-in-ground directly at the
model's default (zero-coordinate) state via the OpenSim API, so it can't be
fooled by a viewer that isn't pushing a realized frame to the display (see
view_osim_native.py's possible missing model.getVisualizer().show(state)).

For each body: prints its ground-frame translation and distance from origin.
For each consecutive pair in EXPECTED_CHAIN: prints the actual 3D distance
between them and compares it against the known joint offset magnitude from
iiwa14.xacro. If the model tree isn't chaining offsets, these distances will
be near zero (or otherwise unrelated to the expected values) even though each
individual PhysicalOffsetFrame's translation, read from the XML, looks correct.

Usage: check_body_chain_offsets.py /path/to/model.osim
"""
import sys

import opensim

# link_i -> link_(i+1) joint origin translation magnitude, from iiwa14.xacro
EXPECTED_CHAIN = [
    ("link_0", "link_1", 0.1575),
    ("link_1", "link_2", 0.2025),
    ("link_2", "link_3", 0.2045),
    ("link_3", "link_4", 0.2155),
    ("link_4", "link_5", 0.1845),
    ("link_5", "link_6", 0.2155),
    ("link_6", "link_7", 0.0810),
]


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} model.osim", file=sys.stderr)
        return 1

    model = opensim.Model(sys.argv[1])
    state = model.initSystem()
    model.realizePosition(state)

    body_set = model.getBodySet()
    positions = {}

    print(f"{'body':<12} {'x':>10} {'y':>10} {'z':>10} {'|p| from ground':>18}   rotation (body-fixed XYZ, deg)")
    print("-" * 100)
    for i in range(body_set.getSize()):
        body = body_set.get(i)
        name = body.getName()
        transform = body.getTransformInGround(state)
        p = transform.p()
        positions[name] = p
        norm = (p[0] ** 2 + p[1] ** 2 + p[2] ** 2) ** 0.5
        angles = transform.R().convertRotationToBodyFixedXYZ()
        deg = [angles[0] * 180.0 / 3.14159265358979,
               angles[1] * 180.0 / 3.14159265358979,
               angles[2] * 180.0 / 3.14159265358979]
        print(f"{name:<12} {p[0]:>10.4f} {p[1]:>10.4f} {p[2]:>10.4f} {norm:>18.4f}   "
              f"[{deg[0]:7.2f}, {deg[1]:7.2f}, {deg[2]:7.2f}]")

    print()
    print(f"{'pair':<20} {'actual dist':>12} {'expected':>10} {'match?':>8}")
    print("-" * 56)
    all_match = True
    for a, b, expected in EXPECTED_CHAIN:
        if a not in positions or b not in positions:
            print(f"{a}->{b:<15} MISSING BODY (check names above)")
            all_match = False
            continue
        pa, pb = positions[a], positions[b]
        dist = ((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2 + (pa[2] - pb[2]) ** 2) ** 0.5
        ok = abs(dist - expected) < 0.01
        all_match &= ok
        print(f"{a}->{b:<15} {dist:>12.4f} {expected:>10.4f} {'OK' if ok else 'MISMATCH':>8}")

    # Step MAGNITUDE alone can't detect a folded-back chain: a wrong rotation can
    # misdirect a correctly-sized translation without changing its length. At an
    # all-zero-joint "candle" pose the arm should extend outward monotonically, so
    # check that |p| from ground actually increases body-to-body down the chain.
    ordered_norms = [
        (name, (p[0] ** 2 + p[1] ** 2 + p[2] ** 2) ** 0.5)
        for name, p in ((a, positions[a]) for a, _, _ in EXPECTED_CHAIN if a in positions)
    ]
    _, last_name, last_expected = EXPECTED_CHAIN[-1]
    if last_name in positions:
        p = positions[last_name]
        ordered_norms.append((last_name, (p[0] ** 2 + p[1] ** 2 + p[2] ** 2) ** 0.5))
    monotonic = all(b[1] > a[1] + 1e-6 for a, b in zip(ordered_norms, ordered_norms[1:]))

    print()
    if all_match and monotonic:
        print("Step magnitudes match AND distance from ground increases monotonically:")
        print("the tree IS chaining offsets correctly - bug is elsewhere (viewer? mesh?).")
    elif all_match and not monotonic:
        print("Step magnitudes match, but distance from ground does NOT increase")
        print("monotonically down the chain - the arm is folding back on itself instead")
        print("of extending outward. Translation magnitude is preserved, so this is an")
        print("ORIENTATION composition bug, not a translation bug - check how each joint's")
        print("rotation is being computed/applied in urdf_to_osim.cpp.")
    else:
        print("Distances do NOT match expected offsets:")
        print("the model tree is NOT chaining joint offsets correctly - bug is upstream")
        print("in how urdf_to_osim.cpp builds the joints/bodies.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
