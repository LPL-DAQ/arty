"""
Fit a TC's Ranger readings against a reference (big DAQ box) and print the tc_bias_deg_c to set.

Input CSV, one row per point, same TC:
    serial,amp,ref_c,ranger_c
    K-01,amp1,22.4,23.1
ref_c = reference reading (average the DAQ readings taken before and after the Ranger one).
ranger_c = TC reading on Ranger `status`, taken with the tc_bias_deg_c given by --old-bias loaded.

Usage: python tc_fit.py points.csv [--old-bias 0]    |    python tc_fit.py --selftest
"""

import argparse
import csv
import statistics
import sys

# Outside this an offset alone can't be right across the range; needs a slope field
SLOPE_OK = (0.98, 1.02)


def fit(points: list[tuple[float, float]], old_bias: float) -> dict:
    """points = [(ref_c, ranger_c)]. Returns slope/intercept of ref vs ranger, new bias, and hold-out error."""
    refs = [r for r, _ in points]
    rangers = [g for _, g in points]
    slope, intercept = statistics.linear_regression(rangers, refs)
    offset = statistics.fmean(r - g for r, g in points)
    # Hold-out: bias from all-but-one point, error on the one left out. Honest accuracy, not fit-to-itself.
    holdout = []
    for i in range(len(points)):
        rest = points[:i] + points[i + 1 :]
        o = statistics.fmean(r - g for r, g in rest)
        holdout.append(points[i][0] - (points[i][1] + o))
    return {
        'slope': slope,
        'intercept': intercept,
        'new_bias': old_bias + offset,
        'worst_holdout_c': max(abs(e) for e in holdout),
        'slope_ok': SLOPE_OK[0] <= slope <= SLOPE_OK[1],
    }


def selftest():
    # Pure offset: Ranger reads 1.5 C high everywhere -> bias -1.5, slope 1, zero hold-out error
    r = fit([(20, 21.5), (50, 51.5), (80, 81.5)], 0.0)
    assert (
        abs(r['slope'] - 1) < 1e-9
        and abs(r['new_bias'] + 1.5) < 1e-9
        and r['worst_holdout_c'] < 1e-9
    ), r
    assert r['slope_ok']
    # Old bias is carried: readings already had +2 applied and are still 1.5 high -> 0.5
    assert abs(fit([(20, 21.5), (50, 51.5)], 2.0)['new_bias'] - 0.5) < 1e-9
    # Gain error (Ranger reads 5% low) is flagged, and hold-out shows the offset can't cover it
    r = fit([(20, 19), (50, 47.5), (80, 76)], 0.0)
    assert not r['slope_ok'] and r['worst_holdout_c'] > 1, r
    print('selftest ok')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='?')
    ap.add_argument(
        '--old-bias', type=float, default=0.0, help='tc_bias_deg_c loaded when ranger_c was read'
    )
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest:
        selftest()
        return 0
    if not a.csv:
        ap.error('csv required')

    by_tc: dict[tuple[str, str], list[tuple[float, float]]] = {}
    with open(a.csv, newline='') as fh:
        for row in csv.DictReader(fh):
            by_tc.setdefault((row['serial'], row['amp']), []).append(
                (float(row['ref_c']), float(row['ranger_c']))
            )

    for (serial, amp), pts in by_tc.items():
        if len(pts) < 2:
            print(f'{serial} / {amp}: need >= 2 points, have {len(pts)}')
            continue
        r = fit(pts, a.old_bias)
        verdict = (
            'offset only is fine'
            if r['slope_ok']
            else 'SLOPE OUT OF RANGE: offset alone is wrong somewhere, needs a slope'
        )
        print(f'{serial} / {amp}  ({len(pts)} pts)')
        print(f'  ref = {r["slope"]:.4f} * ranger + {r["intercept"]:+.2f}   -> {verdict}')
        print(f'  set tc_bias_deg_c: {r["new_bias"]:.2f}')
        print(f'  worst hold-out error: {r["worst_holdout_c"]:.2f} C  (target <= 2)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
