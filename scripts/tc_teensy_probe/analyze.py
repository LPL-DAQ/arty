"""
Find flat sections in a Teensy TC log (from log.sh) and print their averages.
Usage: python3 analyze.py logs/tc_XXXX.txt [--window 10] [--tol 0.3]
A section is "flat" when the nist reading stays within --tol C for at least --window seconds.
Then compare each flat average with its reference (boil 99.8 at ~70 m, ice 0) by hand or with scripts/tc_fit.py.
"""

import argparse
import re
import statistics as st

LINE = re.compile(
    r'([\d.]+) counts=\s*([\d.]+)\s+V=([\d.]+)\s+simple=\s*([-\d.]+) C\s+nist=\s*([-\d.]+)'
)


def flats(rows, window, tol):
    out, i = [], 0
    while i < len(rows):
        j = i
        while j + 1 < len(rows) and abs(rows[j + 1][2] - rows[i][2]) <= tol:
            j += 1
        if rows[j][0] - rows[i][0] >= window:
            seg = [r[2] for r in rows[i : j + 1]]
            out.append(
                (
                    rows[i][0],
                    rows[j][0],
                    st.fmean(seg),
                    st.pstdev(seg),
                    st.fmean(r[1] for r in rows[i : j + 1]),
                )
            )
            i = j + 1
        else:
            i += 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--window', type=float, default=10)
    ap.add_argument('--tol', type=float, default=0.3)
    a = ap.parse_args()
    with open(a.log) as fh:
        rows = [(float(m[1]), float(m[3]), float(m[5])) for m in map(LINE.match, fh) if m]
    if not rows:
        print('no readings in log')
        return 1
    t0 = rows[0][0]
    print(
        f'{len(rows)} readings over {rows[-1][0] - t0:.0f} s, range {min(r[2] for r in rows):.2f}..{max(r[2] for r in rows):.2f} C'
    )
    for s, e, mean, sd, v in flats(rows, a.window, a.tol):
        print(
            f'  flat {s - t0:6.0f}-{e - t0:6.0f} s  ({e - s:4.0f} s)  nist={mean:7.2f} C  sd={sd:.2f}  V={v:.4f}'
        )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
