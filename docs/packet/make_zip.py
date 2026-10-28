"""Builds build/BioHORS_team_packet.zip, the files that go with docs/BioHORS_Team_Packet.pdf.

Run from the repo root after the tests (they write the logs):  python docs/packet/make_zip.py
Videos are included when build/video/ has them (they are screen recordings, not made by the tests).
"""
import glob
import os
import subprocess
import sys
import zipfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
OUT = os.path.join(ROOT, 'build', 'BioHORS_team_packet.zip')
VIDEOS = ('biohors-auto-scoring.mp4', 'biohors-auto-physics.mp4', 'biohors-auto-garden-closer.mp4')


def main():
    os.chdir(ROOT)
    logs = sorted(glob.glob('TeamCode/build/sim/*.rlog') + glob.glob('TeamCode/build/sim/*.trace.json'))
    if not logs:
        raise SystemExit('no logs in TeamCode/build/sim: run the tests first (Section 2 of the packet)')
    entries = [('docs/BioHORS_Team_Packet.pdf', 'BioHORS_Team_Packet.pdf'), ('zenith.json', 'zenith.json')]
    entries += [(f, 'logs/' + os.path.basename(f)) for f in logs]
    entries += [(os.path.join('build/video', v), 'videos/' + v) for v in VIDEOS
                if os.path.isfile(os.path.join('build/video', v))]
    entries += [(f, 'screenshots/' + os.path.basename(f)) for f in sorted(glob.glob('docs/packet/img/*.png'))]
    for top in ('autos', 'advantagescope'):
        for dp, _, fs in os.walk(top):
            entries += [(os.path.join(dp, f), os.path.join(dp, f).replace(os.sep, '/')) for f in fs]
    entries += [(f, f) for f in ('docs/TUNING.md', 'docs/ROBOT_CAD.md', 'docs/two-tip.png', 'docs/two-tip-blue.png')
                if os.path.isfile(f)]
    open_values = subprocess.run([sys.executable, 'docs/packet/open_values.py'], capture_output=True,
                                 text=True, check=True).stdout

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with zipfile.ZipFile(OUT, 'w', zipfile.ZIP_DEFLATED) as z:
        for src, arc in entries:
            z.write(src, arc)
        z.writestr('open-values.txt', open_values)
    print('wrote', os.path.relpath(OUT), '(%d files, %.1f MB)' % (len(entries), os.path.getsize(OUT) / 1e6))


if __name__ == '__main__':
    main()
