#!/usr/bin/env python3
"""Re-measure the venue from the auto-aim recordings in clips/.

    python3 measure_video.py            # writes measurements.json and frames/

Needs ffmpeg/ffprobe on PATH, numpy and opencv-python. Takes about a minute.

Recordings: Hikrobot MV-CS016-10UC, 1440 x 1080, exposure 4000 us, gain 8 dB, Bayer
demosaic without gamma or white balance (RMCS / rmcs_auto_aim_v2), transcoded to H.264
yuv420p. Decoded 8-bit values (DN) are therefore linear in radiance. Only ratios measured
within one frame (or one clip) are used downstream, so the exposure cancels out.

ROIs are (x0, y0, x1, y1) in full-resolution pixels; they were picked by eye on the frames
written to frames/ (see README.md for what each one shows).
"""
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
CLIPS = HERE / 'clips'
FRAMES = HERE / 'frames'
W, H = 1440, 1080
FIELD = [f'2026-08-05_14-{t}' for t in
         ('41-19', '41-28', '41-44', '41-49', '41-57', '42-29', '42-34', '43-37', '44-05', '44-20')]

# Reference frames: clip suffix -> (start second, frames averaged). The camera is close to
# static over these spans, averaging removes most of the noise.
REF = {'14-41-28': (3.5, 6), '14-42-34': (1.5, 6), '14-43-37': (5.5, 1),
       '14-44-05': (5.5, 6), '14-44-20': (2.5, 6), '14-41-49': (4.0, 1)}


def clip(suffix):
    return CLIPS / f'2026-08-05_{suffix}.mp4'


def decode(path, ss=None, n=None, fps=None):
    cmd = ['ffmpeg', '-v', 'error']
    if ss is not None:
        cmd += ['-ss', str(ss)]
    cmd += ['-i', str(path)]
    if fps:
        cmd += ['-vf', f'fps={fps}']
    if n:
        cmd += ['-frames:v', str(n)]
    raw = subprocess.run(cmd + ['-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'],
                         capture_output=True, check=True).stdout
    return np.frombuffer(raw, np.uint8).reshape(-1, H, W, 3)


def probe(path):
    out = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
                          'stream=width,height,r_frame_rate:format=duration', '-of', 'json', str(path)],
                         capture_output=True, check=True, text=True).stdout
    j = json.loads(out)
    s = j['streams'][0]
    num, den = map(int, s['r_frame_rate'].split('/'))
    return {'width': s['width'], 'height': s['height'], 'fps': num / den,
            'duration_s': round(float(j['format']['duration']), 3)}


def category(name):
    if name in FIELD:
        return 'field'
    return 'pit' if name.startswith('2026-08-05_15') else 'off-site'


def brighten(img):
    """Display only: gamma 1/2.2 so the dark linear frames are visible."""
    return np.clip(255 * (img.astype(float) / 255) ** 0.45, 0, 255).astype(np.uint8)


def roi(img, box):
    x0, y0, x1, y1 = box
    return img[y0:y1, x0:x1].reshape(-1, 3).astype(float)


def rgb(v):
    return [round(float(x), 2) for x in v]


def main():
    FRAMES.mkdir(exist_ok=True)
    result = {'camera': {'model': 'Hikrobot MV-CS016-10UC (Sony IMX273)', 'exposure_us': 4000,
                         'gain_db': 8, 'framerate': 120, 'gamma': None, 'white_balance': 'unknown',
                         'source': 'RMCS rmcs_bringup/config/deformable-infantry-omni.yaml'},
              'clips': {}}

    # Inventory and a thumbnail sheet of all clips.
    thumbs = []
    for p in sorted(CLIPS.glob('*.mp4')):
        info = probe(p) | {'category': category(p.stem)}
        result['clips'][p.stem] = info
        f = decode(p, ss=info['duration_s'] / 2, n=1)[0]
        thumbs.append(cv2.resize(brighten(f), (288, 216)))
    while len(thumbs) % 5:
        thumbs.append(np.zeros_like(thumbs[0]))
    sheet = np.vstack([np.hstack(thumbs[i:i + 5]) for i in range(0, len(thumbs), 5)])
    cv2.imwrite(str(FRAMES / 'all_clips_sheet.jpg'), sheet[..., ::-1])

    # Reference frames.
    ref = {}
    for k, (ss, n) in REF.items():
        a = decode(clip(k), ss=ss, n=n).astype(float).mean(0)
        ref[k] = a
        cv2.imwrite(str(FRAMES / f'{k}_linear.png'), np.clip(a, 0, 255).astype(np.uint8)[..., ::-1])
        cv2.imwrite(str(FRAMES / f'{k}_bright.jpg'), brighten(a)[..., ::-1])

    m = {}
    # 1. White floor marking vs floor next to it (14-44-20, marking ~1.5 m away). Single
    #    frames (the camera drifts ~1 px/frame, averaging would smear the thin marking).
    #    Marking = interior of the G > 35 mask (eroded 2 px), floor = 4-20 px band around it;
    #    medians, so noise does not bias either side.
    whites, floors_ = [], []
    for ss in (2.0, 2.5):
        r = decode(clip('14-44-20'), ss=ss, n=1)[0][820:1000, 1100:1440].astype(float)
        mk = (cv2.GaussianBlur(r[..., 1], (0, 0), 1.5) > 35).astype(np.uint8)
        inner = cv2.erode(mk, np.ones((5, 5), np.uint8)) > 0
        band = (cv2.dilate(mk, np.ones((41, 41), np.uint8)) > 0) & (cv2.dilate(mk, np.ones((9, 9), np.uint8)) == 0)
        whites.append(np.median(r[inner], 0))
        floors_.append(np.median(r[band], 0))
    m['white_marking_dn'] = rgb(np.mean(whites, 0))
    m['floor_next_to_marking_dn'] = rgb(np.mean(floors_, 0))

    # 2. Floor in several clips.
    floors = {'14-41-28': (300, 950, 1100, 1060), '14-42-34': (100, 880, 600, 960),
              '14-44-05': (600, 850, 1100, 1000), '14-44-20': (100, 700, 1000, 900)}
    m['floor_dn'] = {k: rgb(np.median(roi(ref[k], b), 0)) for k, b in floors.items()}

    # 3. Walls / structures, with the floor of the same frame.
    walls = {'14-41-28 wall behind robot': ('14-41-28', (900, 400, 1250, 700)),
             '14-42-34 pillar face': ('14-42-34', (800, 300, 1300, 850)),
             '14-44-05 block side': ('14-44-05', (50, 600, 300, 1000)),
             '14-44-20 ramp side': ('14-44-20', (150, 420, 700, 600))}
    m['wall_dn'] = {}
    for name, (k, b) in walls.items():
        w = roi(ref[k], b).mean(0)
        fl = np.median(roi(ref[k], floors[k]), 0)
        m['wall_dn'][name] = {'wall': rgb(w), 'floor_same_frame': rgb(fl),
                              'wall_over_floor_G': round(float(w[1] / fl[1]), 3)}

    # 4. Shadow vs lit floor along the bottom of 14-42-34 (left part is shadowed).
    a = cv2.GaussianBlur(ref['14-42-34'][..., 1], (0, 0), 6)
    shadow, lit = float(np.median(a[990:1010, 0:300])), float(np.median(a[990:1010, 840:1200]))
    m['shadow_vs_lit_floor_G'] = {'shadow': round(shadow, 2), 'lit': round(lit, 2),
                                  'ratio': round(shadow / lit, 3)}

    # 5. Upright white armour-number stickers, R channel (blue-team LEDs add little red).
    #    Single frames: the robots move.
    stickers = {'14-41-49 "2"': ('14-41-49', 4.0, (170, 600, 330, 900)),
                '14-44-05 "4"': ('14-44-05', 5.5, (1370, 790, 1440, 890))}
    m['upright_white_sticker_R_dn'] = {}
    for name, (k, ss, b) in stickers.items():
        r = roi(decode(clip(k), ss=ss, n=1)[0], b)
        s = r.sum(1)
        m['upright_white_sticker_R_dn'][name] = round(float(r[s >= np.percentile(s, 85)][:, 0].mean()), 2)

    # 6. Blue floor paint vs floor, single sharp frames.
    m['blue_paint'] = {}
    for k, ss, b in (('14-44-05', 5.5, (330, 850, 640, 1080)), ('14-41-28', 3.5, (1250, 540, 1440, 720)),
                     ('14-44-20', 2.5, (0, 560, 500, 1080))):
        r = roi(decode(clip(k), ss=ss, n=1)[0], b)
        blueness = r[:, 2] - r[:, :2].mean(1)
        bl = r[blueness > np.percentile(blueness, 95)].mean(0)
        sel = (blueness < np.percentile(blueness, 50)) & (r.sum(1) > np.percentile(r.sum(1), 30))
        fl = np.median(r[sel], 0)
        m['blue_paint'][k] = {'paint': rgb(bl), 'floor': rgb(fl), 'paint_over_floor_G': rgb(bl / fl[1])}

    # 7. LED banner around the stands.
    m['banner_dn'] = rgb(np.median(roi(decode(clip('14-44-05'), ss=5.5, n=1)[0], (770, 150, 1000, 195)), 0))

    # 8. Temporal noise on the floor (camera nearly static).
    a = decode(clip('14-44-20'), ss=2.5, n=8).astype(float)[..., 1]
    mean, std = a.mean(0), a.std(0)
    sel = (mean > 10) & (mean < 20)
    m['floor_temporal_noise_G_dn'] = round(float(np.median(std[sel])), 2)

    # 9. Whole-field statistics and light bars, all field clips at 6 fps.
    hist = np.zeros(256)
    cores = {'blue': [], 'red': [], 'green': []}
    edge = {'blue': [], 'red': [], 'green': []}
    n_frames = 0
    for name in FIELD:
        for img in decode(CLIPS / f'{name}.mp4', fps=6):
            n_frames += 1
            luma = (img.astype(np.uint16) @ np.array([77, 150, 29], np.uint16)) >> 8
            hist += np.bincount(luma.ravel(), minlength=256)
            mx = img.max(2)
            sat = (mx >= 245).astype(np.uint8)
            k, lab, stats, _ = cv2.connectedComponentsWithStats(sat, 8)
            ring_all = cv2.dilate(sat, np.ones((9, 9), np.uint8)) - cv2.dilate(sat, np.ones((3, 3), np.uint8))
            for i in range(1, k):
                x, y, w, h, area = stats[i]
                if not 15 <= area <= 20000:
                    continue
                # The clipped core is near white for every colour; classify by its halo.
                c = img[y:y + h, x:x + w][lab[y:y + h, x:x + w] == i].mean(0)
                x0, y0 = max(x - 6, 0), max(y - 6, 0)
                halo = img[y0:y + h + 6, x0:x + w + 6][ring_all[y0:y + h + 6, x0:x + w + 6] > 0]
                if not len(halo):
                    continue
                r_, g_, b_ = halo.mean(0)
                cls = ('blue' if b_ > 1.5 * r_ and b_ > g_ else 'red' if r_ > 1.5 * b_ and r_ > g_
                       else 'green' if g_ > 1.3 * r_ and g_ > 1.3 * b_ else None)
                if cls:
                    cores[cls].append(c)
            ring = (cv2.dilate((mx >= 250).astype(np.uint8), np.ones((5, 5), np.uint8)) > 0) & (mx >= 120) & (mx <= 235)
            p = img[ring].astype(float)
            if len(p):
                c = p / p.max(1, keepdims=True)
                edge['blue'] += list(c[(c[:, 2] == 1) & (c[:, 0] < 0.6)])
                edge['red'] += list(c[(c[:, 0] == 1) & (c[:, 2] < 0.6)])
                edge['green'] += list(c[(c[:, 1] == 1) & (c[:, 0] < 0.8) & (c[:, 2] < 0.95)])
    cdf = np.cumsum(hist) / hist.sum()
    m['field_frames_analysed'] = n_frames
    m['field_luma_percentiles'] = {str(q): int(np.searchsorted(cdf, q / 100)) for q in (25, 50, 75, 90, 95, 99, 99.9)}
    m['field_luma_fraction_zero'] = round(float(hist[0] / hist.sum()), 3)
    m['led_core_dn_median'] = {k: {'n': len(v), 'rgb': rgb(np.median(v, 0))} for k, v in cores.items() if v}
    m['led_edge_chromaticity_median'] = {k: {'n': len(v), 'rgb': rgb(np.median(v, 0))} for k, v in edge.items() if v}

    # Derived ratios used by tools/build_venue_lighting.py.
    wf = [v['wall_over_floor_G'] for v in m['wall_dn'].values()]
    result['measurements'] = m
    result['derived'] = {
        'floor_over_white': round(float(np.mean(np.array(m['floor_next_to_marking_dn']) / np.array(m['white_marking_dn']))), 3),
        'wall_over_floor_range': [min(wf), max(wf)],
        'blue_paint_over_floor_median': rgb(np.median([v['paint_over_floor_G'] for v in m['blue_paint'].values()], 0)),
        'upright_over_horizontal_white': [round(v / m['white_marking_dn'][0], 2)
                                          for v in m['upright_white_sticker_R_dn'].values()],
        'led_over_white_lower_bound': round(255 / m['white_marking_dn'][1], 1),
    }
    (HERE / 'measurements.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(result['derived'], indent=2))


if __name__ == '__main__':
    main()
