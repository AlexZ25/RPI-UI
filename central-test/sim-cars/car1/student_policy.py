"""Lane-following policy: follow the single yellow center line (version 2026-10-22).

How it works, once per camera frame:

1. Find the line.  The lower part of the image (the floor) is cut into horizontal
   bands.  In each band we look for runs of line pixels and keep the run closest to
   where we expect the line to be (last frame's fit), so noise blobs and reflections
   are ignored.  Each kept run gives one point (x, y) on the line.
2. Fit a straight line x = a*y + b through those points.  From it we read
   - e_near: how far the line is from the car's center just in front of the car,
   - e_far:  how far it is at a "look-ahead" row further up the image.
   Both are scaled to [-1, 1] (-1 = left edge of the image, +1 = right edge).
3. Steering = KP_NEAR*e_near + KP_FAR*e_far + KD*(change of e_far per second),
   then smoothed and rate-limited so the servo does not jitter.
4. Throttle: fast on straights, slower when the look-ahead shows a curve or the
   steering is large, slower when only a few line pixels are visible.
5. Line lost: keep turning toward the side where the line was last seen, slowly,
   for LOST_TIMEOUT_S seconds; after that, stop.

6. Policy view (dashboard): a half-size picture of what the policy saw and decided
   (line pixels, band points, fitted line, look-ahead/near rows, steering bar).
   Set DEBUG_VIEW = False to skip drawing it and save a little time per step.

Everything is in this one file and uses only numpy, OpenCV + the aoe API.
"""
import cv2
import numpy as np

from aoe.api import Action, Observation


class StudentPolicy:
    def __init__(self):
        # --- where to look (fractions of the image height; row 0 = top) ---
        self.ROI_TOP = 0.50          # ignore everything above this (walls, horizon)
        self.ROI_BOTTOM = 0.97       # ignore the very bottom rows (car body on the real car)
        self.N_BANDS = 10            # number of horizontal bands between ROI_TOP and ROI_BOTTOM
        self.LOOK_AHEAD = 0.58       # row (fraction of H) used for e_far
        self.NEAR_ROW = 0.95         # row (fraction of H) used for e_near
        self.CENTER_X = 0.50         # column that is "straight ahead" (measure on your car!)
        self.MIN_BAND_PX = 6         # a band needs at least this many line pixels to count
        self.MIN_BANDS = 2           # need this many good bands to trust a fit
        self.SUBSAMPLE = 2           # use every 2nd row and column (4x faster, same result)

        # --- steering ---
        self.KP_NEAR = 1.0
        self.KP_FAR = 2.0
        self.KD = 0.06               # damping on the look-ahead error rate (per second)
        self.STEER_SMOOTH_S = 0.04   # low-pass time constant (seconds)
        self.STEER_RATE = 8.0        # max steering change per second (full range = 2.0)

        # --- throttle (runtime caps at limits.max_throttle, 0.5 by default) ---
        self.THROTTLE_MAX = 0.45     # on straights
        self.THROTTLE_MIN = 0.22     # in the sharpest curves (must stay above ~0.05 to move)
        self.CURVE_SLOWDOWN = 1.6    # how strongly curves reduce speed
        self.STARTUP_S = 0.5         # ramp throttle up over this long at the start of a run

        # --- line lost ---
        self.LOST_THROTTLE = 0.18
        self.LOST_STEER = 0.8        # turn this hard toward the last seen side
        self.LOST_TIMEOUT_S = 1.0    # stop after the line has been missing this long

        # --- dashboard Policy view ---
        self.DEBUG_VIEW = True       # draw the debug picture (costs ~1 ms)
        self.DEBUG_SCALE = 0.5       # picture size relative to the camera image

        self.reset()

    def reset(self):
        """Called before each run. Clear everything that remembers the past."""
        self.steering = 0.0          # last steering command sent
        self.prev_e_far = None       # for the derivative term
        self.fit = None              # (a, b) of x = a*y + b from the last good frame
        self.last_side = 0.0         # -1 = line was last seen on the left, +1 = right
        self.lost_time = 0.0         # seconds the line has been missing

    # ------------------------------------------------------------------ helpers
    def _line_pixels(self, obs):
        line = obs.label_mask("center_line")
        if obs.label_id("center_line") is None:      # unknown label set: any non-background pixel
            line = obs.mask > 0
        return line

    def _band_points(self, line, h, w):
        """One (x, y, weight) point per band, choosing the run nearest the expected x."""
        s = self.SUBSAMPLE
        y_top = int(h * self.ROI_TOP)
        y_bot = int(h * self.ROI_BOTTOM)
        edges = np.linspace(y_top, y_bot, self.N_BANDS + 1).astype(int)
        cols = np.arange(0, w, s)
        points = []
        for y0, y1 in zip(edges[:-1], edges[1:]):
            if y1 <= y0:
                continue
            counts = line[y0:y1:s, ::s].sum(axis=0)
            if counts.sum() < self.MIN_BAND_PX:
                continue
            yc = 0.5 * (y0 + y1)
            if self.fit is not None:
                expect = self.fit[0] * yc + self.fit[1]
            elif points:
                expect = points[-1][0]
            else:
                expect = w * self.CENTER_X
            # split the column histogram into runs of non-zero columns
            on = counts > 0
            starts = np.flatnonzero(on & ~np.r_[False, on[:-1]])
            ends = np.flatnonzero(on & ~np.r_[on[1:], False]) + 1
            best = None
            for a, b in zip(starts, ends):
                c = counts[a:b]
                n = c.sum()
                if n < self.MIN_BAND_PX:
                    continue
                x = float((cols[a:b] * c).sum() / n)
                if best is None or abs(x - expect) < abs(best[0] - expect):
                    best = (x, float(n))
            if best is not None:
                points.append((best[0], yc, best[1]))
        return points

    def _rate_limit(self, target, dt):
        alpha = dt / (self.STEER_SMOOTH_S + dt) if dt > 0 else 1.0
        smoothed = self.steering + alpha * (target - self.steering)
        max_step = self.STEER_RATE * dt if dt > 0 else 2.0
        return float(np.clip(smoothed, self.steering - max_step, self.steering + max_step))

    def _draw(self, obs, line, points, steering, throttle, state):
        """Small BGR picture for the dashboard Policy view."""
        h, w = obs.height, obs.width
        k = self.DEBUG_SCALE
        sw, sh = max(1, int(w * k)), max(1, int(h * k))
        img = cv2.resize(obs.image_bgr, (sw, sh), interpolation=cv2.INTER_NEAREST)
        img = (img // 2).astype(np.uint8)                                  # darken the camera image
        small = cv2.resize(line.astype(np.uint8), (sw, sh), interpolation=cv2.INTER_NEAREST) > 0
        img[small] = (0, 220, 255)                                         # line pixels in yellow

        def p(x, y):
            return int(round(x * k)), int(round(y * k))

        # region of interest, straight-ahead column
        cv2.rectangle(img, p(0, h * self.ROI_TOP), p(w - 1, h * self.ROI_BOTTOM), (90, 90, 90), 1)
        cx = w * self.CENTER_X
        cv2.line(img, p(cx, h * self.ROI_TOP), p(cx, h - 1), (200, 200, 200), 1)

        # band points (green) and the fitted line (magenta) with near/far targets
        for x, y, _ in points:
            cv2.circle(img, p(x, y), 3, (0, 255, 0), -1)
        if self.fit is not None:
            a, b = self.fit
            y0, y1 = h * self.ROI_TOP, h * self.ROI_BOTTOM
            cv2.line(img, p(a * y0 + b, y0), p(a * y1 + b, y1), (255, 0, 255), 1)
            for row, color in ((self.LOOK_AHEAD, (0, 0, 255)), (self.NEAR_ROW, (255, 128, 0))):
                y = h * row
                cv2.line(img, p(cx, y), p(a * y + b, y), color, 1)
                cv2.circle(img, p(a * y + b, y), 5, color, 2)

        # steering bar (top) and throttle bar (right edge)
        mid = sw // 2
        cv2.rectangle(img, (mid, 4), (mid + int(steering * (sw // 2 - 4)), 12), (255, 255, 0), -1)
        cv2.line(img, (mid, 2), (mid, 14), (255, 255, 255), 1)
        cv2.rectangle(img, (sw - 10, sh - 1), (sw - 3, sh - 1 - int(throttle * 2 * (sh - 20))),
                      (0, 255, 0), -1)
        color = (0, 255, 0) if state == "tracking" else (0, 165, 255) if state == "searching" else (0, 0, 255)
        cv2.putText(img, f"{state}  s={steering:+.2f} t={throttle:.2f}", (4, sh - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
        return img

    # ------------------------------------------------------------------ main
    def step(self, obs: Observation) -> Action:
        h, w = obs.height, obs.width
        dt = min(max(float(obs.dt), 0.0), 0.2)
        half_w = 0.5 * w
        cx = w * self.CENTER_X

        line = self._line_pixels(obs)
        points = self._band_points(line, h, w)

        if len(points) >= self.MIN_BANDS:
            pts = np.asarray(points)
            x, y, wt = pts[:, 0], pts[:, 1], np.sqrt(pts[:, 2])
            if np.ptp(y) > 0:
                a, b = np.polyfit(y, x, 1, w=wt)
            else:
                a, b = 0.0, float(x.mean())
            self.fit = (float(a), float(b))
            e_near = float(np.clip((a * h * self.NEAR_ROW + b - cx) / half_w, -1.5, 1.5))
            e_far = float(np.clip((a * h * self.LOOK_AHEAD + b - cx) / half_w, -1.5, 1.5))

            de = 0.0
            if self.prev_e_far is not None and dt > 0:
                de = (e_far - self.prev_e_far) / dt
            self.prev_e_far = e_far

            target = self.KP_NEAR * e_near + self.KP_FAR * e_far + self.KD * de
            target = float(np.clip(target, -1.0, 1.0))
            self.steering = self._rate_limit(target, dt)

            if abs(e_far) > 0.05:
                self.last_side = 1.0 if e_far > 0 else -1.0
            self.lost_time = 0.0

            curve = abs(e_far - e_near) + 0.5 * abs(self.steering)
            speed = self.THROTTLE_MAX - self.CURVE_SLOWDOWN * curve * (self.THROTTLE_MAX - self.THROTTLE_MIN)
            confidence = min(1.0, len(points) / (0.6 * self.N_BANDS))
            throttle = max(self.THROTTLE_MIN, speed) * (0.6 + 0.4 * confidence)
            state = "tracking"
        else:
            # line not visible: search toward the side it was last seen on, then give up
            self.lost_time += dt
            self.fit = None
            self.prev_e_far = None
            e_near = e_far = 0.0
            if self.lost_time > self.LOST_TIMEOUT_S:
                self.steering = 0.0
                throttle = 0.0
                state = "stopped (line lost)"
            else:
                self.steering = self._rate_limit(self.LOST_STEER * self.last_side, dt)
                throttle = self.LOST_THROTTLE
                state = "searching"

        # gentle start so the wheels do not spin
        if self.STARTUP_S > 0:
            throttle *= min(1.0, 0.4 + 0.6 * obs.elapsed / self.STARTUP_S)

        steering = float(np.clip(self.steering, -1.0, 1.0))
        throttle = float(np.clip(throttle, 0.0, self.THROTTLE_MAX))
        view = self._draw(obs, line, points, steering, throttle, state) if self.DEBUG_VIEW else None
        return Action(
            steering=steering,
            throttle=throttle,
            debug={
                "state": state,
                "bands": len(points),
                "e_near": round(e_near, 3),
                "e_far": round(e_far, 3),
                "lost_s": round(self.lost_time, 2),
            },
            debug_image=view,
        )
