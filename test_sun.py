import pvlib
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 1) Observer’s location: Helsinki
latitude, longitude = 60.1699, 24.9384
timezone = 'Europe/Helsinki'

# 2) Timespan: June 21, 2025
times = pd.date_range(
    start='2025-06-21 00:00',
    end  ='2025-06-21 23:59',
    freq='10min',
    tz=timezone
)

# 3) Compute solar position
location = pvlib.location.Location(latitude, longitude, timezone)
solpos = location.get_solarposition(times)
solpos = solpos.loc[solpos['apparent_elevation'] > 0, :]
alt = solpos['apparent_elevation'].values  # degrees
az  = solpos['azimuth'].values             # degrees

# 4) Convert to Cartesian coords
R = 1000
x = R * np.cos(np.radians(alt)) * np.sin(np.radians(az))
y = R * np.cos(np.radians(alt)) * np.cos(np.radians(az))
z = R * np.sin(np.radians(alt))

# ─── New: color estimation ────────────────────────────────────────────────────

def cct_to_rgb(cct_kelvin):
    """
    Convert color temperature in Kelvin to an RGB tuple (each 0–1).
    Based on Tanner Helland’s algorithm, but correctly uses 0–255 range.
    """
    temp = cct_kelvin / 100.0

    # RED
    if temp <= 66:
        red = 255.0
    else:
        red = 329.698727446 * ((temp - 60) ** -0.1332047592)

    # GREEN
    if temp <= 66:
        green = 99.4708025861 * np.log(temp) - 161.1195681661
    else:
        green = 288.1221695283 * ((temp - 60) ** -0.0755148492)

    # BLUE
    if temp >= 66:
        blue = 255.0
    elif temp <= 19:
        blue = 0.0
    else:
        blue = 138.5177312231 * np.log(temp - 10) - 305.0447927307

    # Clamp to 0–255
    red   = np.clip(red,   0, 255)
    green = np.clip(green, 0, 255)
    blue  = np.clip(blue,  0, 255)

    # Normalize to 0–1 for matplotlib
    return red/255.0, green/255.0, blue/255.0

# Map altitude [0, max_alt] → CCT [2000 K, 6500 K]
min_temp, max_temp = 2000, 6500
max_alt = alt.max()

temps = min_temp + (alt / max_alt) * (max_temp - min_temp)
# Compute RGB for each point
colors = np.array([cct_to_rgb(t) for t in temps])

print(255*colors)

'''
# ─── 6) Send colors to LED strip via HTTP ─────────────────────────────────────
import requests
import time

BASE_URL = "http://192.168.1.120/win"

for c in colors:
    r = f"&R={c[0]*255}"
    g = f"&G={c[1]*255}"
    b = f"&B={c[2]*255}"
    print(f"Setting color: R={r}, G={g}, B={b}")

    params = {
        '&R': {c[0]*255},
        '&G': {c[1]*255},
        '&B': {c[2]*255}
    }

    url = f"{BASE_URL}&R={int(c[0]*255)}&G={int(c[1]*255)}&B={int(c[2]*255)}"

    r = requests.get(url)

    time.sleep(0.1)  # Wait a bit before next request
'''

# ─── 5) Plot with per‐point color ─────────────────────────────────────────────

fig = plt.figure(figsize=(8,6))
ax  = fig.add_subplot(111, projection='3d')

# Use scatter to apply individual colors
ax.scatter(x, y, z, c=colors, marker='o', s=15)
ax.set_xlabel('East (X)')
ax.set_ylabel('North (Y)')
ax.set_zlabel('Up (Z)')
ax.set_title('Sun Path — Helsinki, Finland (June 21, 2025)\n(Points colored by estimated sun hue)')

# Optional: add reference sphere
_u = np.linspace(0, 2*np.pi, 36)
_v = np.linspace(0,  np.pi, 18)
xs = np.outer(np.cos(_u), np.sin(_v))
ys = np.outer(np.sin(_u), np.sin(_v))
zs = np.outer(np.ones_like(_u), np.cos(_v))
ax.plot_wireframe(xs, ys, zs, alpha=0.1, linewidth=0.5)

plt.tight_layout()
plt.show()