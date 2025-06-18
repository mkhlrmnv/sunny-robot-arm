import pvlib
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 1) Observer’s location: Helsinki
latitude, longitude = 60.1699, 24.9384
timezone = 'Europe/Helsinki'

# 2) Timespan: June 21, 2025 from sunrise to sunset (or fixed window)
times = pd.date_range(
    start='2025-06-21 00:00',   
    end  ='2025-06-21 23:59',   
    freq='10min',
    tz=timezone
)

# 3) Compute solar position (altitude & azimuth)
location = pvlib.location.Location(latitude, longitude, timezone)
solpos = location.get_solarposition(times)
solpos = solpos.loc[solpos['apparent_elevation'] > 0, :]
alt = solpos['apparent_elevation']  # degrees above horizon
az  = solpos['azimuth']             # degrees clockwise from North

# 4) Convert to unit‐sphere Cartesian for 3D plotting
#    X axis → East, Y → North, Z → Up
R = 1000   

x = R * np.cos(np.radians(alt)) * np.sin(np.radians(az))
y = R * np.cos(np.radians(alt)) * np.cos(np.radians(az))
z = R * np.sin(np.radians(alt))

# 5) Plot
fig = plt.figure(figsize=(8,6))
ax  = fig.add_subplot(111, projection='3d')

ax.plot(x, y, z, marker='.', linestyle='-')
ax.set_xlabel('East (X)')
ax.set_ylabel('North (Y)')
ax.set_zlabel('Up (Z)')
ax.set_title('Sun Path — Helsinki, Finland (June 21, 2025)')

# Optional: add a wire‐frame sphere for reference
_u = np.linspace(0, 2*np.pi, 36)
_v = np.linspace(0,  np.pi, 18)
xs = np.outer(np.cos(_u), np.sin(_v))
ys = np.outer(np.sin(_u), np.sin(_v))
zs = np.outer(np.ones_like(_u), np.cos(_v))
ax.plot_wireframe(xs, ys, zs, alpha=0.1, linewidth=0.5)

plt.tight_layout()

plt.show()