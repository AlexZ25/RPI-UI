# Trackside

One page for a fleet of cars: live status and camera/policy view of every car, Run / Stop
(one or all), policy upload to several cars at once, car management, and a map of where the
cars are. Python standard library only (3.11+); the localizer needs OpenCV.

How it fits together: **one central computer** runs `hub.py` (and `localizer.py`). Each car,
at its own IP, keeps running its own dashboard on port 8080, and runs `join.py`, which finds
the hub on the network and tells it "I'm here". The hub then talks to each car's dashboard
over HTTP. Nothing else on the cars changes.

```
 car (IP .21) ── dashboard :8080 ◄──┐            ┌── browser(s): http://<central>:9000
              └─ join.py ──register─┤            │
 car (IP .22) ── dashboard :8080 ◄──┼── hub.py :9000 (central computer)
              └─ join.py ──register─┘      ▲
                                           └── localizer.py: pulls each car's camera image,
                                               finds the ArUco markers around the grid
```

| File | What it is |
|---|---|
| `hub.py` | the server, on the central computer (`python3 hub.py` → http://localhost:9000) |
| `index.html` | the page |
| `cars.toml` | the car list (cars that joined + cars added on the page; editable by hand while the hub is stopped) |
| `join.py` | runs on each car: registers it with the hub (standard library only) |
| `trackside-join.service` | starts `join.py` at boot on a car |
| `localizer.py` | car cameras + ArUco markers around the grid → car positions on the map |
| `grid.example.toml` | where the markers are (copy to `grid.toml` and measure yours) |
| `start_sim_cars.sh` | simulated cars for trying everything without hardware |

## Try it with simulated cars

```bash
./start_sim_cars.sh          # terminal 1: 3 simulated cars on ports 8081-8083 (Ctrl+C stops them)
python3 hub.py               # terminal 2: open http://localhost:9000
```

`cars.toml` lists those three plus a fourth that is never started (shows an offline car).
Each simulated car has its own policy copy in `sim-cars/carN/`, so uploads never touch the
car-software repo (expected at `../AOE-DR`; override with `CAR_REPO=...`).
On the map, simulated cars are placed on an oval by the distance they have driven: each one
really drives its own private simulated track, so this shows who is ahead, not a real layout.

## The page

- **Header:** online / running count, **Cars**, **Run all** (online cars), **Stop all**
  (every car; also the **Space** key).
- **Map:** car arrows (position + heading) in each car's color. Collapsible.
- **Card per car:** state, live image (Policy / Overlay / Camera / Mask), steering, throttle,
  steps/s, run time, and one line with the most important message (policy error, PWM not
  started, why it stopped, ...). **Run** starts the camera first if it is off.
- **Cars dialog:** cars that joined by themselves appear here automatically. You can also add
  one by address (**Check** finds the car and fills in its name), edit (address, name, color)
  or remove cars. Removing only takes the car off the hub (a car still running `join.py`
  comes back within 5 s). Saved to `cars.toml` immediately.
- **Upload policy:** pick a `.py`, tick cars with **Select**, **Upload & reload**.

## Using it with real cars

### 1. Network
- Put every Pi and the central computer on **one network**. A small dedicated router at the
  track is best: school / venue Wi-Fi often blocks device-to-device traffic and broadcasts.
- Each car just needs an IP on that network (DHCP is fine: when a car's IP changes, `join.py`
  updates its entry and the car keeps its name and color). Give each Pi a distinct hostname
  when imaging it (`deepracer-1`, `deepracer-2`, ...): the hub tells cars apart by hostname.
- On the central computer, allow incoming connections for Python when the OS firewall asks
  (macOS: System Settings > Network > Firewall). The hub needs TCP 9000 in, and sends UDP
  broadcasts on 9999.

### 2. The central computer
```bash
python3 hub.py --cars real-cars.toml            # a missing file starts an empty list
HUB_PASSWORD=choose-one python3 hub.py ...      # require a password (cars and localizer use it too)
```
It prints the address cars and other computers use, e.g. `http://192.168.1.10:9000`. Open
that from any laptop / tablet on the network, or `http://localhost:9000` on the central
computer. It also broadcasts its address so cars find it (`--no-beacon` turns that off).

### 3. On each car (once)
- Set the car up as usual (see the car repo's README) and make its dashboard start at boot:
  `sudo bash deploy/install_services.sh`. It listens on port 8080 on all interfaces.
- Copy `join.py` to the car and start it at boot (adjust the user in the file if it is not `pi`):
  ```bash
  scp join.py trackside-join.service pi@deepracer-1.local:
  ssh pi@deepracer-1.local 'sudo mv trackside-join.service /etc/systemd/system/ && sudo systemctl enable --now trackside-join'
  ```
  Or by hand: `python3 join.py`. It finds the hub by its broadcast; if broadcasts are blocked,
  give the address: `python3 join.py --hub http://192.168.1.10:9000` (also in the service file).
  `--name "Team Red"` sets the name the first time; after that rename from the page.
- The car appears in the hub within a few seconds. `journalctl -u trackside-join -f` shows
  what it is doing ("in the hub ... as ...", "not reachable", "wrong HUB_PASSWORD").

Cars can still be added by address from the **Cars** dialog without `join.py` (then a changed
IP has to be edited by hand), or listed in a file: copy `cars.example.toml`.

### 4. Each session
For safety, the motor sequence stays on each car's own dashboard (the hub does not do it):
1. On the car's dashboard: **Start PWM**, wait for "PWM is at NEUTRAL", switch the motor **ON**.
   Until then its hub card says "PWM not started".
2. In the hub: **Run** / **Run all**. Run starts the camera if needed.
3. **Stop all** (or Space) to stop. Then motors OFF and **Stop PWM** on each car.

The hub is a convenience, not the emergency stop. If the hub or Wi-Fi goes down, a car keeps
doing what it was last told (a car with no new commands from its own runtime goes neutral
after 0.5 s, but a running policy keeps driving). Keep the on-car STOP / motor switch in reach.

### 5. The map (ArUco markers around the grid)
Each car finds itself with its own camera: printed ArUco markers stand at known places around
the grid, and `localizer.py` (on the central computer) looks for them in every car's camera
image and works out where that car is and which way it points. No overhead camera, nothing
extra on the cars.

```bash
pip install opencv-python numpy
python3 localizer.py --make-markers markers/ --ids 0-11      # printable PNGs
```
1. Print the markers all the same size, keeping the white border (matte paper; glare hides
   them). The black square's side is `marker_size` in `grid.toml`.
2. Stand them **upright** around the grid (walls, posts, boxes), printed side facing in, at
   about the car cameras' height. Place them so a car sees **at least two at once** wherever
   it drives: with one marker the position is rough (off sideways by up to ~15% of the
   distance to it); with two or more it is within a few centimetres. A 15 cm marker is
   readable to about 3 m; use bigger ones for a bigger grid.
3. `cp grid.example.toml grid.toml` and enter each marker: id, centre `x`, `y` (metres, from
   the grid's top-left corner, y pointing down as on the map) and `facing` (the direction its
   printed side faces: 0 = right, 90 = down, 180 = left, 270 = up). Set `camera_forward` to how
   far the camera is in front of the car's centre.
4. Optional but more accurate: calibrate the car camera once with a printed chessboard (all
   cars have the same camera model):
   ```bash
   python3 localizer.py --calibrate http://deepracer-1.local:8080 --board 9x6 --square 0.025
   ```
   Without it the camera is modelled from `--hfov` (66 degrees = Pi Camera Module 3; 102 for the wide one).
5. Run it next to the hub:
   ```bash
   python3 localizer.py --grid grid.toml --intrinsics camera.json     # hub at http://localhost:9000
   ```
   Every 2 s it prints, per car, which markers it sees and where it puts the car. The map
   switches to the grid with the markers drawn on its edges and each located car's arrow.
   A car is only located while its camera is on (Run turns it on); positions older than 2 s
   disappear from the map.

Each car's image comes over Wi-Fi (about 30 kB per frame; 10 per second per car by default,
`--hz` to change). If many cars share a weak network, lower `--hz`.
