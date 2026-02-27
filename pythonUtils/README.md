# Turret Lookup Table Generator

Quick reference for generating, exporting, and deploying the turret lookup table.
A known-good `turret_data.json` is already committed — no new data collection needed unless re-tuning.

---

## 1. Activate the Virtual Environment (Windows Laptop)

Open a terminal in the project folder, then run:

```bat
venv\Scripts\activate
```

You should see `(venv)` appear in your prompt. Then install dependencies if needed:

```bat
pip install -r requirements.txt
```

---

## 2. Generate the Lookup Table

Start the generator:

```bat
python turret_lookup_generator.py
```

### Step 1 — Generate the static table (option 4)

Enter your desired distance range and step when prompted, e.g.:
- Min distance: `10`
- Max distance: `300`
- Step: `1`

### Step 2 — Add shoot-on-fly velocity entries (option 6)

Enter the robot velocity range to cover, e.g.:
- Min velocity: `-300`
- Max velocity: `300`
- Step: `1`

> Positive = moving toward target, Negative = moving away.

---

## 3. Export the Table (option 5)

Choose option **5** and press Enter to accept the default filename (`lookup_table.json`).

---

## 4. Deploy to the Raspberry Pi

Run these commands from your terminal. Replace steps as needed if your SSH client differs.

### Stop the service
```bash
ssh pi@10.4.47.11 "pm2 stop 0"
```

### Upload the lookup table
```bash
scp lookup_table.json pi@10.4.47.11:~/lookup_table.json
```

### Restart the service
```bash
ssh pi@10.4.47.11 "pm2 start 0"
```

---

## 5. Tuning the Velocity RPM Compensation

If shots are still undershooting or overshooting **only while moving**, adjust the compensation factor in [`turret_lookup_generator.py`](turret_lookup_generator.py#L492):

```python
VELOCITY_RPM_COMP_FACTOR = 500.0 / 60.0  # RPM per in/s — tune this value
```

The formula adds `VELOCITY_RPM_COMP_FACTOR × |vpar|` RPM to every shoot-on-fly entry.
At the current setting that's **+500 RPM at 60 in/s (5 ft/s)**.

| Still undershooting while moving | Increase the numerator (e.g. `600.0 / 60.0`) |
|----------------------------------|----------------------------------------------|
| Now overshooting while moving    | Decrease the numerator (e.g. `400.0 / 60.0`) |
| Stationary shots are off         | Do **not** touch this — retune `turret_data.json` instead |

After changing the value, re-run steps 2–4 to regenerate and redeploy the table.

### IF YOU NEED TO RETUNE `turret_data.json`
- Make a backup of the current one
- Delete the contents
- Choose option 1. (collect new data point) and follow it's prompts