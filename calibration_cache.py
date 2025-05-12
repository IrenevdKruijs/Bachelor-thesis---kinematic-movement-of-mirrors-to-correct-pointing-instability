# calibration_cache.py
import os
import json
from functions import calibrate_mirror1_2D  # Adjust if it's from a different module

def get_cached_calibration(amount_steps, stepsize, repeats, steprate, camera, cache_file="calibration_cache.json"):
    """
    Checks if calibration data is cached. If not, performs calibration and stores result.
    Returns calibration data as a list of (step, dx, dy) tuples.
    """
    if os.path.exists(cache_file):
        with open(cache_file, 'r') as f:
            cache = json.load(f)
    else:
        cache = {}

    key = f"{amount_steps}_{stepsize}_{repeats}_{steprate}"

    if key in cache:
        print(f"[CACHE] Using cached calibration for steprate = {steprate}")
        return cache[key]
    else:
        print(f"[CALIBRATION] Performing calibration for steprate = {steprate}")
        calibration_data = calibrate_mirror1_2D(amount_steps, stepsize, repeats, steprate, camera)
        
        # Convert to JSON-safe format
        serializable_data = [
            [[int(s), float(dx), float(dy)] for s, dx, dy in run]
            for run in calibration_data
        ]
        cache[key] = serializable_data

        with open(cache_file, 'w') as f:
            json.dump(cache, f)

        return serializable_data
