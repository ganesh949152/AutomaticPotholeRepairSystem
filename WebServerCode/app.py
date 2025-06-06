from flask import Flask, render_template, request, jsonify
import json
from datetime import datetime
import os
import logging
import traceback

app = Flask(_name_)
DATA_FILE = "/home/yourpythonanywhereusername/APRS/repair_data.json"
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(_name_)


def load_data():
    """Load data from the JSON file."""
    try:
        logger.debug(f"Attempting to open {DATA_FILE} for reading...")
        with open(DATA_FILE, 'r') as f:
            logger.debug(f"Successfully opened {DATA_FILE} for reading.")
            try:
                data = json.load(f)
                logger.debug(f"Successfully loaded JSON data: {data}")
                return data
            except json.JSONDecodeError as e:
                logger.error(f"JSONDecodeError in load_data: {e}")
                logger.error(f"File contents at time of error: {f.read()}")
                return []
    except FileNotFoundError:
        logger.warning(f"FileNotFoundError: {DATA_FILE} not found. Returning empty list.")
        return []
    except Exception as e:
        logger.error(f"Exception in load_data: {e}")
        logger.error(traceback.format_exc())
        return []


def save_data(data):
    """Save data to the JSON file."""
    try:
        logger.debug(f"Attempting to open {DATA_FILE} for writing...")
        with open(DATA_FILE, 'w') as f:
            json.dump(data, f, indent=4)
            logger.debug(f"Successfully saved data to {DATA_FILE}: {data}")
    except Exception as e:
        logger.error(f"Exception in save_data: {e}")
        logger.error(traceback.format_exc())


def append_data(new_data):
    """Append new data to the JSON file."""
    logger.debug(f"Appending data: {new_data}")
    data = load_data()
    data.append(new_data)
    save_data(data)


@app.route('/')
def dashboard():
    logger.debug("Entering dashboard route")
    repair_records = load_data()
    logger.debug(f"Loaded repair records: {repair_records}")
    volumes = [record.get('pothole_volume_cm3', 0) for record in repair_records]
    timestamps = [record.get('timestamp', '') for record in repair_records]
    cement_used = sum(record.get('cement_units_used', 0) for record in repair_records)
    water_used = sum(record.get('water_units_used', 0) for record in repair_records)
    pothole_counts = sum(record.get('cumulative_potholes_fixed',0) for record in repair_records)
    total_distance = sum(record.get('cumulative_distance_m', 0.0) for record in repair_records) * 100 #convert to cm
    logger.debug(
        f"Volumes: {volumes}, Timestamps: {timestamps}, Counts: {pothole_counts}, Distance: {total_distance}, Cement: {cement_used}, Water: {water_used}")

    return render_template('dashboard.html',
                           records=repair_records,
                           volumes=volumes,
                           timestamps=timestamps,
                           pothole_counts=pothole_counts,
                           total_distance=total_distance,
                           cement_used=cement_used,
                           water_used=water_used)



@app.route('/receive_data', methods=['POST'])
def receive_data():
    logger.debug("Entering receive_data route")
    try:
        data = request.get_json()
        logger.debug(f"Received data: {data}")
        if data:
            data['timestamp'] = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')
            append_data(data)
            return 'Data received and saved successfully', 200
        else:
            return 'No data received', 400
    except Exception as e:
        logger.error(f"Exception in receive_data: {e}")
        logger.error(traceback.format_exc())
        return 'Error processing data', 400


@app.route('/update_status', methods=['POST'])
def update_status():
    logger.debug("Entering update_status route")
    return 'Status update endpoint (currently no action)', 200


if _name_ == '_main_':
    if not os.path.exists(DATA_FILE):
        logger.info(f"Data file {DATA_FILE} does not exist, creating it.")
        with open(DATA_FILE, 'w') as f:
            json.dump([], f)
        logger.info(f"Data file {DATA_FILE} successfully created.")
    else:
        logger.info(f"Data file {DATA_FILE} already exists.")
    app.run(debug=False,port = 5023)
