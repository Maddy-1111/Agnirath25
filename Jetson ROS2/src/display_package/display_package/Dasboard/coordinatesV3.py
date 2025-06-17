# from flask_cors import CORS
# from flask import Flask, jsonify
# import random
# import math

# app = Flask(__name__)
# CORS(app)

# def generate_random_coordinates(center, radius, count):
#     coordinates = []
#     for _ in range(count):
#         angle = random.uniform(0, 2 * math.pi)
#         distance = random.uniform(0, radius)
#         dx = distance * math.cos(angle)
#         dy = distance * math.sin(angle)

#         new_lng = center[0] + dx / 111320  # 111320 meters per degree longitude
#         new_lat = center[1] + dy / 110540  # 110540 meters per degree latitude

#         coordinates.append([new_lng, new_lat])

#     return coordinates

# @app.route('/get_random_coordinates')
# def get_random_coordinates():
#     center = [80.23021913617647, 12.993036661158213]
#     radius = 1000  # 1 km
#     count = 1
#     random_coordinates = generate_random_coordinates(center, radius, count)
#     return jsonify(random_coordinates)

# if __name__ == '__main__':
#     app.run(debug=True)

from flask_cors import CORS
from flask import Flask, jsonify
import random
import math
from apscheduler.schedulers.background import BackgroundScheduler

app = Flask(__name__)
CORS(app)

coordinates = []

def generate_random_coordinates(center, radius, count):
    coordinates = []
    for _ in range(count):
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(0, radius)
        dx = distance * math.cos(angle)
        dy = distance * math.sin(angle)

        new_lng = center[0] + dx / 111320  # 111320 meters per degree longitude
        new_lat = center[1] + dy / 110540  # 110540 meters per degree latitude

        coordinates.append([new_lng, new_lat])

    return coordinates

def update_coordinates():
    center = [80.23021913617647, 12.993036661158213]
    radius = 1000  # 1 km
    count = 1
    global coordinates
    coordinates = generate_random_coordinates(center, radius, count)

@app.route('/get_random_coordinates')
def get_random_coordinates():
    return jsonify(coordinates)

if __name__ == '__main__':
    scheduler = BackgroundScheduler()
    scheduler.add_job(update_coordinates, 'interval', seconds=1)
    scheduler.start()

    app.run(debug=True)

