from flask import Flask, request, jsonify
from geopy.geocoders import Nominatim
from geopy.extra.rate_limiter import RateLimiter
from geopy.distance import geodesic
import osmnx as ox
import networkx as nx
from shapely.geometry import Polygon
import pickle
import os
from datetime import datetime
import random
import time

app = Flask(__name__)

# Constants
MAP_CACHE_FILE = "coimbatore_tiruppur_map_cache.pkl"
TRAFFIC_CACHE_FILE = "traffic_data_cache.pkl"
TRAFFIC_UPDATE_INTERVAL = 3600  # 1 hour
BOUNDARY = {
    "north": 11.4,
    "south": 10.7,
    "east": 77.4,
    "west": 76.8
}

class RoutePlanner:
    def __init__(self):
        self.graph = None
        self.traffic_data = None
        self.last_traffic_update = 0
        self.geolocator = Nominatim(user_agent="coimbatore_tiruppur_route_planner")
        self.geocode = RateLimiter(self.geolocator.geocode, min_delay_seconds=1)

    def location_to_coords(self, location_name):
        try:
            location = self.geocode(location_name)
            if location:
                lat, lon = location.latitude, location.longitude
                if not (BOUNDARY["south"] <= lat <= BOUNDARY["north"] and 
                        BOUNDARY["west"] <= lon <= BOUNDARY["east"]):
                    raise ValueError(f"Location '{location_name}' is outside service area")
                return (lat, lon)
            raise ValueError(f"Location '{location_name}' not found")
        except Exception as e:
            raise ValueError(f"Geocoding error: {str(e)}")

    def load_or_download_map(self):
        if os.path.exists(MAP_CACHE_FILE):
            print("Loading map from cache...")
            with open(MAP_CACHE_FILE, 'rb') as f:
                self.graph = pickle.load(f)

            # Check if 'travel_time' exists
            if not all('travel_time' in data for _, _, data in self.graph.edges(data=True)):
                print("travel_time missing in cache! Recomputing...")
                self.graph = ox.add_edge_speeds(self.graph)
                self.graph = ox.add_edge_travel_times(self.graph)
                with open(MAP_CACHE_FILE, 'wb') as f:
                    pickle.dump(self.graph, f)
                print("Updated map cache with travel_time.")
        else:
            print("Downloading new map...")
            polygon = Polygon([
                (BOUNDARY["west"], BOUNDARY["south"]),
                (BOUNDARY["west"], BOUNDARY["north"]),
                (BOUNDARY["east"], BOUNDARY["north"]),
                (BOUNDARY["east"], BOUNDARY["south"])
            ])
            self.graph = ox.graph_from_polygon(polygon, network_type='drive')
            self.graph = ox.add_edge_speeds(self.graph)
            self.graph = ox.add_edge_travel_times(self.graph)
            with open(MAP_CACHE_FILE, 'wb') as f:
                pickle.dump(self.graph, f)
            print("Map downloaded and cached.")

    def update_traffic_data(self):
        now = time.time()
        if self.traffic_data and now - self.last_traffic_update < TRAFFIC_UPDATE_INTERVAL:
            return

        print("Updating traffic data...")
        traffic_data = {}
        for u, v, data in self.graph.edges(data=True):
            traffic_factor = random.uniform(0.8, 3.0)
            hour = datetime.now().hour
            if (7 <= hour <= 10) or (17 <= hour <= 20):
                traffic_factor *= 1.5
            traffic_data[(u, v)] = traffic_factor

        self.traffic_data = traffic_data
        self.last_traffic_update = now
        with open(TRAFFIC_CACHE_FILE, 'wb') as f:
            pickle.dump(traffic_data, f)

    def calculate_route(self, start_coords, end_coords):
        self.update_traffic_data()

        orig_node = ox.distance.nearest_nodes(self.graph, start_coords[1], start_coords[0])
        dest_node = ox.distance.nearest_nodes(self.graph, end_coords[1], end_coords[0])

        def traffic_weight(u, v, d):
            base_time = d.get('travel_time', 1)
            traffic_factor = self.traffic_data.get((u, v), 1.0)
            return base_time * traffic_factor

        try:
            return nx.shortest_path(self.graph, orig_node, dest_node, weight=traffic_weight)
        except Exception as e:
            raise ValueError(f"Routing error: {str(e)}")

    def get_route_coordinates(self, route):
        coords = []
        for u, v in zip(route[:-1], route[1:]):
            data = self.graph.get_edge_data(u, v)
            if data:
                edge_data = list(data.values())[0]
                if 'geometry' in edge_data:
                    coords.extend([[lat, lon] for lon, lat in edge_data['geometry'].coords])
                else:
                    coords.append([self.graph.nodes[u]['y'], self.graph.nodes[u]['x']])
                    coords.append([self.graph.nodes[v]['y'], self.graph.nodes[v]['x']])
        return coords

planner = RoutePlanner()
planner.load_or_download_map()

@app.route('/api/geocode', methods=['GET'])
def geocode():
    location = request.args.get('location')
    if not location:
        return jsonify({"error": "Location parameter is required"}), 400
    try:
        coords = planner.location_to_coords(location)
        return jsonify({"location": location, "latitude": coords[0], "longitude": coords[1]})
    except Exception as e:
        return jsonify({"error": str(e)}), 400

@app.route('/api/route', methods=['GET'])
def get_route():
    start = request.args.get('start')
    end = request.args.get('end')

    if not start or not end:
        return jsonify({"error": "Both start and end are required"}), 400

    try:
        start_coords = planner.location_to_coords(start)
        end_coords = planner.location_to_coords(end)

        route = planner.calculate_route(start_coords, end_coords)
        route_coords = planner.get_route_coordinates(route)

        distance = sum(
            geodesic((route_coords[i][0], route_coords[i][1]), (route_coords[i+1][0], route_coords[i+1][1])).km
            for i in range(len(route_coords) - 1)
        )

        duration = sum(
            planner.graph.get_edge_data(route[i], route[i+1])[0].get('travel_time', 1)
            for i in range(len(route) - 1)
        ) / 60  # minutes

        if planner.traffic_data:
            traffic_duration = sum(
                planner.graph.get_edge_data(route[i], route[i+1])[0].get('travel_time', 1) *
                planner.traffic_data.get((route[i], route[i+1]), 1.0)
                for i in range(len(route) - 1)
            ) / 60
            duration = traffic_duration

        return jsonify({
            "start": start,
            "end": end,
            "start_coords": start_coords,
            "end_coords": end_coords,
            "route": route_coords,
            "distance": distance,
            "duration": duration,
            "traffic_considered": planner.traffic_data is not None
        })

    except Exception as e:
        return jsonify({"error": str(e)}), 400

@app.route('/')
def index():
    return app.send_static_file('index.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
