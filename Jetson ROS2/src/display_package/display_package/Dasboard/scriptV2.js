mapboxgl.accessToken = 'pk.eyJ1IjoiamFheWFudGgiLCJhIjoiY2xpcHVpMHlzMG01MDNmbGI1NTZoNDVpciJ9.GO9DSZT5E9tGdEKg8WAp9Q';

// Initialize the map
var map = new mapboxgl.Map({
    container: 'map',
    style: 'mapbox://styles/mapbox/streets-v11',
    center: [80.23021913617647, 12.993036661158213], // Default center coordinates (San Francisco)
    zoom: 15
});

var coordinates = [];

async function fetchCoordinates() {
    const response = await fetch('http://localhost:5000/get_random_coordinates');
    coordinates = await response.json();
    marker.setLngLat(coordinates[0]);
}

// Create a GeoJSON feature collection for the path
var path = {
    'type': 'FeatureCollection',
    'features': [{
        'type': 'Feature',
        'geometry': {
            'type': 'LineString',
            'coordinates': []
        }
    }]
};

// Create a custom marker
var marker = new mapboxgl.Marker({
        element: createMarkerElement(),
        rotationAlignment: 'map',
        draggable: false
    })
    .setLngLat([80.23021913617647, 12.993036661158213]) // Set initial marker position
    .addTo(map);

// Move marker to the next coordinate
function moveMarker() {
    if (coordinates.length === 0) return;

    var currentCoord = coordinates.shift();
    marker.setLngLat(currentCoord);

    // Add the current coordinate to the path
    path.features[0].geometry.coordinates.push(currentCoord);

    // Update the map's data source with the new path
    map.getSource('path').setData(path);

    // Recenter the map to the marker's location
    map.setCenter(currentCoord);
}

// Create a custom marker element
function createMarkerElement() {
    var img = document.createElement('img');
    img.src = 'location_marker.png'; // Replace 'marker.png' with your marker image file name
    img.style.width = '30px';
    img.style.height = '30px';
    return img;
}

// Move the marker every second
setInterval(async function () {
    const newCoord = await fetchCoordinates();
    moveMarker(newCoord);
}, 1000);

// Add the path layer to the map
map.on('load', function () {
    map.addSource('path', {
        type: 'geojson',
        data: path
    });

    map.addLayer({
        id: 'path',
        type: 'line',
        source: 'path',
        paint: {
            'line-color': '#4287f5',
            'line-opacity': 0.8,
            'line-width': 3
        }
    });

    // fetchCoordinates(); // Fetch coordinates when the map is loaded
});
