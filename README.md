# fatayet_el_9owwa
# Geographic Sampling Points Generator

This project provides tools to generate regularly spaced sampling points within a specified polygon boundary on Earth's surface. It includes accurate distance calculation using the Haversine formula and grid point generation based on desired spatial resolution.

---

## 🧭 Features

- Generate grid-based sampling points inside a geographical polygon.
- Approximate spacing between points using meters.
- Accurate great-circle distance calculation via Haversine formula.
- Customizable resolution.

---

## 🧠 How It Works

### 1. `generate_sampling_points(boundary, S)`
Generates a grid of points within a polygon (latitude/longitude) spaced approximately `S` meters apart.

**Parameters:**
- `boundary`: List of latitude/longitude tuples defining a closed polygon.
- `S`: Desired spacing between points (in meters).

**Returns:**
- A list of `(lat, lon)` tuples representing valid points inside the boundary.

### 2. `haversine(lat1, lon1, lat2, lon2)`
Calculates the great-circle distance between two geographic coordinates using the Haversine formula.

**Parameters:**
- `lat1`, `lon1`: Latitude and longitude of the first point.
- `lat2`, `lon2`: Latitude and longitude of the second point.

**Returns:**
- Distance in meters.

---

## 📦 Dependencies

- Python 3.x
- `math`
- `itertools` (for Cartesian product)
- A helper function: `is_inside(p, boundary)` — must be defined (e.g. using `shapely` or ray-casting algorithm)

---

## 📌 Example

```python
boundary = [(36.0, 10.0), (36.1, 10.0), (36.1, 10.1), (36.0, 10.1)]  # simple rectangle
S = 1000  # spacing: 1 km

points = generate_sampling_points(boundary, S)

for p in points:
    print(p)
