# Grafana Dashboard Files

This folder contains the Grafana dashboard configuration and query files for the ESP32-MQTT-Weather project.

## Project Data Flow

The ESP32-MQTT-Weather system follows this data pipeline:

1. **ESP32 Device** - Reads BH1750 light sensor data and publishes to MQTT broker
2. **MQTT Broker** - Receives sensor data on configured topics
3. **Node-RED** - Subscribes to MQTT topics and processes sensor data
4. **InfluxDB 2.x** - Node-RED inserts time-series data into InfluxDB buckets
5. **Grafana** - Queries InfluxDB to visualize sensor data in dashboards

## Data Architecture

### Sensor Data
- **Light Level**: Ambient light intensity in Lux from BH1750 sensor
- **Sensor Status**: Device health and connectivity status
- **System Metrics**: ESP32 processor metrics and diagnostics

### MQTT Topics
- Default topic: `Data/BH1750`
- Configurable via web interface

## Structure

### Main Files
- `Grafana DualBMS Dashboard.json` - Legacy dashboard configuration (needs updating for weather data)
- `README.md` - This file

### Optimized_Queries/
Contains high-performance Flux queries for weather station data visualization:

**Light Sensor Queries:**
- `Query_Light_Level.flux` - Ambient light intensity over time
- `Query_Sensor_Status.flux` - Device health and connectivity status

**System Monitoring Queries:**
- `Query_System_Metrics.flux` - ESP32 processor and memory metrics
- `Query_WiFi_Signal.flux` - WiFi signal strength monitoring

**Environmental Data:**
- `Query_Daily_Light_Pattern.flux` - Daily light patterns and trends
- `Query_Light_Statistics.flux` - Statistical analysis of light data

### Documentation/
- `Implementation_Guide.md` - Step-by-step implementation guide
- `Performance_Optimization_Guide.md` - Performance optimization details

### Images/
- Screenshots and visual documentation

## Performance Features

All queries are optimized for weather station data and include:
- **Intelligent Bucket Selection**: Automatically chooses optimal data source based on time range
  - ≤ 6 hours: Raw sensor data (Weather bucket)
  - ≤ 2 days: 30-second aggregates (Weather_30s bucket)
  - ≤ 2 weeks: 5-minute aggregates (Weather_5m bucket)
  - > 2 weeks: 1-hour aggregates (Weather_1h bucket)
- **Sub-5-second query times** across all time ranges
- **Elimination of "too many data points" errors**
- **Optimized for light sensor and environmental data**

## Data Flow Integration

### ESP32 → MQTT → Node-RED → InfluxDB → Grafana

1. **ESP32 Device**: 
   - Reads BH1750 light sensor every 5 seconds (configurable)
   - Publishes JSON data to MQTT broker
   - Includes sensor readings and system metrics

2. **MQTT Broker**:
   - Receives data on topic: `Data/BH1750` (configurable)
   - Provides reliable message delivery

3. **Node-RED**:
   - Subscribes to MQTT topics
   - Processes and transforms sensor data
   - Inserts formatted data into InfluxDB buckets
   - Handles data validation and error recovery

4. **InfluxDB 2.x**:
   - Stores time-series sensor data
   - Provides multiple resolution buckets for performance
   - Enables historical data analysis

5. **Grafana**:
   - Queries InfluxDB for visualization
   - Displays real-time and historical data
   - Provides alerting and monitoring capabilities

## Usage

1. Set up the complete data pipeline:
   - Configure ESP32 with WiFi and MQTT broker settings
   - Set up Node-RED to subscribe to ESP32 MQTT topics
   - Configure Node-RED to write data to InfluxDB buckets
   - Set up Grafana with InfluxDB data source

2. Use the optimized queries:
   - Copy contents from `Optimized_Queries/` files
   - Paste into Grafana panel query fields
   - Customize time ranges and visualization preferences

3. Monitor your weather station data:
   - View real-time light sensor readings
   - Analyze daily and seasonal light patterns
   - Monitor system health and connectivity

## Requirements

- **ESP32 Device**: Running ESP32-MQTT-Weather firmware
- **MQTT Broker**: Mosquitto or similar (local or cloud)
- **Node-RED**: With MQTT and InfluxDB nodes configured
- **InfluxDB 2.x**: With weather data buckets and aggregation tasks
- **Grafana**: With InfluxDB data source configured

## Sample JSON Data Structure

ESP32 publishes data in this format:
```json
{
  "sunlight": 523.4,
  "sensor_ok": true,
  "processor": {
    "uptime_seconds": 3600,
    "heap_free_bytes": 245760,
    "wifi_rssi_percent": 85,
    "wifi_rssi_dbm": -55
  }
}
```

## Dashboard Features

- **Real-time Light Monitoring**: Current lux readings with trend indicators
- **Historical Analysis**: Daily, weekly, and monthly light patterns
- **System Health**: Device uptime, memory usage, and WiFi signal strength
- **Environmental Insights**: Daylight patterns and seasonal variations

## Dashboard Screenshot

![Weather Station Dashboard](Images/WeatherStationDashboard.jpg)

