#!/usr/bin/env python3
import http.server
import socketserver
import json
import os

PORT = 8000

HTML_DASHBOARD = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Drone Altitude Telemetry</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        :root {
            --bg-color: #0f172a;
            --panel-bg: #1e293b;
            --text-main: #f8fafc;
            --text-muted: #94a3b8;
            --accent: #38bdf8;
            --error-color: #ef4444;
        }
        body {
            font-family: 'Inter', -apple-system, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-main);
            margin: 0;
            padding: 2rem;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        .header {
            text-align: center;
            margin-bottom: 2rem;
        }
        .header h1 {
            margin: 0;
            font-size: 2.5rem;
            font-weight: 700;
            background: linear-gradient(to right, #38bdf8, #818cf8);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        .header p {
            color: var(--text-muted);
            margin-top: 0.5rem;
        }
        .dashboard {
            width: 100%;
            max-width: 1200px;
            display: grid;
            gap: 2rem;
        }
        .panel {
            background-color: var(--panel-bg);
            border-radius: 1rem;
            padding: 1.5rem;
            box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.5);
            transition: transform 0.2s;
        }
        .panel:hover {
            transform: translateY(-2px);
        }
        .chart-container {
            position: relative;
            height: 400px;
            width: 100%;
        }
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 1rem;
            margin-bottom: 2rem;
        }
        .stat-card {
            background-color: var(--panel-bg);
            padding: 1.5rem;
            border-radius: 1rem;
            text-align: center;
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.3);
            border: 1px solid #334155;
        }
        .stat-card h3 {
            margin: 0;
            font-size: 0.875rem;
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 0.05em;
        }
        .stat-card .value {
            font-size: 2rem;
            font-weight: 700;
            color: var(--accent);
            margin-top: 0.5rem;
        }
        .refresh-btn {
            background-color: var(--accent);
            color: #0f172a;
            border: none;
            padding: 0.75rem 1.5rem;
            font-size: 1rem;
            font-weight: 600;
            border-radius: 0.5rem;
            cursor: pointer;
            transition: background-color 0.2s, transform 0.1s;
            margin-top: 2rem;
        }
        .refresh-btn:hover {
            background-color: #7dd3fc;
        }
        .refresh-btn:active {
            transform: scale(0.95);
        }
        #status-indicator {
            display: inline-block;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background-color: #22c55e;
            margin-right: 0.5rem;
        }
    </style>
</head>
<body>

    <div class="header">
        <h1><span id="status-indicator"></span>Drone Telemetry Dashboard</h1>
        <p>Real-time altitude and error tracking</p>
    </div>

    <div class="dashboard">
        <div class="stats-grid">
            <div class="stat-card">
                <h3>Total Flight Time</h3>
                <div class="value" id="stat-time">0.0 s</div>
            </div>
            <div class="stat-card">
                <h3>Max Altitude</h3>
                <div class="value" id="stat-max-alt">0 cm</div>
            </div>
            <div class="stat-card">
                <h3>Max Error</h3>
                <div class="value" id="stat-max-error" style="color: var(--error-color);">0 cm</div>
            </div>
        </div>

        <div class="panel">
            <div class="chart-container">
                <canvas id="altitudeChart"></canvas>
            </div>
        </div>

        <div class="panel">
            <div class="chart-container">
                <canvas id="errorChart"></canvas>
            </div>
        </div>
    </div>

    <button class="refresh-btn" onclick="fetchData()">Refresh Data</button>

    <script>
        let altChartInstance = null;
        let errChartInstance = null;

        Chart.defaults.color = '#94a3b8';
        Chart.defaults.font.family = 'Inter';

        async function fetchData() {
            try {
                const response = await fetch('/data');
                if (!response.ok) throw new Error('No data found');
                const data = await response.json();
                updateCharts(data);
                updateStats(data);
                document.getElementById('status-indicator').style.backgroundColor = '#22c55e';
            } catch (error) {
                console.error("Error fetching data:", error);
                document.getElementById('status-indicator').style.backgroundColor = '#ef4444';
            }
        }

        function updateStats(data) {
            if (!data || data.length === 0) return;
            const lastPoint = data[data.length - 1];
            document.getElementById('stat-time').innerText = lastPoint.time.toFixed(1) + ' s';
            
            const maxAlt = Math.max(...data.map(d => d.altitude));
            document.getElementById('stat-max-alt').innerText = Math.round(maxAlt) + ' cm';
            
            const maxErr = Math.max(...data.map(d => Math.abs(d.error)));
            document.getElementById('stat-max-error').innerText = Math.round(maxErr) + ' cm';
        }

        function updateCharts(data) {
            const labels = data.map(d => d.time.toFixed(1));
            const altitudes = data.map(d => d.altitude);
            const targets = data.map(d => d.target);
            const errors = data.map(d => d.error);

            if (altChartInstance) altChartInstance.destroy();
            if (errChartInstance) errChartInstance.destroy();

            const ctxAlt = document.getElementById('altitudeChart').getContext('2d');
            altChartInstance = new Chart(ctxAlt, {
                type: 'line',
                data: {
                    labels: labels,
                    datasets: [
                        {
                            label: 'Actual Altitude (cm)',
                            data: altitudes,
                            borderColor: '#38bdf8',
                            backgroundColor: 'rgba(56, 189, 248, 0.1)',
                            borderWidth: 2,
                            fill: true,
                            tension: 0.4,
                            pointRadius: 0
                        },
                        {
                            label: 'Target Altitude (cm)',
                            data: targets,
                            borderColor: '#10b981',
                            borderWidth: 2,
                            borderDash: [5, 5],
                            fill: false,
                            tension: 0.1,
                            pointRadius: 0
                        }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    interaction: {
                        mode: 'index',
                        intersect: false,
                    },
                    plugins: {
                        legend: { position: 'top' },
                        title: { display: true, text: 'Altitude Tracking Profile', color: '#f8fafc', font: {size: 16} }
                    },
                    scales: {
                        y: { title: { display: true, text: 'Altitude (cm)' } },
                        x: { title: { display: true, text: 'Time (s)' } }
                    }
                }
            });

            const ctxErr = document.getElementById('errorChart').getContext('2d');
            errChartInstance = new Chart(ctxErr, {
                type: 'line',
                data: {
                    labels: labels,
                    datasets: [
                        {
                            label: 'Altitude Error (cm)',
                            data: errors,
                            borderColor: '#ef4444',
                            backgroundColor: 'rgba(239, 68, 68, 0.1)',
                            borderWidth: 2,
                            fill: true,
                            tension: 0.4,
                            pointRadius: 0
                        }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    interaction: {
                        mode: 'index',
                        intersect: false,
                    },
                    plugins: {
                        legend: { position: 'top' },
                        title: { display: true, text: 'Error Over Time', color: '#f8fafc', font: {size: 16} }
                    },
                    scales: {
                        y: { title: { display: true, text: 'Error (cm)' } },
                        x: { title: { display: true, text: 'Time (s)' } }
                    }
                }
            });
        }

        // Fetch data once on load
        fetchData();
    </script>
</body>
</html>
"""

class TelemetryHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_DASHBOARD.encode('utf-8'))
        elif self.path == '/data':
            if os.path.exists('altitude_data.json'):
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                with open('altitude_data.json', 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.send_response(404)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(b'[]')
        else:
            # Fallback to default behavior
            super().do_GET()

if __name__ == "__main__":
    with socketserver.TCPServer(("", PORT), TelemetryHandler) as httpd:
        print(f"🚀 Telemetry server running at http://localhost:{PORT}")
        print("Press Ctrl+C to stop.")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down server.")
            httpd.server_close()
