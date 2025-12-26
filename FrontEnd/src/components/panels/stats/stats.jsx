// Logic

import React from "react";
import { useState } from "react";
import { FaMicrophone } from "react-icons/fa";

// Default stats
const FALLBACK_STATS = {
  overview: {
    totalKm: 182.4,
    missionsCompleted: 47,
    avgSpeed: 1.8,
    operatingHours: 326,
    deltaKm: 4.3,
    missionSuccessRate: 98,
  },
  missionTrend: [
    { label: "Mon", completed: 5, incidents: 0 },
    { label: "Tue", completed: 7, incidents: 1 },
    { label: "Wed", completed: 6, incidents: 0 },
    { label: "Thu", completed: 8, incidents: 1 },
    { label: "Fri", completed: 9, incidents: 0 },
  ],
  monthlyMovement: [
    { month: "Jan", km: 118 },
    { month: "Feb", km: 142 },
    { month: "Mar", km: 131 },
    { month: "Apr", km: 155 },
    { month: "May", km: 162 },
    { month: "Jun", km: 174 },
  ],
  batterySeries: [
    { time: "08:00", voltage: 48.2, power: 182 },
    { time: "09:00", voltage: 47.8, power: 176 },
    { time: "10:00", voltage: 47.4, power: 171 },
    { time: "11:00", voltage: 46.9, power: 168 },
    { time: "12:00", voltage: 47.1, power: 170 },
    { time: "13:00", voltage: 46.7, power: 166 },
    { time: "14:00", voltage: 46.3, power: 164 },
  ],
  batteryStatus: {
    packVoltage: 46.9,
    packCurrent: 38.2,
    stateOfCharge: 78,
    temperature: "32°C",
    cycles: 412,
    health: "Good",
    cells: [
      { id: "Cell A", voltage: 3.9 },
      { id: "Cell B", voltage: 3.89 },
      { id: "Cell C", voltage: 3.88 },
      { id: "Cell D", voltage: 3.87 },
    ],
  },
  turns: { left: 132, right: 148 },
};

  const [statsData, setStatsData] = useState(FALLBACK_STATS);
  {
    {
          const payload = await requestV1("/stats");
        if (!cancelled) {
          const statsPayload = payload.data || FALLBACK_STATS;
          setStatsData({
            overview: statsPayload.overview || FALLBACK_STATS.overview,
            missionTrend:
              statsPayload.missionTrend || FALLBACK_STATS.missionTrend,
            monthlyMovement:
              statsPayload.monthlyMovement || FALLBACK_STATS.monthlyMovement,
            batterySeries:
              statsPayload.batterySeries || FALLBACK_STATS.batterySeries,
            batteryStatus:
              statsPayload.batteryStatus || FALLBACK_STATS.batteryStatus,
            turns: statsPayload.turns || FALLBACK_STATS.turns,
          });
        }
      } catch (err) {
        console.error("Stats fetch error", err);
        if (!cancelled) {
          setStatsData(FALLBACK_STATS);
          setStatsError("Using cached telemetry");
        }
      } finally {
        if (!cancelled) {
          setStatsLoading(false);
        }
      }
    };

    loadStats();
    const intervalId = setInterval(loadStats, 60000);
    return () => {
      cancelled = true;
      clearInterval(intervalId);
    };
  }, [requestV1]);

  useEffect(() => {
    let cancelled = false;

const lineChartSize = { width: 420, height: 180 };
  const buildLinePath = (series, key) => {
    if (!series.length) return "";
    const max = Math.max(...series.map((point) => point[key]));
    const min = Math.min(...series.map((point) => point[key]));
    const range = max - min || 1;
    const stepX =
      series.length > 1
        ? lineChartSize.width / (series.length - 1)
        : lineChartSize.width;
    return series
      .map((point, index) => {
        const x = index * stepX;
        const normalized = (point[key] - min) / range;
        const y = lineChartSize.height - normalized * lineChartSize.height;
        return `${x},${y}`;
      })
      .join(" ");
  };

  const buildSimplePath = (points, size) => {
    if (!points.length) return "";
    const max = Math.max(...points);
    const min = Math.min(...points);
    const range = max - min || 1;
    const step =
      points.length > 1 ? size.width / (points.length - 1) : size.width;
    return points
      .map((value, index) => {
        const x = index * step;
        const normalized = (value - min) / range;
        const y = size.height - normalized * size.height;
        return `${x},${y}`;
      })
      .join(" ");
  };

  useEffect(() => {
    let cancelled = false;

    const loadStats = async () => {
      try {
        if (!cancelled) {
          setStatsLoading(true);
          setStatsError("");
        }

        useEffect(() => {
  let cancelled = false;

    useEffect(() => {
  let cancelled = false;
  const {
    overview,
    missionTrend,
    monthlyMovement,
    batterySeries,
    batteryStatus,
    turns,
  } = statsData;
  const analyticsChartSize = { width: 280, height: 80 };
  const analyticsPath = buildSimplePath(analyticsSeries, analyticsChartSize);
  const overviewDeltaLabel =
    typeof overview.deltaKm === "number"
      ? overview.deltaKm.toFixed(1)
      : overview.deltaKm;
  const batteryVoltagePath = buildLinePath(batterySeries, "voltage");
  const batteryPowerPath = buildLinePath(batterySeries, "power");
  const totalTurns = (turns.left || 0) + (turns.right || 0) || 1;
  const leftTurnPercent = Math.round(((turns.left || 0) / totalTurns) * 100);
  const rightTurnPercent = Math.round(((turns.right || 0) / totalTurns) * 100);
  const monthlyMaxKm = monthlyMovement.length
    ? Math.max(...monthlyMovement.map((m) => m.km))
    : 1;
  const totalMonthlyKm = monthlyMovement.reduce(
    (sum, entry) => sum + (entry.km || 0),
    0,
  );
  const avgMonthlyKm = monthlyMovement.length
    ? (totalMonthlyKm / monthlyMovement.length).toFixed(1)
    : "0.0";

//  RightPane logic
  const {
    rightPage,
    setRightPage,
    // stats
    overview,
    missionTrend,
    monthlyMovement,
    batterySeries,
    batteryStatus,
    turns,
    statsLoading,
    statsError,
    lineChartSize,
    buildLinePath,
    buildSimplePath,
    analyticsChartSize,
    analyticsPath,
  } = props;

  // UI
  {/* STATS */}
        {rightPage === "stats" && (
          <div className="stats-pane">
            <div className="stats-status-row">
              {statsLoading && <span className="stats-status">Refreshing telemetry…</span>}
              {statsError && <span className="stats-error">{statsError}</span>}
            </div>
            <div className="stats-summary">
              <div className="stats-card">
                <span className="stats-label">Total Moving Distance</span>
                <h3>{overview.totalKm.toFixed(1)} km</h3>
                <p>+{(typeof overview.deltaKm === "number" ? overview.deltaKm.toFixed(1) : overview.deltaKm)} km vs previous day</p>
              </div>
              <div className="stats-card">
                <span className="stats-label">Missions Completed</span>
                <h3>{overview.missionsCompleted}</h3>
                <p>{overview.missionSuccessRate}% success over 7 days</p>
              </div>
              <div className="stats-card">
                <span className="stats-label">Average Speed</span>
                <h3>{overview.avgSpeed.toFixed(1)} m/s</h3>
                <p>Within safe corridor</p>
              </div>
              <div className="stats-card">
                <span className="stats-label">Operating Hours</span>
                <h3>{overview.operatingHours} h</h3>
                <p>Last maintenance at 300 h</p>
              </div>
            </div>

            <div className="movement-card">
              <div className="stats-card-header">
                <div><h4>Monthly Movement</h4><p>Distance travelled per month</p></div>
              </div>
              <div className="movement-summary">
                <div><span className="movement-label">Total</span><strong>{monthlyMovement.reduce((s,e)=>s+(e.km||0),0).toFixed(1)} km</strong></div>
                <div><span className="movement-label">Average / month</span><strong>{monthlyMovement.length ? (monthlyMovement.reduce((s,e)=>s+(e.km||0),0)/monthlyMovement.length).toFixed(1) : "0.0"} km</strong></div>
              </div>
              <div className="movement-bars">
                {monthlyMovement.map((entry) => (
                  <div key={entry.month} className="movement-bar">
                    <div className="movement-bar-fill" style={{ height: `${(entry.km / (monthlyMovement.length ? Math.max(...monthlyMovement.map(m=>m.km)) : 1)) * 100}%` }} title={`${entry.km} km`} />
                    <span>{entry.month}</span>
                  </div>
                ))}
              </div>
            </div>

            <div className="stats-grid">
              <div className="stats-chart-card">
                <div className="stats-card-header">
                  <div><h4>Battery Voltage & Power</h4><p>Live pack telemetry</p></div>
                  <div className="stats-legend"><span className="legend-dot voltage" /> Voltage <span className="legend-dot power" /> Power</div>
                </div>
                <div className="line-chart">
                  <svg width={lineChartSize.width} height={lineChartSize.height} role="img" aria-label="Battery voltage and power line plot">
                    {[0.25,0.5,0.75,1].map((ratio)=>(
                      <line key={String(ratio)} x1="0" x2={lineChartSize.width} y1={lineChartSize.height*ratio} y2={lineChartSize.height*ratio} className="chart-grid-line" />
                    ))}
                    <polyline points={buildLinePath(batterySeries,"voltage")} className="line-voltage" fill="none" strokeWidth="3" />
                    <polyline points={buildLinePath(batterySeries,"power")} className="line-power" fill="none" strokeWidth="2" />
                  </svg>
                  <div className="chart-x-axis">{batterySeries.map((p)=><span key={p.time}>{p.time}</span>)}</div>
                </div>
              </div>

              <div className="turn-card">
                <div className="stats-card-header"><div><h4>Turn Distribution</h4><p>Number of left/right turns this shift</p></div></div>
                <div className="turn-count-row"><span>Left turns</span><strong>{turns.left}</strong></div>
                <div className="turn-bar"><div className="turn-bar-left" style={{ width: `${Math.round((turns.left / ((turns.left||0)+(turns.right||0)||1))*100)}%` }} /></div>
                <div className="turn-count-row"><span>Right turns</span><strong>{turns.right}</strong></div>
                <div className="turn-bar"><div className="turn-bar-right" style={{ width: `${Math.round((turns.right / ((turns.left||0)+(turns.right||0)||1))*100)}%` }} /></div>
                <div className="turn-footer">Left {Math.round((turns.left/((turns.left||0)+(turns.right||0)||1))*100)}% · Right {Math.round((turns.right/((turns.left||0)+(turns.right||0)||1))*100)}%</div>
              </div>
            </div>

            <div className="trend-card">
              <div className="stats-card-header"><div><h4>Mission Trend</h4><p>Completion vs incidents</p></div></div>
              <table><thead><tr><th>Day</th><th>Completed</th><th>Incidents</th></tr></thead><tbody>{missionTrend.map(r=> <tr key={r.label}><td>{r.label}</td><td>{r.completed}</td><td>{r.incidents}</td></tr>)}</tbody></table>
            </div>
          </div>
        )}