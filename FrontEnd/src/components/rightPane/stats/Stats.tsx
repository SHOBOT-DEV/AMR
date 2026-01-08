import React, { useState, useEffect } from "react";
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

const Stats = (props: any) => {
  const [statsData, setStatsData] = useState(FALLBACK_STATS);
  const [statsLoading, setStatsLoading] = useState(false);
  const [statsError, setStatsError] = useState("");

  const lineChartSize = { width: 420, height: 180 };

  const buildLinePath = (series: any[], key: string) => {
    if (!series.length) return "";
    const max = Math.max(...series.map((point) => point[key]));
    const min = Math.min(...series.map((point) => point[key]));
    const range = max - min || 1;
    const stepX = series.length > 1 ? lineChartSize.width / (series.length - 1) : lineChartSize.width;
    return series
      .map((point, index) => {
        const x = index * stepX;
        const normalized = (point[key] - min) / range;
        const y = lineChartSize.height - normalized * lineChartSize.height;
        return `${x},${y}`;
      })
      .join(" ");
  };

  const buildSimplePath = (points: number[], size: { width: number; height: number }) => {
    if (!points.length) return "";
    const max = Math.max(...points);
    const min = Math.min(...points);
    const range = max - min || 1;
    const step = points.length > 1 ? size.width / (points.length - 1) : size.width;
    return points
      .map((value, index) => {
        const x = index * step;
        const normalized = (value - min) / range;
        const y = size.height - normalized * size.height;
        return `${x},${y}`;
      })
      .join(" ");
  };

  const { rightPage } = props;

  // --- Stats UI ---
  const { overview, monthlyMovement, missionTrend, batterySeries, turns } = statsData;

  const totalMonthlyKm = monthlyMovement.reduce((s, e) => s + (e.km || 0), 0);
  const avgMonthlyKm = monthlyMovement.length ? (totalMonthlyKm / monthlyMovement.length).toFixed(1) : "0.0";
  const maxMonthlyKm = monthlyMovement.length ? Math.max(...monthlyMovement.map((m) => m.km)) : 1;

  const totalTurns = (turns.left || 0) + (turns.right || 0) || 1;

  return (
    <>
      {rightPage === "stats" && (
        <div className="flex flex-col gap-4 p-4">
          {/* Status Row */}
          <div className="flex justify-between text-[13px] text-slate-600">
            {statsLoading && <span className="text-[#0b74d1] animate-pulse">Refreshing telemetry…</span>}
            {statsError && <span className="text-[#c2410c]">{statsError}</span>}
          </div>

          {/* Summary Cards */}
          <div className="grid grid-cols-[repeat(auto-fit,minmax(160px,1fr))] gap-3">
            <div className="bg-slate-50 rounded-xl p-3 shadow-inner border border-slate-200">
              <span className="text-[12px] uppercase tracking-wider text-slate-400">Total Moving Distance</span>
              <h3 className="text-[24px] text-slate-900 mt-1">{overview.totalKm.toFixed(1)} km</h3>
              <p className="text-[13px] text-slate-500">
                +{typeof overview.deltaKm === "number" ? overview.deltaKm.toFixed(1) : overview.deltaKm} km vs previous day
              </p>
            </div>
            <div className="bg-slate-50 rounded-xl p-3 shadow-inner border border-slate-200">
              <span className="text-[12px] uppercase tracking-wider text-slate-400">Missions Completed</span>
              <h3 className="text-[24px] text-slate-900 mt-1">{overview.missionsCompleted}</h3>
              <p className="text-[13px] text-slate-500">{overview.missionSuccessRate}% success over 7 days</p>
            </div>
            <div className="bg-slate-50 rounded-xl p-3 shadow-inner border border-slate-200">
              <span className="text-[12px] uppercase tracking-wider text-slate-400">Average Speed</span>
              <h3 className="text-[24px] text-slate-900 mt-1">{overview.avgSpeed.toFixed(1)} m/s</h3>
              <p className="text-[13px] text-slate-500">Within safe corridor</p>
            </div>
            <div className="bg-slate-50 rounded-xl p-3 shadow-inner border border-slate-200">
              <span className="text-[12px] uppercase tracking-wider text-slate-400">Operating Hours</span>
              <h3 className="text-[24px] text-slate-900 mt-1">{overview.operatingHours} h</h3>
              <p className="text-[13px] text-slate-500">Last maintenance at 300 h</p>
            </div>
          </div>

          {/* Monthly Movement */}
          <div className="bg-white rounded-xl shadow-lg p-4">
            <div className="flex justify-between items-center mb-3">
              <div>
                <h4 className="text-[16px] text-slate-900 font-semibold">Monthly Movement</h4>
                <p className="text-[13px] text-slate-500">Distance travelled per month</p>
              </div>
            </div>

            <div className="flex gap-6 text-[13px] text-slate-600 mb-3">
              <div>
                <span className="uppercase text-[11px] tracking-wider text-slate-400 block">Total</span>
                <strong className="text-[20px] text-slate-900 block">{totalMonthlyKm.toFixed(1)} km</strong>
              </div>
              <div>
                <span className="uppercase text-[11px] tracking-wider text-slate-400 block">Average / month</span>
                <strong className="text-[20px] text-slate-900 block">{avgMonthlyKm} km</strong>
              </div>
            </div>

            <div className="flex items-end gap-3 h-40">
              {monthlyMovement.map((entry) => (
                <div key={entry.month} className="flex flex-col items-center gap-1 text-[12px] text-slate-400">
                  <div
                    className="w-full rounded-t-lg bg-gradient-to-b from-sky-400 to-sky-500"
                    style={{ height: `${(entry.km / maxMonthlyKm) * 100}%` }}
                    title={`${entry.km} km`}
                  />
                  <span>{entry.month}</span>
                </div>
              ))}
        </div>


      {/* Battery chart */}
      <div className="bg-white rounded-xl p-4 shadow-[0_15px_40px_rgba(15,23,42,0.08)]">
        <div className="flex justify-between gap-3">
          <div>
            <h4 className="text-base text-slate-900">
              Battery Voltage & Power
            </h4>
            <p className="text-[13px] text-gray-500">Live pack telemetry</p>
          </div>

          <div className="flex items-center gap-3 text-[13px] text-slate-600">
            <span className="flex items-center gap-1">
              <span className="w-2.5 h-2.5 rounded-full bg-sky-500" /> Voltage
            </span>
            <span className="flex items-center gap-1">
              <span className="w-2.5 h-2.5 rounded-full bg-orange-500" /> Power
            </span>
          </div>
        </div>

        <svg
          className="mt-3"
          width={lineChartSize.width}
          height={lineChartSize.height}
        >
          {[0.25, 0.5, 0.75, 1].map((r) => (
            <line
              key={r}
              x1={0}
              x2={lineChartSize.width}
              y1={lineChartSize.height * r}
              y2={lineChartSize.height * r}
              stroke="rgba(148,163,184,0.35)"
              strokeWidth={1}
            />
          ))}

          <polyline
            fill="none"
            stroke="#0ea5e9"
            strokeWidth={3}
            points={buildLinePath(batterySeries, "voltage")}
          />
          <polyline
            fill="none"
            stroke="#f97316"
            strokeWidth={2}
            points={buildLinePath(batterySeries, "power")}
          />
        </svg>
      </div>

      {/* Turn distribution */}
      <div className="bg-white rounded-xl p-4 shadow-[0_15px_40px_rgba(15,23,42,0.08)]">
        <h4 className="text-base text-slate-900">Turn Distribution</h4>
        <p className="text-[13px] text-gray-500">
          Number of left/right turns
        </p>

        <div className="mt-3">
          <div className="flex justify-between text-sm text-slate-900">
            <span>Left turns</span>
            <strong>{turns.left}</strong>
          </div>
          <div className="mt-1 h-3 bg-slate-100 rounded-full overflow-hidden">
            <div
              className="h-full bg-gradient-to-r from-cyan-400 to-sky-500"
              style={{
                width: `${
                  (turns.left / (turns.left + turns.right)) * 100
                }%`,
              }}
            />
          </div>
        </div>

        <div className="mt-3">
          <div className="flex justify-between text-sm text-slate-900">
            <span>Right turns</span>
            <strong>{turns.right}</strong>
          </div>
          <div className="mt-1 h-3 bg-slate-100 rounded-full overflow-hidden">
            <div
              className="h-full bg-gradient-to-r from-yellow-300 to-orange-500"
              style={{
                width: `${
                  (turns.right / (turns.left + turns.right)) * 100
                }%`,
              }}
            />
          </div>
        </div>

        <p className="mt-4 text-[13px] text-slate-600 text-right">
          Left{' '}
          {Math.round(
            (turns.left / (turns.left + turns.right)) * 100,
          )}% · Right{' '}
          {Math.round(
            (turns.right / (turns.left + turns.right)) * 100,
          )}%
        </p>
      </div>

      {/* Mission trend */}
      <div className="bg-white rounded-xl p-4 shadow-[0_15px_40px_rgba(15,23,42,0.08)]">
        <h4 className="text-base text-slate-900">Mission Trend</h4>
        <p className="text-[13px] text-gray-500">Completion vs incidents</p>

        <table className="mt-3 w-full text-sm">
          <thead className="text-slate-500">
            <tr>
              <th className="text-left">Day</th>
              <th className="text-right">Completed</th>
              <th className="text-right">Incidents</th>
            </tr>
          </thead>
          <tbody>
            {missionTrend.map((r) => (
              <tr key={r.label} className="border-t text-slate-700">
                <td className="py-1">{r.label}</td>
                <td className="py-1 text-right">{r.completed}</td>
                <td className="py-1 text-right">{r.incidents}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      
          </div>
        </div>
      )}
    </>
  );
};

export default Stats;
 