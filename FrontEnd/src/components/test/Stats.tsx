import React, { useMemo } from "react";

// --- Types ---

export interface Overview {
  totalKm: number;
  deltaKm?: number;
  missionsCompleted: number;
  missionSuccessRate?: number;
  avgSpeed: number;
  operatingHours: number;
}

export interface MovementEntry {
  month: string;
  km?: number;
}

export interface BatteryPoint {
  time: string;
  voltage: number;
  power: number;
}

export interface TrendEntry {
  label: string;
  completed: number;
  incidents: number;
}

export interface Turns {
  left?: number;
  right?: number;
}

export interface StatsProps {
  overview?: Overview;
  monthlyMovement?: MovementEntry[];
  missionTrend?: TrendEntry[];
  batterySeries?: BatteryPoint[];
  turns?: Turns;
  statsLoading?: boolean;
  statsError?: string;
  lineChartSize?: { width: number; height: number };
  // Optional: You can pass a custom path builder, or use the default internal one
  buildLinePath?: (series: BatteryPoint[], key: "voltage" | "power") => string;
}

// --- Helpers ---

const safeNumber = (value?: number) => {
  return typeof value === "number" && !isNaN(value) ? value : 0;
};

// Default path builder if none is provided
const defaultBuildLinePath = (
  series: BatteryPoint[],
  key: "voltage" | "power",
  width: number,
  height: number
): string => {
  if (!series.length) return "";
  const maxVal = Math.max(...series.map((p) => p[key])) || 100;
  const stepX = width / (series.length - 1);
  
  return series
    .map((p, i) => {
      const x = i * stepX;
      // Invert Y because SVG 0 is at top
      const y = height - (p[key] / maxVal) * height * 0.8 - 10; 
      return `${x},${y}`;
    })
    .join(" ");
};

// --- Component ---

const Stats: React.FC<StatsProps> = ({
  overview = {
    totalKm: 0,
    deltaKm: 0,
    missionsCompleted: 0,
    missionSuccessRate: 0,
    avgSpeed: 0,
    operatingHours: 0,
  },
  monthlyMovement = [],
  missionTrend = [],
  batterySeries = [],
  turns = { left: 0, right: 0 },
  statsLoading = false,
  statsError = "",
  lineChartSize = { width: 500, height: 200 },
  buildLinePath,
}) => {
  // 1. Calculate Monthly Movement Stats
  const { totalMovement, avgMovement, maxMonthlyKm } = useMemo(() => {
    const total = monthlyMovement.reduce((acc, curr) => acc + (curr.km || 0), 0);
    const avg = monthlyMovement.length ? total / monthlyMovement.length : 0;
    const max = Math.max(...monthlyMovement.map((m) => m.km || 0)) || 1;
    return { totalMovement: total, avgMovement: avg, maxMonthlyKm: max };
  }, [monthlyMovement]);

  // 2. Calculate Turn Percentages
  const { leftTurns, rightTurns, leftPercent, rightPercent } = useMemo(() => {
    const left = turns.left || 0;
    const right = turns.right || 0;
    const total = left + right || 1; // Prevent div by zero
    return {
      leftTurns: left,
      rightTurns: right,
      leftPercent: Math.round((left / total) * 100),
      rightPercent: Math.round((right / total) * 100),
    };
  }, [turns]);

  // 3. Resolve Line Path Function
  const getPath = (key: "voltage" | "power") => {
    if (buildLinePath) return buildLinePath(batterySeries, key);
    return defaultBuildLinePath(batterySeries, key, lineChartSize.width, lineChartSize.height);
  };

  return (
    <div className="flex flex-col gap-6 p-4 bg-white text-slate-900 dark:bg-slate-950 dark:text-white">
      {/* Header / Status */}
      <div className="flex justify-between text-xs font-semibold text-slate-500 dark:text-slate-400 h-4">
        {statsLoading && (
          <span className="text-indigo-500 animate-pulse">Refreshing telemetry…</span>
        )}
        {statsError && <span className="text-rose-500">{statsError}</span>}
      </div>

      {/* KPI Cards Grid */}
      <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-4">
        {/* Total Distance */}
        <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
          <span className="text-[11px] font-semibold uppercase tracking-wider text-slate-500 dark:text-slate-400">
            Total Moving Distance
          </span>
          <h3 className="text-2xl font-bold text-slate-900 dark:text-white">
            {safeNumber(overview.totalKm).toFixed(1)} km
          </h3>
          <p className="text-sm text-slate-500 dark:text-slate-400">
            +{safeNumber(overview.deltaKm).toFixed(1)} km vs previous day
          </p>
        </div>

        {/* Missions */}
        <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
          <span className="text-[11px] font-semibold uppercase tracking-wider text-slate-500 dark:text-slate-400">
            Missions Completed
          </span>
          <h3 className="text-2xl font-bold">
            {safeNumber(overview.missionsCompleted)}
          </h3>
          <p className="text-sm text-slate-500 dark:text-slate-400">
            {safeNumber(overview.missionSuccessRate)}% success over 7 days
          </p>
        </div>

        {/* Average Speed */}
        <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
          <span className="text-[11px] font-semibold uppercase tracking-wider text-slate-500 dark:text-slate-400">
            Average Speed
          </span>
          <h3 className="text-2xl font-bold">
            {safeNumber(overview.avgSpeed).toFixed(1)} m/s
          </h3>
          <p className="text-sm text-slate-500 dark:text-slate-400">
            Within safe corridor
          </p>
        </div>

        {/* Operating Hours */}
        <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
          <span className="text-[11px] font-semibold uppercase tracking-wider text-slate-500 dark:text-slate-400">
            Operating Hours
          </span>
          <h3 className="text-2xl font-bold">
            {safeNumber(overview.operatingHours)} h
          </h3>
          <p className="text-sm text-slate-500 dark:text-slate-400">
            Last maintenance at 300 h
          </p>
        </div>
      </div>

      {/* Monthly Movement Bar Chart */}
      <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
        <div className="flex items-start justify-between">
          <div>
            <h4 className="text-lg font-semibold text-slate-900 dark:text-white">
              Monthly Movement
            </h4>
            <p className="text-sm text-slate-500 dark:text-slate-400">
              Distance travelled per month
            </p>
          </div>
        </div>
        <div className="mt-4 flex flex-wrap gap-6 text-sm text-slate-500 dark:text-slate-400">
          <div>
            <span className="text-xs uppercase tracking-[.25em]">Total</span>
            <strong className="block text-2xl text-slate-900 dark:text-white">
              {totalMovement.toFixed(1)} km
            </strong>
          </div>
          <div>
            <span className="text-xs uppercase tracking-[.25em]">
              Average / month
            </span>
            <strong className="block text-2xl text-slate-900 dark:text-white">
              {avgMovement.toFixed(1)} km
            </strong>
          </div>
        </div>
        <div className="mt-6 flex gap-2 items-end h-48">
          {monthlyMovement.map((entry) => {
            const height = (entry.km ?? 0) / maxMonthlyKm;
            return (
              <div
                key={entry.month}
                className="flex flex-col items-center gap-2 flex-1"
              >
                <div className="relative w-6 h-full flex items-end">
                  <div
                    className="w-full rounded-t-full bg-gradient-to-b from-sky-400 to-blue-600 transition-all duration-500"
                    style={{ height: `${height * 100}%` }}
                  />
                </div>
                <span className="text-xs text-slate-500 dark:text-slate-400">
                  {entry.month}
                </span>
              </div>
            );
          })}
        </div>
      </div>

      <div className="grid gap-4 lg:grid-cols-2">
        {/* Battery Telemetry Line Chart */}
        <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
          <div className="flex items-start justify-between">
            <div>
              <h4 className="text-lg font-semibold text-slate-900 dark:text-white">
                Battery Voltage & Power
              </h4>
              <p className="text-sm text-slate-500 dark:text-slate-400">
                Live pack telemetry
              </p>
            </div>
            <div className="flex items-center gap-2 text-xs text-slate-500 dark:text-slate-400">
              <span className="flex items-center gap-1">
                <span className="inline-flex h-2 w-2 rounded-full bg-sky-500" />{" "}
                Voltage
              </span>
              <span className="flex items-center gap-1">
                <span className="inline-flex h-2 w-2 rounded-full bg-orange-500" />{" "}
                Power
              </span>
            </div>
          </div>
          <div className="mt-4 overflow-hidden">
            <svg
              viewBox={`0 0 ${lineChartSize.width} ${lineChartSize.height}`}
              className="w-full h-auto"
              role="img"
              aria-label="Battery voltage and power line plot"
            >
              {/* Grid Lines */}
              {[0.25, 0.5, 0.75, 1].map((ratio) => (
                <line
                  key={String(ratio)}
                  x1="0"
                  x2={lineChartSize.width}
                  y1={lineChartSize.height * ratio}
                  y2={lineChartSize.height * ratio}
                  className="stroke-slate-200 dark:stroke-slate-600"
                  strokeDasharray="4 4"
                />
              ))}
              <polyline
                points={getPath("voltage")}
                className="fill-none stroke-sky-400"
                strokeWidth="3"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
              <polyline
                points={getPath("power")}
                className="fill-none stroke-orange-500"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
            <div className="mt-3 flex justify-between text-xs text-slate-500 dark:text-slate-400">
              {batterySeries.map((point) => (
                <span key={point.time}>{point.time}</span>
              ))}
            </div>
          </div>
        </div>

        {/* Turn Distribution */}
        <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
          <div className="flex items-start justify-between">
            <div>
              <h4 className="text-lg font-semibold text-slate-900 dark:text-white">
                Turn Distribution
              </h4>
              <p className="text-sm text-slate-500 dark:text-slate-400">
                Left/right turns this shift
              </p>
            </div>
          </div>
          <div className="mt-4 flex flex-col gap-4 text-sm text-slate-500 dark:text-slate-400">
            {/* Left */}
            <div className="space-y-1">
              <div className="flex items-center justify-between">
                <span>Left turns</span>
                <strong className="text-slate-900 dark:text-white">
                  {leftTurns}
                </strong>
              </div>
              <div className="h-3 w-full rounded-full bg-slate-200/60 dark:bg-slate-600/60 overflow-hidden">
                <div
                  className="h-full rounded-full bg-gradient-to-r from-sky-500 to-blue-500 transition-all duration-500"
                  style={{ width: `${leftPercent}%` }}
                />
              </div>
            </div>

            {/* Right */}
            <div className="space-y-1">
              <div className="flex items-center justify-between">
                <span>Right turns</span>
                <strong className="text-slate-900 dark:text-white">
                  {rightTurns}
                </strong>
              </div>
              <div className="h-3 w-full rounded-full bg-slate-200/60 dark:bg-slate-600/60 overflow-hidden">
                <div
                  className="h-full rounded-full bg-gradient-to-r from-amber-400 to-orange-500 transition-all duration-500"
                  style={{ width: `${rightPercent}%` }}
                />
              </div>
            </div>

            <div className="text-right text-xs font-semibold text-slate-500 dark:text-slate-400">
              Left {leftPercent}% · Right {rightPercent}%
            </div>
          </div>
        </div>
      </div>

      {/* Mission Table */}
      <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
        <div className="flex items-start justify-between">
          <div>
            <h4 className="text-lg font-semibold text-slate-900 dark:text-white">
              Mission Trend
            </h4>
            <p className="text-sm text-slate-500 dark:text-slate-400">
              Completion vs incidents
            </p>
          </div>
        </div>
        <div className="mt-4 overflow-x-auto">
          <table className="min-w-full divide-y divide-slate-200 text-sm dark:divide-slate-700">
            <thead className="bg-slate-50 text-[11px] uppercase tracking-[.3em] text-slate-500 dark:bg-slate-800 dark:text-slate-400">
              <tr>
                <th className="px-3 py-2 text-left font-medium">Day</th>
                <th className="px-3 py-2 text-left font-medium">Completed</th>
                <th className="px-3 py-2 text-left font-medium">Incidents</th>
              </tr>
            </thead>
            <tbody className="divide-y divide-slate-200 text-slate-900 dark:divide-slate-700 dark:text-white">
              {missionTrend.map((entry) => (
                <tr key={entry.label}>
                  <td className="px-3 py-2">{entry.label}</td>
                  <td className="px-3 py-2 font-semibold">{entry.completed}</td>
                  <td className="px-3 py-2">{entry.incidents}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );
};

export default Stats;