
import React from "react";

type MovementEntry = {
  month: string;
  km?: number;
};

type TrendEntry = {
  label: string;
  completed: number;
  incidents: number;
};

type BatteryPoint = {
  time: string;
  voltage: number;
  power: number;
};

type Overview = {
  totalKm: number;
  missionsCompleted: number;
  avgSpeed: number;
  operatingHours: number;
  deltaKm?: number;
  missionSuccessRate?: number;
};

type StatsProps = {
  overview?: Overview;
  missionTrend?: TrendEntry[];
  monthlyMovement?: MovementEntry[];
  batterySeries?: BatteryPoint[];
  turns?: { left?: number; right?: number };
  statsLoading?: boolean;
  statsError?: string;
  lineChartSize?: { width: number; height: number };
  buildLinePath?: (series: BatteryPoint[], key: "voltage" | "power") => string;
};

const DEFAULT_OVERVIEW: Overview = {
  totalKm: 0,
  missionsCompleted: 0,
  avgSpeed: 0,
  operatingHours: 0,
  deltaKm: 0,
  missionSuccessRate: 0,
};

const DEFAULT_TURNS = { left: 0, right: 0 };
const DEFAULT_LINE_SIZE = { width: 420, height: 180 };

const safeNumber = (value?: number) => (typeof value === "number" ? value : 0);

const Stats1 = ({
  overview = DEFAULT_OVERVIEW,
  missionTrend = [],
  monthlyMovement = [],
  batterySeries = [],
  turns = DEFAULT_TURNS,
  statsLoading = false,
  statsError = "",
  lineChartSize = DEFAULT_LINE_SIZE,
  buildLinePath = () => "",
}: StatsProps) => {
  const totalMovement = monthlyMovement.reduce((sum, entry) => sum + (entry.km || 0), 0);
  const avgMovement = monthlyMovement.length ? totalMovement / monthlyMovement.length : 0;
  const maxMonthlyKm = monthlyMovement.length
    ? Math.max(...monthlyMovement.map((entry) => entry.km || 0))
    : 1;
  const totalTurns = (turns.left || 0) + (turns.right || 0) || 1;

  return (
    <div className="flex flex-col gap-6 p-4 bg-white text-slate-900 dark:bg-slate-950 dark:text-white">
      <div className="flex justify-between text-xs font-semibold text-slate-500 dark:text-slate-400">
        {statsLoading && <span className="text-indigo-500">Refreshing telemetry…</span>}
        {statsError && <span className="text-rose-500">{statsError}</span>}
      </div>

      <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-4">
        <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
          <span className="text-[11px] font-semibold uppercase tracking-wider text-slate-500 dark:text-slate-400">
            Total Moving Distance
          </span>
          <h3 className="text-2xl font-bold text-slate-900 dark:text-white">{safeNumber(overview.totalKm).toFixed(1)} km</h3>
          <p className="text-sm text-slate-500 dark:text-slate-400">
            +{safeNumber(overview.deltaKm).toFixed(1)} km vs previous day
          </p>
        </div>
        <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
          <span className="text-[11px] font-semibold uppercase tracking-wider text-slate-500 dark:text-slate-400">
            Missions Completed
          </span>
          <h3 className="text-2xl font-bold">{safeNumber(overview.missionsCompleted)}</h3>
          <p className="text-sm text-slate-500 dark:text-slate-400">{safeNumber(overview.missionSuccessRate)}% success over 7 days</p>
        </div>
        <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
          <span className="text-[11px] font-semibold uppercase tracking-wider text-slate-500 dark:text-slate-400">Average Speed</span>
          <h3 className="text-2xl font-bold">{safeNumber(overview.avgSpeed).toFixed(1)} m/s</h3>
          <p className="text-sm text-slate-500 dark:text-slate-400">Within safe corridor</p>
        </div>
        <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
          <span className="text-[11px] font-semibold uppercase tracking-wider text-slate-500 dark:text-slate-400">
            Operating Hours
          </span>
          <h3 className="text-2xl font-bold">{safeNumber(overview.operatingHours)} h</h3>
          <p className="text-sm text-slate-500 dark:text-slate-400">Last maintenance at 300 h</p>
        </div>
      </div>

      <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
        <div className="flex items-start justify-between">
          <div>
            <h4 className="text-lg font-semibold text-slate-900 dark:text-white">Monthly Movement</h4>
            <p className="text-sm text-slate-500 dark:text-slate-400">Distance travelled per month</p>
          </div>
        </div>
        <div className="mt-4 flex flex-wrap gap-6 text-sm text-slate-500 dark:text-slate-400">
          <div>
            <span className="text-xs uppercase tracking-[.25em]">Total</span>
            <strong className="block text-2xl text-slate-900 dark:text-white">{totalMovement.toFixed(1)} km</strong>
          </div>
          <div>
            <span className="text-xs uppercase tracking-[.25em]">Average / month</span>
            <strong className="block text-2xl text-slate-900 dark:text-white">{avgMovement.toFixed(1)} km</strong>
          </div>
        </div>
        <div className="mt-6 flex gap-2">
          {monthlyMovement.map((entry) => {
            const height = maxMonthlyKm ? (entry.km || 0) / maxMonthlyKm : 0;
            return (
              <div key={entry.month} className="flex flex-col items-center gap-2">
                <div
                  className="h-36 w-6 rounded-t-full bg-gradient-to-b from-sky-400 to-blue-600"
                  style={{ height: `${height * 100}%` }}
                />
                <span className="text-xs text-slate-500 dark:text-slate-400">{entry.month}</span>
              </div>
            );
          })}
        </div>
      </div>

      <div className="grid gap-4 lg:grid-cols-2">
        <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
          <div className="flex items-start justify-between">
            <div>
              <h4 className="text-lg font-semibold text-slate-900 dark:text-white">Battery Voltage & Power</h4>
              <p className="text-sm text-slate-500 dark:text-slate-400">Live pack telemetry</p>
            </div>
            <div className="flex items-center gap-2 text-xs text-slate-500 dark:text-slate-400">
              <span className="flex items-center gap-1">
                <span className="inline-flex h-2 w-2 rounded-full bg-sky-500" /> Voltage
              </span>
              <span className="flex items-center gap-1">
                <span className="inline-flex h-2 w-2 rounded-full bg-orange-500" /> Power
              </span>
            </div>
          </div>
          <div className="mt-4">
            <svg width={lineChartSize.width} height={lineChartSize.height} role="img" aria-label="Battery voltage and power line plot">
              {[0.25, 0.5, 0.75, 1].map((ratio) => (
                <line
                  key={String(ratio)}
                  x1="0"
                  x2={lineChartSize.width}
                  y1={lineChartSize.height * ratio}
                  y2={lineChartSize.height * ratio}
                className="stroke-slate-200 dark:stroke-slate-600"
                />
              ))}
              <polyline
                points={buildLinePath(batterySeries, "voltage")}
                className="fill-none stroke-sky-400"
                strokeWidth="3"
              />
              <polyline
                points={buildLinePath(batterySeries, "power")}
                className="fill-none stroke-orange-500"
                strokeWidth="2"
              />
            </svg>
            <div className="mt-3 flex justify-between text-xs text-slate-500 dark:text-slate-400">
              {batterySeries.map((point) => (
                <span key={point.time}>{point.time}</span>
              ))}
            </div>
          </div>
        </div>

        <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
          <div className="flex items-start justify-between">
            <div>
              <h4 className="text-lg font-semibold text-slate-900 dark:text-white">Turn Distribution</h4>
            <p className="text-sm text-slate-500 dark:text-slate-400">Left/right turns this shift</p>
            </div>
          </div>
          <div className="mt-4 flex flex-col gap-2 text-sm text-slate-500 dark:text-slate-400">
            <div className="flex items-center justify-between">
              <span>Left turns</span>
              <strong className="text-slate-900 dark:text-white">{turns.left}</strong>
            </div>
            <div className="h-3 w-full rounded-full bg-slate-200/60 dark:bg-slate-600/60">
              <div
                className="h-full rounded-full bg-gradient-to-r from-sky-500 to-blue-500"
                style={{ width: `${Math.round((turns.left / totalTurns) * 100)}%` }}
              />
            </div>
            <div className="flex items-center justify-between">
              <span>Right turns</span>
              <strong className="text-slate-900 dark:text-white">{turns.right}</strong>
            </div>
            <div className="h-3 w-full rounded-full bg-slate-200/60 dark:bg-slate-600/60">
              <div
                className="h-full rounded-full bg-gradient-to-r from-amber-400 to-orange-500"
                style={{ width: `${Math.round((turns.right / totalTurns) * 100)}%` }}
              />
            </div>
            <div className="text-right text-xs font-semibold text-slate-500 dark:text-slate-400">
              Left {Math.round((turns.left / totalTurns) * 100)}% · Right{" "}
              {Math.round((turns.right / totalTurns) * 100)}%
            </div>
          </div>
        </div>
      </div>

      <div className="rounded-2xl border border-slate-200 bg-slate-50 p-4 shadow-sm dark:border-slate-700 dark:bg-slate-900">
        <div className="flex items-start justify-between">
          <div>
            <h4 className="text-lg font-semibold text-slate-900 dark:text-white">Mission Trend</h4>
            <p className="text-sm text-slate-500 dark:text-slate-400">Completion vs incidents</p>
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

export default Stats1;
