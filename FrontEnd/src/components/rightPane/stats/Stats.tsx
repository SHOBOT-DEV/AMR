import React from "react";
import { useStats } from "../../../context/StatsContext.tsx";

const Stats = (props: any) => {
  const { rightPage } = props;

  // ✅ Get data ONLY from context
  const { statsData, statsLoading, statsError } = useStats();

  const { overview, monthlyMovement, missionTrend, batterySeries, turns } =
    statsData;

  const lineChartSize = { width: 420, height: 180 };

  const buildLinePath = (series: any[], key: string) => {
    if (!series.length) return "";
    const max = Math.max(...series.map((p) => p[key]));
    const min = Math.min(...series.map((p) => p[key]));
    const range = max - min || 1;
    const stepX =
      series.length > 1
        ? lineChartSize.width / (series.length - 1)
        : lineChartSize.width;

    return series
      .map((p, i) => {
        const x = i * stepX;
        const y =
          lineChartSize.height -
          ((p[key] - min) / range) * lineChartSize.height;
        return `${x},${y}`;
      })
      .join(" ");
  };

  const totalMonthlyKm = monthlyMovement.reduce(
    (sum: number, e: any) => sum + e.km,
    0,
  );
  const avgMonthlyKm = monthlyMovement.length
    ? (totalMonthlyKm / monthlyMovement.length).toFixed(1)
    : "0.0";
  const maxMonthlyKm = Math.max(...monthlyMovement.map((m: any) => m.km), 1);

  return (
    <>
      {rightPage === "stats" && (
        <div className="flex flex-col gap-4 p-4">
          {/* Status Row */}
          <div className="flex justify-between text-[13px] text-slate-600">
            {statsLoading && (
              <span className="text-[#0b74d1] animate-pulse">
                Refreshing telemetry…
              </span>
            )}
            {statsError && (
              <span className="text-[#c2410c]">{statsError}</span>
            )}
          </div>

          {/* Summary Cards */}
          <div className="grid grid-cols-[repeat(auto-fit,minmax(160px,1fr))] gap-3">
            <div className="bg-slate-50 rounded-xl p-3 shadow-inner border">
              <span className="text-[12px] uppercase tracking-wider text-slate-400">
                Total Moving Distance
              </span>
              <h3 className="text-[24px] mt-1">
                {overview.totalKm.toFixed(1)} km
              </h3>
              <p className="text-[13px] text-slate-500">
                +{overview.deltaKm} km vs previous day
              </p>
            </div>

            <div className="bg-slate-50 rounded-xl p-3 shadow-inner border">
              <span className="text-[12px] uppercase tracking-wider text-slate-400">
                Missions Completed
              </span>
              <h3 className="text-[24px] mt-1">
                {overview.missionsCompleted}
              </h3>
              <p className="text-[13px] text-slate-500">
                {overview.missionSuccessRate}% success
              </p>
            </div>

            <div className="bg-slate-50 rounded-xl p-3 shadow-inner border">
              <span className="text-[12px] uppercase tracking-wider text-slate-400">
                Average Speed
              </span>
              <h3 className="text-[24px] mt-1">
                {overview.avgSpeed.toFixed(1)} m/s
              </h3>
            </div>

            <div className="bg-slate-50 rounded-xl p-3 shadow-inner border">
              <span className="text-[12px] uppercase tracking-wider text-slate-400">
                Operating Hours
              </span>
              <h3 className="text-[24px] mt-1">
                {overview.operatingHours} h
              </h3>
            </div>
          </div>

          {/* Monthly Movement */}
          <div className="bg-white rounded-xl p-4 shadow-lg">
            <h4 className="font-semibold">Monthly Movement</h4>

            <div className="flex gap-6 text-sm my-3">
              <div>
                <span className="uppercase text-xs text-slate-400">Total</span>
                <strong className="block">{totalMonthlyKm.toFixed(1)} km</strong>
              </div>
              <div>
                <span className="uppercase text-xs text-slate-400">
                  Avg / month
                </span>
                <strong className="block">{avgMonthlyKm} km</strong>
              </div>
            </div>

            <div className="flex items-end gap-3 h-40">
              {monthlyMovement.map((m: any) => (
                <div key={m.month} className="flex flex-col items-center text-xs">
                  <div
                    className="w-6 bg-sky-500 rounded-t"
                    style={{ height: `${(m.km / maxMonthlyKm) * 100}%` }}
                  />
                  <span>{m.month}</span>
                </div>
              ))}
            </div>
          </div>

          {/* Battery Chart */}
          <div className="bg-white rounded-xl p-4 shadow-lg">
            <h4 className="font-semibold mb-2">
              Battery Voltage & Power
            </h4>

            <svg width={lineChartSize.width} height={lineChartSize.height}>
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

          {/* Turn Distribution */}
          <div className="bg-white rounded-xl p-4 shadow-lg">
            <h4 className="font-semibold mb-3">Turn Distribution</h4>

            {["left", "right"].map((side) => {
              const value = turns[side];
              const total = turns.left + turns.right || 1;
              return (
                <div key={side} className="mb-3">
                  <div className="flex justify-between text-sm">
                    <span>{side.toUpperCase()}</span>
                    <strong>{value}</strong>
                  </div>
                  <div className="h-3 bg-slate-100 rounded-full">
                    <div
                      className="h-full bg-sky-500 rounded-full"
                      style={{ width: `${(value / total) * 100}%` }}
                    />
                  </div>
                </div>
              );
            })}
          </div>

          {/* Mission Trend */}
          <div className="bg-white rounded-xl p-4 shadow-lg">
            <h4 className="font-semibold mb-3">Mission Trend</h4>

            <table className="w-full text-sm">
              <thead className="text-slate-500">
                <tr>
                  <th className="text-left">Day</th>
                  <th className="text-right">Completed</th>
                  <th className="text-right">Incidents</th>
                </tr>
              </thead>
              <tbody>
                {missionTrend.map((m: any) => (
                  <tr key={m.label} className="border-t">
                    <td>{m.label}</td>
                    <td className="text-right">{m.completed}</td>
                    <td className="text-right">{m.incidents}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </div>
      )}
    </>
  );
};

export default Stats;
