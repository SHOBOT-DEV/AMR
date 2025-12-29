import React, { useState, useMemo } from "react";

interface AnalyticsSummaryCard {
  label: string;
  value: number | string;
  trend: string;
}

interface AnalyticsAlert {
  id: string;
  title: string;
  detail: string;
}

interface AnalyticsProps {
  analyticsSummary?: AnalyticsSummaryCard[];
  analyticsSeries?: number[];
  analyticsAlerts?: AnalyticsAlert[];
  analyticsChartSize?: { width: number; height: number };
  analyticsPath?: string;
}

const Analytics: React.FC<AnalyticsProps> = ({
  analyticsSummary = [
    { label: "Incidents", value: 2, trend: "+1 vs last week" },
    { label: "Stops Issued", value: 14, trend: "-3 vs last week" },
    { label: "Battery Swaps", value: 5, trend: "Stable" },
    { label: "Avg. Cycle", value: "42 min", trend: "±0" },
  ],
  analyticsSeries = [12, 18, 22, 16, 24, 26, 20],
  analyticsAlerts = [
    {
      id: "alert1",
      title: "Obstacle spikes",
      detail: "Lidar reported 5 high-density events on Dock Tunnel.",
    },
    {
      id: "alert2",
      title: "Slow return",
      detail: "Mission Delivery Route 3 exceeded SLA by 4 min.",
    },
  ],
  analyticsChartSize = { width: 600, height: 200 },
  analyticsPath = "0,180 86,140 172,90 258,130 344,50 430,30 516,90",
}) => {
  // Generate chart path from series data
  const generateChartPath = useMemo(() => {
    const height = analyticsChartSize.height;
    const maxValue = Math.max(...analyticsSeries);
    const step = analyticsChartSize.width / (analyticsSeries.length - 1);

    return analyticsSeries
      .map((value, index) => {
        const x = index * step;
        const y = height - (value / maxValue) * (height - 20);
        return `${x},${y}`;
      })
      .join(" ");
  }, [analyticsSeries, analyticsChartSize]);

  return (
    <div className="flex flex-col gap-4">
      {/* KPI Cards */}
      <div className="grid grid-cols-2 lg:grid-cols-4 gap-3">
        {analyticsSummary.map((card) => (
          <div
            key={card.label}
            className="bg-slate-50 border border-slate-200 rounded-lg p-3 shadow-sm hover:shadow-md transition-shadow"
          >
            <span className="text-xs font-bold text-slate-400 uppercase tracking-widest block">
              {card.label}
            </span>
            <strong className="text-2xl text-slate-900 block my-1">
              {card.value}
            </strong>
            <span className="text-xs text-slate-600">
              {card.trend}
            </span>
          </div>
        ))}
      </div>

      {/* Analytics Chart Card */}
      <div className="bg-white border border-slate-200 rounded-lg p-4 shadow-sm">
        <div className="mb-4">
          <h4 className="text-lg font-bold text-slate-900">Cycle Throughput</h4>
          <p className="text-sm text-slate-500">Rolling seven-day view</p>
        </div>
        <div className="w-full overflow-x-auto">
          <svg
            width={analyticsChartSize.width}
            height={analyticsChartSize.height}
            viewBox={`0 0 ${analyticsChartSize.width} ${analyticsChartSize.height}`}
            className="w-full"
          >
            {/* Grid lines */}
            <defs>
              <pattern
                id="grid"
                width="60"
                height="40"
                patternUnits="userSpaceOnUse"
              >
                <path
                  d={`M 60 0 L 0 0 0 40`}
                  fill="none"
                  stroke="#e2e8f0"
                  strokeWidth="0.5"
                />
              </pattern>
            </defs>
            <rect width={analyticsChartSize.width} height={analyticsChartSize.height} fill="url(#grid)" />
            {/* Chart line */}
            <polyline
              points={generateChartPath}
              fill="none"
              stroke="#6366f1"
              strokeWidth="3"
              strokeLinecap="round"
              strokeLinejoin="round"
            />
          </svg>
        </div>
      </div>

      {/* Alerts Section */}
      <div className="flex flex-col gap-2">
        {analyticsAlerts.map((alert) => (
          <div
            key={alert.id}
            className="bg-orange-50 border border-orange-200 rounded-lg p-3 hover:shadow-md transition-shadow"
          >
            <strong className="text-slate-900 block mb-1">{alert.title}</strong>
            <p className="text-sm text-slate-700">{alert.detail}</p>
          </div>
        ))}
      </div>
    </div>
  );
};

export default Analytics;