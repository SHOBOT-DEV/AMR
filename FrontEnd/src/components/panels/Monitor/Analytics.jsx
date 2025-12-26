import React, { useState } from "react";

  const [analyticsSummary, setAnalyticsSummary] = useState([
    { label: "Incidents", value: 2, trend: "+1 vs last week" },
    { label: "Stops Issued", value: 14, trend: "-3 vs last week" },
    { label: "Battery Swaps", value: 5, trend: "Stable" },
    { label: "Avg. Cycle", value: "42 min", trend: "±0" },
  ]);

  const [analyticsSeries, setAnalyticsSeries] = useState([
    12, 18, 22, 16, 24, 26, 20,
  ]);
  const [analyticsAlerts, setAnalyticsAlerts] = useState([
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
  ]);

//   RightPane logic
  const {
    rightPage,
    setRightPage,
    // analytics
    analyticsSummary,
    analyticsSeries,
    analyticsAlerts,
    } = props;

// UI
        {/* ANALYTICS */}
        {rightPage === "analytics" && (
          <div className="analytics-pane">
            <div className="analytics-kpis">
              {analyticsSummary.map((card) => (
                <div key={card.label} className="kpi-card" style={{ background: "var(--card-bg)", color: "var(--text-color)", border: "1px solid var(--card-border)" }}>
                  <span className="kpi-label">{card.label}</span>
                  <strong>{card.value}</strong>
                  <span className="kpi-trend">{card.trend}</span>
                </div>
              ))}
            </div>
            <div className="analytics-chart-card" style={{ background: "var(--card-bg)", color: "var(--text-color)" }}>
              <div className="stats-card-header">
                <div>
                  <h4>Cycle Throughput</h4>
                  <p>Rolling seven-day view</p>
                </div>
              </div>
              <svg width={analyticsChartSize.width} height={analyticsChartSize.height} className="analytics-chart">
                <polyline points={analyticsPath} fill="none" strokeWidth="3" className="analytics-line" />
              </svg>
            </div>
            <div className="analytics-alerts">
              {analyticsAlerts.map((alert) => (
                <div key={alert.id} className="alert-card">
                  <strong>{alert.title}</strong>
                  <p>{alert.detail}</p>
                </div>
              ))}
            </div>
          </div>
        )}