import React from "react";

const AnalyticsView = ({ stats }) => {
  const { overview, batterySeries } = stats;

  return (
    <div className="p-6 grid grid-cols-1 gap-6">
      
      {/* Overview Cards */}
      <div className="grid grid-cols-2 gap-4">
        {[
          { label: "Total Km", value: overview.totalKm, unit: "km", trend: "+4.3" },
          { label: "Missions", value: overview.missionsCompleted, unit: "", trend: "98% rate" },
          { label: "Avg Speed", value: overview.avgSpeed, unit: "m/s", trend: "Normal" },
          { label: "Hours", value: overview.operatingHours, unit: "h", trend: "Maintenance OK" },
        ].map((item, idx) => (
          <div key={idx} className="bg-white p-5 rounded-xl border border-gray-200 shadow-sm flex flex-col gap-1">
            <span className="text-xs font-bold uppercase tracking-wider text-slate-400">{item.label}</span>
            <div className="flex items-baseline gap-1">
              <span className="text-2xl font-bold text-slate-800">{item.value}</span>
              <span className="text-sm text-slate-500 font-medium">{item.unit}</span>
            </div>
            <span className="text-xs text-emerald-600 font-medium mt-1">{item.trend}</span>
          </div>
        ))}
      </div>

      {/* Chart Section (Placeholder for SVG) */}
      <div className="bg-white p-6 rounded-xl border border-gray-200 shadow-sm">
        <h3 className="text-lg font-bold text-slate-800 mb-4">Battery Telemetry</h3>
        <div className="h-48 w-full bg-slate-50 rounded-lg flex items-end justify-between px-4 pb-4 border border-dashed border-slate-300">
           {/* Simple bar visualizer for demo */}
           {batterySeries.map((p, i) => (
             <div key={i} className="flex flex-col items-center gap-2 group cursor-pointer w-full">
                <div 
                  className="w-full max-w-[20px] bg-blue-500 rounded-t-sm opacity-80 group-hover:opacity-100 transition-all" 
                  style={{ height: `${(p.voltage / 60) * 100}%` }} 
                />
                <span className="text-[10px] text-slate-400">{p.time}</span>
             </div>
           ))}
        </div>
      </div>
    </div>
  );
};

export default AnalyticsView;