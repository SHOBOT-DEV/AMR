import React from "react";

interface MissionLogEntry {
  id: string;
  mission: string;
  window: string;
  outcome: "Completed" | "Delayed";
  notes: string;
}

interface MissionLogsProps {
  missionHistory?: MissionLogEntry[];
}

const MissionLogs: React.FC<MissionLogsProps> = ({
  missionHistory,
}) => {
  const defaultMissionHistory: MissionLogEntry[] = [
    {
      id: "mh1",
      mission: "Inspect Zone A",
      window: "08:00–08:18",
      outcome: "Completed",
      notes: "No issues",
    },
    {
      id: "mh2",
      mission: "Delivery Route 3",
      window: "08:30–09:10",
      outcome: "Delayed",
      notes: "Obstacle at Dock Tunnel",
    },
    {
      id: "mh3",
      mission: "Battery Check",
      window: "09:15–09:32",
      outcome: "Completed",
      notes: "Pack swap verified",
    },
  ];

  const missions = missionHistory || defaultMissionHistory;

  return (
    <div className="flex flex-col gap-3">
      {missions.map((entry) => (
        <div
          key={entry.id}
          className="bg-white border border-slate-200 rounded-lg p-4 shadow-sm hover:shadow-md transition-shadow"
        >
          <div className="flex justify-between items-start mb-2">
            <strong className="text-sm font-semibold text-slate-900">
              {entry.mission}
            </strong>
            <span className="text-xs text-slate-500 bg-slate-50 px-2 py-1 rounded">
              {entry.window}
            </span>
          </div>
          <div className="flex items-center gap-2 mb-2">
            <span
              className={`inline-block px-3 py-1 rounded-full text-xs font-semibold uppercase tracking-wide ${
                entry.outcome === "Completed"
                  ? "bg-emerald-100 text-emerald-700"
                  : "bg-red-100 text-red-700"
              }`}
            >
              {entry.outcome}
            </span>
          </div>
          <p className="text-sm text-slate-600 leading-relaxed">
            {entry.notes}
          </p>
        </div>
      ))}
    </div>
  );
};

export default MissionLogs;