import React from "react";

interface LogEvent {
  id: string;
  ts: string;
  system: string;
  message: string;
  level: "info" | "warn" | "error" | "debug";
}

interface LogsProps {
  logEvents?: LogEvent[];
}

const Logs: React.FC<LogsProps> = ({
  logEvents = [
    {
      id: "log1",
      ts: "10:42:01",
      system: "Navigation",
      message: "Replanned path around blocked aisle",
      level: "info",
    },
    {
      id: "log2",
      ts: "10:15:22",
      system: "Safety",
      message: "Emergency stop acknowledged",
      level: "warn",
    },
    {
      id: "log3",
      ts: "09:57:10",
      system: "Battery",
      message: "Pack voltage dipped to 45.9V",
      level: "warn",
    },
  ],
}) => {
  // Log level color mapping
  const getLogLevelStyle = (level: LogEvent["level"]) => {
    switch (level) {
      case "info":
        return "bg-sky-100 text-sky-700";
      case "warn":
        return "bg-yellow-100 text-yellow-700";
      case "error":
        return "bg-red-100 text-red-700";
      case "debug":
        return "bg-gray-100 text-gray-700";
      default:
        return "bg-slate-100 text-slate-700";
    }
  };

  return (
    <div className="bg-white border border-slate-200 rounded-xl overflow-hidden shadow-sm">
      <div className="overflow-x-auto">
        <table className="w-full">
          <thead className="bg-slate-50">
            <tr>
              <th className="px-4 py-3 text-left text-xs font-semibold text-slate-700 uppercase tracking-wider">
                Time
              </th>
              <th className="px-4 py-3 text-left text-xs font-semibold text-slate-700 uppercase tracking-wider">
                Component
              </th>
              <th className="px-4 py-3 text-left text-xs font-semibold text-slate-700 uppercase tracking-wider">
                Message
              </th>
              <th className="px-4 py-3 text-left text-xs font-semibold text-slate-700 uppercase tracking-wider">
                Level
              </th>
            </tr>
          </thead>
          <tbody className="divide-y divide-slate-200">
            {logEvents.map((event) => (
              <tr key={event.id} className="hover:bg-slate-50 transition-colors">
                <td className="px-4 py-3 text-sm font-mono text-slate-600">
                  {event.ts}
                </td>
                <td className="px-4 py-3 text-sm font-medium text-slate-900">
                  {event.system}
                </td>
                <td className="px-4 py-3 text-sm text-slate-700">
                  {event.message}
                </td>
                <td className="px-4 py-3 text-sm">
                  <span
                    className={`inline-flex items-center px-2.5 py-1 rounded-full text-xs font-semibold uppercase tracking-wide ${getLogLevelStyle(
                      event.level
                    )}`}
                  >
                    {event.level}
                  </span>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default Logs;