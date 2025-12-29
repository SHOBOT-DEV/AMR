import React from "react";

interface DiagnosticPanel {
  id: string;
  title: string;
  value: string;
  status: "Nominal" | "Monitoring" | "Warning" | "Critical";
  detail: string;
}

interface DiagnosticsProps {
  diagnosticsPanels?: DiagnosticPanel[];
}

const Diagnostics: React.FC<DiagnosticsProps> = ({
  diagnosticsPanels = [
    {
      id: "battery",
      title: "Battery Health",
      value: "93%",
      status: "Nominal",
      detail: "Cells balanced",
    },
    {
      id: "motors",
      title: "Drive Motors",
      value: "Temp 48°C",
      status: "Monitoring",
      detail: "Torque variance +3%",
    },
    {
      id: "sensors",
      title: "Sensor Suite",
      value: "All online",
      status: "Nominal",
      detail: "Last calibration 12h ago",
    },
  ],
}) => {
  // Status color mapping
  const getStatusColor = (status: DiagnosticPanel["status"]) => {
    switch (status) {
      case "Nominal":
        return "text-green-500";
      case "Monitoring":
        return "text-sky-500";
      case "Warning":
        return "text-yellow-500";
      case "Critical":
        return "text-red-500";
      default:
        return "text-slate-500";
    }
  };

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-3">
      {diagnosticsPanels.map((panel) => (
        <div
          key={panel.id}
          className="bg-white border border-slate-200 rounded-xl p-4 shadow-sm hover:shadow-lg transition-shadow"
        >
          <h4 className="text-base font-semibold text-slate-900 mb-2">
            {panel.title}
          </h4>
          <div className="text-xl font-bold text-slate-800 my-2">
            {panel.value}
          </div>
          <span
            className={`text-xs font-bold uppercase tracking-wider ${getStatusColor(
              panel.status
            )}`}
          >
            {panel.status}
          </span>
          <p className="mt-2 text-sm text-slate-600 leading-relaxed">
            {panel.detail}
          </p>
        </div>
      ))}
    </div>
  );
};

export default Diagnostics;