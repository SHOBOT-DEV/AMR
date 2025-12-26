import React, { useState } from "react";

  const [diagnosticsPanels, setDiagnosticsPanels] = useState([
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
  ]);

//  RightPane logic
    const {
    rightPage,
    setRightPage,
    diagnosticsPanels,
    } = props;

  // UI

        {/* DIAGNOSTICS */}
        {rightPage === "diagnostics" && (
          <div className="diagnostics-pane">
            {diagnosticsPanels.map((panel) => (
              <div key={panel.id} className="diag-card">
                <h4>{panel.title}</h4>
                <div className="diag-value">{panel.value}</div>
                <span className="diag-status">{panel.status}</span>
                <p>{panel.detail}</p>
              </div>
            ))}
          </div>
        )}
