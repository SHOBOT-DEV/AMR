import React, { useState } from "react";

const [missionHistory, setMissionHistory] = useState([
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
  ]);

//   RightPane logic
    const {
        rightPage,
        setRightPage,
        // mission logs
        missionHistory,
    } = props;

// UI

 {/* MISSION LOGS */}
        {rightPage === "mission-logs" && (
          <div className="mission-log-pane">
            {missionHistory.map((entry) => (
              <div key={entry.id} className="timeline-card">
                <div className="timeline-header">
                  <strong>{entry.mission}</strong>
                  <span>{entry.window}</span>
                </div>
                <div className={`timeline-status ${entry.outcome === "Completed" ? "success" : "warn"}`}>{entry.outcome}</div>
                <p>{entry.notes}</p>
              </div>
            ))}
          </div>
        )}