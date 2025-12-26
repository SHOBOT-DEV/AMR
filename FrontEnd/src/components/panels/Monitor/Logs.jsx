import React, { useState } from "react";

  const [logEvents, setLogEvents] = useState([
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
  ]);

//   RightPane logic
const{
    rightPage,
    setRightPage,
    logEvents,
} = props;

// UI

        {/* LOGS */}
        {rightPage === "logs" && (
          <div className="logs-pane">
            <table>
              <thead>
                <tr>
                  <th>Time</th>
                  <th>Component</th>
                  <th>Message</th>
                  <th>Level</th>
                </tr>
              </thead>
              <tbody>
                {logEvents.map((event) => (
                  <tr key={event.id}>
                    <td>{event.ts}</td>
                    <td>{event.system}</td>
                    <td>{event.message}</td>
                    <td><span className={`log-pill ${event.level}`}>{event.level}</span></td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}
  