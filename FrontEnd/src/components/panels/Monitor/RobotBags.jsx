import React, { useState } from "react";

  const [bagFiles, setBagFiles] = useState([
    {
      id: "bag1",
      name: "mission-0915.bag",
      duration: "15m",
      size: "1.4 GB",
      status: "Uploaded",
    },
    {
      id: "bag2",
      name: "mission-1030.bag",
      duration: "26m",
      size: "2.7 GB",
      status: "Processing",
    },
  ]);

//   RightPane logic
    const {
    rightPage,
    setRightPage,
    bagFiles,
    } = props;

// UI
 {/* ROBOT BAGS */}
        {rightPage === "robot-bags" && (
          <div className="bags-pane">
            <table>
              <thead>
                <tr>
                  <th>Filename</th>
                  <th>Duration</th>
                  <th>Size</th>
                  <th>Status</th>
                  <th></th>
                </tr>
              </thead>
              <tbody>
                {bagFiles.map((bag) => (
                  <tr key={bag.id}>
                    <td>{bag.name}</td>
                    <td>{bag.duration}</td>
                    <td>{bag.size}</td>
                    <td><span className={`bag-pill ${bag.status.toLowerCase()}`}>{bag.status}</span></td>
                    <td><button className="ghost-btn">Download</button></td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}

        