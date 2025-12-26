// Header tag
      <header className="mp-header">
        {/* left header group (kept minimal) */}
        <div style={{ display: "flex", alignItems: "center", gap: 12 }}>
          {/* ...existing left items (kept empty or for future icons) ... */}
        </div>
      </header>

import React, { useCallback } from "react";
import { fetchWithAuth } from "../../utils/auth";
import { API_V1_BASE } from "./MainPage";
      import{
        FaSignOutAlt,
        FaBatteryHalf,
        FaCompress,
        FaExpand,
}

// Logic 
  const [batteryModalOpen, setBatteryModalOpen] = useState(false);

    const [batteryLevel, setBatteryLevel] = useState(100);
    useEffect(() => {
      let batteryObj = null;
      let mounted = true;
      const updateLevel = () => {
        if (!batteryObj) return;
        const lvl = Math.round(batteryObj.level * 100);
        if (mounted) setBatteryLevel(lvl);
      };
      if (navigator.getBattery) {
        navigator.getBattery().then((b) => {
          batteryObj = b;
          updateLevel();
          batteryObj.addEventListener("levelchange", updateLevel);
        });
      } else {
        // fallback or simulated value
        setBatteryLevel(100);
      }
      return () => {
        mounted = false;
        if (batteryObj && batteryObj.removeEventListener) {
          batteryObj.removeEventListener("levelchange", updateLevel);
        }
      };
    }, []);

  // helper to map battery level to CSS class
  const batteryClass = (() => {
    if (batteryLevel < 20) return "battery-red";
    if (batteryLevel < 50) return "battery-yellow";
    if (batteryLevel > 70) return "battery-green";
    return "battery-orange";
  })();

  // explicit color value for the icon (applied directly to the SVG via color prop)
  const batteryColor = (() => {
    if (batteryLevel < 20) return "#ff4d4d"; // red
    if (batteryLevel < 50) return "#f59e0b"; // yellow
    if (batteryLevel > 70) return "#10b981"; // green
    return "#f97316"; // mid (orange)
  })();

// UI


        {/* header right: battery + fullscreen */}
        <div className="header-right">
          <button
            className={`battery ${batteryClass}`}
            type="button"
            onClick={() => setBatteryModalOpen(true)}
            title="Battery status"
          >
            <FaBatteryHalf className="battery-icon" color={batteryColor} />
            <div className="battery-percent">{batteryLevel}%</div>
          </button>
          <button
            onClick={toggleFullScreen}
            title={isFullScreen ? "Exit Fullscreen" : "Fullscreen"}
            className="fullscreen-btn"
            type="button"
          >
            {isFullScreen ? <FaCompress /> : <FaExpand />}
          </button>

        

          {/* Logout button — icon-only for compact header */}
          <button
            type="button"
            className="logout-btn"
            onClick={handleLogout}
            title="Log out"
            aria-label="Log out"
          >
            <FaSignOutAlt />
          </button>
        </div>

        // Battery Modal Open
         {batteryModalOpen && (
        <div
          className="modal-backdrop"
          onClick={() => setBatteryModalOpen(false)}
        >
          <div className="battery-modal" onClick={(e) => e.stopPropagation()}>
            <div className="battery-modal-header">
              <h3>Battery Status</h3>
              <button
                className="right-pane-close"
                type="button"
                onClick={() => setBatteryModalOpen(false)}
                aria-label="Close battery modal"
              >
                ✕
              </button>
            </div>
            <div className="battery-modal-body">
              <div className="battery-modal-grid">
                <div>
                  <span className="battery-label">Pack Voltage</span>
                  <strong>{batteryStatus.packVoltage} V</strong>
                </div>
                <div>
                  <span className="battery-label">Pack Current</span>
                  <strong>{batteryStatus.packCurrent} A</strong>
                </div>
                <div>
                  <span className="battery-label">State of Charge</span>
                  <strong>{batteryStatus.stateOfCharge}%</strong>
                </div>
                <div>
                  <span className="battery-label">Temperature</span>
                  <strong>{batteryStatus.temperature}</strong>
                </div>
                <div>
                  <span className="battery-label">Cycles</span>
                  <strong>{batteryStatus.cycles}</strong>
                </div>
                <div>
                  <span className="battery-label">Health</span>
                  <strong>{batteryStatus.health}</strong>
                </div>
              </div>
              <div className="battery-cell-table">
                <table>
                  <thead>
                    <tr>
                      <th>Cell</th>
                      <th>Voltage</th>
                    </tr>
                  </thead>
                  <tbody>
                    {(batteryStatus.cells || []).map((cell) => (
                      <tr key={cell.id}>
                        <td>{cell.id}</td>
                        <td>{cell.voltage} V</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </div>
          </div>
        </div>
      )}