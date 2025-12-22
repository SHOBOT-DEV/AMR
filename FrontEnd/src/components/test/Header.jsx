import React, { useState } from "react";
import {
  FaBatteryHalf,
  FaPlay,
  FaPause,
  FaExpand,
  FaCompress,
  FaLock,
  FaUnlock,
  FaSignOutAlt,
} from "react-icons/fa";
import { toast } from "react-hot-toast";
import "./Header.css";

const Header = ({
  bridgeStatus = { connected: false, error: "", endpoint: "" },
  startBridgeConnection = () => {},
  emergencyClicked = false,
  handleEmergencyToggle = () => {},
  isLocked = false,
  handleToggleLock = () => {},
  handleLogout = () => {},
  batteryLevel = 100,
  setBatteryModalOpen = () => {},
  isFullScreen = false,
  toggleFullScreen = () => {},
  layoutRef = null,
}) => {
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const [isPlaying, setIsPlaying] = useState(false);

  const batteryClass = (() => {
    if (batteryLevel < 20) return "battery-red";
    if (batteryLevel < 50) return "battery-yellow";
    if (batteryLevel > 70) return "battery-green";
    return "battery-orange";
  })();

  const batteryColor = (() => {
    if (batteryLevel < 20) return "#ff4d4d";
    if (batteryLevel < 50) return "#f59e0b";
    if (batteryLevel > 70) return "#10b981";
    return "#f97316";
  })();

  return (
    <header className="mp-header">
      {/* left header group */}
      <div style={{ display: "flex", alignItems: "center", gap: 12 }}>
        <div
          className={`bridge-chip ${bridgeStatus.connected ? "ok" : "warn"}`}
          title={
            bridgeStatus.error
              ? `Bridge: ${bridgeStatus.error}`
              : bridgeStatus.connected
              ? `Connected to AMR bridge (${bridgeStatus.endpoint})`
              : `Bridge offline (${bridgeStatus.endpoint})`
          }
        >
          <span className="bridge-dot" />
          <div className="bridge-chip-text">
            <div className="bridge-chip-label">AMR Bridge</div>
            <div className="bridge-chip-sub">
              {bridgeStatus.connected ? "Connected" : "Offline"}
              {bridgeStatus.endpoint ? ` · ${bridgeStatus.endpoint}` : ""}
            </div>
          </div>
          <button
            type="button"
            className="bridge-retry-btn"
            onClick={() => {
              toast.dismiss("bridge-retry");
              toast.success("Retrying bridge connection…", {
                id: "bridge-retry",
                duration: 1500,
              });
              startBridgeConnection();
            }}
          >
            Retry
          </button>
        </div>
      </div>

      {/* Header-centered Emergency Stop */}
      <div className="header-center-emergency">
        <button
          onClick={handleEmergencyToggle}
          aria-pressed={emergencyClicked}
          className={
            emergencyClicked ? "header-emergency-btn clicked" : "header-emergency-btn"
          }
        >
          <span className="header-emergency-dot" aria-hidden="true">
            <span className="header-emergency-dot-inner" />
          </span>
          <span className="header-emergency-label">Emergency Stop</span>
        </button>
      </div>

      {/* Dropdown with play/pause */}
      <div className="dropdown-wrap">
        <div
          className="dropdown-btn"
          role="button"
          tabIndex={0}
          onClick={() => setIsDropdownOpen((s) => !s)}
          aria-expanded={isDropdownOpen}
        >
          <button
            onClick={(e) => {
              e.stopPropagation();
              setIsPlaying((p) => !p);
            }}
            title={isPlaying ? "Pause" : "Play"}
            className={isPlaying ? "play-btn playing" : "play-btn"}
            aria-pressed={isPlaying}
          >
            {isPlaying ? <FaPause /> : <FaPlay />}
          </button>

          <span>Robot: UI EMERGENCY</span>
          <span className="caret">▾</span>
        </div>

        {isDropdownOpen && (
          <div role="dialog" className="dropdown-menu">
            <div className="dropdown-title">Robot: UI EMERGENCY</div>
            <div className="dropdown-body">
              There is no mission running. Initiate a mission to get started.
            </div>
          </div>
        )}
      </div>

      {/* header right: battery + fullscreen + lock + logout */}
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

        <button
          type="button"
          className="lock-toggle-btn"
          aria-pressed={isLocked}
          aria-label={isLocked ? "Unlock console" : "Lock console"}
          onMouseDown={(e) => e.preventDefault()}
          onClick={handleToggleLock}
          title={isLocked ? "Unlock console" : "Lock console"}
        >
          {isLocked ? <FaLock /> : <FaUnlock />}
        </button>

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
    </header>
  );
};

export default Header;