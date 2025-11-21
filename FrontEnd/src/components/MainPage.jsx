import React, { useRef, useEffect, useState } from "react";
import Sidebar from "./SideBar";
import JoyStick from "./JoyStick";
import { FaBatteryHalf, FaPlay, FaPause, FaExpand, FaCompress, FaTrash } from "react-icons/fa";
import "./MainPage.css";


const MainPage = () => {
  const [emergencyClicked, setEmergencyClicked] = useState(false);
  const mapRef = useRef(null);

  // new: zoom state for map
  const [isZoomed, setIsZoomed] = useState(false);
  const toggleMapZoom = () => setIsZoomed((s) => !s);

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

  // fullscreen state and listener
  const [isFullScreen, setIsFullScreen] = useState(false);
  useEffect(() => {
    const onFsChange = () => {
      setIsFullScreen(document.fullscreenElement === mapRef.current);
    };
    document.addEventListener("fullscreenchange", onFsChange);
    return () => document.removeEventListener("fullscreenchange", onFsChange);
  }, []);

  // toggleFullScreen function
  const toggleFullScreen = async () => {
    try {
      if (!document.fullscreenElement) {
        if (mapRef.current.requestFullscreen) {
          await mapRef.current.requestFullscreen();
        } else if (mapRef.current.webkitRequestFullscreen) {
          mapRef.current.webkitRequestFullscreen(); // Safari
        }
      } else {
        if (document.exitFullscreen) {
          await document.exitFullscreen();
        } else if (document.webkitExitFullscreen) {
          document.webkitExitFullscreen();
        }
      }
    } catch (e) {
      console.warn("Fullscreen error:", e);
    }
  };

  // dropdown & play/pause state for header button
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const [isPlaying, setIsPlaying] = useState(false);

  // new: right-side page (renders when a widen icon is clicked)
  const [rightPage, setRightPage] = useState(null);

  // sample maps data — replace image paths with your real map images
  const mapsList = [
    // use the new screenshot file for CFL_GF preview
    { id: "cfl_gf", name: "CFL_GF", createdBy: "CNDE", image: "file:///home/shobot/Pictures/Screenshot from 2025-11-17 07-25-47.png", status: "Active" },
    { id: "shobot_arena", name: "shobot_arena", createdBy: "ANSCER ADMIN", image: "/images/maps/shobot_arena.png", status: "" },
    { id: "shobot_arena2", name: "shobot_arena2", createdBy: "ANSCER ADMIN", image: "/images/maps/shobot_arena2.png", status: "" },
    /* zones entry: use the new screenshot you supplied */
    { id: "zones", name: "Zones", createdBy: "ANSCER ADMIN", image: "file:///home/shobot/Pictures/Screenshot from 2025-11-17 07-45-26.png", status: "" },
    /* waypoints entry: show the requested screenshot when Waypoints is selected */
    { id: "waypoints", name: "Waypoints", createdBy: "ANSCER ADMIN", image: "file:///home/shobot/Pictures/Screenshot from 2025-11-17 08-32-02.png", status: "" },
    /* users preview shown when selecting Users */
    { id: "users", name: "Users", createdBy: "ANSCER ADMIN", image: "file:///home/shobot/Pictures/Screenshot from 2025-11-17 08-54-24.png", status: "" },
  ];

  const [selectedMap, setSelectedMap] = useState(mapsList[0]);
  const [zoneField, setZoneField] = useState("");
  // waypoints data + selection
  const initialWaypoints = [
    { id: "wp1", name: "WP_A", category: "Nav", active: true, geom: "Point(12 34)", createdAt: "2025-11-17", notes: "First waypoint" },
    { id: "wp2", name: "WP_B", category: "Inspect", active: false, geom: "Point(98 76)", createdAt: "2025-11-17", notes: "Inspection point" },
    { id: "wp3", name: "WP_C", category: "Charge", active: true, geom: "Point(44 55)", createdAt: "2025-11-16", notes: "Charging pad" },
  ];
  const [waypoints, setWaypoints] = useState(initialWaypoints);
  const [selectedWaypointId, setSelectedWaypointId] = useState(null);

  // missions data + selection (added)
  const initialMissions = [
    { id: "m1", name: "Inspect Zone A", owner: "CNDE", status: "Draft", createdAt: "2025-11-17", notes: "Routine inspection" },
    { id: "m2", name: "Delivery Route 3", owner: "ANSCER ADMIN", status: "Scheduled", createdAt: "2025-11-16", notes: "Delivery to docks" },
    { id: "m3", name: "Battery Check", owner: "CNDE", status: "Completed", createdAt: "2025-11-15", notes: "Post-run check" },
  ];
  const [missions, setMissions] = useState(initialMissions);
  const [selectedMissionId, setSelectedMissionId] = useState(null);
  const handleSelectMission = (id) => setSelectedMissionId(id);

  // users data + selection (fixes eslint no-undef)
  const initialUsers = [
    { id: "u1", name: "ANSCER ADMIN", email: "admin@anscer.com", role: "Admin", status: "Active", createdBy: "—", createdAt: "—" },
    { id: "u2", name: "CNDE", email: "cnde@iitm.org", role: "Admin", status: "Active", createdBy: "ANSCER ADMIN", createdAt: "02/18/2025, 5:37:51 PM" },
  ];
  const [users, setUsers] = useState(initialUsers);
  const [selectedUserId, setSelectedUserId] = useState(null);

  const handleSelectWaypoint = (wpId) => {
    setSelectedWaypointId(wpId);
    const wpMap = mapsList.find((m) => m.id === "waypoints");
    if (wpMap) setSelectedMap(wpMap);
  };

  const handleSidebarSelect = (id) => {
    // open right pane for create sub-items and monitor sub-items
    const createIds = new Set(["maps", "zones", "waypoints", "missions", "users"]);
    const monitorIds = new Set(["analytics", "diagnostics", "logs", "mission-logs", "robot-bags"]);
    const settingsIds = new Set(["robot-settings", "account", "appearance", "security", "integrations"]);
    if (createIds.has(id) || monitorIds.has(id) || settingsIds.has(id)) {
      setRightPage(id);
      // if it's the maps "page", preselect first map
      if (id === "maps") {
        setSelectedMap(mapsList[0]);
      } else if (id === "zones") {
        // select the zones entry so left-side panel shows the requested path
        const z = mapsList.find((m) => m.id === "zones");
        if (z) setSelectedMap(z);
      } else if (id === "waypoints") {
        // select the waypoints entry so main area shows the provided screenshot path
        const w = mapsList.find((m) => m.id === "waypoints");
        if (w) setSelectedMap(w);
      } else if (id === "users") {
        // select the users preview so left info shows the screenshot/path
        const u = mapsList.find((m) => m.id === "users");
        if (u) setSelectedMap(u);
      }
    } else {
      // ignore main sidebar icons (do not render a page)
      setRightPage(null);
    }
  };

  // map action handler (Preview / Edit / Delete)
  const handleMapAction = (action, map) => {
    // placeholder behavior — replace with real navigation / API calls
    console.log("Map action:", action, "on", map && map.id);
    if (action === "delete") {
      // example: remove map from mapsList (in real app you'd call API)
      // For now just clear selection if deleting currently selected map
      if (selectedMap && map && selectedMap.id === map.id) setSelectedMap(null);
    }
    if (action === "preview") {
      // example: open a preview modal — placeholder log
    }
    if (action === "edit") {
      // example: open edit UI — placeholder log
    }
  };

  // active action radio state for the map action buttons
  const [activeMapAction, setActiveMapAction] = useState(null);

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
  
  const emergencyClass = emergencyClicked ? "emergency-btn clicked" : "emergency-btn";

  // breadcrumb parts for header (Home → Page → Item)
  const breadcrumbParts = ["Home"];
  if (rightPage) breadcrumbParts.push(rightPage.charAt(0).toUpperCase() + rightPage.slice(1));
  if (rightPage === "maps" && selectedMap) breadcrumbParts.push(selectedMap.name);

  return (
    <div className={`main-container ${rightPage ? "has-right-pane" : ""}`}>
      <header className="mp-header">
        <div style={{ display: "flex", alignItems: "center", gap: 12 }}>
          <div className="brand">⨂</div>
        </div>

        {/* Header-centered Emergency Stop */}
        <div className="header-center-emergency">
          <button
            onClick={() => setEmergencyClicked(true)}
            aria-pressed={emergencyClicked}
            className={emergencyClicked ? "header-emergency-btn clicked" : "header-emergency-btn"}
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

        {/* header right: battery + fullscreen */}
        <div className="header-right">
          <div className={`battery ${batteryClass}`}>
            <FaBatteryHalf className="battery-icon" color={batteryColor} />
            <div className="battery-percent">{batteryLevel}%</div>
            <button
              onClick={toggleFullScreen}
              title={isFullScreen ? "Exit Fullscreen" : "Fullscreen"}
              className="fullscreen-btn"
            >
              {isFullScreen ? <FaCompress /> : <FaExpand />}
            </button>
          </div>
        </div>
      </header>
 
      <div className="content-wrap">
        <Sidebar
          onSelect={handleSidebarSelect}
          onBack={() => {
            // close the right-side page if open; otherwise no-op for now
            if (rightPage) setRightPage(null);
          }}
        />

        {/* Right pane: shows the selected page in the right half of the screen */}
        {rightPage && (
          <aside className="right-pane" role="region" aria-label="Right pane">
            <div className="right-pane-header">
              <strong>{rightPage.charAt(0).toUpperCase() + rightPage.slice(1)}</strong>
              <button className="right-pane-close" onClick={() => setRightPage(null)} aria-label="Close">✕</button>
            </div>
            <div className="right-pane-body">
              {/* Maps list page: clickable rows */}
              {rightPage === "maps" && (
                <div style={{ display: "flex", flexDirection: "column", gap: 8 }}>
                  {/* Search / filter controls */}
                  <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                    <select>
                      <option>Search Map By Name</option>
                    </select>
                    <input style={{ flex: 1, padding: "6px 8px" }} placeholder="Search map..." />
                    <button title="Toggle visibility">👁</button>
                  </div>

                  <div style={{ borderTop: "1px solid #e6eef2", marginTop: 8 }} />

                  {/* Table view for maps (Name | Created By | Created At | Status) */}
                  <div style={{ overflowY: "auto", maxHeight: 420 }}>
                    <table style={{ width: "100%", borderCollapse: "collapse", marginTop: 8 }}>
                      <thead style={{ background: "#f8fafc" }}>
                        <tr>
                          <th style={{ textAlign: "left", padding: "12px 8px", fontSize: 13, color: "#374151" }}>Name</th>
                          <th style={{ textAlign: "left", padding: "12px 8px", fontSize: 13, color: "#374151" }}>Created By</th>
                          <th style={{ textAlign: "left", padding: "12px 8px", fontSize: 13, color: "#374151" }}>Created At</th>
                          <th style={{ textAlign: "right", padding: "12px 8px", fontSize: 13, color: "#374151" }}>Status</th>
                        </tr>
                      </thead>
                      <tbody>
                        {mapsList.map((m) => (
                          <tr
                            key={m.id}
                            onClick={() => setSelectedMap(m)}
                            style={{
                              cursor: "pointer",
                              background: selectedMap && selectedMap.id === m.id ? "rgba(3,48,80,0.04)" : "transparent",
                            }}
                          >
                            <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", fontWeight: 700 }}>{m.name}</td>
                            <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", color: "#6b7280" }}>{m.createdBy}</td>
                            <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", color: "#6b7280" }}>2025-11-12</td>
                            <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", textAlign: "right" }}>
                              {m.status ? <span style={{ background: "#10b981", color: "#fff", padding: "2px 8px", borderRadius: 8 }}>{m.status}</span> : <span style={{ color: "#9ca3af" }}>—</span>}
                            </td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </div>
                </div>
               )}

              {/* Zones page: search, create button, and table matching screenshot layout */}
              {rightPage === "zones" && (
                <div style={{ display: "flex", flexDirection: "column", gap: 12 }}>
                  <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", gap: 12 }}>
                    <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                      <label style={{ fontSize: 13, color: "#475569", fontWeight: 600 }}>Search Zone By</label>
                      <select style={{ padding: "6px 8px" }}>
                        <option>Name</option>
                        <option>Category</option>
                      </select>
                      <input
                        placeholder="Search zone..."
                        style={{ padding: "8px 10px", borderRadius: 8, border: "1px solid #e6eef2", minWidth: 220 }}
                      />
                    </div>

                    <div>
                      <button
                        onClick={() => { /* open create zone flow */ }}
                        style={{
                          background: "#0b74d1",
                          color: "#fff",
                          padding: "10px 14px",
                          borderRadius: 8,
                          border: "none",
                          cursor: "pointer",
                          boxShadow: "0 6px 18px rgba(11,116,209,0.16)"
                        }}
                      >
                        + Create New Zone
                      </button>
                    </div>
                  </div>

                  <div style={{ borderTop: "1px solid #e6eef2", marginTop: 4 }} />

                  {/* single-column layout: table occupies full available width (empty body) */}
                  <div>
                    <div style={{ background: "#fff", borderRadius: 8, boxShadow: "0 1px 3px rgba(2,6,23,0.06)", overflow: "hidden" }}>
                      <div style={{ padding: "12px 16px", borderBottom: "1px solid #eef2f6", display: "flex", alignItems: "center", gap: 12 }}>
                        <div style={{ flex: 1, fontWeight: 700, color: "#0f172a" }}>Zones</div>
                        <div style={{ color: "#94a3b8", fontSize: 13 }}>Rows per page: 5</div>
                      </div>

                      <div style={{ padding: "8px 16px" }}>
                        <table style={{ width: "100%", borderCollapse: "collapse" }}>
                          <thead style={{ background: "#fafafa", color: "#475569", fontSize: 13 }}>
                            <tr>
                              <th style={{ textAlign: "left", padding: "12px 8px" }}>Name</th>
                              <th style={{ textAlign: "left", padding: "12px 8px" }}>Category</th>
                              <th style={{ textAlign: "center", padding: "12px 8px" }}>Active</th>
                              <th style={{ textAlign: "left", padding: "12px 8px" }}>Geometry</th>
                              <th style={{ textAlign: "right", padding: "12px 8px" }}>Created At</th>
                            </tr>
                          </thead>
                          <tbody>
                            {/* no placeholder rows — table body intentionally left empty */}
                          </tbody>
                        </table>
                      </div>
                    </div>
                  </div>
                </div>
              )}

              {/* Waypoints page: show map/file path on left and waypoints table on right */}
              {rightPage === "waypoints" && (
                <div style={{ display: "flex", flexDirection: "column", gap: 12 }}>
                  <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                    <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                      <label style={{ fontSize: 13, color: "#475569", fontWeight: 600 }}>Search Waypoint By</label>
                      <select style={{ padding: "6px 8px" }}>
                        <option>Name</option>
                        <option>Category</option>
                      </select>
                      <input placeholder="Search waypoint..." style={{ padding: "8px 10px", borderRadius: 8, border: "1px solid #e6eef2", minWidth: 220 }} />
                    </div>
                    <div>
                      <button style={{ background: "#0b74d1", color: "#fff", padding: "10px 14px", borderRadius: 8, border: "none", cursor: "pointer", boxShadow: "0 6px 18px rgba(11,116,209,0.16)" }}>
                        + Create New Waypoint
                      </button>
                    </div>
                  </div>

                  <div style={{ background: "#fff", borderRadius: 8, boxShadow: "0 1px 3px rgba(2,6,23,0.06)", overflow: "hidden" }}>
                    <div style={{ padding: "12px 16px", borderBottom: "1px solid #eef2f6", display: "flex", alignItems: "center", gap: 12 }}>
                      <div style={{ flex: 1, fontWeight: 700, color: "#0f172a" }}>Waypoints</div>
                      <div style={{ color: "#94a3b8", fontSize: 13 }}>Rows per page: 10</div>
                    </div>
                    <div style={{ padding: "8px 16px" }}>
                      <table style={{ width: "100%", borderCollapse: "collapse" }}>
                        <thead style={{ background: "#fafafa", color: "#475569", fontSize: 13 }}>
                          <tr>
                            <th style={{ textAlign: "left", padding: "12px 8px" }}>Name</th>
                            <th style={{ textAlign: "left", padding: "12px 8px" }}>Category</th>
                            <th style={{ textAlign: "center", padding: "12px 8px" }}>Active</th>
                            <th style={{ textAlign: "left", padding: "12px 8px" }}>Geometry</th>
                            <th style={{ textAlign: "right", padding: "12px 8px" }}>Created At</th>
                          </tr>
                        </thead>
                        <tbody>
                          {waypoints.length === 0 && (
                            <tr>
                              <td colSpan={5} style={{ padding: "36px 8px", textAlign: "center", color: "#94a3b8" }}>
                                <div style={{ fontWeight: 700, color: "#0f172a", marginBottom: 6 }}>No Waypoints Found</div>
                                <div>Try creating new waypoints.</div>
                              </td>
                            </tr>
                          )}
                          {waypoints.map((wp) => (
                            <tr
                              key={wp.id}
                              onClick={() => handleSelectWaypoint(wp.id)}
                              style={{
                                cursor: "pointer",
                                background: selectedWaypointId === wp.id ? "rgba(3,48,80,0.04)" : "transparent",
                              }}
                            >
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", fontWeight: 700 }}>{wp.name}</td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", color: "#6b7280" }}>{wp.category}</td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", textAlign: "center" }}>{wp.active ? "Yes" : "No"}</td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", color: "#6b7280" }}>{wp.geom}</td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", textAlign: "right" }}>{wp.createdAt}</td>
                            </tr>
                          ))}
                        </tbody>
                      </table>
                    </div>
                  </div>
                </div>
              )}

              {/* Users page: search, create button and table similar to your screenshot */}
              {rightPage === "users" && (
                <div style={{ display: "flex", flexDirection: "column", gap: 12 }}>
                  <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                    <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                      <label style={{ fontSize: 13, color: "#475569", fontWeight: 600 }}>Search User By</label>
                      <select style={{ padding: "6px 8px" }}>
                        <option>Name</option>
                        <option>Email</option>
                      </select>
                      <input placeholder="Search user..." style={{ padding: "8px 10px", borderRadius: 8, border: "1px solid #e6eef2", minWidth: 300 }} />
                    </div>
                    <button style={{ background: "#0b74d1", color: "#fff", padding: "10px 14px", borderRadius: 8, border: "none", cursor: "pointer" }}>
                      + Create New User
                    </button>
                  </div>

                  <div style={{ background: "#fff", borderRadius: 8, overflow: "hidden", boxShadow: "0 1px 3px rgba(2,6,23,0.06)" }}>
                    <div style={{ padding: "12px 16px", borderBottom: "1px solid #eef2f6", display: "flex", gap: 12 }}>
                      <div style={{ fontWeight: 800, fontSize: 18 }}>Users</div>
                    </div>

                    <div style={{ padding: "8px 16px" }}>
                      <table style={{ width: "100%", borderCollapse: "collapse" }}>
                        <thead style={{ background: "#fafafa", color: "#475569", fontSize: 13 }}>
                          <tr>
                            <th style={{ textAlign: "left", padding: "12px 8px" }}>Name</th>
                            <th style={{ textAlign: "left", padding: "12px 8px" }}>Email</th>
                            <th style={{ textAlign: "left", padding: "12px 8px" }}>Role</th>
                            <th style={{ textAlign: "center", padding: "12px 8px" }}>Status</th>
                            <th style={{ textAlign: "left", padding: "12px 8px" }}>Created By</th>
                            <th style={{ textAlign: "right", padding: "12px 8px" }}>Created At</th>
                            <th style={{ textAlign: "center", padding: "12px 8px", width: 48 }}></th>
                          </tr>
                        </thead>
                        <tbody>
                          {users.map((u) => (
                            <tr
                              key={u.id}
                              onClick={() => setSelectedUserId(u.id)}
                              style={{
                                cursor: "pointer",
                                background: selectedUserId === u.id ? "rgba(3,48,80,0.04)" : "transparent",
                              }}
                            >
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", fontWeight: 700 }}>{u.name}</td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", color: "#6b7280" }}>{u.email}</td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6" }}>{u.role}</td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", textAlign: "center" }}>
                                <span style={{ background: "#bbf7d0", color: "#065f46", padding: "4px 8px", borderRadius: 8, fontSize: 12 }}>{u.status}</span>
                              </td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", color: "#6b7280" }}>{u.createdBy}</td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", textAlign: "right", color: "#6b7280" }}>{u.createdAt}</td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", textAlign: "center" }}>
                                <button title="Delete user" onClick={(e) => { e.stopPropagation(); /* placeholder */ }} style={{ background: "transparent", border: "none", cursor: "pointer", color: "#9ca3af" }}>
                                  <FaTrash />
                                </button>
                              </td>
                            </tr>
                          ))}
                        </tbody>
                      </table>
                    </div>

                    <div style={{ display: "flex", alignItems: "center", gap: 12, justifyContent: "flex-end", padding: "12px 16px", borderTop: "1px solid #eef2f6" }}>
                      <div style={{ color: "#94a3b8", fontSize: 13 }}>Rows per page: 5</div>
                      <div style={{ color: "#94a3b8", fontSize: 13 }}>1–{users.length} of {users.length}</div>
                      <div style={{ marginLeft: 8, display: "flex", gap: 8 }}>
                        <button style={{ padding: "8px 12px", borderRadius: 8, border: "1px solid #e6eef2", background: "#fff", color: "#6b7280" }} disabled>Edit</button>
                        <button style={{ padding: "8px 12px", borderRadius: 8, border: "1px solid #e6eef2", background: "#fff", color: "#6b7280" }} disabled>Reset Password</button>
                      </div>
                    </div>
                  </div>
                </div>
              )}

              {/* Missions page: search, create button and table */}
              {rightPage === "missions" && (
                <div style={{ display: "flex", flexDirection: "column", gap: 12 }}>
                  <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                    <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                      <label style={{ fontSize: 13, color: "#475569", fontWeight: 600 }}>Search Mission By</label>
                      <select style={{ padding: "6px 8px" }}>
                        <option>Name</option>
                        <option>Owner</option>
                        <option>Status</option>
                      </select>
                      <input placeholder="Search mission..." style={{ padding: "8px 10px", borderRadius: 8, border: "1px solid #e6eef2", minWidth: 220 }} />
                    </div>

                    <button
                      style={{
                        background: "#0b74d1",
                        color: "#fff",
                        padding: "10px 14px",
                        borderRadius: 8,
                        border: "none",
                        cursor: "pointer",
                        boxShadow: "0 6px 18px rgba(11,116,209,0.16)"
                      }}
                      onClick={() => {
                        // placeholder create action: add a draft mission
                        const id = `m${missions.length + 1}`;
                        const newM = { id, name: `New Mission ${missions.length + 1}`, owner: "You", status: "Draft", createdAt: new Date().toISOString().split("T")[0], notes: "" };
                        setMissions((s) => [newM, ...s]);
                        setSelectedMissionId(id);
                      }}
                    >
                      + Create Mission
                    </button>
                  </div>

                  <div style={{ borderTop: "1px solid #e6eef2", marginTop: 4 }} />

                  <div style={{ background: "#fff", borderRadius: 8, boxShadow: "0 1px 3px rgba(2,6,23,0.06)", overflow: "hidden" }}>
                    <div style={{ padding: "12px 16px", borderBottom: "1px solid #eef2f6", display: "flex", alignItems: "center", gap: 12 }}>
                      <div style={{ flex: 1, fontWeight: 700, color: "#0f172a" }}>Missions</div>
                      <div style={{ color: "#94a3b8", fontSize: 13 }}>Rows per page: 10</div>
                    </div>

                    <div style={{ padding: "8px 16px" }}>
                      <table style={{ width: "100%", borderCollapse: "collapse" }}>
                        <thead style={{ background: "#fafafa", color: "#475569", fontSize: 13 }}>
                          <tr>
                            <th style={{ textAlign: "left", padding: "12px 8px" }}>Name</th>
                            <th style={{ textAlign: "left", padding: "12px 8px" }}>Owner</th>
                            <th style={{ textAlign: "center", padding: "12px 8px" }}>Status</th>
                            <th style={{ textAlign: "right", padding: "12px 8px" }}>Created At</th>
                          </tr>
                        </thead>
                        <tbody>
                          {missions.map((m) => (
                            <tr
                              key={m.id}
                              onClick={() => handleSelectMission(m.id)}
                              style={{
                                cursor: "pointer",
                                background: selectedMissionId === m.id ? "rgba(3,48,80,0.04)" : "transparent",
                              }}
                            >
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", fontWeight: 700 }}>{m.name}</td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", color: "#6b7280" }}>{m.owner}</td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", textAlign: "center" }}>
                                <span style={{ background: m.status === "Completed" ? "#bbf7d0" : "#fee2e2", color: m.status === "Completed" ? "#065f46" : "#9b1b1b", padding: "4px 8px", borderRadius: 8, fontSize: 12 }}>{m.status}</span>
                              </td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid #eef2f6", textAlign: "right", color: "#6b7280" }}>{m.createdAt}</td>
                            </tr>
                          ))}
                        </tbody>
                      </table>
                    </div>
                  </div>

                  {/* mission details */}
                  <div style={{ background: "#fff", borderRadius: 8, padding: 12, boxShadow: "0 1px 3px rgba(2,6,23,0.06)" }}>
                    {selectedMissionId ? (
                      (() => {
                        const m = missions.find((x) => x.id === selectedMissionId);
                        if (!m) return <div style={{ color: "#94a3b8" }}>Mission not found.</div>;
                        return (
                          <div>
                            <div style={{ fontWeight: 800, fontSize: 16 }}>{m.name}</div>
                            <div style={{ marginTop: 8, color: "#6b7280" }}><strong>Owner:</strong> {m.owner}</div>
                            <div style={{ marginTop: 6, color: "#6b7280" }}><strong>Status:</strong> {m.status}</div>
                            <div style={{ marginTop: 6, color: "#6b7280" }}><strong>Created At:</strong> {m.createdAt}</div>
                            <div style={{ marginTop: 10, color: "#475569" }}>{m.notes || "No notes."}</div>
                          </div>
                        );
                      })()
                    ) : (
                      <div style={{ color: "#94a3b8" }}>Select a mission to see details.</div>
                    )}
                  </div>
                </div>
              )}

              {/* fallback pages unchanged */}
              {rightPage === "analytics" && <div>Analytics dashboard goes here.</div>}
              {rightPage === "diagnostics" && <div>Diagnostics tools go here.</div>}
              {rightPage === "logs" && <div>Logs viewer goes here.</div>}
              {rightPage === "mission-logs" && <div>Mission logs list goes here.</div>}
              {rightPage === "robot-bags" && <div>Robot bag management goes here.</div>}
              {/* settings pages */}
              {rightPage === "robot-settings" && <div>Robot settings UI goes here.</div>}
              {rightPage === "account" && <div>Account settings UI goes here.</div>}
              {rightPage === "appearance" && <div>Appearance & theme settings go here.</div>}
              {rightPage === "security" && <div>Security settings go here.</div>}
              {rightPage === "integrations" && <div>Integrations settings go here.</div>}
              {/* fallback */}
              {!["maps","zones","waypoints","missions","users","analytics","diagnostics","logs","mission-logs","robot-bags","robot-settings","account","appearance","security","integrations"].includes(rightPage) && <div>{rightPage}</div>}
            </div>
          </aside>
        )}

        <main className="map-area">
          {/* Map / Workspace placeholder - this is the element we fullscreen */}
          <div
            ref={mapRef}
            className="map-ref"
            /* ref kept on outer container so fullscreen targets the whole map area */>

            {/* inner content that will zoom — click handler here so only map-content scales */}
            <div
              className={`map-content ${isZoomed ? "map-zoomed" : ""}`}
              onClick={toggleMapZoom}
              role="button"
              tabIndex={0}
              onKeyDown={(e) => { if (e.key === "Enter" || e.key === " ") toggleMapZoom(); }}
              style={{ width: "100%", height: "100%", display: "flex", alignItems: "stretch", justifyContent: "stretch" }}
            >
              {selectedMap ? (
                <div style={{ position: "relative", width: "100%", height: "100%" }}>
                  <img
                    src={selectedMap.image}
                    alt={selectedMap.name}
                    style={{ width: "100%", height: "100%", objectFit: "contain", display: "block" }}
                  />

                  {/* bottom info bar like screenshot */}
                  <div style={{ position: "absolute", left: 0, right: 0, bottom: 0, background: "rgba(240,248,255,0.95)", padding: "12px 16px", display: "flex", alignItems: "center", gap: 12 }}>
                    <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                      <span style={{ fontSize: 18 }}>📍</span>
                      <div>
                        <div style={{ fontWeight: 800 }}>
                          <span style={{ fontWeight: 700 }}>
                            {breadcrumbParts.length > 1 ? breadcrumbParts.slice(1).join(" › ") : selectedMap.name}
                          </span>
                        </div>
                        <div style={{ fontSize: 12, color: "#6b7280" }}>Created By: {selectedMap.createdBy}</div>
                      </div>
                    </div>

                    {/* action buttons group (Preview / Edit / Delete) */}
                    <div style={{ display: "flex", gap: 8, marginLeft: 24 }}>
                      <button
                        onClick={() => { setActiveMapAction("preview"); handleMapAction("preview", selectedMap); }}
                        title="Preview map"
                        aria-pressed={activeMapAction === "preview"}
                        style={{
                          padding: "6px 10px",
                          borderRadius: 6,
                          border: "none",
                          background: activeMapAction === "preview" ? "#bfdbfe" : "#eef2ff",
                          cursor: "pointer",
                          boxShadow: activeMapAction === "preview" ? "inset 0 0 0 2px rgba(59,130,246,0.12)" : "none"
                        }}
                      >
                        Preview
                      </button>
                      <button
                        onClick={() => { setActiveMapAction("edit"); handleMapAction("edit", selectedMap); }}
                        title="Edit map"
                        aria-pressed={activeMapAction === "edit"}
                        style={{
                          padding: "6px 10px",
                          borderRadius: 6,
                          border: "none",
                          background: activeMapAction === "edit" ? "#bfdbfe" : "#e6f7ff",
                          cursor: "pointer",
                          boxShadow: activeMapAction === "edit" ? "inset 0 0 0 2px rgba(59,130,246,0.12)" : "none"
                        }}
                      >
                        Edit
                      </button>
                      <button
                        onClick={() => { setActiveMapAction("delete"); handleMapAction("delete", selectedMap); }}
                        title="Delete map"
                        aria-pressed={activeMapAction === "delete"}
                        style={{
                          padding: "6px 10px",
                          borderRadius: 6,
                          border: "none",
                          background: activeMapAction === "delete" ? "#fecaca" : "#ffdce0",
                          cursor: "pointer",
                          color: "#9b1b1b",
                          boxShadow: activeMapAction === "delete" ? "inset 0 0 0 2px rgba(220,38,38,0.08)" : "none"
                        }}
                      >
                        Delete
                      </button>
                     </div>

                    <div style={{ marginLeft: "auto", color: "#6b7280" }}>{selectedMap.status ? <span style={{ background: "#10b981", color: "#fff", padding: "2px 8px", borderRadius: 8 }}>{selectedMap.status}</span> : null}</div>
                  </div>
                </div>
              ) : (
                <div style={{ width: "100%", height: "100%", display: "flex", alignItems: "center", justifyContent: "center", color: "#9ca3af" }}>
                  No map selected
                </div>
              )}
            </div>
           </div>
 
          {/* Right floating controls (stacked icons) */}
          <div className="right-controls">
            <div style={{ width: 40, height: 40 }} className="control-btn" title="Layers">≡</div>
            <div style={{ width: 40, height: 40 }} className="control-btn" title="Center">⊕</div>
          </div>
 
          {/* Joystick (bottom-right) — JoyStick is positioned fixed by the component */}
          <JoyStick width={140} height={140} className="joystick-fixed" />
        </main>
      </div>
    </div>
  );
};

export default MainPage;
