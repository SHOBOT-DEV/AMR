import React, { useState } from "react";
import "./RightPane.css";
import MapsPanel from "../../features/create/maps";
import { FaEdit, FaMicrophone, FaPaperPlane } from "react-icons/fa";

/*
Props expected (pass these from MainPage.jsx):
{
  rightPage, setRightPage,
  // maps
  mapsList, selectedMap, setSelectedMap, mapSearchField, setMapSearchField,
  mapSearchTerm, setMapSearchTerm, createNewMapImmediate, handleActivateMap,
  // zones
  zones, zoneFormOpen, setZoneFormOpen, zoneForm, setZoneForm, setZones,
  zoneSearchField, setZoneSearchField, zoneSearchTerm, setZoneSearchTerm,
  // waypoints
  waypoints, waypointFormOpen, setWaypointFormOpen, waypointForm, setWaypointForm,
  handleSelectWaypoint, setSelectedWaypointId,
  // users
  users, usersLoading, usersError, loadUsers, selectedUserId, setSelectedUserId,
  userActionLoading, handleResetUserPassword,
  // missions
  missions, missionFormOpen, setMissionFormOpen, missionForm, setMissionForm,
  selectedMissionId, setSelectedMissionId, handleSelectMission, setMissions,
  // analytics / diagnostics / logs / mission-history / bags
  analyticsSummary, analyticsSeries, analyticsAlerts, diagnosticsPanels,
  logEvents, missionHistory, bagFiles,
  // robot settings, account, appearance, security, integrations
  robotSettingsState, toggleRobotSetting,
  accountProfile, handleAccountChange, profileError, profileSuccess, profileSaving, handleSaveProfile,
  selectedTheme, setSelectedTheme,
  securityPreferences, toggleSecurityPref, securityEvents,
  integrationItems, toggleIntegrationStatus,
  // stats
  overview, missionTrend, monthlyMovement, batterySeries, batteryStatus, turns,
  statsLoading, statsError, lineChartSize, buildLinePath, buildSimplePath, analyticsChartSize, analyticsPath,
  // chat
  chatMessages, chatInput, setChatInput, handleSendMessage, handleKeyPress,
  isRecording, handleMicClick, isTyping, handleSuggestionClick, chatQuickPrompts, chatContainerRef, formatTimestamp, latestMessage
}
*/

const RightPane = (props) => {
  const [waypointSearchField, setWaypointSearchField] = useState("any");
  const [waypointSearchTerm, setWaypointSearchTerm] = useState("");
  const [missionSearchField, setMissionSearchField] = useState("any");
  const [missionSearchTerm, setMissionSearchTerm] = useState("");
  const [userSearchField, setUserSearchField] = useState("any");
  const [userSearchTerm, setUserSearchTerm] = useState("");
  const [editingUserId, setEditingUserId] = useState(null);
  const [editUserForm, setEditUserForm] = useState({
    username: "",
    email: "",
    company: "",
    amr_type: "",
    role: "",
    approval: "",
  });

  const [isRecording, setIsRecording] = useState(false);

  // ---------- VOICE RECOGNITION SETUP ----------
  const SpeechRecognition =
    typeof window !== "undefined" &&
    (window.SpeechRecognition || window.webkitSpeechRecognition);
  const recognition = SpeechRecognition ? new SpeechRecognition() : null;
  if (recognition) {
    recognition.continuous = false;
    recognition.interimResults = false;
    recognition.lang = "en-US";
  }

  const handleMicClick = () => {
    if (!recognition) {
      alert("Your browser does not support voice input.");
      return;
    }
    if (isRecording) {
      recognition.stop();
      setIsRecording(false);
      return;
    }
    setIsRecording(true);
    recognition.start();
    recognition.onresult = (event) => {
      const text = event.results[0][0].transcript;
      setChatInput((prev) => prev + (prev ? " " : "") + text);
      setIsRecording(false);
    };
    recognition.onerror = () => setIsRecording(false);
    recognition.onend = () => setIsRecording(false);
  };

  // destructure props (removed isRecording/handleMicClick from props)
  const {
    
    rightPage,
    setRightPage,
    // maps
    mapsList,
    selectedMap,
    createNewMapImmediate,
    handleActivateMap,
    mapSearchField,
    setMapSearchField,
    mapSearchTerm,
    setMapSearchTerm,
    requestV1,
    toast,
    setMapsList,
    setSelectedMap,
    // zones
    zones,
    zoneFormOpen,
    setZoneFormOpen,
    zoneForm,
    setZoneForm,
    setZones,
    zoneSearchField,
    setZoneSearchField,
    zoneSearchTerm,
    setZoneSearchTerm,
    // waypoints
    waypoints,
    waypointFormOpen,
    setWaypointFormOpen,
    waypointForm,
    setWaypointForm,
    setWaypoints,
    handleSelectWaypoint,
    setSelectedWaypointId,
    // users
    users,
    usersLoading,
    usersError,
    loadUsers,
    selectedUserId,
    setSelectedUserId,
    userActionLoading,
    handleResetUserPassword,
    // missions
    missions,
    missionFormOpen,
    setMissionFormOpen,
    missionForm,
    setMissionForm,
    selectedMissionId,
    setSelectedMissionId,
    handleSelectMission,
    setMissions,
    // analytics/diagnostics/logs
    analyticsSummary,
    analyticsSeries,
    analyticsAlerts,
    diagnosticsPanels,
    logEvents,
    missionHistory,
    bagFiles,
    // settings
    robotSettingsState,
    toggleRobotSetting,
    accountProfile,
    handleAccountChange,
    profileError,
    profileSuccess,
    profileSaving,
    handleSaveProfile,
    selectedTheme,
    setSelectedTheme,
    securityPreferences,
    toggleSecurityPref,
    securityEvents,
    integrationItems,
    toggleIntegrationStatus,
    // stats
    overview,
    missionTrend,
    monthlyMovement,
    batterySeries,
    batteryStatus,
    turns,
    statsLoading,
    statsError,
    lineChartSize,
    buildLinePath,
    buildSimplePath,
    analyticsChartSize,
    analyticsPath,
    // chat
    chatMessages,
    chatInput,
    setChatInput,
    handleSendMessage,
    handleKeyPress,
    isTyping,
    handleSuggestionClick,
    chatQuickPrompts,
    chatContainerRef,
    formatTimestamp,
    latestMessage,
  } = props;

  // Add this helper function reference (pass from MainPage)
  // Helper function for filtering by search term and field
  const filterBySearch = (items, searchTerm, searchField, fieldMap) => {
    if (!searchTerm.trim()) return items;
    
    const term = searchTerm.trim().toLowerCase();
    
    return items.filter((item) => {
      if (searchField === "any") {
        // Search across all fields
        return Object.values(fieldMap).some(
          (field) => String(item[field] || "").toLowerCase().includes(term)
        );
      }
      // Search in specific field
      const fieldName = fieldMap[searchField];
      return String(item[fieldName] || "").toLowerCase().includes(term);
    });
  };

  // Render function: reproduces the same right-pane layout previously inline
  return (
    <aside className="right-pane" role="region" aria-label="Right pane">
      <div className="right-pane-header">
        <strong style={{ fontSize: "1.8rem" }}>
          {rightPage ? rightPage.charAt(0).toUpperCase() + rightPage.slice(1) : ""}
        </strong>

        <button
          className="right-pane-close"
          onClick={() => setRightPage(null)}
          aria-label="Close"
        >
          ✕
        </button>
      </div>

      <div className="right-pane-body">
        {/* MAPS */}
        {rightPage === "maps" && (
          <MapsPanel
            mapsList={mapsList}
            selectedMap={selectedMap}
            mapSearchField={mapSearchField}
            setMapSearchField={setMapSearchField}
            mapSearchTerm={mapSearchTerm}
            setMapSearchTerm={setMapSearchTerm}
            createNewMapImmediate={createNewMapImmediate}
            handleActivateMap={handleActivateMap}
            requestV1={requestV1}
            toast={toast}
            setMapsList={setMapsList}
            setSelectedMap={setSelectedMap}
          />
        )}

        {/* ZONES */}
        {rightPage === "zones" && (
          <div style={{ display: "flex", flexDirection: "column", gap: 12, padding: "0 16px" }}>
            {!selectedMap && (
              <div style={{ padding: "32px 16px", textAlign: "center", color: "var(--muted-text)" }}>
                <div style={{ fontWeight: 700, fontSize: 16, marginBottom: 8 }}>No Map Selected</div>
                <div>Please select a map from the Maps section to view and create zones.</div>
              </div>
            )}
            
            {selectedMap && (
              <>
                <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", gap: 12 }}>
                  <div style={{ display: "flex", alignItems: "center", gap: 8, flex: 1 }}>
                    <select 
                      style={{ padding: "14px 8px", borderRadius: 8, border: "1px solid var(--card-border)" }}
                      value={zoneSearchField}
                      onChange={(e) => setZoneSearchField(e.target.value)}
                      aria-label="Search by field"
                    >
                      <option value="any">All Fields</option>
                      <option value="name">Name</option>
                      <option value="category">Category</option>
                      <option value="active">Active</option>
                      <option value="geometry">Geometry</option>
                      <option value="createdAt">Created At</option>
                    </select>
                    <input 
                      placeholder="Search zone..." 
                      style={{ 
                        padding: "15px 10px", 
                        borderRadius: 8, 
                        border: "18px 190px solid var(--card-border)",
                        flex: 1,
                        minWidth: 220
                      }}
                      value={zoneSearchTerm}
                      onChange={(e) => setZoneSearchTerm(e.target.value)}
                      aria-label="Search zones"
                    />
                  </div>

                  <button 
                    onClick={() => setZoneFormOpen((v) => !v)} 
                    style={{ background: "#0b74d1", color: "#fff", padding: "14px 14px", borderRadius: 8, border: "none", cursor: "pointer" }}
                    disabled={!selectedMap}
                  >
                    + Create New Zone
                  </button>
                </div>

                {zoneFormOpen && (
                  <div className="zone-form">
                    <div className="zone-form-row">
                      <label>
                        Name
                        <input value={zoneForm.name} onChange={(e) => setZoneForm((prev) => ({ ...prev, name: e.target.value }))} placeholder="Zone name" />
                      </label>
                      <label>
                        Category
                        <select value={zoneForm.category} onChange={(e) => setZoneForm((prev) => ({ ...prev, category: e.target.value }))}>
                          <option value="Safe">Safe</option>
                          <option value="Caution">Caution</option>
                          <option value="No-Go">No-Go</option>
                        </select>
                      </label>
                      <label>
                        Geometry
                        <input value={zoneForm.geometry} onChange={(e) => setZoneForm((prev) => ({ ...prev, geometry: e.target.value }))} placeholder="Polygon(...)" />
                      </label>
                    </div>

                    <div className="zone-form-footer">
                      <label className="toggle-row">
                        <input type="checkbox" checked={zoneForm.active} onChange={() => setZoneForm((prev) => ({ ...prev, active: !prev.active }))} />
                        <div>
                          <strong>Zone Enabled</strong>
                          <p>{zoneForm.active ? "Robots can enter" : "Robots must avoid"}</p>
                        </div>
                      </label>
                      <div className="zone-form-actions">
                        <button className="ghost-btn" type="button" onClick={() => { setZoneFormOpen(false); setZoneForm({ name: "", category: "Safe", geometry: "", active: true }); }}>Cancel</button>
                        <button className="primary-btn" type="button" onClick={async () => {
                          if (!zoneForm.name.trim()) { toast?.error?.("Enter a zone name"); return; }
                          const payload = { name: zoneForm.name.trim(), category: zoneForm.category, geometry: zoneForm.geometry || "Polygon(...)", active: zoneForm.active };
                          try {
                            const response = await requestV1("/zones", { method: "POST", body: JSON.stringify(payload) });
                            const createdZone = response.item || payload;
                            setZones((prev) => [createdZone, ...prev]);
                            setZoneForm({ name: "", category: "Safe", geometry: "", active: true });
                            setZoneFormOpen(false);
                            toast?.success?.("Zone created");
                          } catch (error) {
                            console.error("Zone create error", error);
                            toast?.error?.(error.message || "Failed to create zone");
                          }
                        }}>Save Zone</button>
                      </div>
                    </div>
                  </div>
                )}

                <div style={{ borderTop: "1px solid var(--card-border, #e6eef2)", marginTop: 4 }} />

                <div>
                  <div style={{ background: "var(--card-bg)", borderRadius: 8, boxShadow: "0 1px 3px rgba(2,6,23,0.06)", overflow: "hidden" }}>
                    <div style={{ padding: "12px 16px", borderBottom: "1px solid var(--card-border, #eef2f6)", display: "flex", alignItems: "center", gap: 12 }}>
                      <div style={{ flex: 1, fontWeight: 700, color: "var(--text-color)" }}>
                        Zones {selectedMap && `for ${selectedMap.name}`}
                      </div>
                      <div style={{ color: "var(--muted-text, #94a3b8)", fontSize: 13 }}>
                        {(() => {
                          const filtered = filterBySearch(zones, zoneSearchTerm, zoneSearchField, {
                            any: "name",
                            name: "name",
                            category: "category",
                            active: "active",
                            geometry: "geometry",
                            createdAt: "createdAt",
                          });
                          return `${filtered.length} of ${zones.length} rows`;
                        })()}
                      </div>
                    </div>
                    <div style={{ padding: "8px 16px" }}>
                      <table style={{ width: "100%", borderCollapse: "collapse", background: "transparent" }}>
                        <thead style={{ background: "transparent", color: "var(--muted-text, #475569)", fontSize: 13 }}>
                          <tr>
                            <th style={{ textAlign: "left", padding: "12px 8px" }}>Name</th>
                            <th style={{ textAlign: "left", padding: "12px 8px" }}>Category</th>
                            <th style={{ textAlign: "center", padding: "12px 8px" }}>Active</th>
                            <th style={{ textAlign: "left", padding: "12px 8px" }}>Geometry</th>
                            <th style={{ textAlign: "right", padding: "12px 8px" }}>Created At</th>
                          </tr>
                        </thead>
                        <tbody>
                          {(() => {
                            const filtered = filterBySearch(zones, zoneSearchTerm, zoneSearchField, {
                              any: "name",
                              name: "name",
                              category: "category",
                              active: "active",
                              geometry: "geometry",
                              createdAt: "createdAt",
                            });
                            
                            if (filtered.length === 0) {
                              return (
                                <tr><td colSpan={5} style={{ padding: "36px 8px", textAlign: "center", color: "#94a3b8" }}>
                                  <div style={{ fontWeight: 700, color: "var(--text-color)", marginBottom: 6 }}>No Zones Found</div>
                                  <div>{zoneSearchTerm ? `No zones match "${zoneSearchTerm}"` : `Create zones for ${selectedMap?.name} to get started.`}</div>
                                </td></tr>
                              );
                            }

                            return filtered.map((zone) => (
                              <tr key={zone.id}>
                                <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", fontWeight: 700, color: "var(--text-color)" }}>{zone.name}</td>
                                <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", color: "var(--text-color)" }}>{zone.category}</td>
                                <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", textAlign: "center" }}>
                                  <span style={{ background: zone.active ? "#d1fae5" : "#fee2e2", color: zone.active ? "#047857" : "#b91c1c", padding: "4px 8px", borderRadius: 999, fontSize: 12 }}>
                                    {zone.active ? "Active" : "Disabled"}
                                  </span>
                                </td>
                                <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", color: "var(--muted-text, #6b7280)" }}>{zone.geometry}</td>
                                <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", textAlign: "right", color: "var(--muted-text, #6b7280)" }}>{zone.createdAt}</td>
                              </tr>
                            ));
                          })()}
                        </tbody>
                      </table>
                    </div>
                  </div>
                </div>
              </>
            )}
          </div>
        )}

        {/* WAYPOINTS - add search */}
        {rightPage === "waypoints" && (
          <div style={{ display: "flex", flexDirection: "column", gap: 12, padding: "0 16px" }}>
            {!selectedMap && (
              <div style={{ padding: "32px 16px", textAlign: "center", color: "var(--muted-text)" }}>
                <div style={{ fontWeight: 700, fontSize: 16, marginBottom: 8 }}>No Map Selected</div>
                <div>Please select a map from the Maps section to view and create waypoints.</div>
              </div>
            )}
            
            {selectedMap && (
              <>
                <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", gap: 12 }}>
                  <div style={{ display: "flex", alignItems: "center", gap: 8, flex: 1 }}>
                    <select 
                      style={{ padding: "15px 8px", borderRadius: 8, border: "1px solid var(--card-border)" }}
                      value={waypointSearchField || "any"}
                      onChange={(e) => setWaypointSearchField(e.target.value)}
                      aria-label="Search by field"
                    >
                      <option value="any">All Fields</option>
                      <option value="name">Name</option>
                      <option value="category">Category</option>
                      <option value="active">Active</option>
                      <option value="geom">Geometry</option>
                      <option value="createdAt">Created At</option>
                    </select>
                    <input 
                      placeholder="Search waypoint..." 
                      style={{ 
                        padding: "16px 10px", 
                        borderRadius: 8, 
                        border: "1px solid var(--card-border)",
                        flex: 1,
                        minWidth: 220
                      }}
                      value={waypointSearchTerm || ""}
                      onChange={(e) => setWaypointSearchTerm(e.target.value)}
                      aria-label="Search waypoints"
                    />
                  </div>

                  <button 
                    style={{ background: "#0b74d1", color: "#fff", padding: "14px 14px", borderRadius: 8, border: "none", cursor: "pointer" }} 
                    onClick={() => setWaypointFormOpen((v) => !v)}
                    disabled={!selectedMap}
                  >
                    + Create New Waypoint
                  </button>
                </div>

                {waypointFormOpen && (
                  <div className="waypoint-form">
                    <div className="zone-form-row">
                      <label>
                        Name
                        <input value={waypointForm.name} onChange={(e) => setWaypointForm((prev) => ({ ...prev, name: e.target.value }))} placeholder="Waypoint name" />
                      </label>
                      <label>
                        Category
                        <select value={waypointForm.category} onChange={(e) => setWaypointForm((prev) => ({ ...prev, category: e.target.value }))}>
                          <option value="Nav">Nav</option>
                          <option value="Inspect">Inspect</option>
                          <option value="Charge">Charge</option>
                        </select>
                      </label>
                      <label>
                        Geometry
                        <input value={waypointForm.geom} onChange={(e) => setWaypointForm((prev) => ({ ...prev, geom: e.target.value }))} placeholder="Point(x y)" />
                      </label>
                    </div>
                    <label>
                      Notes
                      <input value={waypointForm.notes} onChange={(e) => setWaypointForm((prev) => ({ ...prev, notes: e.target.value }))} placeholder="Optional operator note" />
                    </label>
                    <div className="zone-form-footer">
                      <label className="toggle-row">
                        <input type="checkbox" checked={waypointForm.active} onChange={() => setWaypointForm((prev) => ({ ...prev, active: !prev.active }))} />
                        <div>
                          <strong>Waypoint Active</strong>
                          <p>{waypointForm.active ? "Included in missions" : "Hidden from routing"}</p>
                        </div>
                      </label>
                      <div className="zone-form-actions">
                        <button className="ghost-btn" type="button" onClick={() => { setWaypointFormOpen(false); setWaypointForm({ name: "", category: "Nav", geom: "", notes: "", active: true }); }}>Cancel</button>
                        <button className="primary-btn" type="button" onClick={async () => {
                          if (!waypointForm.name.trim()) { toast?.error?.("Enter a waypoint name"); return; }
                          if (!waypointForm.geom.trim()) { toast?.error?.("Add waypoint coordinates"); return; }
                          const payload = { 
                            id: `wp-${Date.now()}`,
                            mapId: selectedMap.id,
                            name: waypointForm.name.trim(), 
                            category: waypointForm.category, 
                            geom: waypointForm.geom || "Point(0 0)", 
                            notes: waypointForm.notes, 
                            active: waypointForm.active,
                            createdAt: new Date().toLocaleString()
                          };
                          setWaypoints((prev) => [payload, ...prev]);
                          setSelectedWaypointId(payload.id);
                          setWaypointForm({ name: "", category: "Nav", geom: "", notes: "", active: true });
                          setWaypointFormOpen(false);
                          toast?.success?.("Waypoint created");
                        }}>Save Waypoint</button>
                      </div>
                    </div>
                  </div>
                )}

                {/* waypoints table */}
                <div style={{ background: "var(--card-bg)", borderRadius: 8, boxShadow: "0 1px 3px rgba(2,6,23,0.06)", overflow: "hidden" }}>
                  <div style={{ padding: "12px 16px", borderBottom: "1px solid var(--card-border, #eef2f6)", display: "flex", alignItems: "center", gap: 12 }}>
                    <div style={{ flex: 1, fontWeight: 700, color: "var(--text-color)" }}>
                      Waypoints {selectedMap && `for ${selectedMap.name}`}
                    </div>
                    <div style={{ color: "var(--muted-text, #94a3b8)", fontSize: 13 }}>Rows per page: 10</div>
                  </div>
                  <div style={{ padding: "8px 16px" }}>
                    <table style={{ width: "100%", borderCollapse: "collapse", background: "transparent" }}>
                      <thead style={{ background: "transparent", color: "var(--muted-text, #475569)", fontSize: 13 }}>
                        <tr>
                          <th style={{ textAlign: "left", padding: "12px 8px" }}>Name</th>
                          <th style={{ textAlign: "left", padding: "12px 8px" }}>Map</th>
                          <th style={{ textAlign: "left", padding: "12px 8px" }}>Category</th>
                          <th style={{ textAlign: "center", padding: "12px 8px" }}>Active</th>
                          <th style={{ textAlign: "left", padding: "12px 8px" }}>Geometry</th>
                          <th style={{ textAlign: "right", padding: "12px 8px" }}>Created At</th>
                        </tr>
                      </thead>
                      <tbody>
                        {(() => {
                          const filtered = filterBySearch(waypoints, waypointSearchTerm || "", waypointSearchField || "any", {
                            any: "name",
                            name: "name",
                            category: "category",
                            active: "active",
                            geom: "geom",
                            createdAt: "createdAt",
                          });
                          
                          if (filtered.length === 0) {
                            return (
                              <tr><td colSpan={6} style={{ padding: "36px 8px", textAlign: "center", color: "#94a3b8" }}>
                                <div style={{ fontWeight: 700, color: "var(--text-color)", marginBottom: 6 }}>No Waypoints Found</div>
                                <div>{waypointSearchTerm ? `No waypoints match "${waypointSearchTerm}"` : `Create waypoints for ${selectedMap?.name} to get started.`}</div>
                              </td></tr>
                            );
                          }

                          return filtered.map((wp) => {
                            const waypointMapName = props.mapsList?.find(m => m.id === wp.mapId)?.name || wp.mapId || "Unknown";
                            return (
                              <tr key={wp.id} onClick={() => handleSelectWaypoint(wp.id)} style={{ cursor: "pointer", background: wp.id ? "transparent" : "rgba(3,48,80,0.04)" }}>
                                <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", fontWeight: 700, color: "var(--text-color)" }}>{wp.name}</td>
                                <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", color: "var(--muted-text, #6b7280)", fontSize: 12 }}>{waypointMapName}</td>
                                <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", color: "var(--muted-text, #6b7280)" }}>{wp.category}</td>
                                <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", textAlign: "center" }}>
                                  <span style={{ background: wp.active ? "#d1fae5" : "#fee2e2", color: wp.active ? "#047857" : "#b91c1c", padding: "4px 8px", borderRadius: 999, fontSize: 12 }}>
                                    {wp.active ? "Active" : "Disabled"}
                                  </span>
                                </td>
                                <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", color: "var(--muted-text, #6b7280)" }}>{wp.geom}</td>
                                <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", textAlign: "right", color: "var(--muted-text, #6b7280)" }}>{wp.createdAt}</td>
                              </tr>
                            );
                          });
                        })()}
                      </tbody>
                    </table>
                  </div>
                </div>
              </>
            )}
          </div>
        )}

        {/* MISSIONS - add search */}
        {rightPage === "missions" && (
          <div style={{ display: "flex", flexDirection: "column", gap: 12, padding: "0 16px" }}>
            <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", gap: 12 }}>
              <div style={{ display: "flex", alignItems: "center", gap: 8, flex: 1 }}>
                <select 
                  style={{ padding: "15px 8px", borderRadius: 8, border: "1px solid var(--card-border)" }}
                  value={missionSearchField || "any"}
                  onChange={(e) => setMissionSearchField(e.target.value)}
                  aria-label="Search by field"
                >
                  <option value="any">All Fields</option>
                  <option value="name">Name</option>
                  <option value="owner">Owner</option>
                  <option value="status">Status</option>
                  <option value="createdAt">Created At</option>
                </select>
                <input 
                  placeholder="Search mission..." 
                  style={{ 
                    padding: "15px 10px", 
                    borderRadius: 8, 
                    border: "1px solid var(--card-border)",
                    flex: 1,
                    minWidth: 220
                  }}
                  value={missionSearchTerm || ""}
                  onChange={(e) => setMissionSearchTerm(e.target.value)}
                  aria-label="Search missions"
                />
              </div>
              <button 
                style={{ background: "#0b74d1", color: "#fff", padding: "14px 14px", borderRadius: 8, border: "none", cursor: "pointer" }} 
                onClick={() => setMissionFormOpen((v) => !v)}
              >
                + Create Mission
              </button>
            </div>

            {missionFormOpen && (
              <div className="mission-form">
                <div className="zone-form-row">
                  <label>
                    Name
                    <input value={missionForm.name} onChange={(e) => setMissionForm((prev) => ({ ...prev, name: e.target.value }))} placeholder="Mission name" />
                  </label>
                  <label>
                    Owner
                    <input value={missionForm.owner} onChange={(e) => setMissionForm((prev) => ({ ...prev, owner: e.target.value }))} placeholder="Owner or department" />
                  </label>
                  <label>
                    Status
                    <select value={missionForm.status} onChange={(e) => setMissionForm((prev) => ({ ...prev, status: e.target.value }))}>
                      <option value="Draft">Draft</option>
                      <option value="Scheduled">Scheduled</option>
                      <option value="In Progress">In Progress</option>
                      <option value="Completed">Completed</option>
                    </select>
                  </label>
                </div>
                <label>
                  Notes
                  <textarea value={missionForm.notes} onChange={(e) => setMissionForm((prev) => ({ ...prev, notes: e.target.value }))} placeholder="Mission objectives, constraints, etc." className="mission-notes" />
                </label>
                <div className="zone-form-actions" style={{ marginLeft: "auto" }}>
                  <button className="ghost-btn" type="button" onClick={() => { setMissionFormOpen(false); setMissionForm({ name: "", owner: "", status: "Draft", notes: "" }); }}>Cancel</button>
                  <button className="primary-btn" type="button" onClick={async () => {
                    if (!missionForm.name.trim()) { toast?.error?.("Mission name required"); return; }
                    if (!missionForm.owner.trim()) { toast?.error?.("Mission owner required"); return; }
                    const payload = { name: missionForm.name.trim(), owner: missionForm.owner.trim(), status: missionForm.status, notes: missionForm.notes };
                    try {
                      const response = await requestV1("/missions", { method: "POST", body: JSON.stringify(payload) });
                      const createdMission = response.item || payload;
                      setMissions((prev) => [createdMission, ...prev]);
                      setSelectedMissionId(createdMission.id);
                      setMissionForm({ name: "", owner: "", status: "Draft", notes: "" });
                      setMissionFormOpen(false);
                      toast?.success?.("Mission saved");
                    } catch (error) {
                      console.error("Mission create error", error);
                      toast?.error?.(error.message || "Failed to create mission");
                    }
                  }}>Save Mission</button>
                </div>
              </div>
            )}

            <div style={{ borderTop: "1px solid var(--card-border, #e6eef2)", marginTop: 4 }} />

            <div style={{ background: "var(--card-bg)", borderRadius: 8, boxShadow: "0 1px 3px rgba(2,6,23,0.06)", overflow: "hidden" }}>
              <div style={{ padding: "12px 16px", borderBottom: "1px solid var(--card-border, #eef2f6)", display: "flex", alignItems: "center", gap: 12 }}>
                <div style={{ flex: 1, fontWeight: 700, color: "var(--text-color)" }}>
                  Missions {selectedMap && `for ${selectedMap.name}`}
                </div>
                <div style={{ color: "var(--muted-text, #94a3b8)", fontSize: 13 }}>
                  {(() => {
                    const filtered = filterBySearch(missions, missionSearchTerm || "", missionSearchField || "any", {
                      any: "name",
                      name: "name",
                      owner: "owner",
                      status: "status",
                      createdAt: "createdAt",
                    });
                    return `${filtered.length} of ${missions.length} rows`;
                  })()}
                </div>
              </div>
              <div style={{ padding: "8px 16px" }}>
                <table style={{ width: "100%", borderCollapse: "collapse", background: "transparent" }}>
                  <thead style={{ background: "transparent", color: "var(--muted-text, #475569)", fontSize: 13 }}>
                    <tr>
                      <th style={{ textAlign: "left", padding: "12px 8px" }}>Name</th>
                      <th style={{ textAlign: "left", padding: "12px 8px" }}>Map</th>
                      <th style={{ textAlign: "left", padding: "12px 8px" }}>Owner</th>
                      <th style={{ textAlign: "center", padding: "12px 8px" }}>Status</th>
                      <th style={{ textAlign: "right", padding: "12px 8px" }}>Created At</th>
                    </tr>
                  </thead>
                  <tbody>
                    {(() => {
                      const filtered = filterBySearch(missions, missionSearchTerm || "", missionSearchField || "any", {
                        any: "name",
                        name: "name",
                        owner: "owner",
                        status: "status",
                        createdAt: "createdAt",
                      });
                      
                      if (filtered.length === 0) {
                        return (
                          <tr><td colSpan={5} style={{ padding: "36px 8px", textAlign: "center", color: "#94a3b8" }}>
                            <div style={{ fontWeight: 700, color: "var(--text-color)", marginBottom: 6 }}>No Missions Found</div>
                            <div>{missionSearchTerm ? `No missions match "${missionSearchTerm}"` : `Create missions for ${selectedMap?.name} to get started.`}</div>
                          </td></tr>
                        );
                      }

                      return filtered.map((m) => {
                        const missionMapName = props.mapsList?.find(mp => mp.id === m.mapId)?.name || m.mapId || "Unknown";
                        return (
                          <tr key={m.id} onClick={() => handleSelectMission(m.id)} style={{ cursor: "pointer", background: selectedMissionId === m.id ? "rgba(3,48,80,0.04)" : "transparent" }}>
                            <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", fontWeight: 700 }}>{m.name}</td>
                            <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", color: "var(--muted-text, #6b7280)", fontSize: 12 }}>{missionMapName}</td>
                            <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", color: "var(--muted-text, #6b7280)" }}>{m.owner}</td>
                            <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", textAlign: "center" }}>
                              <span style={{ background: m.status === "Completed" ? "#bbf7d0" : "#fee2e2", color: m.status === "Completed" ? "#065f46" : "#9b1b1b", padding: "4px 8px", borderRadius: 8, fontSize: 12 }}>{m.status}</span>
                            </td>
                            <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", textAlign: "right", color: "var(--muted-text, #6b7280)" }}>{m.createdAt}</td>
                          </tr>
                        );
                      });
                    })()}
                  </tbody>
                </table>
              </div>
            </div>

            <div style={{ background: "var(--card-bg)", borderRadius: 8, padding: 12, boxShadow: "0 1px 3px rgba(2,6,23,0.06)", color: "var(--text-color)" }}>
              {selectedMissionId ? (() => {
                const m = (missions || []).find((x) => x.id === selectedMissionId);
                if (!m) return <div style={{ color: "#94a3b8" }}>Mission not found.</div>;
                return (
                  <div>
                    <div style={{ fontWeight: 800, fontSize: 16 }}>{m.name}</div>
                    <div style={{ marginTop: 8, color: "var(--muted-text, #6b7280)" }}><strong>Owner:</strong> {m.owner}</div>
                    <div style={{ marginTop: 6, color: "var(--muted-text, #6b7280)" }}><strong>Status:</strong> {m.status}</div>
                    <div style={{ marginTop: 6, color: "var(--muted-text, #6b7280)" }}><strong>Created At:</strong> {m.createdAt}</div>
                    <div style={{ marginTop: 10, color: "#475569" }}>{m.notes || "No notes."}</div>
                  </div>
                );
              })() : <div style={{ color: "#94a3b8" }}>Select a mission to see details.</div>}
            </div>
          </div>
        )}

        {/* USERS */}
        {rightPage === "users" && (
          <div style={{ display: "flex", flexDirection: "column", gap: 12, padding: "0 16px" }}>
            {usersError && (
              <div style={{ padding: 12, background: "#fee2e2", color: "#b91c1c", borderRadius: 8, fontSize: 14 }}>
                {usersError}
              </div>
            )}

            {usersLoading && (
              <div style={{ padding: 32, textAlign: "center", color: "var(--muted-text)" }}>
                Loading users...
              </div>
            )}

            {!usersLoading && !usersError && (
              <>
                <div style={{ display: "flex", alignItems: "center", gap: 8, flex: 1 }}>
                  <select 
                    style={{ padding: "15px 8px", borderRadius: 8, border: "1px solid var(--card-border)" }}
                    value={userSearchField}
                    onChange={(e) => setUserSearchField(e.target.value)}
                    aria-label="Search by field"
                  >
                    <option value="any">All Fields</option>
                    <option value="username">Username</option>
                    <option value="email">Email</option>
                    <option value="company">Company</option>
                    <option value="amr_type">AMR Type</option>
                    <option value="role">Role</option>
                    <option value="approval">Approval</option>
                  </select>
                  <input 
                    placeholder="Search user..." 
                    style={{ 
                      padding: "15px 10px", 
                      borderRadius: 8, 
                      border: "1px solid var(--card-border)",
                      flex: 1,
                      minWidth: 220
                    }}
                    value={userSearchTerm}
                    onChange={(e) => setUserSearchTerm(e.target.value)}
                    aria-label="Search users"
                  />
                  <button 
                    onClick={loadUsers} 
                    style={{ 
                      padding: "15px 26px", 
                      background: "#0b74d1", 
                      color: "#fff", 
                      border: "none", 
                      borderRadius: 6, 
                      cursor: "pointer",
                      fontSize: 13
                    }}
                  >
                    Refresh
                  </button>
                </div>

                {/* Edit User Form (shows when editing) */}
                {editingUserId && (() => {
                  const user = users.find((u) => u.id === editingUserId);
                  if (!user) return null;
                  
                  return (
                    <div style={{ background: "var(--card-bg)", border: "2px solid #0b74d1", borderRadius: 8, padding: 16, marginBottom: 12 }}>
                      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 12 }}>
                        <h3 style={{ margin: 0, fontSize: 16, fontWeight: 700 }}>Edit User: {user.username}</h3>
                        <button 
                          onClick={() => {
                            setEditingUserId(null);
                            setEditUserForm({ username: "", email: "", company: "", amr_type: "", role: "", approval: "" });
                          }}
                          style={{ background: "transparent", border: "none", fontSize: 18, cursor: "pointer", color: "var(--text-color)" }}
                        >
                          ✕
                        </button>
                      </div>
                      
                      <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 12 }}>
                        <label style={{ display: "flex", flexDirection: "column", gap: 4 }}>
                          <span style={{ fontSize: 13, fontWeight: 600, color: "var(--muted-text)" }}>Username</span>
                          <input 
                            value={editUserForm.username} 
                            onChange={(e) => setEditUserForm(prev => ({ ...prev, username: e.target.value }))}
                            style={{ padding: "8px", borderRadius: 6, border: "1px solid var(--card-border)" }}
                          />
                        </label>
                        
                        <label style={{ display: "flex", flexDirection: "column", gap: 4 }}>
                          <span style={{ fontSize: 13, fontWeight: 600, color: "var(--muted-text)" }}>Email</span>
                          <input 
                            value={editUserForm.email} 
                            onChange={(e) => setEditUserForm(prev => ({ ...prev, email: e.target.value }))}
                            style={{ padding: "8px", borderRadius: 6, border: "1px solid var(--card-border)" }}
                          />
                        </label>
                        
                        <label style={{ display: "flex", flexDirection: "column", gap: 4 }}>
                          <span style={{ fontSize: 13, fontWeight: 600, color: "var(--muted-text)" }}>Company</span>
                          <input 
                            value={editUserForm.company} 
                            onChange={(e) => setEditUserForm(prev => ({ ...prev, company: e.target.value }))}
                            style={{ padding: "8px", borderRadius: 6, border: "1px solid var(--card-border)" }}
                          />
                        </label>
                        
                        <label style={{ display: "flex", flexDirection: "column", gap: 4 }}>
                          <span style={{ fontSize: 13, fontWeight: 600, color: "var(--muted-text)" }}>AMR Type</span>
                          <input 
                            value={editUserForm.amr_type} 
                            onChange={(e) => setEditUserForm(prev => ({ ...prev, amr_type: e.target.value }))}
                            style={{ padding: "8px", borderRadius: 6, border: "1px solid var(--card-border)" }}
                          />
                        </label>
                        
                        <label style={{ display: "flex", flexDirection: "column", gap: 4 }}>
                          <span style={{ fontSize: 13, fontWeight: 600, color: "var(--muted-text)" }}>Role</span>
                          <select 
                            value={editUserForm.role} 
                            onChange={(e) => setEditUserForm(prev => ({ ...prev, role: e.target.value }))}
                            style={{ padding: "8px", borderRadius: 6, border: "1px solid var(--card-border)" }}
                          >
                            <option value="user">User</option>
                            <option value="admin">Admin</option>
                          </select>
                        </label>
                        
                        <label style={{ display: "flex", flexDirection: "column", gap: 4 }}>
                          <span style={{ fontSize: 13, fontWeight: 600, color: "var(--muted-text)" }}>Approval</span>
                          <select 
                            value={editUserForm.approval} 
                            onChange={(e) => setEditUserForm(prev => ({ ...prev, approval: e.target.value }))}
                            style={{ padding: "8px", borderRadius: 6, border: "1px solid var(--card-border)" }}
                          >
                            <option value="Pending">Pending</option>
                            <option value="Approved">Approved</option>
                            <option value="Rejected">Rejected</option>
                          </select>
                        </label>
                      </div>
                      
                      <div style={{ display: "flex", gap: 8, marginTop: 16, justifyContent: "flex-end" }}>
                        <button 
                          onClick={() => {
                            setEditingUserId(null);
                            setEditUserForm({ username: "", email: "", company: "", amr_type: "", role: "", approval: "" });
                          }}
                          style={{ padding: "8px 16px", borderRadius: 6, border: "1px solid var(--card-border)", background: "transparent", cursor: "pointer" }}
                        >
                          Cancel
                        </button>
                        <button 
                          onClick={async () => {
                            try {
                              // Update users state locally (optimistic update)
                              const updatedUsers = users.map(u => 
                                u.id === editingUserId 
                                  ? { ...u, ...editUserForm } 
                                  : u
                              );
                              // Update parent state if setUsers prop exists
                              if (props.setUsers) {
                                props.setUsers(updatedUsers);
                              }
                              
                              toast?.success?.("User updated successfully");
                              setEditingUserId(null);
                              setEditUserForm({ username: "", email: "", company: "", amr_type: "", role: "", approval: "" });
                            } catch (error) {
                              toast?.error?.("Failed to update user");
                            }
                          }}
                          style={{ padding: "8px 16px", borderRadius: 6, border: "none", background: "#0b74d1", color: "#fff", cursor: "pointer" }}
                        >
                          Save Changes
                        </button>
                      </div>
                    </div>
                  );
                })()}

                <div style={{ background: "var(--card-bg)", borderRadius: 8, boxShadow: "0 1px 3px rgba(2,6,23,0.06)", overflow: "hidden" }}>
                  <div style={{ padding: "12px 16px", borderBottom: "1px solid var(--card-border)", display: "flex", alignItems: "center", gap: 12 }}>
                    <div style={{ flex: 1, fontWeight: 700, color: "var(--text-color)" }}>
                      Registered Users ({users.length})
                    </div>
                    <div style={{ color: "var(--muted-text, #94a3b8)", fontSize: 13 }}>
                      {(() => {
                        const filtered = filterBySearch(users, userSearchTerm, userSearchField, {
                          any: "username",
                          username: "username",
                          email: "email",
                          company: "company",
                          amr_type: "amr_type",
                          role: "role",
                          approval: "approval",
                        });
                        return `${filtered.length} of ${users.length} rows`;
                      })()}
                    </div>
                  </div>

                  <div style={{ padding: "8px 16px", overflowX: "auto" }}>
                    <table style={{ width: "100%", borderCollapse: "collapse", background: "transparent" }}>
                      <thead style={{ background: "transparent", color: "var(--muted-text)", fontSize: 13 }}>
                        <tr>
                          <th style={{ textAlign: "left", padding: "12px 8px" }}>Username</th>
                          <th style={{ textAlign: "left", padding: "12px 8px" }}>Email</th>
                          <th style={{ textAlign: "left", padding: "12px 8px" }}>Company</th>
                          <th style={{ textAlign: "left", padding: "12px 8px" }}>AMR Type</th>
                          <th style={{ textAlign: "center", padding: "12px 8px" }}>Role</th>
                          <th style={{ textAlign: "center", padding: "12px 8px" }}>Approval</th>
                          <th style={{ textAlign: "right", padding: "12px 8px" }}>Actions</th>
                        </tr>
                      </thead>
                      <tbody>
                        {(() => {
                          const filtered = filterBySearch(users, userSearchTerm, userSearchField, {
                            any: "username",
                            username: "username",
                            email: "email",
                            company: "company",
                            amr_type: "amr_type",
                            role: "role",
                            approval: "approval",
                          });

                          if (filtered.length === 0) {
                            return (
                              <tr>
                                <td colSpan={7} style={{ padding: "36px 8px", textAlign: "center", color: "var(--muted-text)" }}>
                                  <div style={{ fontWeight: 700, color: "var(--text-color)", marginBottom: 6 }}>No Users Found</div>
                                  <div>{userSearchTerm ? `No users match "${userSearchTerm}"` : "No users have registered yet."}</div>
                                </td>
                              </tr>
                            );
                          }

                          return filtered.map((user) => (
                            <tr 
                              key={user.id} 
                              onClick={() => setSelectedUserId(user.id)}
                              style={{ 
                                cursor: "pointer", 
                                background: selectedUserId === user.id ? "rgba(3,48,80,0.04)" : "transparent" 
                              }}
                            >
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border)", fontWeight: 700, color: "var(--text-color)" }}>
                                {user.username}
                              </td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border)", color: "var(--muted-text)" }}>
                                {user.email}
                              </td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border)", color: "var(--muted-text)" }}>
                                {user.company || "N/A"}
                              </td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border)", color: "var(--muted-text)" }}>
                                {user.amr_type || "N/A"}
                              </td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border)", textAlign: "center" }}>
                                <span style={{ 
                                  background: user.role === "admin" ? "#dbeafe" : "#f3f4f6", 
                                  color: user.role === "admin" ? "#1e40af" : "#6b7280", 
                                  padding: "4px 10px", 
                                  borderRadius: 999, 
                                  fontSize: 12,
                                  fontWeight: 600
                                }}>
                                  {user.role}
                                </span>
                              </td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border)", textAlign: "center" }}>
                                <span style={{ 
                                  background: 
                                    user.approval === "Approved" ? "#d1fae5" : 
                                    user.approval === "Rejected" ? "#fee2e2" : 
                                    "#fef3c7",
                                  color: 
                                    user.approval === "Approved" ? "#047857" : 
                                    user.approval === "Rejected" ? "#b91c1c" : 
                                    "#b45309",
                                  padding: "4px 10px", 
                                  borderRadius: 999, 
                                  fontSize: 12,
                                  fontWeight: 600
                                }}>
                                  {user.approval}
                                </span>
                              </td>
                              <td style={{ padding: "12px 8px", borderBottom: "1px solid var(--card-border, #eef2f6)", textAlign: "right" }} onClick={(e) => e.stopPropagation()}>
                                <div style={{ display: "flex", gap: 8, justifyContent: "flex-end" }}>
                                  <button 
                                    onClick={(e) => {

                                      e.stopPropagation();
                                      setEditingUserId(user.id);
                                      setEditUserForm({
                                        username: user.username,
                                        email: user.email,
                                        company: user.company || "",
                                        amr_type: user.amr_type || "",
                                        role: user.role,
                                        approval: user.approval,
                                      });
                                    }}
                                    style={{
                                      padding: "4px 10px",
                                      background: "#0b74d1",
                                      color: "#fff",
                                      border: "none",
                                      borderRadius: 6,
                                      cursor: "pointer",
                                      fontSize: 12,
                                      display: "flex",
                                      alignItems: "center",
                                      gap: 4
                                    }}
                                    title="Edit user"
                                  >
                                    <FaEdit /> Edit
                                  </button>
                                  <button 
                                    onClick={(e) => {
                                      e.stopPropagation();
                                      handleResetUserPassword();
                                    }}
                                    disabled={userActionLoading || selectedUserId !== user.id}
                                    style={{
                                      padding: "4px 10px",
                                      background: selectedUserId === user.id && !userActionLoading ? "#10b981" : "#e5e7eb",
                                      color: selectedUserId === user.id && !userActionLoading ? "#fff" : "#9ca3af",
                                      border: "none",
                                      borderRadius: 6,
                                      cursor: selectedUserId === user.id && !userActionLoading ? "pointer" : "not-allowed",
                                      fontSize: 12
                                    }}
                                    title="Reset password"
                                  >
                                    {userActionLoading && selectedUserId === user.id ? "..." : "Reset Pwd"}
                                  </button>
                                </div>
                              </td>
                            </tr>
                          ));
                        })()}
                      </tbody>
                    </table>
                  </div>
                </div>
              </>
            )}

            {selectedUserId && (
              <div style={{ background: "var(--card-bg)", borderRadius: 8, padding: 16, boxShadow: "0 1px 3px rgba(2,6,23,0.06)" }}>
                {(() => {
                  const user = users.find((u) => u.id === selectedUserId);
                  if (!user) return <div style={{ color: "var(--muted-text)" }}>Select a user to view details</div>;
                  return (
                    <div>
                      <div style={{ fontWeight: 700, fontSize: 16, marginBottom: 12 }}>User Details</div>
                      <div style={{ display: "grid", gap: 8, fontSize: 14 }}>
                        <div><strong>ID:</strong> {user.id}</div>
                        <div><strong>Username:</strong> {user.username}</div>
                        <div><strong>Email:</strong> {user.email}</div>
                        <div><strong>Company:</strong> {user.company || "N/A"}</div>
                        <div><strong>AMR Type:</strong> {user.amr_type || "N/A"}</div>
                        <div><strong>Role:</strong> {user.role}</div>
                        <div><strong>Approval Status:</strong> {user.approval}</div>
                      </div>
                    </div>
                  );
                })()}
              </div>
            )}
          </div>
        )}

        {/* ANALYTICS */}
        {rightPage === "analytics" && (
          <div className="analytics-pane">
            <div className="analytics-kpis">
              {analyticsSummary.map((card) => (
                <div key={card.label} className="kpi-card" style={{ background: "var(--card-bg)", color: "var(--text-color)", border: "1px solid var(--card-border)" }}>
                  <span className="kpi-label">{card.label}</span>
                  <strong>{card.value}</strong>
                  <span className="kpi-trend">{card.trend}</span>
                </div>
              ))}
            </div>
            <div className="analytics-chart-card" style={{ background: "var(--card-bg)", color: "var(--text-color)" }}>
              <div className="stats-card-header">
                <div>
                  <h4>Cycle Throughput</h4>
                  <p>Rolling seven-day view</p>
                </div>
              </div>
              <svg width={analyticsChartSize.width} height={analyticsChartSize.height} className="analytics-chart">
                <polyline points={analyticsPath} fill="none" strokeWidth="3" className="analytics-line" />
              </svg>
            </div>
            <div className="analytics-alerts">
              {analyticsAlerts.map((alert) => (
                <div key={alert.id} className="alert-card">
                  <strong>{alert.title}</strong>
                  <p>{alert.detail}</p>
                </div>
              ))}
            </div>
          </div>
        )}

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

        {/* ROBOT SETTINGS */}
        {rightPage === "robot-settings" && (
          <div className="settings-pane">
            {Object.entries(robotSettingsState).map(([key, value]) => (
              <label key={key} className="toggle-row">
                <input type="checkbox" checked={value} onChange={() => toggleRobotSetting(key)} />
                <div>
                  <strong>{key.replace(/([A-Z])/g, " $1")}</strong>
                  <p>{value ? "Enabled" : "Disabled"}</p>
                </div>
              </label>
            ))}
          </div>
        )}

        {/* ACCOUNT */}
        {rightPage === "account" && (
          <div className="account-pane">
            <label>
              Name
              <input value={accountProfile.fullName} onChange={(e) => handleAccountChange("fullName", e.target.value)} />
            </label>
            <label>
              Email
              <input value={accountProfile.email} onChange={(e) => handleAccountChange("email", e.target.value)} />
            </label>
            <label>
              Team
              <input value={accountProfile.team} onChange={(e) => handleAccountChange("team", e.target.value)} />
            </label>
            <label>
              Shift Window
              <input value={accountProfile.shift} onChange={(e) => handleAccountChange("shift", e.target.value)} />
            </label>
            {profileError && <p style={{ color: "red" }}>{profileError}</p>}
            {profileSuccess && <p style={{ color: "green" }}>{profileSuccess}</p>}
            <button className="primary-btn" type="button" onClick={handleSaveProfile} disabled={profileSaving}>{profileSaving ? "Saving..." : "Save Profile"}</button>
          </div>
        )}
{/* APPEARANCE */}
{rightPage === "appearance" && (
  <div className="appearance-pane">
    {[
      { id: "light", label: "Light" },
      { id: "dark", label: "Dark" },
      { id: "system", label: "Match System" },
    ].map((theme) => (
      <button
        key={theme.id}
        className={`theme-card ${selectedTheme === theme.id ? "active" : ""}`}
        onClick={() => setSelectedTheme(theme.id)}
      >
        <strong>{theme.label}</strong>
        <span>{selectedTheme === theme.id ? "Selected" : "Use theme"}</span>
      </button>
    ))}
  </div>
)}


        {/* SECURITY */}
        {rightPage === "security" && (
          <div className="security-pane">
            <div className="security-toggles">
              {Object.entries(securityPreferences).map(([key, value]) => (
                <label key={key} className="toggle-row">
                  <input type="checkbox" checked={value} onChange={() => toggleSecurityPref(key)} />
                  <div>
                    <strong>{key.replace(/([A-Z])/g, " $1")}</strong>
                    <p>{value ? "Enabled" : "Disabled"}</p>
                  </div>
                </label>
              ))}
            </div>
            <table className="security-table">
              <thead>
                <tr>
                  <th>Time</th>
                  <th>Actor</th>
                  <th>Action</th>
                  <th>Context</th>
                </tr>
              </thead>
              <tbody>
                {securityEvents.map((evt) => (
                  <tr key={evt.id}>
                    <td>{evt.ts}</td>
                    <td>{evt.actor}</td>
                    <td>{evt.action}</td>
                    <td>{evt.context}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}

        {/* INTEGRATIONS */}
        {rightPage === "integrations" && (
          <div className="integrations-pane">
            {integrationItems.map((integration) => (
              <div key={integration.id} className="integration-card">
                <div>
                  <strong>{integration.name}</strong>
                  <p>{integration.description}</p>
                </div>
                <div className="integration-meta">
                  <span className={`integration-status ${integration.status === "Connected" ? "ok" : "off"}`}>{integration.status}</span>
                  <button className="ghost-btn" onClick={() => toggleIntegrationStatus && toggleIntegrationStatus(integration.id)}>{integration.status === "Connected" ? "Disconnect" : "Connect"}</button>
                </div>
              </div>
            ))}
          </div>
        )}

        {/* STATS */}
        {rightPage === "stats" && (
          <div className="stats-pane">
            <div className="stats-status-row">
              {statsLoading && <span className="stats-status">Refreshing telemetry…</span>}
              {statsError && <span className="stats-error">{statsError}</span>}
            </div>
            <div className="stats-summary">
              <div className="stats-card">
                <span className="stats-label">Total Moving Distance</span>
                <h3>{overview.totalKm.toFixed(1)} km</h3>
                <p>+{(typeof overview.deltaKm === "number" ? overview.deltaKm.toFixed(1) : overview.deltaKm)} km vs previous day</p>
              </div>
              <div className="stats-card">
                <span className="stats-label">Missions Completed</span>
                <h3>{overview.missionsCompleted}</h3>
                <p>{overview.missionSuccessRate}% success over 7 days</p>
              </div>
              <div className="stats-card">
                <span className="stats-label">Average Speed</span>
                <h3>{overview.avgSpeed.toFixed(1)} m/s</h3>
                <p>Within safe corridor</p>
              </div>
              <div className="stats-card">
                <span className="stats-label">Operating Hours</span>
                <h3>{overview.operatingHours} h</h3>
                <p>Last maintenance at 300 h</p>
              </div>
            </div>

            <div className="movement-card">
              <div className="stats-card-header">
                <div><h4>Monthly Movement</h4><p>Distance travelled per month</p></div>
              </div>
              <div className="movement-summary">
                <div><span className="movement-label">Total</span><strong>{monthlyMovement.reduce((s,e)=>s+(e.km||0),0).toFixed(1)} km</strong></div>
                <div><span className="movement-label">Average / month</span><strong>{monthlyMovement.length ? (monthlyMovement.reduce((s,e)=>s+(e.km||0),0)/monthlyMovement.length).toFixed(1) : "0.0"} km</strong></div>
              </div>
              <div className="movement-bars">
                {monthlyMovement.map((entry) => (
                  <div key={entry.month} className="movement-bar">
                    <div className="movement-bar-fill" style={{ height: `${(entry.km / (monthlyMovement.length ? Math.max(...monthlyMovement.map(m=>m.km)) : 1)) * 100}%` }} title={`${entry.km} km`} />
                    <span>{entry.month}</span>
                  </div>
                ))}
              </div>
            </div>

            <div className="stats-grid">
              <div className="stats-chart-card">
                <div className="stats-card-header">
                  <div><h4>Battery Voltage & Power</h4><p>Live pack telemetry</p></div>
                  <div className="stats-legend"><span className="legend-dot voltage" /> Voltage <span className="legend-dot power" /> Power</div>
                </div>
                <div className="line-chart">
                  <svg width={lineChartSize.width} height={lineChartSize.height} role="img" aria-label="Battery voltage and power line plot">
                    {[0.25,0.5,0.75,1].map((ratio)=>(
                      <line key={String(ratio)} x1="0" x2={lineChartSize.width} y1={lineChartSize.height*ratio} y2={lineChartSize.height*ratio} className="chart-grid-line" />
                    ))}
                    <polyline points={buildLinePath(batterySeries,"voltage")} className="line-voltage" fill="none" strokeWidth="3" />
                    <polyline points={buildLinePath(batterySeries,"power")} className="line-power" fill="none" strokeWidth="2" />
                  </svg>
                  <div className="chart-x-axis">{batterySeries.map((p)=><span key={p.time}>{p.time}</span>)}</div>
                </div>
              </div>

              <div className="turn-card">
                <div className="stats-card-header"><div><h4>Turn Distribution</h4><p>Number of left/right turns this shift</p></div></div>
                <div className="turn-count-row"><span>Left turns</span><strong>{turns.left}</strong></div>
                <div className="turn-bar"><div className="turn-bar-left" style={{ width: `${Math.round((turns.left / ((turns.left||0)+(turns.right||0)||1))*100)}%` }} /></div>
                <div className="turn-count-row"><span>Right turns</span><strong>{turns.right}</strong></div>
                <div className="turn-bar"><div className="turn-bar-right" style={{ width: `${Math.round((turns.right / ((turns.left||0)+(turns.right||0)||1))*100)}%` }} /></div>
                <div className="turn-footer">Left {Math.round((turns.left/((turns.left||0)+(turns.right||0)||1))*100)}% · Right {Math.round((turns.right/((turns.left||0)+(turns.right||0)||1))*100)}%</div>
              </div>
            </div>

            <div className="trend-card">
              <div className="stats-card-header"><div><h4>Mission Trend</h4><p>Completion vs incidents</p></div></div>
              <table><thead><tr><th>Day</th><th>Completed</th><th>Incidents</th></tr></thead><tbody>{missionTrend.map(r=> <tr key={r.label}><td>{r.label}</td><td>{r.completed}</td><td>{r.incidents}</td></tr>)}</tbody></table>
            </div>
          </div>
        )}

{/* CHAT */}
{rightPage === "chat" && (
  <div className="chat-pane">

    <div className="chat-header">
      <div>
        <h2>Assistant Link</h2>
        <p>
          Last sync at {latestMessage ? formatTimestamp(latestMessage.timestamp) : "--:--"} ·{" "}
          {chatMessages.filter((m) => m.sender === "human").length} operator prompts today
        </p>
      </div>
      <div className="chat-status-pill">Robot Online</div>
    </div>

    <div className="chat-subheader">
      <span>Channel: Operations Support</span>
      <span>Voice input {isRecording ? "listening…" : "idle"}</span>
    </div>

    <div className="chat-messages" ref={chatContainerRef}>
      {chatMessages.map((message) => {
        const isRobot = message.sender === "robot";
        return (
          <div key={message.id} className={`chat-row ${isRobot ? "robot" : "human"}`}>
            <div className={`chat-bubble ${isRobot ? "robot" : "human"}`}>
              <div className="chat-meta">
                <span>{isRobot ? "Robot" : "You"}</span>
                <span>{formatTimestamp(message.timestamp)}</span>
              </div>
              <p>{message.text}</p>
              {!isRobot && <span className="chat-status">{message.status || "Sent"}</span>}
            </div>
          </div>
        );
      })}

      {isTyping && (
        <div className="chat-row robot">
          <div className="chat-bubble robot typing">
            <span className="typing-dot" />
            <span className="typing-dot" />
            <span className="typing-dot" />
          </div>
        </div>
      )}
    </div>

    <div className="chat-quick-replies">
      {chatQuickPrompts.map((prompt) => (
        <button
          key={prompt}
          type="button"
          className="chat-chip"
          onClick={() => handleSuggestionClick(prompt)}
        >
          {prompt}
        </button>
      ))}
    </div>

    <div className="chat-input-row">
      <textarea
        value={chatInput}
        onChange={(e) => setChatInput(e.target.value)}
        onKeyPress={handleKeyPress}
        placeholder="Type a request or speak…"
        className="chat-textarea"
        rows={1}
      />

      {/* MIC BUTTON UPDATED */}
      <button
        onClick={handleMicClick}
        type="button"
        className={`chat-icon-btn ${isRecording ? "recording" : ""}`}
        title={isRecording ? "Listening…" : "Start voice input"}
      >
        <FaMicrophone />
      </button>

      <button
        onClick={() => handleSendMessage()}
        disabled={!chatInput.trim()}
        type="button"
        className="chat-send-btn"
        title="Send message"
      >
        <FaPaperPlane />
      </button>
    </div>

    <div className="chat-hint">Press Enter to send · Shift + Enter for newline</div>
  </div>
)}


        {/* fallback */}
        {![
          "maps","zones","waypoints","missions","users","analytics","diagnostics","logs","mission-logs","robot-bags","robot-settings","account","appearance","security","integrations","chat","stats"
        ].includes(rightPage) && <div>{rightPage}</div>}
      </div>
    </aside>
  );
};

export default RightPane;
