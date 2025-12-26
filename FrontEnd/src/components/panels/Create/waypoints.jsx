// Logic

import React from "react";
import React, { useState } from "react";

  // waypoints data + selection
  const initialWaypoints = [
    {
      id: "wp1",
      mapId: "cfl_gf",
      name: "WP_A",
      category: "Nav",
      active: true,
      geom: "Point(12 34)",
      createdAt: "2025-11-17",
      notes: "First waypoint",
    },
    {
      id: "wp2",
      mapId: "cfl_gf",
      name: "WP_B",
      category: "Inspect",
      active: false,
      geom: "Point(98 76)",
      createdAt: "2025-11-17",
      notes: "Inspection point",
    },
    {
      id: "wp3",
      mapId: "cfl_gf",
      name: "WP_C",
      category: "Charge",
      active: true,
      geom: "Point(44 55)",
      createdAt: "2025-11-16",
      notes: "Charging pad",
    },
  ];
  const [waypoints, setWaypoints] = useState(initialWaypoints);
  const [waypointFormOpen, setWaypointFormOpen] = useState(false);
  const [waypointForm, setWaypointForm] = useState({
    name: "",
    category: "Nav",
    geom: "",
    notes: "",
    active: true,
  });

  const [selectedWaypointId, setSelectedWaypointId] = useState(null);

  // Add filtered getters for waypoints, zones, missions based on active map
  const getFilteredWaypoints = () => {
    if (!selectedMap) return [];
    const filtered = waypoints.filter((wp) => wp.mapId === selectedMap.id);
    console.log(`📍 getFilteredWaypoints for map ${selectedMap.id}:`, filtered);
    return filtered;
  };


//   RightPane logic
const RightPane = (props) => {
  const [waypointSearchField, setWaypointSearchField] = useState("any");
  const [waypointSearchTerm, setWaypointSearchTerm] = useState("");
 }; 

  const {
    rightPage,
    setRightPage,
    // waypoints
    waypoints,
    waypointFormOpen,
    setWaypointFormOpen,
    waypointForm,
    setWaypointForm,
    setWaypoints,
    handleSelectWaypoint,
    setSelectedWaypointId,
  } = props;

  // UI
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