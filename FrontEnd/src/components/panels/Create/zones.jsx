// logic

import React, { useState } from "react";
  

  const getFilteredZones = () => {
    if (!selectedMap) return [];
    const filtered = zones.filter((z) => z.mapId === selectedMap.id);
    console.log(`📍 getFilteredZones for map ${selectedMap.id}:`, filtered);
    return filtered;
  };

    const initialZones = [
    {
      id: "z1",
      mapId: "cfl_gf",
      name: "Assembly Lane",
      category: "Safe",
      active: true,
      geometry: "Polygon(32,18…)",
      createdAt: "2025-11-17",
    },
    {
      id: "z2",
      mapId: "cfl_gf",
      name: "Battery Bay",
      category: "No-Go",
      active: true,
      geometry: "Polygon(27,11…)",
      createdAt: "2025-11-15",
    },
    {
      id: "z3",
      mapId: "cfl_gf",
      name: "Dock Tunnel",
      category: "Caution",
      active: false,
      geometry: "Line(82,90…)",
      createdAt: "2025-11-14",
    },
  ];
  const [zones, setZones] = useState(initialZones);
  const [zoneFormOpen, setZoneFormOpen] = useState(false);
  const [zoneForm, setZoneForm] = useState({
    name: "",
    category: "Safe",
    geometry: "",
    active: true,
  });

//   RightPane logic
  const {
    rightPage,
    setRightPage,
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
    } = props;

    // UI
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
