// logic

import React from "react";
import React, { useState } from "react";

  // missions data + selection (added)
  const initialMissions = [
    {
      id: "m1",
      mapId: "cfl_gf",
      name: "Inspect Zone A",
      owner: "CNDE",
      status: "Draft",
      createdAt: "2025-11-17",
      notes: "Routine inspection",
    },
    {
      id: "m2",
      mapId: "cfl_gf",
      name: "Delivery Route 3",
      owner: "ANSCER ADMIN",
      status: "Scheduled",
      createdAt: "2025-11-16",
      notes: "Delivery to docks",
    },
    {
      id: "m3",
      mapId: "cfl_gf",
      name: "Battery Check",
      owner: "CNDE",
      status: "Completed",
      createdAt: "2025-11-15",
      notes: "Post-run check",
    },
  ];
  const [missions, setMissions] = useState(initialMissions);
  const [selectedMissionId, setSelectedMissionId] = useState(null);
  const handleSelectMission = (id) => setSelectedMissionId(id);
  const [missionFormOpen, setMissionFormOpen] = useState(false);
  const [missionForm, setMissionForm] = useState({
    name: "",
    owner: "",
    status: "Draft",
    notes: "",
  });
  const getFilteredMissions = () => {
    if (!selectedMap) return [];
    const filtered = missions.filter((m) => m.mapId === selectedMap.id);
    console.log(`📍 getFilteredMissions for map ${selectedMap.id}:`, filtered);
    return filtered;
  };

//   RightPane logic
 const RightPane = (props) => {
   const [missionSearchField, setMissionSearchField] = useState("any");
   const [missionSearchTerm, setMissionSearchTerm] = useState("");
 };

  const {
    rightPage,
    setRightPage,
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
    } = props;

// UI

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