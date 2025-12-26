// logic

import React, { useState } from "react";
import { toast } from "react-hot-toast";



 // users data + selection (fixes eslint no-undef)
  const [users, setUsers] = useState([
    {
      id: 1,
      username: "john_doe",
      email: "john.doe@example.com",
      company: "ANSCER Robotics",
      amr_type: "Type A",
      role: "user",
      approval: "Approved",
    },
    {
      id: 2,
      username: "jane_smith",
      email: "jane.smith@example.com",
      company: "CNDE IITM",
      amr_type: "Type B",
      role: "user",
      approval: "Pending",
    },
    {
      id: 3,
      username: "bob_wilson",
      email: "bob.wilson@example.com",
      company: "TechCorp",
      amr_type: "Type A",
      role: "user",
      approval: "Approved",
    },
    {
      id: 4,
      username: "alice_johnson",
      email: "alice.johnson@example.com",
      company: "Innovation Labs",
      amr_type: "Type C",
      role: "user",
      approval: "Rejected",
    },
    {
      id: 5,
      username: "charlie_brown",
      email: "charlie.brown@example.com",
      company: "RoboTech Inc",
      amr_type: "Type A",
      role: "user",
      approval: "Approved",
    },
    {
      id: 6,
      username: "diana_prince",
      email: "diana.prince@example.com",
      company: "Wonder Systems",
      amr_type: "Type B",
      role: "user",
      approval: "Approved",
    },
    {
      id: 7,
      username: "edward_norton",
      email: "edward.norton@example.com",
      company: "Norton Industries",
      amr_type: "Type C",
      role: "user",
      approval: "Pending",
    },
    {
      id: 8,
      username: "admin_user",
      email: "admin@example.com",
      company: "ANSCER Admin",
      amr_type: "Admin Console",
      role: "admin",
      approval: "Approved",
    },
  ]);
  const [selectedUserId, setSelectedUserId] = useState(null);
  const [usersLoading, setUsersLoading] = useState(false);
  const [userActionLoading, setUserActionLoading] = useState(false);
  const [usersError, setUsersError] = useState("");
  const LOCK_TOAST_ID = "lock-toast"; 

//   RightPane logic


const RightPane = (props) => {
  const [userSearchField, setUserSearchField] = useState("any");
  const [userSearchTerm, setUserSearchTerm] = useState("");
    };

const RightPane = (props) => {
  const [editingUserId, setEditingUserId] = useState(null);
  const [editUserForm, setEditUserForm] = useState({
    username: "",
    email: "",
    company: "",
    amr_type: "",
    role: "",
    approval: "",
  });

    const {
    rightPage,
    setRightPage,
    // users
    users,
    usersLoading,
    usersError,
    loadUsers,
    selectedUserId,
    setSelectedUserId,
    userActionLoading,
    handleResetUserPassword,
    } = props;

    // UI
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