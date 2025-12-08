import React, { useState, useEffect, useCallback } from "react";
import { useNavigate } from "react-router-dom";
import "./Admin.css";
import { fetchWithAuth, clearAuthTokens, API_BASE } from "../utils/auth";

const API_URL = API_BASE || "http://127.0.0.1:5000";

const Admin = () => {
  const [users, setUsers] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState("");
  const [statusMessage, setStatusMessage] = useState("");
  const [processingUserId, setProcessingUserId] = useState(null);
  const [openDropdown, setOpenDropdown] = useState(null);
  const navigate = useNavigate();

  const fetchUsers = useCallback(async () => {
    try {
      const response = await fetchWithAuth(`${API_URL}/api/admin/users`, {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
        },
      });

      if (response.status === 401 || response.status === 403) {
        clearAuthTokens();
        navigate("/admin");
        return;
      }

      if (response.ok) {
        const data = await response.json();
        setUsers(data);
        setError("");
      } else {
        const data = await response.json().catch(() => ({}));
        setError(data.message || "Failed to fetch users");
      }
    } catch (err) {
      console.error(err);
      if (err.message === "No access token available") {
        clearAuthTokens();
        navigate("/admin");
        return;
      }
      setError("Network error. Make sure Flask server is running.");
    } finally {
      setLoading(false);
    }
  }, [navigate]);

  const handleUserAction = async (userId, action) => {
    const endpoints = {
      approve: `${API_URL}/api/admin/users/${userId}/approve`,
      reject: `${API_URL}/api/admin/users/${userId}/reject`,
      pending: `${API_URL}/api/admin/users/${userId}/pending`,
    };

    const messages = {
      approve: "User approved successfully!",
      reject: "User rejected successfully!",
      pending: "User status set to pending successfully!",
    };

    setProcessingUserId(userId);
    setStatusMessage("");
    setError("");

    try {
      const response = await fetchWithAuth(endpoints[action], {
        method: "PUT",
        headers: {
          "Content-Type": "application/json",
        },
      });

      const data = await response.json().catch(() => ({}));

      if (!response.ok) {
        throw new Error(data.message || `Failed to ${action} user.`);
      }

      setStatusMessage(data.message || messages[action]);

      // Auto-dismiss success message after 3 seconds
      setTimeout(() => {
        setStatusMessage("");
      }, 3000);

      const approvalMap = {
        approve: "Approved",
        reject: "Rejected",
        pending: "Pending",
      };
      setUsers((prev) =>
        prev.map((user) =>
          user.id === userId
            ? { ...user, approval: approvalMap[action] || user.approval }
            : user,
        ),
      );
    } catch (err) {
      console.error(err);
      if (err.message === "No access token available") {
        clearAuthTokens();
        navigate("/admin");
        return;
      }
      setError(err.message);
      // Auto-dismiss error message after 5 seconds
      setTimeout(() => {
        setError("");
      }, 5000);
    } finally {
      setProcessingUserId(null);
    }
  };

  const handleLogout = () => {
    clearAuthTokens();
    navigate("/admin");
  };

  const getApprovalStatusClass = (approval) => {
    switch (approval) {
      case "Approved":
        return "status-approved";
      case "Rejected":
        return "status-rejected";
      case "Pending":
        return "status-pending";
      default:
        return "";
    }
  };

  const handleActionClick = (userId, action) => {
    setOpenDropdown(null);
    handleUserAction(userId, action);
  };

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (openDropdown && !event.target.closest(".dropdown-container")) {
        setOpenDropdown(null);
      }
    };

    if (openDropdown) {
      document.addEventListener("mousedown", handleClickOutside);
      return () =>
        document.removeEventListener("mousedown", handleClickOutside);
    }
  }, [openDropdown]);

  useEffect(() => {
    fetchUsers();
  }, [fetchUsers]);

  if (loading) {
    return (
      <div className="admin-container">
        <div style={{ textAlign: "center", padding: "2rem" }}>Loading...</div>
      </div>
    );
  }

  return (
    <div className="admin-container">
      <div className="admin-header">
        <h1>Admin Dashboard</h1>
        <button className="btn-logout" onClick={handleLogout}>
          Logout
        </button>
      </div>

      {error && (
        <div className="admin-message admin-error">
          <span>{error}</span>
          <button className="close-btn" onClick={() => setError("")}>
            ×
          </button>
        </div>
      )}
      {statusMessage && (
        <div className="admin-message admin-status">
          <span>{statusMessage}</span>
          <button className="close-btn" onClick={() => setStatusMessage("")}>
            ×
          </button>
        </div>
      )}

      <div className="table-container">
        <div className="table-wrapper">
          <table className="admin-table">
            <thead>
              <tr>
                <th>Username</th>
                <th>Email</th>
                <th>Company</th>
                <th>AMR Type</th>
                <th>Role</th>
                <th>Approval</th>
                <th>Actions</th>
              </tr>
            </thead>
            <tbody>
              {users.length === 0 ? (
                <tr>
                  <td colSpan="7" style={{ textAlign: "center" }}>
                    No users found
                  </td>
                </tr>
              ) : (
                users.map((user) => (
                  <tr key={user.id}>
                    <td>{user.username}</td>
                    <td>{user.email}</td>
                    <td>{user.company || "N/A"}</td>
                    <td>{user.amr_type || "N/A"}</td>
                    <td>{user.role}</td>
                    <td>
                      <span
                        className={`approval-status ${getApprovalStatusClass(user.approval)}`}
                      >
                        {user.approval}
                      </span>
                    </td>
                    <td className="admin-actions">
                      <div className="dropdown-container">
                        <button
                          className={`btn-dropdown-toggle ${openDropdown === user.id ? "active" : ""}`}
                          onClick={() =>
                            setOpenDropdown(
                              openDropdown === user.id ? null : user.id,
                            )
                          }
                          disabled={processingUserId === user.id}
                        >
                          Actions
                          <span className="dropdown-arrow">▼</span>
                        </button>
                        {openDropdown === user.id && (
                          <div className="dropdown-menu">
                            <button
                              className="dropdown-item approve"
                              onClick={() =>
                                handleActionClick(user.id, "approve")
                              }
                              disabled={user.approval === "Approved"}
                              title={
                                user.approval === "Approved"
                                  ? "User already approved"
                                  : "Approve user"
                              }
                            >
                              Approve
                            </button>
                            <button
                              className="dropdown-item reject"
                              onClick={() =>
                                handleActionClick(user.id, "reject")
                              }
                              disabled={user.approval === "Rejected"}
                              title={
                                user.approval === "Rejected"
                                  ? "User already rejected"
                                  : "Reject user"
                              }
                            >
                              Reject
                            </button>
                            <button
                              className="dropdown-item pending"
                              onClick={() =>
                                handleActionClick(user.id, "pending")
                              }
                              disabled={user.approval === "Pending"}
                              title={
                                user.approval === "Pending"
                                  ? "User already pending"
                                  : "Set user to pending"
                              }
                            >
                              Pending
                            </button>
                          </div>
                        )}
                      </div>
                    </td>
                  </tr>
                ))
              )}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );
};

export default Admin;
