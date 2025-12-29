// Note: Rename this file to Admin.tsx after applying changes.

import React, { useState, useEffect, useCallback } from "react";
import { useNavigate } from "react-router-dom";
import { fetchWithAuth, clearAuthTokens, API_BASE } from "../../utils/auth";

const API_URL = API_BASE || "http://127.0.0.1:5000";

type ApprovalStatus = "Approved" | "Rejected" | "Pending" | string;

interface User {
  id: number;
  username: string;
  email: string;
  company?: string | null;
  amr_type?: string | null;
  role: string;
  approval: ApprovalStatus;
}

const Admin: React.FC = () => {
  const [users, setUsers] = useState<User[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string>("");
  const [statusMessage, setStatusMessage] = useState<string>("");
  const [processingUserId, setProcessingUserId] = useState<number | null>(null);
  const [openDropdown, setOpenDropdown] = useState<number | null>(null);
  const navigate = useNavigate();

  const fetchUsers = useCallback(async () => {
    try {
      const response = await fetchWithAuth(`${API_URL}/api/admin/users`, {
        method: "GET",
        headers: { "Content-Type": "application/json" },
      } as RequestInit);

      if (response.status === 401 || response.status === 403) {
        clearAuthTokens();
        navigate("/admin");
        return;
      }

      if (response.ok) {
        const data = (await response.json()) as User[];
        setUsers(data);
        setError("");
      } else {
        const data = await response.json().catch(() => ({}));
        setError((data && (data as any).message) || "Failed to fetch users");
      }
    } catch (err: any) {
      console.error(err);
      if (err?.message === "No access token available") {
        clearAuthTokens();
        navigate("/admin");
        return;
      }
      setError("Network error. Make sure Flask server is running.");
    } finally {
      setLoading(false);
    }
  }, [navigate]);

  const handleUserAction = async (userId: number, action: "approve" | "reject" | "pending") => {
    const endpoints: Record<string, string> = {
      approve: `${API_URL}/api/admin/users/${userId}/approve`,
      reject: `${API_URL}/api/admin/users/${userId}/reject`,
      pending: `${API_URL}/api/admin/users/${userId}/pending`,
    };

    const messages: Record<string, string> = {
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
        headers: { "Content-Type": "application/json" },
      } as RequestInit);

      const data = await response.json().catch(() => ({}));

      if (!response.ok) {
        throw new Error((data && (data as any).message) || `Failed to ${action} user.`);
      }

      setStatusMessage((data && (data as any).message) || messages[action]);

      // Auto-dismiss success message after 3 seconds
      setTimeout(() => setStatusMessage(""), 3000);

      const approvalMap: Record<string, ApprovalStatus> = {
        approve: "Approved",
        reject: "Rejected",
        pending: "Pending",
      };

      setUsers((prev) =>
        prev.map((user) => (user.id === userId ? { ...user, approval: approvalMap[action] } : user)),
      );
    } catch (err: any) {
      console.error(err);
      if (err?.message === "No access token available") {
        clearAuthTokens();
        navigate("/admin");
        return;
      }
      setError(err?.message || "An error occurred");
      // Auto-dismiss error message after 5 seconds
      setTimeout(() => setError(""), 5000);
    } finally {
      setProcessingUserId(null);
    }
  };

  const handleLogout = () => {
    clearAuthTokens();
    navigate("/admin");
  };

  const getApprovalStatusClass = (approval: ApprovalStatus) => {
    switch (approval) {
      case "Approved":
        return "text-green-700 bg-green-100";
      case "Rejected":
        return "text-red-700 bg-red-100";
      case "Pending":
        return "text-yellow-700 bg-yellow-100";
      default:
        return "text-gray-700 bg-gray-100";
    }
  };

  const handleActionClick = (userId: number, action: "approve" | "reject" | "pending") => {
    setOpenDropdown(null);
    handleUserAction(userId, action);
  };

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (openDropdown && !(event.target as HTMLElement).closest(".dropdown-container")) {
        setOpenDropdown(null);
      }
    };

    if (openDropdown) {
      document.addEventListener("mousedown", handleClickOutside);
      return () => document.removeEventListener("mousedown", handleClickOutside);
    }
  }, [openDropdown]);

  useEffect(() => {
    fetchUsers();
  }, [fetchUsers]);

  if (loading) {
    return (
      <div className="max-w-6xl mx-auto p-6">
        <div className="text-center py-12 text-gray-600">Loading...</div>
      </div>
    );
  }

  return (
    <div className="max-w-6xl mx-auto p-6">
      <div className="flex items-center justify-between mb-6">
        <h1 className="text-2xl font-semibold text-gray-800">Admin Dashboard</h1>
        <button
          onClick={handleLogout}
          className="px-4 py-2 bg-red-600 text-white rounded hover:bg-red-700 transition"
        >
          Logout
        </button>
      </div>

      {error && (
        <div className="mb-4 p-3 bg-red-50 border border-red-200 text-red-700 rounded flex justify-between items-center">
          <span>{error}</span>
          <button className="ml-4 text-red-700" onClick={() => setError("")}>
            ×
          </button>
        </div>
      )}

      {statusMessage && (
        <div className="mb-4 p-3 bg-green-50 border border-green-200 text-green-700 rounded flex justify-between items-center">
          <span>{statusMessage}</span>
          <button className="ml-4 text-green-700" onClick={() => setStatusMessage("")}>
            ×
          </button>
        </div>
      )}

      <div className="overflow-x-auto bg-white rounded shadow">
        <table className="min-w-full divide-y divide-gray-200">
          <thead className="bg-gray-50">
            <tr>
              <th className="px-4 py-3 text-left text-sm font-medium text-gray-600">Username</th>
              <th className="px-4 py-3 text-left text-sm font-medium text-gray-600">Email</th>
              <th className="px-4 py-3 text-left text-sm font-medium text-gray-600">Company</th>
              <th className="px-4 py-3 text-left text-sm font-medium text-gray-600">AMR Type</th>
              <th className="px-4 py-3 text-left text-sm font-medium text-gray-600">Role</th>
              <th className="px-4 py-3 text-left text-sm font-medium text-gray-600">Approval</th>
              <th className="px-4 py-3 text-left text-sm font-medium text-gray-600">Actions</th>
            </tr>
          </thead>
          <tbody className="bg-white divide-y divide-gray-100">
            {users.length === 0 ? (
              <tr>
                <td colSpan={7} className="px-4 py-6 text-center text-gray-500">
                  No users found
                </td>
              </tr>
            ) : (
              users.map((user) => (
                <tr key={user.id}>
                  <td className="px-4 py-3 text-sm text-gray-700">{user.username}</td>
                  <td className="px-4 py-3 text-sm text-gray-700">{user.email}</td>
                  <td className="px-4 py-3 text-sm text-gray-700">{user.company || "N/A"}</td>
                  <td className="px-4 py-3 text-sm text-gray-700">{user.amr_type || "N/A"}</td>
                  <td className="px-4 py-3 text-sm text-gray-700">{user.role}</td>
                  <td className="px-4 py-3">
                    <span
                      className={`inline-block px-2 py-1 text-xs font-medium rounded ${getApprovalStatusClass(
                        user.approval,
                      )}`}
                    >
                      {user.approval}
                    </span>
                  </td>
                  <td className="px-4 py-3">
                    <div className="relative inline-block dropdown-container">
                      <button
                        onClick={() => setOpenDropdown(openDropdown === user.id ? null : user.id)}
                        disabled={processingUserId === user.id}
                        className={`px-3 py-1 border rounded text-sm flex items-center space-x-2 ${
                          openDropdown === user.id ? "bg-gray-100" : "bg-white"
                        }`}
                        aria-haspopup="true"
                        aria-expanded={openDropdown === user.id}
                      >
                        <span>Actions</span>
                        <span className="text-xs">▼</span>
                      </button>

                      {openDropdown === user.id && (
                        <div className="absolute right-0 mt-2 w-40 bg-white border rounded shadow z-10">
                          <button
                            className="w-full text-left px-3 py-2 text-sm hover:bg-green-50 disabled:opacity-50"
                            onClick={() => handleActionClick(user.id, "approve")}
                            disabled={user.approval === "Approved"}
                            title={user.approval === "Approved" ? "User already approved" : "Approve user"}
                          >
                            Approve
                          </button>
                          <button
                            className="w-full text-left px-3 py-2 text-sm hover:bg-red-50 disabled:opacity-50"
                            onClick={() => handleActionClick(user.id, "reject")}
                            disabled={user.approval === "Rejected"}
                            title={user.approval === "Rejected" ? "User already rejected" : "Reject user"}
                          >
                            Reject
                          </button>
                          <button
                            className="w-full text-left px-3 py-2 text-sm hover:bg-yellow-50 disabled:opacity-50"
                            onClick={() => handleActionClick(user.id, "pending")}
                            disabled={user.approval === "Pending"}
                            title={user.approval === "Pending" ? "User already pending" : "Set user to pending"}
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
  );
};

export default Admin;
