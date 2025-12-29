import React, { useState, useEffect, useCallback } from "react";
import { useNavigate } from "react-router-dom";
import { toast } from "react-hot-toast";
import { fetchWithAuth, clearAuthTokens, API_BASE } from "../../../utils/auth";

interface AccountProfile {
  fullName: string;
  email: string;
  team: string;
  shift: string;
}

interface AccountProps {
  accountProfile?: AccountProfile;
  setAccountProfile?: (profile: AccountProfile) => void;
}

const Account: React.FC<AccountProps> = ({
  accountProfile: externalProfile,
  setAccountProfile: externalSetProfile,
}) => {
  const navigate = useNavigate();
  const [profile, setProfile] = useState<AccountProfile>({
    fullName: "",
    email: "",
    team: "",
    shift: "",
  });
  const [saving, setSaving] = useState(false);
  const [message, setMessage] = useState<{ type: "error" | "success"; text: string } | null>(null);

  // Use external profile if provided, otherwise use local state
  const accountProfile = externalProfile || profile;
  const setAccountProfile = externalSetProfile || setProfile;

  const loadProfile = useCallback(async () => {
    try {
      const response = await fetchWithAuth(`${API_BASE}/api/user/profile`);
      const data = await response.json().catch(() => ({}));

      if (response.status === 401 || response.status === 403) {
        clearAuthTokens();
        navigate("/");
        return;
      }

      if (response.ok && data.success) {
        setAccountProfile({
          fullName: data.username || "",
          email: data.email || "",
          team: data.company || "",
          shift: data.amr_type || "",
        });
        setMessage(null);
      } else {
        setMessage({ type: "error", text: data.message || "Failed to load profile" });
      }
    } catch (error) {
      console.error("Failed to load profile", error);
      if (error instanceof Error && error.message === "No access token available") {
        clearAuthTokens();
        navigate("/");
        return;
      }
      setMessage({ type: "error", text: "Unable to load profile" });
    }
  }, [navigate, setAccountProfile]);

  useEffect(() => {
    loadProfile();
  }, [loadProfile]);

  const handleChange = (field: keyof AccountProfile, value: string) => {
    setAccountProfile({ ...accountProfile, [field]: value });
  };

  const handleSave = async () => {
    setSaving(true);
    setMessage(null);

    try {
      const response = await fetchWithAuth(`${API_BASE}/api/user/profile`, {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          email: accountProfile.email,
          company: accountProfile.team,
          amrType: accountProfile.shift,
        }),
      });

      const data = await response.json().catch(() => ({}));

      if (response.status === 401 || response.status === 403) {
        clearAuthTokens();
        navigate("/");
        return;
      }

      if (!response.ok || !data.success) {
        throw new Error(data.message || "Failed to update profile");
      }

      toast.success("Profile updated successfully");
      setMessage({ type: "success", text: "Profile saved successfully" });
      setTimeout(() => setMessage(null), 3000);
    } catch (error) {
      console.error("Failed to save profile", error);
      if (error instanceof Error && error.message === "No access token available") {
        clearAuthTokens();
        navigate("/");
        return;
      }
      const errorText = error instanceof Error ? error.message : "Unable to save profile";
      setMessage({ type: "error", text: errorText });
    } finally {
      setSaving(false);
    }
  };

  return (
    <div className="flex flex-col gap-4">
      {/* Full Name */}
      <label className="flex flex-col gap-2">
        <span className="text-sm font-semibold text-slate-700">Full Name</span>
        <input
          type="text"
          value={accountProfile.fullName}
          onChange={(e) => handleChange("fullName", e.target.value)}
          className="px-3 py-2 rounded-lg border border-slate-300 focus:outline-none focus:ring-2 focus:ring-blue-500"
          placeholder="Enter your full name"
        />
      </label>

      {/* Email */}
      <label className="flex flex-col gap-2">
        <span className="text-sm font-semibold text-slate-700">Email</span>
        <input
          type="email"
          value={accountProfile.email}
          onChange={(e) => handleChange("email", e.target.value)}
          className="px-3 py-2 rounded-lg border border-slate-300 focus:outline-none focus:ring-2 focus:ring-blue-500"
          placeholder="Enter your email"
        />
      </label>

      {/* Team */}
      <label className="flex flex-col gap-2">
        <span className="text-sm font-semibold text-slate-700">Team</span>
        <input
          type="text"
          value={accountProfile.team}
          onChange={(e) => handleChange("team", e.target.value)}
          className="px-3 py-2 rounded-lg border border-slate-300 focus:outline-none focus:ring-2 focus:ring-blue-500"
          placeholder="Enter your team"
        />
      </label>

      {/* Shift Window */}
      <label className="flex flex-col gap-2">
        <span className="text-sm font-semibold text-slate-700">Shift Window</span>
        <input
          type="text"
          value={accountProfile.shift}
          onChange={(e) => handleChange("shift", e.target.value)}
          className="px-3 py-2 rounded-lg border border-slate-300 focus:outline-none focus:ring-2 focus:ring-blue-500"
          placeholder="Enter your shift"
        />
      </label>

      {/* Message Display */}
      {message && (
        <div
          className={`px-4 py-3 rounded-lg text-sm font-medium ${
            message.type === "error"
              ? "bg-red-50 text-red-700 border border-red-200"
              : "bg-green-50 text-green-700 border border-green-200"
          }`}
        >
          {message.text}
        </div>
      )}

      {/* Save Button */}
      <button
        onClick={handleSave}
        disabled={saving}
        className="w-full px-4 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-blue-400 text-white font-semibold rounded-lg transition-colors"
      >
        {saving ? "Saving..." : "Save Profile"}
      </button>
    </div>
  );
};

export default Account;