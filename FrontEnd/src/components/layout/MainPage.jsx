import React, { useRef, useEffect, useState, useCallback } from "react";
import { useNavigate } from "react-router-dom";
import Sidebar from "../SideBar";
import JoyStick from "../JoyStick";
import {
  FaPlay,
  FaTrash,
  FaMicrophone,
  FaPaperPlane,
  FaPlus,
  FaEdit,
} from "react-icons/fa";
import { toast } from "react-hot-toast";
import { fetchWithAuth, clearAuthTokens, API_BASE } from "../../utils/auth";
import "./MainPage.css";
import RightPane from "../RightPane";

export const API_V1_BASE = `${API_BASE}/api/v1`;

const MainPage = () => {
  const navigate = useNavigate();
  
  const mapImageInputRef = useRef(null); // hidden file input used by map modal

  const [rightPage] = useState(null);

// new: zoom state for map
  const [zoomLevel, setZoomLevel] = useState(0); // 0 .. 0.3
  const toggleMapZoom = useCallback(() => {
    setZoomLevel((z) => (z === 0 ? 0.3 : 0));
  }, []);
  const zoomIn = useCallback(
    (e) => {
      e && e.stopPropagation();
      setZoomLevel((z) => Math.min(0.3, +(z + 0.1).toFixed(2)));
    },
    [setZoomLevel],
  );
  const zoomOut = useCallback(
    (e) => {
      e && e.stopPropagation();
      setZoomLevel((z) => Math.max(0, +(z - 0.1).toFixed(2)));
    },
    [setZoomLevel],
  );
  // minimize main center (map) and joystick
  const [minimizedMain, setMinimizedMain] = useState(false);

  // joystick state (last vector from physical joystick or keyboard WASD)
  const [joystickState, setJoystickState] = useState({
    x: 0,
    y: 0,
    force: 0,
    angle: 0,
    type: null,
  });

  // handler passed to JoyStick so component receives events (stick or keyboard)
  const handleJoystickMove = useCallback((data) => {
    // data shape: { x, y, force, angle, ... }
    setJoystickState({
      x: data.x ?? 0,
      y: data.y ?? 0,
      force: data.force ?? 0,
      angle: data.angle ?? 0,
      type: data.type || "joystick",
    });
    // TODO: translate to robot commands / send to backend if needed
    // console.debug("Joystick move:", data);
  }, []);
  
  const [accountProfile, setAccountProfile] = useState({
    fullName: "",
    email: "",
    team: "",
    shift: "",
  });
  const [profileSaving, setProfileSaving] = useState(false);
  const [profileError, setProfileError] = useState("");
  const [profileSuccess, setProfileSuccess] = useState("");

  // apply a simple theme state for whole app (light|dark|system)
  const [selectedTheme, setSelectedTheme] = useState("light");

  useEffect(() => {
    try {
      document.documentElement.setAttribute("data-theme", selectedTheme);
    } catch {}
  }, [selectedTheme]);

  const [securityPreferences, setSecurityPreferences] = useState({
    twoFactor: true,
    autoLock: true,
    anomalyAlerts: true,
  });

  const [securityEvents, setSecurityEvents] = useState([
    {
      id: "sec1",
      ts: "09:44",
      actor: "ops-admin",
      action: "API token created",
      context: "Main console",
    },
    {
      id: "sec2",
      ts: "08:12",
      actor: "robot-01",
      action: "Cert renewed",
      context: "Device",
    },
  ]);

  const [integrationItems, setIntegrationItems] = useState([
    {
      id: "rest",
      name: "REST API",
      status: "Connected",
      description: "Push missions from MES",
    },
    {
      id: "slack",
      name: "Slack Bot",
      status: "Disconnected",
      description: "Alerts to #robot-ops",
    },
    {
      id: "grafana",
      name: "Grafana",
      status: "Connected",
      description: "Telemetry dashboards",
    },
  ]);

  const [statsError, setStatsError] = useState("");
  const [statsLoading, setStatsLoading] = useState(false);

  const handleSelectWaypoint = (wpId) => {
    setSelectedWaypointId(wpId);
    const wpMap = mapsList.find((m) => m.id === "waypoints");
    if (wpMap) setSelectedMap(wpMap);
  };

 


  // handler for testing: force error on next request
  const [forceError, setForceError] = useState(false);
  const handleTestError = () => setForceError((e) => !e);



  const requestV1 = useCallback(
    async (path, options = {}) => {
      const headers = {
        Accept: "application/json",
        ...(options.headers || {}),
      };
      if (options.body && !headers["Content-Type"]) {
        headers["Content-Type"] = "application/json";
      }

      try {
        const response = await fetchWithAuth(`${API_V1_BASE}${path}`, {
          ...options,
          headers,
        });
        const data = await response.json().catch(() => ({}));
        if (response.status === 401 || response.status === 403) {
          clearAuthTokens();
          navigate("/");
          throw new Error(data.message || "Unauthorized");
        }
        if (!response.ok || data.success === false) {
          throw new Error(data.message || "Request failed");
        }
        return data;
      } catch (error) {
        if (error.message === "No access token available") {
          clearAuthTokens();
          navigate("/");
        }
        throw error;
      }
    },
    [navigate],
  );

  const loadUsers = useCallback(async () => {
    setUsersLoading(true);
    setUsersError("");
    try {
      const response = await requestV1("/users");
      setUsers(response.items || []);
    } catch (error) {
      console.error("Users fetch error", error);
      setUsersError(error.message || "Failed to load users");
      toast.error(error.message || "Failed to load users");
    } finally {
      setUsersLoading(false);
    }
  }, [requestV1]);

  useEffect(() => {
    loadUsers();
  }, [loadUsers]);

  const handleResetUserPassword = useCallback(async () => {
    if (!selectedUserId) {
      toast.error("Select a user first");
      return;
    }
    setUserActionLoading(true);
    try {
      const response = await requestV1(
        `/users/${selectedUserId}/reset-password`,
        { method: "POST" },
      );
      if (response.item) {
        setUsers((prev) =>
          prev.map((user) =>
            user.id === response.item.id ? { ...user, ...response.item } : user,
          ),
        );
      }
      toast.success(response.message || "Password reset token generated");
    } catch (error) {
      console.error("Reset password error", error);
      toast.error(error.message || "Failed to reset password");
    } finally {
      setUserActionLoading(false);
    }
  }, [requestV1, selectedUserId]);


  const persistRobotSetting = useCallback(
    (key, value, previousValue) => {
      requestV1("/settings/robot", {
        method: "PATCH",
        body: JSON.stringify({ [key]: value }),
      }).catch((error) => {
        console.error("Robot setting update failed", error);
        toast.error(error.message || "Failed to update setting");
        setRobotSettingsState((prev) => ({ ...prev, [key]: previousValue }));
      });
    },
    [requestV1],
  );

  const toggleRobotSetting = (key) => {
    setRobotSettingsState((prev) => {
      const nextValue = !prev[key];
      persistRobotSetting(key, nextValue, prev[key]);
      return {
        ...prev,
        [key]: nextValue,
      };
    });
  };

  const handleAccountChange = (field, value) => {
    setAccountProfile((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  const loadAccountProfile = useCallback(async () => {
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
        setProfileError("");
      } else {
        setProfileError(data.message || "Failed to load profile.");
      }
    } catch (error) {
      console.error("Failed to load profile", error);
      if (error.message === "No access token available") {
        clearAuthTokens();
        navigate("/");
        return;
      }
      setProfileError("Unable to load profile.");
    }
  }, [navigate]);

  useEffect(() => {
    loadAccountProfile();
  }, [loadAccountProfile]);

  const handleSaveProfile = async () => {
    setProfileSaving(true);
    setProfileError("");
    setProfileSuccess("");

    try {
      const response = await fetchWithAuth(`${API_BASE}/api/user/profile`, {
        method: "PUT",
        headers: {
          "Content-Type": "application/json",
        },
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
        throw new Error(data.message || "Failed to update profile.");
      }

      toast.success("Profile updated");
      setProfileSuccess("Profile saved successfully.");
    } catch (error) {
      console.error("Failed to save profile", error);
      if (error.message === "No access token available") {
        clearAuthTokens();
        navigate("/");
        return;
      }
      setProfileError(error.message || "Unable to save profile.");
    } finally {
      setProfileSaving(false);
      setTimeout(() => setProfileSuccess(""), 3000);
    }
  };

  const persistSecurityPref = useCallback(
    (key, value, previousValue) => {
      requestV1("/settings/security", {
        method: "PATCH",
        body: JSON.stringify({ [key]: value }),
      }).catch((error) => {
        console.error("Security pref update failed", error);
        toast.error(error.message || "Failed to update security preference");
        setSecurityPreferences((prev) => ({ ...prev, [key]: previousValue }));
      });
    },
    [requestV1],
  );

  const toggleSecurityPref = (key) => {
    setSecurityPreferences((prev) => {
      const nextValue = !prev[key];
      persistSecurityPref(key, nextValue, prev[key]);
      return {
        ...prev,
        [key]: nextValue,
      };
    });
  };

  const persistIntegrationStatus = useCallback(
    (id, status, previousStatus) => {
      requestV1(`/settings/integrations/${id}`, {
        method: "PATCH",
        body: JSON.stringify({ status }),
      }).catch((error) => {
        console.error("Integration status update failed", error);
        toast.error(error.message || "Failed to update integration");
        setIntegrationItems((items) =>
          items.map((item) =>
            item.id === id ? { ...item, status: previousStatus } : item,
          ),
        );
      });
    },
    [requestV1],
  );

  const toggleIntegrationStatus = (id) => {
    setIntegrationItems((items) =>
      items.map((item) => {
        if (item.id !== id) {
          return item;
        }
        const nextStatus =
          item.status === "Connected" ? "Disconnected" : "Connected";
        persistIntegrationStatus(id, nextStatus, item.status);
        return {
          ...item,
          status: nextStatus,
        };
      }),
    );
  };

  

  // code separated from active,inactive maps 
 const loadCreateData = async () => {
    try {
      const [
        mapsRes,
        zonesRes,
        waypointsRes,
        missionsRes,
        usersRes,
      ] = await Promise.all([
        requestV1("/maps"),
        requestV1("/zones"),
        requestV1("/waypoints"),
        requestV1("/missions"),
        requestV1("/users"),
      ]);

      if (cancelled) return;

      processMaps(mapsRes);
      processZones(zonesRes);
      processWaypoints(waypointsRes);
      processMissions(missionsRes);
      processUsers(usersRes);

    } catch (error) {
      console.error("Failed to load workspace data", error);
    }
  };

  loadCreateData();

  return () => {
    cancelled = true;
  };
}, [requestV1]);


  loadCreateData();
  


  // breadcrumb parts for header (Home → Page → Item)
  const latestMessage = chatMessages[chatMessages.length - 1];
  const breadcrumbParts = ["Home"];
  if (rightPage)
    breadcrumbParts.push(
      rightPage.charAt(0).toUpperCase() + rightPage.slice(1),
    );
  if (rightPage === "maps" && selectedMap)
    breadcrumbParts.push(selectedMap.name);

 
  // Runtime diagnostics: show tiny startup toast and report unhandled errors
  useEffect(() => {
    // small startup hint so we know React mounted
    toast.dismiss("app-start");
    toast.success("App initialized", { id: "app-start", duration: 1500 });

    const onUnhandledRejection = (ev) => {
      console.error("Unhandled promise rejection:", ev.reason);
      try {
        toast.error(`Unhandled rejection: ${String(ev.reason).slice(0, 80)}`, { duration: 4000 });
      } catch {}
    };
    const onError = (message, source, lineno, colno, error) => {
      console.error("Window error:", message, { source, lineno, colno, error });
      try {
        toast.error(`Error: ${String(message).slice(0, 80)}`, { duration: 4000 });
      } catch {}
      // return false to allow default browser handling
      return false;
    };

    window.addEventListener("unhandledrejection", onUnhandledRejection);
    window.addEventListener("error", onError);
    return () => {
      window.removeEventListener("unhandledrejection", onUnhandledRejection);
      window.removeEventListener("error", onError);
    };
  }, []);

  return (
    <main className={`map-area ${minimizedMain ? "minimized" : ""}`}>
          <div>
            {/* inner content that will zoom — click handler here so only map-content scales */}
            <div
              className={`map-content`}
              onClick={toggleMapZoom}
              role="button"
              tabIndex={0}
              onKeyDown={(e) => {
                if (e.key === "Enter" || e.key === " ") toggleMapZoom();
              }}
              style={{
                width: "100%",
                height: "100%",
                display: "flex",
                alignItems: "stretch",
                justifyContent: "stretch",
                transform: `scale(${1 + zoomLevel})`,
                transition: "transform 280ms ease",
                cursor: zoomLevel > 0 ? "zoom-out" : "zoom-in",
              }}
            >
              {/* render selected map image if available, otherwise empty white area */}
              {selectedMap?.image ? (
                <img
                  src={selectedMap.image}
                  alt={selectedMap.name || "Map preview"}
                  style={{ width: "100%", height: "100%", objectFit: "contain", display: "block" }}
                />
              ) : (

                <div style={{ width: "100%", height: "100%" }} aria-hidden="true" />
              )}
            </div>
            
              {/* zoom-in and zoom-out buttons */}
            <div className="map-overlays">
              <div className="right-controls" onClick={(e) => e.stopPropagation()}>
                <button
                  className="control-btn"
                  title="Zoom In"
                  onClick={(e) => { e.stopPropagation(); zoomIn(e); }}
                  aria-label="Zoom in"
                  type="button"
                >
                  ＋
                </button>
                <button
                  className="control-btn"
                  title="Zoom Out"
                  onClick={(e) => { e.stopPropagation(); zoomOut(e); }}
                  aria-label="Zoom out"
                  type="button"
                >
                  −
                </button>
              </div>
              <div className={`joystick-overlay ${minimizedMain ? "minimized" : ""}`}>
                <JoyStick width={140} height={140} onMove={handleJoystickMove} />
              </div>
            </div>
          </div>
        </main>

       );
};

export default MainPage;
