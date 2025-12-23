import React, { useState, useRef, useEffect } from "react";
import nipplejs from "nipplejs";
import "./SideBar.css";
// added monitor-related icons
import {
  FaPlus,
  FaEye,
  FaCog,
  FaChartBar,
  FaChevronLeft,
  FaComments,
  FaMapMarkedAlt,
  FaLayerGroup,
  FaMapMarkerAlt,
  FaFlag,
  FaUsers,
  FaFileAlt,
  FaClipboardList,
  FaArchive,
  FaUserCog,
  FaUser,
  FaPalette,
  FaShieldAlt,
  FaPlug,
} from "react-icons/fa"; // Using react-icons

const items = [
  { id: "create", label: "Create", icon: FaPlus },
  { id: "monitor", label: "Monitor", icon: FaEye },
  { id: "stats", label: "Stats", icon: FaChartBar },
  { id: "chat", label: "Chat", icon: FaComments },
  { id: "settings", label: "Settings", icon: FaCog },
];

// named Joystick export used by DashboardLayout (creates/destroys nipplejs and forwards moves)
export function Joystick({ isLocked = false, onMove = () => { }, position = "right", small = false }) {
  const joystickRef = useRef(null);
  const instanceRef = useRef(null);

  useEffect(() => {
    // destroy existing if locked
    if (isLocked) {
      if (instanceRef.current) {
        try { instanceRef.current.destroy(); } catch { }
        instanceRef.current = null;
      }
      return;
    }

    if (!joystickRef.current) return;

    const joy = nipplejs.create({
      zone: joystickRef.current,
      mode: "static",
      position: { top: "50%", left: "50%" },
      color: "#888",
      size: small ? 90 : 120,
    });
    instanceRef.current = joy;

    const handleMove = (_evt, data) => {
      if (!data) return;
      onMove(data);
    };
    const handleEnd = () => {
      onMove({ x: 0, y: 0, force: 0, angle: 0, type: "joystick" });
    };

    joy.on("move", handleMove);
    joy.on("end", handleEnd);

    return () => {
      try {
        joy.off("move", handleMove);
        joy.off("end", handleEnd);
        joy.destroy();
      } catch { }
      instanceRef.current = null;
    };
  }, [isLocked, onMove, small]);

  // compute style based on requested position and small/minimized state
  const sizePx = small ? 140 : 180;
  const zonePx = small ? 110 : 150;
  const panelStyle = {
    position: "fixed",
    top: "50%",
    transform: "translateY(-50%)",
    width: sizePx,
    height: sizePx,
    backgroundColor: "#222",
    borderRadius: 20,
    boxShadow: "0 0 15px rgba(0,0,0,0.7)",
    display: "flex",
    justifyContent: "center",
    alignItems: "center",
    zIndex: 1000,
    pointerEvents: isLocked ? "none" : "auto",
    opacity: isLocked ? 0.45 : 1,
    // position left of map (after sidebar) when requested, otherwise default right
    ...(position === "left"
      ? { left: 84 } // 64px sidebar + ~20px offset (adjust if your sidebar width differs)
      : { right: 20 }),
  };

  return (
    <div style={panelStyle} aria-hidden={isLocked}>
      <div ref={joystickRef} className="joystick-zone" style={{ width: zonePx, height: zonePx }} />
    </div>
  );
}

const Sidebar = (props) => {
  // accept new lock-related props (no local joystick)
  const { onSelect, onBack, isLocked, showLockedAttemptToast } = props || {};
  const [active, setActive] = useState("settings"); // default active

  // create panel state (right-side panel). No left-rail expansion.
  const [showCreateSub, setShowCreateSub] = useState(false);
  const [createPos, setCreatePos] = useState(null); // { left, top }

  // monitor panel state and position (new)
  const [showMonitorSub, setShowMonitorSub] = useState(false);
  const [monitorPos, setMonitorPos] = useState(null);

  // settings panel state and position (new)
  const [showSettingsSub, setShowSettingsSub] = useState(false);
  const [settingsPos, setSettingsPos] = useState(null);

  const handleActivate = (id, e) => {
    // If UI is locked, inform user and don't proceed
    if (isLocked) {
      try {
        if (typeof showLockedAttemptToast === "function") showLockedAttemptToast();
      } catch { }
      return;
    }
    setActive(id);
    if (id === "create") {
      // ensure monitor panel is hidden when opening create
      setShowMonitorSub(false);
      setShowSettingsSub(false);
      // compute position adjacent to the clicked list item
      if (e && e.currentTarget && e.currentTarget.getBoundingClientRect) {
        const rect = e.currentTarget.getBoundingClientRect();
        setCreatePos({
          left: Math.round(rect.right + 8),
          top: Math.round(rect.top + window.scrollY),
        });
      } else {
        setCreatePos(null);
      }
      setShowCreateSub((s) => !s);
    } else if (id === "monitor") {
      // open monitor panel (anchor near monitor icon), hide create
      setShowCreateSub(false);
      setShowSettingsSub(false);
      if (e && e.currentTarget && e.currentTarget.getBoundingClientRect) {
        const rect = e.currentTarget.getBoundingClientRect();
        setMonitorPos({
          left: Math.round(rect.right + 8),
          top: Math.round(rect.top + window.scrollY),
        });
      } else {
        setMonitorPos(null);
      }
      setShowMonitorSub((s) => !s);
    } else if (id === "settings") {
      // open settings panel (anchor near settings icon), hide others
      setShowCreateSub(false);
      setShowMonitorSub(false);
      if (e && e.currentTarget && e.currentTarget.getBoundingClientRect) {
        const rect = e.currentTarget.getBoundingClientRect();
        setSettingsPos({
          left: Math.round(rect.right + 8),
          top: Math.round(rect.top + window.scrollY),
        });
      } else {
        setSettingsPos(null);
      }
      setShowSettingsSub((s) => !s);
    } else {
      // hide create panel when selecting other items
      setShowCreateSub(false);
      setCreatePos(null);
      // hide monitor panel as well
      setShowMonitorSub(false);
      setMonitorPos(null);
      // hide settings panel as well
      setShowSettingsSub(false);
      setSettingsPos(null);
    }
    if (typeof onSelect === "function") onSelect(id);
  };

  const handleKey = (e, id) => {
    if (isLocked) {
      try {
        if (typeof showLockedAttemptToast === "function") showLockedAttemptToast();
      } catch { }
      return;
    }
    if (e.key === "Enter" || e.key === " ") {
      e.preventDefault();
      // pass the event so we can compute position for keyboard activation too
      handleActivate(id, e);
    }
  };
  return (
    <>
      <aside
        className={`sidebar`}
        aria-label="Main sidebar"
        // spacing kept here; visual border/color comes from SideBar.css
        style={{ paddingTop: 12 }}
      >
        {/* center the menu vertically */}
        <div className="menu-center">
          {/* emergency button placed in the middle */}

          <ul className="sidebar-menu" role="menu">
            {items.map(({ id, label, icon: Icon, badge }) => (
              <React.Fragment key={id}>
                <li
                  role="menuitem"
                  tabIndex={0}
                  className={`sidebar-item column ${active === id ? "active" : ""}`}
                  onClick={(e) => handleActivate(id, e)}
                  onKeyDown={(e) => handleKey(e, id)}
                  aria-label={label}
                  title={label}
                >
                  <div
                    className={`icon-bg ${active === id ? "visible" : ""}`}
                    aria-hidden="true"
                  >
                    <Icon className="icon-inner" />
                  </div>
                  {/* label below icon (visible when expanded) */}
                  <span className="label" aria-hidden="true">
                    {label}
                  </span>

                  {/* badge (placed at top-right of icon) */}
                  {badge ? (
                    <span className="badge" aria-hidden="true">
                      {badge}
                    </span>
                  ) : null}

                  {/* floating tooltip visible when collapsed */}
                  <span className="label-tooltip" aria-hidden="true">
                    {label}
                  </span>

                  <span className="active-indicator" aria-hidden="true" />
                </li>
              </React.Fragment>
            ))}
          </ul>

          {/* bottom chevron */}
          <div className="sidebar-bottom" aria-hidden="true">
            <button
              type="button"
              className="chev-btn"
              aria-label={
                showCreateSub || showMonitorSub || showSettingsSub
                  ? "back"
                  : "back"
              }
              onClick={() => {
                // if locked, inform user
                if (isLocked) {
                  try {
                    if (typeof showLockedAttemptToast === "function") showLockedAttemptToast();
                  } catch { }
                  return;
                }
                // back behavior: close any open anchored panel first, else notify parent
                if (showCreateSub) {
                  setShowCreateSub(false);
                  setCreatePos(null);
                  return;
                }
                if (showMonitorSub) {
                  setShowMonitorSub(false);
                  setMonitorPos(null);
                  return;
                }
                if (showSettingsSub) {
                  setShowSettingsSub(false);
                  setSettingsPos(null);
                  return;
                }
                // nothing open in-sidebar: delegate to parent
                if (typeof onBack === "function") onBack();
              }}
            >
              <FaChevronLeft />
            </button>
          </div>
        </div>
      </aside>

      {/* Right-side panel that appears when Create is active (sky-blue background, anchored near Create icon) */}
      {showCreateSub && (
        <div
          className="sidebar-right-panel"
          role="region"
          aria-label="Create options"
          style={
            createPos
              ? { left: `${createPos.left}px`, top: `${createPos.top}px` }
              : {}
          }
        >
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowCreateSub(false);
              setCreatePos(null);
              if (onSelect) onSelect("maps");
            }}
          >
            <FaMapMarkedAlt className="right-sub-icon" />
            <span>Maps</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowCreateSub(false);
              setCreatePos(null);
              if (onSelect) onSelect("zones");
            }}
          >
            <FaLayerGroup className="right-sub-icon" />
            <span>Zones</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowCreateSub(false);
              setCreatePos(null);
              if (onSelect) onSelect("waypoints");
            }}
          >
            <FaMapMarkerAlt className="right-sub-icon" />
            <span>Waypoints</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowCreateSub(false);
              setCreatePos(null);
              if (onSelect) onSelect("missions");
            }}
          >
            <FaFlag className="right-sub-icon" />
            <span>Missions</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowCreateSub(false);
              setCreatePos(null);
              if (onSelect) onSelect("users");
            }}
          >
            <FaUsers className="right-sub-icon" />
            <span>Users</span>
          </button>
        </div>
      )}

      {/* Monitor panel (new) — anchored near Monitor icon */}
      {showMonitorSub && (
        <div
          className="sidebar-right-panel"
          role="region"
          aria-label="Monitor options"
          style={
            monitorPos
              ? { left: `${monitorPos.left}px`, top: `${monitorPos.top}px` }
              : {}
          }
        >
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowMonitorSub(false);
              setMonitorPos(null);
              if (onSelect) onSelect("analytics");
            }}
          >
            <FaChartBar className="right-sub-icon" />
            <span>Analytics</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowMonitorSub(false);
              setMonitorPos(null);
              if (onSelect) onSelect("diagnostics");
            }}
          >
            <FaComments className="right-sub-icon" />
            <span>Diagnostics</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowMonitorSub(false);
              setMonitorPos(null);
              if (onSelect) onSelect("logs");
            }}
          >
            <FaFileAlt className="right-sub-icon" />
            <span>Logs</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowMonitorSub(false);
              setMonitorPos(null);
              if (onSelect) onSelect("mission-logs");
            }}
          >
            <FaClipboardList className="right-sub-icon" />
            <span>Mission Logs</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowMonitorSub(false);
              setMonitorPos(null);
              if (onSelect) onSelect("robot-bags");
            }}
          >
            <FaArchive className="right-sub-icon" />
            <span>Robot Bags</span>
          </button>
        </div>
      )}

      {/* Settings panel — anchored near Settings icon */}
      {showSettingsSub && (
        <div
          className="sidebar-right-panel"
          role="region"
          aria-label="Settings options"
          style={
            settingsPos
              ? { left: `${settingsPos.left}px`, top: `${settingsPos.top}px` }
              : {}
          }
        >
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowSettingsSub(false);
              setSettingsPos(null);
              if (onSelect) onSelect("robot-settings");
            }}
          >
            <FaUserCog className="right-sub-icon" />
            <span>Robot</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowSettingsSub(false);
              setSettingsPos(null);
              if (onSelect) onSelect("account");
            }}
          >
            <FaUser className="right-sub-icon" />
            <span>Account</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowSettingsSub(false);
              setSettingsPos(null);
              if (onSelect) onSelect("appearance");
            }}
          >
            <FaPalette className="right-sub-icon" />
            <span>Appearance</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowSettingsSub(false);
              setSettingsPos(null);
              if (onSelect) onSelect("security");
            }}
          >
            <FaShieldAlt className="right-sub-icon" />
            <span>Security</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              if (isLocked) {
                try { if (typeof showLockedAttemptToast === "function") showLockedAttemptToast(); } catch { }
                return;
              }
              e.stopPropagation();
              e.preventDefault();
              setShowSettingsSub(false);
              setSettingsPos(null);
              if (onSelect) onSelect("integrations");
            }}
          >
            <FaPlug className="right-sub-icon" />
            <span>Integrations</span>
          </button>
        </div>
      )}
    </>
  );
};

export default Sidebar;
