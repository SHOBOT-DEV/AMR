import React from "react";
import { useState } from "react";
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

const Sidebar = (props) => {
  const { onSelect, onBack } = props || {};
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
        // small visual separator between left rail and main content
        style={{ borderRight: "1px solid #e6eef2", paddingTop: 12 }}
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
              e.stopPropagation();
              e.preventDefault();
              setShowMonitorSub(false);
              setMonitorPos(null);
              if (onSelect) onSelect("camera");
            }}
          >
            <FaLayerGroup className="right-sub-icon" />
            <span>Camera</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
              e.stopPropagation();
              e.preventDefault();
              setShowMonitorSub(false);
              setMonitorPos(null);
              if (onSelect) onSelect("bridge");
            }}
          >
            <FaLayerGroup className="right-sub-icon" />
            <span>AMR Bridge</span>
          </button>
          <button
            type="button"
            className="right-create-item"
            onClick={(e) => {
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
