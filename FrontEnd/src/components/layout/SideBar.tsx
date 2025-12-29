import React, { useState, MouseEvent, KeyboardEvent } from "react";
import type { IconType } from "react-icons";
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
  FaCamera,
  FaNetworkWired,
  FaWrench,
} from "react-icons/fa";

type SidebarItem = {
  id: string;
  label: string;
  icon: IconType;
  badge?: string;
};

const items: SidebarItem[] = [
  { id: "create", label: "Create", icon: FaPlus },
  { id: "monitor", label: "Monitor", icon: FaEye },
  { id: "stats", label: "Stats", icon: FaChartBar },
  { id: "chat", label: "Chat", icon: FaComments },
  { id: "settings", label: "Settings", icon: FaCog },
];

type SidebarProps = {
  onSelect?: (id: string) => void;
  onBack?: () => void;
};

type PanelPosition = {
  left: number;
  top: number;
};

type AnchorEvent = MouseEvent<HTMLElement> | KeyboardEvent<HTMLElement>;

const iconBaseClasses =
  "relative flex h-12 w-12 items-center justify-center rounded-2xl border border-white/50 bg-white/70 text-lg text-[#052034] transition-all duration-200";

const Sidebar: React.FC<SidebarProps> = ({ onSelect, onBack }) => {
  const [active, setActive] = useState<string>("settings");
  const [showCreateSub, setShowCreateSub] = useState(false);
  const [createPos, setCreatePos] = useState<PanelPosition | null>(null);
  const [showMonitorSub, setShowMonitorSub] = useState(false);
  const [monitorPos, setMonitorPos] = useState<PanelPosition | null>(null);
  const [showSettingsSub, setShowSettingsSub] = useState(false);
  const [settingsPos, setSettingsPos] = useState<PanelPosition | null>(null);

  const anchorToListItem = (event?: AnchorEvent): PanelPosition | null => {
    const element = event?.currentTarget as HTMLElement | null;
    if (!element || !element.getBoundingClientRect) return null;
    const rect = element.getBoundingClientRect();
    return {
      left: Math.round(rect.right + 8),
      top: Math.round(rect.top + window.scrollY),
    };
  };

  const hideAllPanels = () => {
    setShowCreateSub(false);
    setCreatePos(null);
    setShowMonitorSub(false);
    setMonitorPos(null);
    setShowSettingsSub(false);
    setSettingsPos(null);
  };

  const handleActivate = (id: string, event?: AnchorEvent) => {
    setActive(id);

    if (id === "create") {
      setShowMonitorSub(false);
      setMonitorPos(null);
      setShowSettingsSub(false);
      setSettingsPos(null);
      setCreatePos(anchorToListItem(event));
      setShowCreateSub((prev) => !prev);
    } else if (id === "monitor") {
      setShowCreateSub(false);
      setCreatePos(null);
      setShowSettingsSub(false);
      setSettingsPos(null);
      setMonitorPos(anchorToListItem(event));
      setShowMonitorSub((prev) => !prev);
    } else if (id === "settings") {
      setShowCreateSub(false);
      setCreatePos(null);
      setShowMonitorSub(false);
      setMonitorPos(null);
      setSettingsPos(anchorToListItem(event));
      setShowSettingsSub((prev) => !prev);
    } else {
      hideAllPanels();
    }

    if (typeof onSelect === "function") onSelect(id);
  };

  const handleKeyDown = (event: KeyboardEvent<HTMLLIElement>, id: string) => {
    if (event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      handleActivate(id, event);
    }
  };

  const panelButtonClass =
    "flex items-center gap-3 rounded-xl border border-white/60 bg-[#e3f3ff]/80 px-3 py-2 text-left text-sm font-semibold text-[#052034] shadow-sm transition hover:bg-white hover:text-[#0369a1]";

  const renderPanel = (
    isVisible: boolean,
    position: PanelPosition | null,
    label: string,
    content: React.ReactNode,
  ) => {
    if (!isVisible) return null;
    const style = position
      ? { left: `${position.left}px`, top: `${position.top}px` }
      : undefined;

    return (
      <div
        className="fixed z-[120] w-56 rounded-2xl border border-white/60 bg-white/80 p-4 shadow-2xl backdrop-blur"
        role="region"
        aria-label={label}
        style={style}
      >
        <div className="flex flex-col gap-2">{content}</div>
      </div>
    );
  };

  return (
    <>
      <aside
        className="group fixed left-0 top-0 z-[60] flex h-screen w-[72px] flex-col items-center border-r border-[#e6eef2] bg-[#87ceeb] pt-3 text-[#052034] transition-all duration-300"
        aria-label="Main sidebar"
      >
        <div className="flex h-full w-full flex-col items-center">
          <div className="flex w-full flex-1 items-center">
            <ul className="flex w-full flex-col items-center gap-3 px-1 py-4" role="menu">
              {items.map(({ id, label, icon: Icon, badge }) => {
                const isActive = active === id;
                return (
                  <li
                  key={id}
                  role="menuitem"
                  tabIndex={0}
                  className={`group/list relative flex w-full cursor-pointer flex-col items-center gap-1 rounded-xl px-1 py-2 text-sm font-semibold outline-none transition focus-visible:ring-2 focus-visible:ring-white/60 ${isActive ? "text-[#0369a1]" : "text-[#052034]"}`}
                  onClick={(event) => handleActivate(id, event)}
                  onKeyDown={(event) => handleKeyDown(event, id)}
                  aria-label={label}
                  title={label}
                >
                  <span
                    className={`${iconBaseClasses} ${isActive ? "scale-105 bg-white text-[#0369a1] shadow-xl" : "hover:bg-white hover:text-[#0369a1]"}`}
                    aria-hidden="true"
                  >
                    <Icon className="text-xl" />
                    {badge ? (
                      <span className="absolute -right-1 -top-1 rounded-full bg-[#ff6b6b] px-1.5 text-xs text-white">
                        {badge}
                      </span>
                    ) : null}
                  </span>
                  <span className="text-[11px] uppercase tracking-wide" aria-hidden="true">
                    {label}
                  </span>
                  <span className="pointer-events-none absolute left-[84px] top-1/2 -translate-y-1/2 rounded-md bg-slate-900/90 px-2 py-1 text-xs text-white opacity-0 transition group-hover/list:opacity-100">
                    {label}
                  </span>
                  {isActive ? (
                    <span className="absolute -left-4 h-10 w-2 rounded-r-full bg-gradient-to-b from-[#7fc6ff] to-[#1e90ff] shadow-[0_2px_8px_rgba(30,144,255,0.12)]" aria-hidden="true" />
                  ) : null}
                </li>
              );
            })}
            </ul>
          </div>

          <div className="mt-auto w-full px-3 pb-4 pt-2">
            <button
              type="button"
              className="flex h-10 w-full items-center justify-center rounded-xl text-xl text-[#052034] transition hover:bg-white/40"
              aria-label={showCreateSub || showMonitorSub || showSettingsSub ? "back" : "back"}
              onClick={() => {
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
                if (typeof onBack === "function") onBack();
              }}
            >
              <FaChevronLeft />
            </button>
          </div>
        </div>
      </aside>

      {renderPanel(
        showCreateSub,
        createPos,
        "Create options",
        <>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowCreateSub(false);
              setCreatePos(null);
              onSelect?.("maps");
            }}
          >
            <FaMapMarkedAlt className="text-lg" />
            <span>Maps</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowCreateSub(false);
              setCreatePos(null);
              onSelect?.("zones");
            }}
          >
            <FaLayerGroup className="text-lg" />
            <span>Zones</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowCreateSub(false);
              setCreatePos(null);
              onSelect?.("waypoints");
            }}
          >
            <FaMapMarkerAlt className="text-lg" />
            <span>Waypoints</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowCreateSub(false);
              setCreatePos(null);
              onSelect?.("missions");
            }}
          >
            <FaFlag className="text-lg" />
            <span>Missions</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowCreateSub(false);
              setCreatePos(null);
              onSelect?.("users");
            }}
          >
            <FaUsers className="text-lg" />
            <span>Users</span>
          </button>
        </>,
      )}

      {renderPanel(
        showMonitorSub,
        monitorPos,
        "Monitor options",
        <>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowMonitorSub(false);
              setMonitorPos(null);
              onSelect?.("analytics");
            }}
          >
            <FaChartBar className="text-lg" />
            <span>Analytics</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowMonitorSub(false);
              setMonitorPos(null);
              onSelect?.("camera");
            }}
          >
            <FaCamera className="text-lg" />
            <span>Camera</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowMonitorSub(false);
              setMonitorPos(null);
              onSelect?.("bridge");
            }}
          >
            <FaNetworkWired className="text-lg" />
            <span>AMR Bridge</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowMonitorSub(false);
              setMonitorPos(null);
              onSelect?.("diagnostics");
            }}
          >
            <FaWrench className="text-lg" />
            <span>Diagnostics</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowMonitorSub(false);
              setMonitorPos(null);
              onSelect?.("logs");
            }}
          >
            <FaFileAlt className="text-lg" />
            <span>Logs</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowMonitorSub(false);
              setMonitorPos(null);
              onSelect?.("mission-logs");
            }}
          >
            <FaClipboardList className="text-lg" />
            <span>Mission Logs</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowMonitorSub(false);
              setMonitorPos(null);
              onSelect?.("robot-bags");
            }}
          >
            <FaArchive className="text-lg" />
            <span>Robot Bags</span>
          </button>
        </>,
      )}

      {renderPanel(
        showSettingsSub,
        settingsPos,
        "Settings options",
        <>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowSettingsSub(false);
              setSettingsPos(null);
              onSelect?.("account");
            }}
          >
            <FaUser className="text-lg" />
            <span>Account</span>
          </button>
                    <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowSettingsSub(false);
              setSettingsPos(null);
              onSelect?.("robot-settings");
            }}
          >
            <FaUserCog className="text-lg" />
            <span>Robot</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowSettingsSub(false);
              setSettingsPos(null);
              onSelect?.("appearance");
            }}
          >
            <FaPalette className="text-lg" />
            <span>Appearance</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowSettingsSub(false);
              setSettingsPos(null);
              onSelect?.("security");
            }}
          >
            <FaShieldAlt className="text-lg" />
            <span>Security</span>
          </button>
          <button
            type="button"
            className={panelButtonClass}
            onClick={(event) => {
              event.stopPropagation();
              setShowSettingsSub(false);
              setSettingsPos(null);
              onSelect?.("integrations");
            }}
          >
            <FaPlug className="text-lg" />
            <span>Integrations</span>
          </button>
        </>,
      )}
    </>
  );
};

export default Sidebar;
