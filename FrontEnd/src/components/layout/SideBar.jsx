import React, { useState } from "react";
import FloatingMenu, {MenuItem} from "./FloatingMenu"; // Import the new component
import {
  FaPlus, FaEye, FaCog, FaChartBar, FaChevronLeft, FaComments,
  FaMapMarkedAlt, FaLayerGroup, FaMapMarkerAlt, FaFlag, FaUsers,
  FaFileAlt, FaClipboardList, FaArchive, FaUserCog, FaUser,
  FaPalette, FaShieldAlt, FaPlug,
} from "react-icons/fa";

const items = [
  { id: "create", label: "Create", icon: FaPlus },
  { id: "monitor", label: "Monitor", icon: FaEye },
  { id: "stats", label: "Stats", icon: FaChartBar },
  { id: "chat", label: "Chat", icon: FaComments },
  { id: "settings", label: "Settings", icon: FaCog },
];

const Sidebar = ({ onSelect, onBack }) => {
  const [active, setActive] = useState("settings");

  // Panel States
  const [activePanel, setActivePanel] = useState(null); // 'create' | 'monitor' | 'settings' | null
  const [panelPos, setPanelPos] = useState(null);

  const handleActivate = (id, e) => {
    setActive(id);
    
    // If clicking the items that have sub-menus
    if (["create", "monitor", "settings"].includes(id)) {
      // Toggle logic
      if (activePanel === id) {
        setActivePanel(null);
        setPanelPos(null);
      } else {
        const rect = e?.currentTarget?.getBoundingClientRect();
        if (rect) {
          setActivePanel(id);
          setPanelPos({
            left: Math.round(rect.right + 10), // Gap from sidebar
            top: Math.round(rect.top),
          });
        }
      }
    } else {
      // Direct action items (Stats, Chat)
      setActivePanel(null);
      setPanelPos(null);
      if (onSelect) onSelect(id);
    }
  };

  const closePanel = () => {
    setActivePanel(null);
    setPanelPos(null);
  };

  const handleSubSelect = (viewId) => {
    closePanel();
    if (onSelect) onSelect(viewId);
  };

  return (
    <>
      <aside
        className="fixed left-0 top-14 bottom-0 z-40 flex w-16 flex-col items-center bg-white border-r border-slate-200 pt-4 shadow-sm transition-all duration-300"
        aria-label="Main sidebar"
      >
        <div className="flex w-full flex-1 flex-col items-center">
          <ul className="flex w-full flex-col gap-3 px-2">
            {items.map(({ id, label, icon: Icon }) => {
              const isActive = active === id;
              return (
                <li
                  key={id}
                  className="group relative flex cursor-pointer justify-center"
                  onClick={(e) => handleActivate(id, e)}
                >
                  <div
                    className={`
                      flex h-10 w-10 items-center justify-center rounded-xl text-lg transition-all duration-200
                      ${isActive 
                        ? "bg-sky-100 text-sky-600 shadow-inner" 
                        : "text-slate-400 hover:bg-slate-50 hover:text-slate-600"
                      }
                    `}
                  >
                    <Icon />
                  </div>
                  {/* Tooltip */}
                  <span className="pointer-events-none absolute left-14 top-1/2 z-[60] -translate-y-1/2 whitespace-nowrap rounded bg-slate-800 px-2 py-1 text-xs text-white opacity-0 shadow-lg transition-opacity duration-200 group-hover:opacity-100">
                    {label}
                  </span>
                </li>
              );
            })}
          </ul>

          {/* Bottom Controls */}
          <div className="mt-auto mb-4">
            <button
              className="flex h-10 w-10 items-center justify-center rounded-full text-slate-400 hover:bg-slate-100 hover:text-slate-700 transition-colors"
              onClick={() => {
                if (activePanel) closePanel();
                else if (onBack) onBack();
              }}
            >
              <FaChevronLeft />
            </button>
          </div>
        </div>
      </aside>

      {/* Floating Gradient Panel for Sub-menus */}
      {activePanel === "create" && panelPos && (
        <FloatingMenu pos={panelPos}>
          <div className="px-3 py-1 text-[10px] font-bold uppercase tracking-wider text-sky-800/60">Create</div>
          <MenuItem icon={FaMapMarkedAlt} label="Maps" onClick={() => handleSubSelect("maps")} />
          <MenuItem icon={FaLayerGroup} label="Zones" onClick={() => handleSubSelect("zones")} />
          <MenuItem icon={FaMapMarkerAlt} label="Waypoints" onClick={() => handleSubSelect("waypoints")} />
          <MenuItem icon={FaFlag} label="Missions" onClick={() => handleSubSelect("missions")} />
          <MenuItem icon={FaUsers} label="Users" onClick={() => handleSubSelect("users")} />
        </FloatingMenu>
      )}

      {activePanel === "monitor" && panelPos && (
        <RightPanel pos={panelPos}>
          <div className="px-3 py-1 text-[10px] font-bold uppercase tracking-wider text-sky-800/60">Monitor</div>
          <PanelItem icon={FaChartBar} label="Analytics" onClick={() => handleSubSelect("analytics")} />
          <PanelItem icon={FaEye} label="Camera" onClick={() => handleSubSelect("camera")} />
          <PanelItem icon={FaLayerGroup} label="AMR Bridge" onClick={() => handleSubSelect("bridge")} />
          <PanelItem icon={FaComments} label="Diagnostics" onClick={() => handleSubSelect("diagnostics")} />
          <PanelItem icon={FaFileAlt} label="Logs" onClick={() => handleSubSelect("logs")} />
          <PanelItem icon={FaClipboardList} label="Mission Logs" onClick={() => handleSubSelect("mission-logs")} />
          <PanelItem icon={FaArchive} label="Robot Bags" onClick={() => handleSubSelect("robot-bags")} />
        </RightPanel>
      )}

      {activePanel === "settings" && panelPos && (
        <RightPanel pos={panelPos}>
          <div className="px-3 py-1 text-[10px] font-bold uppercase tracking-wider text-sky-800/60">Settings</div>
          <PanelItem icon={FaUserCog} label="Robot" onClick={() => handleSubSelect("robot-settings")} />
          <PanelItem icon={FaUser} label="Account" onClick={() => handleSubSelect("account")} />
          <PanelItem icon={FaPalette} label="Appearance" onClick={() => handleSubSelect("appearance")} />
          <PanelItem icon={FaShieldAlt} label="Security" onClick={() => handleSubSelect("security")} />
          <PanelItem icon={FaPlug} label="Integrations" onClick={() => handleSubSelect("integrations")} />
        </RightPanel>
      )}
    </>
  );
};

export default Sidebar;