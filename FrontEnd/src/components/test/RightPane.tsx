import React, { useState, useRef } from "react";
import Stats from "../rightPane/stats/Stats.tsx"; // Assumed to be your existing componentimport Chat from "../rightPane/chat/Chat";
import Chat from "../rightPane/chat/chat.tsx";

// Create
import Maps from "../rightPane/create/Maps.tsx";
import Waypoints from "../rightPane/create/Waypoints.tsx";
import Zones from "../rightPane/create/Zones.tsx";
import Missions from "../rightPane/create/Missions.tsx";
import Users from "../rightPane/create/users.tsx";

// Monitor
import Analytics from "../rightPane/monitor/Analytics.tsx";
import Diagnostics from "../rightPane/monitor/Diagnostics.tsx";
import Logs from "../rightPane/monitor/Logs.tsx";
import MissionLogs from "../rightPane/monitor/MissionLogs.tsx";
import Camera from "../rightPane/monitor/Camera.tsx";
import Bridge from "../rightPane/monitor/Bridge.tsx";
import RobotBags from "../rightPane/monitor/RobotBags.tsx";

// Settings
import Account from "../rightPane/settings/Account.tsx";
import Robot from "../rightPane/settings/Robot.tsx";
import Appearance from "../rightPane/settings/Appearance.tsx";
import Security from "../rightPane/settings/Security.tsx";
import Integrations  from "../rightPane/settings/Integrations.tsx";


import {
  FaPlus,
  FaEdit,
  FaTrash,
  FaMicrophone,
  FaPaperPlane,
  FaTimes,
  FaFolderOpen,
} from "react-icons/fa";

const RightPane = (props) => {
  // --- Local State ---
  
  const [editingUserId, setEditingUserId] = useState(null);
  
  const [editUserForm, setEditUserForm] = useState({
    username: "",
    email: "",
    company: "",
    amr_type: "",
    role: "",
    approval: "",
  });

  const [mapModalOpen, setMapModalOpen] = useState(false);
  const [mapModalMode, setMapModalMode] = useState("preview"); // "preview" | "edit" | "create"
  const [mapForm, setMapForm] = useState({
    id: null,
    name: "",
    createdBy: "",
    image: "",
    status: "",
    category: "",
    createdAt: "",
  });
  
  const [isRecording, setIsRecording] = useState(false);
  const mapImageInputRef = useRef(null);
  
  // --- Destructure Props ---
  const {
    rightPage,
    setRightPage,
  } = props;
  console.log(rightPage)
  // --- Helper Functions ---

  const openMapModal = (mode, map = null) => {
    setMapModalMode(mode);
    const today = new Date().toISOString().slice(0, 10);
    if (!map) {
      setMapForm({
        id: null,
        name: "",
        createdBy: "",
        image: "",
        status: "",
        category: "",
        createdAt: today,
      });
    } else {
      setMapForm({
        id: map.id,
        name: map.name || "",
        createdBy: map.createdBy || "",
        image: map.image || "",
        status: map.status || "",
        category: map.category || "",
        createdAt: map.createdAt || today,
      });
    }
    setMapModalOpen(true);
  };

  const closeMapModal = () => {
    setMapModalOpen(false);
    setMapForm({
      id: null,
      name: "",
      createdBy: "",
      image: "",
      status: "",
      category: "",
      createdAt: "",
    });
  };

  const saveMapFromForm = async () => {
    if (!mapForm.name.trim()) {
      toast?.error?.("Map name required");
      return;
    }
    const payload = {
      name: mapForm.name.trim(),
      createdBy: mapForm.createdBy,
      image: mapForm.image,
      status: mapForm.status,
      category: mapForm.category,
      createdAt: mapForm.createdAt,
    };
    try {
      if (mapModalMode === "edit") {
        const id = mapForm.id;
        setMapsList((prev) =>
          prev.map((m) => (m.id === id ? { ...m, ...payload } : m))
        );
        if (selectedMap && selectedMap.id === id) {
          setSelectedMap((s) => ({ ...s, ...payload }));
        }
        toast?.success?.("Map updated");
        closeMapModal();
      }
    } catch (err) {
      console.error("Save map error", err);
      toast?.error?.(err.message || "Failed to save map");
    }
  };

  const handleMapImageChange = (e) => {
    const file = e.target.files?.[0];
    if (!file) return;
    const validTypes = [
      "image/png",
      "image/jpeg",
      "image/jpg",
      "image/gif",
      "image/webp",
    ];
    if (!validTypes.includes(file.type)) {
      toast?.error?.("Invalid image type");
      return;
    }
    if (file.size > 5 * 1024 * 1024) {
      toast?.error?.("File size > 5MB");
      return;
    }
    const reader = new FileReader();
    reader.onload = () => {
      setMapForm((prev) => ({ ...prev, image: reader.result || "" }));
      toast?.success?.("Image loaded");
    };
    reader.readAsDataURL(file);
  };

  const triggerMapImagePicker = (e) => {
    if (e) {
      e.preventDefault();
      e.stopPropagation();
    }
    if (mapImageInputRef.current) {
      mapImageInputRef.current.value = "";
      mapImageInputRef.current.click();
    }
  };

  const filterBySearch = (items, searchTerm, searchField, fieldMap) => {
    if (!searchTerm || !searchTerm.trim()) return items;
    const term = searchTerm.trim().toLowerCase();
    return items.filter((item) => {
      if (searchField === "any") {
        return Object.values(fieldMap).some((field) =>
          String(item[field] || "").toLowerCase().includes(term)
        );
      }
      const fieldName = fieldMap[searchField];
      return String(item[fieldName] || "").toLowerCase().includes(term);
    });
  };

  const safeNumber = (value?: number): number => {
  return typeof value === "number" && !isNaN(value) ? value : 0;
};
  // --- Render ---

  return (
    <aside
      className={`
        fixed right-0 top-14 w-full lg:w-1/2 h-[calc(100vh-3.5rem)]
        bg-white shadow-2xl z-40 flex flex-col border-l border-slate-200
        text-slate-900 transition-all duration-300 ease-in-out
        ${rightPage ? "translate-x-0 opacity-100" : "translate-x-full opacity-0 pointer-events-none"}
      `}
      aria-label="Right pane"
    >
      {/* Header */}
      <div className="flex items-center justify-between px-6 py-4 border-b border-slate-100 bg-gradient-to-b from-sky-50/50 to-white">
        <strong className="text-xl font-bold text-slate-800 capitalize">
          {rightPage}
        </strong>
        <button
          className="p-2 rounded-full text-slate-400 hover:text-slate-600 hover:bg-slate-100 transition-colors"
          onClick={() => setRightPage(null)}
          aria-label="Close"
        >
          <FaTimes />
        </button>
      </div>

      {/* Body */}
      <div className="flex-1 overflow-y-auto p-4 bg-white">

        {/* MAPS */}
        {rightPage === "maps" && (<Maps {...props} />)}

        {/* ZONES */}
        {rightPage === "zones" && (<Zones {...props} />)}


        {/* MISSIONS */}
        {rightPage === "missions" && (<Missions {...props} />)}


        {/* Simplified logic for brevity, using same styling components */}
        {rightPage === "waypoints" && (<Waypoints {...props} />)}


        {/* USERS EXAMPLE (demonstrating conditional form) */}
        {rightPage === "users" && (<Users {...props} />)}


        {/* ANALYTICS */}
        {rightPage === "analytics" && (<Analytics {...props} />)}

        {/* DIAGNOSTICS */}
        {rightPage === "diagnostics" && (<Diagnostics {...props} />)}

        {/* LOGS */}
        {rightPage === "logs" && (<Logs {...props} />)}

        {/* MISSION LOGS */}
        {rightPage === "mission-logs" && (<MissionLogs {...props} />)}

        {/* CAMERA */}
        {rightPage === "camera" && (<Camera {...props} />)}

        {/* AMR BRIDGE */}
        {rightPage === "bridge" && (<Bridge {...props} />)}

        {/* ROBOT BAGS */}
        {rightPage === "robot-bags" && (<RobotBags {...props} />)}

        {/* CHAT */}
        {rightPage === "chat" && (<Chat {...props} />)}

        {/* Stats Component Injection */}
        {rightPage === "stats" && (<Stats {...props} />)}
        
        {/* ACCOUNT SETTINGS */}
        {rightPage === "account" && (<Account {...props} />)}

        {/* ROBOT SETTINGS */}
        {rightPage === "robot-settings" && (<Robot {...props} />)}

        {/* APPEARANCE SETTINGS */}
        {rightPage === "appearance" && (<Appearance {...props} />)}

        {/* SECURITY SETTINGS */}
        {rightPage === "security" && (<Security {...props} />)}
        
        {/* INTEGRATIONS */}
        {rightPage === "integrations" && (<Integrations {...props} />)}        

      </div>
    </aside>
  );
};

export default RightPane;