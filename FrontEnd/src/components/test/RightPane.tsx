import React, { useState, useRef } from "react";
import Stats from "./Stats.tsx"; // Assumed to be your existing component
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
  const [waypointSearchField, setWaypointSearchField] = useState("any");
  const [waypointSearchTerm, setWaypointSearchTerm] = useState("");
  const [missionSearchField, setMissionSearchField] = useState("any");
  const [missionSearchTerm, setMissionSearchTerm] = useState("");
  const [userSearchField, setUserSearchField] = useState("any");
  const [userSearchTerm, setUserSearchTerm] = useState("");
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
  
  // --- Voice Recognition ---
  const handleMicClick = () => {
    const SpeechRecognition =
      window.SpeechRecognition || window.webkitSpeechRecognition;
      
    if (!SpeechRecognition) {
      alert("Your browser does not support voice input.");
      return;
    }

    const recognition = new SpeechRecognition();
    recognition.continuous = false;
    recognition.interimResults = false;
    recognition.lang = "en-US";

    if (isRecording) {
      recognition.stop();
      setIsRecording(false);
      return;
    }

    setIsRecording(true);
    recognition.start();

    recognition.onresult = (event) => {
      const text = event.results[0][0].transcript;
      props.setChatInput((prev) => prev + (prev ? " " : "") + text);
      setIsRecording(false);
    };

    recognition.onerror = () => setIsRecording(false);
    recognition.onend = () => setIsRecording(false);
  };

  // --- Destructure Props ---
  const {
    rightPage,
    setRightPage,
    // Maps
    mapsList,
    selectedMap,
    createNewMapImmediate,
    handleActivateMap,
    handleMapAction,
    mapSearchField,
    setMapSearchField,
    mapSearchTerm,
    setMapSearchTerm,
    requestV1,
    toast,
    setMapsList,
    setSelectedMap,
    // Zones
    zones,
    zoneFormOpen,
    setZoneFormOpen,
    zoneForm,
    setZoneForm,
    setZones,
    zoneSearchField,
    setZoneSearchField,
    zoneSearchTerm,
    setZoneSearchTerm,
    // Waypoints
    waypoints,
    waypointFormOpen,
    setWaypointFormOpen,
    waypointForm,
    setWaypointForm,
    setWaypoints,
    handleSelectWaypoint,
    setSelectedWaypointId,
    // Users
    users,
    usersLoading,
    usersError,
    loadUsers,
    selectedUserId,
    setSelectedUserId,
    userActionLoading,
    handleResetUserPassword,
    // Missions
    missions,
    missionFormOpen,
    setMissionFormOpen,
    missionForm,
    setMissionForm,
    selectedMissionId,
    setSelectedMissionId,
    handleSelectMission,
    setMissions,
    // Analytics/Logs
    analyticsSummary,
    analyticsAlerts,
    diagnosticsPanels,
    logEvents,
    missionHistory,
    bagFiles,
    analyticsChartSize,
    analyticsPath,
    // Settings
    robotSettingsState,
    toggleRobotSetting,
    accountProfile,
    handleAccountChange,
    profileError,
    profileSuccess,
    profileSaving,
    handleSaveProfile,
    selectedTheme,
    setSelectedTheme,
    securityPreferences,
    toggleSecurityPref,
    securityEvents,
    integrationItems,
    toggleIntegrationStatus,
    // Chat
    chatMessages,
    chatInput,
    setChatInput,
    handleSendMessage,
    handleKeyPress,
    isTyping,
    handleSuggestionClick,
    chatQuickPrompts,
    chatContainerRef,
    formatTimestamp,
    latestMessage,
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
      className="fixed right-0 top-14 w-full lg:w-1/2 h-[calc(100vh-3.5rem)] bg-white shadow-2xl z-40 flex flex-col border-l border-slate-200 text-slate-900 transition-all duration-300"
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
        {rightPage === "maps" && (
          <div className="flex flex-col gap-4">
            <input
              type="file"
              ref={mapImageInputRef}
              accept="image/*"
              className="hidden"
              onChange={handleMapImageChange}
            />

            <div className="flex flex-col sm:flex-row gap-2 items-center">
              <select
                className="px-3 py-2.5 border border-slate-200 rounded-lg bg-white text-sm focus:ring-2 focus:ring-sky-500 focus:outline-none"
                value={mapSearchField}
                onChange={(e) => setMapSearchField(e.target.value)}
              >
                <option value="any">Search By</option>
                <option value="name">Name</option>
                <option value="createdBy">Created By</option>
                <option value="category">Category</option>
                <option value="createdAt">Created At</option>
                <option value="status">Status</option>
              </select>

              <input
                className="flex-1 px-3 py-2.5 border border-slate-200 rounded-lg bg-white text-sm focus:ring-2 focus:ring-sky-500 focus:outline-none"
                value={mapSearchTerm}
                onChange={(e) => setMapSearchTerm(e.target.value)}
                placeholder="Type to search..."
              />

              <button
                onClick={createNewMapImmediate}
                className="px-4 py-2.5 bg-sky-600 text-white rounded-lg text-sm font-semibold flex items-center gap-2 hover:bg-sky-700 transition shadow-sm whitespace-nowrap"
              >
                <FaPlus />
                <span>New Map</span>
              </button>
            </div>

            <div className="border-t border-slate-100 pt-2">
              <div className="overflow-x-auto rounded-xl border border-slate-100 shadow-sm">
                <table className="w-full text-sm text-left">
                  <thead className="bg-slate-50 text-slate-500 uppercase text-xs font-semibold">
                    <tr>
                      <th className="px-4 py-3 text-center w-16">Active</th>
                      <th className="px-4 py-3">Name</th>
                      <th className="px-4 py-3">Created By</th>
                      <th className="px-4 py-3 text-right">Status</th>
                      <th className="px-4 py-3 text-right">Actions</th>
                    </tr>
                  </thead>
                  <tbody className="divide-y divide-slate-100">
                    {filterBySearch(mapsList || [], mapSearchTerm, mapSearchField, {
                      any: "name", name: "name", createdBy: "createdBy", category: "category", createdAt: "createdAt", status: "status",
                    }).map((m) => (
                      <tr
                        key={m.id}
                        onClick={() => handleActivateMap(m)}
                        className={`cursor-pointer transition-colors hover:bg-slate-50 ${selectedMap?.id === m.id ? "bg-sky-50/50" : ""}`}
                      >
                        <td className="px-4 py-3 text-center">
                          <input
                            type="radio"
                            name="activeMap"
                            checked={selectedMap?.id === m.id}
                            onChange={(e) => {
                                e.stopPropagation();
                                handleActivateMap(m);
                            }}
                            className="text-sky-600 focus:ring-sky-500"
                          />
                        </td>
                        <td className="px-4 py-3 font-medium text-slate-900">{m.name}</td>
                        <td className="px-4 py-3 text-slate-500">{m.createdBy}</td>
                        <td className="px-4 py-3 text-right">
                          <span
                            className={`px-2 py-1 rounded-full text-xs font-medium ${
                              (m.status || "").toLowerCase() === "active"
                                ? "bg-emerald-100 text-emerald-700"
                                : "bg-rose-100 text-rose-700"
                            }`}
                          >
                            {m.status || "Inactive"}
                          </span>
                        </td>
                        <td className="px-4 py-3 text-right">
                          <div className="flex items-center justify-end gap-2">
                            <button
                              title="Edit"
                              onClick={(e) => { e.stopPropagation(); openMapModal("edit", m); }}
                              className="p-1.5 text-slate-400 hover:text-sky-600 hover:bg-sky-50 rounded transition"
                            >
                              <FaEdit />
                            </button>
                            <button
                              title="Delete"
                              onClick={(e) => { e.stopPropagation(); handleMapAction("delete", m); }}
                              className="p-1.5 text-slate-400 hover:text-rose-600 hover:bg-rose-50 rounded transition"
                            >
                              <FaTrash />
                            </button>
                          </div>
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </div>

            {/* Modal */}
            {mapModalOpen && mapModalMode === "edit" && (
              <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/50 backdrop-blur-sm p-4">
                <div className="bg-white rounded-2xl shadow-2xl w-full max-w-lg overflow-hidden flex flex-col max-h-[90vh]">
                    <div className="flex items-center justify-between p-4 border-b border-slate-100">
                        <h3 className="font-bold text-lg">Edit Map</h3>
                        <button onClick={closeMapModal} className="text-slate-400 hover:text-slate-600"><FaTimes/></button>
                    </div>
                    <div className="p-6 overflow-y-auto space-y-4">
                        <div className="space-y-1">
                            <label className="text-xs font-semibold text-slate-500 uppercase">Map Name</label>
                            <input value={mapForm.name} onChange={e=>setMapForm(prev=>({...prev, name: e.target.value}))} className="w-full px-3 py-2 border border-slate-200 rounded-lg focus:ring-2 focus:ring-sky-500 outline-none" />
                        </div>
                        <div className="grid grid-cols-2 gap-4">
                            <div className="space-y-1">
                                <label className="text-xs font-semibold text-slate-500 uppercase">Created By</label>
                                <input value={mapForm.createdBy} onChange={e=>setMapForm(prev=>({...prev, createdBy: e.target.value}))} className="w-full px-3 py-2 border border-slate-200 rounded-lg outline-none" />
                            </div>
                             <div className="space-y-1">
                                <label className="text-xs font-semibold text-slate-500 uppercase">Status</label>
                                <select value={mapForm.status} onChange={e=>setMapForm(prev=>({...prev, status: e.target.value}))} className="w-full px-3 py-2 border border-slate-200 rounded-lg outline-none bg-white">
                                    <option value="">Inactive</option>
                                    <option value="Active">Active</option>
                                </select>
                            </div>
                        </div>

                        {/* Image Upload Area */}
                        <div className="bg-slate-50 border border-slate-200 rounded-xl p-4 space-y-3">
                            <div className="flex justify-between items-center">
                                <span className="text-xs font-bold text-slate-500 uppercase">Map Image</span>
                                {mapForm.image && (
                                    <button onClick={()=>setMapForm(prev=>({...prev, image:""}))} className="text-xs text-rose-500 hover:text-rose-700 font-medium">Remove Image</button>
                                )}
                            </div>
                            
                            {mapForm.image ? (
                                <img src={mapForm.image} alt="Preview" className="w-full h-40 object-contain bg-white rounded-lg border border-slate-100" />
                            ) : (
                                <div className="h-32 flex flex-col items-center justify-center border-2 border-dashed border-slate-300 rounded-lg text-slate-400 gap-2">
                                    <span className="text-sm">No image selected</span>
                                </div>
                            )}

                            <div className="flex gap-2">
                                <button type="button" onClick={triggerMapImagePicker} className="flex-1 py-2 bg-white border border-slate-300 rounded-lg text-sm font-medium hover:bg-slate-50 flex items-center justify-center gap-2">
                                    <FaFolderOpen /> Browse
                                </button>
                            </div>
                            <input value={!mapForm.image?.startsWith('data:') ? mapForm.image : ''} onChange={e=>setMapForm(prev=>({...prev, image: e.target.value}))} placeholder="Or paste Image URL..." className="w-full px-3 py-2 text-sm border border-slate-200 rounded-lg" />
                        </div>
                    </div>
                    <div className="p-4 border-t border-slate-100 bg-slate-50 flex justify-end gap-3">
                        <button onClick={closeMapModal} className="px-4 py-2 text-slate-600 font-medium hover:bg-slate-200 rounded-lg transition">Cancel</button>
                        <button onClick={saveMapFromForm} className="px-6 py-2 bg-sky-600 text-white font-medium rounded-lg hover:bg-sky-700 transition shadow-sm">Save Changes</button>
                    </div>
                </div>
              </div>
            )}
          </div>
        )}

        {/* ZONES */}
        {rightPage === "zones" && (
          <div className="space-y-4">
             {!selectedMap ? (
                 <div className="text-center py-12 text-slate-400">
                     <p>Please select a map to manage zones.</p>
                 </div>
             ) : (
                 <>
                    <div className="flex flex-col sm:flex-row gap-2">
                        <select className="px-3 py-2.5 border border-slate-200 rounded-lg bg-white text-sm" value={zoneSearchField} onChange={(e) => setZoneSearchField(e.target.value)}>
                            <option value="any">All Fields</option>
                            <option value="name">Name</option>
                            <option value="category">Category</option>
                        </select>
                        <input className="flex-1 px-3 py-2.5 border border-slate-200 rounded-lg text-sm" value={zoneSearchTerm} onChange={(e) => setZoneSearchTerm(e.target.value)} placeholder="Search zones..." />
                        <button onClick={() => setZoneFormOpen(!zoneFormOpen)} className="px-4 py-2.5 bg-sky-600 text-white rounded-lg text-sm font-semibold whitespace-nowrap hover:bg-sky-700 transition">
                            + New Zone
                        </button>
                    </div>

                    {zoneFormOpen && (
                        <div className="bg-slate-50 border border-slate-200 rounded-xl p-4 space-y-4 shadow-sm animate-in slide-in-from-top-2">
                             <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
                                 <input placeholder="Zone Name" value={zoneForm.name} onChange={e=>setZoneForm(p=>({...p, name: e.target.value}))} className="px-3 py-2 border rounded-lg text-sm" />
                                 <select value={zoneForm.category} onChange={e=>setZoneForm(p=>({...p, category: e.target.value}))} className="px-3 py-2 border rounded-lg text-sm bg-white">
                                     <option value="Safe">Safe</option>
                                     <option value="Caution">Caution</option>
                                     <option value="No-Go">No-Go</option>
                                 </select>
                                 <input placeholder="Geometry (Polygon...)" value={zoneForm.geometry} onChange={e=>setZoneForm(p=>({...p, geometry: e.target.value}))} className="px-3 py-2 border rounded-lg text-sm" />
                             </div>
                             <div className="flex justify-between items-center">
                                 <label className="flex items-center gap-2 cursor-pointer">
                                     <input type="checkbox" checked={zoneForm.active} onChange={()=>setZoneForm(p=>({...p, active: !p.active}))} className="rounded text-sky-600 focus:ring-sky-500" />
                                     <span className="text-sm font-medium text-slate-700">Zone Enabled</span>
                                 </label>
                                 <div className="flex gap-2">
                                     <button onClick={() => setZoneFormOpen(false)} className="px-3 py-1.5 text-slate-500 hover:bg-slate-200 rounded-lg text-sm">Cancel</button>
                                     <button 
                                        onClick={async () => {
                                            if (!zoneForm.name.trim()) return toast?.error?.("Name required");
                                            const payload = { name: zoneForm.name, category: zoneForm.category, geometry: zoneForm.geometry || "Polygon", active: zoneForm.active };
                                            // Mock save logic
                                            setZones(p => [payload, ...p]);
                                            setZoneFormOpen(false);
                                            setZoneForm({name:"", category:"Safe", geometry:"", active:true});
                                        }}
                                        className="px-4 py-1.5 bg-sky-600 text-white rounded-lg text-sm font-medium hover:bg-sky-700"
                                     >
                                        Save
                                     </button>
                                 </div>
                             </div>
                        </div>
                    )}

                    <div className="border border-slate-200 rounded-xl overflow-hidden shadow-sm">
                        <div className="bg-slate-50 px-4 py-3 border-b border-slate-200 flex justify-between items-center">
                            <h4 className="font-bold text-slate-700">Zones for {selectedMap.name}</h4>
                            <span className="text-xs text-slate-500">{zones.length} items</span>
                        </div>
                        <table className="w-full text-sm">
                            <thead className="bg-white text-slate-500 border-b border-slate-100">
                                <tr>
                                    <th className="px-4 py-3 text-left font-medium">Name</th>
                                    <th className="px-4 py-3 text-left font-medium">Type</th>
                                    <th className="px-4 py-3 text-center font-medium">Status</th>
                                    <th className="px-4 py-3 text-right font-medium">Created</th>
                                </tr>
                            </thead>
                            <tbody className="divide-y divide-slate-100">
                                {filterBySearch(zones, zoneSearchTerm, zoneSearchField, {any:"name", name:"name", category:"category"}).map((z, i) => (
                                    <tr key={i} className="hover:bg-slate-50">
                                        <td className="px-4 py-3 font-medium">{z.name}</td>
                                        <td className="px-4 py-3">{z.category}</td>
                                        <td className="px-4 py-3 text-center">
                                            <span className={`px-2 py-0.5 rounded-full text-xs font-medium ${z.active ? 'bg-emerald-100 text-emerald-700' : 'bg-slate-100 text-slate-500'}`}>
                                                {z.active ? 'Active' : 'Disabled'}
                                            </span>
                                        </td>
                                        <td className="px-4 py-3 text-right text-slate-500">{z.createdAt || '-'}</td>
                                    </tr>
                                ))}
                                {zones.length === 0 && (
                                    <tr><td colSpan={4} className="px-4 py-8 text-center text-slate-400">No zones found</td></tr>
                                )}
                            </tbody>
                        </table>
                    </div>
                 </>
             )}
          </div>
        )}

        {/* WAYPOINTS, MISSIONS, USERS follow similar patterns... */}
        {/* Simplified logic for brevity, using same styling components */}

        {/* USERS EXAMPLE (demonstrating conditional form) */}
        {rightPage === "users" && (
             <div className="space-y-4">
                 <div className="flex gap-2">
                     <input className="flex-1 px-3 py-2.5 border border-slate-200 rounded-lg text-sm" value={userSearchTerm} onChange={e=>setUserSearchTerm(e.target.value)} placeholder="Search users..." />
                     <button onClick={loadUsers} className="px-4 py-2.5 bg-slate-100 text-slate-600 font-medium rounded-lg hover:bg-slate-200 text-sm">Refresh</button>
                 </div>

                 {editingUserId && (
                     <div className="bg-white border-2 border-sky-100 rounded-xl p-4 shadow-sm">
                         <div className="flex justify-between items-center mb-4">
                             <h3 className="font-bold text-slate-800">Edit User</h3>
                             <button onClick={()=>setEditingUserId(null)}><FaTimes className="text-slate-400"/></button>
                         </div>
                         <div className="grid grid-cols-2 gap-3 mb-4">
                             <input value={editUserForm.username} onChange={e=>setEditUserForm(p=>({...p, username:e.target.value}))} className="px-3 py-2 border rounded-lg text-sm" placeholder="Username" />
                             <input value={editUserForm.email} onChange={e=>setEditUserForm(p=>({...p, email:e.target.value}))} className="px-3 py-2 border rounded-lg text-sm" placeholder="Email" />
                             {/* ... other fields ... */}
                         </div>
                         <div className="flex justify-end gap-2">
                             <button onClick={()=>setEditingUserId(null)} className="px-3 py-1.5 text-slate-500 text-sm">Cancel</button>
                             <button onClick={()=>{/* save logic */ setEditingUserId(null)}} className="px-4 py-1.5 bg-sky-600 text-white rounded-lg text-sm">Save</button>
                         </div>
                     </div>
                 )}

                 <div className="rounded-xl border border-slate-200 overflow-hidden shadow-sm">
                    <table className="w-full text-sm">
                        <thead className="bg-slate-50 text-slate-500 font-medium border-b border-slate-100">
                            <tr>
                                <th className="px-4 py-3 text-left">User</th>
                                <th className="px-4 py-3 text-left">Role</th>
                                <th className="px-4 py-3 text-right">Action</th>
                            </tr>
                        </thead>
                        <tbody className="divide-y divide-slate-100">
                            {filterBySearch(users, userSearchTerm, "any", {any:"username"}).map(u => (
                                <tr key={u.id} className="hover:bg-slate-50">
                                    <td className="px-4 py-3">
                                        <div className="font-medium text-slate-900">{u.username}</div>
                                        <div className="text-xs text-slate-500">{u.email}</div>
                                    </td>
                                    <td className="px-4 py-3">
                                        <span className={`px-2 py-0.5 rounded-full text-xs font-medium ${u.role === 'admin' ? 'bg-indigo-100 text-indigo-700' : 'bg-slate-100 text-slate-600'}`}>
                                            {u.role}
                                        </span>
                                    </td>
                                    <td className="px-4 py-3 text-right">
                                        <button onClick={()=>{setEditingUserId(u.id); setEditUserForm(u);}} className="text-sky-600 hover:text-sky-800 text-xs font-bold uppercase tracking-wider">Edit</button>
                                    </td>
                                </tr>
                            ))}
                        </tbody>
                    </table>
                 </div>
             </div>
        )}

        {/* ANALYTICS */}
        {rightPage === "analytics" && (
            <div className="space-y-6">
                <div className="grid grid-cols-2 lg:grid-cols-4 gap-4">
                    {analyticsSummary.map((card, i) => (
                        <div key={i} className="p-4 rounded-xl bg-slate-50 border border-slate-100">
                            <span className="text-xs font-bold text-slate-400 uppercase tracking-wider">{card.label}</span>
                            <div className="text-2xl font-bold text-slate-800 mt-1">{card.value}</div>
                            <div className="text-xs text-slate-500 mt-1">{card.trend}</div>
                        </div>
                    ))}
                </div>
                <div className="bg-white p-6 rounded-2xl shadow-sm border border-slate-100">
                    <h4 className="font-bold text-slate-800 mb-1">Cycle Throughput</h4>
                    <p className="text-sm text-slate-500 mb-4">Rolling seven-day view</p>
                    <div className="w-full overflow-hidden">
                        <svg width="100%" height={200} viewBox={`0 0 ${analyticsChartSize.width} ${analyticsChartSize.height}`} className="w-full">
                            <polyline points={analyticsPath} fill="none" stroke="#6366f1" strokeWidth="3" vectorEffect="non-scaling-stroke"/>
                        </svg>
                    </div>
                </div>
            </div>
        )}

        {/* CHAT */}
        {rightPage === "chat" && (
            <div className="flex flex-col h-full bg-white">
                <div className="flex-none p-4 bg-slate-50 border-b border-slate-100">
                    <div className="flex justify-between items-start">
                        <div>
                            <h2 className="text-lg font-bold text-slate-800">Assistant Link</h2>
                            <p className="text-xs text-slate-500 mt-0.5">Last sync at {latestMessage ? formatTimestamp(latestMessage.timestamp) : "--:--"}</p>
                        </div>
                        <span className="px-2 py-1 bg-emerald-500 text-white text-xs font-bold rounded-full">Online</span>
                    </div>
                </div>

                <div className="flex-1 overflow-y-auto p-4 space-y-4" ref={chatContainerRef}>
                    {chatMessages.map(msg => {
                        const isRobot = msg.sender === "robot";
                        return (
                            <div key={msg.id} className={`flex w-full ${isRobot ? 'justify-start' : 'justify-end'}`}>
                                <div className={`max-w-[80%] p-3 rounded-2xl text-sm shadow-sm ${isRobot ? 'bg-slate-100 text-slate-800 rounded-bl-sm' : 'bg-sky-600 text-white rounded-br-sm'}`}>
                                    <p>{msg.text}</p>
                                    <div className={`text-[10px] mt-1 opacity-70 ${isRobot ? 'text-slate-500' : 'text-sky-100 text-right'}`}>
                                        {formatTimestamp(msg.timestamp)}
                                    </div>
                                </div>
                            </div>
                        )
                    })}
                    {isTyping && (
                         <div className="flex justify-start">
                             <div className="bg-slate-100 p-3 rounded-2xl rounded-bl-sm flex gap-1">
                                 <div className="w-2 h-2 bg-slate-400 rounded-full animate-bounce"></div>
                                 <div className="w-2 h-2 bg-slate-400 rounded-full animate-bounce delay-75"></div>
                                 <div className="w-2 h-2 bg-slate-400 rounded-full animate-bounce delay-150"></div>
                             </div>
                         </div>
                    )}
                </div>

                <div className="flex-none p-4 border-t border-slate-100 bg-white">
                    <div className="flex gap-2 overflow-x-auto pb-2 mb-2">
                        {chatQuickPrompts.map(p => (
                            <button key={p} onClick={()=>handleSuggestionClick(p)} className="flex-none px-3 py-1 bg-slate-50 border border-slate-200 rounded-full text-xs text-slate-600 hover:bg-sky-50 hover:text-sky-600 hover:border-sky-200 transition">
                                {p}
                            </button>
                        ))}
                    </div>
                    <div className="flex gap-2 items-end">
                        <textarea 
                            value={chatInput} 
                            onChange={e=>setChatInput(e.target.value)} 
                            onKeyPress={handleKeyPress}
                            placeholder="Type a message..." 
                            className="flex-1 bg-slate-50 border border-slate-200 rounded-xl px-4 py-3 text-sm focus:ring-2 focus:ring-sky-500 focus:outline-none resize-none max-h-32" 
                            rows={1}
                        />
                        <button 
                            onClick={handleMicClick}
                            className={`p-3 rounded-xl transition-colors ${isRecording ? 'bg-rose-500 text-white animate-pulse' : 'bg-slate-100 text-slate-500 hover:bg-slate-200'}`}
                        >
                            <FaMicrophone />
                        </button>
                        <button 
                            onClick={()=>handleSendMessage()} 
                            disabled={!chatInput.trim()}
                            className="p-3 bg-sky-600 text-white rounded-xl hover:bg-sky-700 disabled:bg-slate-200 disabled:text-slate-400 transition-colors shadow-sm"
                        >
                            <FaPaperPlane />
                        </button>
                    </div>
                </div>
            </div>
        )}

        {/* Stats Component Injection */}
        {rightPage === "stats" && (<Stats {...props} />)}
        
        {/* SETTINGS / GENERIC */}
        {["robot-settings", "security", "integrations"].includes(rightPage) && (
             <div className="space-y-4">
                 {/* Example generic list for settings/toggles */}
                 {rightPage === "robot-settings" && Object.entries(robotSettingsState).map(([k, v]) => (
                     <div key={k} className="flex items-center justify-between p-4 border border-slate-200 rounded-xl bg-slate-50">
                         <div>
                             <h4 className="font-semibold text-slate-800 capitalize">{k.replace(/([A-Z])/g, " $1")}</h4>
                             <p className="text-xs text-slate-500">{v ? "Enabled" : "Disabled"}</p>
                         </div>
                         <div 
                            onClick={()=>toggleRobotSetting(k)}
                            className={`w-12 h-6 rounded-full cursor-pointer transition-colors relative ${v ? 'bg-sky-500' : 'bg-slate-300'}`}
                         >
                             <div className={`w-4 h-4 bg-white rounded-full absolute top-1 transition-all ${v ? 'left-7' : 'left-1'}`} />
                         </div>
                     </div>
                 ))}
             </div>
        )}
      </div>
    </aside>
  );
};

export default RightPane;