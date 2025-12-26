// import React, { useState, useRef } from "react";
// // import 
// // {
// //     FaEdit,
// // }
// /* ===============================
//    INITIAL DATA
// ================================ */

// const initialMaps = [
//   {
//     id: "cfl_gf",
//     name: "CFL_GF",
//     createdBy: "CNDE IITM",
//     image: "",
//     status: "Active",
//   },
//   {
//     id: "shobot_arena",
//     name: "shobot_arena",
//     createdBy: "ANSCER ADMIN",
//     image: "/images/maps/shobot_arena.png",
//     status: "",
//   },
//   {
//     id: "shobot_arena2",
//     name: "shobot_arena2",
//     createdBy: "ANSCER ADMIN",
//     image: "/images/maps/shobot_arena2.png",
//     status: "",
//   },
// ];

// /* ===============================
//    COMPONENT
// ================================ */

// const Maps = (props) => {
//   const {
//     requestV1,
//     toast,
//     waypoints,
//     setWaypoints,
//     zones,
//     setZones,
//     missions,
//     setMissions,
//   } = props;

//   /* ===============================
//      STATE
//   ================================ */

//   const [mapsList, setMapsList] = useState(initialMaps);
//   const [selectedMap, setSelectedMap] = useState(initialMaps[0]);

//   const [mapSearchField, setMapSearchField] = useState("any");
//   const [mapSearchTerm, setMapSearchTerm] = useState("");

//   const [mapModalOpen, setMapModalOpen] = useState(false);
//   const [mapModalMode, setMapModalMode] = useState("preview"); // preview | edit | create
//   const [mapModalMap, setMapModalMap] = useState(null);

//   const [mapForm, setMapForm] = useState({
//     id: null,
//     name: "",
//     createdBy: "",
//     image: "",
//     status: "",
//     category: "",
//     createdAt: "",
//   });

//   const mapImageInputRef = useRef(null);

//   /* ===============================
//      MODAL HELPERS
//   ================================ */

//   const closeMapModal = () => {
//     setMapModalOpen(false);
//     setMapModalMap(null);
//     setMapForm({
//       id: null,
//       name: "",
//       createdBy: "",
//       image: "",
//       status: "",
//       category: "",
//       createdAt: "",
//     });
//   };

//   /* ===============================
//      IMAGE HANDLING
//   ================================ */

//   const handleMapImageChange = (e) => {
//     const file = e.target.files?.[0];
//     if (!file) return;

//     const validTypes = [
//       "image/png",
//       "image/jpeg",
//       "image/jpg",
//       "image/gif",
//       "image/webp",
//     ];

//     if (!validTypes.includes(file.type)) {
//       toast?.error?.("Invalid image type");
//       return;
//     }

//     if (file.size > 5 * 1024 * 1024) {
//       toast?.error?.("Max file size is 5MB");
//       return;
//     }

//     const reader = new FileReader();
//     reader.onload = () => {
//       setMapForm((prev) => ({ ...prev, image: reader.result }));
//     };
//     reader.readAsDataURL(file);
//   };

//   const triggerMapImagePicker = (e) => {
//     e?.preventDefault();
//     e?.stopPropagation();

//     if (mapImageInputRef.current) {
//       mapImageInputRef.current.value = "";
//       mapImageInputRef.current.click();
//     } else {
//       toast?.error?.("File picker unavailable");
//     }
//   };

//   /* ===============================
//      CRUD ACTIONS
//   ================================ */

//   const handleMapAction = async (action, map) => {
//     if (!map) return;

//     if (action === "edit") {
//       setMapModalMode("edit");
//       setMapModalMap(map);
//       setMapForm(map);
//       setMapModalOpen(true);
//       return;
//     }

//     if (action === "delete") {
//       const ok = window.confirm(`Delete map "${map.name}"?`);
//       if (!ok) return;

//       try {
//         await requestV1(`/maps/${map.id}`, { method: "DELETE" });
//       } catch (err) {
//         console.warn("Delete API failed, local delete");
//       }

//       setMapsList((prev) => prev.filter((m) => m.id !== map.id));
//       if (selectedMap?.id === map.id) setSelectedMap(null);
//       toast?.success?.("Map deleted");
//     }
//   };

//   const createNewMapImmediate = () => {
//     const newMap = {
//       id: `local_${Date.now()}`,
//       name: `New Map ${mapsList.length + 1}`,
//       createdBy: "Operator",
//       image: "",
//       status: "",
//       createdAt: new Date().toISOString().slice(0, 10),
//     };

//     setMapsList((prev) => [newMap, ...prev]);
//     setSelectedMap(newMap);
//     toast?.success?.("Map added");
//   };

//   /* ===============================
//      ACTIVATE MAP
//   ================================ */

//   const handleActivateMap = (map) => {
//     if (!map) return;

//     setMapsList((prev) =>
//       prev.map((m) =>
//         m.id === map.id
//           ? { ...m, status: "Active" }
//           : { ...m, status: "" }
//       )
//     );

//     setSelectedMap({ ...map, status: "Active" });

//     toast?.success?.(`Activated map: ${map.name}`);
//   };

//   /* ===============================
//      SAVE MAP
//   ================================ */

//   const saveMapFromForm = async () => {
//     if (!mapForm.name.trim()) {
//       toast?.error?.("Map name required");
//       return;
//     }

//     const payload = {
//       name: mapForm.name.trim(),
//       createdBy: mapForm.createdBy,
//       image: mapForm.image,
//       status: mapForm.status,
//       category: mapForm.category,
//       createdAt: mapForm.createdAt,
//     };

//     try {
//       if (mapModalMode === "create") {
//         const created = { ...payload, id: `local_${Date.now()}` };
//         setMapsList((prev) => [created, ...prev]);
//         setSelectedMap(created);
//         toast?.success?.("Map created");
//       }

//       if (mapModalMode === "edit") {
//         const id = mapForm.id;
//         setMapsList((prev) =>
//           prev.map((m) => (m.id === id ? { ...m, ...payload } : m))
//         );
//         toast?.success?.("Map updated");
//       }

//       closeMapModal();
//     } catch (err) {
//       toast?.error?.("Failed to save map");
//     }
//   };

//   /* ===============================
//      SEARCH FILTER
//   ================================ */

//   const filterBySearch = (items) => {
//     if (!mapSearchTerm.trim()) return items;
//     const term = mapSearchTerm.toLowerCase();

//     return items.filter((m) =>
//       Object.values(m).some((v) =>
//         String(v || "").toLowerCase().includes(term)
//       )
//     );
//   };

//   /* ===============================
//      RENDER (LOGIC ONLY)
//   ================================ */

//   return (
//     <div>
//       {/* UI intentionally omitted — logic cleaned */}
//       <input
//         type="file"
//         ref={mapImageInputRef}
//         hidden
//         accept="image/*"
//         onChange={handleMapImageChange}
//       />
//     </div>
//   );
// };

// export default Maps;


// logic

import React from "react";
import React from "react";
import { useState } from "react";
import { FaPlus } from "react-icons/fa";


  // sample maps data — replace image paths with your real map images
  const initialMaps = [
    {
      id: "cfl_gf",
      name: "CFL_GF",
      createdBy: "CNDE IITM",
      // removed local screenshot reference
      image: "",
      status: "Active",
    },
    {
      id: "shobot_arena",
      name: "shobot_arena",
      createdBy: "ANSCER ADMIN",
      image: "/images/maps/shobot_arena.png",
      status: "",
    },
    {
      id: "shobot_arena2",
      name: "shobot_arena2",
      createdBy: "ANSCER ADMIN",
      image: "/images/maps/shobot_arena2.png",
      status: "",
    },
    /* zones entry: cleared local screenshot */
    {
      id: "zones",
      name: "Zones",
      createdBy: "ANSCER ADMIN",
      image: "",
      status: "",
    },
    /* waypoints entry: cleared local screenshot */
    {
      id: "waypoints",
      name: "Waypoints",
      createdBy: "ANSCER ADMIN",
      image: "",
      status: "",
    },
    /* users preview cleared */
    {
      id: "users",
      name: "Users",
      createdBy: "ANSCER ADMIN",
      image: "",
      status: "",
    },
  ];
  const [mapsList, setMapsList] = useState(initialMaps);
  const [selectedMap, setSelectedMap] = useState(initialMaps[0]);

    // unified map search (single dropdown selects field, single input is the query)
  const [mapSearchField, setMapSearchField] = useState("any"); // any,name,createdBy,category,createdAt,status
  const [mapSearchTerm, setMapSearchTerm] = useState("");

  
  // Map modal state for preview / edit / create
  const [mapModalOpen, setMapModalOpen] = useState(false);
  const [mapModalMode, setMapModalMode] = useState("preview"); // "preview" | "edit" | "create"
  const [mapModalMap, setMapModalMap] = useState(null);

    // form used for create/edit
  const [mapForm, setMapForm] = useState({
    id: null,
    name: "",
    createdBy: "",
    image: "",
    status: "",
    category: "",
    createdAt: "",
  });

  // Handle image file selection for map modal — read as data URL and store in mapForm.image
  const handleMapImageChange = (e) => {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      setMapForm((prev) => ({ ...prev, image: reader.result || "" }));
    };
    reader.readAsDataURL(file);
  };
  
  const triggerMapImagePicker = () => {
    try {
      if (mapImageInputRef.current) mapImageInputRef.current.click();
    } catch (err) {
      console.warn("Image picker failed", err);
    }
  };

  const handleMapAction = async (action, map) => {
    try {
      if (action === "edit") {
        openMapModal("edit", map);
        return;
      }
      if (action === "delete") {
        const ok = window.confirm(
          `Delete map "${map?.name || map?.id}"? This cannot be undone.`,
        );
        if (!ok) return;

        // Try server delete first; fallback to local removal on failure
        try {
          await requestV1(`/maps/${map.id}`, { method: "DELETE" });
          setMapsList((prev) => prev.filter((m) => m.id !== map.id));
          if (selectedMap && selectedMap.id === map.id) setSelectedMap(null);
          toast.success("Map deleted");
        } catch (err) {
          console.warn("Delete API failed, falling back to local remove", err);
          // fallback local behavior
          setMapsList((prev) => prev.filter((m) => m.id !== map.id));
          if (selectedMap && selectedMap.id === map.id) setSelectedMap(null);
          toast.success("Map removed (local)");
        }
        return;
      }
    } catch (err) {
      console.error("Map action error", err);
      toast.error(err.message || "Map action failed");
    }
  };

  // Create a new map immediately (optimistic local create with server persist attempt)
  const createNewMapImmediate = async () => {
    const newMap = {
      id: `local_${Date.now()}`,
      name: `New Map ${mapsList.length + 1}`,
      createdBy: "Operator",
      image: "",
      status: "Inactive",
      category: "",
      createdAt: new Date().toISOString().slice(0, 10),
    };
    // optimistic UI update
    setMapsList((prev) => [newMap, ...prev]);
    setSelectedMap(newMap);
    toast.success("Map added");

    // try persist to server, replace local id with server item on success
    try {
      const res = await requestV1("/maps", {
        method: "POST",
        body: JSON.stringify({
          name: newMap.name,
          createdBy: newMap.createdBy,
          image: newMap.image,
          status: newMap.status,
          category: newMap.category,
          createdAt: newMap.createdAt,
        }),
      });
      const created = res.item || { ...newMap, id: res.id || newMap.id };
      setMapsList((prev) => prev.map((m) => (m.id === newMap.id ? created : m)));
      setSelectedMap(created);
      toast.success("Map persisted");
    } catch (err) {
      console.warn("Persist new map failed, kept local map", err);
      toast.error("Map added locally (server persist failed)");
    }
  };

  // Activate a map (radio). Optimistic UI update, try to persist via API.
  const handleActivateMap = async (map) => {
    if (!map) return;
    try {
      const mapId = map.id;
      
      console.log(`🗺️ Activating map: ${mapId}`);
      
      // Optimistically update UI: mark chosen map Active, clear previous active status
      setMapsList((prev) =>
        prev.map((m) => {
          if (m.id === map.id) return { ...m, status: "Active" };
          if ((m.status || "").toLowerCase() === "active" && m.id !== map.id) return { ...m, status: "" };
          return m;
        }),
      );
      
      // Set selected map
      setSelectedMap({ ...map, status: "Active" });
      
      // Check if this map already has waypoints/zones/missions
      const hasWaypoints = waypoints.some(wp => wp.mapId === mapId);
      const hasZones = zones.some(z => z.mapId === mapId);
      const hasMissions = missions.some(m => m.mapId === mapId);
      
      console.log(`📍 Existing data - Waypoints: ${hasWaypoints}, Zones: ${hasZones}, Missions: ${hasMissions}`);
      
      // Create and set waypoints immediately
      if (!hasWaypoints) {
        const defaultWaypoints = [
          {
            id: `wp-${mapId}-delivery`,
            mapId: mapId,
            name: "Delivery",
            category: "Normal",
            geom: "Point(10 10)",
            notes: `Delivery waypoint for ${map.name}`,
            active: true,
            createdAt: new Date().toLocaleString(),
          },
          {
            id: `wp-${mapId}-shelf1`,
            mapId: mapId,
            name: "Shelf_1",
            category: "Normal",
            geom: "Point(50 30)",
            notes: `Shelf 1 waypoint for ${map.name}`,
            active: true,
            createdAt: new Date().toLocaleString(),
          },
          {
            id: `wp-${mapId}-shelf2`,
            mapId: mapId,
            name: "Shelf_2",
            category: "Normal",
            geom: "Point(80 50)",
            notes: `Shelf 2 waypoint for ${map.name}`,
            active: true,
            createdAt: new Date().toLocaleString(),
          },
          {
            id: `wp-${mapId}-shelf3`,
            mapId: mapId,
            name: "Shelf_3",
            category: "Normal",
            geom: "Point(20 70)",
            notes: `Shelf 3 waypoint for ${map.name}`,
            active: true,
            createdAt: new Date().toLocaleString(),
          },
          {
            id: `wp-${mapId}-shelf4`,
            mapId: mapId,
            name: "Shelf_4",
            category: "Normal",
            geom: "Point(60 80)",
            notes: `Shelf 4 waypoint for ${map.name}`,
            active: true,
            createdAt: new Date().toLocaleString(),
          },
        ];
        console.log(`✅ Creating ${defaultWaypoints.length} waypoints for map ${mapId}`);
        setWaypoints(prev => [...prev, ...defaultWaypoints]);
      }
      
      // Create and set zones immediately
      if (!hasZones) {
        const defaultZones = [
          {
            id: `zone-${mapId}-1`,
            mapId: mapId,
            name: `${map.name} - Safe Zone`,
            category: "Safe",
            geometry: "Polygon((0,0),(100,0),(100,50),(0,50))",
            active: true,
            createdAt: new Date().toLocaleString(),
          },
          {
            id: `zone-${mapId}-2`,
            mapId: mapId,
            name: `${map.name} - Caution Area`,
            category: "Caution",
            geometry: "Polygon((0,50),(50,50),(50,100),(0,100))",
            active: true,
            createdAt: new Date().toLocaleString(),
          },
          {
            id: `zone-${mapId}-3`,
            mapId: mapId,
            name: `${map.name} - Restricted`,
            category: "No-Go",
            geometry: "Polygon((50,50),(100,50),(100,100),(50,100))",
            active: true,
            createdAt: new Date().toLocaleString(),
          },
        ];
        console.log(`✅ Creating ${defaultZones.length} zones for map ${mapId}`);
        setZones(prev => [...prev, ...defaultZones]);
      }
      
      // Create and set missions immediately
      if (!hasMissions) {
        const defaultMissions = [
          {
            id: `mission-${mapId}-delivery`,
            mapId: mapId,
            name: "Delivery",
            owner: "CNDE",
            status: "Draft",
            notes: `Delivery mission for ${map.name}`,
            createdAt: new Date().toLocaleString(),
            iteration: 1,
          },
          {
            id: `mission-${mapId}-shelf1`,
            mapId: mapId,
            name: "Shelf_1",
            owner: "CNDE",
            status: "Draft",
            notes: `Shelf 1 mission for ${map.name}`,
            createdAt: new Date().toLocaleString(),
            iteration: 1,
          },
          {
            id: `mission-${mapId}-shelf2`,
            mapId: mapId,
            name: "Shelf_2",
            owner: "CNDE",
            status: "Draft",
            notes: `Shelf 2 mission for ${map.name}`,
            createdAt: new Date().toLocaleString(),
            iteration: 1,
          },
          {
            id: `mission-${mapId}-shelf3`,
            mapId: mapId,
            name: "Shelf_3",
            owner: "CNDE",
            status: "Draft",
            notes: `Shelf 3 mission for ${map.name}`,
            createdAt: new Date().toLocaleString(),
            iteration: 1,
          },
          {
            id: `mission-${mapId}-shelf4`,
            mapId: mapId,
            name: "Shelf_4",
            owner: "CNDE",
            status: "Draft",
            notes: `Shelf 4 mission for ${map.name}`,
            createdAt: new Date().toLocaleString(),
            iteration: 1,
          },
        ];
        console.log(`✅ Creating ${defaultMissions.length} missions for map ${mapId}`);
        setMissions(prev => [...prev, ...defaultMissions]);
      }
      
      // Show success toasts
      if (!hasWaypoints) {
        toast.success(`Created 5 waypoints for ${map.name}`);
      }
      if (!hasZones) {
        toast.success(`Created 3 zones for ${map.name}`);
      }
      if (!hasMissions) {
        toast.success(`Created 5 missions for ${map.name}`);
      }
      toast.success(`Activated map: ${map.name}`);

      // Persist change to server (best-effort)
      try {
        await requestV1(`/maps/${map.id}`, {
          method: "PATCH",
          body: JSON.stringify({ status: "Active" }),
        });
      } catch (err) {
        console.warn("Failed to persist map activation:", err);
      }
    } catch (err) {
      console.error("Activation error", err);
      toast.error("Failed to activate map");
    }
  };
 
  const saveMapFromForm = async () => {
    // basic validation
    if (!mapForm.name.trim()) {
      toast.error("Map name required");
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
      if (mapModalMode === "create") {
        // POST /maps
        try {
          const res = await requestV1("/maps", {
            method: "POST",
            body: JSON.stringify(payload),
          });
          const created = res.item || { ...payload, id: res.id || `map_${Date.now()}` };
          setMapsList((prev) => [created, ...prev]);
          setSelectedMap(created);
          toast.success("Map created");
        } catch (err) {
          // fallback local creation
          const created = { ...payload, id: `local_${Date.now()}` };
          setMapsList((prev) => [created, ...prev]);
          setSelectedMap(created);
          toast.success("Map created (local)");
        }
      } else if (mapModalMode === "edit") {
        const id = mapForm.id;
        try {
          const res = await requestV1(`/maps/${id}`, {
            method: "PATCH",
            body: JSON.stringify(payload),
          });
          const updated = res.item || { ...payload, id };
          setMapsList((prev) => prev.map((m) => (m.id === id ? { ...m, ...updated } : m)));
          if (selectedMap && selectedMap.id === id) setSelectedMap((s) => ({ ...s, ...updated }));
          toast.success("Map updated");
        } catch (err) {
          console.warn("Edit API failed, applying local update", err);
          setMapsList((prev) => prev.map((m) => (m.id === id ? { ...m, ...payload } : m)));
          if (selectedMap && selectedMap.id === id) setSelectedMap((s) => ({ ...s, ...payload }));
          toast.success("Map updated (local)");
        }
      }
      closeMapModal();
    } catch (err) {
      console.error("Save map error", err);
      toast.error(err.message || "Failed to save map");
    }
  };

  // active action radio state for the map action buttons
  const [activeMapAction, setActiveMapAction] = useState(null);

        const processMaps = (mapsRes) => {
  if (Array.isArray(mapsRes.items) && mapsRes.items.length) {
    // Normalize active status: ensure exactly one Active map locally
    const items = mapsRes.items.slice();
    const firstActiveIdx = items.findIndex(
      (m) => (m.status || "").toLowerCase() === "active"
    );

    let normalized;
    if (firstActiveIdx === -1) {
      // no active => mark first as Active
      normalized = items.map((m, idx) =>
        idx === 0
          ? { ...m, status: "Active" }
          : { ...m, status: m.status || "" }
      );
    } else {
      // multiple active possible — keep firstActiveIdx active and clear others
      normalized = items.map((m, idx) =>
        idx === firstActiveIdx
          ? { ...m, status: "Active" }
          : {
              ...m,
              status:
                m.status && m.status.toLowerCase() === "active"
                  ? ""
                  : m.status || "",
            }
      );
    }

    setMapsList(normalized);

    // pick the active map as selected
    const activeMap =
      normalized.find(
        (m) => (m.status || "").toLowerCase() === "active"
      ) || normalized[0];

    setSelectedMap(activeMap || null);
  }
};

const processZones = (zonesRes) => {
  if (Array.isArray(zonesRes.items)) {
    setZones(zonesRes.items);
  }
};

const processWaypoints = (waypointsRes) => {
  if (Array.isArray(waypointsRes.items)) {
    // Ensure all waypoints have mapId from server
    const waypointsWithMapId = waypointsRes.items.map((wp) => ({
      ...wp,
      mapId: wp.mapId || "cfl_gf", // default to first map if missing
    }));
    setWaypoints(waypointsWithMapId);
  }
};

const processMissions = (missionsRes) => {
  if (Array.isArray(missionsRes.items)) {
    // Ensure all missions have mapId from server
    const missionsWithMapId = missionsRes.items.map((m) => ({
      ...m,
      mapId: m.mapId || "cfl_gf", // default to first map if missing
    }));
    setMissions(missionsWithMapId);
  }
};

const processUsers = (usersRes) => {
  if (Array.isArray(usersRes.items)) {
    setUsers(usersRes.items);
  }
};

  // Add this helper function reference (pass from MainPage)
  const getMapDisplayName = (mapId) => {
    if (!mapId) return "Unknown";
    // This will be passed as a prop from MainPage
    return mapId;
  };

 const closeMapModal = () => {
  setMapModalOpen(false);
  setMapModalMap(null);
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
        // Optimistic update
        setMapsList((prev) => prev.map((m) => (m.id === id ? { ...m, ...payload } : m)));
        if (props.selectedMap && props.selectedMap.id === id) {
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

  // Handle image file selection (VERIFY THIS EXISTS)
  const handleMapImageChange = (e) => {
    const file = e.target.files?.[0];
    if (!file) return;
    
    // Validate file type
    const validTypes = ['image/png', 'image/jpeg', 'image/jpg', 'image/gif', 'image/webp'];
    if (!validTypes.includes(file.type)) {
      toast?.error?.("Please select a valid image file (PNG, JPG, JPEG, GIF, WebP)");
      return;
    }
    
    // Validate file size (max 5MB)
    if (file.size > 5 * 1024 * 1024) {
      toast?.error?.("File size must be less than 5MB");
      return;
    }
    
    const reader = new FileReader();
    reader.onload = () => {
      setMapForm((prev) => ({ ...prev, image: reader.result || "" }));
      toast?.success?.("Image loaded successfully!");
    };
    reader.onerror = () => {
      toast?.error?.("Failed to read file");
    };
    reader.readAsDataURL(file);
  };

  const triggerMapImagePicker = (e) => {
    // Prevent any default behavior
    if (e) {
      e.preventDefault();
      e.stopPropagation();
    }
    
    try {
      // CRITICAL: Use the exact same ref name
      if (mapImageInputRef.current) {
        mapImageInputRef.current.value = ''; // Reset to allow re-selecting same file
        mapImageInputRef.current.click(); // Trigger the file picker
        console.log("✅ File picker triggered successfully");
      } else {
        console.error("❌ mapImageInputRef.current is NULL");
        toast?.error?.("File picker unavailable. Please refresh the page.");
      }
    } catch (err) {
      console.error("❌ Image picker failed:", err);
      toast?.error?.("Failed to open file picker");
    }
  };

  // Helper function for filtering by search term and field
  const filterBySearch = (items, searchTerm, searchField, fieldMap) => {
    if (!searchTerm.trim()) return items;
    
    const term = searchTerm.trim().toLowerCase();
    
    return items.filter((item) => {
      if (searchField === "any") {
        // Search across all fields
        return Object.values(fieldMap).some(
          (field) => String(item[field] || "").toLowerCase().includes(term)
        );
      }
      // Search in specific field
      const fieldName = fieldMap[searchField];
      return String(item[fieldName] || "").toLowerCase().includes(term);
    });
  };
// RightPane logic

  // Add map modal state locally
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

   const {
    rightPage,
    setRightPage,
    // maps
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

   } = props;

   return (
     
   );