import React, { useState, useRef, useMemo } from "react";
import { useEffect } from "react";
import { FaPlus, FaEdit, FaTrash } from "react-icons/fa";
import map1 from "../../../assets/map1.png";
import test1 from "../../../assets/test_1.png";
import shobotLogo from "../../../assets/shobot_arena.png";
import shobotLogo1 from "../../../assets/shobot_arena2.png";
import simulationMap from "../../../assets/simulation_map.png";
import trasccon4th from "../../../assets/trasccon4th.png";
import MapContext from "../../rightPane/create/MapContext.tsx";
import { createPortal } from "react-dom";
import MapManager from "./MapContext.tsx";

// Type definitions
interface Map {
  id: string;
  name: string;
  createdBy: string;
  image: string;
  status: string;
  category?: string;
  createdAt?: string;
}

interface MapsProps {
  rightPage: string;
  setRightPage: (page: string) => void;
  mapsList: Map[];
  selectedMap: Map | null;
  createNewMapImmediate: () => Promise<void>;
  handleActivateMap: (map: Map) => Promise<void>;
  handleMapAction: (action: string, map: Map) => Promise<void>;
  mapSearchField: string;
  setMapSearchField: (field: string) => void;
  mapSearchTerm: string;
  setMapSearchTerm: (term: string) => void;
  requestV1: (url: string, options?: any) => Promise<any>;
  toast: {
    success: (msg: string) => void;
    error: (msg: string) => void;
  };
  setMapsList: React.Dispatch<React.SetStateAction<Map[]>>;
  setSelectedMap: React.Dispatch<React.SetStateAction<Map | null>>;
}

const Maps: React.FC<MapsProps> = (props) => {
  const {
    rightPage,
    mapsList,
    selectedMap,
    createNewMapImmediate,
    handleActivateMap,
    handleMapAction,
    mapSearchField,
    setMapSearchField,
    mapSearchTerm,
    setMapSearchTerm,
    toast,
    setMapsList,
    setSelectedMap,
  } = props;

  // Modal state
  const [mapModalOpen, setMapModalOpen] = useState(false);
  const [mapModalMode, setMapModalMode] = useState<"preview" | "edit" | "create">("preview");
  const [mapForm, setMapForm] = useState<Partial<Map>>({
    id: undefined,
    name: "",
    createdBy: "",
    image: "",
    status: "",
    category: "",
    createdAt: "",
  });

  const mapImageInputRef = useRef<HTMLInputElement>(null);

  // Open map modal for edit
  const openMapModal = (mode: "preview" | "edit" | "create", map?: Map) => {
    setMapModalMode(mode);
    if (map) {
      setMapForm({
        id: map.id,
        name: map.name,
        createdBy: map.createdBy,
        image: map.image,
        status: map.status,
        category: map.category || "",
        createdAt: map.createdAt || "",
      });
    }
    setMapModalOpen(true);
  };

  // Close map modal
  const closeMapModal = () => {
    setMapModalOpen(false);
    setMapForm({
      id: undefined,
      name: "",
      createdBy: "",
      image: "",
      status: "",
      category: "",
      createdAt: "",
    });
  };

  // Handle image file selection
  const handleMapImageChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    // Validate file type
    const validTypes = ["image/png", "image/jpeg", "image/jpg", "image/gif", "image/webp"];
    if (!validTypes.includes(file.type)) {
      toast?.error("Please select a valid image file (PNG, JPG, JPEG, GIF, WebP)");
      return;
    }

    // Validate file size (max 5MB)
    if (file.size > 5 * 1024 * 1024) {
      toast?.error("File size must be less than 5MB");
      return;
    }

    const reader = new FileReader();
    reader.onload = () => {
      setMapForm((prev) => ({ ...prev, image: (reader.result as string) || "" }));
      toast?.success("Image loaded successfully!");
    };
    reader.onerror = () => {
      toast?.error("Failed to read file");
    };
    reader.readAsDataURL(file);
  };

  // Trigger file picker
  const triggerMapImagePicker = (e?: React.MouseEvent) => {
    if (e) {
      e.preventDefault();
      e.stopPropagation();
    }

    try {
      if (mapImageInputRef.current) {
        mapImageInputRef.current.value = "";
        mapImageInputRef.current.click();
        console.log("✅ File picker triggered successfully");
      } else {
        console.error("❌ mapImageInputRef.current is NULL");
        toast?.error("File picker unavailable. Please refresh the page.");
      }
    } catch (err) {
      console.error("❌ Image picker failed:", err);
      toast?.error("Failed to open file picker");
    }
  };

  // Save map from form
  const saveMapFromForm = async () => {
    if (!mapForm.name?.trim()) {
      toast?.error("Map name required");
      return;
    }

    const payload = {
      name: mapForm.name.trim(),
      createdBy: mapForm.createdBy || "",
      image: mapForm.image || "",
      status: mapForm.status || "",
      category: mapForm.category || "",
      createdAt: mapForm.createdAt || "",
    };

    try {
      if (mapModalMode === "edit" && mapForm.id) {
        const id = mapForm.id;
        // Optimistic update
        setMapsList((prev) =>
          prev.map((m) => (m.id === id ? { ...m, ...payload } : m))
        );
        if (selectedMap && selectedMap.id === id) {
          setSelectedMap((prev) => (prev ? { ...prev, ...payload } : null));
        }
        toast?.success("Map updated");
        closeMapModal();
      }
    } catch (err: any) {
      console.error("Save map error", err);
      toast?.error(err.message || "Failed to save map");
    }
  };

  // Aliases to normalize IDs coming from server/data
  const mapIdAliases: Record<string, string> = {
    shobot_area: "shobot_arena",
    trasccon: "trasccon4th",
  };
  const normalizeId = (id?: string) => mapIdAliases[String(id || "").toLowerCase()] || (id || "");

  // Initialize with imported map images (make stable via useMemo)
  const defaultMapsWithImages = useMemo(
    () => [
      {
        id: "shobot_arena",
        name: "Shobot Arena",
        createdBy: "Branding",
        image: shobotLogo,
        status: "Active",
        category: "Arena",
        createdAt: "2025-11-17",
      },
      {
        id: "shobot_arena2",
        name: "Shobot Arena 2",
        createdBy: "Branding",
        image: shobotLogo1,
        status: "",
        category: "Arena",
        createdAt: "2025-11-16",
      },
      {
        id: "simulation",
        name: "Simulation Map",
        createdBy: "Dev Team",
        image: simulationMap,
        status: "",
        category: "Simulation",
        createdAt: "2025-11-13",
      },
      // Fix id to match asset/use cases to avoid repeated re-render issues
      {
        id: "trasccon4th",
        name: "Trasccon 4th Floor",
        createdBy: "CNDE IITM",
        image: trasccon4th,
        status: "",
        category: "Production",
        createdAt: "2025-11-12",
      },
      {
        id: "map1",
        name: "Map 1",
        createdBy: "Assets",
        image: map1,
        status: "",
        category: "Demo",
        createdAt: "2025-11-10",
      },
      {
        id: "test_1",
        name: "Test 1",
        createdBy: "Assets",
        image: test1,
        status: "",
        category: "Demo",
        createdAt: "2025-11-09",
      },
    ],
    []
  );

  // Fallback lookups for known maps (id/name -> default image), memoized
  const defaultImageById = useMemo(
    () =>
      Object.fromEntries(
        defaultMapsWithImages.map((dm) => [String(dm.id || "").toLowerCase(), dm.image])
      ),
    [defaultMapsWithImages]
  );
  const defaultImageByName = useMemo(
    () =>
      Object.fromEntries(
        defaultMapsWithImages.map((dm) => [String(dm.name || "").toLowerCase(), dm.image])
      ),
    [defaultMapsWithImages]
  );
  const getFallbackImageByIdOrName = (id?: string, name?: string) => {
    const normId = normalizeId(id);
    return (
      defaultImageById[String(normId || "").toLowerCase()] ||
      defaultImageByName[String(name || "").toLowerCase()] ||
      ""
    );
  };

  // Helper to always provide normalized id + ensured image
  const buildPatchedMap = (m: Map): Map => {
    const normalizedId = normalizeId(m.id);
    const ensuredImage =
      (m.image || "").trim() || getFallbackImageByIdOrName(m.id, m.name);
    return { ...m, id: normalizedId, image: ensuredImage };
  };

  // Merge and backfill images using normalized IDs, memoized
 const combinedMaps = useMemo(() => {
  const validBackendMaps = (mapsList || [])
    .filter((m) => {
      const id = String(m.id || "").toLowerCase();
      const name = String(m.name || "").toLowerCase();
      if (!id) return false;
      if (id === "cfl_gf" || name === "cfl_gf") return false;
      return true;
    })
    .map(buildPatchedMap);

  const mapById = new Map<string, Map>();

  // 1️⃣ Insert backend maps first
  for (const m of validBackendMaps) {
    mapById.set(normalizeId(m.id).toLowerCase(), m);
  }

  // 2️⃣ Backfill defaults ONLY if missing
  for (const dm of defaultMapsWithImages) {
    const id = normalizeId(dm.id).toLowerCase();
    if (!mapById.has(id)) {
      mapById.set(id, dm);
    }
  }

  // 3️⃣ Force Shobot Arena to top
  const shobot = mapById.get("shobot_arena");
  const rest = Array.from(mapById.values()).filter(
    (m) => normalizeId(m.id) !== "shobot_arena"
  );

  return shobot ? [shobot, ...rest] : rest;
}, [mapsList, defaultMapsWithImages]);

useEffect(() => {
  if (combinedMaps.length === 0) return;

  const isValid =
    selectedMap &&
    combinedMaps.some(
      (m) => normalizeId(m.id) === normalizeId(selectedMap.id)
    );

  if (!isValid) {
    const shobot =
      combinedMaps.find((m) => normalizeId(m.id) === "shobot_arena") ||
      combinedMaps[0];

    setSelectedMap(shobot);
  }
}, [combinedMaps, selectedMap, setSelectedMap]);




  // Filter maps, memoized
  const filteredMaps = useMemo(() => {
    const term = (mapSearchTerm || "").trim().toLowerCase();
    const field = mapSearchField;
    return combinedMaps.filter((m) => {
      if (!term) return true;
      if (field === "any") {
        const haystack = `${m.name || ""} ${m.createdBy || ""} ${m.category || ""} ${m.createdAt || ""} ${m.status || ""}`.toLowerCase();
        return haystack.includes(term);
      }
      return String((m as any)[field] || "").toLowerCase().includes(term);
    });
  }, [combinedMaps, mapSearchField, mapSearchTerm]);

  const selectMap = async (map: Map) => {
    // Patch: normalize id and guarantee image, and activate with patched map
    const patched = buildPatchedMap(map);

    // Upsert selected map as Active; mark others Inactive
    setMapsList((prev) => {
      const selId = normalizeId(patched.id);
      let found = false;
      const updated = prev.map((m) => {
        const isSel = normalizeId(m.id) === selId;
        if (isSel) found = true;
        return isSel
          ? { ...m, status: "Active", image: (m.image || "").trim() ? m.image : patched.image }
          : { ...m, status: "Inactive" };
      });
      return found ? updated : [...updated, { ...patched, status: "Active" }];
    });
    setSelectedMap(patched);

    try {
      await handleActivateMap(patched);
    } catch (err: any) {
      console.error("Activate map error", err);
      toast?.error(err?.message || "Failed to activate map");
    }
  };

  if (rightPage !== "maps") return null;

  return (
    <div className="flex flex-col gap-2">
      {/* Hidden file input */}
      <input
        type="file"
        ref={mapImageInputRef}
        accept="image/*"
        className="hidden"
        onChange={handleMapImageChange}
      />

      {/* Search and Create Controls */}
      <div className="flex gap-2 flex-wrap items-center">
        <select
          aria-label="Search field"
          value={mapSearchField}
          onChange={(e) => setMapSearchField(e.target.value)}
          className="px-3 py-3 rounded-lg border border-gray-300 bg-white text-gray-900 focus:outline-none focus:ring-2 focus:ring-blue-500"
        >
          <option value="any">Search By</option>
          <option value="name">Name</option>
          <option value="createdBy">Created By</option>
          <option value="category">Category</option>
          <option value="createdAt">Created At</option>
          <option value="status">Status</option>
        </select>

        <input
          value={mapSearchTerm}
          onChange={(e) => setMapSearchTerm(e.target.value)}
          placeholder="Type to search..."
          className="px-4 py-3 rounded-lg border border-gray-300 bg-white text-gray-900 min-w-[220px] flex-1 focus:outline-none focus:ring-2 focus:ring-blue-500"
        />

        <button
          onClick={createNewMapImmediate}
          aria-label="Create new map"
          title="Create New Map"
          className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-3 rounded-lg shadow-md hover:shadow-lg transition-all duration-200 inline-flex items-center gap-2 font-bold"
        >
          <FaPlus />
          <span>Create New Map</span>
        </button>
      </div>

      {/* Divider */}
      <div className="border-t border-gray-200 my-2" />

      {/* Maps Table */}
      <div className="overflow-y-auto max-h-[420px]">
        <table className="w-full border-collapse mt-2">
          <thead className="bg-gray-50 sticky top-0">
            <tr>
              <th className="text-center p-3 w-18 text-sm font-semibold text-gray-700">
                Active
              </th>
              <th className="text-center p-3 w-24 text-sm font-semibold text-gray-700">
                Image
              </th>
              <th className="text-left p-3 text-sm font-semibold text-gray-700">
                Name
              </th>
              <th className="text-left p-3 text-sm font-semibold text-gray-700">
                Created By
              </th>
              <th className="text-left p-3 text-sm font-semibold text-gray-700">
                Created At
              </th>
              <th className="text-right p-3 text-sm font-semibold text-gray-700">
                Status
              </th>
              <th className="text-right p-3 w-40 text-sm font-semibold text-gray-700">
                Actions
              </th>
            </tr>
          </thead>
          <tbody>
            {filteredMaps.map((m) => (
              <tr
                key={normalizeId(m.id)} // Patch: use normalized id for stable keys
                onClick={() => selectMap(m)}
                className={`cursor-pointer transition-colors hover:bg-blue-50 ${
                  selectedMap?.id === normalizeId(m.id) ? "bg-blue-50" : "bg-white"
                }`}
              >
                <td className="p-3 border-b border-gray-200 text-center">
                  <input
                    type="radio"
                    name="activeMap"
                    checked={selectedMap?.id === normalizeId(m.id)} // Patch: compare normalized ids
                    onChange={(e) => {
                      e.stopPropagation();
                      selectMap(m);
                    }}
                    aria-label={`Activate ${m.name}`}
                    className="w-4 h-4 cursor-pointer"
                  />
                </td>

                {/* Image thumbnail cell */}
                <td className="p-3 border-b border-gray-200 text-center">
                  <div className="w-16 h-10 rounded border border-gray-200 bg-gray-50 overflow-hidden mx-auto">
                    {m.image ? (
                      <img
                        src={m.image as any}
                        alt={m.name}
                        className="w-full h-full object-cover"
                        onError={(e) => {
                          const fb = getFallbackImageByIdOrName(m.id, m.name);
                          if (fb && e.currentTarget.src !== fb) {
                            e.currentTarget.src = fb;
                            e.currentTarget.onerror = null;
                          } else {
                            e.currentTarget.style.display = "none";
                          }
                        }}
                      />
                    ) : (
                      <div className="w-full h-full flex items-center justify-center text-[10px] text-gray-400">
                        No Img
                      </div>
                    )}
                  </div>
                </td>

                <td className="p-3 border-b border-gray-200 font-bold text-gray-900">
                  {m.name}
                </td>
                <td className="p-3 border-b border-gray-200 text-gray-600">
                  {m.createdBy}
                </td>
                <td className="p-3 border-b border-gray-200 text-gray-600">
                  {m.createdAt || "—"}
                </td>
                <td className="p-3 border-b border-gray-200 text-right">
                  <span
                    className={`inline-flex items-center px-3 py-1 rounded-full text-xs font-bold ${
                      selectedMap?.id === normalizeId(m.id) || String(m.status || "").toLowerCase() === "active"
                        ? "bg-green-100 text-green-700"
                        : "bg-red-100 text-red-700"
                    }`}
                  >
                    {selectedMap?.id === normalizeId(m.id) || String(m.status || "").toLowerCase() === "active"
                      ? "Active"
                      : "Inactive"}
                  </span>
                </td>
                <td
                  className="p-3 border-b border-gray-200"
                  onClick={(e) => e.stopPropagation()}
                >
                  <div className="flex gap-2 justify-end">
                    <button
                      title="Edit"
                      onClick={(e) => {
                        e.stopPropagation();
                        openMapModal("edit", m);
                      }}
                      className="p-2 rounded-lg border border-gray-300 bg-transparent hover:bg-gray-50 transition-colors"
                    >
                      <FaEdit className="text-gray-600" />
                    </button>
                    <button
                      title="Delete"
                      onClick={(e) => {
                        e.stopPropagation();
                        handleMapAction("delete", m);
                      }}
                      className="p-2 rounded-lg border border-gray-300 bg-transparent hover:bg-red-50 transition-colors"
                    >
                      <FaTrash className="text-red-500" />
                    </button>
                  </div>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      {/* Edit Map Modal */}
      {mapModalOpen && mapModalMode === "edit" &&
        createPortal(
          (
            <div
              className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-[9999]"
              onClick={closeMapModal}
            >
              <div
                className="bg-white rounded-xl p-6 max-w-2xl w-11/12 shadow-2xl max-h-[90vh] overflow-y-auto"
                onClick={(e) => e.stopPropagation()}
              >
                {/* Modal Header */}
                <div className="flex justify-between items-center mb-5">
                  <h3 className="text-xl font-bold text-gray-900">Edit Map</h3>
                  <button
                    onClick={closeMapModal}
                    className="text-gray-400 hover:text-gray-600 text-2xl font-light transition-colors"
                  >
                    ×
                  </button>
                </div>

                {/* Modal Form */}
                <div className="flex flex-col gap-4">
                  {/* Map Name */}
                  <label className="flex flex-col gap-1.5">
                    <span className="text-sm font-semibold text-gray-700">
                      Map Name
                    </span>
                    <input
                      value={mapForm.name || ""}
                      onChange={(e) =>
                        setMapForm((prev) => ({ ...prev, name: e.target.value }))
                      }
                      className="px-3 py-2 rounded-lg border border-gray-300 focus:outline-none focus:ring-2 focus:ring-blue-500"
                    />
                  </label>

                  {/* Created By */}
                  <label className="flex flex-col gap-1.5">
                    <span className="text-sm font-semibold text-gray-700">
                      Created By
                    </span>
                    <input
                      value={mapForm.createdBy || ""}
                      onChange={(e) =>
                        setMapForm((prev) => ({
                          ...prev,
                          createdBy: e.target.value,
                        }))
                      }
                      className="px-3 py-2 rounded-lg border border-gray-300 focus:outline-none focus:ring-2 focus:ring-blue-500"
                    />
                  </label>

                  {/* Category */}
                  <label className="flex flex-col gap-1.5">
                    <span className="text-sm font-semibold text-gray-700">
                      Category
                    </span>
                    <input
                      value={mapForm.category || ""}
                      onChange={(e) =>
                        setMapForm((prev) => ({
                          ...prev,
                          category: e.target.value,
                        }))
                      }
                      placeholder="Optional"
                      className="px-3 py-2 rounded-lg border border-gray-300 focus:outline-none focus:ring-2 focus:ring-blue-500"
                    />
                  </label>

                  {/* Status */}
                  <label className="flex flex-col gap-1.5">
                    <span className="text-sm font-semibold text-gray-700">
                      Status
                    </span>
                    <select
                      value={mapForm.status || ""}
                      onChange={(e) =>
                        setMapForm((prev) => ({ ...prev, status: e.target.value }))
                      }
                      className="px-3 py-2 rounded-lg border border-gray-300 focus:outline-none focus:ring-2 focus:ring-blue-500"
                    >
                      <option value="">Inactive</option>
                      <option value="Active">Active</option>
                    </select>
                  </label>

                  {/* Map Image Upload Section */}
                  <div className="flex flex-col gap-3 p-4 bg-gray-50 rounded-lg border border-gray-200">
                    <span className="text-sm font-semibold text-gray-700">
                      Map Image
                    </span>

                    {/* Image preview */}
                    {mapForm.image && (
                      <div className="relative w-full max-h-52 overflow-hidden rounded-lg border border-gray-300">
                        <img
                          src={mapForm.image}
                          alt="Map preview"
                          className="w-full h-auto object-contain"
                          onError={(e) => {
                            const fb = getFallbackImageByIdOrName(mapForm.id as string, mapForm.name || "");
                            if (fb && e.currentTarget.src !== fb) {
                              e.currentTarget.src = fb;
                              e.currentTarget.onerror = null;
                            }
                          }}
                        />
                        <button
                          onClick={() =>
                            setMapForm((prev) => ({ ...prev, image: "" }))
                          }
                          className="absolute top-2 right-2 bg-red-500 hover:bg-red-600 text-white px-2 py-1 rounded-md text-xs font-semibold transition-colors"
                        >
                          Remove
                        </button>
                      </div>
                    )}

                    {/* URL Input */}
                    <label className="flex flex-col gap-1.5">
                      <span className="text-xs text-gray-600">Image URL</span>
                      <input
                        type="url"
                        value={
                          mapForm.image && !mapForm.image.startsWith("data:")
                            ? mapForm.image
                            : ""
                        }
                        onChange={(e) =>
                          setMapForm((prev) => ({ ...prev, image: e.target.value }))
                        }
                        placeholder="https://example.com/map.png"
                        className="px-3 py-2 rounded-lg border border-gray-300 text-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
                      />
                    </label>

                    {/* Divider */}
                    <div className="flex items-center gap-2">
                      <div className="flex-1 h-px bg-gray-300"></div>
                      <span className="text-xs text-gray-500">OR</span>
                      <div className="flex-1 h-px bg-gray-300"></div>
                    </div>

                    {/* File Browse Button */}
                    <button
                      type="button"
                      onClick={(e) => {
                        console.log("🔵 Browse button clicked");
                        e.preventDefault();
                        e.stopPropagation();
                        triggerMapImagePicker(e);
                      }}
                      className="px-4 py-2.5 bg-blue-600 hover:bg-blue-700 rounded-lg text-white text-sm font-semibold flex items-center justify-center gap-2 transition-colors"
                    >
                      <svg
                        width="16"
                        height="16"
                        viewBox="0 0 24 24"
                        fill="none"
                        stroke="currentColor"
                        strokeWidth="2"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                      >
                        <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
                        <polyline points="17 8 12 3 7 8" />
                        <line x1="12" y1="3" x2="12" y2="15" />
                      </svg>
                      📁 Browse from computer
                    </button>

                    <div className="text-xs text-gray-500 text-center">
                      Supported formats: PNG, JPG, JPEG, GIF, WebP (Max 5MB)
                    </div>
                  </div>

                  {/* Action Buttons */}
                  <div className="flex gap-2 justify-end mt-2">
                    <button
                      onClick={closeMapModal}
                      className="px-4 py-2 rounded-lg border border-gray-300 bg-transparent hover:bg-gray-50 transition-colors"
                    >
                      Cancel
                    </button>
                    <button
                      onClick={saveMapFromForm}
                      className="px-4 py-2 rounded-lg bg-blue-600 hover:bg-blue-700 text-white transition-colors"
                    >
                      Save Changes
                    </button>
                  </div>

                        <MapContext>
                            {(selectedMap, filteredWaypoints, filteredZones, filteredMissions) => (
                              <>
                                
                                  selectedMap={selectedMap}
                                  waypoints={filteredWaypoints}
                                  zones={filteredZones}
                                  missions={filteredMissions}
                                
                              </>
                            )}
                        </MapContext>

                </div>
              </div>
            </div>
          ),
          document.body
        )}
    </div>
  );
};

export default Maps;
