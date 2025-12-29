import React, { useState, useMemo } from "react";
import { FaPlus, FaEdit, FaTrash } from "react-icons/fa";
import { createPortal } from "react-dom";

// Type definitions
interface Zone {
  id: string;
  mapId: string;
  name: string;
  category: string;
  geometry: string;
  active: boolean;
  createdAt?: string;
}

interface ZonesProps {
  rightPage: string;
  selectedMap: any;
  zones: Zone[];
  setZones: React.Dispatch<React.SetStateAction<Zone[]>>;
  zoneSearchField: string;
  setZoneSearchField: (field: string) => void;
  zoneSearchTerm: string;
  setZoneSearchTerm: (term: string) => void;
  toast: {
    success: (msg: string) => void;
    error: (msg: string) => void;
  };
}

const Zones: React.FC<ZonesProps> = ({
  rightPage,
  selectedMap,
  zones,
  setZones,
  zoneSearchField,
  setZoneSearchField,
  zoneSearchTerm,
  setZoneSearchTerm,
  toast,
}) => {
  // Modal state
  const [zoneModalOpen, setZoneModalOpen] = useState(false);
  const [zoneModalMode, setZoneModalMode] = useState<"edit" | "create">("create");
  const [zoneForm, setZoneForm] = useState<Partial<Zone>>({
    id: undefined,
    name: "",
    category: "",
    geometry: "",
    active: true,
    createdAt: "",
  });

  // Default mock zones for UI display
  const mockZones: Zone[] = [
    {
      id: "zone-1",
      mapId: "map-1",
      name: "Safe Zone 1",
      category: "Safe",
      geometry: "Polygon((0,0),(100,0),(100,50),(0,50))",
      active: true,
      createdAt: "2025-12-28T10:30:00Z",
    },
    {
      id: "zone-2",
      mapId: "map-1",
      name: "Caution Area",
      category: "Caution",
      geometry: "Polygon((100,50),(150,50),(150,100),(100,100))",
      active: true,
      createdAt: "2025-12-28T11:45:00Z",
    },
    {
      id: "zone-3",
      mapId: "map-1",
      name: "Restricted Zone",
      category: "No-Go",
      geometry: "Polygon((150,100),(200,100),(200,150),(150,150))",
      active: true,
      createdAt: "2025-12-28T14:20:00Z",
    },
  ];

  // Filter zones by selected map
  const filteredZonesByMap = useMemo(() => {
    if (!selectedMap) {
      // Use mock zones for display when no map is selected
      return mockZones.filter((z) => z.mapId === "map-1");
    }
    return zones.filter((z) => z.mapId === selectedMap.id);
  }, [zones, selectedMap, mockZones]);

  // Apply search filter
  const filteredZones = useMemo(() => {
    const term = (zoneSearchTerm || "").trim().toLowerCase();
    const field = zoneSearchField;
    return filteredZonesByMap.filter((z) => {
      if (!term) return true;
      if (field === "any") {
        const haystack = `${z.name || ""} ${z.category || ""} ${z.geometry || ""} ${z.createdAt || ""}`.toLowerCase();
        return haystack.includes(term);
      }
      return String((z as any)[field] || "").toLowerCase().includes(term);
    });
  }, [filteredZonesByMap, zoneSearchField, zoneSearchTerm]);

  // Open zone modal
  const openZoneModal = (mode: "edit" | "create", zone?: Zone) => {
    setZoneModalMode(mode);
    if (zone) {
      setZoneForm(zone);
    } else {
      setZoneForm({
        id: undefined,
        name: "",
        category: "",
        geometry: "",
        active: true,
        createdAt: "",
      });
    }
    setZoneModalOpen(true);
  };

  // Close zone modal
  const closeZoneModal = () => {
    setZoneModalOpen(false);
    setZoneForm({
      id: undefined,
      name: "",
      category: "",
      geometry: "",
      active: true,
      createdAt: "",
    });
  };

  // Create new zone
  const createNewZone = () => {
    if (!selectedMap) {
      toast.error("Please select a map first");
      return;
    }
    openZoneModal("create");
  };

  // Save zone
  const saveZoneFromForm = () => {
    if (!zoneForm.name?.trim()) {
      toast.error("Zone name is required");
      return;
    }

    if (!selectedMap) {
      toast.error("No map selected");
      return;
    }

    if (zoneModalMode === "create") {
      const newZone: Zone = {
        id: `zone-${Date.now()}`,
        mapId: selectedMap.id,
        name: zoneForm.name.trim(),
        category: zoneForm.category || "",
        geometry: zoneForm.geometry || "",
        active: zoneForm.active ?? true,
        createdAt: new Date().toISOString(),
      };
      setZones((prev) => [...prev, newZone]);
      toast.success("Zone created successfully");
    } else if (zoneModalMode === "edit" && zoneForm.id) {
      setZones((prev) =>
        prev.map((z) =>
          z.id === zoneForm.id
            ? {
                ...z,
                name: zoneForm.name!.trim(),
                category: zoneForm.category || "",
                geometry: zoneForm.geometry || "",
                active: zoneForm.active ?? true,
              }
            : z
        )
      );
      toast.success("Zone updated successfully");
    }
    closeZoneModal();
  };

  // Delete zone
  const deleteZone = (zone: Zone) => {
    if (window.confirm(`Delete zone "${zone.name}"?`)) {
      setZones((prev) => prev.filter((z) => z.id !== zone.id));
      toast.success("Zone deleted successfully");
    }
  };

  return (
    <div className="flex flex-col gap-2">
      {/* Search and Create Controls */}
      <div className="flex gap-2 flex-wrap items-center">
        <select
          aria-label="Search field"
          value={zoneSearchField}
          onChange={(e) => setZoneSearchField(e.target.value)}
          className="px-3 py-3 rounded-lg border border-gray-300 bg-white text-gray-900 focus:outline-none focus:ring-2 focus:ring-blue-500"
        >
          <option value="any">All Fields</option>
          <option value="name">Name</option>
          <option value="category">Category</option>
          <option value="geometry">Geometry</option>
          <option value="createdAt">Created At</option>
        </select>

        <input
          value={zoneSearchTerm}
          onChange={(e) => setZoneSearchTerm(e.target.value)}
          placeholder="Search zones..."
          className="px-4 py-3 rounded-lg border border-gray-300 bg-white text-gray-900 min-w-[220px] flex-1 focus:outline-none focus:ring-2 focus:ring-blue-500"
        />

        <button
          onClick={createNewZone}
          aria-label="Create new zone"
          title="Create New Zone"
          className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-3 rounded-lg shadow-md hover:shadow-lg transition-all duration-200 inline-flex items-center gap-2 font-bold"
        >
          <FaPlus />
          <span>Create Zone</span>
        </button>
      </div>

      {/* Divider */}
      <div className="border-t border-gray-200 my-2" />

      {/* Zones Table */}
      <div className="overflow-y-auto max-h-[420px]">
        <table className="w-full border-collapse mt-2">
          <thead className="bg-gray-50 sticky top-0">
            <tr>
              <th className="text-left p-3 text-sm font-semibold text-gray-700">Name</th>
              <th className="text-left p-3 text-sm font-semibold text-gray-700">Category</th>
              <th className="text-center p-3 text-sm font-semibold text-gray-700">Active</th>
              <th className="text-left p-3 text-sm font-semibold text-gray-700">Geometry</th>
              <th className="text-left p-3 text-sm font-semibold text-gray-700">Created</th>
              <th className="text-right p-3 w-40 text-sm font-semibold text-gray-700">Actions</th>
            </tr>
          </thead>
          <tbody>
            {filteredZones.length === 0 ? (
              <tr>
                <td colSpan={6} className="p-8 text-center text-gray-500">
                  No zones found. Click 'Create Zone' to add one.
                </td>
              </tr>
            ) : (
              filteredZones.map((zone) => (
                <tr
                  key={zone.id}
                  className="cursor-pointer transition-colors hover:bg-blue-50 bg-white"
                >
                  <td className="p-3 border-b border-gray-200 font-bold text-gray-900">
                    {zone.name}
                  </td>
                  <td className="p-3 border-b border-gray-200 text-gray-600">
                    <span
                      className={`inline-flex items-center px-3 py-1 rounded-full text-xs font-bold ${
                        zone.category === "Safe"
                          ? "bg-green-100 text-green-700"
                          : zone.category === "Caution"
                          ? "bg-yellow-100 text-yellow-700"
                          : zone.category === "No-Go"
                          ? "bg-red-100 text-red-700"
                          : "bg-gray-100 text-gray-700"
                      }`}
                    >
                      {zone.category || "—"}
                    </span>
                  </td>
                  <td className="p-3 border-b border-gray-200 text-center">
                    <span
                      className={`inline-flex items-center px-3 py-1 rounded-full text-xs font-bold ${
                        zone.active
                          ? "bg-green-100 text-green-700"
                          : "bg-gray-100 text-gray-700"
                      }`}
                    >
                      {zone.active ? "Active" : "Inactive"}
                    </span>
                  </td>
                  <td className="p-3 border-b border-gray-200 text-gray-600 text-sm font-mono">
                    {zone.geometry
                      ? zone.geometry.length > 30
                        ? zone.geometry.substring(0, 30) + "..."
                        : zone.geometry
                      : "—"}
                  </td>
                  <td className="p-3 border-b border-gray-200 text-gray-600">
                    {zone.createdAt || "—"}
                  </td>
                  <td
                    className="p-3 border-b border-gray-200"
                    onClick={(e) => e.stopPropagation()}
                  >
                    <div className="flex gap-2 justify-end">
                      <button
                        title="Edit"
                        onClick={() => openZoneModal("edit", zone)}
                        className="p-2 rounded-lg border border-gray-300 bg-transparent hover:bg-gray-50 transition-colors"
                      >
                        <FaEdit className="text-gray-600" />
                      </button>
                      <button
                        title="Delete"
                        onClick={() => deleteZone(zone)}
                        className="p-2 rounded-lg border border-gray-300 bg-transparent hover:bg-red-50 transition-colors"
                      >
                        <FaTrash className="text-red-500" />
                      </button>
                    </div>
                  </td>
                </tr>
              ))
            )}
          </tbody>
        </table>
      </div>

      {/* Zone Modal */}
      {zoneModalOpen &&
        createPortal(
          <div
            className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-[9999]"
            onClick={closeZoneModal}
          >
            <div
              className="bg-white rounded-xl p-6 max-w-2xl w-11/12 shadow-2xl max-h-[90vh] overflow-y-auto"
              onClick={(e) => e.stopPropagation()}
            >
              {/* Modal Header */}
              <div className="flex justify-between items-center mb-5">
                <h3 className="text-xl font-bold text-gray-900">
                  {zoneModalMode === "create" ? "Create Zone" : "Edit Zone"}
                </h3>
                <button
                  onClick={closeZoneModal}
                  className="text-gray-400 hover:text-gray-600 text-2xl font-light transition-colors"
                >
                  ×
                </button>
              </div>

              {/* Modal Form */}
              <div className="flex flex-col gap-4">
                {/* Zone Name */}
                <label className="flex flex-col gap-1.5">
                  <span className="text-sm font-semibold text-gray-700">Zone Name *</span>
                  <input
                    value={zoneForm.name || ""}
                    onChange={(e) =>
                      setZoneForm((prev) => ({ ...prev, name: e.target.value }))
                    }
                    placeholder="e.g., Safe Zone 1"
                    className="px-3 py-2 rounded-lg border border-gray-300 focus:outline-none focus:ring-2 focus:ring-blue-500"
                  />
                </label>

                {/* Category */}
                <label className="flex flex-col gap-1.5">
                  <span className="text-sm font-semibold text-gray-700">Category</span>
                  <select
                    value={zoneForm.category || ""}
                    onChange={(e) =>
                      setZoneForm((prev) => ({ ...prev, category: e.target.value }))
                    }
                    className="px-3 py-2 rounded-lg border border-gray-300 focus:outline-none focus:ring-2 focus:ring-blue-500"
                  >
                    <option value="">Select category</option>
                    <option value="Safe">Safe</option>
                    <option value="Caution">Caution</option>
                    <option value="No-Go">No-Go</option>
                  </select>
                </label>

                {/* Geometry */}
                <label className="flex flex-col gap-1.5">
                  <span className="text-sm font-semibold text-gray-700">Geometry</span>
                  <textarea
                    value={zoneForm.geometry || ""}
                    onChange={(e) =>
                      setZoneForm((prev) => ({ ...prev, geometry: e.target.value }))
                    }
                    placeholder="e.g., Polygon((0,0),(100,0),(100,50),(0,50))"
                    rows={3}
                    className="px-3 py-2 rounded-lg border border-gray-300 focus:outline-none focus:ring-2 focus:ring-blue-500 font-mono text-sm"
                  />
                </label>

                {/* Active Status */}
                <label className="flex items-center gap-2">
                  <input
                    type="checkbox"
                    checked={zoneForm.active ?? true}
                    onChange={(e) =>
                      setZoneForm((prev) => ({ ...prev, active: e.target.checked }))
                    }
                    className="w-4 h-4 cursor-pointer"
                  />
                  <span className="text-sm font-semibold text-gray-700">Active</span>
                </label>

                {/* Action Buttons */}
                <div className="flex gap-2 justify-end mt-2">
                  <button
                    onClick={closeZoneModal}
                    className="px-4 py-2 rounded-lg border border-gray-300 bg-transparent hover:bg-gray-50 transition-colors"
                  >
                    Cancel
                  </button>
                  <button
                    onClick={saveZoneFromForm}
                    className="px-4 py-2 rounded-lg bg-blue-600 hover:bg-blue-700 text-white transition-colors"
                  >
                    {zoneModalMode === "create" ? "Create Zone" : "Save Changes"}
                  </button>
                </div>
              </div>
            </div>
          </div>,
          document.body
        )}
    </div>
  );
};

export default Zones;
