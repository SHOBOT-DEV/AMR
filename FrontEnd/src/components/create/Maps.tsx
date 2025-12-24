import React, { useEffect, useMemo, useRef, useState } from "react";
import { FaEdit, FaPlus, FaTrash } from "react-icons/fa";
import arena from "../../assets/shobot_arena.png";
import arena2 from "../../assets/shobot_arena2.png";
import simulationMap from "../../assets/simulation_map.png";

type MapRecord = {
  id: string | number | null;
  name: string;
  createdBy?: string;
  image?: string;
  status?: string;
  category?: string;
  createdAt?: string;
};

type MapForm = {
  id: string | number | null;
  name: string;
  createdBy: string;
  image: string;
  status: string;
  category: string;
  createdAt: string;
};

type MapModalMode = "preview" | "edit" | "create";

type ToastApi = {
  success?: (message: string) => void;
  error?: (message: string) => void;
};

export const SAMPLE_MAPS: MapRecord[] = [
  {
    id: "cfl_gf",
    name: "CFL_GF",
    createdBy: "CNDE IITM",
    image: simulationMap,
    status: "Active",
    category: "Factory",
    createdAt: "2025-11-17",
  },
  {
    id: "shobot_arena",
    name: "shobot_arena",
    createdBy: "ANSCER ADMIN",
    image: arena,
    status: "Maintenance",
    category: "Arena",
    createdAt: "2025-11-16",
  },
  {
    id: "shobot_arena2",
    name: "shobot_arena2",
    createdBy: "ANSCER ADMIN",
    image: arena2,
    status: "Inactive",
    category: "Arena",
    createdAt: "2025-11-15",
  },
  {
    id: "zones",
    name: "Zones",
    createdBy: "ANSCER ADMIN",
    image: "",
    status: "Inactive",
    category: "Reference",
    createdAt: "2025-11-15",
  },
  {
    id: "waypoints",
    name: "Waypoints",
    createdBy: "ANSCER ADMIN",
    image: "",
    status: "Inactive",
    category: "Reference",
    createdAt: "2025-11-15",
  },
  {
    id: "users",
    name: "Users",
    createdBy: "ANSCER ADMIN",
    image: "",
    status: "Inactive",
    category: "Reference",
    createdAt: "2025-11-15",
  },
];

export const useSampleMapState = () => {
  const [mapsList, setMapsList] = useState<MapRecord[]>(SAMPLE_MAPS);
  const [selectedMap, setSelectedMap] = useState<MapRecord | null>(SAMPLE_MAPS[0] ?? null);
  const [mapSearchField, setMapSearchField] = useState<string>("any");
  const [mapSearchTerm, setMapSearchTerm] = useState<string>("");

  return {
    mapsList,
    setMapsList,
    selectedMap,
    setSelectedMap,
    mapSearchField,
    setMapSearchField,
    mapSearchTerm,
    setMapSearchTerm,
  };
};

type Props = {
  mapsList: MapRecord[];
  selectedMap: MapRecord | null;
  createNewMapImmediate: () => void;
  handleActivateMap: (map: MapRecord) => void;
  requestV1?: (path: string, options?: RequestInit) => Promise<any>;
  toast?: ToastApi;
  setMapsList: React.Dispatch<React.SetStateAction<MapRecord[]>>;
  setSelectedMap: React.Dispatch<React.SetStateAction<MapRecord | null>>;
};

const makeBlankMapForm = (): MapForm => ({
  id: null,
  name: "",
  createdBy: "",
  image: "",
  status: "",
  category: "",
  createdAt: new Date().toISOString().slice(0, 10),
});

const statusClasses = (status?: string) => {
  if ((status || "").toLowerCase() === "active") {
    return "bg-green-500/90";
  }
  if ((status || "").toLowerCase() === "maintenance") {
    return "bg-yellow-500/90";
  }
  return "bg-slate-400";
};

const MapsPanel: React.FC<Props> = ({
  mapsList = [],
  selectedMap,
  createNewMapImmediate,
  handleActivateMap,
  requestV1,
  toast,
  setMapsList,
  setSelectedMap,
}) => {
  const [mapModalOpen, setMapModalOpen] = useState(false);
  const [mapModalMode, setMapModalMode] = useState<MapModalMode>("edit");
  const [mapForm, setMapForm] = useState<MapForm>(() => makeBlankMapForm());
  const [mapSearchField, setMapSearchField] = useState<string>("any");
  const [mapSearchTerm, setMapSearchTerm] = useState<string>("");
  const fileInputRef = useRef<HTMLInputElement | null>(null);

  useEffect(() => {
    if (!mapsList.length) return;
    const selectedExists = selectedMap && mapsList.some((map) => map.id === selectedMap.id);
    if (!selectedExists) {
      setSelectedMap(mapsList[0]);
    }
  }, [mapsList, selectedMap, setSelectedMap]);

  const filteredMaps = useMemo(() => {
    const term = (mapSearchTerm ?? "").trim().toLowerCase();
    if (!term) return mapsList;
    if ((mapSearchField ?? "any") === "any") {
      return mapsList.filter((map) =>
        [map.name, map.createdBy, map.category, map.createdAt, map.status]
          .map((value) => (value || "").toLowerCase())
          .some((value) => value.includes(term)),
      );
    }
    return mapsList.filter((map) =>
      String((map as any)[mapSearchField ?? "any"] || "").toLowerCase().includes(term),
    );
  }, [mapSearchField, mapSearchTerm, mapsList]);

  const openMapModal = (mode: MapModalMode, map?: MapRecord | null) => {
    setMapModalMode(mode);
    if (map) {
      setMapForm({
        id: map.id ?? null,
        name: map.name ?? "",
        createdBy: map.createdBy ?? "",
        image: map.image ?? "",
        status: map.status ?? "",
        category: map.category ?? "",
        createdAt: map.createdAt ?? new Date().toISOString().slice(0, 10),
      });
    } else {
      setMapForm(makeBlankMapForm());
    }
    setMapModalOpen(true);
  };

  const closeMapModal = () => {
    setMapModalOpen(false);
    setMapForm(makeBlankMapForm());
  };

  const saveMapFromForm = async () => {
    if (!mapForm.name.trim()) {
      toast?.error?.("Map name required");
      return;
    }

    if (mapModalMode === "edit" && mapForm.id !== null) {
      const payload = {
        name: mapForm.name.trim(),
        createdBy: mapForm.createdBy,
        image: mapForm.image,
        status: mapForm.status,
        category: mapForm.category,
        createdAt: mapForm.createdAt,
      };

      const applyLocalUpdate = () => {
        setMapsList((prev) => prev.map((map) => (map.id === mapForm.id ? { ...map, ...payload } : map)));
        setSelectedMap((prev) => {
          if (!prev || prev.id !== mapForm.id) return prev;
          return { ...prev, ...payload };
        });
      };

      try {
        if (requestV1) {
          await requestV1(`/maps/${mapForm.id}`, {
            method: "PATCH",
            body: JSON.stringify(payload),
          });
        }
        applyLocalUpdate();
        toast?.success?.("Map updated");
        closeMapModal();
      } catch (error: any) {
        console.warn("Map update failed, applying local change", error);
        applyLocalUpdate();
        toast?.error?.(error?.message || "Map saved locally (server error)");
        closeMapModal();
      }
    }
  };

  const handleMapImageChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const validTypes = ["image/png", "image/jpeg", "image/jpg", "image/gif", "image/webp"];
    if (!validTypes.includes(file.type)) {
      toast?.error?.("Please select an image (PNG, JPG, JPEG, GIF, WebP)");
      return;
    }

    if (file.size > 5 * 1024 * 1024) {
      toast?.error?.("File size must be below 5MB");
      return;
    }

    const reader = new FileReader();
    reader.onload = () => {
      setMapForm((prev) => ({ ...prev, image: String(reader.result || "") }));
      toast?.success?.("Image loaded successfully");
    };
    reader.onerror = () => toast?.error?.("Failed to read file");
    reader.readAsDataURL(file);
  };

  const triggerMapImagePicker = () => {
    if (!fileInputRef.current) {
      toast?.error?.("File picker unavailable. Please refresh the page.");
      return;
    }
    fileInputRef.current.value = "";
    fileInputRef.current.click();
  };

  const handleDeleteMap = async (map: MapRecord) => {
    if (!map || !map.id) return;
    const confirmed = window.confirm(`Delete map "${map.name || map.id}"? This cannot be undone.`);
    if (!confirmed) return;

    const removeLocally = () => {
      setMapsList((prev) => prev.filter((item) => item.id !== map.id));
      setSelectedMap((prev) => (prev && prev.id === map.id ? null : prev));
    };

    try {
      if (requestV1) {
        await requestV1(`/maps/${map.id}`, { method: "DELETE" });
      }
      removeLocally();
      toast?.success?.("Map deleted");
    } catch (error: any) {
      console.warn("Delete map API failed, removing locally", error);
      removeLocally();
      toast?.error?.(error?.message || "Map removed locally (server error)");
    }
  };

  return (
    <section className="flex flex-col gap-4">
      <input
        ref={fileInputRef}
        type="file"
        accept="image/*"
        className="hidden"
        onChange={handleMapImageChange}
      />

      <div className="flex flex-wrap items-center gap-3">
        <select
          aria-label="Search field"
          value={mapSearchField}
          onChange={(event) => setMapSearchField(event.target.value)}
          className="rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-600 outline-none transition focus:border-sky-500"
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
          onChange={(event) => setMapSearchTerm(event.target.value)}
          placeholder="Type to search..."
          className="min-w-[220px] flex-1 rounded-xl border border-slate-200 px-4 py-2 text-sm text-slate-800 outline-none transition focus:border-sky-500"
        />

        <button
          type="button"
          onClick={createNewMapImmediate}
          className="inline-flex items-center gap-2 rounded-xl bg-[#0b74d1] px-4 py-3 text-sm font-semibold text-white shadow-lg shadow-sky-500/20 transition hover:bg-[#095da8]"
        >
          <FaPlus className="text-xs" />
          <span>Create New Map</span>
        </button>
      </div>

      <div className="flex items-center justify-between rounded-xl border border-slate-100 bg-slate-50/60 px-4 py-2 text-xs font-semibold uppercase tracking-wide text-slate-500">
        <span>Active Maps</span>
        <span>
          {filteredMaps.length} of {mapsList.length} rows
        </span>
      </div>

      <div className="max-h-[420px] overflow-y-auto">
        <table className="w-full border-collapse text-sm">
          <thead className="bg-slate-50 text-left text-xs font-semibold uppercase tracking-wide text-slate-500">
            <tr>
              <th className="px-4 py-3 text-center">Active</th>
              <th className="px-4 py-3">Name</th>
              <th className="px-4 py-3">Created By</th>
              <th className="px-4 py-3">Created At</th>
              <th className="px-4 py-3 text-right">Status</th>
              <th className="px-4 py-3 text-right">Actions</th>
            </tr>
          </thead>
          <tbody>
            {filteredMaps.length === 0 ? (
              <tr>
                <td colSpan={6} className="px-4 py-10 text-center text-sm text-slate-500">
                  {mapSearchTerm
                    ? `No maps match "${mapSearchTerm}"`
                    : "No maps available yet. Create one to get started."}
                </td>
              </tr>
            ) : null}
            {filteredMaps.map((map) => (
              <tr
                key={String(map.id)}
                onClick={() => handleActivateMap(map)}
                className={`cursor-pointer border-b border-slate-100 transition hover:bg-sky-50 ${
                  selectedMap?.id === map.id ? "bg-sky-50" : "bg-white"
                }`}
              >
                <td className="px-4 py-3 text-center">
                  <input
                    type="radio"
                    name="activeMap"
                    checked={selectedMap?.id === map.id}
                    onChange={(event) => {
                      event.stopPropagation();
                      handleActivateMap(map);
                    }}
                    className="h-4 w-4 accent-sky-500"
                    aria-label={`Activate ${map.name}`}
                  />
                </td>
                <td className="px-4 py-3 font-semibold text-slate-900">{map.name}</td>
                <td className="px-4 py-3 text-slate-600">{map.createdBy || "—"}</td>
                <td className="px-4 py-3 text-slate-600">{map.createdAt || "—"}</td>
                <td className="px-4 py-3 text-right">
                  <span className={`rounded-full px-3 py-1 text-xs font-semibold text-white ${statusClasses(map.status)}`}>
                    {map.status || "Inactive"}
                  </span>
                </td>
                <td className="px-4 py-3">
                  <div className="flex justify-end gap-2" onClick={(event) => event.stopPropagation()}>
                    <button
                      type="button"
                      className="inline-flex items-center gap-1 rounded-lg border border-slate-200 px-3 py-1 text-xs font-medium text-slate-600 transition hover:border-slate-400 hover:text-slate-900"
                      onClick={() => openMapModal("edit", map)}
                      title="Edit"
                    >
                      <FaEdit />
                      Edit
                    </button>
                    <button
                      type="button"
                      className="inline-flex items-center gap-1 rounded-lg border border-red-200 px-3 py-1 text-xs font-medium text-red-600 transition hover:border-red-400 hover:bg-red-50"
                      onClick={() => handleDeleteMap(map)}
                      title="Delete"
                    >
                      <FaTrash />
                      Delete
                    </button>
                  </div>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      {mapModalOpen && mapModalMode === "edit" && (
        <div
          className="fixed inset-0 z-[999] flex items-center justify-center bg-black/50 p-4"
          onClick={closeMapModal}
        >
          <div
            className="w-full max-w-xl rounded-2xl bg-white p-6 shadow-2xl"
            onClick={(event) => event.stopPropagation()}
          >
            <div className="mb-4 flex items-center justify-between">
              <h3 className="text-lg font-semibold text-slate-900">Edit Map</h3>
              <button type="button" className="text-2xl" onClick={closeMapModal}>
                ×
              </button>
            </div>

            <div className="flex flex-col gap-4">
              <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
                Map Name
                <input
                  value={mapForm.name}
                  onChange={(event) => setMapForm((prev) => ({ ...prev, name: event.target.value }))}
                  className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
                />
              </label>

              <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
                Created By
                <input
                  value={mapForm.createdBy}
                  onChange={(event) => setMapForm((prev) => ({ ...prev, createdBy: event.target.value }))}
                  className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
                />
              </label>

              <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
                Category
                <input
                  value={mapForm.category}
                  onChange={(event) => setMapForm((prev) => ({ ...prev, category: event.target.value }))}
                  className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
                  placeholder="Optional"
                />
              </label>

              <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
                Status
                <select
                  value={mapForm.status}
                  onChange={(event) => setMapForm((prev) => ({ ...prev, status: event.target.value }))}
                  className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
                >
                  <option value="">Inactive</option>
                  <option value="Active">Active</option>
                  <option value="Maintenance">Maintenance</option>
                </select>
              </label>

              <div className="rounded-2xl border border-slate-200 bg-slate-50/70 p-4">
                <div className="text-sm font-semibold text-slate-600">Map Image</div>
                {mapForm.image ? (
                  <div className="relative mt-3 overflow-hidden rounded-xl border border-slate-200">
                    <img src={mapForm.image} alt="Map preview" className="h-auto w-full object-contain" />
                    <button
                      type="button"
                      className="absolute right-3 top-3 rounded-lg bg-red-500 px-3 py-1 text-xs font-semibold text-white"
                      onClick={() => setMapForm((prev) => ({ ...prev, image: "" }))}
                    >
                      Remove
                    </button>
                  </div>
                ) : null}

                <label className="mt-3 flex flex-col gap-2 text-xs font-medium text-slate-500">
                  Image URL
                  <input
                    type="url"
                    value={mapForm.image && !mapForm.image.startsWith("data:") ? mapForm.image : ""}
                    onChange={(event) => setMapForm((prev) => ({ ...prev, image: event.target.value }))}
                    placeholder="https://example.com/map.png"
                    className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
                  />
                </label>

                <div className="my-2 flex items-center gap-2 text-xs uppercase tracking-wide text-slate-400">
                  <span className="h-px flex-1 bg-slate-200" />
                  OR
                  <span className="h-px flex-1 bg-slate-200" />
                </div>

                <button
                  type="button"
                  onClick={triggerMapImagePicker}
                  className="inline-flex w-full items-center justify-center gap-2 rounded-xl border border-sky-400 bg-white px-4 py-3 text-sm font-semibold text-sky-600 transition hover:bg-sky-50"
                >
                  <FaPlus className="text-xs" />
                  Browse from computer
                </button>
                <p className="mt-2 text-center text-xs text-slate-500">
                  Supported formats: PNG, JPG, JPEG, GIF, WebP (max 5MB)
                </p>
              </div>
            </div>

            <div className="mt-6 flex justify-end gap-3">
              <button
                type="button"
                onClick={closeMapModal}
                className="rounded-xl border border-slate-200 px-4 py-2 text-sm font-semibold text-slate-600"
              >
                Cancel
              </button>
              <button
                type="button"
                onClick={saveMapFromForm}
                className="rounded-xl bg-[#0b74d1] px-5 py-2 text-sm font-semibold text-white shadow-lg shadow-sky-500/20"
              >
                Save Changes
              </button>
            </div>
          </div>
        </div>
      )}
    </section>
  );
};

export type { MapRecord };
export default MapsPanel;
