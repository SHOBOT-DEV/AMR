import React, { useMemo, useState } from "react";
import { FaPlus } from "react-icons/fa";

type ZoneRecord = {
  id: string | number;
  mapId: string | number;
  name: string;
  category: string;
  geometry: string;
  active: boolean;
  createdAt?: string;
};

type MapRecord = {
  id: string | number;
  name?: string;
};

type ToastApi = {
  success?: (message: string) => void;
  error?: (message: string) => void;
};

type Props = {
  zones: ZoneRecord[];
  setZones: React.Dispatch<React.SetStateAction<ZoneRecord[]>>;
  selectedMap?: MapRecord | null;
  requestV1?: (path: string, options?: RequestInit) => Promise<any>;
  toast?: ToastApi;
};

const defaultFormState = {
  name: "",
  category: "Safe",
  geometry: "",
  active: true,
};

const ZonesPanel: React.FC<Props> = ({ zones, setZones, selectedMap, requestV1, toast }) => {
  const [searchField, setSearchField] = useState("any");
  const [searchTerm, setSearchTerm] = useState("");
  const [formOpen, setFormOpen] = useState(false);
  const [formState, setFormState] = useState(defaultFormState);

  const zonesForMap = useMemo(() => {
    if (!selectedMap?.id) return [];
    return zones.filter((zone) => zone.mapId === selectedMap.id);
  }, [zones, selectedMap]);

  const filteredZones = useMemo(() => {
    const term = searchTerm.trim().toLowerCase();
    if (!term) return zonesForMap;
    if (searchField === "any") {
      return zonesForMap.filter((zone) =>
        [zone.name, zone.category, zone.geometry, zone.createdAt]
          .map((value) => (value || "").toString().toLowerCase())
          .some((value) => value.includes(term)),
      );
    }
    if (searchField === "active") {
      return zonesForMap.filter((zone) => (zone.active ? "active" : "disabled").includes(term));
    }
    return zonesForMap.filter((zone) =>
      String((zone as any)[searchField] || "").toLowerCase().includes(term),
    );
  }, [zonesForMap, searchField, searchTerm]);

  const resetForm = () => {
    setFormState(defaultFormState);
    setFormOpen(false);
  };

  const saveZone = async () => {
    if (!selectedMap?.id) {
      toast?.error?.("Select a map before creating zones");
      return;
    }
    if (!formState.name.trim()) {
      toast?.error?.("Enter a zone name");
      return;
    }
    if (!formState.geometry.trim()) {
      toast?.error?.("Provide zone geometry");
      return;
    }
    const payload: ZoneRecord = {
      id: `zone-${Date.now()}`,
      mapId: selectedMap.id,
      name: formState.name.trim(),
      category: formState.category,
      geometry: formState.geometry.trim(),
      active: formState.active,
      createdAt: new Date().toLocaleString(),
    };

    try {
      let createdZone = payload;
      if (requestV1) {
        const response = await requestV1("/zones", {
          method: "POST",
          body: JSON.stringify(payload),
        });
        if (response?.item) {
          createdZone = { ...payload, ...response.item };
        }
      }
      setZones((previous) => [createdZone, ...previous]);
      resetForm();
      toast?.success?.("Zone created");
    } catch (error: any) {
      console.error("Zone create error", error);
      setZones((previous) => [payload, ...previous]);
      toast?.error?.(error?.message || "Zone saved locally (server error)");
    }
  };

  if (!selectedMap) {
    return (
      <section className="flex flex-col gap-4 px-4 py-6">
        <div className="rounded-2xl border border-slate-200 bg-white px-6 py-8 text-center text-slate-500">
          Select a map from the Maps section to view and create zones.
        </div>
      </section>
    );
  }

  return (
    <section className="flex flex-col gap-4 px-4 py-4">
      <div className="flex flex-wrap items-center gap-3">
        <select
          value={searchField}
          onChange={(event) => setSearchField(event.target.value)}
          className="min-w-[160px] rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-600 outline-none transition focus:border-sky-500"
        >
          <option value="any">All Fields</option>
          <option value="name">Name</option>
          <option value="category">Category</option>
          <option value="active">Active</option>
          <option value="geometry">Geometry</option>
          <option value="createdAt">Created At</option>
        </select>
        <input
          value={searchTerm}
          onChange={(event) => setSearchTerm(event.target.value)}
          placeholder="Search zone..."
          className="min-w-[220px] flex-1 rounded-xl border border-slate-200 px-4 py-2 text-sm text-slate-800 outline-none transition focus:border-sky-500"
        />
        <button
          type="button"
          onClick={() => setFormOpen((previous) => !previous)}
          className="inline-flex items-center gap-2 rounded-xl bg-[#0b74d1] px-4 py-3 text-sm font-semibold text-white shadow-lg shadow-sky-500/20 transition hover:bg-[#095da8]"
        >
          <FaPlus className="text-xs" />
          <span>Create Zone</span>
        </button>
      </div>

      {formOpen ? (
        <div className="rounded-2xl border border-slate-200 bg-slate-50/80 p-4">
          <div className="grid gap-4 md:grid-cols-3">
            <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
              Name
              <input
                value={formState.name}
                onChange={(event) => setFormState((prev) => ({ ...prev, name: event.target.value }))}
                placeholder="Zone name"
                className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
              />
            </label>
            <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
              Category
              <select
                value={formState.category}
                onChange={(event) => setFormState((prev) => ({ ...prev, category: event.target.value }))}
                className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
              >
                <option value="Safe">Safe</option>
                <option value="Caution">Caution</option>
                <option value="No-Go">No-Go</option>
              </select>
            </label>
            <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
              Geometry
              <input
                value={formState.geometry}
                onChange={(event) => setFormState((prev) => ({ ...prev, geometry: event.target.value }))}
                placeholder="Polygon((...))"
                className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
              />
            </label>
          </div>

          <label className="mt-4 flex items-start gap-3 rounded-xl border border-slate-200 bg-white px-4 py-3 text-sm text-slate-600">
            <input
              type="checkbox"
              checked={formState.active}
              onChange={() => setFormState((prev) => ({ ...prev, active: !prev.active }))}
              className="mt-1 h-4 w-4 accent-sky-500"
            />
            <div>
              <div className="text-sm font-semibold text-slate-700">Zone Enabled</div>
              <p className="text-xs text-slate-500">
                {formState.active ? "Robots can enter" : "Robots must avoid"}
              </p>
            </div>
          </label>

          <div className="mt-4 flex justify-end gap-3">
            <button
              type="button"
              onClick={resetForm}
              className="rounded-xl border border-slate-200 px-4 py-2 text-sm font-semibold text-slate-600 transition hover:bg-white"
            >
              Cancel
            </button>
            <button
              type="button"
              onClick={saveZone}
              className="rounded-xl bg-[#0b74d1] px-5 py-2 text-sm font-semibold text-white shadow-lg shadow-sky-500/20"
            >
              Save Zone
            </button>
          </div>
        </div>
      ) : null}

      <div className="overflow-hidden rounded-2xl border border-slate-200 bg-white">
        <table className="w-full border-collapse text-sm">
          <thead className="bg-slate-50 text-left text-xs font-semibold uppercase tracking-wide text-slate-500">
            <tr>
              <th className="px-4 py-3">Name</th>
              <th className="px-4 py-3">Category</th>
              <th className="px-4 py-3">Active</th>
              <th className="px-4 py-3">Geometry</th>
              <th className="px-4 py-3">Created</th>
            </tr>
          </thead>
          <tbody>
            {filteredZones.length === 0 ? (
              <tr>
                <td colSpan={5} className="px-4 py-8 text-center text-sm text-slate-500">
                  {searchTerm
                    ? `No zones match "${searchTerm}"`
                    : `No zones defined for ${selectedMap?.name}.`}
                </td>
              </tr>
            ) : null}
            {filteredZones.map((zone) => (
              <tr key={String(zone.id)} className="border-b border-slate-100">
                <td className="px-4 py-3 font-semibold text-slate-900">{zone.name}</td>
                <td className="px-4 py-3 text-slate-600">{zone.category}</td>
                <td className="px-4 py-3">
                  <span
                    className={`rounded-full px-3 py-1 text-xs font-semibold ${
                      zone.active ? "bg-emerald-100 text-emerald-700" : "bg-slate-200 text-slate-600"
                    }`}
                  >
                    {zone.active ? "Active" : "Disabled"}
                  </span>
                </td>
                <td className="px-4 py-3 text-slate-500">{zone.geometry}</td>
                <td className="px-4 py-3 text-slate-500">{zone.createdAt || "—"}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </section>
  );
};

export default ZonesPanel;
