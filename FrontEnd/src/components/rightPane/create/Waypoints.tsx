import React, { useMemo, useState } from "react";
import { FaPlus } from "react-icons/fa";
import { getFilteredWaypoints, type WaypointRecord, type MapRecord } from "../../../utils/mapFilters.ts";

type ToastApi = {
  success?: (message: string) => void;
  error?: (message: string) => void;
};

type Props = {
  waypoints: WaypointRecord[];
  setWaypoints: React.Dispatch<React.SetStateAction<WaypointRecord[]>>;
  selectedMap?: MapRecord | null;
  mapsList?: MapRecord[];
  handleSelectWaypoint: (id: string | number) => void;
  setSelectedWaypointId: (id: string | number | null) => void;
  toast?: ToastApi;
};

const defaultForm = {
  name: "",
  category: "Nav",
  geom: "",
  notes: "",
  active: true,
};

const filterFields: Record<string, keyof WaypointRecord | "active"> = {
  name: "name",
  category: "category",
  active: "active",
  geom: "geom",
  createdAt: "createdAt",
};

const WaypointsPanel: React.FC<Props> = ({
  waypoints = [],
  setWaypoints,
  selectedMap,
  mapsList = [],
  handleSelectWaypoint,
  setSelectedWaypointId,
  toast,
}) => {
  const [formOpen, setFormOpen] = useState(false);
  const [form, setForm] = useState(defaultForm);
  const [searchField, setSearchField] = useState<keyof typeof filterFields | "any">("any");
  const [searchTerm, setSearchTerm] = useState("");

  const filteredWaypoints = useMemo(() => {
    const baseWaypoints = getFilteredWaypoints(waypoints, selectedMap);
    const term = searchTerm.trim().toLowerCase();
    if (!term) return baseWaypoints;
    if (searchField === "any") {
      return baseWaypoints.filter((wp) =>
        Object.values(filterFields)
          .map((field) => {
            if (field === "active") return (wp.active ? "active" : "disabled").toLowerCase();
            return String(wp[field as keyof WaypointRecord] || "").toLowerCase();
          })
          .some((value) => value.includes(term)),
      );
    }
    if (searchField === "active") {
      return baseWaypoints.filter((wp) => (wp.active ? "active" : "disabled").toLowerCase().includes(term));
    }
    return baseWaypoints.filter((wp) => String(wp[searchField as keyof WaypointRecord] || "").toLowerCase().includes(term));
  }, [waypoints, selectedMap, searchField, searchTerm]);

  const mapName = selectedMap?.name || mapsList.find((map) => map.id === selectedMap?.id)?.name;

  const resetForm = () => {
    setForm(defaultForm);
    setFormOpen(false);
  };

  const saveWaypoint = () => {
    if (!selectedMap?.id) {
      toast?.error?.("Select a map to add waypoints");
      return;
    }
    if (!form.name.trim()) {
      toast?.error?.("Enter a waypoint name");
      return;
    }
    if (!form.geom.trim()) {
      toast?.error?.("Add waypoint coordinates");
      return;
    }
    const payload: WaypointRecord = {
      id: `wp-${Date.now()}`,
      mapId: selectedMap.id,
      name: form.name.trim(),
      category: form.category,
      geom: form.geom.trim(),
      notes: form.notes.trim(),
      active: form.active,
      createdAt: new Date().toLocaleString(),
    };
    setWaypoints((previous) => [payload, ...previous]);
    setSelectedWaypointId(payload.id);
    handleSelectWaypoint(payload.id);
    resetForm();
    toast?.success?.("Waypoint created");
  };

  if (!selectedMap) {
    return (
      <section className="flex flex-col gap-4 px-4 py-6">
        <div className="rounded-2xl border border-slate-200 bg-white px-6 py-8 text-center text-slate-500">
          Select a map from the Maps panel to view waypoints.
        </div>
      </section>
    );
  }

  return (
    <section className="flex flex-col gap-4 px-4 py-4">
      <div className="flex flex-wrap items-center gap-3">
        <select
          value={searchField}
          onChange={(event) => setSearchField(event.target.value as keyof typeof filterFields | "any")}
          className="min-w-[160px] rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-600 outline-none transition focus:border-sky-500"
        >
          <option value="any">All Fields</option>
          <option value="name">Name</option>
          <option value="category">Category</option>
          <option value="active">Active</option>
          <option value="geom">Geometry</option>
          <option value="createdAt">Created At</option>
        </select>
        <input
          value={searchTerm}
          onChange={(event) => setSearchTerm(event.target.value)}
          placeholder="Search waypoint..."
          className="min-w-[220px] flex-1 rounded-xl border border-slate-200 px-4 py-2 text-sm text-slate-800 outline-none transition focus:border-sky-500"
        />
        <button
          type="button"
          onClick={() => setFormOpen((prev) => !prev)}
          disabled={!selectedMap?.id}
          className="inline-flex items-center gap-2 rounded-xl bg-[#0b74d1] px-4 py-3 text-sm font-semibold text-white shadow-lg shadow-sky-500/20 transition hover:bg-[#095da8] disabled:cursor-not-allowed disabled:opacity-60"
        >
          <FaPlus className="text-xs" />
          <span>Create Waypoint</span>
        </button>
      </div>

      {formOpen ? (
        <div className="rounded-2xl border border-slate-200 bg-slate-50/80 p-4">
          <div className="grid gap-4 md:grid-cols-3">
            <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
              Name
              <input
                value={form.name}
                onChange={(event) => setForm((prev) => ({ ...prev, name: event.target.value }))}
                placeholder="Waypoint name"
                className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
              />
            </label>
            <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
              Category
              <select
                value={form.category}
                onChange={(event) => setForm((prev) => ({ ...prev, category: event.target.value }))}
                className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
              >
                <option value="Nav">Nav</option>
                <option value="Inspect">Inspect</option>
                <option value="Charge">Charge</option>
              </select>
            </label>
            <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
              Geometry
              <input
                value={form.geom}
                onChange={(event) => setForm((prev) => ({ ...prev, geom: event.target.value }))}
                placeholder="Point(x y)"
                className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
              />
            </label>
          </div>
          <label className="mt-4 flex flex-col gap-2 text-sm font-semibold text-slate-600">
            Notes
            <input
              value={form.notes}
              onChange={(event) => setForm((prev) => ({ ...prev, notes: event.target.value }))}
              className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
            />
          </label>
          <label className="mt-4 flex items-center gap-2 text-sm font-semibold text-slate-600">
            <input
              type="checkbox"
              checked={form.active}
              onChange={() => setForm((prev) => ({ ...prev, active: !prev.active }))}
              className="h-4 w-4 accent-sky-500"
            />
            Active waypoint
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
              onClick={saveWaypoint}
              className="rounded-xl bg-[#0b74d1] px-5 py-2 text-sm font-semibold text-white shadow-lg shadow-sky-500/20"
            >
              Save Waypoint
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
            </tr>
          </thead>
          <tbody>
            {filteredWaypoints.length === 0 ? (
              <tr>
                <td colSpan={4} className="px-4 py-8 text-center text-sm text-slate-500">
                  {searchTerm
                    ? `No waypoints match "${searchTerm}"`
                    : `No waypoints found for ${mapName || selectedMap?.id}. Create one to get started.`}
                </td>
              </tr>
            ) : null}
            {filteredWaypoints.map((waypoint) => (
              <tr
                key={String(waypoint.id)}
                className="cursor-pointer border-b border-slate-100 transition hover:bg-sky-50"
                onClick={() => handleSelectWaypoint(waypoint.id)}
              >
                <td className="px-4 py-3 font-semibold text-slate-900">{waypoint.name}</td>
                <td className="px-4 py-3 text-slate-600">{waypoint.category}</td>
                <td className="px-4 py-3">
                  <span
                    className={`rounded-full px-3 py-1 text-xs font-semibold ${
                      waypoint.active ? "bg-emerald-100 text-emerald-700" : "bg-slate-200 text-slate-600"
                    }`}
                  >
                    {waypoint.active ? "Active" : "Disabled"}
                  </span>
                </td>
                <td className="px-4 py-3 text-slate-500">{waypoint.geom}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </section>
  );
};

export default WaypointsPanel;
